# Phase 1: CUDA Docker + Basic Pipeline Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Stand up the multi-service Docker Compose architecture with NATS messaging, Caddy reverse proxy, and a working SHM ringbuffer proving the zero-copy video pipeline between Python writer and Python reader.

**Architecture:** Evolve the existing single-profile docker-compose.yml into a multi-service, multi-network setup. Add NATS for the backplane message bus, Caddy as the edge reverse proxy, and build the SHM ringbuffer as a Python library with a test harness that validates the atomic commit protocol.

**Tech Stack:** Docker Compose v2 (profiles), NATS + JetStream, Caddy 2, ROS 2 Jazzy, Python 3.10+ (asyncio, mmap, struct), pytest

**Ref:** [Multi-Service Backend Design](/docs/plans/2026-02-05-multi-service-backend-design.md)

**Working Reference:** `/home/ethan/dev/Genesis/SDR_OS` — contains a proven ROS 2 Jazzy Dockerfile, run_nodes.sh, genesis_bridge package, and robot configs. Copy and adapt from there rather than writing from scratch where applicable.

---

## Task 1: Update .gitignore for New Service Artifacts

**Files:**
- Modify: `.gitignore`

**Step 1: Add ignores for service artifacts**

Append to `.gitignore`:

```gitignore
# Service data
nats_data/
prometheus_data/
grafana_data/

# Secrets / TLS
*.key
*.pem
!configs/*.example

# Rust build artifacts
services/*/target/
```

**Step 2: Verify gitignore works**

Run: `git check-ignore nats_data/ services/transport-server/target/`
Expected: Both paths printed (ignored)

**Step 3: Commit**

```bash
git add .gitignore
git commit -m "chore: update gitignore for multi-service artifacts"
```

---

## Task 2: Create NATS Configuration

**Files:**
- Create: `configs/nats.conf`

**Step 1: Write NATS config**

```conf
# SDR_OS NATS Server Configuration
# Backplane-only: not exposed to edge network

listen: 0.0.0.0:4222
server_name: sdr_nats

# Monitoring
http_port: 8222

# JetStream
jetstream {
    store_dir: /data/jetstream
    max_mem: 256MB
    max_file: 50GB
}

# Logging
logtime: true
debug: false
trace: false

# Limits
max_payload: 1MB
max_connections: 64
```

**Step 2: Verify config parses**

We'll validate this in Task 4 when NATS starts.

**Step 3: Commit**

```bash
git add configs/nats.conf
git commit -m "feat: add NATS JetStream configuration"
```

---

## Task 3: Create Caddyfile

**Files:**
- Create: `configs/Caddyfile`

**Step 1: Write Caddyfile**

```caddyfile
# SDR_OS Reverse Proxy
# Listens on :80, routes to backend services
# Tailscale TLS is added when DOMAIN env var is set

{
    admin off
}

:{$PORT:80} {
    # Static web app
    handle /assets/* {
        reverse_proxy webserver:3000
    }

    handle /configs/* {
        reverse_proxy webserver:3000
    }

    # Video stream transport
    handle /stream/* {
        reverse_proxy transport-server:8080
    }

    # Socket.io for controller relay
    handle /socket.io/* {
        reverse_proxy webserver:3000 {
            transport http {
                versions h1 1.1
            }
        }
    }

    # ROS bridge WebSocket
    handle /rosbridge/* {
        reverse_proxy ros-bridge:9091
    }

    # Default: serve the web app
    handle {
        reverse_proxy webserver:3000
    }
}
```

**Step 2: Commit**

```bash
git add configs/Caddyfile
git commit -m "feat: add Caddy reverse proxy configuration"
```

---

## Task 4: ROS 2 Jazzy Container + Run Script

Adapted from the working setup at `/home/ethan/dev/Genesis/SDR_OS/Dockerfile.ros-jazzy`.

**Files:**
- Create: `containers/ros-jazzy/Dockerfile`
- Create: `scripts/ros/run_nodes.sh`
- Copy+adapt: Robot configs from Genesis repo

### Step 1: Write the ROS 2 Jazzy Dockerfile

Create `containers/ros-jazzy/Dockerfile`:

```dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-server \
    ros-jazzy-rclpy \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-msgs \
    ros-jazzy-nav-msgs \
    python3-opengl \
    python3-pip \
    curl \
  && rm -rf /var/lib/apt/lists/*

# Install Python deps, filtering out ROS-provided packages to avoid pip/apt conflicts
COPY requirements/requirements-base.txt /tmp/requirements-base.txt
RUN grep -Ev '^(rclpy|geometry_msgs|sensor_msgs|python3-opengl|#)' \
    /tmp/requirements-base.txt > /tmp/requirements.noros.txt 2>/dev/null \
  && pip3 install --no-cache-dir -r /tmp/requirements.noros.txt 2>/dev/null || true

WORKDIR /workspace

CMD ["/workspace/scripts/ros/run_nodes.sh"]
```

**Why this differs from Genesis version:** Adds tf2-msgs and nav-msgs (needed for /tf and /odom topics from the design doc). Uses our `requirements/` directory structure instead of a flat requirements.txt.

### Step 2: Write the ROS node runner script

Create `scripts/ros/run_nodes.sh` (adapted from Genesis):

```bash
#!/usr/bin/env bash
set -eo pipefail

# Source ROS setup
if ! source /opt/ros/jazzy/setup.bash 2>/dev/null; then
  echo "Warning: Failed to source ROS setup. Continuing anyway..." >&2
fi

ROS_ENABLE_ROSBRIDGE="${ROS_ENABLE_ROSBRIDGE:-1}"
ROS_NODE_COMMANDS="${ROS_NODE_COMMANDS:-}"

pids=()

if [[ "${ROS_ENABLE_ROSBRIDGE}" == "1" ]]; then
  echo "Starting rosbridge_server..." >&2
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
  pids+=("$!")
fi

if [[ -n "${ROS_NODE_COMMANDS}" ]]; then
  IFS=';' read -r -a commands <<< "${ROS_NODE_COMMANDS}"
  for cmd in "${commands[@]}"; do
    cmd_trimmed="${cmd## }"
    cmd_trimmed="${cmd_trimmed%% }"
    if [[ -n "${cmd_trimmed}" ]]; then
      bash -lc "${cmd_trimmed}" &
      pids+=("$!")
    fi
  done
fi

if [[ ${#pids[@]} -eq 0 ]]; then
  echo "No ROS processes configured. Set ROS_NODE_COMMANDS or enable rosbridge." >&2
  echo "Container will stay alive. Use 'docker exec' to get a shell." >&2
  tail -f /dev/null
else
  echo "Started ${#pids[@]} background process(es). Waiting..." >&2
  wait -n || true
  echo "One of the background processes exited. Container will stay alive." >&2
  tail -f /dev/null
fi
```

### Step 3: Copy robot configs from Genesis repo

```bash
# Copy the proven configs directory structure
cp -r /home/ethan/dev/Genesis/SDR_OS/configs/robots/ configs/robots/
cp /home/ethan/dev/Genesis/SDR_OS/configs/genesis_bridge.yaml configs/genesis_bridge.yaml
cp /home/ethan/dev/Genesis/SDR_OS/configs/genesis_ros_publisher.yaml configs/genesis_ros_publisher.yaml
```

### Step 4: Make run script executable and verify Dockerfile builds

Run: `chmod +x scripts/ros/run_nodes.sh`
Run: `docker build -f containers/ros-jazzy/Dockerfile -t sdr_os-ros-jazzy .`
Expected: Build succeeds, image tagged sdr_os-ros-jazzy

### Step 5: Commit

```bash
git add containers/ros-jazzy/Dockerfile scripts/ros/run_nodes.sh configs/robots/ configs/genesis_bridge.yaml configs/genesis_ros_publisher.yaml
git commit -m "feat: add ROS 2 Jazzy container and robot configs (from Genesis)"
```

---

## Task 5: Rewrite docker-compose.yml for Multi-Service Architecture

**Files:**
- Modify: `docker-compose.yml`

This is the biggest structural change. The existing file has 3 monolithic app containers. We're replacing it with the service-per-concern architecture from the design doc. The ros-bridge now uses our ROS 2 Jazzy Dockerfile from Task 4.

**Step 1: Write the new docker-compose.yml**

```yaml
# SDR_OS Multi-Service Architecture
# Usage:
#   docker compose up                     # Core services (web, nats, caddy)
#   docker compose --profile sim up       # + simulation
#   docker compose --profile train up     # + training
#   docker compose --profile obs up       # + observability
#   docker compose --profile dev up       # + dev tools

networks:
  edge:
    driver: bridge
  backplane:
    driver: bridge
    internal: true

volumes:
  sdr_ipc:
    driver_opts:
      type: tmpfs
      device: tmpfs
      o: size=512m
  nats_data:

services:
  # ─── Edge ───────────────────────────────────────────────
  caddy:
    image: caddy:2-alpine
    restart: unless-stopped
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./configs/Caddyfile:/etc/caddy/Caddyfile:ro
    networks:
      - edge
      - backplane
    depends_on:
      webserver:
        condition: service_healthy
    healthcheck:
      test: ["CMD", "wget", "--spider", "-q", "http://localhost:80/"]
      interval: 10s
      timeout: 3s
      retries: 3

  # ─── Web App Server (Express + Socket.io) ───────────────
  webserver:
    build:
      context: .
      dockerfile: containers/cuda/Dockerfile
    image: sdr_os-webserver
    command: ["node", "server.js"]
    working_dir: /workspace
    volumes:
      - .:/workspace
      - /workspace/node_modules
    networks:
      - backplane
    expose:
      - "3000"
    healthcheck:
      test: ["CMD", "node", "-e", "require('http').get('http://localhost:3000/', r => { process.exit(r.statusCode === 200 ? 0 : 1) }).on('error', () => process.exit(1))"]
      interval: 10s
      timeout: 5s
      retries: 3

  # ─── NATS Message Bus ───────────────────────────────────
  nats:
    image: nats:2-alpine
    restart: unless-stopped
    command: ["-c", "/etc/nats/nats.conf"]
    volumes:
      - ./configs/nats.conf:/etc/nats/nats.conf:ro
      - nats_data:/data/jetstream
    networks:
      - backplane
    expose:
      - "4222"
      - "8222"
    healthcheck:
      test: ["CMD", "nats-server", "--signal", "ldm=http://localhost:8222"]
      interval: 10s
      timeout: 3s
      retries: 3
      start_period: 5s

  # ─── Genesis Simulation (GPU) ───────────────────────────
  genesis-sim:
    profiles: ["sim"]
    build:
      context: .
      dockerfile: containers/cuda/Dockerfile
    image: sdr_os-genesis
    command: ["python", "-m", "genesis_bridge"]
    working_dir: /workspace
    volumes:
      - .:/workspace
      - sdr_ipc:/dev/shm/sdr_os_ipc
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video
      - SDR_SHM_PATH=/dev/shm/sdr_os_ipc/frames
      - SDR_NATS_URL=nats://nats:4222
    device_requests:
      - driver: nvidia
        count: -1
        capabilities: ["gpu"]
    networks:
      - backplane
    depends_on:
      nats:
        condition: service_healthy
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    shm_size: "2gb"
    healthcheck:
      test: ["CMD", "python", "-c", "import genesis_bridge; print('ok')"]
      interval: 15s
      timeout: 5s
      retries: 3
      start_period: 30s

  # ─── ROS Bridge (ROS 2 Jazzy) ───────────────────────────
  ros-bridge:
    profiles: ["sim"]
    build:
      context: .
      dockerfile: containers/ros-jazzy/Dockerfile
    image: sdr_os-ros-bridge
    command: ["/workspace/scripts/ros/run_nodes.sh"]
    working_dir: /workspace
    volumes:
      - .:/workspace
    environment:
      - ROS_ENABLE_ROSBRIDGE=1
      - ROS_DOMAIN_ID=0
      - SDR_NATS_URL=nats://nats:4222
    networks:
      - backplane
    depends_on:
      nats:
        condition: service_healthy
    expose:
      - "9090"
      - "9091"
    healthcheck:
      test: ["CMD", "bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 topic list >/dev/null 2>&1"]
      interval: 15s
      timeout: 5s
      retries: 3
      start_period: 15s

  # ─── Training Runner ────────────────────────────────────
  training-runner:
    profiles: ["train"]
    build:
      context: .
      dockerfile: containers/cuda/Dockerfile
    image: sdr_os-training
    command: ["python", "-m", "training_runner"]
    working_dir: /workspace
    volumes:
      - .:/workspace
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility
      - SDR_NATS_URL=nats://nats:4222
    device_requests:
      - driver: nvidia
        count: -1
        capabilities: ["gpu"]
    networks:
      - backplane
    depends_on:
      nats:
        condition: service_healthy
    shm_size: "2gb"

# Legacy profiles preserved for backward compatibility
  app-cuda:
    profiles: ["legacy-cuda"]
    build:
      context: .
      dockerfile: containers/cuda/Dockerfile
    image: sdr_os-cuda
    volumes:
      - .:/workspace
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
    device_requests:
      - driver: nvidia
        count: -1
        capabilities: ["gpu"]
    shm_size: "2gb"

  app-rocm:
    profiles: ["legacy-rocm"]
    build:
      context: .
      dockerfile: containers/rocm/Dockerfile
    image: sdr_os-rocm
    volumes:
      - .:/workspace
    devices:
      - /dev/kfd
      - /dev/dri
    group_add:
      - video
    security_opt:
      - seccomp=unconfined
    shm_size: "2gb"

  app-mlx:
    profiles: ["legacy-mlx"]
    build:
      context: .
      dockerfile: containers/mlx/Dockerfile
      args:
        MLX_VARIANT: mlx_cpu
    image: sdr_os-mlx
    volumes:
      - .:/workspace
    shm_size: "2gb"
```

**Step 2: Validate compose file**

Run: `docker compose config --quiet`
Expected: Exit 0, no errors

**Step 3: Commit**

```bash
git add docker-compose.yml
git commit -m "feat: multi-service docker-compose with NATS, Caddy, service isolation"
```

---

## Task 6: Create genesis_bridge Python Package - SHM Ringbuffer

This is the core data structure from the design doc. We build and test just the ringbuffer in isolation.

**Files:**
- Create: `genesis_bridge/__init__.py`
- Create: `genesis_bridge/shm/__init__.py`
- Create: `genesis_bridge/shm/ringbuffer.py`
- Create: `genesis_bridge/shm/protocol.py`
- Test: `tests/test_shm_ringbuffer.py`

### Step 1: Write the failing tests

Create `tests/test_shm_ringbuffer.py`:

```python
"""Tests for the SHM ringbuffer protocol.

Uses a temp file instead of /dev/shm so tests run anywhere.
"""

import os
import struct
import tempfile
import zlib

import pytest

from genesis_bridge.shm.protocol import FrameHeader, FLAG_KEYFRAME, CODEC_H264, CODEC_HEVC
from genesis_bridge.shm.ringbuffer import ShmWriter, ShmReader


@pytest.fixture
def shm_path(tmp_path):
    """Create a temp file for SHM ringbuffer testing."""
    path = tmp_path / "test_frames"
    return str(path)


class TestFrameHeader:
    def test_pack_unpack_roundtrip(self):
        hdr = FrameHeader(
            frame_id=1,
            frame_seq=42,
            size=1024,
            flags=FLAG_KEYFRAME,
            codec=CODEC_H264,
            crc32=0xDEADBEEF,
        )
        data = hdr.pack()
        assert len(data) == 32

        unpacked = FrameHeader.unpack(data)
        assert unpacked.frame_id == 1
        assert unpacked.frame_seq == 42
        assert unpacked.size == 1024
        assert unpacked.flags == FLAG_KEYFRAME
        assert unpacked.codec == CODEC_H264
        assert unpacked.crc32 == 0xDEADBEEF

    def test_is_keyframe(self):
        hdr = FrameHeader(frame_id=0, frame_seq=0, size=0, flags=FLAG_KEYFRAME, codec=CODEC_H264, crc32=0)
        assert hdr.is_keyframe

        hdr2 = FrameHeader(frame_id=0, frame_seq=0, size=0, flags=0, codec=CODEC_H264, crc32=0)
        assert not hdr2.is_keyframe


class TestShmRingbuffer:
    def test_write_then_read_single_frame(self, shm_path):
        buf_size = 4 * 1024 * 1024  # 4MB
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        payload = b"fake H.264 NAL unit data" * 100
        writer.write_frame(payload, codec=CODEC_H264, keyframe=True)

        header, data = reader.read_frame()
        assert header is not None
        assert data == payload
        assert header.is_keyframe
        assert header.codec == CODEC_H264
        assert header.crc32 == zlib.crc32(payload) & 0xFFFFFFFF

        writer.close()
        reader.close()

    def test_sequential_writes_increment_sequence(self, shm_path):
        buf_size = 4 * 1024 * 1024
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        for i in range(5):
            payload = f"frame-{i}".encode()
            writer.write_frame(payload, codec=CODEC_H264, keyframe=(i == 0))

        # Reader should get the latest frame (not all 5)
        header, data = reader.read_frame()
        assert header is not None
        assert header.frame_seq == 5  # 1-indexed, 5 writes

        writer.close()
        reader.close()

    def test_crc32_mismatch_detected(self, shm_path):
        buf_size = 4 * 1024 * 1024
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        payload = b"valid data"
        writer.write_frame(payload, codec=CODEC_H264, keyframe=True)

        # Corrupt the payload in the mmapped region
        # Find the payload offset: metadata(16) + header(32) = 48
        reader._mm[48:58] = b"XXXXXXXXXX"

        header, data = reader.read_frame()
        # Should return None on CRC mismatch
        assert header is None

        writer.close()
        reader.close()

    def test_lap_detection(self, shm_path):
        """Writer wraps around the buffer, reader detects lap."""
        buf_size = 1024  # Tiny buffer to force wrapping
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        # Write a frame, read it
        writer.write_frame(b"A" * 100, codec=CODEC_H264, keyframe=True)
        header, data = reader.read_frame()
        assert header is not None

        # Write many more frames to wrap around
        for i in range(20):
            writer.write_frame(f"B{i:03d}".encode() * 20, codec=CODEC_H264, keyframe=(i == 0))

        # Reader should detect the lap (frame_seq jumped) and still return latest
        header, data = reader.read_frame()
        assert header is not None
        assert header.frame_seq > 1  # Sequence advanced past our first read

        writer.close()
        reader.close()

    def test_reader_returns_none_when_no_new_frame(self, shm_path):
        buf_size = 4 * 1024 * 1024
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        # Write and read once
        writer.write_frame(b"data", codec=CODEC_H264, keyframe=True)
        header, data = reader.read_frame()
        assert header is not None

        # Read again without writing - should return None
        header2, data2 = reader.read_frame()
        assert header2 is None

        writer.close()
        reader.close()

    def test_hevc_codec(self, shm_path):
        buf_size = 4 * 1024 * 1024
        writer = ShmWriter(shm_path, buf_size)
        reader = ShmReader(shm_path, buf_size)

        payload = b"hevc data"
        writer.write_frame(payload, codec=CODEC_HEVC, keyframe=False)

        header, data = reader.read_frame()
        assert header.codec == CODEC_HEVC
        assert not header.is_keyframe

        writer.close()
        reader.close()
```

### Step 2: Run tests to verify they fail

Run: `cd /home/ethan/SDR_OS && python -m pytest tests/test_shm_ringbuffer.py -v`
Expected: FAIL - `ModuleNotFoundError: No module named 'genesis_bridge'`

### Step 3: Write the protocol module

Create `genesis_bridge/__init__.py`:

```python
"""Genesis simulation bridge for SDR_OS."""
```

Create `genesis_bridge/shm/__init__.py`:

```python
"""Shared memory primitives for zero-copy frame transport."""

from genesis_bridge.shm.protocol import FrameHeader, FLAG_KEYFRAME, CODEC_H264, CODEC_HEVC
from genesis_bridge.shm.ringbuffer import ShmWriter, ShmReader

__all__ = [
    "FrameHeader",
    "FLAG_KEYFRAME",
    "CODEC_H264",
    "CODEC_HEVC",
    "ShmWriter",
    "ShmReader",
]
```

Create `genesis_bridge/shm/protocol.py`:

```python
"""SHM ringbuffer frame header protocol.

Frame Header (32 bytes):
    frame_id:   uint64  # Application-level ID
    frame_seq:  uint64  # Monotonic, never resets (lap detection)
    size:       uint32  # Payload size in bytes
    flags:      uint16  # FLAG_KEYFRAME, etc.
    codec:      uint16  # 1=H.264, 2=HEVC
    crc32:      uint32  # zlib.crc32 of payload
    reserved:   uint32
"""

import struct
from dataclasses import dataclass

# Header format: 2 uint64, 1 uint32, 1 uint16, 1 uint16, 2 uint32 = 32 bytes
HEADER_FORMAT = "<QQIHHiI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
assert HEADER_SIZE == 32, f"Header must be 32 bytes, got {HEADER_SIZE}"

# Flags
FLAG_KEYFRAME = 0x0001
FLAG_WRAP_MARKER = 0x0002

# Codec IDs
CODEC_H264 = 1
CODEC_HEVC = 2

# Ringbuffer metadata offsets (first 16 bytes of SHM region)
META_SIZE = 16
META_SEQ_OFFSET = 0       # uint64: sequence_number (written LAST)
META_WIDX_OFFSET = 8      # uint64: write_index


@dataclass
class FrameHeader:
    frame_id: int
    frame_seq: int
    size: int
    flags: int
    codec: int
    crc32: int
    reserved: int = 0

    @property
    def is_keyframe(self) -> bool:
        return bool(self.flags & FLAG_KEYFRAME)

    def pack(self) -> bytes:
        return struct.pack(
            HEADER_FORMAT,
            self.frame_id,
            self.frame_seq,
            self.size,
            self.flags,
            self.codec,
            self.crc32,
            self.reserved,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "FrameHeader":
        fields = struct.unpack(HEADER_FORMAT, data)
        return cls(
            frame_id=fields[0],
            frame_seq=fields[1],
            size=fields[2],
            flags=fields[3],
            codec=fields[4],
            crc32=fields[5],
            reserved=fields[6],
        )
```

### Step 4: Write the ringbuffer implementation

Create `genesis_bridge/shm/ringbuffer.py`:

```python
"""SHM ringbuffer writer and reader.

Layout:
    [0:8]   sequence_number (uint64, written LAST - commit marker)
    [8:16]  write_index (uint64)
    [16:]   frame data (header + payload)

Atomic commit pattern:
    Writer: write header+payload, then write_index, then sequence LAST
    Reader: read seq1 -> read write_index -> read data -> read seq2
            accept only if seq1 == seq2

Drop policy: "latest wins" - reader always gets the most recent frame.
"""

import mmap
import os
import struct
import zlib

from genesis_bridge.shm.protocol import (
    CODEC_H264,
    FLAG_KEYFRAME,
    HEADER_SIZE,
    META_SEQ_OFFSET,
    META_SIZE,
    META_WIDX_OFFSET,
    FrameHeader,
)


class ShmWriter:
    """Writes encoded video frames to a shared memory ringbuffer."""

    def __init__(self, path: str, buf_size: int):
        self._path = path
        self._buf_size = buf_size
        self._data_size = buf_size - META_SIZE
        self._seq = 0
        self._frame_id = 0

        # Create and size the backing file
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
        os.ftruncate(fd, buf_size)
        self._fd = fd
        self._mm = mmap.mmap(fd, buf_size)

        # Zero metadata
        self._mm[0:META_SIZE] = b"\x00" * META_SIZE

    def write_frame(self, payload: bytes, codec: int = CODEC_H264, keyframe: bool = False) -> None:
        """Write a single frame (header + payload) into the ringbuffer.

        Overwrites whatever was there before (latest-wins single-slot).
        """
        self._seq += 1
        self._frame_id += 1

        flags = FLAG_KEYFRAME if keyframe else 0
        crc = zlib.crc32(payload) & 0xFFFFFFFF

        header = FrameHeader(
            frame_id=self._frame_id,
            frame_seq=self._seq,
            size=len(payload),
            flags=flags,
            codec=codec,
            crc32=crc,
        )

        frame_data = header.pack() + payload
        frame_len = len(frame_data)

        if frame_len > self._data_size:
            raise ValueError(
                f"Frame ({frame_len}B) exceeds data region ({self._data_size}B)"
            )

        # Write frame data at the start of data region
        write_offset = META_SIZE
        self._mm[write_offset : write_offset + frame_len] = frame_data

        # Commit: write write_index, then sequence LAST
        struct.pack_into("<Q", self._mm, META_WIDX_OFFSET, frame_len)
        struct.pack_into("<Q", self._mm, META_SEQ_OFFSET, self._seq)

    def close(self) -> None:
        self._mm.close()
        os.close(self._fd)


class ShmReader:
    """Reads encoded video frames from a shared memory ringbuffer."""

    def __init__(self, path: str, buf_size: int):
        self._path = path
        self._buf_size = buf_size
        self._last_seq = 0

        fd = os.open(path, os.O_RDONLY)
        self._fd = fd
        self._mm = mmap.mmap(fd, buf_size, access=mmap.ACCESS_READ)

    def read_frame(self) -> tuple:
        """Read the latest frame if available.

        Returns:
            (FrameHeader, bytes) on success
            (None, None) if no new frame or CRC mismatch
        """
        # Atomic read: seq1 -> data -> seq2, accept if seq1 == seq2
        seq1 = struct.unpack_from("<Q", self._mm, META_SEQ_OFFSET)[0]

        if seq1 == 0 or seq1 == self._last_seq:
            return (None, None)

        write_len = struct.unpack_from("<Q", self._mm, META_WIDX_OFFSET)[0]

        # Read frame data
        read_offset = META_SIZE
        frame_data = bytes(self._mm[read_offset : read_offset + write_len])

        # Verify sequence didn't change during read
        seq2 = struct.unpack_from("<Q", self._mm, META_SEQ_OFFSET)[0]
        if seq1 != seq2:
            return (None, None)  # Writer was mid-write, skip

        if len(frame_data) < HEADER_SIZE:
            return (None, None)

        header = FrameHeader.unpack(frame_data[:HEADER_SIZE])
        payload = frame_data[HEADER_SIZE:]

        # CRC check
        expected_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if expected_crc != header.crc32:
            return (None, None)

        self._last_seq = seq1
        return (header, payload)

    def close(self) -> None:
        self._mm.close()
        os.close(self._fd)
```

### Step 5: Run tests to verify they pass

Run: `cd /home/ethan/SDR_OS && python -m pytest tests/test_shm_ringbuffer.py -v`
Expected: All 7 tests PASS

### Step 6: Commit

```bash
git add genesis_bridge/ tests/test_shm_ringbuffer.py
git commit -m "feat: SHM ringbuffer with atomic commit protocol and CRC validation"
```

---

## Task 7: Add pytest + Test Dependencies to pyproject.toml

**Files:**
- Modify: `pyproject.toml`

### Step 1: Add test dependencies

Add to the `[dependency-groups]` section:

```toml
dev = [
    "pytest>=8.0",
    "pytest-asyncio>=0.23",
]
```

Also add a `[tool.pytest.ini_options]` section:

```toml
[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = ["test_*.py"]
python_functions = ["test_*"]
```

### Step 2: Install test deps

Run: `cd /home/ethan/SDR_OS && uv sync --group dev`
Expected: pytest installed successfully

### Step 3: Run the test suite

Run: `cd /home/ethan/SDR_OS && python -m pytest tests/ -v`
Expected: All SHM ringbuffer tests pass

### Step 4: Commit

```bash
git add pyproject.toml uv.lock
git commit -m "chore: add pytest to dev dependencies"
```

---

## Task 8: Create justfile for Common Operations

**Files:**
- Create: `justfile`

### Step 1: Write justfile

```just
# SDR_OS Task Runner
# Usage: just <recipe>

# Default recipe
default:
    @just --list

# ─── Development ─────────────────────────────────────────

# Run all tests
test:
    python -m pytest tests/ -v

# Run tests with coverage
test-cov:
    python -m pytest tests/ -v --cov=genesis_bridge --cov=ros_bridge

# Build webpack frontend
build:
    npm run build

# Dev mode (webpack watch)
dev:
    npm run dev

# Run Express server
serve:
    node server.js

# ─── Docker ──────────────────────────────────────────────

# Start core services (web + nats + caddy)
up:
    docker compose up -d

# Start with simulation
up-sim:
    docker compose --profile sim up -d

# Start with training
up-train:
    docker compose --profile train up -d

# Stop all services
down:
    docker compose --profile sim --profile train down

# View logs
logs *ARGS:
    docker compose logs {{ ARGS }}

# Rebuild containers
rebuild:
    docker compose build --no-cache

# ─── Validation ──────────────────────────────────────────

# Validate docker-compose.yml
check-compose:
    docker compose config --quiet && echo "docker-compose.yml OK"

# Check NATS connectivity
check-nats:
    docker compose exec nats nats-server --signal ldm

# Full validation
check: check-compose test
```

### Step 2: Verify justfile parses

Run: `cd /home/ethan/SDR_OS && just --list`
Expected: Lists all recipes

### Step 3: Commit

```bash
git add justfile
git commit -m "feat: add justfile task runner for dev and docker operations"
```

---

## Task 9: NVENC Validation Script

**Files:**
- Create: `scripts/validate-nvenc.sh`

### Step 1: Write the validation script

```bash
#!/usr/bin/env bash
# Validate NVENC availability inside a CUDA container.
# Usage: docker compose run --rm genesis-sim bash scripts/validate-nvenc.sh

set -euo pipefail

echo "=== NVENC Validation ==="
echo

# Check NVIDIA driver
if ! command -v nvidia-smi &>/dev/null; then
    echo "FAIL: nvidia-smi not found"
    exit 1
fi

echo "GPU Info:"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
echo

# Check for NVENC library
NVENC_LIB=$(ldconfig -p 2>/dev/null | grep -i nvencode | head -1 || true)
if [ -z "$NVENC_LIB" ]; then
    # Try direct path check
    if [ -f /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1 ]; then
        NVENC_LIB="/usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1"
    elif [ -f /usr/lib/libnvidia-encode.so.1 ]; then
        NVENC_LIB="/usr/lib/libnvidia-encode.so.1"
    fi
fi

if [ -z "$NVENC_LIB" ]; then
    echo "WARN: libnvidia-encode not found via ldconfig"
    echo "      NVENC may still work if driver provides it at runtime"
else
    echo "NVENC library: $NVENC_LIB"
fi
echo

# Check CUDA availability via Python
echo "CUDA Python check:"
python3 -c "
import subprocess, sys
try:
    import torch
    print(f'  PyTorch CUDA: {torch.cuda.is_available()}')
    if torch.cuda.is_available():
        print(f'  Device: {torch.cuda.get_device_name(0)}')
        print(f'  VRAM: {torch.cuda.get_device_properties(0).total_mem / 1e9:.1f} GB')
except ImportError:
    print('  PyTorch not installed (optional)')

try:
    result = subprocess.run(
        ['nvidia-smi', '--query-gpu=encoder.stats.sessionCount,encoder.stats.averageFps',
         '--format=csv,noheader'],
        capture_output=True, text=True
    )
    print(f'  NVENC sessions: {result.stdout.strip()}')
except Exception as e:
    print(f'  NVENC session query failed: {e}')
" 2>&1 || echo "  Python CUDA check failed (non-fatal)"

echo
echo "=== Validation complete ==="
```

### Step 2: Make executable

Run: `chmod +x scripts/validate-nvenc.sh`

### Step 3: Commit

```bash
git add scripts/validate-nvenc.sh
git commit -m "feat: add NVENC validation script for GPU container verification"
```

---

## Task 10: SHM Ringbuffer Benchmark Script

**Files:**
- Create: `tests/benchmarks/bench_shm_ringbuffer.py`

### Step 1: Write benchmark

```python
"""Benchmark the SHM ringbuffer write/read throughput.

Simulates the genesis-sim → transport-server data path.
Run: python tests/benchmarks/bench_shm_ringbuffer.py
"""

import os
import tempfile
import time

from genesis_bridge.shm.protocol import CODEC_H264
from genesis_bridge.shm.ringbuffer import ShmReader, ShmWriter


def bench_write_read(frame_size: int, num_frames: int, buf_size: int = 4 * 1024 * 1024):
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "bench_frames")
        writer = ShmWriter(path, buf_size)
        reader = ShmReader(path, buf_size)

        payload = os.urandom(frame_size)

        # Benchmark writes
        t0 = time.perf_counter()
        for i in range(num_frames):
            writer.write_frame(payload, codec=CODEC_H264, keyframe=(i % 60 == 0))
        write_elapsed = time.perf_counter() - t0

        # Benchmark read (single latest frame)
        t0 = time.perf_counter()
        header, data = reader.read_frame()
        read_elapsed = time.perf_counter() - t0

        writer.close()
        reader.close()

        return write_elapsed, read_elapsed, header is not None


def main():
    print("SHM Ringbuffer Benchmark")
    print("=" * 60)

    scenarios = [
        ("720p H.264 (~50KB)", 50 * 1024, 1000),
        ("1080p H.264 (~150KB)", 150 * 1024, 1000),
        ("1080p HEVC (~100KB)", 100 * 1024, 1000),
        ("4K H.264 (~500KB)", 500 * 1024, 500),
    ]

    for name, frame_size, num_frames in scenarios:
        write_t, read_t, success = bench_write_read(frame_size, num_frames)
        write_fps = num_frames / write_t
        write_mbps = (frame_size * num_frames) / write_t / 1e6

        print(f"\n{name}:")
        print(f"  Write: {write_fps:.0f} fps ({write_mbps:.0f} MB/s) over {num_frames} frames")
        print(f"  Read (latest): {read_t * 1e6:.1f} us (success={success})")

    print(f"\n{'=' * 60}")
    print("Done.")


if __name__ == "__main__":
    main()
```

### Step 2: Run benchmark

Run: `cd /home/ethan/SDR_OS && python tests/benchmarks/bench_shm_ringbuffer.py`
Expected: Prints throughput numbers. Write should be >10,000 fps for small frames (memcpy-bound).

### Step 3: Commit

```bash
git add tests/benchmarks/bench_shm_ringbuffer.py
git commit -m "feat: add SHM ringbuffer throughput benchmark"
```

---

## Task 11: Update Documentation

**Files:**
- Modify: `documents/SETUP.md` (add Docker Compose setup instructions)
- Modify: `documents/TECH_SPEC.md` (add service architecture reference)
- Modify: `CLAUDE.md` (add new services to project overview)

### Step 1: Update SETUP.md

Add a new section for Docker Compose multi-service setup:

```markdown
## Docker Compose (Multi-Service)

### Prerequisites
- Docker Engine 24+ with Compose v2
- NVIDIA Container Toolkit (for GPU services)
- `just` task runner (optional): `cargo install just` or via package manager

### Quick Start (Core Services)

```bash
# Start web server + NATS + Caddy
docker compose up -d

# Check services are healthy
docker compose ps

# View logs
docker compose logs -f
```

### Simulation Profile

```bash
# Start core + genesis-sim + ros-bridge
docker compose --profile sim up -d

# Validate NVENC inside the container
docker compose run --rm genesis-sim bash scripts/validate-nvenc.sh
```

### Available Profiles
| Profile | Services |
|---------|----------|
| (default) | webserver, nats, caddy |
| sim | + genesis-sim, ros-bridge |
| train | + training-runner |
```

### Step 2: Update CLAUDE.md

Add to the Architecture section, after the Backend subsection:

```markdown
### Services (`services/`, `genesis_bridge/`, `ros_bridge/`)
- **genesis_bridge/** - Python package: SHM ringbuffer, NVENC encoding, simulation I/O
- **ros_bridge/** - Python package: ROS2 ↔ NATS relay with topic allowlist
- **configs/** - Caddy, NATS, Prometheus configuration files
- **scripts/** - Host tuning, GPU validation, IRQ affinity scripts
```

### Step 3: Commit

```bash
git add documents/SETUP.md documents/TECH_SPEC.md CLAUDE.md
git commit -m "docs: update setup, tech spec, and CLAUDE.md for multi-service architecture"
```

---

## Task 12: Integration Smoke Test

**Files:**
- Create: `tests/test_compose_config.py`

### Step 1: Write the test

```python
"""Smoke tests for Docker Compose configuration validity."""

import subprocess


def test_compose_config_valid():
    """docker-compose.yml must parse without errors."""
    result = subprocess.run(
        ["docker", "compose", "config", "--quiet"],
        capture_output=True,
        text=True,
        cwd="/home/ethan/SDR_OS",
    )
    assert result.returncode == 0, f"docker-compose.yml invalid:\n{result.stderr}"


def test_compose_services_present():
    """All expected services must be defined."""
    result = subprocess.run(
        ["docker", "compose", "config", "--services"],
        capture_output=True,
        text=True,
        cwd="/home/ethan/SDR_OS",
    )
    services = set(result.stdout.strip().split("\n"))
    expected = {"caddy", "webserver", "nats"}
    assert expected.issubset(services), f"Missing services: {expected - services}"


def test_compose_sim_profile_services():
    """Sim profile must include genesis-sim and ros-bridge."""
    result = subprocess.run(
        ["docker", "compose", "--profile", "sim", "config", "--services"],
        capture_output=True,
        text=True,
        cwd="/home/ethan/SDR_OS",
    )
    services = set(result.stdout.strip().split("\n"))
    expected = {"genesis-sim", "ros-bridge"}
    assert expected.issubset(services), f"Missing sim services: {expected - services}"
```

### Step 2: Run the smoke tests

Run: `cd /home/ethan/SDR_OS && python -m pytest tests/test_compose_config.py -v`
Expected: All 3 tests PASS (requires docker CLI available)

### Step 3: Commit

```bash
git add tests/test_compose_config.py
git commit -m "test: add Docker Compose config smoke tests"
```

---

## Task Summary

| # | Task | Key Deliverable |
|---|------|-----------------|
| 1 | Update .gitignore | Service artifact ignores |
| 2 | NATS configuration | `configs/nats.conf` |
| 3 | Caddyfile | `configs/Caddyfile` |
| 4 | ROS 2 Jazzy container | `containers/ros-jazzy/Dockerfile` + robot configs (from Genesis) |
| 5 | docker-compose.yml rewrite | Multi-service, multi-network compose |
| 6 | SHM ringbuffer (TDD) | `genesis_bridge/shm/` with 7 tests |
| 7 | Test dependencies | pytest in pyproject.toml |
| 8 | justfile | Task runner for dev/docker |
| 9 | NVENC validation | `scripts/validate-nvenc.sh` |
| 10 | SHM benchmark | Throughput measurement |
| 11 | Documentation | SETUP.md, TECH_SPEC.md, CLAUDE.md |
| 12 | Integration smoke test | Compose config validation |

**Estimated commits:** 12 (one per task)
