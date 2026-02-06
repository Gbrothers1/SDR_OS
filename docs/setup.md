# Setup

First-time environment setup for SDR_OS development.

## Requirements

| Tool | Version | Purpose | Install |
|------|---------|---------|---------|
| **Python** | ≥3.10, <3.14 | Genesis, PyTorch, RL pipeline | System or [pyenv](https://github.com/pyenv/pyenv) |
| **uv** | ≥0.9.30 | Python package management | `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| **pnpm** | ≥10.x | Node.js package management | `curl -fsSL https://get.pnpm.io/install.sh \| sh -` |
| **Node.js** | 20.x | Express server, Webpack | Via pnpm or [nvm](https://github.com/nvm-sh/nvm) |
| **Docker** | 24+ with Compose v2 | Backend containers | [docs.docker.com](https://docs.docker.com/engine/install/) |

!!! warning "Use uv and pnpm"
    Do **not** use `pip` or `npm`/`yarn` directly. The project enforces lockfiles (`uv.lock`, `pnpm-lock.yaml`) for reproducibility.

### GPU-specific

| Target | Additional requirements |
|--------|----------------------|
| CUDA | NVIDIA driver 590+, CUDA 13.1, `nvidia-container-toolkit` |
| ROCm | ROCm drivers, `/dev/kfd` accessible |
| MLX | macOS 14+ on Apple Silicon (no extra setup) |

??? info "CUDA installation (Ubuntu 22.04)"

    The project defaults to **CUDA 13.1** with **NVIDIA Driver 590.48.01**. Full guide: [CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/).

    **Verify GPU:**

    ```bash
    lspci | grep -i nvidia
    ```

    **Verify driver and CUDA:**

    ```bash
    nvidia-smi          # shows driver version and CUDA version
    nvcc --version      # shows CUDA compiler version
    ```

    **Install via package manager (Ubuntu 22.04):**

    ```bash
    # Install keyring
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring_1.1-1_all.deb

    # Install CUDA toolkit
    sudo apt update
    sudo apt install cuda-toolkit

    # Install nvidia-container-toolkit (for Docker GPU access)
    # See: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
    ```

    **Post-install (add to `~/.bashrc`):**

    ```bash
    export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    ```

    **Supported host compilers:** GCC 6.x--15.x, Clang 7.x--21.x (see [system requirements](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#system-requirements)).

## Known-good versions

Versions are tracked per release tag. Use `git checkout <tag>` to get a reproducible snapshot.
Full spec: [TECH_SPEC.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/TECH_SPEC.md).

### `v1.0.1.0` (latest)

| Component | Version |
|-----------|---------|
| Python | 3.12.12 |
| uv | 0.9.30 |
| pnpm | 10.28.2 |
| Node.js | 20.19.6 |
| NVIDIA Driver | 590.48.01 |
| CUDA | 13.1 |
| PyTorch | 2.10.0+cu128 |
| Genesis | 0.3.13 |
| Genesis Forge | 0.3.0 |
| NumPy | 2.3.5 |
| OpenCV | 4.13.0 |
| MuJoCo | 3.4.0 |

??? note "`v0.1.0` (initial)"

    | Component | Version |
    |-----------|---------|
    | Python | 3.12.12 |
    | uv | 0.9.30 |
    | pnpm | 10.28.2 |
    | Node.js | 20.19.6 |
    | PyTorch | 2.10.0+cu128 |
    | Genesis | 0.3.13 |
    | Genesis Forge | 0.3.0 |

!!! tip "Updating this table"
    When you tag a new release, add a new `### vX.Y.Z` section above with the current versions. Move the previous latest into a collapsed `??? note` block. Run `uv pip list` and `node --version` to capture current versions.

## Quick start

```bash
# 1. Clone
git clone https://github.com/Gbrothers1/SDR_OS.git
cd SDR_OS

# 2. Python environment
uv sync                    # installs all deps from uv.lock

# 3. Node.js environment
pnpm install               # installs from pnpm-lock.yaml

# 4. Build the web frontend
pnpm run build             # webpack → dist/bundle.js

# 5. Start the server
pnpm start                 # Express on http://localhost:3000

# 6. Verify Genesis (optional, requires GPU)
uv run python -c "import genesis; print(genesis.__version__)"
```

## Python environment details

### Install PyTorch

=== "CUDA (NVIDIA)"

    Default: CUDA 13.1, Driver 590.48.01

    ```bash
    # CUDA 13.1 (default for this project)
    uv pip install torch --index-url https://download.pytorch.org/whl/cu131

    # Or CUDA 12.6 if on an older driver
    uv pip install torch --index-url https://download.pytorch.org/whl/cu126
    ```

=== "ROCm (AMD)"

    The recommended way is via the official ROCm wheels:

    ```bash
    # ROCm 7.0 (check pytorch.org for latest ROCm version)
    pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/rocm7.0
    ```

    Or use the prebuilt ROCm Docker image (recommended for production):

    ```bash
    docker pull rocm/pytorch:latest
    docker run -it --device=/dev/kfd --device=/dev/dri --group-add video \
        --ipc=host --shm-size 8G rocm/pytorch:latest
    ```

    Full guide: [PyTorch on ROCm installation](https://rocm.docs.amd.com/projects/install-on-linux/en/develop/install/3rd-party/pytorch-install.html)

=== "CPU only"

    ```bash
    uv pip install torch --index-url https://download.pytorch.org/whl/cpu
    ```

=== "Apple Silicon (MPS)"

    ```bash
    uv pip install torch
    ```

### Dependency groups

The project uses uv dependency groups in `pyproject.toml`:

| Group | What it installs |
|-------|-----------------|
| (default) | genesis-forge, genesis-world |
| `cuda` | mlx[cuda12] |
| `rocm` | mlx[cpu] |
| `mlx` | mlx (Apple Silicon native) |
| `mlx_cpu` | mlx[cpu] (Linux CPU parity) |
| `mlx_cuda12` | mlx[cuda12] |
| `mlx_cuda13` | mlx[cuda13] |
| `docs` | mkdocs, mkdocs-material, mike, pymdown-extensions |

```bash
# Sync a specific group
uv sync --group cuda
uv sync --group docs

# Sync only one group (avoids resolving everything else)
uv sync --only-group docs
```

## ROS 2 bridge (real robot mode)

Required for ROS 2 communication. The browser connects to rosbridge via ROSLIB.

```bash
# Install rosbridge_server (ROS 2 Jazzy)
sudo apt install ros-jazzy-rosbridge-server

# Launch
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Default URL: `ws://localhost:9090`. Configurable via localStorage key `rosBridgeUrl` in the browser.

## Common uv commands

| Command | Description |
|---------|-------------|
| `uv sync` | Install deps from pyproject.toml + uv.lock |
| `uv add <pkg>` | Add a dependency |
| `uv remove <pkg>` | Remove a dependency |
| `uv run <cmd>` | Run command in the venv |
| `uv pip list` | List installed packages |

## Common pnpm commands

| Command | Description |
|---------|-------------|
| `pnpm install` | Install deps from lockfile |
| `pnpm add <pkg>` | Add a dependency |
| `pnpm run <script>` | Run a script from package.json |
| `pnpm list` | List installed packages |

## References

- [documents/SETUP.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/SETUP.md) — Detailed setup guide.
- [documents/TECH_SPEC.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/TECH_SPEC.md) — Hardware and software specs.
- [ref/web/reff_web_memory.md](https://github.com/Gbrothers1/SDR_OS/blob/main/ref/web/reff_web_memory.md) — Bookmarked reference URLs.
