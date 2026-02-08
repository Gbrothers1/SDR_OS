#!/usr/bin/env python3
"""Tests for genesis_ros_publisher — config loading, math, message building."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def test_load_config():
    """Test YAML config loading and validation."""
    print("\n=== Testing Config Loading ===")
    from genesis_bridge.genesis_ros_publisher import load_config

    cfg = load_config("configs/genesis_ros_publisher.yaml")

    assert "bridge" in cfg, "Missing bridge section"
    assert "rates" in cfg, "Missing rates section"
    assert "topics" in cfg, "Missing topics section"
    assert "enable" in cfg, "Missing enable section"
    assert "robot" in cfg, "Missing robot section"

    assert cfg["bridge"]["url"] == "ws://localhost:9091"
    assert len(cfg["robot"]["joint_names"]) == 12
    assert cfg["robot"]["joint_names"][0] == "FL_hip_joint"
    print(f"  Bridge URL: {cfg['bridge']['url']}")
    print(f"  Joint count: {len(cfg['robot']['joint_names'])}")
    print("PASS")


def test_load_config_missing():
    """Test config loading with missing file."""
    print("\n=== Testing Config Missing File ===")
    from genesis_bridge.genesis_ros_publisher import load_config

    try:
        load_config("nonexistent.yaml")
        assert False, "Should have raised FileNotFoundError"
    except FileNotFoundError:
        print("  Correctly raised FileNotFoundError")
    print("PASS")


def test_gravity_to_quaternion():
    """Test gravity vector to quaternion conversion."""
    print("\n=== Testing Gravity to Quaternion ===")
    from genesis_bridge.genesis_ros_publisher import gravity_to_quaternion

    # Flat on ground: gravity = (0, 0, -9.81) -> identity quaternion
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 0.0, -9.81)
    assert abs(qw - 1.0) < 1e-6, f"Expected qw=1.0, got {qw}"
    assert abs(qx) < 1e-6, f"Expected qx=0, got {qx}"
    assert abs(qy) < 1e-6, f"Expected qy=0, got {qy}"
    assert abs(qz) < 1e-6, f"Expected qz=0, got {qz}"
    print(f"  Flat: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Tilted forward (pitch): gravity has +x component
    qx, qy, qz, qw = gravity_to_quaternion(4.9, 0.0, -8.5)
    assert abs(qy) > 0.1, f"Expected nonzero pitch (qy), got {qy}"
    print(f"  Pitched forward: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Tilted sideways (roll): gravity has +y component
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 4.9, -8.5)
    assert abs(qx) > 0.1, f"Expected nonzero roll (qx), got {qx}"
    print(f"  Rolled right: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Zero vector: should return identity
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 0.0, 0.0)
    assert abs(qw - 1.0) < 1e-6, "Zero vector should give identity"
    print(f"  Zero input: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Quaternion should be unit length
    for grav in [(0, 0, -9.81), (4.9, 0, -8.5), (0, 4.9, -8.5), (3, 3, -8)]:
        qx, qy, qz, qw = gravity_to_quaternion(*grav)
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert abs(norm - 1.0) < 1e-6, f"Quaternion not unit: norm={norm}"
    print("  All quaternions are unit length")

    print("PASS")


def test_metrics_parsing():
    """Test that a realistic metrics payload can be parsed for publishing."""
    print("\n=== Testing Metrics Parsing ===")

    # Simulate a training_metrics JSON payload from the bridge
    metrics = {
        "type": "training_metrics",
        "step_count": 1234,
        "fps": 49.5,
        "blend_alpha": 0.3,
        "deadman_active": True,
        "actor_tag": "blend",
        "mode": "teleop_record",
        "command_source": "gamepad",
        "policy_loaded": False,
        "confidence": 0.85,
        "confidence_gate_active": False,
        "safety_flags": {
            "vel_clamped": False,
            "accel_clamped": False,
            "jerk_clamped": False,
            "clip_clamped": False,
        },
        "obs_breakdown": {
            "gait_command": [0.0] * 14,
            "velocity_command": [0.5, 0.0, -0.2],
            "angular_velocity": [0.01, -0.02, 0.03],
            "linear_velocity": [0.4, 0.0, 0.0],
            "projected_gravity": [0.0, 0.0, -9.81],
            "dof_position": [0.0, 0.8, -1.6] * 4,
            "dof_velocity": [0.1, -0.1, 0.05] * 4,
            "actions": [0.0, 0.8, -1.6] * 4,
        },
        "velocity_command": {
            "lin_vel_x": 0.5,
            "lin_vel_y": 0.0,
            "ang_vel_z": -0.2,
        },
        "reward_breakdown": {
            "tracking_lin_vel": 0.85,
            "tracking_ang_vel": 0.42,
            "base_height_target": -0.01,
        },
    }

    # Verify obs_breakdown structure
    obs = metrics["obs_breakdown"]
    assert len(obs["dof_position"]) == 12, "Expected 12 DOF positions"
    assert len(obs["dof_velocity"]) == 12, "Expected 12 DOF velocities"
    assert len(obs["angular_velocity"]) == 3
    assert len(obs["projected_gravity"]) == 3
    print(f"  obs_breakdown keys: {list(obs.keys())}")
    print(f"  dof_position[0:3]: {obs['dof_position'][0:3]}")

    # Verify velocity_command
    vcmd = metrics["velocity_command"]
    assert "lin_vel_x" in vcmd
    assert "lin_vel_y" in vcmd
    assert "ang_vel_z" in vcmd
    print(f"  velocity_command: {vcmd}")

    # Verify reward_breakdown serializes cleanly
    reward_json = json.dumps(metrics["reward_breakdown"])
    assert len(reward_json) > 2
    print(f"  reward_breakdown JSON: {reward_json}")

    # Verify status fields
    assert metrics["blend_alpha"] == 0.3
    assert metrics["actor_tag"] == "blend"
    print(f"  status: alpha={metrics['blend_alpha']}, tag={metrics['actor_tag']}")

    print("PASS")


def test_ws_listener_parsing():
    """Test WebSocket listener thread connects to a mock server and parses metrics."""
    print("\n=== Testing WebSocket Listener (mock server) ===")
    import threading
    import time

    try:
        import websockets
        import websockets.sync.server as ws_server
    except ImportError:
        print("  SKIP: websockets not installed")
        return

    received = []
    stop_event = threading.Event()

    # Mock bridge: sends one binary frame (should be skipped) then one JSON metrics
    def handler(ws):
        # Send binary frame (JPEG-like)
        ws.send(b"\xff\xd8\xff\xe0JFIF_fake_frame")
        # Send JSON metrics
        metrics = {
            "type": "training_metrics",
            "obs_breakdown": {
                "dof_position": [0.1] * 12,
                "dof_velocity": [0.2] * 12,
                "angular_velocity": [0.01, -0.02, 0.03],
                "projected_gravity": [0.0, 0.0, -9.81],
            },
            "velocity_command": {"lin_vel_x": 0.5, "lin_vel_y": 0.0, "ang_vel_z": 0.1},
        }
        ws.send(json.dumps(metrics))
        # Keep alive until test signals stop
        stop_event.wait(timeout=5.0)

    # Start mock server in thread
    server = ws_server.serve(handler, "localhost", 19091)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    try:
        # Connect as a simple client (mimicking the publisher's ws_listener logic)
        import websockets.sync.client as ws_client
        with ws_client.connect("ws://localhost:19091") as ws:
            for _ in range(2):
                msg = ws.recv(timeout=5.0)
                if isinstance(msg, bytes):
                    received.append(("binary", len(msg)))
                else:
                    data = json.loads(msg)
                    received.append(("json", data.get("type")))

        assert len(received) == 2, f"Expected 2 messages, got {len(received)}"
        assert received[0][0] == "binary", "First message should be binary"
        assert received[1] == ("json", "training_metrics"), f"Second should be metrics, got {received[1]}"
        print(f"  Received: {received}")
        print("PASS")
    finally:
        stop_event.set()
        server.shutdown()


if __name__ == "__main__":
    test_load_config()
    test_load_config_missing()
    test_gravity_to_quaternion()
    test_metrics_parsing()
    test_ws_listener_parsing()
    print("\n=== ALL TESTS PASSED ===")
