# Real Hardware Observation Pipeline Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a unified observation pipeline that builds identical observation vectors from both Genesis simulation and real hardware sensors, enabling sim-to-real policy transfer.

**Architecture:** A new `ObservationSpec` class (mirroring the existing `ActionSpec`) reads a per-robot observation layout from YAML configs. An `ObservationBuilder` class consumes ROS sensor data (JointState, IMU, F/T, contacts) and assembles a flat observation vector matching the spec. This builder is used by `gs_ros_bridge.py` for HIL inference, `teleop_record.py` for BC data collection, and a new `RealRobotVecEnv` for on-hardware evaluation. The bridge server also gets wired to populate `current_obs` from the Genesis env state.

**Tech Stack:** Python 3, NumPy, PyTorch, ROS2 (rclpy, sensor_msgs, geometry_msgs), YAML configs

---

### Task 1: Create ObservationSpec with YAML Config

**Files:**
- Create: `rl/observations/__init__.py`
- Create: `rl/observations/obs_spec.py`
- Create: `rl/observations/test_obs_spec.py`
- Modify: `rl/configs/default.yaml` (add `observation_space` section)

This task defines the formal observation layout. `ObservationSpec` mirrors `ActionSpec` — it reads segments from YAML and provides methods to validate and query the layout.

**Step 1: Add observation_space to default.yaml**

Add this section to `rl/configs/default.yaml` under `env:`:

```yaml
env:
  num_envs: 4096
  num_observations: 48
  num_actions: 12
  dt: 0.02
  episode_length: 500

  observation_space:
    segments:
      - name: base_ang_vel
        size: 3
        source: imu.angular_velocity
      - name: projected_gravity
        size: 3
        source: imu.projected_gravity
      - name: joint_pos
        size: 12
        source: joint_state.position
      - name: joint_vel
        size: 12
        source: joint_state.velocity
      - name: prev_action
        size: 12
        source: prev_action
      - name: desired_vel
        size: 3
        source: command
      - name: foot_contacts
        size: 3
        source: contacts
        default: 0.0
    # Total: 3+3+12+12+12+3+3 = 48
```

**Step 2: Write the failing test**

Create `rl/observations/test_obs_spec.py`:

```python
"""Tests for ObservationSpec."""
import pytest
import numpy as np
from pathlib import Path


def test_obs_spec_loads_from_config():
    """ObservationSpec loads segments from default.yaml."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    assert spec.num_obs == 48
    assert len(spec.segments) == 7
    assert spec.segments[0].name == "base_ang_vel"
    assert spec.segments[0].size == 3


def test_obs_spec_segment_index_lookup():
    """Can look up start index and size by segment name."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    start, size = spec.get_segment("joint_pos")
    assert size == 12
    # base_ang_vel(3) + projected_gravity(3) = 6
    assert start == 6


def test_obs_spec_validates_total():
    """Sum of segment sizes must equal num_observations."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    total = sum(s.size for s in spec.segments)
    assert total == spec.num_obs


def test_obs_spec_empty_vector():
    """empty_vector() returns zeros of correct shape."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    vec = spec.empty_vector()
    assert vec.shape == (48,)
    assert vec.dtype == np.float32
    assert np.all(vec == 0.0)


def test_obs_spec_set_segment():
    """set_segment() writes values into the correct slice."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    vec = spec.empty_vector()
    vals = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    spec.set_segment(vec, "base_ang_vel", vals)
    assert np.allclose(vec[:3], vals)
    assert np.all(vec[3:] == 0.0)


def test_obs_spec_set_segment_wrong_size_raises():
    """set_segment() raises if values don't match segment size."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    vec = spec.empty_vector()
    with pytest.raises(ValueError):
        spec.set_segment(vec, "base_ang_vel", np.array([1.0, 2.0]))


def test_obs_spec_get_segment_values():
    """get_segment_values() reads the correct slice."""
    from rl.observations.obs_spec import ObservationSpec
    spec = ObservationSpec()
    vec = spec.empty_vector()
    vec[6:18] = np.arange(12, dtype=np.float32)
    vals = spec.get_segment_values(vec, "joint_pos")
    assert np.allclose(vals, np.arange(12, dtype=np.float32))
```

**Step 3: Run tests to verify they fail**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/observations/test_obs_spec.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'rl.observations'`

**Step 4: Create the __init__.py**

Create `rl/observations/__init__.py`:

```python
from .obs_spec import ObservationSpec, ObservationSegment

__all__ = ["ObservationSpec", "ObservationSegment"]
```

**Step 5: Implement ObservationSpec**

Create `rl/observations/obs_spec.py`:

```python
"""ObservationSpec — formal observation vector layout loaded from YAML config.

Mirrors ActionSpec: reads segments from config, provides helpers to build
and query flat observation vectors.
"""
import numpy as np
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional


@dataclass
class ObservationSegment:
    """One named slice of the observation vector."""
    name: str
    start: int
    size: int
    source: str
    default: float = 0.0


class ObservationSpec:
    """Reads observation layout from YAML and provides vector helpers.

    Usage:
        spec = ObservationSpec()                   # loads default.yaml
        spec = ObservationSpec("path/to/config.yaml")

        vec = spec.empty_vector()                  # np.zeros(48, dtype=float32)
        spec.set_segment(vec, "joint_pos", values) # write into correct slice
        vals = spec.get_segment_values(vec, "joint_pos")
    """

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = str(
                Path(__file__).parent.parent / "configs" / "default.yaml"
            )
        self._load_config(config_path)

    def _load_config(self, config_path: str):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        env_cfg = cfg.get("env", {})
        self.num_obs: int = env_cfg.get("num_observations", 48)

        obs_cfg = env_cfg.get("observation_space", {})
        raw_segments = obs_cfg.get("segments", [])

        self.segments: List[ObservationSegment] = []
        offset = 0
        for seg in raw_segments:
            s = ObservationSegment(
                name=seg["name"],
                start=offset,
                size=seg["size"],
                source=seg.get("source", ""),
                default=seg.get("default", 0.0),
            )
            self.segments.append(s)
            offset += seg["size"]

        if offset != self.num_obs:
            raise ValueError(
                f"Observation segments sum to {offset}, "
                f"but num_observations={self.num_obs}"
            )

        self._by_name = {s.name: s for s in self.segments}

    def get_segment(self, name: str) -> Tuple[int, int]:
        """Return (start_index, size) for a named segment."""
        seg = self._by_name[name]
        return seg.start, seg.size

    def empty_vector(self) -> np.ndarray:
        """Return a zero-filled observation vector."""
        vec = np.zeros(self.num_obs, dtype=np.float32)
        for seg in self.segments:
            if seg.default != 0.0:
                vec[seg.start : seg.start + seg.size] = seg.default
        return vec

    def set_segment(self, vec: np.ndarray, name: str, values: np.ndarray):
        """Write values into the named segment of vec (in-place)."""
        seg = self._by_name[name]
        if len(values) != seg.size:
            raise ValueError(
                f"Segment '{name}' expects {seg.size} values, got {len(values)}"
            )
        vec[seg.start : seg.start + seg.size] = values

    def get_segment_values(self, vec: np.ndarray, name: str) -> np.ndarray:
        """Read the named segment from vec."""
        seg = self._by_name[name]
        return vec[seg.start : seg.start + seg.size].copy()

    @property
    def segment_names(self) -> List[str]:
        return [s.name for s in self.segments]
```

**Step 6: Run tests to verify they pass**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/observations/test_obs_spec.py -v`
Expected: All 7 tests PASS

**Step 7: Commit**

```bash
git add rl/observations/__init__.py rl/observations/obs_spec.py rl/observations/test_obs_spec.py rl/configs/default.yaml
git commit -m "feat: add ObservationSpec with YAML-driven observation layout"
```

---

### Task 2: Create ObservationBuilder for Real Sensor Data

**Files:**
- Create: `rl/observations/obs_builder.py`
- Create: `rl/observations/test_obs_builder.py`

ObservationBuilder consumes sensor messages (as dicts mirroring ROS msg fields) and assembles the flat observation vector using ObservationSpec. It is pure Python with no ROS dependency — ROS callbacks feed it data.

**Step 1: Write the failing test**

Create `rl/observations/test_obs_builder.py`:

```python
"""Tests for ObservationBuilder."""
import pytest
import numpy as np


def test_builder_produces_correct_shape():
    """Builder outputs a (48,) float32 vector."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    obs = builder.build()
    assert obs.shape == (48,)
    assert obs.dtype == np.float32


def test_builder_incorporates_joint_state():
    """Joint positions and velocities appear in correct segments."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    positions = np.ones(12, dtype=np.float32) * 0.5
    velocities = np.ones(12, dtype=np.float32) * 0.1
    builder.update_joint_state(positions, velocities)
    obs = builder.build()
    # joint_pos starts at index 6 (after base_ang_vel=3 + projected_gravity=3)
    assert np.allclose(obs[6:18], 0.5)
    assert np.allclose(obs[18:30], 0.1)


def test_builder_incorporates_imu():
    """IMU angular velocity and projected gravity appear in correct segments."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    ang_vel = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    orientation_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # w,x,y,z identity
    builder.update_imu(ang_vel, orientation_quat)
    obs = builder.build()
    # base_ang_vel at index 0:3
    assert np.allclose(obs[0:3], ang_vel)
    # projected_gravity at index 3:6 — identity quat -> gravity = [0, 0, -1]
    assert np.allclose(obs[3:6], [0.0, 0.0, -1.0], atol=1e-5)


def test_builder_incorporates_prev_action():
    """Previous action appears in correct segment."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    action = np.ones(12, dtype=np.float32) * 0.7
    builder.update_prev_action(action)
    obs = builder.build()
    # prev_action starts at index 30 (3+3+12+12)
    assert np.allclose(obs[30:42], 0.7)


def test_builder_incorporates_command():
    """Desired velocity command appears in correct segment."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    cmd = np.array([1.0, 0.0, 0.5], dtype=np.float32)
    builder.update_command(cmd)
    obs = builder.build()
    # desired_vel at index 42 (3+3+12+12+12)
    assert np.allclose(obs[42:45], cmd)


def test_builder_handles_fewer_joints():
    """If robot has fewer joints than segment size, pads with zeros."""
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder()
    positions = np.ones(7, dtype=np.float32)  # Franka has 7 joints
    velocities = np.ones(7, dtype=np.float32)
    builder.update_joint_state(positions, velocities)
    obs = builder.build()
    assert np.allclose(obs[6:13], 1.0)   # first 7 filled
    assert np.allclose(obs[13:18], 0.0)  # remaining 5 zero-padded


def test_builder_staleness_check():
    """is_stale() returns True when data is older than threshold."""
    import time
    from rl.observations.obs_builder import ObservationBuilder
    builder = ObservationBuilder(stale_threshold_s=0.05)
    builder.update_joint_state(np.zeros(12), np.zeros(12))
    assert not builder.is_stale()
    time.sleep(0.06)
    assert builder.is_stale()


def test_projected_gravity_tilted():
    """Projected gravity with a 90-degree pitch gives [1, 0, 0]."""
    from rl.observations.obs_builder import ObservationBuilder
    import math
    builder = ObservationBuilder()
    # 90-degree pitch around Y axis: quat = [cos(45), 0, sin(45), 0] (w,x,y,z)
    c = math.cos(math.pi / 4)
    s = math.sin(math.pi / 4)
    quat = np.array([c, 0.0, s, 0.0], dtype=np.float32)  # w,x,y,z
    builder.update_imu(np.zeros(3), quat)
    obs = builder.build()
    # Gravity projected into body frame with 90deg pitch -> [1, 0, 0] or [-1, 0, 0]
    proj_grav = obs[3:6]
    assert abs(abs(proj_grav[0]) - 1.0) < 0.01
    assert abs(proj_grav[1]) < 0.01
    assert abs(proj_grav[2]) < 0.01
```

**Step 2: Run tests to verify they fail**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/observations/test_obs_builder.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'rl.observations.obs_builder'`

**Step 3: Implement ObservationBuilder**

Create `rl/observations/obs_builder.py`:

```python
"""ObservationBuilder — assembles flat observation vectors from sensor data.

Pure Python, no ROS dependency. ROS callbacks (or other sources) call
update_*() methods; build() returns the current observation vector.
"""
import time
import numpy as np
from typing import Optional
from .obs_spec import ObservationSpec


def _quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q (w,x,y,z format).

    Used to project world-frame gravity into the body frame.
    """
    w, x, y, z = q_wxyz
    # Inverse of unit quaternion is conjugate
    # v_body = q* . v . q  (Hamilton product shortcut)
    t = 2.0 * np.cross(np.array([x, y, z]), v)
    return v + w * t + np.cross(np.array([x, y, z]), t)


class ObservationBuilder:
    """Builds a flat observation vector from sensor updates.

    Usage:
        builder = ObservationBuilder()               # or pass config_path
        builder.update_joint_state(positions, velocities)
        builder.update_imu(angular_velocity, orientation_quat)
        builder.update_prev_action(action)
        builder.update_command(desired_vel)
        obs = builder.build()                        # -> np.ndarray(48,)
    """

    GRAVITY_WORLD = np.array([0.0, 0.0, -1.0], dtype=np.float32)

    def __init__(
        self,
        config_path: Optional[str] = None,
        stale_threshold_s: float = 0.1,
    ):
        self.spec = ObservationSpec(config_path)
        self.stale_threshold_s = stale_threshold_s

        # Cached sensor data
        self._joint_pos: Optional[np.ndarray] = None
        self._joint_vel: Optional[np.ndarray] = None
        self._ang_vel = np.zeros(3, dtype=np.float32)
        self._orientation_quat = np.array([1, 0, 0, 0], dtype=np.float32)  # w,x,y,z
        self._prev_action = np.zeros(
            self.spec.get_segment("prev_action")[1], dtype=np.float32
        )
        self._command = np.zeros(3, dtype=np.float32)
        self._contacts = np.zeros(
            self.spec.get_segment("foot_contacts")[1], dtype=np.float32
        )

        self._last_update_time: float = 0.0

    def update_joint_state(
        self, positions: np.ndarray, velocities: np.ndarray
    ):
        """Update joint positions and velocities from JointState msg."""
        self._joint_pos = np.asarray(positions, dtype=np.float32)
        self._joint_vel = np.asarray(velocities, dtype=np.float32)
        self._last_update_time = time.monotonic()

    def update_imu(
        self, angular_velocity: np.ndarray, orientation_quat_wxyz: np.ndarray
    ):
        """Update IMU data. Quaternion in (w, x, y, z) order."""
        self._ang_vel = np.asarray(angular_velocity, dtype=np.float32)
        self._orientation_quat = np.asarray(
            orientation_quat_wxyz, dtype=np.float32
        )
        self._last_update_time = time.monotonic()

    def update_prev_action(self, action: np.ndarray):
        """Update the previous action (from the last control step)."""
        self._prev_action = np.asarray(action, dtype=np.float32)

    def update_command(self, desired_vel: np.ndarray):
        """Update desired velocity command (e.g. from joystick)."""
        self._command = np.asarray(desired_vel, dtype=np.float32)

    def update_contacts(self, contacts: np.ndarray):
        """Update contact sensor readings."""
        self._contacts = np.asarray(contacts, dtype=np.float32)

    def is_stale(self) -> bool:
        """True if no sensor update within stale_threshold_s."""
        if self._last_update_time == 0.0:
            return True
        return (time.monotonic() - self._last_update_time) > self.stale_threshold_s

    def build(self) -> np.ndarray:
        """Assemble and return the flat observation vector."""
        vec = self.spec.empty_vector()

        # base_ang_vel
        self.spec.set_segment(vec, "base_ang_vel", self._ang_vel)

        # projected_gravity: rotate world gravity into body frame
        proj_grav = _quat_rotate_inverse(
            self._orientation_quat, self.GRAVITY_WORLD
        )
        self.spec.set_segment(vec, "projected_gravity", proj_grav)

        # joint_pos — pad/truncate to segment size
        _, jp_size = self.spec.get_segment("joint_pos")
        jp = np.zeros(jp_size, dtype=np.float32)
        if self._joint_pos is not None:
            n = min(len(self._joint_pos), jp_size)
            jp[:n] = self._joint_pos[:n]
        self.spec.set_segment(vec, "joint_pos", jp)

        # joint_vel — pad/truncate to segment size
        _, jv_size = self.spec.get_segment("joint_vel")
        jv = np.zeros(jv_size, dtype=np.float32)
        if self._joint_vel is not None:
            n = min(len(self._joint_vel), jv_size)
            jv[:n] = self._joint_vel[:n]
        self.spec.set_segment(vec, "joint_vel", jv)

        # prev_action
        _, pa_size = self.spec.get_segment("prev_action")
        pa = np.zeros(pa_size, dtype=np.float32)
        n = min(len(self._prev_action), pa_size)
        pa[:n] = self._prev_action[:n]
        self.spec.set_segment(vec, "prev_action", pa)

        # desired_vel (command)
        self.spec.set_segment(vec, "desired_vel", self._command)

        # foot_contacts
        self.spec.set_segment(vec, "foot_contacts", self._contacts)

        return vec
```

**Step 4: Update `rl/observations/__init__.py`**

```python
from .obs_spec import ObservationSpec, ObservationSegment
from .obs_builder import ObservationBuilder

__all__ = ["ObservationSpec", "ObservationSegment", "ObservationBuilder"]
```

**Step 5: Run tests to verify they pass**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/observations/test_obs_builder.py -v`
Expected: All 8 tests PASS

**Step 6: Commit**

```bash
git add rl/observations/obs_builder.py rl/observations/__init__.py rl/observations/test_obs_builder.py
git commit -m "feat: add ObservationBuilder for assembling obs from sensor data"
```

---

### Task 3: Fix GenesisVecEnv Mock Mode with Structured Observations

**Files:**
- Modify: `rl/envs/genesis_vecenv.py` (replace random noise with ObservationBuilder-driven mock)
- Create: `rl/envs/test_vecenv_mock.py`

Currently `GenesisVecEnv` mock mode uses `torch.randn * 0.1` for observations — pure noise with no structure, no action response, and random rewards. This makes eval, training, and any pipeline testing meaningless. This task adds a lightweight mock dynamics model so actions affect observations and rewards reflect behavior, letting the full eval stack work without Genesis or a real robot.

**Step 1: Write the failing test**

Create `rl/envs/test_vecenv_mock.py`:

```python
"""Tests for GenesisVecEnv mock mode with structured observations."""
import pytest
import torch
import numpy as np


def test_mock_reset_shape():
    """reset() returns structured obs of correct shape."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=4, num_obs=48, num_actions=12)
    obs = env.reset()
    assert obs.shape == (4, 48)
    assert obs.dtype == torch.float32


def test_mock_obs_has_gravity():
    """After reset, projected gravity segment should be non-zero (body upright -> [0,0,-1])."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    obs = env.reset()
    # projected_gravity is at index 3:6
    grav = obs[0, 3:6]
    assert torch.allclose(grav, torch.tensor([0.0, 0.0, -1.0]), atol=0.01)


def test_mock_actions_affect_joint_pos():
    """Applying nonzero action should change joint_pos in next obs."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    obs0 = env.reset()
    jp_before = obs0[0, 6:18].clone()
    action = torch.ones(1, 12) * 0.5
    obs1, _, _, _ = env.step(action)
    jp_after = obs1[0, 6:18]
    assert not torch.allclose(jp_before, jp_after), "Joint positions should change after action"


def test_mock_prev_action_in_obs():
    """Previous action should appear in the prev_action segment after step."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    env.reset()
    action = torch.ones(1, 12) * 0.3
    obs, _, _, _ = env.step(action)
    # prev_action at index 30:42
    assert torch.allclose(obs[0, 30:42], action[0], atol=1e-5)


def test_mock_rewards_not_random():
    """Rewards should be deterministic for same state+action, not random noise."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    torch.manual_seed(42)
    env1 = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    env1.reset()
    _, r1, _, _ = env1.step(torch.zeros(1, 12))

    torch.manual_seed(42)
    env2 = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    env2.reset()
    _, r2, _, _ = env2.step(torch.zeros(1, 12))

    assert torch.allclose(r1, r2), "Same seed + action should produce same reward"


def test_mock_episode_termination():
    """Episode should terminate after max_episode_length steps."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=1, num_obs=48, num_actions=12)
    env.reset()
    action = torch.zeros(1, 12)
    for i in range(499):
        _, _, done, _ = env.step(action)
        assert not done.any(), f"Should not be done at step {i+1}"
    _, _, done, _ = env.step(action)
    assert done.any(), "Should be done at step 500"


def test_mock_multi_env():
    """Multiple envs should have independent state."""
    from rl.envs.genesis_vecenv import GenesisVecEnv
    env = GenesisVecEnv(env=None, num_envs=4, num_obs=48, num_actions=12)
    env.reset()
    # Different actions for each env
    actions = torch.zeros(4, 12)
    actions[0] = 1.0
    actions[1] = -1.0
    obs, _, _, _ = env.step(actions)
    # Env 0 and env 1 should have different joint positions
    assert not torch.allclose(obs[0, 6:18], obs[1, 6:18])
```

**Step 2: Run tests to verify they fail**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/envs/test_vecenv_mock.py -v`
Expected: Several failures — gravity segment is random noise, actions don't affect obs, rewards are random.

**Step 3: Implement structured mock mode in GenesisVecEnv**

Modify `rl/envs/genesis_vecenv.py`. Add import at top:

```python
from rl.observations import ObservationSpec
```

Add a `_MockState` helper class and modify the mock paths in `reset()` and `step()`:

```python
class _MockState:
    """Lightweight mock dynamics for testing without Genesis."""

    def __init__(self, num_envs: int, obs_spec: ObservationSpec, device: str, dt: float = 0.02):
        self.num_envs = num_envs
        self.spec = obs_spec
        self.device = device
        self.dt = dt

        # State: joint positions and velocities
        _, self.jp_size = obs_spec.get_segment("joint_pos")
        _, self.jv_size = obs_spec.get_segment("joint_vel")
        self.joint_pos = torch.zeros(num_envs, self.jp_size, device=device)
        self.joint_vel = torch.zeros(num_envs, self.jv_size, device=device)
        self.prev_action = torch.zeros(num_envs, obs_spec.get_segment("prev_action")[1], device=device)

    def reset(self, env_ids=None):
        if env_ids is None:
            self.joint_pos.zero_()
            self.joint_vel.zero_()
            self.prev_action.zero_()
        else:
            self.joint_pos[env_ids] = 0
            self.joint_vel[env_ids] = 0
            self.prev_action[env_ids] = 0

    def step(self, actions: torch.Tensor):
        """Simple integrator: action -> velocity -> position."""
        n = min(actions.shape[1], self.jv_size)
        # Action drives velocity (with damping)
        self.joint_vel[:, :n] = 0.8 * self.joint_vel[:, :n] + 0.2 * actions[:, :n]
        # Integrate position
        self.joint_pos += self.joint_vel * self.dt
        # Clamp positions to [-pi, pi]
        self.joint_pos.clamp_(-3.14159, 3.14159)
        # Store prev action
        self.prev_action = actions.clone()

    def build_obs(self) -> torch.Tensor:
        """Build structured observation tensor for all envs."""
        obs = torch.zeros(self.num_envs, self.spec.num_obs, device=self.device)
        # base_ang_vel (index 0:3) — stays zero (no base rotation in mock)
        # projected_gravity (index 3:6) — upright body
        obs[:, 3:6] = torch.tensor([0.0, 0.0, -1.0], device=self.device)
        # joint_pos
        s, sz = self.spec.get_segment("joint_pos")
        obs[:, s:s+sz] = self.joint_pos
        # joint_vel
        s, sz = self.spec.get_segment("joint_vel")
        obs[:, s:s+sz] = self.joint_vel
        # prev_action
        s, sz = self.spec.get_segment("prev_action")
        obs[:, s:s+sz] = self.prev_action
        # desired_vel and foot_contacts stay zero
        return obs

    def compute_reward(self) -> torch.Tensor:
        """Simple reward: penalize large velocities and joint displacement."""
        vel_penalty = -0.1 * self.joint_vel.pow(2).sum(dim=1)
        pos_penalty = -0.01 * self.joint_pos.pow(2).sum(dim=1)
        return vel_penalty + pos_penalty
```

Then modify the `__init__` method — after `self._extras = {}` add:

```python
# Mock dynamics (used when env=None)
self._mock = None
if self._env is None:
    try:
        obs_spec = ObservationSpec()
        self._mock = _MockState(num_envs, obs_spec, device)
    except Exception:
        pass  # Fall back to random noise if ObservationSpec not available
```

Replace the mock branch in `reset()`:

```python
# In reset(), replace the mock observation block:
if self._env is None:
    if self._mock is not None:
        self._mock.reset(env_ids=None if env_ids is None else env_ids)
        self._obs = self._mock.build_obs()
    else:
        self._obs = torch.randn(self._num_envs, self._num_obs, device=self._device) * 0.1
```

And the partial reset branch similarly:

```python
# Partial reset branch:
if self._env is None:
    if self._mock is not None:
        self._mock.reset(env_ids)
        self._obs[env_ids] = self._mock.build_obs()[env_ids]
    else:
        self._obs[env_ids] = torch.randn(len(env_ids), self._num_obs, device=self._device) * 0.1
```

Replace the mock branch in `step()`:

```python
# In step(), replace the mock step block:
if self._env is None:
    if self._mock is not None:
        self._mock.step(actions)
        self._obs = self._mock.build_obs()
        rewards = self._mock.compute_reward()
    else:
        self._obs = torch.randn(self._num_envs, self._num_obs, device=self._device) * 0.1
        rewards = torch.randn(self._num_envs, device=self._device) * 0.1
    dones = self._episode_lengths >= self.max_episode_length
    self._extras = {}
```

**Step 4: Run tests to verify they pass**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/envs/test_vecenv_mock.py -v`
Expected: All 7 tests PASS

**Step 5: Verify eval_policy.py works end-to-end**

```bash
cd /home/ethan/dev/Genesis/SDR_OS
python rl/scripts/eval_policy.py --checkpoint nonexistent --num-envs 4 --episodes 10
```
Expected: Runs with mock mode, prints structured eval results (negative rewards from velocity/position penalties, consistent per seed, episode lengths = 500).

**Step 6: Commit**

```bash
git add rl/envs/genesis_vecenv.py rl/envs/test_vecenv_mock.py
git commit -m "feat: structured mock dynamics in GenesisVecEnv for eval without hardware"
```

---

### Task 4: Wire ObservationBuilder into gs_ros_bridge.py (requires ROS2)

**Files:**
- Modify: `genesis_bridge/gs_ros_bridge.py` (replace `joint_state_to_obs()` with ObservationBuilder, add IMU/F-T to obs)

This task replaces the incomplete `joint_state_to_obs()` with the ObservationBuilder, making HIL inference use full 48-dim observations from real sensors.

**Step 1: Read and understand the current code**

Read these sections of `genesis_bridge/gs_ros_bridge.py`:
- Constructor `__init__` around lines 70-120 (member variables)
- `joint_state_to_obs()` at lines 306-325
- `joint_state_callback()` at lines 274-280
- `imu_callback()` at line 298
- `ft_callback()` at line 302

**Step 2: Add ObservationBuilder import and initialization**

At the top of `gs_ros_bridge.py`, add:

```python
from rl.observations import ObservationBuilder
```

In `__init__()`, after `self.current_obs = None`, add:

```python
self.obs_builder = ObservationBuilder()
```

**Step 3: Replace joint_state_to_obs with ObservationBuilder calls**

Replace the `joint_state_callback` method body:

```python
def joint_state_callback(self, msg: 'JointState'):
    self.current_joint_state = msg
    positions = np.array(msg.position[:self.num_joints], dtype=np.float32)
    velocities = np.array(msg.velocity[:self.num_joints], dtype=np.float32)
    self.obs_builder.update_joint_state(positions, velocities)
    self.current_obs = self.obs_builder.build()
```

**Step 4: Wire IMU callback to ObservationBuilder**

Replace the `imu_callback` method body:

```python
def imu_callback(self, msg: 'Imu'):
    self.current_imu = msg
    ang_vel = np.array([
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    ], dtype=np.float32)
    # ROS quaternion is (x,y,z,w), ObservationBuilder expects (w,x,y,z)
    quat_wxyz = np.array([
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
    ], dtype=np.float32)
    self.obs_builder.update_imu(ang_vel, quat_wxyz)
```

**Step 5: Delete the old `joint_state_to_obs` method**

Remove lines 306-325 (the entire `joint_state_to_obs` method). It is no longer called.

**Step 6: Test manually**

Run (if ROS2 is available):
```bash
cd /home/ethan/dev/Genesis/SDR_OS
python -c "from genesis_bridge.gs_ros_bridge import GenesisROSBridge; print('import OK')"
```
Expected: No import errors.

If no ROS2 available, verify the import path is correct:
```bash
python -c "from rl.observations import ObservationBuilder; print('OK')"
```

**Step 7: Commit**

```bash
git add genesis_bridge/gs_ros_bridge.py
git commit -m "feat: wire ObservationBuilder into gs_ros_bridge for full 48-dim obs"
```

---

### Task 5: Wire bridge_server.py to Populate current_obs

**Files:**
- Modify: `genesis_bridge/bridge_server.py` (populate `current_obs` from sim state or ObservationBuilder)

Currently `self.current_obs` is initialized to `None` and never set, making policy inference dead code. This task fixes that.

**Step 1: Read the relevant sections**

Read `bridge_server.py`:
- `self.current_obs = None` initialization (line ~384)
- `step_simulation()` method (lines ~935-1009), specifically where it checks `self.current_obs is not None`
- `send_env_info()` where `num_observations: 48` is hardcoded

**Step 2: Add ObservationBuilder import**

At the top of `bridge_server.py`, add:

```python
from rl.observations import ObservationBuilder, ObservationSpec
```

**Step 3: Initialize ObservationBuilder in constructor**

In `__init__`, after `self.current_obs = None`:

```python
self.obs_builder = ObservationBuilder()
self.obs_spec = ObservationSpec()
```

**Step 4: Populate current_obs in step_simulation()**

In `step_simulation()`, after the Genesis scene step produces state (look for where `self.scene.step()` is called), add observation building. The exact location depends on whether Genesis provides joint state after stepping. Add after the scene step:

```python
# Build observation from sim state
if self.scene is not None and self.scene_built:
    try:
        # If Genesis env provides observations directly
        if hasattr(self, '_genesis_env') and self._genesis_env is not None:
            self.current_obs = self._genesis_env.get_obs()
        else:
            # Build from available state
            obs = self.obs_builder.build()
            self.current_obs = obs
    except Exception as e:
        logger.debug(f"Could not build observation: {e}")
```

**Step 5: Update send_env_info to use ObservationSpec**

Replace the hardcoded `"num_observations": 48` with:

```python
"num_observations": self.obs_spec.num_obs,
```

**Step 6: Test the import chain**

```bash
cd /home/ethan/dev/Genesis/SDR_OS
python -c "
from rl.observations import ObservationBuilder, ObservationSpec
spec = ObservationSpec()
builder = ObservationBuilder()
print(f'num_obs={spec.num_obs}, build shape={builder.build().shape}')
"
```
Expected: `num_obs=48, build shape=(48,)`

**Step 7: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: wire ObservationBuilder into bridge_server for policy inference"
```

---

### Task 6: Update teleop_record.py to Use Real Observations

**Files:**
- Modify: `rl/scripts/teleop_record.py` (replace mock observations with ObservationBuilder)

This task makes teleop recording collect real sensor observations instead of random noise, enabling BC training on real data.

**Step 1: Read current teleop_record.py**

Read `rl/scripts/teleop_record.py` fully. Note:
- Lines 63-74: `obs = np.random.randn(obs_dim).astype(np.float32) * 0.1`
- The script currently has no ROS or sensor connection.

**Step 2: Add imports**

At the top of `teleop_record.py`, add:

```python
from rl.observations import ObservationBuilder
```

**Step 3: Replace mock observation with ObservationBuilder**

Before the recording loop, initialize the builder:

```python
obs_builder = ObservationBuilder()
```

Replace the mock observation line inside the loop:

```python
# OLD: obs = np.random.randn(obs_dim).astype(np.float32) * 0.1
# NEW: Build observation from available sensor data
obs = obs_builder.build()
```

**Step 4: Feed teleop action back as prev_action**

After computing `clamped_action`, add:

```python
obs_builder.update_prev_action(clamped_action)
```

And feed the command velocity from the joystick:

```python
# Update command velocity from joystick
cmd_vel = np.array([
    mock_joystick['linear']['x'],
    mock_joystick['linear']['y'],
    mock_joystick['angular']['z'],
], dtype=np.float32)
obs_builder.update_command(cmd_vel)
```

**Step 5: Add Socket.io connection for real joystick data (optional enhancement)**

Add a `--socket-url` argument and connect to receive real joystick data instead of mock sinusoids. This is optional — the key change is using ObservationBuilder instead of random noise.

Add to argparse:

```python
parser.add_argument('--socket-url', type=str, default=None,
                    help='Socket.io URL for real joystick (e.g. http://localhost:3000)')
```

**Step 6: Test that the script still runs**

```bash
cd /home/ethan/dev/Genesis/SDR_OS
python rl/scripts/teleop_record.py --steps 10 --seed 42 --num-envs 1
```
Expected: Script completes, saves `.npz` file. Observations should be zeros (no sensor connected) instead of random noise.

Verify the file:
```bash
python -c "
import numpy as np
data = np.load('rl/datasets/teleop_episode_0.npz')
print('obs shape:', data['observations'].shape)
print('obs[0] first 6:', data['observations'][0, :6])
print('actions shape:', data['actions_teleop'].shape)
"
```
Expected: `obs shape: (10, 48)`, first 6 values should be 0.0 (no sensor data), actions should be non-zero (from mock joystick).

**Step 7: Commit**

```bash
git add rl/scripts/teleop_record.py
git commit -m "feat: use ObservationBuilder in teleop_record instead of random noise"
```

---

### Task 7: Create RealRobotVecEnv for On-Hardware Evaluation (requires ROS2)

**Files:**
- Create: `rl/envs/real_robot_env.py`
- Create: `rl/envs/test_real_robot_env.py`

A thin env wrapper that implements the same interface as `GenesisVecEnv` but gets observations from `ObservationBuilder` and sends actions via ROS. This allows `eval_policy.py` to run on real hardware.

**Step 1: Write the failing test**

Create `rl/envs/test_real_robot_env.py`:

```python
"""Tests for RealRobotVecEnv (offline/mock mode)."""
import pytest
import torch
import numpy as np


def test_env_reset_returns_correct_shape():
    """reset() returns (1, 48) tensor."""
    from rl.envs.real_robot_env import RealRobotVecEnv
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    obs = env.reset()
    assert obs.shape == (1, 48)
    assert obs.dtype == torch.float32


def test_env_step_returns_correct_shapes():
    """step() returns (obs, rewards, dones, infos) with correct shapes."""
    from rl.envs.real_robot_env import RealRobotVecEnv
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    env.reset()
    action = torch.zeros(1, 12)
    obs, rewards, dones, infos = env.step(action)
    assert obs.shape == (1, 48)
    assert rewards.shape == (1,)
    assert dones.shape == (1,)


def test_env_get_observations():
    """get_observations() returns last obs."""
    from rl.envs.real_robot_env import RealRobotVecEnv
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    obs = env.reset()
    assert torch.allclose(env.get_observations(), obs)


def test_env_properties():
    """num_envs, num_obs, num_actions, device are correct."""
    from rl.envs.real_robot_env import RealRobotVecEnv
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    assert env.num_envs == 1
    assert env.num_obs == 48
    assert env.num_actions == 12


def test_env_feeds_prev_action():
    """After step(), the action is fed back as prev_action in obs."""
    from rl.envs.real_robot_env import RealRobotVecEnv
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    env.reset()
    action = torch.ones(1, 12) * 0.5
    obs, _, _, _ = env.step(action)
    # prev_action segment is at index 30:42 (3+3+12+12 = 30)
    assert torch.allclose(obs[0, 30:42], action[0], atol=1e-5)
```

**Step 2: Run tests to verify they fail**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/envs/test_real_robot_env.py -v`
Expected: FAIL — `ModuleNotFoundError`

**Step 3: Implement RealRobotVecEnv**

Create `rl/envs/real_robot_env.py`:

```python
"""RealRobotVecEnv — VecEnv interface backed by real hardware sensors.

Implements the same interface as GenesisVecEnv so eval_policy.py and
rsl_rl runners can use it transparently. Always num_envs=1.

Usage:
    env = RealRobotVecEnv(num_obs=48, num_actions=12)
    obs = env.reset()
    obs, rewards, dones, infos = env.step(action)
"""
import time
import torch
import numpy as np
from typing import Optional
from rl.observations import ObservationBuilder


class RealRobotVecEnv:
    """Single-env wrapper that reads observations from ObservationBuilder."""

    def __init__(
        self,
        num_obs: int = 48,
        num_actions: int = 12,
        device: str = "cpu",
        config_path: Optional[str] = None,
        dt: float = 0.02,
        action_callback=None,
    ):
        self.num_envs = 1
        self.num_obs = num_obs
        self.num_actions = num_actions
        self.device = device
        self.dt = dt
        self._action_callback = action_callback

        self.obs_builder = ObservationBuilder(config_path)
        self._obs = torch.zeros(1, num_obs, dtype=torch.float32, device=device)
        self._step_count = 0
        self.max_episode_length = 500

    def reset(self) -> torch.Tensor:
        """Reset and return initial observation."""
        self._step_count = 0
        obs_np = self.obs_builder.build()
        self._obs = torch.from_numpy(obs_np).unsqueeze(0).to(self.device)
        return self._obs

    def step(self, actions: torch.Tensor):
        """Execute action, wait dt, read new observation."""
        action_np = actions[0].detach().cpu().numpy()

        # Send action to hardware if callback provided
        if self._action_callback is not None:
            self._action_callback(action_np)

        # Feed action back as prev_action for next observation
        self.obs_builder.update_prev_action(action_np)

        # Wait for next control period
        time.sleep(self.dt)

        # Build new observation
        obs_np = self.obs_builder.build()
        self._obs = torch.from_numpy(obs_np).unsqueeze(0).to(self.device)

        self._step_count += 1
        done = self._step_count >= self.max_episode_length
        reward = torch.zeros(1, dtype=torch.float32, device=self.device)
        dones = torch.tensor([done], dtype=torch.float32, device=self.device)

        return self._obs, reward, dones, {}

    def get_observations(self) -> torch.Tensor:
        """Return last observation tensor."""
        return self._obs
```

**Step 4: Run tests to verify they pass**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/envs/test_real_robot_env.py -v`
Expected: All 5 tests PASS

**Step 5: Commit**

```bash
git add rl/envs/real_robot_env.py rl/envs/test_real_robot_env.py
git commit -m "feat: add RealRobotVecEnv for on-hardware policy evaluation"
```

---

### Task 8: Add --real Flag to eval_policy.py (requires ROS2)

**Files:**
- Modify: `rl/scripts/eval_policy.py` (add `--real` flag that uses `RealRobotVecEnv`)

**Step 1: Read current eval_policy.py**

Read `rl/scripts/eval_policy.py` fully. Note the env creation around lines 120-130.

**Step 2: Add import and CLI flag**

Add import:

```python
from rl.envs.real_robot_env import RealRobotVecEnv
```

Add argument:

```python
parser.add_argument('--real', action='store_true',
                    help='Use real hardware sensors instead of simulation')
```

**Step 3: Switch env creation based on flag**

Replace the env creation block:

```python
if args.real:
    env = RealRobotVecEnv(
        num_obs=obs_dim,
        num_actions=action_dim,
        device=args.device,
    )
    print(f"Using REAL hardware env (num_obs={obs_dim}, num_actions={action_dim})")
else:
    env = GenesisVecEnv(
        env=None,
        num_envs=args.num_envs,
        num_obs=obs_dim,
        num_actions=action_dim,
        device=args.device,
    )
    print(f"Using simulation env (num_envs={args.num_envs})")
```

**Step 4: Test that the flag works**

```bash
cd /home/ethan/dev/Genesis/SDR_OS
python rl/scripts/eval_policy.py --real --num-episodes 1
```
Expected: Script runs with `RealRobotVecEnv`, uses zeros for obs (no sensors connected), completes episode.

**Step 5: Commit**

```bash
git add rl/scripts/eval_policy.py
git commit -m "feat: add --real flag to eval_policy for on-hardware evaluation"
```

---

### Task 9: Run Full Integration Test

**Files:**
- Create: `rl/observations/test_integration.py`

End-to-end test verifying the full pipeline: ObservationSpec -> ObservationBuilder -> obs vector -> BC policy inference.

**Step 1: Write integration test**

Create `rl/observations/test_integration.py`:

```python
"""Integration test: full observation pipeline -> policy inference."""
import pytest
import torch
import numpy as np


def test_full_pipeline_obs_to_policy():
    """Obs built from sensor data can be fed to a BC policy."""
    from rl.observations import ObservationBuilder, ObservationSpec

    spec = ObservationSpec()
    builder = ObservationBuilder()

    # Simulate sensor data arriving
    builder.update_joint_state(
        np.random.randn(12).astype(np.float32) * 0.1,
        np.random.randn(12).astype(np.float32) * 0.01,
    )
    builder.update_imu(
        np.array([0.01, -0.02, 0.03], dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )
    builder.update_prev_action(np.zeros(12, dtype=np.float32))
    builder.update_command(np.array([0.5, 0.0, 0.1], dtype=np.float32))

    obs = builder.build()
    assert obs.shape == (48,)
    assert not np.any(np.isnan(obs))

    # Feed to a simple policy network
    obs_tensor = torch.from_numpy(obs).unsqueeze(0)
    policy = torch.nn.Sequential(
        torch.nn.Linear(48, 256),
        torch.nn.ELU(),
        torch.nn.Linear(256, 12),
        torch.nn.Tanh(),
    )
    with torch.no_grad():
        action = policy(obs_tensor)

    assert action.shape == (1, 12)
    assert torch.all(action >= -1.0) and torch.all(action <= 1.0)


def test_real_robot_env_with_builder():
    """RealRobotVecEnv produces valid obs and accepts actions."""
    from rl.envs.real_robot_env import RealRobotVecEnv

    env = RealRobotVecEnv(num_obs=48, num_actions=12, dt=0.001)
    obs = env.reset()
    assert obs.shape == (1, 48)

    for _ in range(3):
        action = torch.randn(1, 12).clamp(-1, 1)
        obs, reward, done, info = env.step(action)
        assert obs.shape == (1, 48)
        assert not torch.any(torch.isnan(obs))


def test_obs_spec_matches_action_spec():
    """ObservationSpec prev_action segment matches ActionSpec num_actions."""
    from rl.observations import ObservationSpec
    from rl.actions.action_spec import ActionSpec

    obs_spec = ObservationSpec()
    action_spec = ActionSpec()

    _, prev_action_size = obs_spec.get_segment("prev_action")
    assert prev_action_size == action_spec.num_actions, (
        f"prev_action segment ({prev_action_size}) != "
        f"num_actions ({action_spec.num_actions})"
    )


def test_teleop_record_data_format():
    """Verify .npz from teleop_record has correct keys and shapes."""
    from rl.observations import ObservationBuilder
    from rl.actions.teleop_bridge import TeleopBridge
    from rl.actions.action_spec import ActionSpec

    builder = ObservationBuilder()
    teleop = TeleopBridge()
    spec = ActionSpec()

    observations = []
    actions = []
    for i in range(5):
        obs = builder.build()
        observations.append(obs)

        joystick = {
            'linear': {'x': 0.5, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': 0.3},
        }
        action = teleop.get_action(joystick_state=joystick)
        clamped, _ = spec.clamp_action(action, 0.02)
        actions.append(clamped)
        builder.update_prev_action(clamped)

    obs_array = np.stack(observations)
    act_array = np.stack(actions)
    assert obs_array.shape == (5, 48)
    assert act_array.shape == (5, 12)
```

**Step 2: Run all tests**

```bash
cd /home/ethan/dev/Genesis/SDR_OS && python -m pytest rl/observations/ rl/envs/test_real_robot_env.py -v
```
Expected: All tests PASS (spec tests + builder tests + integration tests + env tests).

**Step 3: Commit**

```bash
git add rl/observations/test_integration.py
git commit -m "test: add integration tests for full observation pipeline"
```

---

## Dependency Graph

```
Task 1 (ObservationSpec)
  └─→ Task 2 (ObservationBuilder)
        ├─→ Task 3 (GenesisVecEnv mock mode) ★ PRIORITY — makes eval work now
        ├─→ Task 4 (gs_ros_bridge) [requires ROS2]
        ├─→ Task 5 (bridge_server)
        ├─→ Task 6 (teleop_record)
        └─→ Task 7 (RealRobotVecEnv) [requires ROS2] ─→ Task 8 (eval --real)
  └─→ Task 9 (Integration tests) [after all above]
```

**Recommended execution order without ROS2:** Tasks 1 → 2 → 3 → 5 → 6 → 9
Tasks 4, 7, 8 require ROS2 and can be deferred until a robot is connected.

Tasks 3, 5, 6 are independent of each other (all depend on Tasks 1+2).
Task 8 depends on Task 7.
Task 9 depends on all others.
