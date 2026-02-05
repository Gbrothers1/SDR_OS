# Go2 Velocity Control Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Train a Go2 locomotion policy using genesis-forge, load it in the bridge server so the robot moves from gamepad/ROS velocity commands, add ROS `/cmd_vel` input with source switching, and wire up deadman/safety/observability.

**Architecture:** The gait_trainer example environment trains a PPO policy (48-dim obs → 12-dim actions) via rsl_rl. The bridge loads the checkpoint and calls `policy(obs)` in `_step_forge_env()` instead of zeros. Both gamepad (Socket.io) and ROS `/cmd_vel` write to `VelocityCommandManager`, gated by a `command_source` field. Deadman enforced for gamepad; anomaly detector wired in. Status extended with command source, policy state, deadman.

**Tech Stack:** genesis-forge, rsl_rl (PPO/OnPolicyRunner), rclpy (ROS2 in Docker), Socket.io, React

**Reference files:**
- Gait trainer example: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/`
- Bridge server: `genesis_bridge/bridge_server.py`
- Go2 env: `genesis_bridge/envs/go2_env.py`
- Node server: `server.js`
- Genesis context: `src/client/contexts/GenesisContext.jsx`
- Control panel: `src/client/components/GenesisControlPanel.jsx`

---

### Task 1: Training Script

Create `rl/scripts/train_go2_locomotion.py` that trains a Go2 locomotion policy using the gait_trainer environment.

**Files:**
- Create: `rl/scripts/train_go2_locomotion.py`
- Copy reference: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/environment.py` → `rl/envs/go2_gait_training_env.py`
- Copy reference: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/gait_command_manager.py` → `rl/envs/gait_command_manager.py`

**Step 1: Copy the gait_trainer environment and gait command manager**

Copy `environment.py` to `rl/envs/go2_gait_training_env.py` and `gait_command_manager.py` to `rl/envs/gait_command_manager.py`. Fix the import in the env file:

```python
# In rl/envs/go2_gait_training_env.py, change:
from gait_command_manager import GaitCommandManager
# To:
from rl.envs.gait_command_manager import GaitCommandManager
```

**Step 2: Write the training script**

Create `rl/scripts/train_go2_locomotion.py` modeled on the gait_trainer `train.py`:

```python
#!/usr/bin/env python3
"""Train Go2 locomotion policy using genesis-forge gait trainer environment."""

import os
import sys
import copy
import torch
import shutil
import pickle
import argparse

# Ensure project root is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import genesis as gs
from genesis_forge.wrappers import VideoWrapper, RslRlWrapper
from rl.envs.go2_gait_training_env import Go2GaitTrainingEnv
from rsl_rl.runners import OnPolicyRunner

EXPERIMENT_NAME = "go2-locomotion"

parser = argparse.ArgumentParser(description="Train Go2 locomotion policy")
parser.add_argument("-n", "--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=2000)
parser.add_argument("-d", "--device", type=str, default="gpu")
parser.add_argument("-e", "--exp_name", type=str, default=EXPERIMENT_NAME)
parser.add_argument("--log_dir", type=str, default="rl/checkpoints")
args = parser.parse_args()


def training_cfg(exp_name: str, max_iterations: int, num_envs: int):
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 1.0,
            "class_name": "ActorCritic",
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1,
        "num_steps_per_env": round(98_304 / num_envs),
        "save_interval": 100,
        "empirical_normalization": None,
        "obs_groups": {"policy": ["policy"], "critic": ["policy", "critic"]},
    }


def main():
    backend = gs.gpu
    if args.device == "cpu":
        backend = gs.cpu
        torch.set_default_device("cpu")
    gs.init(logging_level="warning", backend=backend, performance_mode=True)

    log_path = os.path.join(args.log_dir, args.exp_name)
    if os.path.exists(log_path):
        shutil.rmtree(log_path)
    os.makedirs(log_path, exist_ok=True)
    print(f"Logging to: {log_path}")

    cfg = training_cfg(args.exp_name, args.max_iterations, args.num_envs)
    pickle.dump([cfg], open(os.path.join(log_path, "cfgs.pkl"), "wb"))

    env = Go2GaitTrainingEnv(num_envs=args.num_envs, headless=True)
    env = VideoWrapper(
        env,
        video_length_sec=12,
        out_dir=os.path.join(log_path, "videos"),
        episode_trigger=lambda episode_id: episode_id % 2 == 0,
    )
    env = RslRlWrapper(env)
    env.build()
    env.reset()

    print("Training model...")
    runner = OnPolicyRunner(env, copy.deepcopy(cfg), log_path, device=gs.device)
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=False)
    env.close()


if __name__ == "__main__":
    main()
```

**Step 3: Verify script imports resolve**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -c "from rl.envs.go2_gait_training_env import Go2GaitTrainingEnv; print('OK')"`

Expected: `OK` (or import error to fix)

**Step 4: Commit**

```bash
git add rl/envs/go2_gait_training_env.py rl/envs/gait_command_manager.py rl/scripts/train_go2_locomotion.py
git commit -m "feat: add Go2 locomotion training script using gait_trainer env"
```

---

### Task 2: Eval Script

Create `rl/scripts/eval_go2_locomotion.py` to load a checkpoint and visually verify the gait.

**Files:**
- Create: `rl/scripts/eval_go2_locomotion.py`

**Step 1: Write the eval script**

```python
#!/usr/bin/env python3
"""Evaluate a trained Go2 locomotion policy."""

import os
import sys
import glob
import torch
import pickle
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import genesis as gs
from genesis_forge.wrappers import RslRlWrapper
from rl.envs.go2_gait_training_env import Go2GaitTrainingEnv
from rsl_rl.runners import OnPolicyRunner

EXPERIMENT_NAME = "go2-locomotion"

parser = argparse.ArgumentParser(description="Evaluate Go2 locomotion policy")
parser.add_argument("-d", "--device", type=str, default="gpu")
parser.add_argument("-e", "--exp_name", type=str, default=EXPERIMENT_NAME)
parser.add_argument("--log_dir", type=str, default="rl/checkpoints")
parser.add_argument("--checkpoint", type=str, default=None, help="Specific checkpoint path")
args = parser.parse_args()


def get_latest_model(log_dir: str) -> str:
    model_checkpoints = glob.glob(os.path.join(log_dir, "model_*.pt"))
    if len(model_checkpoints) == 0:
        print(f"No model files found at '{log_dir}'")
        sys.exit(1)
    model_checkpoints.sort()
    return model_checkpoints[-1]


def main():
    backend = gs.gpu
    if args.device == "cpu":
        backend = gs.cpu
        torch.set_default_device("cpu")
    gs.init(logging_level="warning", backend=backend)

    log_path = os.path.join(args.log_dir, args.exp_name)
    [cfg] = pickle.load(open(os.path.join(log_path, "cfgs.pkl"), "rb"))
    model = args.checkpoint or get_latest_model(log_path)

    env = Go2GaitTrainingEnv(num_envs=1, headless=False)
    env = RslRlWrapper(env)
    env.build()

    print(f"Loading model: {model}")
    runner = OnPolicyRunner(env, cfg, log_path, device=gs.device)
    runner.load(model)
    policy = runner.get_inference_policy(device=gs.device)

    obs, _ = env.reset()
    try:
        with torch.no_grad():
            while True:
                actions = policy(obs)
                obs, _, _, _ = env.step(actions)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "Viewer closed" not in str(e):
            raise


if __name__ == "__main__":
    main()
```

**Step 2: Commit**

```bash
git add rl/scripts/eval_go2_locomotion.py
git commit -m "feat: add Go2 locomotion eval script"
```

---

### Task 3: Run Training

Train the Go2 locomotion policy. This produces the checkpoint needed by all subsequent tasks.

**Step 1: Run training**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python rl/scripts/train_go2_locomotion.py -n 4096 --max_iterations 2000 -d gpu`

Expected: Training runs, prints iteration logs, saves `model_*.pt` files to `rl/checkpoints/go2-locomotion/`

**Step 2: Verify checkpoint exists**

Run: `ls rl/checkpoints/go2-locomotion/model_*.pt`

Expected: One or more checkpoint files (e.g., `model_2000.pt`)

**Step 3: Verify with eval**

Run: `python rl/scripts/eval_go2_locomotion.py -d gpu`

Expected: Viewer opens, Go2 walks and follows velocity commands visually

**Step 4: Commit checkpoint reference**

Do NOT commit the model weights (large binary). Add to `.gitignore` if not already there:

```bash
echo "rl/checkpoints/go2-locomotion/*.pt" >> .gitignore
git add .gitignore
git commit -m "chore: ignore Go2 locomotion checkpoints"
```

---

### Task 4: Policy Loading in Bridge

Add `load_policy()` method to `GenesisBridgeServer` and modify `_step_forge_env()` to use it.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:376-405` (add policy fields to `__init__`)
- Modify: `genesis_bridge/bridge_server.py:572-615` (`_initialize_forge_env` — load policy after env init)
- Modify: `genesis_bridge/bridge_server.py:1107-1145` (`_step_forge_env` — use policy)

**Step 1: Add policy loading infrastructure to `__init__`**

In `GenesisBridgeServer.__init__` (around line 404), after `self.policy = None`, add:

```python
        self.policy_checkpoint_path = None  # Path to loaded checkpoint
```

**Step 2: Add `load_policy` method**

Add after `_initialize_forge_env` (after line 615):

```python
    def load_policy(self, checkpoint_dir: str) -> bool:
        """Load a trained locomotion policy from checkpoint directory.

        Args:
            checkpoint_dir: Path to directory containing model_*.pt and cfgs.pkl

        Returns:
            True if policy loaded successfully
        """
        try:
            import torch
            import glob
            import pickle
            from rsl_rl.runners import OnPolicyRunner
            from genesis_forge.wrappers import RslRlWrapper

            # Find latest checkpoint
            model_files = glob.glob(os.path.join(checkpoint_dir, "model_*.pt"))
            if not model_files:
                logger.error(f"No model files found in {checkpoint_dir}")
                return False
            model_files.sort()
            model_path = model_files[-1]

            # Load config
            cfg_path = os.path.join(checkpoint_dir, "cfgs.pkl")
            if not os.path.exists(cfg_path):
                logger.error(f"No cfgs.pkl found in {checkpoint_dir}")
                return False
            [cfg] = pickle.load(open(cfg_path, "rb"))

            # We need a dummy env for OnPolicyRunner to extract policy dims.
            # Use the forge_env wrapped for rsl_rl compatibility.
            if self.forge_env is None:
                logger.error("Cannot load policy: forge_env not initialized")
                return False

            wrapped_env = RslRlWrapper(self.forge_env)

            runner = OnPolicyRunner(wrapped_env, cfg, checkpoint_dir, device=gs.device)
            runner.load(model_path)
            self.policy = runner.get_inference_policy(device=gs.device)
            self.policy_checkpoint_path = model_path

            logger.info(f"Policy loaded from {model_path}")
            return True
        except Exception as e:
            import traceback
            logger.error(f"Failed to load policy: {e}\n{traceback.format_exc()}")
            self.policy = None
            self.policy_checkpoint_path = None
            return False
```

**Step 3: Auto-load policy in `_initialize_forge_env`**

At the end of `_initialize_forge_env` (before `return True` at line 609), add:

```python
            # Try to auto-load locomotion policy
            default_checkpoint = os.path.join(
                str(Path(__file__).parent.parent), "rl", "checkpoints", "go2-locomotion"
            )
            if os.path.exists(default_checkpoint):
                self.load_policy(default_checkpoint)
            else:
                logger.info(f"No locomotion policy found at {default_checkpoint}, using zero actions")
```

**Step 4: Modify `_step_forge_env` to use policy**

Replace lines 1107-1145 of `bridge_server.py`:

```python
    def _step_forge_env(self):
        """Step the genesis-forge environment."""
        try:
            import torch

            # Get actions from policy or use zeros
            if self.policy is not None:
                obs = self.forge_env.get_observations()
                with torch.no_grad():
                    actions = self.policy(obs)
            else:
                num_actions = self.forge_env.action_manager.num_actions
                actions = torch.zeros(
                    (self.forge_env.num_envs, num_actions),
                    device=gs.device,
                )

            obs, reward, terminated, truncated, extras = self.forge_env.step(actions)

            self.current_obs = obs
            self.step_count += 1
            self.fps_counter += 1

            # Track rewards
            if reward is not None:
                self.total_reward = float(reward[0])

            # Track episode resets
            if terminated is not None and truncated is not None:
                done = (terminated | truncated).any().item()
                if done:
                    self.episode_count += 1
                    self.episode_rewards.append(self.total_reward)
                    self.total_reward = 0.0

            # Update FPS every second
            now = time.time()
            if now - self.fps_time >= 1.0:
                self.fps = self.fps_counter
                self.fps_counter = 0
                self.fps_time = now

        except Exception as e:
            logger.error(f"Forge env step error: {e}")
```

**Step 5: Add Socket.io handler for policy loading**

After the `genesis_estop` handler (around line 1477), add:

```python
        @self.sio.on('genesis_load_policy')
        async def on_load_policy(data):
            checkpoint_dir = data.get('checkpoint_dir', '')
            if not checkpoint_dir:
                # Default path
                checkpoint_dir = str(Path(__file__).parent.parent / "rl" / "checkpoints" / "go2-locomotion")
            success = self.load_policy(checkpoint_dir)
            await self.sio.emit('genesis_training_metrics', self.get_training_metrics())
            logger.info(f"Policy load {'succeeded' if success else 'failed'}: {checkpoint_dir}")
```

**Step 6: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: add policy loading and inference in bridge _step_forge_env"
```

---

### Task 5: Command Source Switching

Add `command_source` gating so only the active input (gamepad or ROS) reaches `VelocityCommandManager`.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:376-405` (`__init__` — add command_source field)
- Modify: `genesis_bridge/bridge_server.py:1394-1410` (joystick/button handlers — gate by source)
- Modify: `server.js:30-54` (add `genesis_set_command_source` to forwarded events)

**Step 1: Add command_source state to `__init__`**

After `self.policy_checkpoint_path = None`, add:

```python
        self.command_source = "gamepad"  # "gamepad" or "ros"
        self.deadman_active = False  # L1 held state for gamepad mode
```

**Step 2: Gate gamepad input by command_source**

Modify the `on_joystick` handler (line 1394):

```python
        @self.sio.on('controller_joystick_state')
        async def on_joystick(data):
            if not hasattr(self, '_joystick_log_count'):
                self._joystick_log_count = 0
            if self._joystick_log_count < 5:
                logger.info(f"Received joystick state: {data}")
                self._joystick_log_count += 1

            if self.command_source != "gamepad":
                return

            if not self.deadman_active:
                # Zero velocity if deadman not held
                if self.forge_env is not None:
                    self.forge_env.set_velocity_from_gamepad({
                        'leftStickX': 0, 'leftStickY': 0,
                        'rightStickX': 0, 'rightStickY': 0,
                    })
                return

            if self.forge_env is not None:
                self.forge_env.set_velocity_from_gamepad(data)
            else:
                self.teleop_bridge.get_action(joystick_state=data)
```

**Step 3: Track deadman (L1) state from button events**

Modify the `on_buttons` handler (line 1408):

```python
        @self.sio.on('controller_button_states')
        async def on_buttons(data):
            # Track deadman switch (L1 button)
            was_active = self.deadman_active
            self.deadman_active = bool(data.get('L1', False))

            if was_active and not self.deadman_active:
                logger.info("Deadman released — zeroing velocity commands")
                if self.forge_env is not None and self.command_source == "gamepad":
                    self.forge_env.set_velocity_from_gamepad({
                        'leftStickX': 0, 'leftStickY': 0,
                        'rightStickX': 0, 'rightStickY': 0,
                    })

            self.teleop_bridge.process_button_states(data)
```

**Step 4: Add command source switching Socket.io handler**

After the `genesis_load_policy` handler, add:

```python
        @self.sio.on('genesis_set_command_source')
        async def on_set_command_source(data):
            new_source = data.get('source', 'gamepad')
            if new_source not in ('gamepad', 'ros'):
                logger.warning(f"Invalid command source: {new_source}")
                return
            old_source = self.command_source
            self.command_source = new_source
            # Zero velocity on source switch to prevent stale commands
            if self.forge_env is not None and old_source != new_source:
                self.forge_env.set_velocity_from_gamepad({
                    'leftStickX': 0, 'leftStickY': 0,
                    'rightStickX': 0, 'rightStickY': 0,
                })
            logger.info(f"Command source switched: {old_source} -> {new_source}")
            await self.sio.emit('genesis_training_metrics', self.get_training_metrics())
```

**Step 5: Forward the new event in server.js**

In `server.js`, add `'genesis_set_command_source'` to the `GENESIS_EVENTS` array (line 30-53):

```javascript
const GENESIS_EVENTS = [
  // ... existing events ...
  'genesis_set_command_source',
  'genesis_load_policy'
];
```

**Step 6: Commit**

```bash
git add genesis_bridge/bridge_server.py server.js
git commit -m "feat: add command source switching (gamepad/ros) with deadman gating"
```

---

### Task 6: ROS2 Integration in Bridge

Add rclpy node for `/cmd_vel` subscriber, `/cmd_vel` publisher (gamepad echo), `/set_command_source` service, and `/go2_debug` publisher.

**Files:**
- Create: `genesis_bridge/ros_bridge_node.py`
- Modify: `genesis_bridge/bridge_server.py` (integrate ROS node)

**Step 1: Create the ROS bridge node**

Create `genesis_bridge/ros_bridge_node.py`:

```python
"""
ROS2 node for the Genesis bridge server.

Provides:
- /cmd_vel subscriber: receives velocity commands from ROS
- /cmd_vel publisher: echoes gamepad commands to ROS
- /set_command_source service: switches between gamepad and ros input
- /go2_debug publisher: publishes debug status as JSON string

Runs inside Docker where ROS2 is available.
Optional: if rclpy is not available, the bridge runs without ROS.
"""

import json
import logging
import threading

logger = logging.getLogger("genesis_bridge.ros_node")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    logger.warning("rclpy not available — ROS integration disabled")


class GenesisBridgeROSNode:
    """Manages ROS2 communication for the Genesis bridge server."""

    def __init__(self, bridge_server):
        """
        Args:
            bridge_server: GenesisBridgeServer instance to read/write state from
        """
        self.bridge = bridge_server
        self.node = None
        self._spin_thread = None
        self._running = False
        self._cmd_vel_pub = None
        self._debug_pub = None

    def start(self):
        """Initialize ROS2 node and start spinning in a background thread."""
        if not ROS_AVAILABLE:
            logger.warning("Cannot start ROS node: rclpy not available")
            return False

        try:
            rclpy.init()
            self.node = rclpy.create_node("genesis_bridge")

            # Subscriber: /cmd_vel from external ROS nodes
            self.node.create_subscription(
                Twist, "/cmd_vel", self._cmd_vel_callback, 10
            )

            # Publisher: /cmd_vel for echoing gamepad commands
            self._cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

            # Publisher: /go2_debug status
            self._debug_pub = self.node.create_publisher(String, "/go2_debug", 10)

            # Service: /set_command_source
            self.node.create_service(
                SetBool, "/set_command_source", self._set_source_callback
            )

            # Timer for debug publishing at ~10Hz
            self.node.create_timer(0.1, self._publish_debug)

            # Spin in background thread
            self._running = True
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()

            logger.info("ROS2 node 'genesis_bridge' started")
            return True
        except Exception as e:
            logger.error(f"Failed to start ROS node: {e}")
            return False

    def stop(self):
        """Shutdown ROS2 node."""
        self._running = False
        if self.node is not None:
            self.node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def publish_cmd_vel(self, lin_x: float, lin_y: float, ang_z: float):
        """Publish gamepad velocity to /cmd_vel for external visibility."""
        if self._cmd_vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = lin_x
        msg.linear.y = lin_y
        msg.angular.z = ang_z
        self._cmd_vel_pub.publish(msg)

    def _cmd_vel_callback(self, msg: 'Twist'):
        """Handle incoming /cmd_vel from external ROS nodes."""
        if self.bridge.command_source != "ros":
            return

        if self.bridge.forge_env is None:
            return

        # Clamp to VelocityCommandManager ranges
        import torch
        import genesis as gs

        lin_x = max(-1.0, min(1.0, msg.linear.x))
        lin_y = max(-0.5, min(0.5, msg.linear.y))
        ang_z = max(-1.0, min(1.0, msg.angular.z))

        vcm = self.bridge.forge_env.velocity_command
        vcm.set_command("lin_vel_x", torch.tensor(lin_x, device=gs.device))
        vcm.set_command("lin_vel_y", torch.tensor(lin_y, device=gs.device))
        vcm.set_command("ang_vel_z", torch.tensor(ang_z, device=gs.device))

    def _set_source_callback(self, request, response):
        """ROS service: SetBool where True=ros, False=gamepad."""
        new_source = "ros" if request.data else "gamepad"
        old_source = self.bridge.command_source
        self.bridge.command_source = new_source

        # Zero velocity on switch
        if self.bridge.forge_env is not None and old_source != new_source:
            self.bridge.forge_env.set_velocity_from_gamepad({
                'leftStickX': 0, 'leftStickY': 0,
                'rightStickX': 0, 'rightStickY': 0,
            })

        response.success = True
        response.message = f"Command source: {old_source} -> {new_source}"
        logger.info(response.message)
        return response

    def _publish_debug(self):
        """Publish debug status to /go2_debug."""
        if self._debug_pub is None:
            return

        status = {
            "command_source": self.bridge.command_source,
            "deadman_active": self.bridge.deadman_active,
            "policy_loaded": self.bridge.policy is not None,
            "policy_checkpoint": self.bridge.policy_checkpoint_path,
            "paused": self.bridge.paused,
            "step_count": self.bridge.step_count,
            "fps": self.bridge.fps,
        }

        # Add velocity command if available
        if self.bridge.forge_env is not None:
            status["velocity_command"] = self.bridge.forge_env.get_velocity_command()

        msg = String()
        msg.data = json.dumps(status)
        self._debug_pub.publish(msg)

    def _spin(self):
        """Background thread for ROS spinning."""
        while self._running:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                break
```

**Step 2: Integrate ROS node into bridge server**

In `bridge_server.py`, add import at top (after other imports, around line 30):

```python
from genesis_bridge.ros_bridge_node import GenesisBridgeROSNode, ROS_AVAILABLE
```

In `__init__` (after `self.deadman_active = False`):

```python
        self.ros_node = GenesisBridgeROSNode(self)
```

In `_initialize_forge_env`, after loading the policy (before `return True`):

```python
            # Start ROS node if available
            if ROS_AVAILABLE:
                self.ros_node.start()
```

**Step 3: Publish gamepad to /cmd_vel**

In the `on_joystick` handler, after `self.forge_env.set_velocity_from_gamepad(data)`, add:

```python
                # Publish to ROS /cmd_vel for external visibility
                if ROS_AVAILABLE and self.ros_node.node is not None:
                    ranges = self.forge_env.velocity_command.range
                    lin_x = self.forge_env._map_stick(
                        data.get('leftStickY', 0), *ranges['lin_vel_x']
                    )
                    lin_y = self.forge_env._map_stick(
                        data.get('leftStickX', 0), *ranges['lin_vel_y']
                    )
                    ang_z = self.forge_env._map_stick(
                        data.get('rightStickX', 0), *ranges['ang_vel_z']
                    )
                    self.ros_node.publish_cmd_vel(lin_x, lin_y, ang_z)
```

**Step 4: Commit**

```bash
git add genesis_bridge/ros_bridge_node.py genesis_bridge/bridge_server.py
git commit -m "feat: add ROS2 /cmd_vel subscriber, publisher, source service, debug topic"
```

---

### Task 7: Anomaly Detection Wiring

Wire the existing `AnomalyDetector` into `_step_forge_env()`.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:1107-1145` (`_step_forge_env`)

**Step 1: Add anomaly checking after policy inference**

In `_step_forge_env`, after `actions = self.policy(obs)` and before `forge_env.step(actions)`, add:

```python
            # Anomaly detection on actions
            action_np = actions[0].cpu().numpy()
            anomalies = self.anomaly_detector.check_action(action_np, self.config.dt)
            if self.current_obs is not None:
                obs_np = self.current_obs[0].cpu().numpy() if hasattr(self.current_obs, 'cpu') else self.current_obs
                # Check velocity (dims 3-5 are linear velocity in obs)
                if hasattr(obs_np, '__getitem__') and len(obs_np) >= 6:
                    vel_anomalies = self.anomaly_detector.check_state(
                        velocity=obs_np[3:6]
                    )
                    anomalies.update(vel_anomalies)

            if self.anomaly_detector.update(anomalies):
                # Auto-stop triggered
                self.auto_stopped = True
                self.paused = True
                if self.forge_env is not None:
                    self.forge_env.set_velocity_from_gamepad({
                        'leftStickX': 0, 'leftStickY': 0,
                        'rightStickX': 0, 'rightStickY': 0,
                    })
                logger.warning("Anomaly auto-stop triggered in forge env")
                return
```

**Step 2: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: wire anomaly detector into forge env step loop"
```

---

### Task 8: Observability — Status Extension

Extend `get_training_metrics` and `_get_forge_metrics` with command source, policy, and deadman state.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:954-1016` (`get_training_metrics`, `_get_forge_metrics`)

**Step 1: Add new fields to `get_training_metrics`**

In `get_training_metrics` (line 974), add these fields to the returned dict:

```python
            "command_source": self.command_source,
            "deadman_active": self.deadman_active,
            "policy_loaded": self.policy is not None,
            "policy_checkpoint": os.path.basename(self.policy_checkpoint_path) if self.policy_checkpoint_path else None,
```

**Step 2: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: add command source, deadman, policy status to training metrics"
```

---

### Task 9: Frontend — Display New Status Fields

Update GenesisContext and GenesisControlPanel to show command source, deadman, and policy status.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx` (parse new metrics fields)
- Modify: `src/client/components/GenesisControlPanel.jsx` (display new fields + source toggle)

**Step 1: Add state fields to GenesisContext**

In `GenesisContext.jsx`, after `const [velocityCommand, setVelocityCommand] = useState(null);` (line 47), add:

```javascript
  const [commandSource, setCommandSource] = useState('gamepad');
  const [deadmanActive, setDeadmanActive] = useState(false);
  const [policyLoaded, setPolicyLoaded] = useState(false);
  const [policyCheckpoint, setPolicyCheckpoint] = useState(null);
```

**Step 2: Parse the new fields from training metrics**

Find where `genesis_training_metrics` is handled (search for `setTrainingMetrics` in the WebSocket message handler). Add after the existing metric parsing:

```javascript
        if (data.command_source !== undefined) setCommandSource(data.command_source);
        if (data.deadman_active !== undefined) setDeadmanActive(data.deadman_active);
        if (data.policy_loaded !== undefined) setPolicyLoaded(data.policy_loaded);
        if (data.policy_checkpoint !== undefined) setPolicyCheckpoint(data.policy_checkpoint);
```

**Step 3: Add action to switch command source**

Add a callback (near other actions like `setMode`):

```javascript
  const setCommandSource_ = useCallback((source) => {
    if (socket) {
      socket.emit('genesis_set_command_source', { source });
    }
  }, [socket]);
```

**Step 4: Expose in context value**

Add to the context provider value object:

```javascript
        commandSource,
        deadmanActive,
        policyLoaded,
        policyCheckpoint,
        setCommandSource: setCommandSource_,
```

**Step 5: Add display to GenesisControlPanel**

In `GenesisControlPanel.jsx`, destructure the new values:

```javascript
  const {
    // ... existing ...
    commandSource,
    deadmanActive,
    policyLoaded,
    policyCheckpoint,
    setCommandSource,
  } = useGenesis();
```

Add a new section after the mode selection section:

```jsx
        {/* Command Source */}
        <div className="control-section">
          <div className="control-label">Command Source</div>
          <select
            className="control-select"
            value={commandSource}
            onChange={(e) => setCommandSource(e.target.value)}
          >
            <option value="gamepad">Gamepad</option>
            <option value="ros">ROS /cmd_vel</option>
          </select>
        </div>

        {/* Deadman & Policy Status */}
        <div className="control-section">
          <div className="control-label">Safety</div>
          <div className={`status-indicator ${deadmanActive ? 'connected' : 'disconnected'}`}>
            <span className="status-dot"></span>
            <span>Deadman (L1): {deadmanActive ? 'Active' : 'Released'}</span>
          </div>
          <div className={`status-indicator ${policyLoaded ? 'connected' : 'disconnected'}`}>
            <span className="status-dot"></span>
            <span>Policy: {policyLoaded ? (policyCheckpoint || 'Loaded') : 'None'}</span>
          </div>
        </div>
```

**Step 6: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx src/client/components/GenesisControlPanel.jsx
git commit -m "feat: display command source, deadman, policy status in UI"
```

---

### Task 10: Integration Testing

Verify the full pipeline works per the test matrix.

**Step 1: Test no-policy fallback**

Start bridge without a checkpoint (move/rename `rl/checkpoints/go2-locomotion/` temporarily). Verify:
- Bridge starts without error
- Zero-action behavior (robot stands)
- No crashes

**Step 2: Test policy loading**

Restore the checkpoint directory. Start bridge. Verify:
- Policy auto-loads on startup (check logs for "Policy loaded from...")
- `_step_forge_env` calls `policy(obs)` instead of zeros

**Step 3: Test gamepad → Go2 moves**

Connect browser, hold L1, push left stick forward. Verify Go2 walks forward in the sim viewer.

**Step 4: Test deadman**

Release L1 while pushing stick. Verify velocity commands zero and Go2 returns to standing.

**Step 5: Test command source switching**

Switch source to "ros" via UI dropdown. Verify gamepad input is ignored.

**Step 6: Test ROS /cmd_vel (if ROS Docker available)**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
```

Verify Go2 walks forward.

**Step 7: Test gamepad → /cmd_vel echo**

Switch source back to "gamepad", hold L1, push sticks:

```bash
ros2 topic echo /cmd_vel
```

Verify matching values appear.

**Step 8: Test E-STOP**

Trigger E-STOP from UI. Verify sim pauses and velocity zeroes regardless of command source.

**Step 9: Commit any test fixes**

```bash
git add -A
git commit -m "fix: integration test fixes for go2 velocity control pipeline"
```
