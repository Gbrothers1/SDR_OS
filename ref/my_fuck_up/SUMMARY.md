# SDR_OS Backup - Feb 5, 2026

## What Happened
A git merge (commit `fe3c7c507b618b81f16de3448f4d47057e640f53`) caused confusion about which files were tracked vs untracked. Many critical implementation files existed locally but were never committed to git, meaning they could be lost during branch operations.

## Critical Files That Were Untracked
These files exist locally but need to be tracked in git:

### 1. rl/actions/ (CRITICAL - imported by bridge_server.py)
- `action_spec.py` - ActionSpec class for defining action segments
- `router.py` - ActionRouter for alpha blending and safety
- `teleop_bridge.py` - TeleopBridge for normalizing gamepad input
- `joint_controller.py` - Joint-level control
- `cartesian_controller.py` - Cartesian control
- `__init__.py` - Package exports

### 2. configs/
- `configs/robots/*.yaml` - Robot definitions (go2, franka, anymal, etc.)
- `configs/profiles/*.yaml` - Simulation/real profiles

### 3. genesis_bridge/envs/
- `__init__.py` - Package init (required for imports)
- `go2_env.py` - Go2BridgeEnv class

### 4. rl/envs/ (partial)
- `genesis_vecenv.py` - rsl_rl-compatible VecEnv wrapper
- `go2_gamepad_env.py` - Gamepad training environment
- `go2_skill_env.py` - Skill training environment
- `__init__.py`

### 5. rl/scripts/ (partial)
- Training scripts: train_go2_freeze/sit/stand/gamepad_locomotion.py
- Eval scripts: eval_go2_freeze/sit/stand/gamepad_locomotion.py
- `teleop_record.py`, `eval_policy.py`

## Issues Found

### 1. Browser Cache
The browser was caching old bundle.js, showing old UI. Solution: hard refresh (Ctrl+Shift+R) or clear cache.

### 2. Broken .venv
The virtual environment had broken symlinks pointing to `/home/h1ght0w3r/...` which doesn't exist. Needs recreation.

### 3. Missing aiortc
WebRTC disabled because aiortc package not installed. Need to install after fixing venv.

### 4. No Logo Image
The "SDR_OS logo" was never actually created as an image file - it's just text in ViewerLayer.jsx.

## Restoration Steps

1. **Track untracked files:**
   ```bash
   git add rl/actions/ rl/envs/ rl/scripts/ configs/robots/ configs/profiles/ genesis_bridge/envs/__init__.py
   git commit -m "feat: track previously untracked critical modules"
   ```

2. **Fix venv:**
   ```bash
   rm -rf .venv
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   pip install aiortc  # for WebRTC
   ```

3. **Clear browser cache:**
   - Hard refresh: Ctrl+Shift+R
   - Or open in incognito window

4. **Rebuild frontend:**
   ```bash
   npm run build
   ```

## Files in This Backup

- `plans/` - All implementation plan documents
- `code/` - All code files (155 files)
- `conversations/` - Summaries of recent Claude sessions
- `SUMMARY.md` - This file

## Lesson Learned
Always commit new files promptly. Untracked files are invisible to git and can be lost during branch operations.
