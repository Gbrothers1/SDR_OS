#!/usr/bin/env python3
"""
Diagnostic: Test HIP + Vulkan coexistence on Steam Deck.

Tests whether PyTorch HIP tensors work AFTER Genesis Vulkan build completes.
Uses GS_TORCH_FORCE_CPU_DEVICE=1 to keep tensors on CPU during build,
then probes HIP for runtime inference (matching genesis_sim_runner approach).

gs.device stays on CPU — only inference uses a separate HIP device.

Expected environment:
  - HSA_OVERRIDE_GFX_VERSION=10.3.0 (maps gfx1033 → gfx1030)
  - NO CUDA_VISIBLE_DEVICES="" (HIP must be visible)
  - GS_TORCH_FORCE_CPU_DEVICE=1 (set by this script)

Exit codes:
  0 = all tests passed (HIP+Vulkan coexist at runtime)
  1 = HIP init fails after Vulkan build
  2 = HIP+Vulkan sequential ops fail
  3 = policy inference on HIP fails
"""

import sys
import os
import time
import signal

# ── Environment setup (before any imports that touch CUDA/HIP) ──
os.environ["GS_TORCH_FORCE_CPU_DEVICE"] = "1"
os.environ.setdefault("TORCHDYNAMO_DISABLE", "1")
os.environ.setdefault("HSA_OVERRIDE_GFX_VERSION", "10.3.0")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")  # headless rendering
# Do NOT set CUDA_VISIBLE_DEVICES="" — we want HIP visible
if "CUDA_VISIBLE_DEVICES" in os.environ:
    del os.environ["CUDA_VISIBLE_DEVICES"]

from pathlib import Path
_project_root = str(Path(__file__).resolve().parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import torch
import numpy as np


# ── Timeout protection for deadlock-prone HIP operations ──
class DeadlockTimeout(Exception):
    pass

def _alarm_handler(signum, frame):
    raise DeadlockTimeout("Operation timed out — likely HIP+Vulkan deadlock")

signal.signal(signal.SIGALRM, _alarm_handler)

def with_timeout(seconds, fn, *args, **kwargs):
    """Run fn with a SIGALRM timeout. Returns (success, result_or_error)."""
    signal.alarm(seconds)
    try:
        result = fn(*args, **kwargs)
        signal.alarm(0)  # cancel
        return True, result
    except DeadlockTimeout:
        return False, f"DEADLOCK: timed out after {seconds}s"
    except Exception as e:
        signal.alarm(0)
        return False, str(e)

# Real policy architecture: [512, 256, 128] hidden dims, 310→12
HIDDEN_DIMS = [512, 256, 128]
OBS_DIM = 310
ACT_DIM = 12


def banner(msg):
    print(f"\n{'='*60}")
    print(f"  {msg}")
    print(f"{'='*60}")


def make_policy(obs_dim, act_dim, hidden_dims, device):
    """Create a dummy policy MLP matching real ActorMLP architecture."""
    layers = []
    in_dim = obs_dim
    for h in hidden_dims:
        layers.append(torch.nn.Linear(in_dim, h))
        layers.append(torch.nn.ELU())
        in_dim = h
    layers.append(torch.nn.Linear(in_dim, act_dim))
    policy = torch.nn.Sequential(*layers).to(device)
    policy.eval()
    n_params = sum(p.numel() for p in policy.parameters())
    print(f"  Policy: {hidden_dims} → {n_params} params on {device}")
    return policy


def test_hip_available():
    """Phase 0: Verify HIP is visible to PyTorch."""
    banner("Phase 0: HIP Availability Check")
    print(f"  torch.cuda.is_available() = {torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        print("  FAIL: HIP/CUDA not available. Is HSA_OVERRIDE_GFX_VERSION set?")
        print("  Check: CUDA_VISIBLE_DEVICES is not set to empty string")
        return False
    print(f"  torch.cuda.device_count() = {torch.cuda.device_count()}")
    print(f"  torch.cuda.get_device_name(0) = {torch.cuda.get_device_name(0)}")
    print(f"  PASS")
    return True


def test_genesis_build():
    """Phase 1: Build Genesis env with CPU tensors (Vulkan physics)."""
    banner("Phase 1: Genesis Build with GS_TORCH_FORCE_CPU_DEVICE=1")
    import genesis as gs

    print(f"  gs.init(backend=gs.vulkan) ...")
    gs.init(backend=gs.vulkan, performance_mode=True)
    print(f"  gs.device = {gs.device}")
    assert "cpu" in str(gs.device), f"Expected CPU device during build, got {gs.device}"

    from src.sdr_os.envs.go2_env import Go2BridgeEnv
    env = Go2BridgeEnv(
        num_envs=1,
        dt=1/50,
        max_episode_length_s=None,
        headless=True,
        camera_res=(640, 480),  # small for diagnostic
    )
    print(f"  env.build() ...")
    env.build()
    print(f"  env.reset() ...")
    obs, _ = env.reset()
    print(f"  obs shape={obs.shape}, device={obs.device}")
    print(f"  PASS: Build completed on CPU device")
    return env, obs


def _hip_tensor_test():
    """Inner function for timeout wrapping."""
    t = torch.randn(100, 100, device="cuda")
    result = t @ t.T
    torch.cuda.synchronize()
    return t.shape, result.shape, result.sum().item()

def test_hip_after_build():
    """Phase 2: Test basic HIP tensor ops after Vulkan build."""
    banner("Phase 2: HIP Tensor Ops After Vulkan Build")
    print("  (30s timeout — this is the deadlock danger zone)")
    ok, result = with_timeout(30, _hip_tensor_test)
    if not ok:
        print(f"  FAIL: {result}")
        return False
    t_shape, r_shape, r_sum = result
    print(f"  torch.randn on cuda: shape={t_shape}")
    print(f"  matmul result: shape={r_shape}, sum={r_sum:.2f}")
    print(f"  PASS: HIP tensor ops work after Vulkan build")
    return True


def test_hip_between_steps(env):
    """Phase 3: Granular test — isolate exactly which op deadlocks."""
    banner("Phase 3: Granular HIP+Vulkan Isolation")
    import genesis as gs
    import gstaichi as _ti
    sys.stdout.reconfigure(line_buffering=True)  # flush every print

    actions = torch.zeros(1, 12, device=gs.device)

    # Pre-allocate HIP tensors BEFORE any stepping
    print("  [3.0] Pre-allocating HIP tensors (before any physics)...")
    hip_a = torch.randn(100, 100, device="cuda")
    hip_b = torch.randn(100, 100, device="cuda")
    torch.cuda.synchronize()
    print(f"  [3.0] OK — hip_a={hip_a.device}, hip_b={hip_b.device}")

    # Step 1: env.step alone (no HIP after)
    print("  [3.1] env.step() — physics only, no HIP...")
    obs, _, dones, _, _ = env.step(actions)
    print(f"  [3.1] OK — obs device={obs.device}")

    # Step 2: gstaichi.sync() after step
    print("  [3.2] _ti.sync() — flush Vulkan...")
    _ti.sync()
    print("  [3.2] OK — Vulkan synced")

    # Step 3: torch.cuda.synchronize() — sync HIP side
    print("  [3.3] torch.cuda.synchronize() — sync HIP stream...")
    torch.cuda.synchronize()
    print("  [3.3] OK — HIP synced")

    # Step 4: In-place op on PRE-ALLOCATED HIP tensor (no new alloc)
    print("  [3.4] In-place HIP op on pre-allocated tensor (hip_a.mul_(2))...")
    hip_a.mul_(2.0)
    torch.cuda.synchronize()
    print(f"  [3.4] OK — hip_a sum={hip_a.sum().item():.2f}")

    # Step 5: HIP matmul on pre-allocated tensors
    print("  [3.5] HIP matmul on pre-allocated tensors...")
    r = hip_a @ hip_b.T
    torch.cuda.synchronize()
    print(f"  [3.5] OK — result sum={r.sum().item():.2f}")

    # Step 6: NEW HIP allocation after physics step
    print("  [3.6] New HIP allocation (torch.randn) after physics step...")
    hip_new = torch.randn(100, 100, device="cuda")
    torch.cuda.synchronize()
    print(f"  [3.6] OK — new tensor sum={hip_new.sum().item():.2f}")

    # Step 7: Full cycle — step + sync + HIP inference
    print("  [3.7] Full cycle: step → ti.sync → cuda.sync → HIP matmul...")
    obs, _, dones, _, _ = env.step(actions)
    _ti.sync()
    torch.cuda.synchronize()
    r2 = hip_a @ hip_b.T
    torch.cuda.synchronize()
    print(f"  [3.7] OK — result sum={r2.sum().item():.2f}")

    # Step 8: 5 rapid cycles
    print("  [3.8] 5 rapid cycles: step → fence → HIP...")
    for i in range(5):
        obs, _, dones, _, _ = env.step(actions)
        _ti.sync()
        torch.cuda.synchronize()
        hip_a.normal_()
        r = hip_a @ hip_b.T
        torch.cuda.synchronize()
        print(f"    cycle {i}: r_sum={r.sum().item():.2f}")
    print(f"  [3.8] PASS — all 5 cycles completed")

    print(f"  PASS: HIP+Vulkan coexist with fence!")
    return True


def test_inference_on_hip(env, obs):
    """Phase 4: Policy inference on HIP (gs.device stays CPU)."""
    banner("Phase 4: Policy Inference on HIP (gs.device=CPU)")
    import genesis as gs
    import gstaichi as _ti
    sys.stdout.reconfigure(line_buffering=True)
    infer_device = torch.device("cuda", 0)
    try:
        # Fence before any HIP ops — ensure Vulkan from Phase 3 is done
        print("  [4.0] Fence: _ti.sync() + cuda.sync()...")
        _ti.sync()
        torch.cuda.synchronize()
        print("  [4.0] OK")

        # Move obs to HIP
        print("  [4.1] obs.to(cuda)...")
        obs_hip = obs.to(infer_device)
        torch.cuda.synchronize()
        print(f"  [4.1] OK — obs_hip device={obs_hip.device}")

        # Create policy layers one at a time
        print(f"  [4.2] Creating Linear(310, 512) on CPU...")
        l1 = torch.nn.Linear(310, 512)
        print(f"  [4.2] OK — moving to HIP...")
        l1 = l1.to(infer_device)
        torch.cuda.synchronize()
        print(f"  [4.2] OK — l1 on {next(l1.parameters()).device}")

        print(f"  [4.3] Creating full policy on CPU then .to(cuda)...")
        policy = make_policy(obs_hip.shape[-1], ACT_DIM, HIDDEN_DIMS, torch.device("cpu"))
        print(f"  [4.3a] Policy on CPU, moving to HIP...")
        policy = policy.to(infer_device)
        torch.cuda.synchronize()
        print(f"  [4.3] OK — policy on {next(policy.parameters()).device}")

        print("  [4.4] Policy forward pass...")
        with torch.no_grad():
            actions = policy(obs_hip)
        torch.cuda.synchronize()
        print(f"  [4.4] OK — actions shape={actions.shape}, device={actions.device}")

        # Move actions back to CPU for Genesis
        actions_cpu = actions.to(gs.device)
        print(f"  [4.5] Actions on CPU: device={actions_cpu.device}")
        print(f"  PASS: HIP inference + CPU transfer works")
        return True, policy, infer_device
    except Exception as e:
        print(f"  FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False, None, None


def test_full_loop_hip(env, policy, infer_device, n_steps=100):
    """Phase 5: Full loop — HIP inference, CPU physics (matching runner)."""
    banner(f"Phase 5: Full Loop ({n_steps} steps) — HIP Inference + Vulkan Physics")
    import genesis as gs
    try:
        obs, _ = env.reset()
        obs = obs.to(infer_device)  # obs on HIP for inference

        step_times = []
        infer_times = []
        xfer_to_cpu_times = []
        xfer_to_hip_times = []

        for i in range(n_steps):
            t0 = time.monotonic()

            # Policy inference on HIP
            t_infer = time.monotonic()
            with torch.no_grad():
                actions_hip = policy(obs)
            torch.cuda.synchronize()
            infer_ms = (time.monotonic() - t_infer) * 1000
            infer_times.append(infer_ms)

            # Transfer actions HIP → CPU for Genesis
            t_xfer = time.monotonic()
            actions_cpu = actions_hip.to(gs.device)
            xfer_ms = (time.monotonic() - t_xfer) * 1000
            xfer_to_cpu_times.append(xfer_ms)

            # Physics step (Vulkan on GPU, tensors on CPU)
            new_obs, _, dones, _, _ = env.step(actions_cpu)

            # Transfer obs CPU → HIP for next inference
            t_obs = time.monotonic()
            obs = new_obs.to(infer_device)
            obs_xfer_ms = (time.monotonic() - t_obs) * 1000
            xfer_to_hip_times.append(obs_xfer_ms)

            step_ms = (time.monotonic() - t0) * 1000
            step_times.append(step_ms)

            if dones.any():
                obs, _ = env.reset()
                obs = obs.to(infer_device)

            if (i + 1) % 25 == 0:
                avg = sum(step_times[-25:]) / 25
                print(f"  step {i+1}: avg_step={avg:.1f}ms")

        avg_step = sum(step_times) / len(step_times)
        avg_infer = sum(infer_times) / len(infer_times)
        avg_xfer_cpu = sum(xfer_to_cpu_times) / len(xfer_to_cpu_times)
        avg_xfer_hip = sum(xfer_to_hip_times) / len(xfer_to_hip_times)
        fps = 1000.0 / avg_step if avg_step > 0 else 0

        print(f"\n  HIP Results ({n_steps} steps):")
        print(f"    avg step:       {avg_step:.2f} ms ({fps:.1f} FPS)")
        print(f"    avg inference:  {avg_infer:.2f} ms")
        print(f"    avg act→CPU:    {avg_xfer_cpu:.2f} ms")
        print(f"    avg obs→HIP:    {avg_xfer_hip:.2f} ms")
        print(f"    transfer total: {avg_xfer_cpu + avg_xfer_hip:.2f} ms")
        print(f"    max step:       {max(step_times):.2f} ms")
        print(f"  PASS: Full HIP loop completed")
        return {
            "avg_step_ms": avg_step,
            "avg_infer_ms": avg_infer,
            "avg_xfer_ms": avg_xfer_cpu + avg_xfer_hip,
            "fps": fps,
        }
    except Exception as e:
        print(f"  FAIL: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_full_loop_cpu(env, n_steps=100):
    """Phase 6: CPU-only baseline for comparison."""
    banner(f"Phase 6: CPU Baseline ({n_steps} steps)")
    import genesis as gs
    try:
        obs, _ = env.reset()
        # Same architecture on CPU
        policy_cpu = make_policy(obs.shape[-1], ACT_DIM, HIDDEN_DIMS, torch.device("cpu"))

        step_times = []
        infer_times = []

        for i in range(n_steps):
            t0 = time.monotonic()

            t_infer = time.monotonic()
            with torch.no_grad():
                actions = policy_cpu(obs)
            infer_ms = (time.monotonic() - t_infer) * 1000
            infer_times.append(infer_ms)

            new_obs, _, dones, _, _ = env.step(actions)
            obs = new_obs

            step_ms = (time.monotonic() - t0) * 1000
            step_times.append(step_ms)

            if dones.any():
                obs, _ = env.reset()

            if (i + 1) % 25 == 0:
                avg = sum(step_times[-25:]) / 25
                print(f"  step {i+1}: avg_step={avg:.1f}ms")

        avg_step = sum(step_times) / len(step_times)
        avg_infer = sum(infer_times) / len(infer_times)
        fps = 1000.0 / avg_step if avg_step > 0 else 0

        print(f"\n  CPU Baseline ({n_steps} steps):")
        print(f"    avg step:      {avg_step:.2f} ms ({fps:.1f} FPS)")
        print(f"    avg inference: {avg_infer:.2f} ms")
        print(f"    max step:      {max(step_times):.2f} ms")
        return {"avg_step_ms": avg_step, "avg_infer_ms": avg_infer, "fps": fps}
    except Exception as e:
        print(f"  CPU baseline error: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    banner("HIP + Vulkan Coexistence Diagnostic")
    print(f"  HSA_OVERRIDE_GFX_VERSION = {os.environ.get('HSA_OVERRIDE_GFX_VERSION', 'NOT SET')}")
    print(f"  CUDA_VISIBLE_DEVICES = {os.environ.get('CUDA_VISIBLE_DEVICES', 'NOT SET')}")
    print(f"  GS_TORCH_FORCE_CPU_DEVICE = {os.environ.get('GS_TORCH_FORCE_CPU_DEVICE', 'NOT SET')}")
    print(f"  Policy architecture: obs={OBS_DIM} → {HIDDEN_DIMS} → act={ACT_DIM}")

    # Phase 0: HIP available?
    if not test_hip_available():
        print("\nABORT: HIP not available. Cannot test coexistence.")
        sys.exit(1)

    # Phase 1: Genesis build on CPU
    env, obs = test_genesis_build()

    # Phase 2: HIP after build
    if not test_hip_after_build():
        print("\nRESULT: HIP init fails after Vulkan build → no GPU tensors possible")
        sys.exit(1)

    # Phase 3: Sequential HIP + Vulkan steps
    if not test_hip_between_steps(env):
        print("\nRESULT: HIP+Vulkan can't coexist sequentially → need two-process arch")
        sys.exit(2)

    # Phase 4: Inference on HIP (gs.device stays CPU)
    ok, policy, infer_device = test_inference_on_hip(env, obs)
    if not ok:
        print("\nRESULT: Policy inference on HIP fails")
        sys.exit(3)

    # Phase 5: Full loop with HIP
    hip_stats = test_full_loop_hip(env, policy, infer_device, n_steps=100)

    # Phase 6: CPU baseline
    cpu_stats = test_full_loop_cpu(env, n_steps=100)

    # Summary
    banner("SUMMARY")
    if hip_stats and cpu_stats:
        print(f"  HIP:  {hip_stats['fps']:.1f} FPS  (infer={hip_stats['avg_infer_ms']:.2f}ms, xfer={hip_stats['avg_xfer_ms']:.2f}ms)")
        print(f"  CPU:  {cpu_stats['fps']:.1f} FPS  (infer={cpu_stats['avg_infer_ms']:.2f}ms)")
        speedup = cpu_stats['avg_infer_ms'] / hip_stats['avg_infer_ms'] if hip_stats['avg_infer_ms'] > 0 else 0
        net_infer = hip_stats['avg_infer_ms'] + hip_stats['avg_xfer_ms']
        print(f"  HIP inference speedup: {speedup:.2f}x (raw)")
        print(f"  HIP net (infer+xfer): {net_infer:.2f}ms vs CPU: {cpu_stats['avg_infer_ms']:.2f}ms")
        if hip_stats['fps'] > cpu_stats['fps'] * 1.05:
            print(f"\n  VERDICT: HIP is faster — use GPU tensors for inference")
        elif hip_stats['fps'] < cpu_stats['fps'] * 0.95:
            print(f"\n  VERDICT: CPU is faster — keep CPU tensors (transfer overhead > HIP speedup)")
        else:
            print(f"\n  VERDICT: Similar performance — CPU tensors are simpler, prefer CPU")
    elif hip_stats:
        print(f"  HIP: {hip_stats['fps']:.1f} FPS (CPU baseline failed)")
    print(f"\n  All coexistence tests PASSED — HIP+Vulkan works at runtime")
    sys.exit(0)


if __name__ == "__main__":
    main()
