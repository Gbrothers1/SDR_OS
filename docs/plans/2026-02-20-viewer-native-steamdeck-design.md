# Viewer-Native Mode for Steam Deck

**Date:** 2026-02-20
**Status:** Design approved

## Problem

The genesis_sim_runner.py main loop is capped at ~30fps on the Steam Deck APU.
The Genesis viewer example with 20 parallel envs on CPU achieves 60fps on the same hardware.

### Root Cause

`camera.render()` calls `glReadPixels` every frame — a synchronous GPU pipeline drain
that stalls the CPU for 4-15ms while it waits for the GPU to finish all pending draw
commands and DMA-transfers the full framebuffer to system memory.

The Genesis viewer **never calls glReadPixels**. It renders to the back buffer and
calls `flip()` (OS buffer swap). Zero CPU involvement in pixel data. Physics runs
unblocked.

Additionally, the GPU YUV fast path in `render_and_enqueue()` is dead code —
`camera.render()` returns numpy (from glReadPixels), not a torch tensor, so the
`frame_tensor` branch never executes.

### Per-Frame Cost Comparison

| Operation | Genesis Viewer | Our Runner |
|-----------|---------------|------------|
| Physics step | Same | Same |
| GL scene graph update | Same | Same |
| OpenGL rasterize | GPU back-buffer, stays on GPU | GPU offscreen FBO, stays on GPU |
| Pixel readback | **None** — `flip()` to screen | **glReadPixels** — 4-15ms stall |
| Encoding | None | JPEG/H.264 — background thread |
| Total extra cost | 0 | +4-15ms/frame |

## Solution: Viewer-Native Mode with Virtual Display Capture

Two decoupled processes:

1. **genesis_sim_runner --viewer** — physics + Genesis viewer on virtual display, no SHM/encoding
2. **Capture sidecar** — ffmpeg x11grab captures the virtual display and writes to SHM

```
Xvfb :2 -screen 0 640x360x24      <-- hidden virtual framebuffer
         |
genesis_sim_runner --viewer         <-- renders to DISPLAY=:2 at 60fps
         |
ffmpeg -f x11grab -i :2            <-- captures from virtual display
         |
SHM --> transport-server --> WS     <-- existing pipeline, unchanged
         |
Browser (SimViewer panel)           <-- user sees ONLY this
```

The Genesis viewer window is invisible to the user — it exists only on the virtual
X display. The browser on the real display is the sole interface.

## File Changes

### 1. `scripts/genesis_sim_runner.py`

- Add `--viewer` CLI flag (default: off)
- Add `headless` parameter to constructor, passed through to Go2BridgeEnv
- When `--viewer`:
  - Pass `headless=False` to Go2BridgeEnv
  - Skip `init_shm()` and `_start_encoder_thread()`
  - Skip `render_and_enqueue()` in the main loop
  - Main loop: `_enforce_cmd_ttl()` -> `step_sim()` -> NATS telemetry -> `asyncio.sleep(0)`
- NATS command/telemetry pipeline remains fully functional

### 2. `src/sdr_os/envs/go2_env.py`

- Change `max_FPS=int(0.5 / self.dt)` (=25) to `max_FPS=60`
- The viewer was artificially capped at 25fps by this setting

### 3. `scripts/viewer_capture.py` (new)

Capture sidecar that:
1. Launches `ffmpeg -f x11grab -video_size WxH -framerate N -i :D -c:v mjpeg -q:v Q -f image2pipe pipe:1`
2. Reads JPEG frames from ffmpeg stdout (SOI/EOI marker parsing)
3. Writes each frame to `ShmRingWriter` at the standard SHM path
4. Transport-server picks them up unchanged

CLI args: `--display :2`, `--fps 30`, `--res 640x360`, `--quality 80`

### 4. `scripts/start_viewer_mode.sh` (new)

Convenience launcher:
1. Start `Xvfb :2 -screen 0 WxHx24` if not already running
2. Launch `genesis_sim_runner.py --viewer` with `DISPLAY=:2`
3. Wait for viewer window to appear
4. Launch `viewer_capture.py --display :2`
5. Trap SIGINT/SIGTERM to clean up all child processes

## What Stays the Same

- Transport server — reads SHM, fans out via WebSocket (unchanged)
- Browser/SimViewer — receives WebSocket frames as before (unchanged)
- WebSocket binary protocol — same type bytes, same headers
- NATS command pipeline — gamepad, settings, policy loading (unchanged)
- Safety stack — all 3 layers functional (unchanged)
- Web UI controls, telemetry, settings panels (unchanged)
- Headless stream mode — still works when `--viewer` is not passed

## Expected Results

- Physics runs at natural 50Hz rate, unblocked by rendering
- Genesis viewer renders at up to 60fps on virtual display
- ffmpeg captures at 30fps and feeds to existing stream pipeline
- Web UI shows the sim feed in the browser, same as before
- Total latency: ~1 frame of capture delay (~33ms at 30fps)

## Testing Plan

1. Start Xvfb, sim runner with --viewer, and capture sidecar
2. Verify viewer window appears on :2 (xdpyinfo, xdotool)
3. Verify JPEG frames appear in SHM (transport-server logs)
4. Verify browser SimViewer shows the feed
5. Verify gamepad commands work via NATS
6. Verify safety stack HOLD/ESTOP triggers correctly
7. Compare step_sim rate: headless mode vs viewer mode
