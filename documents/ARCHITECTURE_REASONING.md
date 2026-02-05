# Architecture Reasoning & Backend Tradeoffs

Date: 2026-02-05

## Goals/Constraints
- Sub-50 ms input→actuation latency where possible; steady 60 fps visualization target.
- Reuse existing ROS2/Python control stack; minimal rewrites of working UI.
- Enable Genesis-in-browser experience while keeping server-side GPU path for fidelity and compute-heavy training.

## Backend Options

### Option A: Hybrid (Python ROS2 + Node Gateway) **[Recommended]**
- **Why**: Aligns with current code (ROS nodes in Python, gateway/UI already Node). Minimizes refactor risk while enabling fast websocket/socket.io delivery and easy WebRTC signaling with existing JS libs.
- **Perf**: Python handles ROS2 callbacks; heavy lifting (sim + encoding) stays in C++/CUDA. Node gateway mostly I/O bound; negligible CPU cost.
- **Work**: Keep `server.js` (harden) or replace with Nest/fastify; add signaling + health; keep ROSBridge or implement direct bridge adapter.

### Option B: Python-only (FastAPI/Starlette + rclpy + aiortc)
- **Why**: Single language simplifies deployments and shared models; aiortc gives WebRTC; uvicorn handles WS.
- **Perf**: Python WS stack is fine for tens of clients; but high-throughput signaling + media best left to native SFU. WebRTC in Python can be heavier than Node libs unless offloaded.
- **Work**: Rewrite socket.io semantics to native WS; add WebRTC signaling; migrate client expectations.

### Option C: Node-dominant (ts-node/Nest) with ROS in sidecar
- **Why**: Co-locate gateway + thin ROSBridge adapter in TS; use rclnodejs for ROS2.
- **Perf**: JS GC in ROS loop could add jitter; rclnodejs maturity vs rclpy? Risky for real-time teleop.
- **Work**: Port ROS control logic; less ML ecosystem access.

## CUDA / NVENC Path
- Keep Genesis and rsl_rl on GPU server. Render offscreen (EGL) → pipe RGBA to encoder.
- Use NVENC H.264/AV1; target 1080p60 ~6–12 Mbps. 2080 Ti supports up to 2 simultaneous 4K60 sessions (practical check needed).
- Transport via WebRTC SFU (mediasoup/ion-sfu) for low-latency; HLS fallback for debugging.
- If Genesis-in-browser (WASM/WebGPU) active, skip NVENC and stream only telemetry; optionally send sparse keyframes/state snapshots for recovery.

## Genesis in Browser
- Primary path: GPU-hosted Genesis (full fidelity). Browser receives video + telemetry.
- Experimental path: WASM/WebGPU build (if available) runs physics client-side; backend supplies initial scene/assets + occasional authority sync; actions computed locally from inputs/policy.
- Risks: Browser thread budget, lack of native CUDA, potential divergence from server physics; fall back to server mode automatically when unsupported.

## Decision
- Proceed with **Hybrid (Option A)** for MVP while keeping architecture toggles for Option B.
- Implement media stack with NVENC + WebRTC; keep browser-sim flag for future WASM/WebGPU builds.

## Next Steps (in order)
1) Finalize schemas for control (`/controller/*`), telemetry, and media signaling; document in `documents/ARCHITECTURE_OVERVIEW.md`.
2) Choose gateway implementation (Node baseline, evaluate FastAPI parity); add health + metrics.
3) Add media-encoder service and WebRTC signaling; integrate `CameraViewer` to consume WebRTC.
4) Prototype Genesis headless render → NVENC pipeline; measure latency/bandwidth.
5) Investigate Genesis WASM/WebGPU feasibility; gate behind `SIM_MODE=browser`.
