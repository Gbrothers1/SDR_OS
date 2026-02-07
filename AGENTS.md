# Repository Guidelines

## Project Structure & Module Organization
- `src/client/`: React frontend (JSX, contexts, UI components).
- `src/sdr_os/`: Python backend (shared memory IPC, ring buffer, core logic).
- `services/transport-server/`: Rust service for SHM→WS fanout and NATS relay.
- `configs/`: Caddy, NATS, Prometheus configuration.
- `containers/`: CUDA/ROCm/MLX/ROS2 Dockerfiles.
- `tests/`: unit, integration, and benchmarks.
- `docs/` + `mkdocs.yml`: documentation site.

## Build, Test, and Development Commands
- `just verify`: CPU-only verification (fast, no Docker builds).
- `just verify-gpu` / `just verify-all`: include Docker builds and CUDA checks.
- `just test`: Python unit tests (`PYTHONPATH=src pytest tests/unit/ -v`).
- `just test-cov`: unit tests with coverage report.
- `cargo test --manifest-path services/transport-server/Cargo.toml`: Rust transport tests.
- `npm run build`: production bundle via Webpack.
- `npm run dev`: watch mode for frontend builds.
- `docker compose --profile dev up`: run ROS2 + web dev stack.

## Coding Style & Naming Conventions
- JavaScript/JSX uses 2-space indentation (see `src/client/`).
- Python uses 4-space indentation and type annotations where already present (see `src/sdr_os/`).
- Rust follows standard `rustfmt` conventions.
- No repo-wide formatter config is enforced; match the surrounding file style.

## Testing Guidelines
- Python tests live under `tests/unit/` and are run with `pytest`.
- Rust tests are co-located in `services/transport-server/` and run via `cargo test`.
- Name tests descriptively and keep new coverage focused on changed behavior.

## Commit & Pull Request Guidelines
- Commit messages follow Conventional Commits (examples from history: `feat: ...`, `fix(gamepad): ...`, `docs: ...`).
- Open an issue before large changes (per README). Keep PRs scoped and describe:
  - What changed and why.
  - Commands run (e.g., `just test`, `npm run build`).
  - Screenshots or short clips for UI changes.

## Configuration & Docs
- Prefer editing service configs in `configs/` and documenting changes in `docs/`.
- Architecture references live in `documents/` and `docs/architecture.md`.
