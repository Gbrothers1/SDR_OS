# SDR_OS

SDR_OS is a multi-backend robotics simulation and control platform targeting CUDA (NVIDIA), ROCm (AMD), and MLX (Apple Silicon).

## Docs
- Built from `docs/` using MkDocs.
- Versioned with Git tags (via `mike`).

Local build:

```bash
pip install -r requirements/requirements-docs.txt
mkdocs serve
```

## Repository Layout
- `src/` shared source (single codebase)
- `containers/` backend-specific environments (cuda/rocm/mlx)
- `docs/` release-facing documentation site
- `documents/` internal architecture and planning docs

## License
MIT. See `LICENSE`.
