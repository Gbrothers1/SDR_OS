# Runner Setup (GPU + macOS)

## Labels Used
- CUDA: `self-hosted`, `linux`, `gpu`, `nvidia`
- ROCm: `self-hosted`, `linux`, `gpu`, `rocm`
- MLX: `self-hosted`, `macos`, `arm64`, `mlx`

## CUDA Runner Requirements
- NVIDIA driver + `nvidia-container-toolkit`
- Docker + Compose v2
- Access to the repo and build cache

## ROCm Runner Requirements
- ROCm drivers installed
- `/dev/kfd` accessible to the runner user
- Docker + Compose v2

## MLX Runner Requirements
- macOS 14+ on Apple Silicon
- Python 3.11 available
- No Docker required for MLX tests
