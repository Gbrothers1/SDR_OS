# Environment Setup

## Hard Requirements

| Tool | Version | Purpose |
|------|---------|---------|
| **Python** | >=3.10, <3.14 | Genesis/Genesis Forge compatibility |
| **uv** | latest | Python package management |
| **pnpm** | latest | Node.js package management |
| **PyTorch** | latest | ML backend for Genesis |

> **DO NOT USE:** pip (use uv), npm/yarn (use pnpm)

## Prerequisites

### System Requirements
- Linux (Ubuntu 22.04+ recommended)
- CUDA-compatible GPU (optional, for accelerated training)

### Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Verify: `uv --version`

### Install pnpm

```bash
curl -fsSL https://get.pnpm.io/install.sh | sh -
```

Verify: `pnpm --version`

## Python Environment

### Initialize Project

```bash
# Initialize uv project (creates pyproject.toml, .python-version)
uv init

# Set Python version
uv python pin 3.12
```

### Install PyTorch

```bash
# NVIDIA GPU (CUDA 12.6)
uv pip install torch --index-url https://download.pytorch.org/whl/cu126

# CPU only
uv pip install torch --index-url https://download.pytorch.org/whl/cpu

# Apple Silicon (Metal/MPS)
uv pip install torch
```

### Install Genesis Ecosystem

```bash
# Genesis simulator
uv add genesis-world

# Genesis Forge (training framework)
uv add genesis-forge
```

## Node.js Environment

### Initialize Project

```bash
pnpm init
```

### Install Dependencies

```bash
pnpm add <package>        # Add dependency
pnpm add -D <package>     # Add dev dependency
pnpm install              # Install all dependencies
```

## Quick Start

```bash
# 1. Clone and enter project
cd SDR_OS

# 2. Sync Python environment
uv sync

# 3. Install Node dependencies (if package.json exists)
pnpm install

# 4. Verify Genesis installation
uv run python -c "import genesis; print(genesis.__version__)"
```

## Common Commands

### uv (Python)

| Command | Description |
|---------|-------------|
| `uv sync` | Install dependencies from pyproject.toml |
| `uv add <pkg>` | Add a dependency |
| `uv remove <pkg>` | Remove a dependency |
| `uv run <cmd>` | Run command in virtual environment |
| `uv pip list` | List installed packages |

### pnpm (Node.js)

| Command | Description |
|---------|-------------|
| `pnpm install` | Install dependencies from package.json |
| `pnpm add <pkg>` | Add a dependency |
| `pnpm remove <pkg>` | Remove a dependency |
| `pnpm run <script>` | Run a script from package.json |
| `pnpm list` | List installed packages |
