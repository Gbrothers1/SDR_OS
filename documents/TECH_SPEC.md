# Technical Specifications

## Hardware

### Computer

| Component | Specification |
|-----------|--------------|
| CPU | AMD Ryzen 5 2600 Six-Core (12 threads) |
| RAM | 8 GB |
| GPU | NVIDIA GeForce RTX 2080 Ti (11 GB VRAM) |

### Connected Devices

<!-- Sensors, controllers, peripherals - add as connected -->

## Software

### OS

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-65-generic |

### Development Tools

| Tool | Version |
|------|---------|
| Python | 3.12.12 |
| uv | 0.9.30 |
| pnpm | 10.28.2 |

### Key Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| PyTorch | 2.10.0+cu128 | ML backend (CUDA enabled) |
| Genesis | 0.3.13 | Physics simulation |
| Genesis Forge | 0.3.0 | Training framework |
| NumPy | 2.3.5 | Numerical computing |
| OpenCV | 4.13.0 | Computer vision |
| MuJoCo | 3.4.0 | Physics engine |

### Performance Guidance
- Avoid giant global lists/dicts; use scoped data structures.
- Prefer generators and iterators for streaming pipelines.
- Slim object graphs to reduce GC pressure.
- Profile to identify hot paths before optimizing.

## Instrumentation Results

<!--
Add metrics under dated headers:
### [Test Name] - YYYY-MM-DD
Results here
-->
