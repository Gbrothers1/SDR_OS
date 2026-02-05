---
name: env-docs
description: Use when setting up project environment, installing dependencies, running system benchmarks, capturing hardware specs, or documenting instrumentation results
---

# Environment Documentation

Maintains two canonical files for project setup and technical specifications.

## Files

| File | Purpose |
|------|---------|
| `documents/SETUP.md` | Environment setup, pip installs, special commands |
| `documents/TECH_SPEC.md` | Hardware specs, app specs, instrumentation metrics |

## SETUP.md Structure

```markdown
# Environment Setup

## Prerequisites
[System requirements, OS, etc.]

## Python Environment
[venv creation, Python version]

## Dependencies
[pip install commands - exact versions]

## Special Setup
[ROS setup, hardware drivers, env vars, etc.]

## Quick Start
[Minimal steps to get running]
```

## TECH_SPEC.md Structure

```markdown
# Technical Specifications

## Hardware
### Computer
[CPU, RAM, storage, GPU]

### Connected Devices
[Sensors, controllers, peripherals]

## Software
### OS
[Distro, kernel version]

### Key Dependencies
[ROS version, Python version, critical libs]

## Instrumentation Results
### [Test Name] - [Date]
[Metrics, benchmarks, performance data]
```

## When to Update

**Update SETUP.md when:**
- Adding new pip dependencies
- Changing environment setup steps
- Adding setup scripts or commands
- Modifying prerequisites

**Update TECH_SPEC.md when:**
- Running benchmarks or performance tests
- Profiling the application
- Gathering hardware information
- Measuring sensor performance
- Any instrumentation producing metrics

## Commands for Gathering Specs

```bash
# CPU info
lscpu | grep -E "Model name|CPU\(s\)|Thread|Core"

# Memory
free -h

# GPU
nvidia-smi 2>/dev/null || lspci | grep -i vga

# Storage
df -h /

# OS
cat /etc/os-release | grep -E "NAME|VERSION"
uname -r
```

## Rule

**All environment setup and technical specs go in these files. Not README.md. Not ad-hoc docs.**
