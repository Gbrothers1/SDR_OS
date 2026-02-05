# AMD RTX Host Machine - Hardware Architecture Diagram

## System Information
- **Hostname**: amd-rtx
- **User**: ethan
- **Working Directory**: ~/SDR_OS

## Expected Hardware Components (AMD RTX Configuration)

```
┌─────────────────────────────────────────────────────────────┐
│                  AMD RTX HOST MACHINE                        │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────────┐         ┌──────────────────┐          │
│  │   CPU CORES      │         │   GPU (RTX)      │          │
│  │  (AMD RYZEN)     │         │   Architecture   │          │
│  │                  │         │                  │          │
│  │ • Multi-core     │         │ • CUDA Cores     │          │
│  │   processing     │         │ • VRAM (GDDR)    │          │
│  │ • Cache hierarchy│         │ • Tensor ops     │          │
│  │ • Thermal design │         │ • Deep Learning  │          │
│  └──────────────────┘         └──────────────────┘          │
│          │                            │                      │
│          └────────┬───────────────────┘                      │
│                   │ System Bus                               │
│                   │                                          │
│  ┌────────────────┴──────────────────────────────┐           │
│  │                                               │           │
│  ▼                ▼                              ▼           │
│ ┌──────────┐  ┌──────────┐  ┌──────────────┐ ┌──────────┐  │
│ │   RAM    │  │   SSD    │  │  IIO Sensors │ │ Network  │  │
│ │ (System) │  │ (Storage)│  │  (Telemetry) │ │Interface │  │
│ │          │  │          │  │              │ │          │  │
│ │ DDR5/DDR4│  │ NVMe/SSD │  │ • Accelero-  │ │ • Ethernet│  │
│ │ Multi-GB │  │ Multi-TB │  │   meters     │ │ • Potential│ │
│ │          │  │          │  │ • Gyroscope  │ │   WiFi    │  │
│ └──────────┘  └──────────┘  │ • Magnetometer │ └──────────┘  │
│                               │ • Temperature │               │
│                               │ • Pressure    │               │
│                               └───────────────┘               │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐    │
│  │           I/O Interfaces                             │    │
│  │  • PCIe Slots (Gen 3/4)   • USB (3.0/3.1/2.0)      │    │
│  │  • SATA Ports             • Serial/UART             │    │
│  │  • DisplayPort/HDMI       • GPIO                     │    │
│  └──────────────────────────────────────────────────────┘    │
│                                                               │
└─────────────────────────────────────────────────────────────┘

## Software Stack
- **OS**: Linux (likely Ubuntu/Debian)
- **Runtime**: Python (ROS integration visible)
- **Web Server**: Node.js (server.js)
- **Build System**: npm, ROS, Python setuptools
- **GPU Framework**: CUDA (for RTX optimization)

## Key Features for SDR_OS Project
1. **Real-time Processing**: CPU + GPU for telemetry
2. **Sensor Integration**: IIO subsystem for hardware sensors
3. **Network Communication**: Remote git access, web interface
4. **Development Tools**: Python virtual environment (.venv)

## Network Architecture
```
amd-rtx (dev machine)
    │
    └─→ origin/main (remote)
        └─→ ethans-mac-mini (backup host on Tailscale network)
```

## Performance Characteristics
- High-parallel computation (GPU)
- Real-time sensor sampling
- Low-latency network communication via Tailscale
- Suitable for machine learning & signal processing tasks

---
*Generated on 2026-02-05 for SDR_OS project*
