# Solutions & Architecture Reference

This section contains finalized architecture diagrams, design decisions, and reference material from each development phase. These are the "solved" designs that inform implementation.

## Phase Index

| Phase | Title | Status | Key Diagrams |
|-------|-------|--------|-------------|
| [Phase 1](phase1-architecture.md) | CUDA Docker + Basic Pipeline | Complete | Service topology, SHM ringbuffer, video pipeline, CI pipeline |
| [Phase 2](phase2-architecture.md) | NATS Backbone + Safety Stack | Complete | Safety state machine, binary WS protocol, Caddy routing, NATS message flow |
| Phase 3 | Production Hardening | Planned | — |
| Phase 4 | WebRTC + Control Path | Planned | — |
| Phase 5 | Training Integration | Planned | — |
| Phase 6 | Distribution | Planned | — |

## How This Section Works

Each phase gets a solutions page when its implementation completes. The page captures:

- **Service topology** — what runs where and how services connect
- **Memory layouts** — byte-level data structures for IPC protocols
- **Data flow diagrams** — how data moves through the pipeline
- **State machines** — protocol behavior under different conditions
- **CI/CD architecture** — what gets tested and how

These diagrams are the authoritative reference for the architecture. If code and diagrams disagree, update the diagrams (they may be stale) or fix the code (it may have drifted).
