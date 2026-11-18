# physkernel

**[Status](STATUS.md) · [Roadmap](ROADMAP.md) · [Contributing](CONTRIBUTING.md) · [Issues](https://github.com/SMC17/physkernel/issues) · [Public roadmap](https://github.com/users/SMC17/projects/16)**

**Zero-Allocation Real-Time Physical-AI Sensor-Fusion, Anytime-Valid Safety E-Stop, MPC World-Model Rollout, and Closed-Loop SRE Kernel.**

Designed for 1kHz+ deterministic control loops (drones, robotics, autonomous edge telemetry) with zero heap allocations or GC pauses.

## Quickstart
```sh
zig build test
zig build -Doptimize=ReleaseFast
./zig-out/bin/physkernel bench
```
