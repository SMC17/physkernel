# physkernel

**Zero-Allocation Real-Time Physical-AI Sensor-Fusion, Anytime-Valid Safety E-Stop, MPC World-Model Rollout, and Closed-Loop SRE Kernel.**

Designed for 1kHz+ deterministic control loops (drones, robotics, autonomous edge telemetry) with zero heap allocations or GC pauses.

## Quickstart
```sh
zig build test
zig build -Doptimize=ReleaseFast
./zig-out/bin/physkernel bench
```
