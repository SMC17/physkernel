# physkernel

**[Status](STATUS.md) · [Roadmap](ROADMAP.md) · [Contributing](CONTRIBUTING.md) · [Issues](https://github.com/SMC17/physkernel/issues)**

Small, allocation-free building blocks for a sensor-to-control loop, in Zig 0.16: ring
buffers for IMU and position fixes, a vibration detector, a candidate-trajectory selector, and
a service-health controller. A C API (`libphyskernel`) exposes the rollout step.

Status: **unit-tested** (5 tests). This is a prototype. **The safety cutoff is not usable as
one:** see below.

## What is in it

| Part | What it does |
| --- | --- |
| `src/rt_fusion.zig` (`RtFusionKernel`) | Fixed-capacity ring buffers sized at comptime, with no heap use. An as-of lookup of the latest position fix (a linear scan). Goertzel power at one target frequency over the last 64 samples. A Welford baseline learned from the first 64 samples. A cumulative detector that moves `SAFE → WARN → E_STOP`. |
| `src/world_sim.zig` (`rolloutPlan`) | Simulates each caller-supplied sequence of accelerations on a 2-D point mass. Drops any sequence that comes within `radius + margin` of an obstacle at a step endpoint, and returns the cheapest survivor (control effort + squared distance to goal). |
| `src/self_heal_sre.zig` (`SreController`) | Compares p99 latency and error rate against a baseline and returns `none`, `scale_out`, `canary_rollback` or `page`, with a cooldown. |
| `src/c_api.zig` | `physkernel_rollout_step` for C callers. |

## What it is not

- **The E-STOP is not anytime-valid, and its thresholds are not error rates.** The comments say
  `alpha <= 0.05` for WARN and `alpha <= 0.001` for E_STOP. The statistic is a CUSUM recursion
  (clamped at zero) over a likelihood ratio that is invalid under the null hypothesis.
  **Measured** on pure noise with no anomaly (2,000 runs × 1,000 steps, seed 17):
  WARN fires in **99.8%** of runs and E_STOP in **54%**
  ([#2](https://github.com/SMC17/physkernel/issues/2)).
- **`rolloutPlan` is not MPC.** It selects among candidates you supply and does not optimise.
  Collisions are checked only at step endpoints, so fast trajectories can pass through
  obstacles ([#3](https://github.com/SMC17/physkernel/issues/3)).
- **"1 kHz" is a parameter, not a measurement.** No loop here has been timed against a real
  clock or real hardware.
- **The Lean file does not verify the Zig code.** `lean/PhysKernelProofs.lean` proves that a Lean
  model of the state machine latches in E_STOP and that 20 < 1000. Nothing links it to
  `rt_fusion.zig`.

## Build

```sh
# Zig 0.16.0
zig build test --summary all      # 5/5
zig build -Doptimize=ReleaseFast
./zig-out/bin/physkernel bench
```

## Measured

Apple M2 Max, macOS 26.4, Zig 0.16.0 ReleaseFast, commit `c1ead21`, 2026-11-19.

| What | Workload | Result |
| --- | --- | ---: |
| `stepImu` | 10,000 constant samples, 128-sample buffer | ~180 ns per step |
| `rolloutPlan` | 2 candidates × 2 steps, 1 obstacle | **not measured.** The bench prints 0 because the compiler removes the loop ([#3](https://github.com/SMC17/physkernel/issues/3)) |

## License

MIT
