# physkernel

**[Status](STATUS.md) · [Roadmap](ROADMAP.md) · [Contributing](CONTRIBUTING.md) · [Issues](https://github.com/SMC17/physkernel/issues)**

Small, allocation-free building blocks for a sensor-to-control loop, in Zig 0.16:
- ring buffers for IMU samples and position fixes;
- a mean-shift detector that drives a SAFE / WARN / E_STOP state;
- a candidate-trajectory selector;
- a service-health controller.

A C API (`libphyskernel`) exposes the rollout step.

Status: **unit-tested** (8 tests). This is a prototype: nothing here has run on real sensors or
hardware.

## What is in it

| Part | What it does |
| --- | --- |
| `src/rt_fusion.zig` (`RtFusionKernel`) | Ring buffers sized at comptime, with no heap use. As-of lookup of the latest position fix (a linear scan). Goertzel power at one frequency. A baseline mean and variance learned from the first 256 samples. Then a **CUSUM e-detector** on the standardised vertical acceleration, which sets `SAFE → WARN → E_STOP`. E_STOP latches. |
| `src/world_sim.zig` (`rolloutPlan`) | Simulates each caller-supplied acceleration sequence on a 2-D point mass and rejects any whose path between steps comes within `radius + margin` of an obstacle. Returns the cheapest survivor (control effort + squared distance to goal). It chooses among candidates; it does not optimise them, so it is not MPC. |
| `src/self_heal_sre.zig` (`SreController`) | Latency CUSUM returns `shed_load`; an error-rate CUSUM returns `canary_rollback`; with a cooldown. Its thresholds are heuristics (see below). |
| `src/c_api.zig` | `physkernel_rollout_step` for C callers. |

## The E-STOP, and what its thresholds mean

Each step multiplies in `e = cosh(μz)·exp(−μ²/2)`. This is the likelihood ratio of a ±μσ
shift against no shift, with μ = 3 by default, and `E[e] = 1` exactly when `z ~ N(0,1)`. The
test suite checks this numerically. The statistic is the CUSUM e-detector (Shin, Ramdas and
Rinaldo, *E-detectors*, 2023): with no shift, the **mean number of steps before a false alarm is
at least the threshold**. That is a run-length bound, not a per-run error rate.

| Threshold | Default | No-shift mean steps to alarm ≥ | At 1 kHz |
| --- | ---: | ---: | ---: |
| WARN | 1e5 | 1e5 | ~100 s |
| E_STOP | 1e9 | 1e9 | ~11.6 days |

The bound assumes Gaussian, independent noise, and the mean and variance are estimated from
the warm-up samples, so it holds approximately at best. Correlated or heavy-tailed noise will
alarm sooner.

**Measured** with Zig 0.16.0 ReleaseFast on an M2 Max. The noise was i.i.d. Gaussian with no
shift, over 2,000 runs of 1,000 steps after warm-up (seed 17):

| | Before (2026-11-17) | Now |
| --- | ---: | ---: |
| Runs with any E_STOP | **54.2%** | **0 / 2,000** |
| Runs with any WARN | 99.8% | 0.4% |
| Steps to E_STOP after a sustained 3σ shift (1,000 runs) | — | median 5, p95 9, max 14 |

The old detector used `3|z| − 4.5` as its increment, and its expectation under no shift is
about 2, not ≤ 1. Its thresholds (20, 1000) also claimed error rates they could not give.
Fixed in [#2](https://github.com/SMC17/physkernel/issues/2), with a regression test.

## Other limits

- `rolloutPlan` checks each step's straight-line segment against every obstacle. Before
  [#3](https://github.com/SMC17/physkernel/issues/3) it checked only step endpoints, so a fast
  step could pass through an obstacle; there is now a regression test for that. Paths between
  steps are still assumed to be straight.
- `SreController`'s error detector is **not** anytime-valid. "Rate above 2× baseline" does not
  occur with probability `baseline_err` under normal operation, so its increments are not
  e-values. Treat both of its thresholds as heuristics. `quarantine_tenant` is never returned,
  and `cost_per_req` is unused.
- **The Lean file does not verify the Zig code.** `lean/PhysKernelProofs.lean` proves that a Lean
  model of the state machine latches in E_STOP and that WARN < E_STOP. It checks with
  Lean 4.35.0 (`lean lean/PhysKernelProofs.lean`), but nothing links it to `rt_fusion.zig`.

## Build

```sh
# Zig 0.16.0
zig build test --summary all      # 8/8
zig build -Doptimize=ReleaseFast
./zig-out/bin/physkernel bench
```

## Measured speed

Apple M2 Max, macOS 26.4, Zig 0.16.0 ReleaseFast, 2026-11-19, three runs of `physkernel bench`:

| What | Workload | Result |
| --- | --- | ---: |
| `stepImu` | 10,000 samples, 128-sample buffer | 237–285 ns per step |
| `rolloutPlan` | 2 candidates × 2 steps, 1 obstacle, start varied per call | ~13 ns |

`stepImu` costs more than the old ~180 ns because the valid increment needs `exp` and
`log1p`. At 1 kHz that is about 0.03% of the step budget.

## License

MIT
