const std = @import("std");

/// Allocation-free sensor ring buffers and a mean-shift detector for a control loop.
/// - Fixed-capacity ring buffers for IMU samples and position fixes, sized at comptime.
/// - As-of lookup of the latest position fix (linear scan of the fix buffer).
/// - Goertzel power at one frequency, and a Welford baseline learned during warm-up.
/// - A CUSUM e-detector over the standardised vertical acceleration (`safe` / `warn` / `e_stop`).
///
/// The detector, and what its thresholds mean:
/// each step multiplies in the e-value e_t = cosh(mu * z_t) * exp(-mu^2 / 2). That is the
/// likelihood ratio of a symmetric shift of +/- mu standard deviations against no shift, and
/// E[e_t] = 1 exactly when z_t ~ N(0, 1). The statistic L_t = max(0, L_{t-1} + log e_t) is the
/// CUSUM e-detector (Shin, Ramdas and Rinaldo, "E-detectors", 2023). Under the no-shift
/// hypothesis, its mean time to a false alarm at threshold c is at least c steps. A threshold
/// is therefore an average-run-length bound, not a per-run error rate.
/// The bound assumes z_t ~ N(0, 1). The mean and variance are estimated from the warm-up
/// samples, so the bound is approximate, and it fails if the noise is not Gaussian or not
/// independent.
pub const SafetyState = enum(u8) {
    safe,
    warn,
    e_stop,
};

pub const ImuSample = struct {
    ts_ns: i64,
    accel_z: f64,
    gyro_z: f64,
};

pub const SpatialFix = struct {
    ts_ns: i64,
    h3_cell: u64,
    speed_mps: f64,
};

pub const FusedFrame = struct {
    ts_ns: i64,
    accel_z: f64,
    h3_cell: u64,
    speed_mps: f64,
    resonant_energy: f64,
    e_value: f64,
    safety: SafetyState,
};

/// log e for e = cosh(mu z) exp(-mu^2/2), computed without overflow for large |z|.
pub fn logMixtureE(z: f64, mu: f64) f64 {
    const a = @abs(mu * z);
    // log cosh(a) = a + log1p(exp(-2a)) - log 2
    return a + std.math.log1p(@exp(-2.0 * a)) - std.math.ln2 - 0.5 * mu * mu;
}

pub fn RtFusionKernel(comptime imu_cap: usize, comptime spatial_cap: usize) type {
    return struct {
        const Self = @This();

        imu_buf: [imu_cap]ImuSample = undefined,
        imu_head: usize = 0,
        imu_len: usize = 0,

        spatial_buf: [spatial_cap]SpatialFix = undefined,
        spatial_head: usize = 0,
        spatial_len: usize = 0,

        // Online Welford baseline statistics for accel_z
        count: u64 = 0,
        mean: f64 = 0.0,
        m2: f64 = 0.0,

        // CUSUM e-detector statistic, in log space (see the file comment).
        log_e_value: f64 = 0.0,
        safety: SafetyState = .safe,

        /// Samples used to learn the baseline mean and variance before detection starts.
        warm_up: u64 = 256,
        /// Shift size, in standard deviations, that the detector is tuned to.
        shift_sigmas: f64 = 3.0,
        /// Thresholds on the e-detector. Under no shift the mean steps to cross them are at least
        /// these values: 1e5 steps is about 100 s at 1 kHz, and 1e9 steps about 11.6 days.
        warn_e_threshold: f64 = 1e5,
        estop_e_threshold: f64 = 1e9,
        target_freq_hz: f64 = 120.0,
        sample_rate_hz: f64 = 1000.0,

        pub fn init(target_freq_hz: f64, sample_rate_hz: f64) Self {
            return .{
                .target_freq_hz = target_freq_hz,
                .sample_rate_hz = sample_rate_hz,
            };
        }

        pub fn pushSpatial(self: *Self, fix: SpatialFix) void {
            const idx = (self.spatial_head + self.spatial_len) % spatial_cap;
            if (self.spatial_len == spatial_cap) {
                self.spatial_head = (self.spatial_head + 1) % spatial_cap;
            } else {
                self.spatial_len += 1;
            }
            self.spatial_buf[idx] = fix;
        }

        /// Zero-alloc `asof` lookup: finds latest spatial fix with `ts_ns <= query_ts`.
        pub fn asofSpatial(self: *const Self, query_ts: i64) ?SpatialFix {
            if (self.spatial_len == 0) return null;
            var best: ?SpatialFix = null;
            var i: usize = 0;
            while (i < self.spatial_len) : (i += 1) {
                const idx = (self.spatial_head + i) % spatial_cap;
                const s = self.spatial_buf[idx];
                if (s.ts_ns <= query_ts) {
                    best = s;
                } else {
                    break;
                }
            }
            return best;
        }

        /// Zero-alloc Goertzel power at `target_freq_hz` over the trailing `window` IMU samples.
        pub fn goertzelPower(self: *const Self, window: usize) f64 {
            const n = @min(window, self.imu_len);
            if (n < 4) return 0.0;
            const k = (self.target_freq_hz * @as(f64, @floatFromInt(n))) / self.sample_rate_hz;
            const omega = (2.0 * std.math.pi * k) / @as(f64, @floatFromInt(n));
            const coeff = 2.0 * @cos(omega);

            var q1: f64 = 0.0;
            var q2: f64 = 0.0;
            const start = self.imu_len - n;
            var i: usize = 0;
            while (i < n) : (i += 1) {
                const idx = (self.imu_head + start + i) % imu_cap;
                const x = self.imu_buf[idx].accel_z - self.mean;
                const q0 = coeff * q1 - q2 + x;
                q2 = q1;
                q1 = q0;
            }
            return q1 * q1 + q2 * q2 - q1 * q2 * coeff;
        }

        /// One control-loop step. No allocation; cost is measured by `physkernel bench`.
        pub fn stepImu(self: *Self, sample: ImuSample) FusedFrame {
            const idx = (self.imu_head + self.imu_len) % imu_cap;
            if (self.imu_len == imu_cap) {
                self.imu_head = (self.imu_head + 1) % imu_cap;
            } else {
                self.imu_len += 1;
            }
            self.imu_buf[idx] = sample;

            // Learn the baseline (Welford) during warm-up only.
            self.count += 1;
            if (self.count <= self.warm_up) {
                const delta = sample.accel_z - self.mean;
                self.mean += delta / @as(f64, @floatFromInt(self.count));
                self.m2 += delta * (sample.accel_z - self.mean);
            }

            if (self.count > self.warm_up and self.safety != .e_stop) {
                const sigma2 = @max(1e-12, self.m2 / @as(f64, @floatFromInt(self.warm_up - 1)));
                const z = (sample.accel_z - self.mean) / @sqrt(sigma2);
                self.log_e_value = @max(0.0, self.log_e_value + logMixtureE(z, self.shift_sigmas));
                if (self.log_e_value >= @log(self.estop_e_threshold)) {
                    self.safety = .e_stop; // latches
                } else if (self.log_e_value >= @log(self.warn_e_threshold)) {
                    self.safety = .warn;
                } else {
                    self.safety = .safe;
                }
            }

            const sp = self.asofSpatial(sample.ts_ns);
            const power = self.goertzelPower(64);
            return .{
                .ts_ns = sample.ts_ns,
                .accel_z = sample.accel_z,
                .h3_cell = if (sp) |s| s.h3_cell else 0,
                .speed_mps = if (sp) |s| s.speed_mps else 0.0,
                .resonant_energy = power,
                .e_value = @exp(@min(self.log_e_value, 40.0)),
                .safety = self.safety,
            };
        }
    };
}

test "RtFusionKernel fusion, Goertzel resonance, and E_STOP on a severe vibration" {
    var rt = RtFusionKernel(128, 16).init(120.0, 1000.0);

    rt.pushSpatial(.{ .ts_ns = 1_000_000, .h3_cell = 0x8928308280fffff, .speed_mps = 12.5 });
    rt.pushSpatial(.{ .ts_ns = 2_000_000, .h3_cell = 0x8928308280bffff, .speed_mps = 13.0 });

    // Nominal warm-up (256 samples) around 9.81 m/s^2 with small noise
    var i: usize = 0;
    while (i < 256) : (i += 1) {
        const noise: f64 = if ((i & 1) == 0) 0.02 else -0.02;
        const f = rt.stepImu(.{
            .ts_ns = @intCast(1_500_000 + i * 1_000),
            .accel_z = 9.81 + noise,
            .gyro_z = 0.01,
        });
        try std.testing.expectEqual(SafetyState.safe, f.safety);
        try std.testing.expectEqual(@as(u64, 0x8928308280fffff), f.h3_cell);
    }

    // Inject severe 120Hz vibration anomaly after ts=2_000_000 -> must align to new H3 cell and trip E_STOP
    var tripped = false;
    var j: usize = 0;
    while (j < 40) : (j += 1) {
        const phase = 2.0 * std.math.pi * 120.0 * (@as(f64, @floatFromInt(j)) / 1000.0);
        const f = rt.stepImu(.{
            .ts_ns = @intCast(2_000_000 + j * 1_000),
            .accel_z = 9.81 + 2.5 * @sin(phase),
            .gyro_z = 0.4,
        });
        try std.testing.expectEqual(@as(u64, 0x8928308280bffff), f.h3_cell);
        if (f.safety == .e_stop) {
            tripped = true;
        }
    }
    try std.testing.expect(tripped);
    try std.testing.expect(rt.goertzelPower(64) > 100.0);
}

test "logMixtureE is a valid e-value increment: E[exp(.)] = 1 under N(0,1)" {
    // Trapezoid rule on [-12, 12] with the standard normal density.
    var acc: f64 = 0;
    const h = 1e-3;
    var x: f64 = -12.0;
    while (x <= 12.0) : (x += h) {
        acc += @exp(logMixtureE(x, 3.0)) * @exp(-0.5 * x * x) / @sqrt(2.0 * std.math.pi) * h;
    }
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), acc, 1e-6);
    try std.testing.expect(std.math.isFinite(logMixtureE(1e6, 3.0)));
}

fn runNoise(seed: u64, steps: usize, shift_at: ?usize, shift: f64) struct { estop_at: ?usize, warned: bool } {
    var prng = std.Random.DefaultPrng.init(seed);
    const r = prng.random();
    var k = RtFusionKernel(256, 4).init(120.0, 1000.0);
    var warned = false;
    var t: usize = 0;
    while (t < 256 + steps) : (t += 1) {
        const mean_shift: f64 = if (shift_at) |s| (if (t >= 256 + s) shift else 0.0) else 0.0;
        const f = k.stepImu(.{ .ts_ns = @intCast(t * 1_000_000), .accel_z = 9.81 + (r.floatNorm(f64) + mean_shift) * 0.05, .gyro_z = 0 });
        if (f.safety == .warn) warned = true;
        if (f.safety == .e_stop) return .{ .estop_at = t - 256, .warned = true };
    }
    return .{ .estop_at = null, .warned = warned };
}

test "regression #2: no false E_STOP on pure noise; fast E_STOP on a 3-sigma shift" {
    // 500 runs x 1,000 steps of N(0, sigma) noise. The old detector stopped in 54% of runs.
    var false_stops: usize = 0;
    var seed: u64 = 1;
    while (seed <= 500) : (seed += 1) {
        if (runNoise(seed, 1000, null, 0).estop_at != null) false_stops += 1;
    }
    try std.testing.expectEqual(@as(usize, 0), false_stops);

    // A sustained 3-sigma shift starting at step 100 is stopped within 25 steps.
    seed = 1;
    while (seed <= 100) : (seed += 1) {
        const at = runNoise(seed, 1000, 100, 3.0).estop_at orelse return error.MissedShift;
        try std.testing.expect(at >= 100 and at < 125);
    }
}
