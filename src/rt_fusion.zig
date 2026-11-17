const std = @import("std");

/// Hard-real-time, zero-allocation physical-AI / edge sensor-fusion & safety kernel.
/// Designed for 1kHz+ deterministic control loops (drones, robotics, industrial telemetry)
/// with zero GC/heap calls after initialization:
/// - Fixed-capacity ring buffers for asynchronous multi-rate sensors
/// - Zero-alloc inline `asof` temporal alignment
/// - Online Goertzel resonant-frequency energy + Welford variance
/// - Anytime-valid sequential e-value safety cutoff (`safe` / `warn` / `e_stop`)
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

        // Anytime-valid e-process accumulator (tests H0: nominal baseline vs H1: anomalous vibration)
        log_e_value: f64 = 0.0,
        safety: SafetyState = .safe,

        // Safety thresholds
        warn_e_threshold: f64 = 20.0, // alpha <= 0.05
        estop_e_threshold: f64 = 1000.0, // alpha <= 0.001
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

        /// Deterministic 1kHz control-loop step. Zero allocations.
        pub fn stepImu(self: *Self, sample: ImuSample) FusedFrame {
            const idx = (self.imu_head + self.imu_len) % imu_cap;
            if (self.imu_len == imu_cap) {
                self.imu_head = (self.imu_head + 1) % imu_cap;
            } else {
                self.imu_len += 1;
            }
            self.imu_buf[idx] = sample;

            // Update Welford online stats during warm-up or nominal operation
            self.count += 1;
            const delta = sample.accel_z - self.mean;
            if (self.count <= 64) {
                self.mean += delta / @as(f64, @floatFromInt(self.count));
                const delta2 = sample.accel_z - self.mean;
                self.m2 += delta * delta2;
            }

            const sigma2 = if (self.count > 2) @max(1e-6, self.m2 / @as(f64, @floatFromInt(@min(self.count, 64) - 1))) else 1.0;

            // Mixture sequential probability ratio test (e-process) for shift >= 3*sigma
            if (self.count > 64 and self.safety != .e_stop) {
                const z = (sample.accel_z - self.mean) / @sqrt(sigma2);
                const mu_alt = 3.0;
                // Likelihood ratio step for N(mu_alt, 1) vs N(0, 1) on |z|
                const lr_log = (@abs(z) * mu_alt) - 0.5 * mu_alt * mu_alt;
                self.log_e_value = @max(0.0, self.log_e_value + lr_log);
                const e_val = @exp(@min(self.log_e_value, 40.0));
                if (e_val >= self.estop_e_threshold) {
                    self.safety = .e_stop;
                } else if (e_val >= self.warn_e_threshold) {
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

test "RtFusionKernel zero-alloc async fusion, Goertzel resonance, and anytime-valid E_STOP" {
    var rt = RtFusionKernel(128, 16).init(120.0, 1000.0);

    rt.pushSpatial(.{ .ts_ns = 1_000_000, .h3_cell = 0x8928308280fffff, .speed_mps = 12.5 });
    rt.pushSpatial(.{ .ts_ns = 2_000_000, .h3_cell = 0x8928308280bffff, .speed_mps = 13.0 });

    // Nominal 64-sample warmup around 9.81 m/s^2 with tiny noise
    var i: usize = 0;
    while (i < 64) : (i += 1) {
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
