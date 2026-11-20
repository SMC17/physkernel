const std = @import("std");

/// Service-health controller: two cumulative detectors and a cooldown.
/// - Latency: one-sided CUSUM on p99 above 1.25x baseline; at 2.5 it returns `shed_load`.
/// - Errors: a CUSUM over a Bernoulli log-likelihood ratio, fed with "error rate above 2x
///   baseline". At e >= 20 it returns `canary_rollback`.
/// The thresholds are heuristics. The error detector is not anytime-valid, because the
/// probability of "rate above 2x baseline" under normal operation is not `baseline_err`, so
/// its increments are not e-values. `quarantine_tenant` is defined but never returned, and
/// `cost_per_req` is read by nothing.
pub const RemediationAction = enum(u8) {
    none,
    shed_load,
    canary_rollback,
    quarantine_tenant,
};

pub const TelemetrySample = struct {
    ts_ns: i64,
    p99_ms: f64,
    error_rate: f64,
    cost_per_req: f64,
};

pub const SreController = struct {
    baseline_p99_ms: f64,
    baseline_err: f64,
    cooldown_ns: i64,
    last_action_ns: i64 = -1_000_000_000_000,
    cusum_lat: f64 = 0.0,
    log_e_err: f64 = 0.0,
    rollbacks_triggered: u32 = 0,

    pub fn init(baseline_p99_ms: f64, baseline_err: f64, cooldown_ns: i64) SreController {
        return .{
            .baseline_p99_ms = baseline_p99_ms,
            .baseline_err = baseline_err,
            .cooldown_ns = cooldown_ns,
        };
    }

    pub fn ingest(self: *SreController, s: TelemetrySample) RemediationAction {
        // One-sided CUSUM on p99 latency above 1.25x baseline
        const lat_excess = (s.p99_ms - self.baseline_p99_ms * 1.25) / @max(1.0, self.baseline_p99_ms);
        self.cusum_lat = @max(0.0, self.cusum_lat + lat_excess);

        // CUSUM over a Bernoulli log-likelihood ratio on "error rate above 2x baseline" (heuristic)
        const p0 = @max(0.001, self.baseline_err);
        const p1 = @min(0.95, p0 * 4.0);
        const lr = if (s.error_rate > p0 * 2.0) @log(p1 / p0) else @log((1.0 - p1) / (1.0 - p0));
        self.log_e_err = @max(0.0, self.log_e_err + lr);

        if (s.ts_ns - self.last_action_ns < self.cooldown_ns) {
            return .none;
        }

        if (@exp(@min(self.log_e_err, 30.0)) >= 20.0) {
            self.last_action_ns = s.ts_ns;
            self.log_e_err = 0.0;
            self.rollbacks_triggered += 1;
            return .canary_rollback;
        }
        if (self.cusum_lat >= 2.5) {
            self.last_action_ns = s.ts_ns;
            self.cusum_lat = 0.0;
            return .shed_load;
        }
        return .none;
    }
};

test "SreController triggers canary_rollback on error spike and respects cooldown hysteresis" {
    var sre = SreController.init(50.0, 0.01, 10_000_000);

    // Nominal samples
    var i: i64 = 0;
    while (i < 5) : (i += 1) {
        try std.testing.expectEqual(RemediationAction.none, sre.ingest(.{
            .ts_ns = i * 1_000_000,
            .p99_ms = 48.0,
            .error_rate = 0.01,
            .cost_per_req = 0.002,
        }));
    }

    // Error rate spike triggers canary_rollback
    var rolled_back = false;
    var j: i64 = 0;
    while (j < 6) : (j += 1) {
        const act = sre.ingest(.{
            .ts_ns = (10 + j) * 1_000_000,
            .p99_ms = 55.0,
            .error_rate = 0.08,
            .cost_per_req = 0.002,
        });
        if (act == .canary_rollback) {
            rolled_back = true;
            break;
        }
    }
    try std.testing.expect(rolled_back);
    try std.testing.expectEqual(@as(u32, 1), sre.rollbacks_triggered);

    // Immediate next spike within cooldown window must be suppressed (hysteresis)
    try std.testing.expectEqual(RemediationAction.none, sre.ingest(.{
        .ts_ns = 12_000_000,
        .p99_ms = 300.0,
        .error_rate = 0.50,
        .cost_per_req = 0.01,
    }));
}
