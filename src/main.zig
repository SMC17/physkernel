const std = @import("std");
pub const rt_fusion = @import("rt_fusion.zig");
pub const world_sim = @import("world_sim.zig");
pub const self_heal_sre = @import("self_heal_sre.zig");

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const alloc = arena.allocator();

    const args = try std.process.argsAlloc(alloc);
    if (args.len >= 2 and std.mem.eql(u8, args[1], "bench")) {
        var timer = try std.time.Timer.start();

        var rt = rt_fusion.RtFusionKernel(128, 16).init(120.0, 1000.0);
        rt.pushSpatial(.{ .ts_ns = 0, .h3_cell = 0x8928308280fffff, .speed_mps = 12.0 });
        timer.reset();
        var i: usize = 0;
        var rt_sink: f64 = 0;
        while (i < 10_000) : (i += 1) {
            rt_sink += rt.stepImu(.{ .ts_ns = @intCast(i * 1000), .accel_z = 9.81, .gyro_z = 0.01 }).resonant_energy;
        }
        std.mem.doNotOptimizeAway(rt_sink);
        const rt_ns = timer.read() / 10_000;

        const start = world_sim.State2D{ .pos = .{ .x = 0, .y = 0 }, .vel = .{ .x = 0, .y = 0 } };
        const goal = world_sim.Vec2{ .x = 10, .y = 0 };
        const obs = [_]world_sim.Obstacle{.{ .pos = .{ .x = 5, .y = 0 }, .radius = 1.5 }};
        const c0 = [_]world_sim.ControlCmd{ .{ .ax = 5.0, .ay = 0.0 }, .{ .ax = 0.0, .ay = 0.0 } };
        const c1 = [_]world_sim.ControlCmd{ .{ .ax = 5.0, .ay = 5.0 }, .{ .ax = 5.0, .ay = -2.0 } };
        const cands = [_][]const world_sim.ControlCmd{ &c0, &c1 };
        timer.reset();
        i = 0;
        var w_sink: usize = 0;
        while (i < 10_000) : (i += 1) {
            w_sink +%= world_sim.rolloutPlan(start, goal, &obs, &cands, 1.0, 0.2).safe_candidates;
        }
        std.mem.doNotOptimizeAway(w_sink);
        const rollout_ns = timer.read() / 10_000;

        var buf: [160]u8 = undefined;
        const msg = try std.fmt.bufPrint(&buf, "{{\"rt_fusion_ns_per_tick\":{d},\"mpc_rollout_ns\":{d}}}\n", .{ rt_ns, rollout_ns });
        try std.fs.File.stdout().writeAll(msg);
        return;
    }

    try std.fs.File.stdout().writeAll("physkernel — Zero-Alloc Physical-AI Sensor Fusion, Safety E-Stop & MPC World-Model Rollout\n\nUsage:\n  physkernel bench\n");
}

test {
    std.testing.refAllDecls(@This());
}
