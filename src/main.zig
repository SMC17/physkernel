const std = @import("std");
pub const rt_fusion = @import("rt_fusion.zig");
pub const world_sim = @import("world_sim.zig");
pub const self_heal_sre = @import("self_heal_sre.zig");
pub const c_api = @import("c_api.zig");

pub fn main(init: std.process.Init) !void {
    const alloc = init.arena.allocator();
    const io = init.io;
    const stdout = std.Io.File.stdout();

    const args = try init.minimal.args.toSlice(alloc);
    if (args.len >= 2 and std.mem.eql(u8, args[1], "bench")) {
        var rt = rt_fusion.RtFusionKernel(128, 16).init(120.0, 1000.0);
        rt.pushSpatial(.{ .ts_ns = 0, .h3_cell = 0x8928308280fffff, .speed_mps = 12.0 });
        const t0 = std.Io.Clock.awake.now(io);
        var i: usize = 0;
        var rt_sink: f64 = 0;
        while (i < 10_000) : (i += 1) {
            rt_sink += rt.stepImu(.{ .ts_ns = @intCast(i * 1000), .accel_z = 9.81, .gyro_z = 0.01 }).resonant_energy;
        }
        std.mem.doNotOptimizeAway(rt_sink);
        const t1 = std.Io.Clock.awake.now(io);
        const rt_ns: i96 = @divTrunc(t0.durationTo(t1).nanoseconds, 10_000);

        const start = world_sim.State2D{ .pos = .{ .x = 0, .y = 0 }, .vel = .{ .x = 0, .y = 0 } };
        const goal = world_sim.Vec2{ .x = 10, .y = 0 };
        const obs = [_]world_sim.Obstacle{.{ .pos = .{ .x = 5, .y = 0 }, .radius = 1.5 }};
        const c0 = [_]world_sim.ControlCmd{ .{ .ax = 5.0, .ay = 0.0 }, .{ .ax = 0.0, .ay = 0.0 } };
        const c1 = [_]world_sim.ControlCmd{ .{ .ax = 5.0, .ay = 5.0 }, .{ .ax = 5.0, .ay = -2.0 } };
        const cands = [_][]const world_sim.ControlCmd{ &c0, &c1 };
        const t2 = std.Io.Clock.awake.now(io);
        i = 0;
        var w_sink: f64 = 0;
        while (i < 10_000) : (i += 1) {
            // Vary the start each iteration and hide it from the optimiser, so the work
            // cannot be hoisted out of the loop (it was, and the bench printed 0 ns).
            var s0 = start;
            s0.pos.y = @as(f64, @floatFromInt(i % 7)) * 1e-3;
            std.mem.doNotOptimizeAway(&s0);
            w_sink += world_sim.rolloutPlan(s0, goal, &obs, &cands, 1.0, 0.2).best_cost;
        }
        std.mem.doNotOptimizeAway(w_sink);
        const t3 = std.Io.Clock.awake.now(io);
        const rollout_ns: i96 = @divTrunc(t2.durationTo(t3).nanoseconds, 10_000);

        var buf: [160]u8 = undefined;
        const msg = try std.fmt.bufPrint(&buf, "{{\"rt_fusion_ns_per_tick\":{d},\"rollout_ns\":{d}}}\n", .{ rt_ns, rollout_ns });
        try stdout.writeStreamingAll(io, msg);
        return;
    }

    if (args.len >= 2 and std.mem.eql(u8, args[1], "stream")) {
        var rt = rt_fusion.RtFusionKernel(128, 16).init(120.0, 1000.0);
        var stdin_buf: [4096]u8 = undefined;
        var stdin_reader = std.Io.File.stdin().reader(io, &stdin_buf);
        var out_buf: [256]u8 = undefined;

        while (try stdin_reader.interface.takeDelimiter('\n')) |raw_line| {
            const line = std.mem.trim(u8, raw_line, " \t\r\n");
            if (line.len == 0) continue;

            var it = std.mem.splitScalar(u8, line, ',');
            const ts_str = std.mem.trim(u8, it.next() orelse continue, " \t");
            const az_str = std.mem.trim(u8, it.next() orelse continue, " \t");
            const gz_str = std.mem.trim(u8, it.next() orelse continue, " \t");

            const ts_ns = std.fmt.parseInt(i64, ts_str, 10) catch continue;
            const accel_z = std.fmt.parseFloat(f64, az_str) catch continue;
            const gyro_z = std.fmt.parseFloat(f64, gz_str) catch continue;

            const frame = rt.stepImu(.{
                .ts_ns = ts_ns,
                .accel_z = accel_z,
                .gyro_z = gyro_z,
            });
            const msg = try std.fmt.bufPrint(
                &out_buf,
                "{{\"ts_ns\":{d},\"safety\":\"{s}\",\"e_value\":{d:.6},\"resonant_energy\":{d:.6}}}\n",
                .{ frame.ts_ns, @tagName(frame.safety), frame.e_value, frame.resonant_energy },
            );
            try stdout.writeStreamingAll(io, msg);
        }
        return;
    }

    try stdout.writeStreamingAll(
        io,
        "physkernel — allocation-free sensor buffers, a CUSUM e-detector E-STOP, candidate-trajectory selection\n\nUsage:\n  physkernel bench\n  physkernel stream   (reads CSV ts_ns,accel_z,gyro_z from stdin, emits JSONL)\n",
    );
}

test {
    std.testing.refAllDecls(@This());
}
