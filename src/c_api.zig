const std = @import("std");
const world_sim = @import("world_sim.zig");

pub const CControlCmd = extern struct {
    ax: f64,
    ay: f64,
};

pub const CObstacle = extern struct {
    x: f64,
    y: f64,
    radius: f64,
};

pub const CRolloutOut = extern struct {
    best_idx: i32,
    best_cost: f64,
    safe_candidates: u32,
};

const MAX_OBS = 64;
const MAX_CANDS = 64;
const MAX_HORIZON = 64;

/// C-ABI rollout step over up to 64 candidate trajectories of length `horizon` (<= 64)
/// and up to 64 obstacles. Zero heap allocations.
export fn physkernel_rollout_step(
    px: f64,
    py: f64,
    vx: f64,
    vy: f64,
    goal_x: f64,
    goal_y: f64,
    obs_ptr: ?[*]const CObstacle,
    obs_len: usize,
    cmds_ptr: ?[*]const CControlCmd,
    num_cands: usize,
    horizon: usize,
    dt: f64,
    safety_margin: f64,
    out: *CRolloutOut,
) callconv(.c) i32 {
    if (obs_len > MAX_OBS or num_cands == 0 or num_cands > MAX_CANDS or horizon == 0 or horizon > MAX_HORIZON) {
        return -1;
    }
    const c_ptr = cmds_ptr orelse return -1;
    if (obs_len > 0 and obs_ptr == null) return -1;

    var obs_buf: [MAX_OBS]world_sim.Obstacle = undefined;
    for (0..obs_len) |i| {
        const o = obs_ptr.?[i];
        obs_buf[i] = .{ .pos = .{ .x = o.x, .y = o.y }, .radius = o.radius };
    }

    var cmd_buf: [MAX_CANDS * MAX_HORIZON]world_sim.ControlCmd = undefined;
    var slices: [MAX_CANDS][]const world_sim.ControlCmd = undefined;
    for (0..num_cands) |c| {
        const base = c * horizon;
        for (0..horizon) |h| {
            const cmd = c_ptr[base + h];
            cmd_buf[base + h] = .{ .ax = cmd.ax, .ay = cmd.ay };
        }
        slices[c] = cmd_buf[base .. base + horizon];
    }

    const res = world_sim.rolloutPlan(
        .{ .pos = .{ .x = px, .y = py }, .vel = .{ .x = vx, .y = vy } },
        .{ .x = goal_x, .y = goal_y },
        obs_buf[0..obs_len],
        slices[0..num_cands],
        dt,
        safety_margin,
    );

    out.* = .{
        .best_idx = if (res.best_idx) |idx| @intCast(idx) else -1,
        .best_cost = res.best_cost,
        .safe_candidates = @intCast(res.safe_candidates),
    };
    return 0;
}

test "c_api physkernel_rollout_step selects safe candidate" {
    const obs = [_]CObstacle{.{ .x = 5.0, .y = 0.0, .radius = 1.5 }};
    const cmds = [_]CControlCmd{
        // Candidate 0: collides
        .{ .ax = 5.0, .ay = 0.0 },
        .{ .ax = 0.0, .ay = 0.0 },
        // Candidate 1: swerves safely
        .{ .ax = 5.0, .ay = 5.0 },
        .{ .ax = 5.0, .ay = -2.0 },
    };
    var out: CRolloutOut = undefined;
    const rc = physkernel_rollout_step(
        0.0, 0.0, 0.0, 0.0,
        10.0, 0.0,
        &obs, obs.len,
        &cmds, 2, 2,
        1.0, 0.2,
        &out,
    );
    try std.testing.expectEqual(@as(i32, 0), rc);
    try std.testing.expectEqual(@as(u32, 1), out.safe_candidates);
    try std.testing.expectEqual(@as(i32, 1), out.best_idx);
}
