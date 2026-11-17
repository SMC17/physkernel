const std = @import("std");

/// Deterministic Fixed-Timestep Physical-AI World-Model & MPC Trajectory Rollout Engine.
/// Zero heap allocations: simulates `K` candidate control sequences over `H` steps,
/// checks obstacle-clearance invariants at every sub-step, and selects the lowest-cost safe plan.
pub const Vec2 = struct {
    x: f64,
    y: f64,

    pub fn distSq(a: Vec2, b: Vec2) f64 {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
};

pub const Obstacle = struct {
    pos: Vec2,
    radius: f64,
};

pub const State2D = struct {
    pos: Vec2,
    vel: Vec2,
};

pub const ControlCmd = struct {
    ax: f64,
    ay: f64,
};

pub const RolloutResult = struct {
    best_idx: ?usize,
    best_cost: f64,
    safe_candidates: usize,
};

pub fn rolloutPlan(
    init_state: State2D,
    goal: Vec2,
    obstacles: []const Obstacle,
    candidates: []const []const ControlCmd,
    dt: f64,
    safety_margin: f64,
) RolloutResult {
    var best_idx: ?usize = null;
    var best_cost: f64 = std.math.inf(f64);
    var safe_count: usize = 0;

    for (candidates, 0..) |traj, idx| {
        var st = init_state;
        var safe = true;
        var cost: f64 = 0.0;

        for (traj) |cmd| {
            st.vel.x += cmd.ax * dt;
            st.vel.y += cmd.ay * dt;
            st.pos.x += st.vel.x * dt;
            st.pos.y += st.vel.y * dt;

            for (obstacles) |obs| {
                const min_r = obs.radius + safety_margin;
                if (Vec2.distSq(st.pos, obs.pos) < min_r * min_r) {
                    safe = false;
                    break;
                }
            }
            if (!safe) break;
            cost += 0.01 * (cmd.ax * cmd.ax + cmd.ay * cmd.ay);
        }

        if (safe) {
            safe_count += 1;
            cost += Vec2.distSq(st.pos, goal);
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = idx;
            }
        }
    }

    return .{
        .best_idx = best_idx,
        .best_cost = best_cost,
        .safe_candidates = safe_count,
    };
}

test "world_sim prunes colliding trajectory and picks minimum-cost safe rollout" {
    const start = State2D{ .pos = .{ .x = 0, .y = 0 }, .vel = .{ .x = 0, .y = 0 } };
    const goal = Vec2{ .x = 10, .y = 0 };
    const obs = [_]Obstacle{.{ .pos = .{ .x = 5, .y = 0 }, .radius = 1.5 }};

    // Candidate 0: straight through obstacle at x=5, y=0 -> collides!
    const c0 = [_]ControlCmd{ .{ .ax = 5.0, .ay = 0.0 }, .{ .ax = 0.0, .ay = 0.0 } };
    // Candidate 1: swerves around y > 2 -> safe!
    const c1 = [_]ControlCmd{ .{ .ax = 5.0, .ay = 5.0 }, .{ .ax = 5.0, .ay = -2.0 } };

    const candidates = [_][]const ControlCmd{ &c0, &c1 };
    const res = rolloutPlan(start, goal, &obs, &candidates, 1.0, 0.2);

    try std.testing.expectEqual(@as(usize, 1), res.safe_candidates);
    try std.testing.expectEqual(@as(?usize, 1), res.best_idx);
}
