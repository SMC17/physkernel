const std = @import("std");

/// Candidate-trajectory selection on a 2-D point mass, with no heap use.
/// Simulates each caller-supplied control sequence (semi-implicit Euler), rejects any whose
/// straight-line path between consecutive steps comes within `radius + margin` of an obstacle,
/// and returns the cheapest survivor (control effort + squared distance to goal).
/// This chooses among candidates; it does not optimise them, so it is not MPC.
pub const Vec2 = struct {
    x: f64,
    y: f64,

    pub fn distSq(a: Vec2, b: Vec2) f64 {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
};

/// Squared distance from point `c` to the segment `a`-`b`.
pub fn segmentDistSq(a: Vec2, b: Vec2, c: Vec2) f64 {
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const len2 = abx * abx + aby * aby;
    if (len2 == 0) return Vec2.distSq(a, c);
    const t = std.math.clamp(((c.x - a.x) * abx + (c.y - a.y) * aby) / len2, 0.0, 1.0);
    return Vec2.distSq(.{ .x = a.x + t * abx, .y = a.y + t * aby }, c);
}

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
            const prev = st.pos;
            st.vel.x += cmd.ax * dt;
            st.vel.y += cmd.ay * dt;
            st.pos.x += st.vel.x * dt;
            st.pos.y += st.vel.y * dt;

            for (obstacles) |obs| {
                const min_r = obs.radius + safety_margin;
                // Check the whole step, not only where it ends: a fast step can jump over an obstacle.
                if (segmentDistSq(prev, st.pos, obs.pos) < min_r * min_r) {
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

test "regression #3: a step that jumps over an obstacle is a collision" {
    const start = State2D{ .pos = .{ .x = 0, .y = 0 }, .vel = .{ .x = 0, .y = 0 } };
    const obs = [_]Obstacle{.{ .pos = .{ .x = 5, .y = 0 }, .radius = 1.5 }};
    // One step of a = 20 moves the point from x = 0 to x = 20, straight through the obstacle.
    // Both endpoints are clear, so an endpoint-only check accepts it.
    const fast = [_]ControlCmd{.{ .ax = 20.0, .ay = 0.0 }};
    const cands = [_][]const ControlCmd{&fast};
    const res = rolloutPlan(start, .{ .x = 20, .y = 0 }, &obs, &cands, 1.0, 0.0);
    try std.testing.expectEqual(@as(usize, 0), res.safe_candidates);
    try std.testing.expectEqual(@as(?usize, null), res.best_idx);
}
