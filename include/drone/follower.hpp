#pragma once

#include <vector>

#include "drone/drone.hpp"
#include "drone/pid.hpp"
#include "drone/world2d.hpp"

namespace drone {

struct FollowParams {
    PidGains gains;
    double dt = 0.05;                 // control and physics step (s)
    double switch_radius = 0.5;       // move on to the next waypoint inside this distance (m)
    double goal_tol = 0.1;            // reached: within goal_tol of the last waypoint (m)...
    double settle_speed = 0.1;        // ...and slower than this (m/s)
    double max_time = 60.0;           // give up after this much simulated time (s)
    Vec2 disturbance = Vec2::Zero();  // constant acceleration (e.g. wind) added to the command (m/s^2)
};

// State at time t, plus the command applied over [t - dt, t] and the waypoint
// index that was being tracked. The first sample is the initial state at t = 0
// with a zero command.
struct TrajectorySample {
    double t = 0.0;
    Vec2 position = Vec2::Zero();
    Vec2 velocity = Vec2::Zero();
    Vec2 accel_cmd = Vec2::Zero();
    int target = 0;
};

struct FollowResult {
    std::vector<TrajectorySample> samples;
    bool reached = false;
    double time_to_goal = -1.0;  // t of the first sample that met the goal condition, -1 if never
};

// Returns the smallest index >= target whose waypoint is at least switch_radius
// from pos, stopping at the last waypoint (which is never skipped).
int advanceWaypoint(const std::vector<Vec2>& path, int target, const Vec2& pos, double switch_radius);

// Fly `drone` along `path` with a PID controller on the error to the current
// waypoint, stepping the physics every dt. The first waypoint to track is
// path[1] (path[0] is taken to be the start), or path[0] for a one-point path.
// After each step the goal is reached when the target is the last waypoint,
// the drone is within goal_tol of it and slower than settle_speed; the run
// stops there, or once t reaches max_time. Throws std::invalid_argument on an
// empty path or dt <= 0.
FollowResult followPath(Drone drone, const std::vector<Vec2>& path, const FollowParams& params);

}  // namespace drone
