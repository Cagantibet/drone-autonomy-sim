#pragma once

#include <vector>

#include "drone/follower.hpp"
#include "drone/world2d.hpp"

namespace drone {

struct TrackingMetrics {
    bool reached = false;
    bool collided = false;          // any flown segment between samples hits the world
    double time_to_goal = -1.0;     // s, -1 if the goal was never reached
    double mean_error = 0.0;        // cross-track error over all samples (m)
    double rms_error = 0.0;
    double max_error = 0.0;
    double overshoot = 0.0;         // furthest distance past the goal along the final segment (m)
    double distance_flown = 0.0;    // m
};

// Distance from p to the nearest point on the polyline (to the point itself for
// a one-point path). Throws std::invalid_argument on an empty path.
double crossTrackError(const std::vector<Vec2>& path, const Vec2& p);

// Summarize a run of followPath against the planned path. Overshoot is measured
// along the direction of the last path segment (or from the first sample toward
// the goal for a one-point path) and is 0 if the drone never passes the goal.
// Collisions use w.drone_radius. Throws std::invalid_argument on an empty path.
TrackingMetrics computeMetrics(const World2D& w, const std::vector<Vec2>& path, const FollowResult& run);

}  // namespace drone
