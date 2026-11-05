#pragma once

#include <random>
#include <vector>

#include "drone/world2d.hpp"

namespace drone {

// Random shortcutting: for `iterations` rounds, pick two waypoints and, if the
// straight segment between them is collision-free for a disc of radius
// w.drone_radius, drop every waypoint in between. The result keeps the same
// endpoints, is a subsequence of the input, and is never longer. Paths with
// fewer than three points are returned unchanged.
std::vector<Vec2> shortcutPath(const World2D& w, const std::vector<Vec2>& path, int iterations,
                               std::mt19937& rng);

}  // namespace drone
