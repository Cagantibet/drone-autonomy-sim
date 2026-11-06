#include "drone/follower.hpp"

#include <stdexcept>

namespace drone {

int advanceWaypoint(const std::vector<Vec2>& path, int target, const Vec2& pos, double switch_radius) {
    // TODO: implement.
    (void)path;
    (void)pos;
    (void)switch_radius;
    return target;
}

FollowResult followPath(Drone drone, const std::vector<Vec2>& path, const FollowParams& params) {
    // TODO: implement.
    (void)drone;
    (void)path;
    (void)params;
    return {};
}

}  // namespace drone
