#include "drone/drone.hpp"

namespace drone {

void step(Drone& d, const Eigen::Vector2d& accel_cmd, double dt) {
    // TODO(you): semi-implicit Euler with accel and speed clamping (see drone.hpp).
    (void)d;
    (void)accel_cmd;
    (void)dt;
}

}  // namespace drone
