#include "drone/drone.hpp"

namespace drone {

void step(Drone& d, const Eigen::Vector2d& accel_cmd, double dt) {
    if (dt <= 0.0) return;

    Eigen::Vector2d accel = accel_cmd;
    const double accel_norm = accel.norm();
    if (accel_norm > d.max_accel) {
        accel *= d.max_accel / accel_norm;
    }

    // Semi-implicit Euler: update velocity first, then position.
    d.velocity += accel * dt;

    const double speed = d.velocity.norm();
    if (speed > d.max_speed) {
        d.velocity *= d.max_speed / speed;
    }

    d.position += d.velocity * dt;
}

}  // namespace drone
