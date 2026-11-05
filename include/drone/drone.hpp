#pragma once

#include <Eigen/Core>

namespace drone {

// Point-mass drone in the continuous 2D world (units: m, m/s, m/s^2).
struct Drone {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double max_speed = 2.0;  // |velocity| never exceeds this after a step
    double max_accel = 1.0;  // |applied acceleration| never exceeds this
};

// Advance the drone by dt seconds under the commanded acceleration, using
// semi-implicit Euler (velocity first, then position from the new velocity).
// The acceleration and resulting velocity are clamped to max_accel and
// max_speed by norm, preserving direction. dt <= 0 leaves the state unchanged.
void step(Drone& d, const Eigen::Vector2d& accel_cmd, double dt);

}  // namespace drone
