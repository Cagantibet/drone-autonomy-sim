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

// Advance the drone by dt seconds under the commanded acceleration.
//
// Approach (semi-implicit Euler):
//  1. a = accel_cmd, clamped to |a| <= max_accel. Clamp by scaling the
//     vector (a *= max_accel / |a|) so the direction is kept.
//  2. velocity += a * dt, then clamp |velocity| <= max_speed the same way.
//  3. position += velocity * dt, using the NEW velocity. This is what makes
//     it semi-implicit (more stable than plain Euler for controllers later).
//  Guard: if dt <= 0, leave the state unchanged.
//  Eigen: v.norm(), v *= s. No need to special-case a zero vector if you only
//  scale when the norm exceeds the limit.
void step(Drone& d, const Eigen::Vector2d& accel_cmd, double dt);

}  // namespace drone
