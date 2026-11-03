#pragma once

#include <Eigen/Core>

namespace drone {

// Point-mass drone state in the 2D world (units: grid cells, cells/s).
struct Drone {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
};

// Advance position by velocity * dt (explicit Euler). Placeholder dynamics
// until the controller milestone.
void step(Drone& d, double dt);

}  // namespace drone
