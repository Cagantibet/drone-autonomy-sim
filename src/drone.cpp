#include "drone/drone.hpp"

namespace drone {

void step(Drone& d, double dt) { d.position += d.velocity * dt; }

}  // namespace drone
