#include "drone/pid.hpp"

namespace drone {

Vec2 Pid2D::update(const Vec2& error, const Vec2& error_rate, double dt) {
    // TODO: implement.
    (void)error;
    (void)error_rate;
    (void)dt;
    return Vec2::Zero();
}

}  // namespace drone
