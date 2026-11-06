#pragma once

#include "drone/world2d.hpp"

namespace drone {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 2.0;  // max norm of the integral state (m*s), for anti-windup
};

// PID controller on a 2D error, applied independently per axis with shared gains.
// The derivative is passed in rather than differenced from consecutive errors, so
// a jump in the target (e.g. a waypoint switch) doesn't produce a derivative spike.
class Pid2D {
public:
    explicit Pid2D(const PidGains& gains) : gains_(gains) {}

    // error = target - position, error_rate = d(error)/dt (-velocity for a fixed target).
    // Adds error * dt to the integral, then scales the integral back to
    // integral_limit if its norm exceeds it. dt <= 0 skips the integration.
    // Returns kp * error + ki * integral + kd * error_rate.
    Vec2 update(const Vec2& error, const Vec2& error_rate, double dt);

    void reset() { integral_.setZero(); }

    const Vec2& integral() const { return integral_; }
    const PidGains& gains() const { return gains_; }

private:
    PidGains gains_;
    Vec2 integral_ = Vec2::Zero();
};

}  // namespace drone
