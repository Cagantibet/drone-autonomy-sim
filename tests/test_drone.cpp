#include <gtest/gtest.h>

#include <cmath>

#include "drone/drone.hpp"

using drone::Drone;
using Eigen::Vector2d;

namespace {

constexpr double kTol = 1e-12;

void expectVec(const Vector2d& actual, const Vector2d& expected) {
    EXPECT_NEAR(actual.x(), expected.x(), kTol);
    EXPECT_NEAR(actual.y(), expected.y(), kTol);
}

}  // namespace

TEST(Drone, DefaultState) {
    Drone d;
    EXPECT_TRUE(d.position.isZero());
    EXPECT_TRUE(d.velocity.isZero());
    EXPECT_GT(d.max_speed, 0.0);
    EXPECT_GT(d.max_accel, 0.0);
}

// TODO: remove the DISABLED_ prefix once step() is implemented.

TEST(DISABLED_DroneStep, ZeroCommandKeepsConstantVelocity) {
    Drone d;
    d.velocity = {1.0, 0.0};
    drone::step(d, Vector2d::Zero(), 0.5);
    expectVec(d.velocity, {1.0, 0.0});
    expectVec(d.position, {0.5, 0.0});
}

TEST(DISABLED_DroneStep, CommandWithinLimitIsAppliedAsIs) {
    Drone d;  // max_accel = 1
    drone::step(d, {0.5, 0.0}, 1.0);
    expectVec(d.velocity, {0.5, 0.0});
}

TEST(DISABLED_DroneStep, UsesNewVelocityForPosition) {
    Drone d;
    d.max_accel = 2.0;
    d.max_speed = 10.0;
    d.velocity = {1.0, 0.0};
    drone::step(d, {1.0, 0.0}, 0.5);
    expectVec(d.velocity, {1.5, 0.0});
    expectVec(d.position, {0.75, 0.0});  // plain Euler would give 0.5
}

TEST(DISABLED_DroneStep, ClampsAccelerationKeepingDirection) {
    Drone d;
    d.max_accel = 1.0;
    d.max_speed = 10.0;
    drone::step(d, {3.0, 4.0}, 1.0);  // |cmd| = 5 -> scaled to (0.6, 0.8)
    expectVec(d.velocity, {0.6, 0.8});
    expectVec(d.position, {0.6, 0.8});
}

TEST(DISABLED_DroneStep, ClampsSpeedKeepingDirection) {
    Drone d;
    d.max_speed = 2.0;
    d.max_accel = 1.0;
    d.velocity = {2.0, 0.0};
    drone::step(d, {0.0, 1.0}, 1.0);  // v = (2, 1), |v| = sqrt(5) -> scaled to 2
    const Vector2d expected = Vector2d(2.0, 1.0) * (2.0 / std::sqrt(5.0));
    expectVec(d.velocity, expected);
    expectVec(d.position, expected);
}

TEST(DISABLED_DroneStep, NonPositiveDtIsANoOp) {
    for (double dt : {0.0, -0.1}) {
        Drone d;
        d.position = {1.0, 2.0};
        d.velocity = {0.5, 0.5};
        drone::step(d, {1.0, 1.0}, dt);
        expectVec(d.position, {1.0, 2.0});
        expectVec(d.velocity, {0.5, 0.5});
    }
}

TEST(DISABLED_DroneStep, LongRunReachesButNeverExceedsMaxSpeed) {
    Drone d;  // max_speed = 2, max_accel = 1
    double prevX = d.position.x();
    for (int i = 0; i < 100; ++i) {
        drone::step(d, {5.0, 5.0}, 0.1);
        EXPECT_LE(d.velocity.norm(), d.max_speed + kTol) << "step " << i;
        EXPECT_GT(d.position.x(), prevX) << "step " << i;
        prevX = d.position.x();
    }
    EXPECT_NEAR(d.velocity.norm(), d.max_speed, kTol);
    // Direction stays along the command: (1, 1) / sqrt(2).
    EXPECT_NEAR(d.velocity.x(), d.velocity.y(), kTol);
}
