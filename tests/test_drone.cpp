#include <gtest/gtest.h>

#include "drone/drone.hpp"

using drone::Drone;

TEST(Drone, DefaultsToZeroState) {
    Drone d;
    EXPECT_TRUE(d.position.isZero());
    EXPECT_TRUE(d.velocity.isZero());
}

TEST(Drone, StepIntegratesVelocity) {
    Drone d;
    d.position = {1.0, 2.0};
    d.velocity = {0.5, -1.0};
    drone::step(d, 2.0);
    EXPECT_DOUBLE_EQ(d.position.x(), 2.0);
    EXPECT_DOUBLE_EQ(d.position.y(), 0.0);
}
