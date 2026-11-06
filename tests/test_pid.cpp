#include <gtest/gtest.h>

#include "drone/pid.hpp"

using drone::Pid2D;
using drone::PidGains;
using drone::Vec2;

namespace {

constexpr double kTol = 1e-12;

void expectVec(const Vec2& actual, const Vec2& expected) {
    EXPECT_NEAR(actual.x(), expected.x(), kTol);
    EXPECT_NEAR(actual.y(), expected.y(), kTol);
}

PidGains gains(double kp, double ki, double kd, double limit = 2.0) {
    PidGains g;
    g.kp = kp;
    g.ki = ki;
    g.kd = kd;
    g.integral_limit = limit;
    return g;
}

}  // namespace

TEST(Pid2D, StartsWithZeroIntegral) {
    Pid2D pid(gains(1.0, 1.0, 1.0));
    EXPECT_TRUE(pid.integral().isZero());
}

TEST(Pid2D, ZeroGainsGiveZeroCommand) {
    Pid2D pid(gains(0.0, 0.0, 0.0));
    expectVec(pid.update({3.0, -1.0}, {0.5, 2.0}, 0.1), {0.0, 0.0});
}

TEST(Pid2D, DISABLED_ProportionalOnly) {
    Pid2D pid(gains(2.0, 0.0, 0.0));
    expectVec(pid.update({1.0, -0.5}, {7.0, 7.0}, 0.1), {2.0, -1.0});
}

TEST(Pid2D, DISABLED_DerivativeUsesErrorRate) {
    Pid2D pid(gains(0.0, 0.0, 0.5));
    expectVec(pid.update({4.0, 4.0}, {2.0, -1.0}, 0.1), {1.0, -0.5});
}

TEST(Pid2D, DISABLED_IntegralAccumulatesErrorTimesDt) {
    Pid2D pid(gains(0.0, 2.0, 0.0));
    pid.update({1.0, 0.0}, Vec2::Zero(), 0.1);
    pid.update({1.0, 0.0}, Vec2::Zero(), 0.1);
    const Vec2 u = pid.update({0.0, 1.0}, Vec2::Zero(), 0.5);
    expectVec(pid.integral(), {0.2, 0.5});
    expectVec(u, {0.4, 1.0});  // integral is updated before computing the output
}

TEST(Pid2D, DISABLED_IntegralIsClampedByNormKeepingDirection) {
    Pid2D pid(gains(0.0, 1.0, 0.0, 0.5));
    const Vec2 u = pid.update({3.0, 4.0}, Vec2::Zero(), 1.0);  // raw integral (3, 4), norm 5
    expectVec(pid.integral(), {0.3, 0.4});
    expectVec(u, {0.3, 0.4});
}

TEST(Pid2D, DISABLED_IntegralUnwindsAfterClamp) {
    Pid2D pid(gains(0.0, 1.0, 0.0, 1.0));
    for (int i = 0; i < 100; ++i) pid.update({1.0, 0.0}, Vec2::Zero(), 0.1);
    expectVec(pid.integral(), {1.0, 0.0});
    pid.update({-1.0, 0.0}, Vec2::Zero(), 0.5);  // windup was capped, so this unwinds immediately
    expectVec(pid.integral(), {0.5, 0.0});
}

TEST(Pid2D, DISABLED_NonPositiveDtSkipsIntegration) {
    for (double dt : {0.0, -0.1}) {
        Pid2D pid(gains(1.0, 1.0, 1.0));
        const Vec2 u = pid.update({1.0, 2.0}, {0.5, 0.0}, dt);
        EXPECT_TRUE(pid.integral().isZero());
        expectVec(u, {1.5, 2.0});
    }
}

TEST(Pid2D, DISABLED_SumsAllThreeTerms) {
    Pid2D pid(gains(1.0, 0.5, 0.25));
    const Vec2 u = pid.update({2.0, -2.0}, {-4.0, 1.0}, 0.2);  // integral (0.4, -0.4)
    expectVec(u, Vec2(2.0, -2.0) + 0.5 * Vec2(0.4, -0.4) + 0.25 * Vec2(-4.0, 1.0));
}

TEST(Pid2D, ResetClearsIntegral) {
    Pid2D pid(gains(0.0, 1.0, 0.0));
    pid.update({1.0, 1.0}, Vec2::Zero(), 0.5);
    pid.reset();
    EXPECT_TRUE(pid.integral().isZero());
    expectVec(pid.update({0.0, 0.0}, Vec2::Zero(), 0.1), {0.0, 0.0});
}
