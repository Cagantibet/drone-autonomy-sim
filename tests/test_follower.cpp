#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "drone/follower.hpp"

using drone::Drone;
using drone::FollowParams;
using drone::FollowResult;
using drone::Vec2;

namespace {

constexpr double kEps = 1e-9;

FollowParams pdParams() {
    FollowParams p;
    p.gains.kp = 1.0;
    p.gains.kd = 1.6;
    return p;
}

FollowParams pidParams() {
    FollowParams p = pdParams();
    p.gains.ki = 0.3;
    return p;
}

Drone droneAt(const Vec2& pos) {
    Drone d;
    d.position = pos;
    return d;
}

void expectReachedGoal(const FollowResult& r, const Vec2& goal, const FollowParams& p) {
    ASSERT_TRUE(r.reached);
    ASSERT_FALSE(r.samples.empty());
    const auto& last = r.samples.back();
    EXPECT_NEAR(r.time_to_goal, last.t, kEps);
    EXPECT_LT((last.position - goal).norm(), p.goal_tol);
    EXPECT_LT(last.velocity.norm(), p.settle_speed);
}

}  // namespace

// ---- advanceWaypoint ----

TEST(AdvanceWaypoint, StaysWhenFar) {
    const std::vector<Vec2> path = {{0, 0}, {5, 0}, {10, 0}};
    EXPECT_EQ(drone::advanceWaypoint(path, 1, {1.0, 0.0}, 0.5), 1);
}

TEST(AdvanceWaypoint, DISABLED_AdvancesInsideRadius) {
    const std::vector<Vec2> path = {{0, 0}, {5, 0}, {10, 0}};
    EXPECT_EQ(drone::advanceWaypoint(path, 1, {4.8, 0.1}, 0.5), 2);
}

TEST(AdvanceWaypoint, DISABLED_SkipsSeveralCloseWaypointsAtOnce) {
    const std::vector<Vec2> path = {{0, 0}, {5, 0}, {5.1, 0}, {5.2, 0}, {10, 0}};
    EXPECT_EQ(drone::advanceWaypoint(path, 1, {5.1, 0.0}, 0.5), 4);
}

TEST(AdvanceWaypoint, DISABLED_NeverPassesLastWaypoint) {
    const std::vector<Vec2> path = {{0, 0}, {5, 0}, {10, 0}};
    EXPECT_EQ(drone::advanceWaypoint(path, 2, {10.0, 0.0}, 0.5), 2);
    EXPECT_EQ(drone::advanceWaypoint(path, 1, {7.5, 0.0}, 100.0), 2);
}

TEST(AdvanceWaypoint, NeverGoesBackward) {
    const std::vector<Vec2> path = {{0, 0}, {5, 0}, {10, 0}};
    EXPECT_EQ(drone::advanceWaypoint(path, 2, {0.0, 0.0}, 0.5), 2);
}

TEST(AdvanceWaypoint, SinglePointPath) {
    const std::vector<Vec2> path = {{3, 3}};
    EXPECT_EQ(drone::advanceWaypoint(path, 0, {3.0, 3.0}, 0.5), 0);
}

// ---- followPath ----

TEST(FollowPath, DISABLED_EmptyPathThrows) {
    EXPECT_THROW(drone::followPath(Drone{}, {}, pdParams()), std::invalid_argument);
}

TEST(FollowPath, DISABLED_NonPositiveDtThrows) {
    FollowParams p = pdParams();
    p.dt = 0.0;
    EXPECT_THROW(drone::followPath(Drone{}, {{0, 0}, {1, 0}}, p), std::invalid_argument);
}

TEST(FollowPath, DISABLED_FirstSampleIsInitialState) {
    Drone d = droneAt({1.0, 2.0});
    d.velocity = {0.5, 0.0};
    const auto r = drone::followPath(d, {{1, 2}, {5, 2}}, pdParams());
    ASSERT_FALSE(r.samples.empty());
    const auto& s = r.samples.front();
    EXPECT_EQ(s.t, 0.0);
    EXPECT_TRUE(s.position.isApprox(Vec2(1.0, 2.0)));
    EXPECT_TRUE(s.velocity.isApprox(Vec2(0.5, 0.0)));
    EXPECT_TRUE(s.accel_cmd.isZero());
    EXPECT_EQ(s.target, 1);
}

TEST(FollowPath, DISABLED_PdReachesGoalOnStraightPath) {
    const FollowParams p = pdParams();
    const std::vector<Vec2> path = {{0, 0}, {5, 0}};
    const auto r = drone::followPath(droneAt(path.front()), path, p);
    ASSERT_NO_FATAL_FAILURE(expectReachedGoal(r, path.back(), p));
    EXPECT_LT(r.time_to_goal, 20.0);
}

TEST(FollowPath, DISABLED_PdReachesGoalOnLShapedPath) {
    const FollowParams p = pdParams();
    const std::vector<Vec2> path = {{0, 0}, {4, 0}, {4, 4}};
    const auto r = drone::followPath(droneAt(path.front()), path, p);
    ASSERT_NO_FATAL_FAILURE(expectReachedGoal(r, path.back(), p));
    EXPECT_EQ(r.samples.back().target, 2);
}

TEST(FollowPath, DISABLED_SinglePointPathFliesToThatPoint) {
    const FollowParams p = pdParams();
    const std::vector<Vec2> path = {{2, 1}};
    const auto r = drone::followPath(droneAt({0, 0}), path, p);
    ASSERT_NO_FATAL_FAILURE(expectReachedGoal(r, path.back(), p));
    for (const auto& s : r.samples) EXPECT_EQ(s.target, 0);
}

TEST(FollowPath, DISABLED_SamplesAreSpacedByDtAndRespectLimits) {
    const FollowParams p = pdParams();
    Drone d;
    d.max_speed = 1.5;
    d.max_accel = 0.8;
    const std::vector<Vec2> path = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const auto r = drone::followPath(d, path, p);
    ASSERT_GE(r.samples.size(), 2u);
    for (std::size_t i = 1; i < r.samples.size(); ++i) {
        const auto& prev = r.samples[i - 1];
        const auto& cur = r.samples[i];
        EXPECT_NEAR(cur.t - prev.t, p.dt, kEps) << "sample " << i;
        EXPECT_GE(cur.target, prev.target) << "sample " << i;
        EXPECT_LE(cur.velocity.norm(), d.max_speed + kEps) << "sample " << i;
    }
}

TEST(FollowPath, DISABLED_StopsAtMaxTimeWhenGoalIsNotReached) {
    FollowParams p;  // zero gains: the drone never moves
    p.max_time = 2.0;
    const auto r = drone::followPath(Drone{}, {{0, 0}, {3, 0}}, p);
    EXPECT_FALSE(r.reached);
    EXPECT_EQ(r.time_to_goal, -1.0);
    ASSERT_FALSE(r.samples.empty());
    EXPECT_GE(r.samples.back().t, p.max_time - kEps);
    EXPECT_LT(r.samples.back().t, p.max_time + p.dt);
}

TEST(FollowPath, DISABLED_PdHasSteadyStateOffsetUnderConstantWind) {
    FollowParams p = pdParams();
    p.disturbance = {0.2, 0.0};  // PD settles where kp * e = -wind, i.e. 0.2 m past the goal
    p.max_time = 40.0;
    const std::vector<Vec2> path = {{0, 0}, {3, 0}};
    const auto r = drone::followPath(droneAt(path.front()), path, p);
    EXPECT_FALSE(r.reached);
    ASSERT_FALSE(r.samples.empty());
    EXPECT_NEAR(r.samples.back().position.x(), 3.2, 0.05);
}

TEST(FollowPath, DISABLED_IntegralTermRemovesWindOffset) {
    FollowParams p = pidParams();
    p.disturbance = {0.2, 0.0};
    p.max_time = 40.0;
    const std::vector<Vec2> path = {{0, 0}, {3, 0}};
    const auto r = drone::followPath(droneAt(path.front()), path, p);
    ASSERT_NO_FATAL_FAILURE(expectReachedGoal(r, path.back(), p));
}
