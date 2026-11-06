#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "drone/tracking_metrics.hpp"

using drone::FollowResult;
using drone::Vec2;
using drone::World2D;

namespace {

constexpr double kTol = 1e-12;

World2D emptyWorld() {
    World2D w;
    w.width = 10.0;
    w.height = 10.0;
    w.drone_radius = 0.2;
    return w;
}

FollowResult runThrough(const std::vector<Vec2>& positions, double dt = 0.1) {
    FollowResult r;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        drone::TrajectorySample s;
        s.t = static_cast<double>(i) * dt;
        s.position = positions[i];
        r.samples.push_back(s);
    }
    return r;
}

}  // namespace

TEST(CrossTrackError, EmptyPathThrows) {
    EXPECT_THROW(drone::crossTrackError({}, {0, 0}), std::invalid_argument);
}

TEST(CrossTrackError, SinglePointIsDistanceToIt) {
    EXPECT_NEAR(drone::crossTrackError({{1, 1}}, {4, 5}), 5.0, kTol);
}

TEST(CrossTrackError, ZeroOnThePath) {
    const std::vector<Vec2> path = {{0, 0}, {4, 0}, {4, 4}};
    EXPECT_NEAR(drone::crossTrackError(path, {2, 0}), 0.0, kTol);
    EXPECT_NEAR(drone::crossTrackError(path, {4, 3}), 0.0, kTol);
}

TEST(CrossTrackError, UsesNearestSegment) {
    const std::vector<Vec2> path = {{0, 0}, {4, 0}, {4, 4}};
    EXPECT_NEAR(drone::crossTrackError(path, {2, 0.5}), 0.5, kTol);
    EXPECT_NEAR(drone::crossTrackError(path, {3.5, 3}), 0.5, kTol);  // closer to 2nd segment
    EXPECT_NEAR(drone::crossTrackError(path, {5, -1}), std::sqrt(2.0), kTol);  // corner
}

TEST(ComputeMetrics, EmptyPathThrows) {
    EXPECT_THROW(drone::computeMetrics(emptyWorld(), {}, FollowResult{}), std::invalid_argument);
}

TEST(ComputeMetrics, NoSamplesGivesZeros) {
    FollowResult r;
    const auto m = drone::computeMetrics(emptyWorld(), {{1, 1}, {5, 1}}, r);
    EXPECT_FALSE(m.reached);
    EXPECT_FALSE(m.collided);
    EXPECT_EQ(m.max_error, 0.0);
    EXPECT_EQ(m.distance_flown, 0.0);
}

TEST(ComputeMetrics, CopiesReachedAndTime) {
    FollowResult r = runThrough({{1, 1}, {2, 1}});
    r.reached = true;
    r.time_to_goal = 0.1;
    const auto m = drone::computeMetrics(emptyWorld(), {{1, 1}, {2, 1}}, r);
    EXPECT_TRUE(m.reached);
    EXPECT_EQ(m.time_to_goal, 0.1);
}

TEST(ComputeMetrics, ErrorStatistics) {
    const std::vector<Vec2> path = {{1, 1}, {9, 1}};
    const auto r = runThrough({{1, 1}, {3, 1.3}, {5, 0.6}, {7, 1}});  // errors 0, 0.3, 0.4, 0
    const auto m = drone::computeMetrics(emptyWorld(), path, r);
    EXPECT_NEAR(m.mean_error, 0.7 / 4.0, 1e-12);
    EXPECT_NEAR(m.rms_error, std::sqrt((0.09 + 0.16) / 4.0), 1e-12);
    EXPECT_NEAR(m.max_error, 0.4, 1e-12);
}

TEST(ComputeMetrics, DistanceFlown) {
    const auto r = runThrough({{1, 1}, {4, 5}, {4, 7}});
    const auto m = drone::computeMetrics(emptyWorld(), {{1, 1}, {4, 7}}, r);
    EXPECT_NEAR(m.distance_flown, 7.0, kTol);
}

TEST(ComputeMetrics, NoOvershootWhenStoppingShort) {
    const auto r = runThrough({{1, 1}, {3, 1}, {4.9, 1}});
    const auto m = drone::computeMetrics(emptyWorld(), {{1, 1}, {5, 1}}, r);
    EXPECT_EQ(m.overshoot, 0.0);
}

TEST(ComputeMetrics, OvershootAlongFinalSegment) {
    // Final segment points +y; going 0.4 past the goal and drifting sideways counts only along y.
    const std::vector<Vec2> path = {{1, 1}, {5, 1}, {5, 5}};
    const auto r = runThrough({{5, 4}, {5.3, 5.4}, {5.1, 5.2}, {5, 5}});
    const auto m = drone::computeMetrics(emptyWorld(), path, r);
    EXPECT_NEAR(m.overshoot, 0.4, 1e-12);
}

TEST(ComputeMetrics, OvershootForSinglePointPathUsesApproachDirection) {
    const auto r = runThrough({{1, 1}, {3.5, 1}, {3, 1}});
    const auto m = drone::computeMetrics(emptyWorld(), {{3, 1}}, r);
    EXPECT_NEAR(m.overshoot, 0.5, 1e-12);
}

TEST(ComputeMetrics, DetectsCollisionBetweenSamples) {
    World2D w = emptyWorld();
    w.rects.push_back({{4.9, 0.0}, {5.1, 10.0}});  // thin wall both samples are clear of
    const auto r = runThrough({{3, 5}, {7, 5}});
    EXPECT_TRUE(drone::computeMetrics(w, {{3, 5}, {7, 5}}, r).collided);
    EXPECT_FALSE(drone::computeMetrics(emptyWorld(), {{3, 5}, {7, 5}}, r).collided);
}

TEST(ComputeMetrics, DetectsCollisionForSingleSample) {
    World2D w = emptyWorld();
    w.circles.push_back({{5, 5}, 1.0});
    const auto r = runThrough({{5, 5}});
    EXPECT_TRUE(drone::computeMetrics(w, {{5, 5}}, r).collided);
}
