#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include "drone/collision.hpp"
#include "drone/rrt.hpp"

using drone::RrtParams;
using drone::RrtResult;
using drone::RrtTree;
using drone::Vec2;
using drone::World2D;

namespace {

constexpr double kEps = 1e-9;

World2D emptyWorld() {
    World2D w;
    w.width = 10.0;
    w.height = 10.0;
    w.drone_radius = 0.2;
    w.start = {1, 1};
    w.goal = {9, 9};
    return w;
}

// A wall across the middle with a single gap near the top.
World2D wallWorld() {
    World2D w = emptyWorld();
    w.rects.push_back({Vec2(4.5, 0), Vec2(5.5, 7.5)});
    w.rects.push_back({Vec2(4.5, 8.8), Vec2(5.5, 10)});
    w.start = {1, 1};
    w.goal = {9, 1};
    return w;
}

World2D shippedWorld(const char* name) { return drone::loadWorld2D(std::string(DRONE_MAPS_DIR) + "/" + name); }

RrtResult run(const World2D& w, const RrtParams& p, unsigned seed) {
    std::mt19937 rng(seed);
    return drone::rrt(w, p, rng);
}

// Every property a returned path and tree must satisfy, regardless of the
// random samples drawn.
void expectValidResult(const World2D& w, const RrtParams& p, const RrtResult& r) {
    ASSERT_GE(r.path.size(), 2u);
    EXPECT_TRUE(r.path.front().isApprox(w.start));
    EXPECT_TRUE(r.path.back().isApprox(w.goal));
    for (size_t i = 1; i < r.path.size(); ++i)
        EXPECT_FALSE(drone::segmentCollides(w, r.path[i - 1], r.path[i])) << "path edge " << i;
    EXPECT_GE(drone::pathLength(r.path), (w.goal - w.start).norm() - kEps);

    const RrtTree& t = r.tree;
    ASSERT_EQ(t.nodes.size(), t.parent.size());
    ASSERT_GE(t.size(), 2);
    EXPECT_TRUE(t.nodes[0].isApprox(w.start));
    EXPECT_EQ(t.parent[0], -1);
    const double maxEdge = std::max(p.step, p.goal_tol) + kEps;
    for (int i = 1; i < t.size(); ++i) {
        ASSERT_GE(t.parent[i], 0) << "node " << i;
        ASSERT_LT(t.parent[i], i) << "parent must be inserted before child, node " << i;
        const Vec2& a = t.nodes[t.parent[i]];
        EXPECT_LE((t.nodes[i] - a).norm(), maxEdge) << "node " << i;
        EXPECT_FALSE(drone::segmentCollides(w, a, t.nodes[i])) << "tree edge into node " << i;
    }
    EXPECT_GT(r.iterations, 0);
    EXPECT_LE(r.iterations, p.max_iters);
}

}  // namespace

// ---- helpers (implemented, always on) ----

TEST(RrtHelpers, NearestIndex) {
    RrtTree t;
    t.addNode({0, 0}, -1);
    t.addNode({5, 0}, 0);
    t.addNode({5, 5}, 1);
    EXPECT_EQ(drone::nearestIndex(t, {0.4, 0.1}), 0);
    EXPECT_EQ(drone::nearestIndex(t, {4, 1}), 1);
    EXPECT_EQ(drone::nearestIndex(t, {6, 6}), 2);
}

TEST(RrtHelpers, SteerCapsAtStep) {
    EXPECT_TRUE(drone::steer({0, 0}, {3, 4}, 1.0).isApprox(Vec2(0.6, 0.8)));
    EXPECT_TRUE(drone::steer({1, 1}, {1.2, 1}, 1.0).isApprox(Vec2(1.2, 1)));  // closer than step
    EXPECT_TRUE(drone::steer({1, 1}, {1, 1}, 1.0).isApprox(Vec2(1, 1)));      // same point
}

TEST(RrtHelpers, ExtractPathWalksParents) {
    RrtTree t;
    t.addNode({0, 0}, -1);  // 0
    t.addNode({1, 0}, 0);   // 1
    t.addNode({0, 1}, 0);   // 2 (other branch)
    t.addNode({2, 0}, 1);   // 3
    const auto path = drone::extractPath(t, 3);
    ASSERT_EQ(path.size(), 3u);
    EXPECT_TRUE(path[0].isApprox(Vec2(0, 0)));
    EXPECT_TRUE(path[1].isApprox(Vec2(1, 0)));
    EXPECT_TRUE(path[2].isApprox(Vec2(2, 0)));
    EXPECT_EQ(drone::extractPath(t, 0).size(), 1u);
}

TEST(RrtHelpers, PathLength) {
    EXPECT_DOUBLE_EQ(drone::pathLength({}), 0.0);
    EXPECT_DOUBLE_EQ(drone::pathLength({Vec2(1, 1)}), 0.0);
    EXPECT_DOUBLE_EQ(drone::pathLength({Vec2(0, 0), Vec2(3, 4), Vec2(3, 6)}), 7.0);
}

// ---- planner (remove DISABLED_ once rrt() is implemented) ----

TEST(DISABLED_Rrt, FindsPathInEmptyWorld) {
    const World2D w = emptyWorld();
    const RrtParams p;
    expectValidResult(w, p, run(w, p, 1));
}

TEST(DISABLED_Rrt, FindsPathThroughGapForManySeeds) {
    const World2D w = wallWorld();
    RrtParams p;
    p.max_iters = 20000;
    for (unsigned seed = 0; seed < 10; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        expectValidResult(w, p, run(w, p, seed));
    }
}

TEST(DISABLED_Rrt, SolvesShippedMaps) {
    RrtParams p;
    p.max_iters = 20000;
    for (const char* name : {"forest.world", "rooms.world"}) {
        SCOPED_TRACE(name);
        const World2D w = shippedWorld(name);
        int solved = 0;
        for (unsigned seed = 0; seed < 5; ++seed) {
            const RrtResult r = run(w, p, seed);
            if (r.path.empty()) continue;
            ++solved;
            expectValidResult(w, p, r);
        }
        EXPECT_GE(solved, 4);
    }
}

TEST(DISABLED_Rrt, SameSeedGivesSameTree) {
    const World2D w = wallWorld();
    const RrtParams p;
    const RrtResult a = run(w, p, 42);
    const RrtResult b = run(w, p, 42);
    ASSERT_EQ(a.tree.size(), b.tree.size());
    for (int i = 0; i < a.tree.size(); ++i) {
        EXPECT_EQ(a.tree.nodes[i], b.tree.nodes[i]);
        EXPECT_EQ(a.tree.parent[i], b.tree.parent[i]);
    }
    EXPECT_EQ(a.path.size(), b.path.size());
}

TEST(DISABLED_Rrt, StartNearGoalConnectsDirectly) {
    World2D w = emptyWorld();
    w.goal = w.start + Vec2(0.3, 0.0);  // within goal_tol
    const RrtParams p;
    const RrtResult r = run(w, p, 0);
    expectValidResult(w, p, r);
    EXPECT_LE(drone::pathLength(r.path), p.step + p.goal_tol + kEps);
}

TEST(DISABLED_Rrt, UnreachableGoalReturnsEmptyPath) {
    World2D w = emptyWorld();
    // Goal boxed in by four walls.
    w.rects = {{Vec2(7, 7), Vec2(9.8, 7.3)}, {Vec2(7, 9.5), Vec2(9.8, 9.8)},
               {Vec2(7, 7), Vec2(7.3, 9.8)}, {Vec2(9.5, 7), Vec2(9.8, 9.8)}};
    w.goal = {8.4, 8.4};
    RrtParams p;
    p.max_iters = 2000;
    const RrtResult r = run(w, p, 3);
    EXPECT_TRUE(r.path.empty());
    EXPECT_EQ(r.iterations, p.max_iters);
    EXPECT_GT(r.tree.size(), 1);  // tree still grew and is returned for plotting
}

TEST(DISABLED_Rrt, GoalOrStartInCollisionReturnsEmptyPath) {
    World2D w = emptyWorld();
    w.circles.push_back({w.goal, 0.5});
    EXPECT_TRUE(run(w, RrtParams{}, 0).path.empty());

    w = emptyWorld();
    w.circles.push_back({w.start, 0.5});
    EXPECT_TRUE(run(w, RrtParams{}, 0).path.empty());
}
