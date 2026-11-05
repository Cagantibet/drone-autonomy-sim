#include <gtest/gtest.h>

#include <random>

#include "drone/collision.hpp"
#include "drone/rrt.hpp"
#include "drone/smoothing.hpp"

using drone::Vec2;
using drone::World2D;

namespace {

constexpr unsigned kSeed = 7;
constexpr double kEps = 1e-9;

World2D emptyWorld() {
    World2D w;
    w.width = 10.0;
    w.height = 10.0;
    w.drone_radius = 0.2;
    return w;
}

std::vector<Vec2> smooth(const World2D& w, const std::vector<Vec2>& path, int iterations = 200,
                         unsigned seed = kSeed) {
    std::mt19937 rng(seed);
    return drone::shortcutPath(w, path, iterations, rng);
}

// Output must keep the endpoints, only drop points (preserving order), stay
// collision-free, and never get longer.
void expectValidShortcut(const World2D& w, const std::vector<Vec2>& in, const std::vector<Vec2>& out) {
    ASSERT_GE(out.size(), 2u);
    ASSERT_LE(out.size(), in.size());
    EXPECT_EQ(out.front(), in.front());
    EXPECT_EQ(out.back(), in.back());

    size_t j = 0;
    for (const Vec2& p : out) {
        while (j < in.size() && in[j] != p) ++j;
        ASSERT_LT(j, in.size()) << "output point is not from the input, or out of order";
        ++j;
    }
    for (size_t i = 1; i < out.size(); ++i)
        EXPECT_FALSE(drone::segmentCollides(w, out[i - 1], out[i])) << "segment " << i;
    EXPECT_LE(drone::pathLength(out), drone::pathLength(in) + kEps);
}

// Zig-zag from (1, 1) to (9, 1) through free space.
std::vector<Vec2> zigZag() {
    return {Vec2(1, 1), Vec2(2, 3), Vec2(3, 1), Vec2(4, 3), Vec2(5, 1),
            Vec2(6, 3), Vec2(7, 1), Vec2(8, 3), Vec2(9, 1)};
}

}  // namespace

TEST(Shortcut, ShortPathsAreUnchanged) {
    const World2D w = emptyWorld();
    EXPECT_TRUE(smooth(w, {}).empty());
    const std::vector<Vec2> one{Vec2(1, 1)};
    EXPECT_EQ(smooth(w, one), one);
    const std::vector<Vec2> two{Vec2(1, 1), Vec2(5, 5)};
    EXPECT_EQ(smooth(w, two), two);
}

TEST(Shortcut, ZeroIterationsIsANoOp) {
    const auto in = zigZag();
    EXPECT_EQ(smooth(emptyWorld(), in, 0), in);
}

TEST(Shortcut, FreeSpaceCollapsesToStraightLine) {
    const World2D w = emptyWorld();
    const auto in = zigZag();
    const auto out = smooth(w, in);
    expectValidShortcut(w, in, out);
    ASSERT_EQ(out.size(), 2u);
}

TEST(Shortcut, KeepsCornerAroundObstacle) {
    // Wall between start and goal; the path goes over the top of it.
    World2D w = emptyWorld();
    w.rects.push_back({Vec2(4.5, 0), Vec2(5.5, 7)});
    const std::vector<Vec2> in{Vec2(1, 1), Vec2(1, 4), Vec2(2, 8), Vec2(4, 8), Vec2(5, 8.5),
                               Vec2(6, 8), Vec2(8, 8), Vec2(9, 4), Vec2(9, 1)};
    for (size_t i = 0; i + 1 < in.size(); ++i)
        ASSERT_FALSE(drone::segmentCollides(w, in[i], in[i + 1])) << "bad fixture, segment " << i;

    const auto out = smooth(w, in);
    expectValidShortcut(w, in, out);
    EXPECT_GE(out.size(), 3u);  // start -> goal goes straight through the wall
    EXPECT_LT(drone::pathLength(out), drone::pathLength(in));
}

TEST(Shortcut, SameSeedGivesSameResult) {
    World2D w = emptyWorld();
    w.circles.push_back({Vec2(5, 2), 0.8});
    const auto in = zigZag();
    EXPECT_EQ(smooth(w, in, 5, 3), smooth(w, in, 5, 3));
}

TEST(Shortcut, ShortensRrtPathsOnShippedMaps) {
    drone::RrtParams params;
    params.max_iters = 20000;
    for (const char* name : {"forest.world", "rooms.world"}) {
        const World2D w = drone::loadWorld2D(std::string(DRONE_MAPS_DIR) + "/" + name);
        for (unsigned seed = 0; seed < 5; ++seed) {
            SCOPED_TRACE(std::string(name) + " seed " + std::to_string(seed));
            std::mt19937 rng(seed);
            const auto raw = drone::rrt(w, params, rng).path;
            ASSERT_FALSE(raw.empty());
            const auto out = smooth(w, raw);
            expectValidShortcut(w, raw, out);
            EXPECT_LT(drone::pathLength(out), drone::pathLength(raw));
            EXPECT_LT(out.size(), raw.size() / 4);
        }
    }
}
