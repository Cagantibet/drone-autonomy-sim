#include <gtest/gtest.h>

#include <cstdlib>
#include <sstream>

#include "drone/astar.hpp"
#include "drone/grid.hpp"

using drone::astar;
using drone::Cell;
using drone::GridMap;
using drone::World;

namespace {

World parse(const std::string& text) {
    std::istringstream in(text);
    return drone::parseWorld(in);
}

// Checks the path is a valid route: correct endpoints, every cell free,
// every consecutive pair 4-adjacent.
void expectValidPath(const GridMap& g, const std::vector<Cell>& path, Cell start, Cell goal) {
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
    for (size_t i = 0; i < path.size(); ++i) {
        EXPECT_TRUE(g.isFree(path[i])) << "cell " << i << " is blocked";
        if (i > 0) {
            const int d = std::abs(path[i].x - path[i - 1].x) + std::abs(path[i].y - path[i - 1].y);
            EXPECT_EQ(d, 1) << "step " << i << " is not 4-adjacent";
        }
    }
}

}  // namespace

TEST(AStar, StraightLineOnOpenGrid) {
    const World w = parse("S....G\n");
    const auto path = astar(w.grid, w.start, w.goal);
    expectValidPath(w.grid, path, w.start, w.goal);
    EXPECT_EQ(path.size(), 6u);
}

TEST(AStar, DetoursAroundWallOptimally) {
    // Wall at x=2 forces going through the gap at the bottom row.
    const World w = parse(
        "S.#.G\n"
        "..#..\n"
        ".....\n");
    const auto path = astar(w.grid, w.start, w.goal);
    expectValidPath(w.grid, path, w.start, w.goal);
    EXPECT_EQ(path.size(), 9u);  // 2 down, 4 right, 2 up = 8 moves
}

TEST(AStar, StartEqualsGoal) {
    GridMap g(3, 3);
    const auto path = astar(g, {1, 1}, {1, 1});
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path[0], (Cell{1, 1}));
}

TEST(AStar, NoPathWhenGoalWalledOff) {
    const World w = parse(
        "S..#.\n"
        "...#G\n"
        "...##\n");
    EXPECT_TRUE(astar(w.grid, w.start, w.goal).empty());
}

TEST(AStar, EmptyWhenStartOrGoalBlocked) {
    GridMap g(4, 4);
    g.setObstacle({0, 0});
    g.setObstacle({3, 3});
    EXPECT_TRUE(astar(g, {0, 0}, {2, 2}).empty());
    EXPECT_TRUE(astar(g, {1, 1}, {3, 3}).empty());
}

TEST(AStar, EmptyWhenOutOfBounds) {
    GridMap g(4, 4);
    EXPECT_TRUE(astar(g, {-1, 0}, {2, 2}).empty());
    EXPECT_TRUE(astar(g, {0, 0}, {4, 0}).empty());
}

TEST(AStar, SimpleMapOptimalLength) {
    const World w = drone::loadWorld(DRONE_MAPS_DIR "/simple.txt");
    const auto path = astar(w.grid, w.start, w.goal);
    expectValidPath(w.grid, path, w.start, w.goal);
    EXPECT_EQ(path.size(), 23u);  // verified with BFS
}

TEST(AStar, MazeOptimalLength) {
    const World w = drone::loadWorld(DRONE_MAPS_DIR "/maze.txt");
    const auto path = astar(w.grid, w.start, w.goal);
    expectValidPath(w.grid, path, w.start, w.goal);
    EXPECT_EQ(path.size(), 37u);  // verified with BFS
}
