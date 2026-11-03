#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

#include "drone/grid.hpp"

using drone::Cell;
using drone::GridMap;
using drone::parseWorld;
using drone::World;

namespace {
World parse(const std::string& text) {
    std::istringstream in(text);
    return parseWorld(in);
}
}  // namespace

TEST(GridMap, BoundsAndOccupancy) {
    GridMap g(3, 2);
    EXPECT_TRUE(g.inBounds({0, 0}));
    EXPECT_TRUE(g.inBounds({2, 1}));
    EXPECT_FALSE(g.inBounds({3, 0}));
    EXPECT_FALSE(g.inBounds({0, -1}));
    g.setObstacle({1, 1});
    EXPECT_FALSE(g.isFree({1, 1}));
    EXPECT_TRUE(g.isFree({1, 0}));
    EXPECT_FALSE(g.isFree({-1, 0}));  // out of bounds is never free
}

TEST(ParseWorld, ReadsDimensionsStartGoalAndObstacles) {
    const World w = parse(
        "S..#\n"
        ".#..\n"
        "...G\n");
    EXPECT_EQ(w.grid.width, 4);
    EXPECT_EQ(w.grid.height, 3);
    EXPECT_EQ(w.start, (Cell{0, 0}));
    EXPECT_EQ(w.goal, (Cell{3, 2}));
    EXPECT_FALSE(w.grid.isFree({3, 0}));
    EXPECT_FALSE(w.grid.isFree({1, 1}));
    EXPECT_TRUE(w.grid.isFree(w.start));
    EXPECT_TRUE(w.grid.isFree(w.goal));
}

TEST(ParseWorld, IgnoresBlankLinesAndCRLF) {
    const World w = parse("\nS.\r\n\n.G\r\n\n");
    EXPECT_EQ(w.grid.width, 2);
    EXPECT_EQ(w.grid.height, 2);
}

TEST(ParseWorld, RejectsRaggedRows) { EXPECT_THROW(parse("S..\n.G\n"), std::runtime_error); }

TEST(ParseWorld, RejectsUnknownCharacter) { EXPECT_THROW(parse("S.x\n..G\n"), std::runtime_error); }

TEST(ParseWorld, RequiresExactlyOneStartAndGoal) {
    EXPECT_THROW(parse("...\n..G\n"), std::runtime_error);
    EXPECT_THROW(parse("S..\n...\n"), std::runtime_error);
    EXPECT_THROW(parse("SS.\n..G\n"), std::runtime_error);
    EXPECT_THROW(parse("S.G\n..G\n"), std::runtime_error);
}

TEST(ParseWorld, RejectsEmpty) { EXPECT_THROW(parse("\n\n"), std::runtime_error); }

TEST(LoadWorld, ThrowsOnMissingFile) {
    EXPECT_THROW(drone::loadWorld("/nonexistent/map.txt"), std::runtime_error);
}

TEST(LoadWorld, LoadsBundledMaps) {
    const World simple = drone::loadWorld(DRONE_MAPS_DIR "/simple.txt");
    EXPECT_GT(simple.grid.width, 0);
    const World maze = drone::loadWorld(DRONE_MAPS_DIR "/maze.txt");
    EXPECT_GT(maze.grid.width, 0);
}
