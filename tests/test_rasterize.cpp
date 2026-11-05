#include <gtest/gtest.h>

#include <stdexcept>

#include "drone/astar.hpp"
#include "drone/rasterize.hpp"

using drone::Cell;
using drone::Vec2;
using drone::World2D;

TEST(Rasterize, CellConversions) {
    EXPECT_EQ(drone::worldToCell({0.0, 0.0}, 0.5), (Cell{0, 0}));
    EXPECT_EQ(drone::worldToCell({1.24, 0.76}, 0.5), (Cell{2, 1}));
    EXPECT_TRUE(drone::cellCenter({2, 1}, 0.5).isApprox(Vec2(1.25, 0.75)));
    EXPECT_EQ(drone::worldToCell(drone::cellCenter({7, 3}, 0.25), 0.25), (Cell{7, 3}));
}

TEST(Rasterize, SizeRoundsUp) {
    World2D w;
    w.width = 10.0;
    w.height = 4.1;
    const auto g = drone::rasterize(w, 0.5);
    EXPECT_EQ(g.width, 20);
    EXPECT_EQ(g.height, 9);
    EXPECT_THROW(drone::rasterize(w, 0.0), std::invalid_argument);
}

TEST(Rasterize, MarksObstaclesAndInflatesByDroneRadius) {
    World2D w;
    w.width = w.height = 10.0;
    w.rects.push_back({Vec2(4, 4), Vec2(6, 6)});
    auto g = drone::rasterize(w, 1.0);
    EXPECT_FALSE(g.isFree({4, 4}));  // center (4.5, 4.5) inside
    EXPECT_FALSE(g.isFree({5, 5}));
    EXPECT_TRUE(g.isFree({3, 4}));   // center (3.5, 4.5), 0.5 from the edge
    EXPECT_TRUE(g.isFree({0, 0}));

    w.drone_radius = 0.6;
    g = drone::rasterize(w, 1.0);
    EXPECT_FALSE(g.isFree({3, 4}));  // now within the inflated rect
    EXPECT_FALSE(g.isFree({0, 0}));  // body would poke out of bounds
    EXPECT_TRUE(g.isFree({1, 1}));
}

TEST(Rasterize, GridPathToWorldPinsEndpoints) {
    const std::vector<Cell> cells{{0, 0}, {1, 0}, {1, 1}};
    const auto pts = drone::gridPathToWorld(cells, 1.0, {0.2, 0.3}, {1.9, 1.8});
    ASSERT_EQ(pts.size(), 3u);
    EXPECT_TRUE(pts[0].isApprox(Vec2(0.2, 0.3)));
    EXPECT_TRUE(pts[1].isApprox(Vec2(1.5, 0.5)));
    EXPECT_TRUE(pts[2].isApprox(Vec2(1.9, 1.8)));
    EXPECT_TRUE(drone::gridPathToWorld({}, 1.0, {0, 0}, {1, 1}).empty());
}

TEST(Rasterize, AStarSolvesRasterizedShippedMaps) {
    for (const char* name : {"forest.world", "rooms.world"}) {
        SCOPED_TRACE(name);
        const World2D w = drone::loadWorld2D(std::string(DRONE_MAPS_DIR) + "/" + name);
        const double res = 0.25;
        const auto grid = drone::rasterize(w, res);
        const auto path = drone::astar(grid, drone::worldToCell(w.start, res), drone::worldToCell(w.goal, res));
        EXPECT_FALSE(path.empty());
    }
}
