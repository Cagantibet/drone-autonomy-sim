#include <gtest/gtest.h>

#include <cmath>

#include "drone/collision.hpp"

using drone::Circle;
using drone::Rect;
using drone::Vec2;
using drone::World2D;

namespace {

// 10 x 10 world with one circle at (3, 3) r=1 and one rect [6, 8] x [2, 4].
World2D makeWorld() {
    World2D w;
    w.width = 10.0;
    w.height = 10.0;
    w.circles.push_back(Circle{Vec2(3, 3), 1.0});
    w.rects.push_back(Rect{Vec2(6, 2), Vec2(8, 4)});
    return w;
}

}  // namespace

// ---- distance helpers ----

TEST(Distance, PointSegment) {
    const Vec2 a(0, 0), b(4, 0);
    EXPECT_DOUBLE_EQ(drone::pointSegmentDistance(Vec2(2, 3), a, b), 3.0);   // above the middle
    EXPECT_DOUBLE_EQ(drone::pointSegmentDistance(Vec2(-3, 4), a, b), 5.0);  // past endpoint a
    EXPECT_DOUBLE_EQ(drone::pointSegmentDistance(Vec2(1, 1), a, a), std::sqrt(2.0));  // degenerate
}

TEST(Distance, PointRect) {
    const Rect r{Vec2(0, 0), Vec2(2, 1)};
    EXPECT_DOUBLE_EQ(drone::pointRectDistance(Vec2(1, 0.5), r), 0.0);  // inside
    EXPECT_DOUBLE_EQ(drone::pointRectDistance(Vec2(1, 3), r), 2.0);    // above an edge
    EXPECT_DOUBLE_EQ(drone::pointRectDistance(Vec2(5, 5), r), 5.0);    // off a corner (3-4-5)
}

// ---- point collision ----

TEST(PointCollides, FreeSpace) {
    EXPECT_FALSE(drone::pointCollides(makeWorld(), Vec2(5, 8), 0.5));
}

TEST(PointCollides, InsideObstacles) {
    const World2D w = makeWorld();
    EXPECT_TRUE(drone::pointCollides(w, Vec2(3, 3), 0.0));
    EXPECT_TRUE(drone::pointCollides(w, Vec2(7, 3), 0.0));  // rect interior with zero radius
}

TEST(PointCollides, RadiusInflatesObstacles) {
    const World2D w = makeWorld();
    // 1.5 from the circle center: free for a point, hit for a 0.6 disc.
    EXPECT_FALSE(drone::pointCollides(w, Vec2(4.5, 3), 0.0));
    EXPECT_TRUE(drone::pointCollides(w, Vec2(4.5, 3), 0.6));
    // 0.5 left of the rect: free for 0.4, hit for 0.6.
    EXPECT_FALSE(drone::pointCollides(w, Vec2(5.5, 3), 0.4));
    EXPECT_TRUE(drone::pointCollides(w, Vec2(5.5, 3), 0.6));
}

TEST(PointCollides, ExactContactIsNotACollision) {
    const World2D w = makeWorld();
    EXPECT_FALSE(drone::pointCollides(w, Vec2(4.5, 3), 0.5));  // touches circle
    EXPECT_FALSE(drone::pointCollides(w, Vec2(5.5, 3), 0.5));  // touches rect edge
    EXPECT_FALSE(drone::pointCollides(w, Vec2(6, 3), 0.0));    // on rect edge, zero radius
}

TEST(PointCollides, Bounds) {
    const World2D w = makeWorld();
    EXPECT_TRUE(drone::pointCollides(w, Vec2(-0.1, 5), 0.0));
    EXPECT_TRUE(drone::pointCollides(w, Vec2(5, 10.1), 0.0));
    EXPECT_TRUE(drone::pointCollides(w, Vec2(0.2, 5), 0.3));  // body pokes out the left side
    EXPECT_FALSE(drone::pointCollides(w, Vec2(0.3, 5), 0.3)); // body touches the wall
    EXPECT_FALSE(drone::pointCollides(w, Vec2(0, 0), 0.0));
}

TEST(PointCollides, OverloadUsesDroneRadius) {
    World2D w = makeWorld();
    w.drone_radius = 0.6;
    EXPECT_TRUE(drone::pointCollides(w, Vec2(4.5, 3)));
    w.drone_radius = 0.0;
    EXPECT_FALSE(drone::pointCollides(w, Vec2(4.5, 3)));
}

// ---- segment collision ----

TEST(SegmentCollides, FreeSegment) {
    EXPECT_FALSE(drone::segmentCollides(makeWorld(), Vec2(1, 8), Vec2(9, 8), 0.5));
}

TEST(SegmentCollides, PassesThroughCircleWithFreeEndpoints) {
    const World2D w = makeWorld();
    EXPECT_FALSE(drone::pointCollides(w, Vec2(1, 3), 0.0));
    EXPECT_FALSE(drone::pointCollides(w, Vec2(5, 3), 0.0));
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(1, 3), Vec2(5, 3), 0.0));
}

TEST(SegmentCollides, PassesThroughThinRectWithFreeEndpoints) {
    World2D w;
    w.width = w.height = 10.0;
    w.rects.push_back(Rect{Vec2(5, 0), Vec2(5.05, 10)});  // 5 cm wall
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(1, 5), Vec2(9, 5), 0.0));
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(1, 7), Vec2(9, 5), 0.0));  // diagonal
}

TEST(SegmentCollides, CircleRadiusInflation) {
    const World2D w = makeWorld();
    // Horizontal line 1.5 above the circle center.
    EXPECT_FALSE(drone::segmentCollides(w, Vec2(0.5, 4.5), Vec2(5, 4.5), 0.4));
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(0.5, 4.5), Vec2(5, 4.5), 0.6));
}

TEST(SegmentCollides, GrazingRectCorner) {
    const World2D w = makeWorld();
    // Diagonal line x + y = 12.2 passes ~0.141 from the corner (8, 4).
    const Vec2 a(9.5, 2.7), b(7.7, 4.5);
    EXPECT_FALSE(drone::segmentCollides(w, a, b, 0.0));
    EXPECT_FALSE(drone::segmentCollides(w, a, b, 0.1));
    EXPECT_TRUE(drone::segmentCollides(w, a, b, 0.2));
}

TEST(SegmentCollides, RunningAlongRectEdgeIsNotACollision) {
    const World2D w = makeWorld();
    EXPECT_FALSE(drone::segmentCollides(w, Vec2(5, 4), Vec2(9, 4), 0.0));  // top edge
    EXPECT_FALSE(drone::segmentCollides(w, Vec2(6, 0.5), Vec2(6, 6), 0.0)); // left edge
}

TEST(SegmentCollides, EndpointInsideObstacle) {
    const World2D w = makeWorld();
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(1, 8), Vec2(7, 3), 0.0));
}

TEST(SegmentCollides, LeavingBounds) {
    const World2D w = makeWorld();
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(5, 8), Vec2(11, 8), 0.0));
    EXPECT_TRUE(drone::segmentCollides(w, Vec2(5, 9.5), Vec2(5, 9.8), 0.3));
}

TEST(SegmentCollides, ZeroLengthSegmentMatchesPointCheck) {
    const World2D w = makeWorld();
    for (const Vec2& p : {Vec2(3, 3), Vec2(4.5, 3), Vec2(7, 3), Vec2(5, 8)})
        EXPECT_EQ(drone::segmentCollides(w, p, p, 0.3), drone::pointCollides(w, p, 0.3));
}
