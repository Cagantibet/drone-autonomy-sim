#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

#include "drone/world2d.hpp"

using drone::World2D;

namespace {

World2D parse(const std::string& text) {
    std::istringstream in(text);
    return drone::parseWorld2D(in);
}

constexpr const char* kMinimal = "bounds 10 5\nstart 1 1\ngoal 9 4\n";

}  // namespace

TEST(ParseWorld2D, ReadsAllDirectives) {
    const World2D w = parse(
        "# a comment line\n"
        "bounds 20 10\n"
        "radius 0.3   # trailing comment\n"
        "\n"
        "start 1 2\n"
        "goal 18.5 9\n"
        "circle 5 5 1.5\n"
        "rect 10 0 11 6\n"
        "circle 15 3 0.5\n");
    EXPECT_DOUBLE_EQ(w.width, 20.0);
    EXPECT_DOUBLE_EQ(w.height, 10.0);
    EXPECT_DOUBLE_EQ(w.drone_radius, 0.3);
    EXPECT_TRUE(w.start.isApprox(drone::Vec2(1, 2)));
    EXPECT_TRUE(w.goal.isApprox(drone::Vec2(18.5, 9)));
    ASSERT_EQ(w.circles.size(), 2u);
    EXPECT_DOUBLE_EQ(w.circles[0].radius, 1.5);
    EXPECT_TRUE(w.circles[1].center.isApprox(drone::Vec2(15, 3)));
    ASSERT_EQ(w.rects.size(), 1u);
    EXPECT_TRUE(w.rects[0].min.isApprox(drone::Vec2(10, 0)));
    EXPECT_TRUE(w.rects[0].max.isApprox(drone::Vec2(11, 6)));
}

TEST(ParseWorld2D, RadiusDefaultsToZeroAndObstaclesAreOptional) {
    const World2D w = parse(kMinimal);
    EXPECT_DOUBLE_EQ(w.drone_radius, 0.0);
    EXPECT_TRUE(w.circles.empty());
    EXPECT_TRUE(w.rects.empty());
}

TEST(ParseWorld2D, StartAndGoalMayLieOnTheBoundary) {
    EXPECT_NO_THROW(parse("bounds 10 5\nstart 0 0\ngoal 10 5\n"));
}

TEST(ParseWorld2D, RejectsMissingRequiredDirectives) {
    EXPECT_THROW(parse("start 1 1\ngoal 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse("bounds 10 5\ngoal 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse("bounds 10 5\nstart 1 1\n"), std::runtime_error);
    EXPECT_THROW(parse(""), std::runtime_error);
}

TEST(ParseWorld2D, RejectsDuplicates) {
    EXPECT_THROW(parse(std::string(kMinimal) + "bounds 3 3\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "start 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "radius 1\nradius 2\n"), std::runtime_error);
}

TEST(ParseWorld2D, RejectsBadNumbersAndArity) {
    EXPECT_THROW(parse("bounds 10\nstart 1 1\ngoal 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse("bounds 10 five\nstart 1 1\ngoal 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "circle 1 1 1 1\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "rect 1 1 2\n"), std::runtime_error);
}

TEST(ParseWorld2D, RejectsInvalidValues) {
    EXPECT_THROW(parse("bounds 0 5\nstart 0 0\ngoal 0 0\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "radius -0.1\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "circle 3 3 0\n"), std::runtime_error);
    EXPECT_THROW(parse(std::string(kMinimal) + "rect 3 3 2 4\n"), std::runtime_error);  // x0 > x1
    EXPECT_THROW(parse(std::string(kMinimal) + "rect 3 3 4 3\n"), std::runtime_error);  // zero height
}

TEST(ParseWorld2D, RejectsStartOrGoalOutsideBounds) {
    EXPECT_THROW(parse("bounds 10 5\nstart -1 1\ngoal 2 2\n"), std::runtime_error);
    EXPECT_THROW(parse("bounds 10 5\nstart 1 1\ngoal 2 5.1\n"), std::runtime_error);
}

TEST(ParseWorld2D, RejectsUnknownDirectiveWithLineNumber) {
    try {
        parse("bounds 10 5\nstart 1 1\ngoal 2 2\ntriangle 1 2 3\n");
        FAIL() << "expected throw";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("line 4"), std::string::npos) << e.what();
    }
}

TEST(LoadWorld2D, LoadsShippedMapsAndThrowsOnMissingFile) {
    for (const char* name : {"forest.world", "rooms.world"}) {
        const World2D w = drone::loadWorld2D(std::string(DRONE_MAPS_DIR) + "/" + name);
        EXPECT_GT(w.circles.size() + w.rects.size(), 0u) << name;
    }
    EXPECT_THROW(drone::loadWorld2D("does/not/exist.world"), std::runtime_error);
}
