#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace drone {

using Vec2 = Eigen::Vector2d;

struct Circle {
    Vec2 center = Vec2::Zero();
    double radius = 0.0;
};

// Axis-aligned rectangle, min corner strictly below/left of max corner.
struct Rect {
    Vec2 min = Vec2::Zero();
    Vec2 max = Vec2::Zero();
};

// Continuous 2D world. Units are meters, y points up, and the free space is
// [0, width] x [0, height] minus the obstacles.
struct World2D {
    double width = 0.0;
    double height = 0.0;
    double drone_radius = 0.0;
    Vec2 start = Vec2::Zero();
    Vec2 goal = Vec2::Zero();
    std::vector<Circle> circles;
    std::vector<Rect> rects;
};

// .world text format, one directive per line ('#' starts a comment):
//   bounds W H          required, once, W > 0 and H > 0
//   radius r            optional, once, r >= 0 (drone radius, default 0)
//   start  x y          required, once, inside bounds
//   goal   x y          required, once, inside bounds
//   circle cx cy r      any number, r > 0
//   rect   x0 y0 x1 y1  any number, x0 < x1 and y0 < y1
// Throws std::runtime_error (with the line number) on malformed input.
World2D parseWorld2D(std::istream& in);
World2D loadWorld2D(const std::string& path);

}  // namespace drone
