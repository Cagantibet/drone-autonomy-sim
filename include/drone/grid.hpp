#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace drone {

// Integer grid coordinate. x = column, y = row (row 0 is the top of the map file).
struct Cell {
    int x = 0;
    int y = 0;
};

inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Cell a, Cell b) { return !(a == b); }

// Occupancy grid stored row-major in a flat vector (1 = obstacle).
struct GridMap {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> occupied;

    GridMap() = default;
    GridMap(int w, int h) : width(w), height(h), occupied(static_cast<size_t>(w) * h, 0) {}

    int index(Cell c) const { return c.y * width + c.x; }
    bool inBounds(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height; }
    bool isFree(Cell c) const { return inBounds(c) && occupied[index(c)] == 0; }
    void setObstacle(Cell c, bool blocked = true) { occupied[index(c)] = blocked ? 1 : 0; }
};

// A grid plus the mission's start and goal.
struct World {
    GridMap grid;
    Cell start;
    Cell goal;
};

// Map text format, one char per cell:
//   '.' free   '#' obstacle   'S' start (exactly one)   'G' goal (exactly one)
// Blank lines are ignored. All rows must have the same width.
// Throws std::runtime_error on malformed input.
World parseWorld(std::istream& in);
World loadWorld(const std::string& path);

}  // namespace drone
