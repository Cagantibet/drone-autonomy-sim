#pragma once

#include <vector>

#include "drone/grid.hpp"

namespace drone {

// Shortest 4-connected path (unit step cost) from start to goal, inclusive of
// both endpoints. Returns an empty vector if start or goal is out of bounds or
// blocked, or if no path exists. start == goal returns {start}.
//
// Approach:
//  - Min-heap (std::priority_queue with std::greater) of {f, index}, where
//    f = g + h, g = steps from start, h = Manhattan distance to goal.
//    Manhattan is admissible for 4-connected unit moves, so the first time
//    the goal is popped its path is optimal.
//  - Store g (init to a large value) and cameFrom (init -1) in flat vectors
//    of size width*height, indexed with grid.index(cell).
//  - Pop the lowest f. Skip it if it's stale (its f > g[idx] + h). Stop when
//    the goal is popped. Otherwise relax the 4 neighbours that pass isFree.
//  - Rebuild the path by walking cameFrom back from the goal, then reverse it.
//    (index -> cell: x = idx % width, y = idx / width)
std::vector<Cell> astar(const GridMap& grid, Cell start, Cell goal);

}  // namespace drone
