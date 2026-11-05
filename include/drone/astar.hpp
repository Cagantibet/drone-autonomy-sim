#pragma once

#include <vector>

#include "drone/grid.hpp"

namespace drone {

// Shortest 4-connected path (unit step cost) from start to goal, inclusive of
// both endpoints. Returns an empty vector if start or goal is out of bounds or
// blocked, or if no path exists. start == goal returns {start}.
//
// Uses a Manhattan heuristic (admissible for 4-connected unit moves, so the
// result is optimal), a binary-heap open set with lazy deletion of stale
// entries, and flat g-score/parent arrays indexed by cell.
std::vector<Cell> astar(const GridMap& grid, Cell start, Cell goal);

}  // namespace drone
