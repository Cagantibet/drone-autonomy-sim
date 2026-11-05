#pragma once

#include <vector>

#include "drone/grid.hpp"
#include "drone/world2d.hpp"

namespace drone {

// Bridges the continuous world and the grid planners. Cell (x, y) covers
// [x*res, (x+1)*res) x [y*res, (y+1)*res), y-up like World2D.

Cell worldToCell(const Vec2& p, double resolution);
Vec2 cellCenter(Cell c, double resolution);

// Occupancy grid of ceil(width/res) x ceil(height/res) cells. A cell is
// blocked when a drone centered at the cell center would collide
// (obstacles inflated by w.drone_radius, bounds included).
GridMap rasterize(const World2D& w, double resolution);

// Grid path -> world waypoints at cell centers, with the first and last
// replaced by the exact continuous start and goal.
std::vector<Vec2> gridPathToWorld(const std::vector<Cell>& path, double resolution, const Vec2& start,
                                  const Vec2& goal);

}  // namespace drone
