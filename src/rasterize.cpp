#include "drone/rasterize.hpp"

#include <cmath>
#include <stdexcept>

#include "drone/collision.hpp"

namespace drone {

Cell worldToCell(const Vec2& p, double resolution) {
    return {static_cast<int>(std::floor(p.x() / resolution)), static_cast<int>(std::floor(p.y() / resolution))};
}

Vec2 cellCenter(Cell c, double resolution) { return {(c.x + 0.5) * resolution, (c.y + 0.5) * resolution}; }

GridMap rasterize(const World2D& w, double resolution) {
    if (resolution <= 0.0) throw std::invalid_argument("rasterize: resolution must be > 0");
    GridMap grid(static_cast<int>(std::ceil(w.width / resolution)),
                 static_cast<int>(std::ceil(w.height / resolution)));
    for (int y = 0; y < grid.height; ++y)
        for (int x = 0; x < grid.width; ++x)
            if (pointCollides(w, cellCenter({x, y}, resolution))) grid.setObstacle({x, y});
    return grid;
}

std::vector<Vec2> gridPathToWorld(const std::vector<Cell>& path, double resolution, const Vec2& start,
                                  const Vec2& goal) {
    std::vector<Vec2> out;
    out.reserve(path.size());
    for (const Cell& c : path) out.push_back(cellCenter(c, resolution));
    if (!out.empty()) {
        out.front() = start;
        out.back() = goal;
    }
    return out;
}

}  // namespace drone
