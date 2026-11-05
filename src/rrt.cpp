#include "drone/rrt.hpp"

#include <algorithm>
#include <limits>

#include "drone/collision.hpp"

namespace drone {

int nearestIndex(const RrtTree& tree, const Vec2& p) {
    int best = 0;
    double bestDist2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < tree.size(); ++i) {
        const double d2 = (tree.nodes[i] - p).squaredNorm();
        if (d2 < bestDist2) {
            bestDist2 = d2;
            best = i;
        }
    }
    return best;
}

Vec2 steer(const Vec2& from, const Vec2& to, double step) {
    const Vec2 delta = to - from;
    const double dist = delta.norm();
    if (dist <= step) return to;
    return from + delta * (step / dist);
}

std::vector<Vec2> extractPath(const RrtTree& tree, int idx) {
    std::vector<Vec2> path;
    for (int i = idx; i != -1; i = tree.parent[i]) path.push_back(tree.nodes[i]);
    std::reverse(path.begin(), path.end());
    return path;
}

double pathLength(const std::vector<Vec2>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) len += (path[i] - path[i - 1]).norm();
    return len;
}

RrtResult rrt(const World2D& w, const RrtParams& params, std::mt19937& rng) {
    // TODO: implement.
    (void)w;
    (void)params;
    (void)rng;
    return {};
}

}  // namespace drone
