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
    RrtResult result;
    result.tree.addNode(w.start, -1);

    if (pointCollides(w, w.start, w.drone_radius) || pointCollides(w, w.goal, w.drone_radius)) {
        return result;
    }

    std::uniform_real_distribution<double> xDist(0.0, w.width);
    std::uniform_real_distribution<double> yDist(0.0, w.height);
    std::uniform_real_distribution<double> goalBiasDist(0.0, 1.0);

    for (int iter = 0; iter < params.max_iters; ++iter) {
        Vec2 sample;
        if (goalBiasDist(rng) < params.goal_bias) {
            sample = w.goal;
        } else {
            sample = Vec2(xDist(rng), yDist(rng));
        }

        const int nearestIdx = nearestIndex(result.tree, sample);
        const Vec2 newNode = steer(result.tree.nodes[nearestIdx], sample, params.step);

        if (!segmentCollides(w, result.tree.nodes[nearestIdx], newNode, w.drone_radius)) {
            const int newIdx = result.tree.addNode(newNode, nearestIdx);
            const double distToGoal = (newNode - w.goal).norm();
            if (distToGoal <= params.goal_tol) {
                // newNode may already be the goal (a goal sample within one
                // step); otherwise only finish if the final edge is free.
                int goalIdx = -1;
                if (distToGoal <= 1e-9) {
                    goalIdx = newIdx;
                } else if (!segmentCollides(w, newNode, w.goal, w.drone_radius)) {
                    goalIdx = result.tree.addNode(w.goal, newIdx);
                }
                if (goalIdx != -1) {
                    result.path = extractPath(result.tree, goalIdx);
                    result.iterations = iter + 1;
                    return result;
                }
            }
        }
    }

    result.iterations = params.max_iters;
    return result;
}

}  // namespace drone
