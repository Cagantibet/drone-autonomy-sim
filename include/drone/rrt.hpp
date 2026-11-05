#pragma once

#include <random>
#include <vector>

#include "drone/world2d.hpp"

namespace drone {

struct RrtParams {
    double step = 0.5;        // max edge length when extending the tree (m)
    double goal_tol = 0.5;    // connect to the goal once within this distance (m)
    double goal_bias = 0.05;  // probability of sampling the goal instead of a random point
    int max_iters = 5000;     // sampling iterations before giving up
};

// Tree stored as parallel arrays in insertion order (handy for animating growth).
// nodes[0] is the root (start) and parent[0] == -1.
struct RrtTree {
    std::vector<Vec2> nodes;
    std::vector<int> parent;

    int addNode(const Vec2& p, int parentIdx) {
        nodes.push_back(p);
        parent.push_back(parentIdx);
        return static_cast<int>(nodes.size()) - 1;
    }
    int size() const { return static_cast<int>(nodes.size()); }
};

struct RrtResult {
    RrtTree tree;
    std::vector<Vec2> path;  // start ... goal; empty if no path was found
    int iterations = 0;      // loop iterations actually used
};

// Index of the tree node closest to p (linear scan). Tree must be non-empty.
int nearestIndex(const RrtTree& tree, const Vec2& p);

// Move from `from` toward `to` by at most `step`. Returns `to` if it's closer.
Vec2 steer(const Vec2& from, const Vec2& to, double step);

// Walk parents from node idx back to the root; returns root ... idx.
std::vector<Vec2> extractPath(const RrtTree& tree, int idx);

// Sum of segment lengths (0 for fewer than two points).
double pathLength(const std::vector<Vec2>& path);

// Plan a collision-free path from w.start to w.goal for a disc of radius
// w.drone_radius. Samples are uniform over the bounds, or the goal with
// probability goal_bias; all randomness comes from rng, so a fixed seed gives
// a reproducible tree. Returns an empty path if start or goal is in collision
// or no path is found within max_iters (the tree is still returned).
RrtResult rrt(const World2D& w, const RrtParams& params, std::mt19937& rng);

}  // namespace drone
