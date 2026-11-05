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

// ---- helpers (implemented) ----

// Index of the tree node closest to p (linear scan). Tree must be non-empty.
int nearestIndex(const RrtTree& tree, const Vec2& p);

// Move from `from` toward `to` by at most `step`. Returns `to` if it's closer.
Vec2 steer(const Vec2& from, const Vec2& to, double step);

// Walk parents from node idx back to the root; returns root ... idx.
std::vector<Vec2> extractPath(const RrtTree& tree, int idx);

// Sum of segment lengths (0 for fewer than two points).
double pathLength(const std::vector<Vec2>& path);

// ---- planner (yours) ----

// Plan from w.start to w.goal for a disc of radius w.drone_radius.
//
// Approach:
//  - If w.start or w.goal collides (pointCollides), return an empty result
//    right away (tree with only the start node is fine too).
//  - result.tree.addNode(w.start, -1).
//  - Loop it = 0 .. max_iters-1 (set result.iterations as you go):
//      sample q: with probability goal_bias q = goal, else uniform in
//        [0, width] x [0, height]. Use std::uniform_real_distribution<double>
//        with the given rng (don't make your own, or tests can't seed it).
//      n = nearestIndex(tree, q); q_new = steer(nodes[n], q, step).
//      If segmentCollides(w, nodes[n], q_new), skip this sample.
//      Else id = addNode(q_new, n).
//      If |q_new - goal| <= goal_tol and segment q_new -> goal is free:
//        g = addNode(goal, id); path = extractPath(tree, g); return.
//  - Out of iterations: return with an empty path (the tree is kept for plots).
RrtResult rrt(const World2D& w, const RrtParams& params, std::mt19937& rng);

}  // namespace drone
