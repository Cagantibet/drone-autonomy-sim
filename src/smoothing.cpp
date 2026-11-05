#include "drone/smoothing.hpp"

#include <utility>

#include "drone/collision.hpp"

namespace drone {

std::vector<Vec2> shortcutPath(const World2D& w, const std::vector<Vec2>& path, int iterations,
                               std::mt19937& rng) {
    std::vector<Vec2> out = path;
    for (int it = 0; it < iterations; ++it) {
        if (out.size() < 3) return out;
        const int n = static_cast<int>(out.size());
        std::uniform_int_distribution<int> pick(0, n - 1);
        int i = pick(rng);
        int j = pick(rng);
        if (i > j) std::swap(i, j);
        if (j - i <= 1) continue;  // no waypoints in between
        if (!segmentCollides(w, out[i], out[j], w.drone_radius)) {
            out.erase(out.begin() + i + 1, out.begin() + j);
        }
    }
    return out;
}

}  // namespace drone
