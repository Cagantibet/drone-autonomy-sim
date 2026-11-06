#include "drone/tracking_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "drone/collision.hpp"

namespace drone {

double crossTrackError(const std::vector<Vec2>& path, const Vec2& p) {
    if (path.empty()) throw std::invalid_argument("crossTrackError: empty path");
    if (path.size() == 1) return (p - path.front()).norm();
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < path.size(); ++i) {
        best = std::min(best, pointSegmentDistance(p, path[i - 1], path[i]));
    }
    return best;
}

TrackingMetrics computeMetrics(const World2D& w, const std::vector<Vec2>& path, const FollowResult& run) {
    if (path.empty()) throw std::invalid_argument("computeMetrics: empty path");

    TrackingMetrics m;
    m.reached = run.reached;
    m.time_to_goal = run.time_to_goal;
    if (run.samples.empty()) return m;

    const Vec2& goal = path.back();
    Vec2 dir = path.size() >= 2 ? Vec2(goal - path[path.size() - 2]) : Vec2(goal - run.samples.front().position);
    const double dirNorm = dir.norm();
    if (dirNorm > 0.0) dir /= dirNorm;

    double sum = 0.0;
    double sumSq = 0.0;
    for (std::size_t i = 0; i < run.samples.size(); ++i) {
        const Vec2& p = run.samples[i].position;
        const double e = crossTrackError(path, p);
        sum += e;
        sumSq += e * e;
        m.max_error = std::max(m.max_error, e);
        if (dirNorm > 0.0) m.overshoot = std::max(m.overshoot, (p - goal).dot(dir));
        if (i > 0) {
            const Vec2& prev = run.samples[i - 1].position;
            m.distance_flown += (p - prev).norm();
            if (!m.collided && segmentCollides(w, prev, p)) m.collided = true;
        }
    }
    if (run.samples.size() == 1 && pointCollides(w, run.samples.front().position)) m.collided = true;

    const auto n = static_cast<double>(run.samples.size());
    m.mean_error = sum / n;
    m.rms_error = std::sqrt(sumSq / n);
    return m;
}

}  // namespace drone
