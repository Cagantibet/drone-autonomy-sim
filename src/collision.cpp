#include "drone/collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace drone {

namespace {

bool outOfBounds(const World2D& w, const Vec2& p, double radius) {
    return p.x() < radius || p.y() < radius || p.x() > w.width - radius || p.y() > w.height - radius;
}

bool strictlyInside(const Vec2& p, const Rect& r) {
    return p.x() > r.min.x() && p.x() < r.max.x() && p.y() > r.min.y() && p.y() < r.max.y();
}

// Does the segment [a, b] pass through the open interior of r? (Liang-Barsky
// clipping against open slabs, so running along an edge does not count.)
bool segmentCrossesRectInterior(const Vec2& a, const Vec2& b, const Rect& r) {
    const Vec2 d = b - a;
    double t0 = 0.0, t1 = 1.0;
    for (int axis = 0; axis < 2; ++axis) {
        if (d[axis] == 0.0) {
            if (a[axis] <= r.min[axis] || a[axis] >= r.max[axis]) return false;
            continue;
        }
        double tEnter = (r.min[axis] - a[axis]) / d[axis];
        double tExit = (r.max[axis] - a[axis]) / d[axis];
        if (tEnter > tExit) std::swap(tEnter, tExit);
        t0 = std::max(t0, tEnter);
        t1 = std::min(t1, tExit);
    }
    return t0 < t1;
}

// Distance between segment [a, b] and rectangle r. If they don't intersect,
// the closest pair involves an endpoint of the segment or a corner of the rect.
double segmentRectDistance(const Vec2& a, const Vec2& b, const Rect& r) {
    if (segmentCrossesRectInterior(a, b, r)) return 0.0;
    const std::array<Vec2, 4> corners{r.min, Vec2(r.max.x(), r.min.y()), r.max, Vec2(r.min.x(), r.max.y())};
    double best = std::min(pointRectDistance(a, r), pointRectDistance(b, r));
    for (const Vec2& c : corners) best = std::min(best, pointSegmentDistance(c, a, b));
    return best;
}

}  // namespace

double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.squaredNorm();
    if (len2 == 0.0) return (p - a).norm();
    const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

double pointRectDistance(const Vec2& p, const Rect& r) {
    const Vec2 closest = p.cwiseMax(r.min).cwiseMin(r.max);
    return (p - closest).norm();
}

bool pointCollides(const World2D& w, const Vec2& p, double radius) {
    if (outOfBounds(w, p, radius)) return true;
    for (const Circle& c : w.circles)
        if ((p - c.center).norm() < c.radius + radius) return true;
    for (const Rect& r : w.rects)
        if (strictlyInside(p, r) || pointRectDistance(p, r) < radius) return true;
    return false;
}

bool segmentCollides(const World2D& w, const Vec2& a, const Vec2& b, double radius) {
    // The shrunken bounds box is convex, so checking both endpoints is enough.
    if (outOfBounds(w, a, radius) || outOfBounds(w, b, radius)) return true;
    for (const Circle& c : w.circles)
        if (pointSegmentDistance(c.center, a, b) < c.radius + radius) return true;
    for (const Rect& r : w.rects) {
        if (segmentCrossesRectInterior(a, b, r)) return true;
        if (radius > 0.0 && segmentRectDistance(a, b, r) < radius) return true;
    }
    return false;
}

}  // namespace drone
