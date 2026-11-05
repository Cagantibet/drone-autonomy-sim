#pragma once

#include "drone/world2d.hpp"

namespace drone {

// Collision model: the drone is a disc of the given radius. It collides when
// it overlaps an obstacle or pokes outside the world bounds. Exact contact
// (distance == radius) is NOT a collision.

// Euclidean distance from point p to the segment [a, b].
double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b);

// Euclidean distance from point p to rectangle r (0 if p is inside).
double pointRectDistance(const Vec2& p, const Rect& r);

// Does a disc of `radius` centered at p collide with the world?
bool pointCollides(const World2D& w, const Vec2& p, double radius);

// Does a disc of `radius` swept along the segment [a, b] collide with the world?
// Exact (no sampling), so a thin obstacle between two free endpoints is caught.
bool segmentCollides(const World2D& w, const Vec2& a, const Vec2& b, double radius);

// Convenience overloads using w.drone_radius.
inline bool pointCollides(const World2D& w, const Vec2& p) { return pointCollides(w, p, w.drone_radius); }
inline bool segmentCollides(const World2D& w, const Vec2& a, const Vec2& b) {
    return segmentCollides(w, a, b, w.drone_radius);
}

}  // namespace drone
