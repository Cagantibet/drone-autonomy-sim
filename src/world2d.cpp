#include "drone/world2d.hpp"

#include <fstream>
#include <istream>
#include <sstream>
#include <stdexcept>

namespace drone {

namespace {

[[noreturn]] void fail(int line, const std::string& msg) {
    throw std::runtime_error("world parse error (line " + std::to_string(line) + "): " + msg);
}

// Reads exactly n numbers from the rest of the line; anything else is an error.
std::vector<double> readNumbers(std::istringstream& ss, int n, int line, const std::string& key) {
    std::vector<double> v(n);
    for (double& x : v)
        if (!(ss >> x)) fail(line, "'" + key + "' expects " + std::to_string(n) + " numbers");
    std::string extra;
    if (ss >> extra) fail(line, "unexpected trailing token '" + extra + "'");
    return v;
}

bool inside(const World2D& w, const Vec2& p) {
    return p.x() >= 0.0 && p.y() >= 0.0 && p.x() <= w.width && p.y() <= w.height;
}

}  // namespace

World2D parseWorld2D(std::istream& in) {
    World2D w;
    bool hasBounds = false, hasRadius = false, hasStart = false, hasGoal = false;
    int startLine = 0, goalLine = 0;

    std::string raw;
    for (int line = 1; std::getline(in, raw); ++line) {
        if (const auto hash = raw.find('#'); hash != std::string::npos) raw.erase(hash);
        std::istringstream ss(raw);
        std::string key;
        if (!(ss >> key)) continue;  // blank or comment-only line

        auto once = [&](bool& seen) {
            if (seen) fail(line, "duplicate '" + key + "'");
            seen = true;
        };

        if (key == "bounds") {
            once(hasBounds);
            const auto v = readNumbers(ss, 2, line, key);
            if (v[0] <= 0.0 || v[1] <= 0.0) fail(line, "bounds must be positive");
            w.width = v[0];
            w.height = v[1];
        } else if (key == "radius") {
            once(hasRadius);
            const auto v = readNumbers(ss, 1, line, key);
            if (v[0] < 0.0) fail(line, "radius must be >= 0");
            w.drone_radius = v[0];
        } else if (key == "start") {
            once(hasStart);
            const auto v = readNumbers(ss, 2, line, key);
            w.start = {v[0], v[1]};
            startLine = line;
        } else if (key == "goal") {
            once(hasGoal);
            const auto v = readNumbers(ss, 2, line, key);
            w.goal = {v[0], v[1]};
            goalLine = line;
        } else if (key == "circle") {
            const auto v = readNumbers(ss, 3, line, key);
            if (v[2] <= 0.0) fail(line, "circle radius must be > 0");
            w.circles.push_back({{v[0], v[1]}, v[2]});
        } else if (key == "rect") {
            const auto v = readNumbers(ss, 4, line, key);
            if (v[0] >= v[2] || v[1] >= v[3]) fail(line, "rect needs x0 < x1 and y0 < y1");
            w.rects.push_back({{v[0], v[1]}, {v[2], v[3]}});
        } else {
            fail(line, "unknown directive '" + key + "'");
        }
    }

    if (!hasBounds) fail(0, "missing 'bounds'");
    if (!hasStart) fail(0, "missing 'start'");
    if (!hasGoal) fail(0, "missing 'goal'");
    if (!inside(w, w.start)) fail(startLine, "start is outside bounds");
    if (!inside(w, w.goal)) fail(goalLine, "goal is outside bounds");
    return w;
}

World2D loadWorld2D(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open world file: " + path);
    return parseWorld2D(file);
}

}  // namespace drone
