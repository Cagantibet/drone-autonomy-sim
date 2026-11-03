#include "drone/grid.hpp"

#include <fstream>
#include <istream>
#include <stdexcept>

namespace drone {

namespace {

[[noreturn]] void fail(const std::string& msg) { throw std::runtime_error("map parse error: " + msg); }

}  // namespace

World parseWorld(std::istream& in) {
    std::vector<std::string> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // tolerate CRLF files
        if (line.empty()) continue;
        if (!rows.empty() && line.size() != rows.front().size())
            fail("row " + std::to_string(rows.size()) + " has width " + std::to_string(line.size()) +
                 ", expected " + std::to_string(rows.front().size()));
        rows.push_back(line);
    }
    if (rows.empty()) fail("map is empty");

    World world;
    world.grid = GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()));
    int starts = 0, goals = 0;

    for (int y = 0; y < world.grid.height; ++y) {
        for (int x = 0; x < world.grid.width; ++x) {
            const char ch = rows[y][x];
            const Cell c{x, y};
            switch (ch) {
                case '.': break;
                case '#': world.grid.setObstacle(c); break;
                case 'S': world.start = c; ++starts; break;
                case 'G': world.goal = c; ++goals; break;
                default:
                    fail(std::string("unknown character '") + ch + "' at (" + std::to_string(x) + ", " +
                         std::to_string(y) + ")");
            }
        }
    }
    if (starts != 1) fail("expected exactly one 'S', found " + std::to_string(starts));
    if (goals != 1) fail("expected exactly one 'G', found " + std::to_string(goals));
    return world;
}

World loadWorld(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open map file: " + path);
    return parseWorld(file);
}

}  // namespace drone
