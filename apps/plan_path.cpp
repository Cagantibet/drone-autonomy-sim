// plan_path: load a map, run A*, write the path as CSV for viz/plot_world.py.
//
// Usage: plan_path <map.txt> <path_out.csv>
// Exit codes: 0 = path found, 1 = usage/IO/parse error, 2 = no path.

#include <exception>
#include <fstream>
#include <iostream>

#include "drone/astar.hpp"
#include "drone/grid.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: " << argv[0] << " <map.txt> <path_out.csv>\n";
        return 1;
    }
    try {
        const drone::World world = drone::loadWorld(argv[1]);
        const auto path = drone::astar(world.grid, world.start, world.goal);

        std::ofstream out(argv[2]);
        if (!out) {
            std::cerr << "error: cannot write " << argv[2] << "\n";
            return 1;
        }
        out << "x,y\n";
        for (const auto& c : path) out << c.x << ',' << c.y << '\n';

        if (path.empty()) {
            std::cout << "no path found (" << world.grid.width << "x" << world.grid.height << " map)\n";
            return 2;
        }
        std::cout << "path found: " << path.size() << " cells, " << path.size() - 1 << " moves\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
