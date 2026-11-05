// plan_rrt: load a .world map, run RRT, write the tree and path as CSV for
// viz/animate_rrt.py.
//
// Usage: plan_rrt <map.world> <out_dir> [--seed N] [--step S] [--goal-tol T]
//                 [--goal-bias B] [--max-iters N]
// Writes <out_dir>/tree.csv (id,parent,x,y in insertion order) and
//        <out_dir>/path.csv (x,y from start to goal; header only if no path).
// Exit codes: 0 = path found, 1 = usage/IO/parse error, 2 = no path.

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "drone/rrt.hpp"
#include "drone/world2d.hpp"

namespace {

void usage(const char* prog) {
    std::cerr << "usage: " << prog
              << " <map.world> <out_dir> [--seed N] [--step S] [--goal-tol T]"
                 " [--goal-bias B] [--max-iters N]\n";
}

void writeTree(const std::filesystem::path& file, const drone::RrtTree& tree) {
    std::ofstream out(file);
    if (!out) throw std::runtime_error("cannot write " + file.string());
    out << std::setprecision(10) << "id,parent,x,y\n";
    for (int i = 0; i < tree.size(); ++i)
        out << i << ',' << tree.parent[i] << ',' << tree.nodes[i].x() << ',' << tree.nodes[i].y() << '\n';
}

void writePath(const std::filesystem::path& file, const std::vector<drone::Vec2>& path) {
    std::ofstream out(file);
    if (!out) throw std::runtime_error("cannot write " + file.string());
    out << std::setprecision(10) << "x,y\n";
    for (const auto& p : path) out << p.x() << ',' << p.y() << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }
    try {
        drone::RrtParams params;
        unsigned seed = 0;
        for (int i = 3; i < argc; ++i) {
            const std::string flag = argv[i];
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + flag);
            const std::string value = argv[++i];
            if (flag == "--seed") seed = static_cast<unsigned>(std::stoul(value));
            else if (flag == "--step") params.step = std::stod(value);
            else if (flag == "--goal-tol") params.goal_tol = std::stod(value);
            else if (flag == "--goal-bias") params.goal_bias = std::stod(value);
            else if (flag == "--max-iters") params.max_iters = std::stoi(value);
            else throw std::invalid_argument("unknown option " + flag);
        }

        const drone::World2D world = drone::loadWorld2D(argv[1]);
        const std::filesystem::path outDir = argv[2];
        std::filesystem::create_directories(outDir);

        std::mt19937 rng(seed);
        const auto t0 = std::chrono::steady_clock::now();
        const drone::RrtResult result = drone::rrt(world, params, rng);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        writeTree(outDir / "tree.csv", result.tree);
        writePath(outDir / "path.csv", result.path);

        std::cout << std::fixed << std::setprecision(2);
        if (result.path.empty()) {
            std::cout << "no path found after " << result.iterations << " iterations (" << result.tree.size()
                      << " nodes, " << ms << " ms)\n";
            return 2;
        }
        std::cout << "path found: length " << drone::pathLength(result.path) << " m, " << result.path.size()
                  << " waypoints, " << result.tree.size() << " nodes, " << result.iterations << " iterations, "
                  << ms << " ms\n";
        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "error: " << e.what() << "\n";
        usage(argv[0]);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
