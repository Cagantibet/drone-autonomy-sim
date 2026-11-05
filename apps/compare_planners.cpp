// compare_planners: A* (on a rasterized grid) vs RRT (continuous) on one .world map.
//
// Usage: compare_planners <map.world> [--res R] [--runs N] [--max-iters N] [--out DIR]
//   --res        grid cell size for A* in m (default 0.25)
//   --runs       RRT seeds 0..N-1, and A* timing repetitions (default 20)
//   --max-iters  RRT iteration budget (default 20000)
//   --out        write astar_path.csv and rrt_path.csv (shortest RRT run) here
// Prints a markdown table. Exit codes: 0 = ok, 1 = usage/IO/parse error.

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "drone/astar.hpp"
#include "drone/rasterize.hpp"
#include "drone/rrt.hpp"
#include "drone/world2d.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double mean(const std::vector<double>& v) {
    return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

void writePath(const std::filesystem::path& file, const std::vector<drone::Vec2>& path) {
    std::ofstream out(file);
    if (!out) throw std::runtime_error("cannot write " + file.string());
    out << std::setprecision(10) << "x,y\n";
    for (const auto& p : path) out << p.x() << ',' << p.y() << '\n';
}

std::string fmt(double v, int precision = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << v;
    return ss.str();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <map.world> [--res R] [--runs N] [--max-iters N] [--out DIR]\n";
        return 1;
    }
    try {
        double res = 0.25;
        int runs = 20;
        drone::RrtParams params;
        params.max_iters = 20000;
        std::string outDir;
        for (int i = 2; i < argc; ++i) {
            const std::string flag = argv[i];
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + flag);
            const std::string value = argv[++i];
            if (flag == "--res") res = std::stod(value);
            else if (flag == "--runs") runs = std::max(1, std::stoi(value));
            else if (flag == "--max-iters") params.max_iters = std::stoi(value);
            else if (flag == "--out") outDir = value;
            else throw std::invalid_argument("unknown option " + flag);
        }

        const drone::World2D world = drone::loadWorld2D(argv[1]);

        // ---- A* on the rasterized map (deterministic; repeat only for timing) ----
        auto t0 = Clock::now();
        const drone::GridMap grid = drone::rasterize(world, res);
        const double rasterMs = msSince(t0);
        const drone::Cell s = drone::worldToCell(world.start, res), g = drone::worldToCell(world.goal, res);

        std::vector<drone::Cell> cells;
        std::vector<double> astarMs;
        for (int r = 0; r < runs; ++r) {
            t0 = Clock::now();
            cells = drone::astar(grid, s, g);
            astarMs.push_back(msSince(t0));
        }
        const auto astarPath = drone::gridPathToWorld(cells, res, world.start, world.goal);

        // ---- RRT over seeds ----
        std::vector<double> rrtLen, rrtMs, rrtNodes;
        std::vector<drone::Vec2> bestRrt;
        for (int seed = 0; seed < runs; ++seed) {
            std::mt19937 rng(static_cast<unsigned>(seed));
            t0 = Clock::now();
            const auto result = drone::rrt(world, params, rng);
            rrtMs.push_back(msSince(t0));
            if (result.path.empty()) continue;
            const double len = drone::pathLength(result.path);
            rrtLen.push_back(len);
            rrtNodes.push_back(result.tree.size());
            if (bestRrt.empty() || len < drone::pathLength(bestRrt)) bestRrt = result.path;
        }

        // ---- report ----
        std::cout << "Map: " << argv[1] << "  (" << world.width << " x " << world.height
                  << " m, drone radius " << world.drone_radius << " m)\n"
                  << "A*: " << grid.width << "x" << grid.height << " grid at " << res
                  << " m/cell, 4-connected, rasterized in " << fmt(rasterMs) << " ms\n"
                  << "RRT: step " << params.step << " m, goal bias " << params.goal_bias << ", max "
                  << params.max_iters << " iters, seeds 0.." << runs - 1 << "\n\n";

        std::cout << "| Planner | Success | Path length (m) median [min-max] | Time (ms) mean | Size |\n"
                  << "|---|---|---|---|---|\n";
        if (cells.empty()) {
            std::cout << "| A* | 0/1 | - | " << fmt(mean(astarMs), 3) << " | - |\n";
        } else {
            std::cout << "| A* (grid) | 1/1 | " << fmt(drone::pathLength(astarPath)) << " | "
                      << fmt(mean(astarMs), 3) << " | " << cells.size() << " cells |\n";
        }
        if (rrtLen.empty()) {
            std::cout << "| RRT | 0/" << runs << " | - | " << fmt(mean(rrtMs), 3) << " | - |\n";
        } else {
            std::cout << "| RRT | " << rrtLen.size() << "/" << runs << " | " << fmt(median(rrtLen)) << " ["
                      << fmt(*std::min_element(rrtLen.begin(), rrtLen.end())) << "-"
                      << fmt(*std::max_element(rrtLen.begin(), rrtLen.end())) << "] | " << fmt(mean(rrtMs), 3)
                      << " | " << fmt(median(rrtNodes), 0) << " nodes (median) |\n";
        }

        if (!outDir.empty()) {
            std::filesystem::create_directories(outDir);
            writePath(std::filesystem::path(outDir) / "astar_path.csv", astarPath);
            writePath(std::filesystem::path(outDir) / "rrt_path.csv", bestRrt);
            std::cout << "\nwrote " << outDir << "/astar_path.csv and rrt_path.csv (shortest RRT run)\n";
        }
        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
