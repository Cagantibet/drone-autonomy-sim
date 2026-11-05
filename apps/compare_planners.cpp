// compare_planners: A* (on a rasterized grid) vs raw and shortcut-smoothed RRT
// (continuous) on one .world map.
//
// Usage: compare_planners <map.world> [--res R] [--runs N] [--max-iters N]
//                         [--smooth-iters N] [--out DIR]
//   --res           grid cell size for A* in m (default 0.25)
//   --runs          RRT seeds 0..N-1, and A* timing repetitions (default 20)
//   --max-iters     RRT iteration budget (default 20000)
//   --smooth-iters  shortcutting rounds per RRT path (default 200)
//   --out           write astar_path.csv, rrt_path.csv and rrt_smooth_path.csv
//                   here; the RRT run shown is the one with median raw length
// Prints a markdown table. Exit codes: 0 = ok, 1 = usage/IO/parse error.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drone/astar.hpp"
#include "drone/rasterize.hpp"
#include "drone/rrt.hpp"
#include "drone/smoothing.hpp"
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

std::string lengthStats(const std::vector<double>& v) {
    return fmt(median(v)) + " [" + fmt(*std::min_element(v.begin(), v.end())) + "-" +
           fmt(*std::max_element(v.begin(), v.end())) + "]";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <map.world> [--res R] [--runs N] [--max-iters N] [--smooth-iters N] [--out DIR]\n";
        return 1;
    }
    try {
        double res = 0.25;
        int runs = 20;
        drone::RrtParams params;
        params.max_iters = 20000;
        int smoothIters = 200;
        std::string outDir;
        for (int i = 2; i < argc; ++i) {
            const std::string flag = argv[i];
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + flag);
            const std::string value = argv[++i];
            if (flag == "--res") res = std::stod(value);
            else if (flag == "--runs") runs = std::max(1, std::stoi(value));
            else if (flag == "--max-iters") params.max_iters = std::stoi(value);
            else if (flag == "--smooth-iters") smoothIters = std::stoi(value);
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

        // ---- RRT over seeds, each path then shortcut with the same seed ----
        struct Run {
            double rawLen, smoothLen;
            std::vector<drone::Vec2> raw, smooth;
        };
        std::vector<Run> solved;
        std::vector<double> rrtMs, smoothMs, rrtNodes, smoothPts;
        for (int seed = 0; seed < runs; ++seed) {
            std::mt19937 rng(static_cast<unsigned>(seed));
            t0 = Clock::now();
            const auto result = drone::rrt(world, params, rng);
            rrtMs.push_back(msSince(t0));
            if (result.path.empty()) continue;

            std::mt19937 smoothRng(static_cast<unsigned>(seed));
            t0 = Clock::now();
            auto smooth = drone::shortcutPath(world, result.path, smoothIters, smoothRng);
            smoothMs.push_back(msSince(t0));

            rrtNodes.push_back(result.tree.size());
            smoothPts.push_back(static_cast<double>(smooth.size()));
            solved.push_back({drone::pathLength(result.path), drone::pathLength(smooth), result.path,
                              std::move(smooth)});
        }
        std::vector<double> rawLen, smoothLen;
        for (const Run& r : solved) {
            rawLen.push_back(r.rawLen);
            smoothLen.push_back(r.smoothLen);
        }

        // ---- report ----
        std::cout << "Map: " << argv[1] << "  (" << world.width << " x " << world.height
                  << " m, drone radius " << world.drone_radius << " m)\n"
                  << "A*: " << grid.width << "x" << grid.height << " grid at " << res
                  << " m/cell, 4-connected, rasterized in " << fmt(rasterMs) << " ms\n"
                  << "RRT: step " << params.step << " m, goal bias " << params.goal_bias << ", max "
                  << params.max_iters << " iters, seeds 0.." << runs - 1 << "; shortcutting " << smoothIters
                  << " rounds\n\n";

        std::cout << "| Planner | Success | Path length (m) median [min-max] | Time (ms) mean | Size |\n"
                  << "|---|---|---|---|---|\n";
        if (cells.empty()) {
            std::cout << "| A* (grid) | 0/1 | - | " << fmt(mean(astarMs), 3) << " | - |\n";
        } else {
            std::cout << "| A* (grid) | 1/1 | " << fmt(drone::pathLength(astarPath)) << " | "
                      << fmt(mean(astarMs), 3) << " | " << cells.size() << " cells |\n";
        }
        const std::string success = std::to_string(solved.size()) + "/" + std::to_string(runs);
        if (solved.empty()) {
            std::cout << "| RRT | " << success << " | - | " << fmt(mean(rrtMs), 3) << " | - |\n";
        } else {
            std::cout << "| RRT (raw) | " << success << " | " << lengthStats(rawLen) << " | "
                      << fmt(mean(rrtMs), 3) << " | " << fmt(median(rrtNodes), 0) << " nodes (median) |\n"
                      << "| RRT + shortcut | " << success << " | " << lengthStats(smoothLen) << " | "
                      << fmt(mean(rrtMs) + mean(smoothMs), 3) << " (+" << fmt(mean(smoothMs), 3)
                      << ") | " << fmt(median(smoothPts), 0) << " waypoints (median) |\n";
        }

        if (!outDir.empty()) {
            // Show a typical run: the one whose raw length is closest to the median.
            std::vector<drone::Vec2> rawShown, smoothShown;
            if (!solved.empty()) {
                const double m = median(rawLen);
                const auto it = std::min_element(solved.begin(), solved.end(), [m](const Run& a, const Run& b) {
                    return std::abs(a.rawLen - m) < std::abs(b.rawLen - m);
                });
                rawShown = it->raw;
                smoothShown = it->smooth;
            }
            const std::filesystem::path dir = outDir;
            std::filesystem::create_directories(dir);
            writePath(dir / "astar_path.csv", astarPath);
            writePath(dir / "rrt_path.csv", rawShown);
            writePath(dir / "rrt_smooth_path.csv", smoothShown);
            std::cout << "\nwrote astar_path.csv, rrt_path.csv, rrt_smooth_path.csv to " << outDir << "\n";
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
