#include "drone/astar.hpp"

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace drone {

std::vector<Cell> astar(const GridMap& grid, Cell start, Cell goal) {
    if (!grid.isFree(start) || !grid.isFree(goal)) return {};
    if (start == goal) return {start};

    const int n = grid.width * grid.height;
    std::vector<int> g(n, std::numeric_limits<int>::max());
    std::vector<int> cameFrom(n, -1);
    auto h = [&](Cell c) { return std::abs(c.x - goal.x) + std::abs(c.y - goal.y); };
    auto score = [&](Cell c) { return g[grid.index(c)] + h(c); };

    using Item = std::pair<int, int>;  // {f, index}
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    g[grid.index(start)] = 0;
    open.push({score(start), grid.index(start)});

    static constexpr int dx[4] = {1, -1, 0, 0};
    static constexpr int dy[4] = {0, 0, 1, -1};
    Cell cur = start;
    while (!open.empty()) {
        auto [f, curIdx] = open.top();
        open.pop();
        cur = Cell{curIdx % grid.width, curIdx / grid.width};
        if (cur == goal) break;
        if (f > score(cur)) continue;  // stale entry

        const int newG = g[curIdx] + 1;
        for (int d = 0; d < 4; ++d) {
            const Cell nb{cur.x + dx[d], cur.y + dy[d]};
            if (!grid.isFree(nb)) continue;
            const int nbIdx = grid.index(nb);
            if (newG < g[nbIdx]) {
                g[nbIdx] = newG;
                cameFrom[nbIdx] = curIdx;
                open.push({score(nb), nbIdx});
            }
        }
    }
    if (cur != goal) return {};

    std::vector<Cell> path;
    for (int idx = grid.index(goal); idx != -1; idx = cameFrom[idx]) {
        path.push_back(Cell{idx % grid.width, idx / grid.width});
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace drone
