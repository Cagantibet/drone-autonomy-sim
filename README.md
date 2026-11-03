# Drone Autonomy Simulator

![CI](https://github.com/Cagantibet/drone-autonomy-sim/actions/workflows/ci.yml/badge.svg)

A 2D drone autonomy simulator built from scratch in **C++17**, with Python visualization.
The drone plans a route through an obstacle map, and later milestones add path
following with a PID controller, sampling-based planning (RRT), and reactive obstacle
avoidance.

The focus is clean, tested, interview-grade implementations of core robotics
algorithms rather than wrapping existing libraries.

## Results

A* finds the optimal 36-move route through a 21×13 maze (4-connected, unit cost):

![A* path through a maze](docs/maze.png)

## Features

- **Grid world:** occupancy grid loaded from a plain-text map format, with strict validation (ragged rows, unknown characters, missing or duplicate start/goal).
- **A\* path planning:** 4-connected search with a Manhattan heuristic, a binary-heap open set, lazy deletion of stale entries, and flat `g`/parent arrays for cache-friendly lookups.
- **Drone state:** position and velocity as Eigen vectors, with simple Euler integration.
- **CLI:** `plan_path` loads a map, plans, and exports the path as CSV.
- **Visualization:** a matplotlib script that renders the grid, obstacles, start, goal, and path.
- **Testing:** 20 GoogleTest unit tests, including optimal-length checks verified against BFS and path-validity checks (endpoints, free cells, 4-adjacency).
- **CI:** GitHub Actions builds and runs the tests on every push.

## Build and run

Requirements: CMake ≥ 3.16, a C++17 compiler, Eigen 3, and Python 3 with matplotlib.
GoogleTest is fetched automatically by CMake.

```bash
# Dependencies
#   macOS:  brew install cmake eigen
#   Ubuntu: sudo apt install cmake libeigen3-dev
pip install matplotlib

# Build and test
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure

# Plan a path and plot it
./build/plan_path maps/maze.txt build/path.csv
python3 viz/plot_world.py maps/maze.txt build/path.csv -o maze.png
```

`plan_path` exits with `0` if a path is found, `2` if there's no path, and `1` on bad input.

### Map format

One character per cell. All rows have the same width, and blank lines are ignored.

| Char | Meaning             |
|------|---------------------|
| `.`  | free                |
| `#`  | obstacle            |
| `S`  | start (exactly one) |
| `G`  | goal (exactly one)  |

`x` is the column and `y` is the row, with row 0 at the top of the file.

## Project layout

```
include/drone/   public headers (grid, drone, astar)
src/             core library (drone_core)
apps/            CLI tools (plan_path)
tests/           GoogleTest unit tests
maps/            sample worlds
viz/             Python plotting
docs/            images used in this README
```

## Roadmap

- [x] **Milestone 1:** grid world, map loading, A* planning, plotting, CI
- [ ] **Milestone 2:** PID controller that follows the planned path with continuous drone dynamics, plus an animated GIF
- [ ] **Milestone 3:** RRT / RRT* planning in continuous space, compared with A*
- [ ] **Milestone 4:** reactive obstacle avoidance with dynamic obstacles and replanning
- [ ] **Milestone 5:** learned policy (RL) for navigation, benchmarked against the classical stack
