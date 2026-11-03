# Drone Autonomy Simulator

![CI](https://github.com/Cagantibet/drone-autonomy-sim/actions/workflows/ci.yml/badge.svg)

A 2D drone autonomy simulator: path planning (A*, RRT), PID path following, and
obstacle avoidance. Core in C++17, visualization in Python.

## Status

**Milestone 1:** grid world from text files, drone state, A* planning, path plotting, CI.

## Build & test

Requirements: CMake ≥ 3.16, a C++17 compiler, Eigen 3. GoogleTest is fetched automatically.

```bash
# macOS: brew install cmake eigen      Ubuntu: sudo apt install cmake libeigen3-dev
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Plan and plot a path

```bash
./build/plan_path maps/maze.txt build/path.csv
pip install matplotlib
python3 viz/plot_world.py maps/maze.txt build/path.csv -o maze.png
```

## Map format

One character per cell. Every row has the same width, and blank lines are ignored.

| Char | Meaning  |
|------|----------|
| `.`  | free     |
| `#`  | obstacle |
| `S`  | start (exactly one) |
| `G`  | goal (exactly one)  |

Coordinates: `x` = column, `y` = row, and row 0 is the first line of the file.

## Layout

```
include/drone/   public headers (grid, drone, astar)
src/             core library (drone_core)
apps/            CLI tools (plan_path)
tests/           GoogleTest unit tests
maps/            sample worlds
viz/             Python plotting
```
