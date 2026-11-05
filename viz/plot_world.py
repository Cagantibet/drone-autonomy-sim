#!/usr/bin/env python3
"""Plot a grid world (obstacles, start, goal) and optionally a planned path.

Usage:
    python3 viz/plot_world.py maps/simple.txt [build/path.csv] [-o world.png] [--show]
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def load_map(path):
    rows = [line.rstrip("\r\n") for line in Path(path).read_text().splitlines()]
    rows = [r for r in rows if r]
    if not rows:
        sys.exit(f"error: {path} is empty")
    width = len(rows[0])
    if any(len(r) != width for r in rows):
        sys.exit(f"error: {path} has ragged rows")

    grid = [[1 if ch == "#" else 0 for ch in r] for r in rows]
    start = goal = None
    for y, r in enumerate(rows):
        for x, ch in enumerate(r):
            if ch == "S":
                start = (x, y)
            elif ch == "G":
                goal = (x, y)
    return grid, start, goal


def load_path(path):
    with open(path, newline="") as f:
        return [(int(row["x"]), int(row["y"])) for row in csv.DictReader(f)]


def plot(grid, start, goal, path, title):
    height, width = len(grid), len(grid[0])
    fig, ax = plt.subplots(figsize=(max(4, width * 0.4), max(3, height * 0.4)))

    # Cell (x, y) is centered at (x, y); row 0 drawn at the top like the map file.
    ax.imshow(grid, cmap=ListedColormap(["#f7f7f7", "#2b2b2b"]), origin="upper",
              extent=(-0.5, width - 0.5, height - 0.5, -0.5), interpolation="nearest")

    ax.set_xticks([x - 0.5 for x in range(width + 1)], minor=True)
    ax.set_yticks([y - 0.5 for y in range(height + 1)], minor=True)
    ax.grid(which="minor", color="#d0d0d0", linewidth=0.5)
    ax.tick_params(which="both", length=0)
    ax.set_xticks([])
    ax.set_yticks([])

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color="#1f77b4", linewidth=2.5, label=f"A* path ({len(path) - 1} moves)")
    if start:
        ax.plot(*start, marker="o", markersize=10, color="#2ca02c", linestyle="none", label="start")
    if goal:
        ax.plot(*goal, marker="*", markersize=14, color="#d62728", linestyle="none", label="goal")

    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=3, frameon=False)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="map text file")
    ap.add_argument("path", nargs="?", help="path CSV from plan_path (optional)")
    ap.add_argument("-o", "--output", default="world.png", help="output image (default: world.png)")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    grid, start, goal = load_map(args.map)
    path = load_path(args.path) if args.path else []
    if args.path and not path:
        print("note: path CSV is empty (no path found)")

    fig = plot(grid, start, goal, path, Path(args.map).name)
    fig.savefig(args.output, dpi=150)
    print(f"saved {args.output}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
