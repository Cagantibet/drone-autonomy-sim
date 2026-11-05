#!/usr/bin/env python3
"""Overlay the A* and RRT paths written by compare_planners on one map.

Usage:
    python3 viz/plot_compare.py maps/rooms.world build/compare [-o compare.png] [--show]

<out_dir> must contain astar_path.csv and rrt_path.csv from compare_planners --out.
"""

import argparse
import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

from world2d import draw_world, load_world, read_points

STYLES = {
    "astar_path.csv": ("A* (grid)", "#1f77b4", "-"),
    "rrt_path.csv": ("RRT (best seed)", "#ff7f0e", "-"),
}


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help=".world map file")
    ap.add_argument("out_dir", help="directory with astar_path.csv and rrt_path.csv")
    ap.add_argument("-o", "--output", default="compare.png", help="output image (default: compare.png)")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    world = load_world(args.world)
    fig, ax = plt.subplots(figsize=(8, 8 * world.height / world.width + 0.6))
    draw_world(ax, world, f"{Path(args.world).name}: A* vs RRT")

    for fname, (label, color, ls) in STYLES.items():
        path = read_points(Path(args.out_dir) / fname)
        if not path:
            print(f"note: {fname} is empty")
            continue
        xs, ys = zip(*path)
        ax.plot(xs, ys, color=color, linestyle=ls, linewidth=2.2, zorder=4,
                label=f"{label}: {path_length(path):.2f} m")

    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"saved {args.output}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
