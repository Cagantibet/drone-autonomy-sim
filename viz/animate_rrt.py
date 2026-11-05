#!/usr/bin/env python3
"""Animate RRT tree growth and the final path from plan_rrt output, saved as a GIF.

Usage:
    python3 viz/animate_rrt.py maps/forest.world build/rrt [-o rrt.gif] [--png rrt.png]
                               [--frames 60] [--fps 20] [--hold 2.0]

<out_dir> must contain tree.csv and path.csv written by plan_rrt.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection

from world2d import draw_world, load_world, read_points, read_tree

TREE_COLOR = "#7fa7d1"
PATH_COLOR = "#ff7f0e"


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def make_figure(world, title):
    fig, ax = plt.subplots(figsize=(8, 8 * world.height / world.width + 0.6))
    draw_world(ax, world, title)
    tree = LineCollection([], colors=TREE_COLOR, linewidths=0.8, zorder=3)
    ax.add_collection(tree)
    (path_line,) = ax.plot([], [], color=PATH_COLOR, linewidth=2.5, zorder=4)
    fig.tight_layout()
    return fig, ax, tree, path_line


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help=".world map file")
    ap.add_argument("out_dir", help="directory with tree.csv and path.csv from plan_rrt")
    ap.add_argument("-o", "--output", default="rrt.gif", help="output GIF (default: rrt.gif)")
    ap.add_argument("--png", help="also save a static image of the final frame")
    ap.add_argument("--frames", type=int, default=60, help="frames for tree growth (default: 60)")
    ap.add_argument("--fps", type=int, default=20, help="GIF frame rate (default: 20)")
    ap.add_argument("--hold", type=float, default=2.0, help="seconds to hold the final path (default: 2)")
    args = ap.parse_args()

    world = load_world(args.world)
    tree_csv = Path(args.out_dir) / "tree.csv"
    if not tree_csv.exists():
        sys.exit(f"error: {tree_csv} not found (run plan_rrt first)")
    nodes, parents = read_tree(tree_csv)
    path = read_points(Path(args.out_dir) / "path.csv")

    edges = [(nodes[p], nodes[i]) for i, p in enumerate(parents) if p >= 0]
    if path:
        title = f"RRT: {len(nodes)} nodes, path {path_length(path):.2f} m"
    else:
        title = f"RRT: {len(nodes)} nodes, no path found"
        print("note: path.csv is empty (no path found)")

    fig, ax, tree, path_line = make_figure(world, f"{Path(args.world).name}  |  {title}")

    grow = max(1, min(args.frames, len(edges)))
    hold = max(1, round(args.hold * args.fps))

    def update(frame):
        shown = len(edges) if frame >= grow else math.ceil(len(edges) * (frame + 1) / grow)
        tree.set_segments(edges[:shown])
        if frame >= grow and path:
            xs, ys = zip(*path)
            path_line.set_data(xs, ys)
        return tree, path_line

    anim = FuncAnimation(fig, update, frames=grow + hold, blit=False)
    anim.save(args.output, writer=PillowWriter(fps=args.fps))
    print(f"saved {args.output}")

    if args.png:
        update(grow + hold - 1)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=2, frameon=False)
        fig.savefig(args.png, dpi=150, bbox_inches="tight")
        print(f"saved {args.png}")


if __name__ == "__main__":
    main()
