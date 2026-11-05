"""Shared helpers for continuous .world maps: loading, CSV readers, and drawing."""

import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path

from matplotlib.patches import Circle, Rectangle

OBSTACLE_COLOR = "#4a4a4a"
START_COLOR = "#2ca02c"
GOAL_COLOR = "#d62728"


@dataclass
class World2D:
    width: float = 0.0
    height: float = 0.0
    radius: float = 0.0
    start: tuple = (0.0, 0.0)
    goal: tuple = (0.0, 0.0)
    circles: list = field(default_factory=list)  # (cx, cy, r)
    rects: list = field(default_factory=list)    # (x0, y0, x1, y1)


def load_world(path):
    """Lenient mirror of the C++ parser (the C++ side does the strict validation)."""
    w = World2D()
    for lineno, raw in enumerate(Path(path).read_text().splitlines(), 1):
        tokens = raw.split("#", 1)[0].split()
        if not tokens:
            continue
        key, nums = tokens[0], tokens[1:]
        try:
            vals = [float(t) for t in nums]
        except ValueError:
            sys.exit(f"error: {path}:{lineno}: bad number in '{raw.strip()}'")
        if key == "bounds":
            w.width, w.height = vals
        elif key == "radius":
            (w.radius,) = vals
        elif key == "start":
            w.start = tuple(vals)
        elif key == "goal":
            w.goal = tuple(vals)
        elif key == "circle":
            w.circles.append(tuple(vals))
        elif key == "rect":
            w.rects.append(tuple(vals))
        else:
            sys.exit(f"error: {path}:{lineno}: unknown directive '{key}'")
    return w


def read_points(path):
    """x,y CSV -> list of (x, y). Missing or header-only file -> []."""
    p = Path(path)
    if not p.exists():
        return []
    with p.open(newline="") as f:
        return [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]


def read_tree(path):
    """id,parent,x,y CSV -> (nodes, parents) in insertion order."""
    nodes, parents = [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            nodes.append((float(r["x"]), float(r["y"])))
            parents.append(int(r["parent"]))
    return nodes, parents


def draw_world(ax, w, title=None):
    """Bounds, obstacles, start and goal on a y-up, equal-aspect axis."""
    ax.set_xlim(0, w.width)
    ax.set_ylim(0, w.height)
    ax.set_aspect("equal")
    ax.set_facecolor("#f7f7f7")
    for cx, cy, r in w.circles:
        ax.add_patch(Circle((cx, cy), r, color=OBSTACLE_COLOR, zorder=2))
    for x0, y0, x1, y1 in w.rects:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=OBSTACLE_COLOR, zorder=2))
    ax.plot(*w.start, marker="o", markersize=9, color=START_COLOR, linestyle="none", zorder=5, label="start")
    ax.plot(*w.goal, marker="*", markersize=14, color=GOAL_COLOR, linestyle="none", zorder=5, label="goal")
    ax.set_xticks([])
    ax.set_yticks([])
    if title:
        ax.set_title(title)
