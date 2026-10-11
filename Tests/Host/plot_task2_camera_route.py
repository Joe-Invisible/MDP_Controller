#!/usr/bin/env python3
"""Render camera viewing poses and both production-planner route geometries."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon, Rectangle, Wedge
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def body_corners(x, y, heading):
    forward = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-math.sin(heading), math.cos(heading)])
    return np.array([np.array([x, y]) + f * 150 * forward + s * 100 * left
                     for f, s in [(1, 1), (1, -1), (-1, -1), (-1, 1)]])


def scene(ax, clearance):
    for x, half_height in [(1100, 50), (2250, 250)]:
        ax.add_patch(Rectangle((x - 50, -half_height), 100, 2 * half_height,
                               color="#465568", zorder=5))
        ax.plot([x - 50, x - 50], [-50, 50], color="#df6650", linewidth=4, zorder=6)
        style = dict(color="#bd6d6d", linestyle="--", linewidth=0.9)
        ax.plot([x-50, x+50], [half_height+clearance]*2, **style)
        ax.plot([x-50, x+50], [-half_height-clearance]*2, **style)
        ax.plot([x-50-clearance]*2, [-half_height, half_height], **style)
        ax.plot([x+50+clearance]*2, [-half_height, half_height], **style)
        for dx, dy, start, end in [(50, half_height, 0, 90), (-50, half_height, 90, 180),
                                    (-50, -half_height, 180, 270), (50, -half_height, 270, 360)]:
            ax.add_patch(Arc((x+dx, dy), clearance*2, clearance*2,
                             theta1=start, theta2=end, **style))
    ax.plot([0, -500, -500, 0], [300, 300, -300, -300], color="#465568", linewidth=2)
    for y in [-750, 750]:
        ax.axhline(y, color="#8767aa", linestyle=":", linewidth=1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.13)


def view_cone(ax, view, color):
    x, y = view["capture_camera_xy_mm"]
    heading = view["capture_yaw_deg"]
    ax.add_patch(Wedge((x, y), 500, heading-30, heading+30,
                       facecolor=color, edgecolor=color, alpha=0.11, linewidth=1.2))
    target = (1050 if view["arrow"] == 1 else 2200, 0)
    ax.plot([x, target[0]], [y, target[1]], color=color, linewidth=1.2, linestyle="--")
    ax.scatter([x], [y], s=28, color=color, zorder=8)
    ax.add_patch(Polygon(body_corners(*view["capture_axle_xy_mm"], math.radians(heading)),
                         facecolor=color, edgecolor=color, alpha=0.16))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, default=ROOT/"Tests/task2_camera_route_plan.json")
    parser.add_argument("--traces", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT/"Tests/task2_camera_route.svg")
    args = parser.parse_args()
    data = json.loads(args.plan.read_text())
    poses = np.loadtxt(args.traces/"nominal.csv", delimiter=",")
    executed = np.loadtxt(args.traces/"executed_b.csv", delimiter=",")
    matplotlib.rcParams["svg.fonttype"] = "none"
    matplotlib.rcParams["svg.hashsalt"] = "task2-camera-five-arcs"
    fig = plt.figure(figsize=(13, 9), layout="constrained")
    grid = fig.add_gridspec(2, 2, height_ratios=[1.5, 1])
    overview = fig.add_subplot(grid[0, :])
    zooms = [fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])]
    for ax in [overview] + zooms:
        scene(ax, data["required_body_clearance_mm"])
        for run, color, name in [(1, "#e49a27", "B: nominal stopped route"),
                                  (0, "#2465a5", "A: production 500 mm blends")]:
            route = poses[poses[:, 0] == run]
            ax.plot(route[:, 1], route[:, 2], color=color, linewidth=2, label=name)
        ax.plot(executed[:, 1], executed[:, 2], color="#906925", linestyle="--",
                linewidth=1, label="B: ideal executed route")
    views = data["camera_views"]["A_nominal"]["views_100mm"]
    for view in views:
        view_cone(overview, view, "#208863")
    overview.scatter([-250], [0], color="#208863", zorder=8)
    overview.text(-250, -230, "Start / park", ha="center", fontsize=9)
    overview.text(1100, -210, "Obstacle 1: left pass", ha="center", fontsize=9)
    overview.text(2240, -690, "Obstacle 2: right pass", ha="center", fontsize=9)
    overview.text(2750, 60, "One back arc\nNo back boundary", ha="center", fontsize=9)
    overview.text(2840, 780, "500 mm beyond each obstacle-2 end", ha="center", fontsize=8,
                  color="#73528e")
    overview.set_xlim(-600, 3350)
    overview.set_ylim(-850, 850)
    overview.set_title("Camera-aware candidate: five arcs, no straight approach primitives · 7.81 m", fontsize=14)
    overview.legend(loc="upper left", fontsize=8)
    for i, (ax, view) in enumerate(zip(zooms, views), 1):
        view_cone(ax, view, "#208863")
        x, y = view["capture_camera_xy_mm"]
        target = 1050 if i == 1 else 2200
        ax.set_xlim(target-650, target+130)
        ax.set_ylim(-150, 500)
        ax.set_title(f"Arrow {i}: {view['capture_range_mm']:.0f} mm range, "
                     f"{view['capture_bearing_deg']:+.1f}° bearing", fontsize=11)
        ax.text(0.03, 0.04, "60° horizontal FoV; 500 mm centre range\n"
                "Red face segment: assumed 100 mm-wide target",
                transform=ax.transAxes, fontsize=8,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.95))
    fig.supxlabel("x [mm] — initial heading +x; positive y is left")
    fig.supylabel("y [mm]")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, metadata={"Date": None})
    if args.output.suffix == ".svg":
        args.output.write_text("\n".join(line.rstrip() for line in args.output.read_text().splitlines())+"\n")
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
