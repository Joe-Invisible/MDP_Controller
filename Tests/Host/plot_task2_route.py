#!/usr/bin/env python3
"""Plot the Task 2 course evaluated by the production planner in the host test.

Run: python3 Tests/Host/plot_task2_route.py
Requires matplotlib. --csv can reuse TASK2_ROUTE_CSV output from test_fused_motion.py.
"""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import math

ROOT = Path(__file__).resolve().parents[2]


def plot(csv_path, output):
    paths = {0: [(-250.0, 0.0)], 1: [(-250.0, 0.0)]}
    final_yaw = {}
    with csv_path.open() as source:
        for stopped, x, y, yaw in csv.reader(source):
            paths[int(stopped)].append((float(x), float(y)))
            final_yaw[int(stopped)] = float(yaw)
    matplotlib.rcParams["svg.fonttype"] = "none"
    matplotlib.rcParams["svg.hashsalt"] = "task2-fused-course"
    fig, ax = plt.subplots(figsize=(12, 5.8), layout="constrained")
    for stopped, color, width, label in (
        (1, "#ed9e29", 4, "B: stopped at each segment"),
        (0, "#1b63a5", 1.7, "A: fused (up to 200 mm blends)"),
    ):
        x, y = zip(*paths[stopped])
        ax.plot(x, y, color=color, linewidth=width, label=label)
    for x, half_height, label in ((1100, 50, "1: 100 × 100"), (2250, 250, "2: 100 × 500")):
        ax.add_patch(Rectangle((x - 50, -half_height), 100, 2 * half_height,
                               facecolor="#495567", zorder=5))
        ax.text(x, -half_height - 95, label, ha="center", fontsize=9,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.9})
    # Open car park: rear wall and two side walls; no wall across x=0.
    ax.plot([0, -500, -500, 0], [300, 300, -300, -300], color="#495567", linewidth=3)
    ax.scatter([-250], [0], color="#19835c", s=40, zorder=8)
    ax.text(-250, -190, "Start / park\n(-250, 0)", ha="center", fontsize=9)
    ax.text(-250, 365, "Park: 500 deep\n× 600 wide", ha="center", fontsize=9)
    for point, offset, text in (
        ((1100, 275), (0, 145), "1 · first left →"),
        ((2250, -475), (0, -135), "2 · second right →"),
        ((1800, 433), (0, 125), "3 · one straight return ↙"),
    ):
        ax.text(point[0] + offset[0], point[1] + offset[1], text, ha="center", fontsize=9)
    # The assumed centred footprint at the fused finish is entirely parked.
    px, py = paths[0][-1]
    heading = final_yaw[0]
    corners = [(px + f*150*math.cos(heading) - t*100*math.sin(heading),
                py + f*150*math.sin(heading) + t*100*math.cos(heading))
               for f, t in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    ax.add_patch(Polygon(corners, facecolor="#19835c", edgecolor="#19835c", alpha=0.18,
                         label="Assumed body at fused finish"))
    for start, end, y in ((0, 1050, 110), (1150, 2200, 150)):
        ax.annotate("", xy=(end, y), xytext=(start, y),
                    arrowprops={"arrowstyle": "<->", "color": "#687487"})
        ax.text((start + end) / 2, y + 25, "1050 face-to-face", ha="center", fontsize=8)
    ax.axhline(800, color="#687487", linestyle=":", linewidth=1)
    ax.axhline(-800, color="#687487", linestyle=":", linewidth=1)
    ax.set(xlim=(-600, 3300), ylim=(-850, 850), aspect="equal",
           xlabel="x [mm] — car-park mouth at 0; initial heading +x",
           ylabel="y [mm] — positive to the left",
           title="Task 2 outward S + straight return · 7.73 m · 10 forward segments")
    ax.legend(loc="upper right", fontsize=8, framealpha=1)
    ax.grid(alpha=0.12)
    fused_x, fused_y = paths[0][-1]
    stopped_x, stopped_y = paths[1][-1]
    if abs(stopped_y) < 0.0005:
        stopped_y = 0.0
    ax.text(0.5, -0.19,
            "Compact layout: 500 mm second obstacle, 1600 mm clear width; both face-to-face gaps remain 1050 mm.\n"
            f"Nominal endpoints: stopped ({stopped_x:.0f}, {stopped_y:.0f}); "
            f"fused ({fused_x:.1f}, {fused_y:+.1f}) mm, heading {math.degrees(heading):.1f}°. Physical parking requires a trial.",
            transform=ax.transAxes, ha="center", va="top", fontsize=9)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, metadata={"Date": None})
    if output.suffix.lower() == ".svg":
        output.write_text("\n".join(line.rstrip() for line in output.read_text().splitlines()) + "\n")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "Tests/task2_route.svg")
    args = parser.parse_args()
    if args.csv:
        plot(args.csv, args.output)
    else:
        with tempfile.TemporaryDirectory(prefix="task2-route-") as directory:
            csv_path = Path(directory) / "route.csv"
            env = dict(os.environ, TASK2_ROUTE_CSV=str(csv_path))
            subprocess.run([sys.executable, str(ROOT / "Tests/Host/test_fused_motion.py")],
                           check=True, env=env)
            plot(csv_path, args.output)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
