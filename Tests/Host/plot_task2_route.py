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
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[2]


def plot(csv_path, output):
    paths = {0: [(-300.0, 0.0)], 1: [(-300.0, 0.0)]}
    with csv_path.open() as source:
        for stopped, x, y, _ in csv.reader(source):
            paths[int(stopped)].append((float(x), float(y)))
    matplotlib.rcParams["svg.fonttype"] = "none"
    matplotlib.rcParams["svg.hashsalt"] = "task2-fused-course"
    fig, ax = plt.subplots(figsize=(12, 5.8), layout="constrained")
    for stopped, color, width, label in (
        (1, "#ed9e29", 4, "B: stopped at each segment"),
        (0, "#1b63a5", 1.7, "A: fused (200 mm blends)"),
    ):
        x, y = zip(*paths[stopped])
        ax.plot(x, y, color=color, linewidth=width, label=label)
    for x, half_height, label in ((1100, 50, "1: 100 × 100"), (2250, 325, "2: 100 × 650")):
        ax.add_patch(Rectangle((x - 50, -half_height), 100, 2 * half_height,
                               facecolor="#495567", zorder=5))
        ax.text(x, -half_height - 95, label, ha="center", fontsize=9,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.9})
    # Open car park: rear wall and two side walls; no wall across x=0.
    ax.plot([0, -600, -600, 0], [250, 250, -250, -250], color="#495567", linewidth=3)
    ax.scatter([-300], [0], color="#19835c", s=40, zorder=8)
    ax.text(-300, -145, "Start / park\n(-300, 0)", ha="center", fontsize=9)
    ax.text(-300, 315, "Park: 600 × 500", ha="center", fontsize=9)
    for point, offset, text in (
        ((1100, 550), (0, 85), "1 · first left →"),
        ((2250, -550), (0, -135), "2 · second right →"),
        ((2600, 550), (0, 85), "3 · return ←"),
        ((1100, -550), (0, -135), "4 · first right ←"),
    ):
        ax.text(point[0] + offset[0], point[1] + offset[1], text, ha="center", fontsize=9)
    for start, end, y in ((0, 1050, 110), (1150, 2200, 150)):
        ax.annotate("", xy=(end, y), xytext=(start, y),
                    arrowprops={"arrowstyle": "<->", "color": "#687487"})
        ax.text((start + end) / 2, y + 25, "1050 face-to-face", ha="center", fontsize=8)
    ax.axhline(1000, color="#687487", linestyle=":", linewidth=1)
    ax.axhline(-1000, color="#687487", linestyle=":", linewidth=1)
    ax.set(xlim=(-700, 3300), ylim=(-1050, 1050), aspect="equal",
           xlabel="x [mm] — car-park mouth at 0; initial heading +x",
           ylabel="y [mm] — positive to the left",
           title="Task 2 figure eight · 9.31 m · 17 forward segments · return facing −x")
    ax.legend(loc="upper right", fontsize=8, framealpha=1)
    ax.grid(alpha=0.12)
    fused_x, fused_y = paths[0][-1]
    stopped_x, stopped_y = paths[1][-1]
    ax.text(0.5, -0.19,
            "Assumed clear width: 2000 mm. Second obstacle: midpoint of 300–1000 mm under this assumption.\n"
            f"Nominal endpoints: stopped ({stopped_x:.0f}, {stopped_y:.0f}); "
            f"fused ({fused_x:.0f}, {fused_y:+.1f}) mm. Physical parking accuracy requires a trial.",
            transform=ax.transAxes, ha="center", va="top", fontsize=9)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, metadata={"Date": None})
    if output.suffix.lower() == ".svg":
        output.write_text("\n".join(line.rstrip() for line in output.read_text().splitlines()) + "\n")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "Tests/task2_figure_eight.svg")
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
