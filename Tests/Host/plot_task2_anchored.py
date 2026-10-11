#!/usr/bin/env python3
"""Plot default-config ideal execution traces produced by test_task2_anchored.py.

This is a host-fixture illustration, not measured tracking or camera performance.
Run the host test with --traces DIR, then pass DIR and an output SVG/PNG here.
"""
import argparse
from pathlib import Path
import re

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def default_number(name):
    source = (ROOT / "Tests/Src/Task2Route.c").read_text()
    return float(re.search(rf"\.{name}\s*=\s*([\d.]+)f", source)[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    first_face = default_number("parkToObstacle1Mm")
    second_face = first_face + 100 + default_number("obstacleFaceGapMm")
    width = default_number("obstacle2WidthMm")
    side_limit = width / 2 + default_number("obstacle2EndSpaceMm")
    plt.rcParams["svg.fonttype"] = "none"
    plt.rcParams["svg.hashsalt"] = "task2-anchored"
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True, sharey=True)
    colours = ["#1765ad", "#cc691a"]
    for label, axis in zip(["LL", "LR", "RL", "RR"], axes.flat):
        data = np.loadtxt(args.traces / f"{label}.csv", delimiter=",")
        for run in (0, 1):
            path = data[data[:, 0] == run]
            shown = np.vstack([path[::40], path[-1]])
            axis.plot(shown[:, 2], shown[:, 3], color=colours[run], lw=1.6,
                      label=["A fused", "B stopped"][run])
        axis.add_patch(Rectangle((first_face, -50), 100, 100,
                                 color="#424952"))
        axis.add_patch(Rectangle((second_face, -width / 2), 100, width,
                                 color="#424952"))
        axis.text(first_face + 50, 0, "O1", color="white", ha="center", va="center", fontsize=8)
        axis.text(second_face + 50, 0, "O2", color="white", ha="center", va="center", fontsize=8)
        axis.plot([-500, -500, 0], [300, -300, -300], color="#424952")
        axis.plot([-500, 0], [300, 300], color="#424952")
        axis.add_patch(Rectangle((-400, -100), 300, 200, fill=False,
                                 edgecolor="#5b6978", lw=1))
        for side in (-side_limit, side_limit):
            axis.axhline(side, color="#b33737", ls="--", lw=1)
        axis.axhline(0, color="#84909d", ls=":", lw=0.8)
        axis.set_title(f"O1 {label[0]} / O2 {label[1]}", loc="left", fontsize=11)
        axis.set_aspect("equal")
        axis.grid(alpha=0.14)
        axis.tick_params(labelsize=8)
        axis.set_ylim(-side_limit - 80, side_limit + 80)
        axis.set_xlim(-550, data[:, 2].max() + 180)
    for axis in axes[:, 0]:
        axis.set_ylabel("Left / right position (mm)", fontsize=9)
    for axis in axes[-1]:
        axis.set_xlabel("Forward position from park mouth (mm)", fontsize=9)
    axes[0, 0].legend(loc="upper left", fontsize=8)
    fig.suptitle("Eight primitives · four independent arrow combinations", x=0.07,
                 ha="left", fontsize=16, weight="bold")
    fig.text(0.07, 0.035,
             "Host execution only. Dashed red: 500 mm beyond each O2 end. No rear boundary.\n"
             "Default widths/distances and 100 mm nominal body buffer; physical accuracy remains unvalidated.",
             fontsize=9, color="#465363")
    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.17, top=0.88, wspace=0.12, hspace=0.2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, metadata={"Date": None} if args.output.suffix == ".svg" else None)
    print(args.output)


if __name__ == "__main__":
    main()
