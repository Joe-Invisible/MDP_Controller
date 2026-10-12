#!/usr/bin/env python3
"""Plot default O1 host traces. No servo/tyre/camera plant is modelled."""
import argparse
from pathlib import Path
import re
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = (ROOT / "Tests/Src/Obstacle1Route.c").read_text()
    def number(name):
        return float(re.search(rf"\.{name}\s*=\s*([\d.]+)f", source)[1])
    front, buffer = number("parkToObstacleMm"), number("obstacleBufferMm")
    lateral = number("arenaHalfWidthMm")
    plt.rcParams["svg.fonttype"] = "none"
    plt.rcParams["svg.hashsalt"] = "isolated-o1"
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, sharey=True)
    for direction, axis in zip("LR", axes):
        data = np.loadtxt(args.traces / f"{direction}.csv", delimiter=",")
        for run, colour, label in [(0, "#1765ad", "A fused"), (1, "#cc691a", "B stopped")]:
            path = data[data[:, 0] == run]
            shown = np.vstack([[-1, 0, -250, 0, 0], path[::30], path[-1]])
            axis.plot(shown[:, 2], shown[:, 3], color=colour, lw=1.8, label=label)
            axis.scatter(path[-1, 2], path[-1, 3], s=18, color=colour)
        axis.add_patch(FancyBboxPatch((front-buffer, -50-buffer), 100+2*buffer, 100+2*buffer,
            boxstyle=f"round,pad=0,rounding_size={buffer}", facecolor="#e2e6eb",
            edgecolor="#9ba6b3", ls="--", lw=0.8))
        axis.add_patch(Rectangle((front, -50), 100, 100, color="#424952"))
        axis.text(front+50, 0, "O1", color="white", ha="center", va="center", fontsize=9)
        axis.plot([-500,-500,0], [300,-300,-300], color="#424952")
        axis.plot([-500,0], [300,300], color="#424952")
        axis.add_patch(Rectangle((-250+number("bodyCenterAheadOfRearAxleMm")-number("bodyLengthMm")/2,
            -number("bodyWidthMm")/2),number("bodyLengthMm")+number("frontProtrusionMm"),
            number("bodyWidthMm"),fill=False,edgecolor="#5b6978"))
        recognitionX = front - (number("bodyCenterAheadOfRearAxleMm")+number("bodyLengthMm")/2-number("cameraBehindFrontMm")) - number("recognitionCameraRangeMm")
        axis.scatter(recognitionX, 0, color="#287d53", marker="D", s=25, zorder=6)
        if direction == 'L':
            axis.annotate("Recognition → accelerate", (recognitionX,0), xytext=(100,-240),
                arrowprops={"arrowstyle":"-", "color":"#287d53"}, color="#287d53",fontsize=9)
        for side in (-lateral, lateral):
            axis.axhline(side,color="#b33737",ls="--",lw=1)
        axis.axhline(0,color="#84909d",ls=":",lw=0.8)
        axis.set_title(f"O1 arrow {'LEFT' if direction == 'L' else 'RIGHT'}",loc="left",fontsize=11)
        axis.set_aspect("equal")
        axis.grid(alpha=0.14)
        axis.set_ylabel("Lateral position (mm)",fontsize=9)
        axis.set_xlim(-550,data[:,2].max()+180)
        axis.set_ylim(-lateral-60,lateral+60)
        axis.tick_params(labelsize=8)
    axes[0].legend(loc="upper right",fontsize=9)
    axes[-1].set_xlabel("Forward position from park mouth (mm)",fontsize=9)
    fig.suptitle("O1 alone · 500 / 500 / 1500 mm arcs",x=0.08,ha="left",fontsize=17,weight="bold")
    fig.text(0.08,0.02,"Ideal host execution. Grey: 100 mm obstacle buffer; red: lateral arena boundaries.\n"
             "Paths show the rear axle; preflight checks the measured chassis plus sensor protrusion. O2 is absent.",
             fontsize=9,color="#465363")
    fig.subplots_adjust(left=0.08,right=0.98,bottom=0.14,top=0.9,hspace=0.28)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=160,metadata={"Date":None} if args.output.suffix == '.svg' else None)
    if args.output.suffix == ".svg":
        args.output.write_text("\n".join(line.rstrip() for line in args.output.read_text().splitlines()) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
