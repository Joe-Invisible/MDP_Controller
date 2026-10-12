#!/usr/bin/env python3
"""Plot the standalone route produced by the actual C planner and preflight."""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PROBE = r'''
#include "Task2Route.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
    if (argc != 6) return 2;
    Task2RouteConfig c = task2RouteConfig;
    if (atoi(argv[1])) c.firstArrow = atoi(argv[1]);
    if (atoi(argv[2])) c.secondArrow = atoi(argv[2]);
    if (atof(argv[3])) c.parkToObstacle1Mm = atof(argv[3]);
    if (atof(argv[4])) c.obstacleFaceGapMm = atof(argv[4]);
    if (atof(argv[5])) c.obstacle2WidthMm = atof(argv[5]);
    Task2Route route;
    if (!Task2Route_Create(&c, &route)) { fprintf(stderr,"Preflight: %s\n",route.report.reason); return 1; }
    printf("{\"first\":%d,\"second\":%d,\"o1\":%.6f,\"gap\":%.6f,\"width\":%.6f,",c.firstArrow,c.secondArrow,c.parkToObstacle1Mm,c.obstacleFaceGapMm,c.obstacle2WidthMm);
    printf("\"buffer\":%.6f,\"side\":%.6f,\"body_length\":%.6f,\"body_width\":%.6f,\"body_centre\":%.6f,\"protrusion\":%.6f,",c.obstacleBufferMm,c.obstacle2EndSpaceMm+c.obstacle2WidthMm/2,c.bodyLengthMm,c.bodyWidthMm,c.bodyCenterAheadOfRearAxleMm,c.frontProtrusionMm);
    printf("\"camera_offset\":%.6f,\"recognition_range\":%.6f,\"total\":%.6f,\"minimum_gap\":%.6f,",c.bodyCenterAheadOfRearAxleMm+c.bodyLengthMm/2-c.cameraBehindFrontMm,c.recognitionCameraRangeMm,route.totalTravelMm,route.report.minimumObstacleGapMm);
    printf("\"recognition\":[");
    for(int i=0;i<2;i++)printf("%s[%.6f,%.6f,%.6f]",i?",":"",route.report.recognitionPose[i].xMm,route.report.recognitionPose[i].yMm,route.report.recognitionPose[i].yawRad);
    printf("],\"steps\":[");
    for(int i=0;i<8;i++)printf("%s[%.6f,%.6f,%.6f]",i?",":"",route.steps[i].distanceMm,route.steps[i].radiusMm,route.steps[i].speedCps);
    puts("]}"); return 0;
}
'''


def planned_route(args):
    with tempfile.TemporaryDirectory(prefix="task2-route-plot-") as directory:
        work = Path(directory)
        (work / "probe.c").write_text(PROBE)
        command = shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", f"-I{ROOT / 'Tests/Inc'}",
            str(work / "probe.c"), str(ROOT / "Tests/Src/Task2Route.c"), "-lm", "-o", str(work / "probe")]
        subprocess.run(command, check=True)
        directions = {None: 0, "left": 1, "right": -1}
        result = subprocess.run([str(work / "probe"), str(directions[args.first_arrow]),
            str(directions[args.second_arrow]), str(args.park_to_o1 or 0), str(args.gap or 0),
            str(args.width or 0)], check=True, capture_output=True, text=True)
        return json.loads(result.stdout)


def body(pose, c):
    rear = c["body_centre"] - c["body_length"] / 2
    front = c["body_centre"] + c["body_length"] / 2 + c["protrusion"]
    points = np.array([[rear, -c["body_width"]/2], [front, -c["body_width"]/2],
                       [front, c["body_width"]/2], [rear, c["body_width"]/2]])
    x, y, angle = pose
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    return points @ rotation.T + [x, y]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-arrow", choices=("left", "right"))
    parser.add_argument("--second-arrow", choices=("left", "right"))
    parser.add_argument("--park-to-o1", type=float, help="Measured/configured park mouth to O1 face, mm")
    parser.add_argument("--gap", type=float, help="O1 back face to O2 front face, mm")
    parser.add_argument("--width", type=float, help="O2 width, mm")
    parser.add_argument("--output", type=Path, default=ROOT / "Tests/task2_motion_route.svg")
    args = parser.parse_args()
    c = planned_route(args)
    plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "task2-stopped", "font.size": 10})
    fig = plt.figure(figsize=(12, 6.8), facecolor="white")
    axis = fig.add_axes([0.07, 0.22, 0.70, 0.66])
    notes = fig.add_axes([0.80, 0.23, 0.19, 0.60]); notes.axis("off")
    colours = ["#506888", "#007c91", "#ad5b06", "#1a8a60", "#746bb0", "#bd4668", "#3d67b0", "#7d5697"]
    names = ["Approach O1", "O1 departure", "O1 bypass", "O1 recenter", "Approach O2", "O2 departure", "O2 back arc", "Return to park"]
    faces = [c["o1"], c["o1"] + 100 + c["gap"]]
    for i, (face, width) in enumerate(zip(faces, [100, c["width"]])):
        axis.add_patch(FancyBboxPatch((face-c["buffer"], -width/2-c["buffer"]),
            100+2*c["buffer"], width+2*c["buffer"], boxstyle=f'round,pad=0,rounding_size={c["buffer"]}',
            facecolor="#e8ebef", edgecolor="#9aa5af", linestyle="--", linewidth=0.9))
        axis.add_patch(Rectangle((face, -width/2), 100, width, color="#354350"))
        axis.text(face+50, 0, f"O{i+1}", ha="center", va="center", color="white", weight="bold")
    axis.plot([-500, -500, 0], [300, -300, -300], color="#354350", linewidth=1.6)
    axis.plot([-500, 0], [300, 300], color="#354350", linewidth=1.6)
    for side in [-c["side"], c["side"]]:
        axis.axhline(side, color="#b04d50", linestyle="--", linewidth=0.9)
    axis.axhline(0, color="#9faab4", linestyle=":", linewidth=0.8)
    pose = np.array([-250.0, 0.0, 0.0]); boundaries = [pose.copy()]; x_max = pose[0]
    for i, (distance, radius, speed) in enumerate(c["steps"]):
        ds = np.linspace(0, distance, max(4, int(distance / 3)+1))
        if radius == 0:
            points = np.column_stack((pose[0]+ds*np.cos(pose[2]), pose[1]+ds*np.sin(pose[2])))
            angles = np.full_like(ds, pose[2])
        else:
            angles = pose[2]+ds/radius
            points = np.column_stack((pose[0]+radius*(np.sin(angles)-np.sin(pose[2])),
                                      pose[1]+radius*(np.cos(pose[2])-np.cos(angles))))
        x_max = max(x_max, float(points[:,0].max()))
        axis.plot(points[:,0], points[:,1], color=colours[i], linewidth=2.3, zorder=4)
        mid = len(points)//2
        offset = (0, 14) if i in (1, 2, 3, 5, 6) else (0, -22)
        if i == 4: offset = (32, -30)
        axis.annotate(str(i+1), points[mid], xytext=offset, textcoords="offset points",
            color=colours[i], weight="bold", ha="center", fontsize=11,
            bbox={"boxstyle":"circle,pad=0.22", "fc":"white", "ec":colours[i], "lw":1},
            arrowprops={"arrowstyle":"-", "color":colours[i], "lw":0.8} if i == 4 else None)
        pose = np.array([points[-1,0], points[-1,1], angles[-1]])
        boundaries.append(pose.copy())
        radius_text = "straight" if radius == 0 else f"R {radius:+.0f} mm"
        notes.text(0, 1-i*0.095, f"{i+1}  {names[i]}", color=colours[i], weight="bold", fontsize=10)
        notes.text(0, 0.962-i*0.095, f"{distance:.0f} mm · {radius_text}", color="#5c6975", fontsize=9)
    for pose in boundaries[1:-1]:
        axis.plot(pose[0], pose[1], "o", color="#354350", markerfacecolor="white", markersize=4, zorder=6)
    for pose, style in [(boundaries[0], "--"), (boundaries[-1], "-")]:
        axis.add_patch(Polygon(body(pose,c), fill=False, edgecolor="#354350", linestyle=style, linewidth=1.0))
        axis.arrow(pose[0],pose[1],55*math.cos(pose[2]),55*math.sin(pose[2]),head_width=18,color="#354350",zorder=7)
    axis.annotate("Start / finish", (-250,0), xytext=(-490,-470), arrowprops={"arrowstyle":"-", "color":"#5c6975"}, color="#5c6975", fontsize=9)
    for i, (x,y,angle) in enumerate(c["recognition"]):
        lens = (x+c["camera_offset"]*math.cos(angle),y+c["camera_offset"]*math.sin(angle))
        axis.plot([lens[0],faces[i]],[lens[1],0],color="#33946f",alpha=0.5,linestyle="--",linewidth=0.8)
        axis.plot(x,y,"D",color="#33946f",markersize=5,zorder=7)
    axis.set_aspect("equal"); axis.grid(alpha=0.12)
    axis.set_xlabel("Forward position from park mouth (mm)"); axis.set_ylabel("Lateral position · positive = left (mm)")
    axis.set_xlim(-550,x_max+180)
    axis.set_ylim(-c["side"]-100,c["side"]+100)
    arrows = lambda value: "LEFT" if value == 1 else "RIGHT"
    fig.text(0.07,0.945,"Task 2 · stopped primitive route",fontsize=20,weight="bold",color="#273644")
    fig.text(0.07,0.90,f'O1 {arrows(c["first"])} · O2 {arrows(c["second"])} · O2 width {c["width"]:.0f} mm · travel {c["total"]/1000:.2f} m',fontsize=11,color="#5c6975")
    notes.text(0,0.10,f'Nominal body clearance\n{c["minimum_gap"]:.1f} mm minimum',fontsize=10,color="#273644")
    fig.text(0.07,0.11,f'Circles: stopped primitive boundaries. Green diamonds: nominal camera recognition poses.\nGrey: {c["buffer"]:.0f} mm obstacle buffer. Path tracks the rear axle; preflight checks the complete footprint.',fontsize=9,color="#5c6975")
    fig.text(0.07,0.045,f'Configured layout: park → O1 {c["o1"]:.0f} mm; O1 back → O2 {c["gap"]:.0f} mm. No rear arena boundary.\nNominal planning geometry; physical tracking and startup ultrasonic error are not represented.',fontsize=9,color="#5c6975")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output,dpi=180,metadata={"Date":None} if args.output.suffix==".svg" else None)
    if args.output.suffix==".svg":
        args.output.write_text("\n".join(line.rstrip() for line in args.output.read_text().splitlines())+"\n")
    print(args.output)

if __name__ == "__main__":
    main()
