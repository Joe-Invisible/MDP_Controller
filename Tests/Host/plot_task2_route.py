#!/usr/bin/env python3
"""Plot production-planner Task 2 paths and explicit clearance assumptions.

Run: python3 Tests/Host/plot_task2_route.py
Requires matplotlib. --csv reuses TASK2_ROUTE_CSV from test_fused_motion.py.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon, Rectangle
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
SIDE_BOUNDARY_MM = 250 + 500
BODY_CLEARANCE_MM = 200


def corners(x, y, heading):
    return [(x+f*150*math.cos(heading)-s*100*math.sin(heading),
             y+f*150*math.sin(heading)+s*100*math.cos(heading))
            for f,s in ((1,1),(1,-1),(-1,-1),(-1,1))]


def plot(csv_path, output):
    paths = {0:[(-250.,0.,0.)],1:[(-250.,0.,0.)]}
    with csv_path.open() as source:
        for stopped,x,y,yaw in csv.reader(source):
            paths[int(stopped)].append((float(x),float(y),float(yaw)))
    plan = json.loads((ROOT/'Tests/task2_route_plan.json').read_text())
    matplotlib.rcParams['svg.fonttype'] = 'none'
    matplotlib.rcParams['svg.hashsalt'] = 'task2-stopped-body-clearance'
    fig,ax = plt.subplots(figsize=(12,5.8),layout='constrained')
    for stopped,color,width,label in ((1,'#ed9e29',3.5,'B: stopped primitives; yaw-priority arc exits'),
                                      (0,'#1b63a5',1.3,'A: secondary fused comparison')):
        ax.plot(*zip(*[(x,y) for x,y,h in paths[stopped]]),color=color,linewidth=width,label=label)
    for ox,hh,label in ((1100,50,'1: 100 × 100'),(2250,250,'2: 100 × 500')):
        ax.add_patch(Rectangle((ox-50,-hh),100,2*hh,facecolor='#495567',zorder=5))
        ax.text(ox,-hh-85,label,ha='center',fontsize=9,
                bbox={'facecolor':'white','edgecolor':'none','alpha':.9})
        # Exact 200 mm dilation of each obstacle: straight edges and quarter circles.
        m = BODY_CLEARANCE_MM
        style = {'color':'#b45b5b','linestyle':'--','linewidth':.9}
        ax.plot([ox-50,ox+50],[hh+m,hh+m],**style)
        ax.plot([ox-50,ox+50],[-hh-m,-hh-m],**style)
        ax.plot([ox-50-m,ox-50-m],[-hh,hh],**style)
        ax.plot([ox+50+m,ox+50+m],[-hh,hh],**style)
        for dx,dy,lo,hi in ((50,hh,0,90),(-50,hh,90,180),(-50,-hh,180,270),(50,-hh,270,360)):
            ax.add_patch(Arc((ox+dx,dy),2*m,2*m,theta1=lo,theta2=hi,**style))
    for side in (-1,1):
        ax.axhline(side*SIDE_BOUNDARY_MM,color='#80629d',linestyle=':',linewidth=1.2)
        ax.annotate('',xy=(2450,side*750),xytext=(2450,side*250),
                    arrowprops={'arrowstyle':'<->','color':'#80629d'})
        ax.text(2520,side*705,'500 mm worst-case space',ha='left',va='center',fontsize=8,
                color='#684582',bbox={'facecolor':'white','edgecolor':'none','alpha':.9})
    ax.text(2820,100,'No specified back limit',fontsize=8,color='#687487',ha='center',
            bbox={'facecolor':'white','edgecolor':'none','alpha':.9})
    ax.plot([0,-500,-500,0],[300,300,-300,-300],color='#495567',linewidth=3)
    ax.scatter([-250],[0],color='#19835c',s=35,zorder=8)
    ax.text(-250,-205,'Start / park\n(-250, 0)',ha='center',fontsize=9)
    ax.text(-250,370,'Park: 500 deep\n× 600 wide',ha='center',fontsize=9)
    for start,end,y in ((0,1050,110),(1150,2200,150)):
        ax.annotate('',xy=(end,y),xytext=(start,y),arrowprops={'arrowstyle':'<->','color':'#687487'})
        ax.text((start+end)/2,y+25,'1050 face-to-face',ha='center',fontsize=8)
    # Selected full body poses make the clearance reference explicit.
    for target in ((1100,400),(2250,-600),(2600,0)):
        x,y,h = min(paths[1],key=lambda p:(p[0]-target[0])**2+(p[1]-target[1])**2)
        ax.add_patch(Polygon(corners(x,y,h),facecolor='#ed9e29',edgecolor='#ab7323',alpha=.18,zorder=4))
    x,y,h = paths[1][-1]
    finish_x,finish_y,finish_heading = x,y,h
    ax.add_patch(Polygon(corners(x,y,h),facecolor='#19835c',edgecolor='#19835c',alpha=.2,
                         label='Assumed 300 × 200 mm body; B parked'))
    for x,y,text in ((1100,530,'First left →'),(2060,-665,'Second right →'),
                      (1500,610,'Broad return arc ↙')):
        ax.text(x,y,text,ha='center',fontsize=9)
    handles,labels = ax.get_legend_handles_labels()
    handles.append(Line2D([0],[0],color='#b45b5b',linestyle='--',linewidth=1))
    labels.append('200 mm obstacle buffer: robot body stays outside')
    ax.legend(handles,labels,loc='upper left',fontsize=8,framealpha=1)
    ax.set(xlim=(-600,3300),ylim=(-820,820),aspect='equal',
           xlabel='x [mm] — car-park mouth at 0; initial heading +x',
           ylabel='y [mm] — positive to the left',
           title=f"Task 2: stopped-route optimisation · {plan['distance_mm']/1000:.2f} m · 7 primitives")
    ax.grid(alpha=.12)
    ax.text(.5,-.18,
            'B is the planning target: 200 mm body clearance; no minimum primitive length.\n'
            'Obstacle 2: exactly 500 mm beyond each end; no assumed boundary behind it.\n'
            f"B nominal finish: ({finish_x:.0f}, {finish_y:.0f}) mm, heading {math.degrees(finish_heading):.1f}°. Assumed body; physical tracking requires a trial.",
            transform=ax.transAxes,ha='center',va='top',fontsize=9)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,metadata={'Date':None})
    if output.suffix.lower()=='.svg':
        output.write_text('\n'.join(line.rstrip() for line in output.read_text().splitlines())+'\n')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'Tests/task2_route.svg')
    args = parser.parse_args()
    if args.csv:
        plot(args.csv,args.output)
    else:
        with tempfile.TemporaryDirectory(prefix='task2-route-') as directory:
            csv_path = Path(directory)/'route.csv'
            subprocess.run([sys.executable,str(ROOT/'Tests/Host/test_fused_motion.py')],check=True,
                           env=dict(os.environ,TASK2_ROUTE_CSV=str(csv_path)))
            plot(csv_path,args.output)
    print(f'Saved {args.output}')


if __name__=='__main__':
    main()
