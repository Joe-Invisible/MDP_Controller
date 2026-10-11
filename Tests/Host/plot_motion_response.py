#!/usr/bin/env python3
"""Plot the validation path evaluated by production C, not a second planner."""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='response-path-') as temp:
    path=Path(temp)/'path.csv'
    subprocess.run(['python3',str(ROOT/'Tests/Host/test_motion_response.py')],check=True,
                   env=dict(os.environ,MOTION_RESPONSE_PATH_CSV=str(path)))
    with path.open() as stream:
        rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
plt.rcParams.update({'svg.fonttype':'none','font.size':10})
fig,ax=plt.subplots(1,2,figsize=(11,5.6),gridspec_kw={'width_ratios':[1,1.15]})
phases={1:('Lead-in','#526a80'),2:('Measured entry','#e09127'),
        3:('Constant arc + extension','#217a9c'),4:('Measured exit','#cb4b62'),
        5:('Zero-command recovery','#6d56a0'),6:('Slowing portion','#54794b')}
for phase,(label,color) in phases.items():
    data=[r for r in rows if r['phase']==phase]
    ax[0].plot([r['xMm'] for r in data],[r['yMm'] for r in data],c=color,lw=3,label=label)
    ax[1].plot([r['progressMm'] for r in data],[r['yawDeg'] for r in data],c=color,lw=2.5)
    if data: ax[0].scatter(data[0]['xMm'],data[0]['yMm'],c=color,s=22,zorder=4)
end=rows[-1]
ax[0].scatter(0,0,c='black',s=25,zorder=5)
ax[0].annotate('Start; heading +x',(0,0),xytext=(15,-60))
ax[0].annotate(f"End ({end['xMm']:.0f}, {end['yMm']:.0f}) mm\nheading 90°",
    (end['xMm'],end['yMm']),xytext=(-150,12),textcoords='offset points',fontsize=10)
ax[0].set(xlabel='x (mm)',ylabel='y (mm)',aspect='equal',title='Predicted rear-axle path')
ax[0].set_xlim(-120,1200);ax[0].set_ylim(-120,1190)
ax[0].legend(loc='center left',fontsize=8,framealpha=.9)
ax[1].set(xlabel='Measured progress (mm)',ylabel='Predicted heading (degrees)',title='Heading includes both asymmetric transitions')
second=ax[1].twinx();second.plot([r['progressMm'] for r in rows],[r['rawCommand'] for r in rows],
    c='#555555',ls='--',lw=1.2,label='Raw steering command')
second.set_ylabel('Raw steering command');second.set_ylim(-55,8)
second.legend(loc='lower right',fontsize=8)
for a in ax:a.grid(alpha=.22)
fig.suptitle('Response-based validation · forward 4500 CPS · +R525 actuator schedule',fontsize=13)
fig.text(.5,.035,'Prediction from three 11.30 V batches. Constant arc extended by 384 mm: quasi-steady assumption under test.\n'
         'Centre-path span ≈ 1044 × 1016 mm; display includes 281 mm padding per side. No parking endpoint fit.',
         ha='center',fontsize=9)
fig.tight_layout(rect=(0,.10,1,.95))
fig.savefig(ROOT/'Tests/motion_response_path.svg')
(ROOT/'build/fused-limits').mkdir(parents=True,exist_ok=True)
fig.savefig(ROOT/'build/fused-limits/motion_response_path.png',dpi=150)
print('Saved Tests/motion_response_path.svg')
