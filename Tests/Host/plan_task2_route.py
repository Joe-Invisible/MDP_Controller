#!/usr/bin/env python3
"""Numerically minimise nominal forward travel for a stopped Task 2 route.

Requires numpy/scipy. Optimises a five- or seven-primitive tangent-circle
family, not all possible bounded-curvature routes. Robot: assumed centred 300 x 200 mm.
Obstacle/body clearance: 200 mm. Only obstacle 2's end gaps are constrained
(500 mm); there is no assumed limit behind obstacle 2. Dense production-planner
verification remains in test_fused_motion.py.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[2]
START = np.array([-250., 0.])
OBSTACLES = ((1100., 50.), (2250., 250.))
BODY_HALF = np.array([150., 100.])
MIN_RADIUS = 275.
PLANNING_CLEARANCE_MM = 222.  # 200 required + 20 terminal-exit reserve + 2 sampling reserve


def normal(yaw):
    return np.array([-math.sin(yaw), math.cos(yaw)])


def tangent(a, ra, sa, b, rb, sb):
    delta = b - a
    length = np.linalg.norm(delta)
    q = sb*rb - sa*ra
    if length <= abs(q):
        raise ValueError('No forward tangent')
    yaw = math.atan2(delta[1], delta[0]) - math.asin(q / length)
    return yaw, math.sqrt(length*length - q*q)


def build(v, samples=129):
    if len(v) == 7:
        return build_single_back_arc(v,samples)
    x1, y1, r1, x2, y2, r2, x3, y3, r3, exit_yaw = v
    c1, c2, c3 = (np.array([x, y]) for x, y in ((x1,y1),(x2,y2),(x3,y3)))
    rd = ((x1+250)**2 + y1*y1 - r1*r1) / (2*(y1+r1))
    if rd <= 0:
        raise ValueError('Invalid departure')
    first_yaw = math.atan2(x1+250, rd-y1)
    cross_yaw, cross_length = tangent(c1,r1,-1,c2,r2,1)
    side_yaw, side_length = tangent(c2,r2,1,c3,r3,1)
    if side_yaw < 0:
        side_yaw += 2*math.pi
    p3 = c3-r3*normal(exit_yaw)
    delta = START-p3
    denominator = 2*np.dot(delta,normal(exit_yaw))
    if denominator <= 0:
        raise ValueError('Return does not curve into park')
    rr = np.dot(delta,delta)/denominator
    cr = p3+rr*normal(exit_yaw)
    end_yaw = math.atan2(START[0]-cr[0], -(START[1]-cr[1]))
    if end_yaw < exit_yaw:
        end_yaw += 2*math.pi
    sections = ((np.array([-250.,rd]),rd,0.,first_yaw),
                (c1,-r1,first_yaw,cross_yaw),
                (None,0.,cross_yaw,cross_length),
                (c2,r2,cross_yaw,side_yaw),
                (None,0.,side_yaw,side_length),
                (c3,r3,side_yaw,exit_yaw),
                (cr,rr,exit_yaw,end_yaw))
    points, yaws, steps = [], [], []
    pos = START.copy()
    for centre, radius, a, b in sections:
        if radius:
            angle = (b-a)*np.sign(radius)
            if angle <= 0 or angle > math.pi:
                raise ValueError('Invalid arc angle')
            yaw = np.linspace(a,b,samples)
            xy = centre[None,:] - radius*np.column_stack((-np.sin(yaw),np.cos(yaw)))
            length = abs(radius)*angle
        else:
            length = b
            yaw = np.full(samples,a)
            xy = pos+np.linspace(0,length,samples)[:,None]*np.array([math.cos(a),math.sin(a)])
        points.append(xy); yaws.append(yaw); steps.append((length,radius))
        pos = xy[-1]
    return np.concatenate(points), np.concatenate(yaws), steps, end_yaw



def build_single_back_arc(v, samples):
    x1,y1,r1,xb,yb,rb,exit_yaw = v
    c1,cb = np.array([x1,y1]),np.array([xb,yb])
    rd = ((x1+250)**2+y1*y1-r1*r1)/(2*(y1+r1))
    if rd <= 0:
        raise ValueError('Invalid departure')
    first_yaw = math.atan2(x1+250,rd-y1)
    cross_yaw,cross_length = tangent(c1,r1,-1,cb,rb,1)
    start_return = cb-rb*normal(exit_yaw)
    delta = START-start_return
    denominator = 2*np.dot(delta,normal(exit_yaw))
    if denominator <= 0:
        raise ValueError('Invalid return')
    rr = np.dot(delta,delta)/denominator
    cr = start_return+rr*normal(exit_yaw)
    end_yaw = math.atan2(START[0]-cr[0],-(START[1]-cr[1]))
    if end_yaw < exit_yaw:
        end_yaw += 2*math.pi
    sections = ((np.array([-250.,rd]),rd,0.,first_yaw),
                (c1,-r1,first_yaw,cross_yaw),
                (None,0.,cross_yaw,cross_length),
                (cb,rb,cross_yaw,exit_yaw),
                (cr,rr,exit_yaw,end_yaw))
    points,yaws,steps = [],[],[]
    pos = START.copy()
    for i,(centre,radius,a,b) in enumerate(sections):
        if radius:
            angle = (b-a)*np.sign(radius)
            if angle <= 0 or angle > (2*math.pi if i == 3 else math.pi):
                raise ValueError('Invalid arc angle')
            yaw = np.linspace(a,b,samples)
            xy = centre[None,:]-radius*np.column_stack((-np.sin(yaw),np.cos(yaw)))
            length = abs(radius)*angle
        else:
            length = b
            yaw = np.full(samples,a)
            xy = pos+np.linspace(0,length,samples)[:,None]*np.array([math.cos(a),math.sin(a)])
        points.append(xy);yaws.append(yaw);steps.append((length,radius));pos=xy[-1]
    return np.concatenate(points),np.concatenate(yaws),steps,end_yaw

def body_clearances(xy, yaw):
    t = np.column_stack((np.cos(yaw), np.sin(yaw)))
    n = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    corners = np.array([xy + f*150*t+s*100*n for f,s in ((1,1),(1,-1),(-1,-1),(-1,1))]).transpose(1,0,2)
    gaps = []
    for ox, hh in OBSTACLES:
        oc = np.array([[ox-50,-hh],[ox-50,hh],[ox+50,hh],[ox+50,-hh]])
        d = np.maximum(abs(corners-np.array([ox,0]))-np.array([50,hh]),0)
        vertex_to_obstacle = np.linalg.norm(d,axis=2).min(axis=1)
        rel = oc[None,:,:]-xy[:,None,:]
        local = np.stack((np.einsum('nki,ni->nk',rel,t),np.einsum('nki,ni->nk',rel,n)),axis=2)
        d = np.maximum(abs(local)-BODY_HALF,0)
        obstacle_to_body = np.linalg.norm(d,axis=2).min(axis=1)
        overlap = ((corners[:,:,0].max(axis=1)>=ox-50)&(corners[:,:,0].min(axis=1)<=ox+50)&
                   (corners[:,:,1].max(axis=1)>=-hh)&(corners[:,:,1].min(axis=1)<=hh)&
                   (local[:,:,0].max(axis=1)>=-150)&(local[:,:,0].min(axis=1)<=150)&
                   (local[:,:,1].max(axis=1)>=-100)&(local[:,:,1].min(axis=1)<=100))
        gaps.append(np.where(overlap,0,np.minimum(vertex_to_obstacle,obstacle_to_body)))
    # Park walls apply to body corners inside the park. Dense host test also
    # checks each body's intersection with the two mouth-wall tips.
    bounds = np.where(corners[:,:,0]<0,300.,750.)-abs(corners[:,:,1])
    rear = corners[:,:,0]+500
    return np.concatenate(gaps), np.concatenate((bounds.ravel(),rear.ravel()))



def objective(v):
    try:
        _,_,steps,_ = build(v)
        return sum(s[0] for s in steps)/1000
    except ValueError:
        return 1000.


def constraints(v):
    try:
        xy,yaw,steps,_ = build(v)
        obstacles,bounds = body_clearances(xy,yaw)
        result = np.r_[obstacles-PLANNING_CLEARANCE_MM, bounds-2., abs(steps[0][1])-MIN_RADIUS,
                       abs(steps[-1][1])-MIN_RADIUS]
        return result
    except ValueError:
        return np.full(10*(5 if len(v)==7 else 7)*129+2,-1e5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--write-route',action='store_true')
    parser.add_argument('--single-back-arc',action='store_true',
                        help='Replace the two back arcs and straight with one arc (5 primitives)')
    args = parser.parse_args()
    initial = [1100,50,355,2250,-250,355,2250,250,355,math.pi]
    bounds = [(900,1500),(-400,250),(275,1200),(2050,3000),(-450,100),(275,750),
              (2050,4000),(0,350),(275,750),(math.radians(160),math.radians(195))]
    if args.single_back_arc:
        initial = [1100,0,400,2250,0,620,math.pi+0.05]
        bounds = [(900,1500),(-400,250),(275,1200),(2050,3200),(-100,100),
                  (275,750),(math.radians(165),math.radians(195))]
    candidates = []
    for shift in (0.,30.,-30.):
        seed = initial.copy(); seed[3] += shift
        if not args.single_back_arc:
            seed[6] += shift
        result = minimize(objective,seed,method='SLSQP',bounds=bounds,
                          constraints=[{'type':'ineq','fun':constraints}],
                          options={'maxiter':180,'ftol':1e-9})
        xy,yaw,steps,end_yaw = build(result.x,2001)
        obstacles,edges = body_clearances(xy,yaw)
        if obstacles.min()>=PLANNING_CLEARANCE_MM-0.2 and edges.min()>=1.99:
            candidates.append((sum(d for d,r in steps),result,steps,end_yaw,obstacles.min(),edges.min()))
    if not candidates:
        raise RuntimeError('No densely verified candidate; leave the route unchanged')
    length,result,steps,end_yaw,obstacle_margin,boundary_margin = min(candidates,key=lambda r:r[0])
    data = {'family':('five forward primitives; single obstacle-2 back arc and broad return'
                      if args.single_back_arc else 'seven forward primitives; tangent circles and a broad return arc'),
            'global_optimum_proven':False,
            'required_body_clearance_mm':200.,
            'planned_body_clearance_mm':PLANNING_CLEARANCE_MM,'solver_success':bool(result.success),
            'solver_message':result.message,'parameters':result.x.tolist(),
            'distance_mm':length,'final_yaw_deg':math.degrees(end_yaw),
            'minimum_body_obstacle_distance_mm':float(obstacle_margin),
            'minimum_body_boundary_margin_mm':float(boundary_margin),
            'steps':[{'distance_mm':d,'radius_mm':r} for d,r in steps]}
    if args.output:
        args.output.write_text(json.dumps(data,indent=2)+'\n')
    if args.write_route:
        target = ROOT/'Tests/Src/MotionSequenceFusionTest.c'
        text = target.read_text()
        start = text.index('    /* Generated by Tests/Host/plan_task2_route.py.')
        end = text.index('    _Static_assert(sizeof(fusionTestSteps)',start)
        block = ('    /* Generated by Tests/Host/plan_task2_route.py. B is the planning target.\n'
                 '     * 200 mm body clearance; only the side gaps at obstacle 2 are 500 mm.\n'
                 '     * There is no minimum primitive length or back-boundary assumption. */\n'
                 '    const MotionSequenceFusionTestStep fusionTestSteps[] = {\n')
        for d,r in steps:
            block += f'        {{{d:.6f}f, {r:.6f}f, FUSION_SPEED_CPS}},\n'
        block += '    };\n'
        target.write_text(text[:start]+block+text[end:])
    print(json.dumps(data,indent=2))


if __name__=='__main__':
    main()
