#!/usr/bin/env python3
"""Search five forward arcs with simultaneous stopped/fused camera visibility.

All solver lengths are metres; output distances/radii are millimetres.
The 100 mm arrow width is a planning assumption, not a dimension in the rules.
This is a restricted numerical search, not proof of minimum primitive count.
"""
import argparse
import sys, json, math
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from plan_task2_route import body_clearances, normal
START = np.array([-0.25, 0.0])
GEOMETRY_SAMPLES = 101

def build(v, samples=GEOMETRY_SAMPLES):
    """Construct five tangent arcs; lengths and circle centres use metres."""
    r0, l0, x1, y1, r1, rb, phi, exit_yaw = v[:8]
    h0 = l0 / r0
    c0 = START + r0 * normal(0)
    p0 = c0 - r0 * normal(h0)
    c1 = np.array([x1, y1])
    delta = c1 - p0
    den = 2 * (np.dot(delta, normal(h0)) + r1)
    if den <= 0:
        raise ValueError()
    rd = (np.dot(delta, delta) - r1 * r1) / den
    if rd < 0.275:
        raise ValueError()
    cd = p0 + rd * normal(h0)
    first = math.atan2(c1[0] - cd[0], -(c1[1] - cd[1]))
    cb = c1 + (r1 + rb) * np.array([math.cos(phi), math.sin(phi)])
    cross = phi - math.pi / 2
    p3 = cb - rb * normal(exit_yaw)
    delta = START - p3
    den = 2 * np.dot(delta, normal(exit_yaw))
    if den <= 0:
        raise ValueError()
    rr = np.dot(delta, delta) / den
    cr = p3 + rr * normal(exit_yaw)
    end = math.atan2(START[0] - cr[0], -(START[1] - cr[1]))
    if end < exit_yaw:
        end += 2 * math.pi
    if rr < 0.275:
        raise ValueError()
    sections = [(c0, r0, 0, h0), (cd, rd, h0, first), (c1, -r1, first, cross), (cb, rb, cross, exit_yaw), (cr, rr, exit_yaw, end)]
    xy = []
    hs = []
    ss = []
    steps = []
    offset = 0
    for i, (c, r, a, b) in enumerate(sections):
        delta = (b - a) * np.sign(r)
        if delta <= 0 or delta > (2 * math.pi if i == 3 else math.pi):
            raise ValueError()
        yaw = np.linspace(a, b, samples)
        xy.append(c[None, :] - r * np.c_[-np.sin(yaw), np.cos(yaw)])
        hs.append(yaw)
        length = abs(r) * delta
        ss.append(offset + np.linspace(0, length, samples))
        offset += length
        steps.append((length, r))
    return (np.concatenate(xy), np.concatenate(hs), np.concatenate(ss), steps, sections)

def pose(v, q):
    """Evaluate a nominal rear-axle pose at a fraction of total route travel."""
    _, _, _, steps, sections = build(v, 2)
    dist = q * sum((d for d, r in steps))
    for (d, r), (c, r, a, b) in zip(steps, sections):
        if dist <= d:
            h = a + (b - a) * dist / d
            return (c - r * normal(h), h)
        dist -= d
    c, r, a, h = sections[-1]
    return (c - r * normal(h), h)

def length_objective(v):
    try:
        return sum((d for d, r in build(v, 2)[3]))
    except ValueError:
        return 1000.0
FUSED_SAMPLES = 801

def fused(v, n=FUSED_SAMPLES):
    """Integrate the additive 500 mm curvature ramps used by the C planner."""
    steps = build(v, 2)[3]
    lengths = np.array([d for d, r in steps])
    k = np.array([1 / r for d, r in steps])
    total = lengths.sum()
    ds = total / n
    s = np.arange(n + 1) * ds

    def heading(t):
        result = k[0] * t
        for i, bound in enumerate(np.cumsum(lengths)[:-1]):
            half = min(0.25, bound, total - bound)
            offset = np.clip(t - (bound - half), 0, 2 * half)
            area = offset * offset / (4 * half) + np.maximum(t - (bound + half), 0)
            result += (k[i + 1] - k[i]) * area
        return result
    hmid = heading((s[:-1] + s[1:]) * 0.5)
    xy = np.vstack((START, START + ds * np.cumsum(np.c_[np.cos(hmid), np.sin(hmid)], axis=0)))
    return (xy, heading(s), s)

def cam(p, h, tx):
    """Constrain horizontal framing of a centred target, range and face incidence."""
    c = p + 0.07 * np.array([math.cos(h), math.sin(h)])
    d = np.array([tx, 0]) - c
    out = [0.49 - np.linalg.norm(d), tx - c[0] - 0.03, (tx - c[0]) * math.tan(math.radians(50)) - abs(c[1])]
    for ty in [-TARGET_HALF_WIDTH_M, TARGET_HALF_WIDTH_M]:
        d = np.array([tx, ty]) - c
        front = np.dot(d, [math.cos(h), math.sin(h)])
        lat = np.dot(d, [-math.sin(h), math.cos(h)])
        out.extend([math.tan(math.radians(29)) * front - lat, math.tan(math.radians(29)) * front + lat])
    return out
TARGET_HALF_WIDTH_M = 0.04
VIEW_WINDOW_M = 0.05

def cons(v, cl):
    """Check both strategies, camera windows, body clearance, boundaries and parking."""
    try:
        xy, h, s, steps, _ = build(v)
        g, e = body_clearances(xy * 1000, h)
        fa, ha, sa = fused(v)
        ga, ea = body_clearances(fa * 1000, ha)
        camera = []
        total = sum((d for d, r in steps))
        for i, tx in enumerate([1.05, 2.2]):
            for dq in [-VIEW_WINDOW_M / 2, 0, VIEW_WINDOW_M / 2]:
                p, y = pose(v, v[8 + i] + dq / total)
                camera.extend(cam(p, y, tx))
                target = v[10 + i] * total + dq
                p = np.array([np.interp(target, sa, fa[:, a]) for a in [0, 1]])
                y = np.interp(target, sa, ha)
                camera.extend(cam(p, y, tx))
        parked = -fa[-1, 0] - 0.15 * abs(math.cos(ha[-1])) - 0.1 * abs(math.sin(ha[-1])) - 0.01
        return np.r_[g / 1000 - cl - 0.022, e / 1000 - 0.035, ga / 1000 - cl - 0.02, ea / 1000 - 0.025, camera, parked, v[9] - v[8] - 0.08, v[11] - v[10] - 0.08]
    except ValueError:
        return np.full(5 * GEOMETRY_SAMPLES * 10 + (FUSED_SAMPLES + 1) * 10 + 84 + 3, -1000.0)

def objective(v):
    return length_objective(v)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clearance', type=float, default=100.0, help='Required body clearance, mm')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--attempts', type=int, default=6)
    parser.add_argument('--verify-plan', type=Path, help='Verify a saved plan without rerunning the search')
    args = parser.parse_args()
    if args.verify_plan:
        data = json.loads(args.verify_plan.read_text())
        v = np.array(data['parameters'])
        cl = data['required_body_clearance_mm'] / 1000
        margin = cons(v, cl).min()
        xy, h, s, steps, _ = build(v, 4001)
        fa, ha, sa = fused(v, 20001)
        for label, p, y in [('B nominal', xy, h), ('A nominal', fa, ha)]:
            g, e = body_clearances(p * 1000, y)
            assert g.min() >= cl * 1000 and e.min() > 0, (label, g.min(), e.min())
        assert margin >= -0.0002, margin
        print('PASS: saved camera-route nominal geometry and horizontal viewing constraints')
        return
    bounds = [(3, 18), (0.45, 1.1), (0.8, 1.7), (-0.9, 0.35), (0.275, 1.3), (0.35, 0.72), (-0.45, 0.6), (math.radians(170), math.radians(195)), (0.04, 0.2), (0.18, 0.48), (0.04, 0.2), (0.18, 0.48)]
    initial = np.array([8.4552751295, 0.666820624, 1.3653655684, -0.0128777041, 0.5022249619, 0.5315114118, -0.0514406995, 3.0697359431, 0.100659083, 0.282105626, 0.0988791061, 0.2814318785])
    rng = np.random.default_rng(944)
    found = []
    for i in range(args.attempts):
        v = initial.copy()
        if i:
            v += rng.uniform(-1, 1, 12) * np.array([2, 0.06, 0.1, 0.06, 0.08, 0.03, 0.06, 0.04, 0.01, 0.01, 0.01, 0.01])
        v = np.clip(v, [a for a, c in bounds], [c for a, c in bounds])
        res = minimize(objective, v, method='SLSQP', bounds=bounds, constraints=[{'type': 'ineq', 'fun': lambda q: cons(q, args.clearance / 1000)}], options={'maxiter': 230, 'ftol': 1e-09})
        if cons(res.x, args.clearance / 1000).min() >= -1e-05:
            xy, h, s, steps, _ = build(res.x, 4001)
            fa, ha, sa = fused(res.x, 20001)
            if min(body_clearances(xy * 1000, h)[0].min(), body_clearances(fa * 1000, ha)[0].min()) >= args.clearance:
                found.append((length_objective(res.x), res))
        print(f'Seed {i}: length {length_objective(res.x):.4f} m; constraint margin {cons(res.x, args.clearance / 1000).min():.6f} m', flush=True)
    if not found:
        raise RuntimeError('No verified candidate in this family; no output changed')
    length, res = min(found, key=lambda a: a[0])
    steps = build(res.x, 2)[3]
    data = {'family': 'five forward arcs: left, left, right, left, left', 'global_minimum_primitive_count_proven': False, 'required_body_clearance_mm': args.clearance, 'body_mm': [300, 200], 'camera_ahead_of_axle_mm': 70, 'camera_horizontal_fov_deg': 60, 'recognition_center_range_mm': 500, 'optimization_target_width_mm': 80, 'optimization_view_window_mm': 50, 'fused_blend_request_mm': 500, 'parameters': res.x.tolist(), 'distance_mm': length * 1000, 'solver_success': bool(res.success), 'steps': [{'distance_mm': d * 1000, 'radius_mm': r * 1000} for d, r in steps]}
    if args.output:
        args.output.write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps(data, indent=2))
if __name__ == '__main__':
    main()
