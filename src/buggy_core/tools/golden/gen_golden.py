#!/usr/bin/env python3
"""Writes the golden data the C++ tests compare against, computed by the PYTHON reference code:
  test/actuation/golden_uart.txt        angle yellow red -> 6 frame bytes  (buggy_uart.UartInterface.create_message)
  test/perception/golden_cluster.txt     points + corridor + the Python detector's clusters
  test/control/golden_tracking.txt       path + poses -> match / pure pursuit / cross track / block_dist / stop distance
                                         (OperationManager methods, called on a stub object, no ROS graph)
  test/planning/golden_routes.txt        all maps: lanes, successors (incl. ring links), random start/goal plans + smoothed path, laps
  test/control/golden_handoff.jsonl      detour results applied by OperationManager.on_avoid_result (path splice, status, message)
  test/localization/golden_locstate.jsonl  event sequences (TF, NDT pose, score, state sets) -> /localization/state text (LocalizationManager stub)
  test/actuation/golden_feedback.jsonl     byte chunks (valid frames, garbage, splits) -> /wheel_uart messages (real UartBridge.timer_callback)
  test/localization/golden_preprocess.jsonl  synthetic scans -> lidar_preprocess output cloud (real LidarPreprocess.on_cloud)
  test/perception/golden_groundaware.jsonl  ground-aware detector scenes (slopes, leaks, boxes, thin rings) -> clusters of the real Python on_cloud
  test/planning/golden_avoid.jsonl       detour scenes (route, obstacles, lane, LiDAR grids) -> plan_detour result
  test/perception/golden_ground.txt      synthetic VLP-16 scan -> ground mask (segment_linefit_ground + classify) and self-hit keep mask
Run:  python3 src/buggy_core/tools/golden/gen_golden.py   (needs the workspace sourced)
"""
import math, os
import numpy as np
from scipy import ndimage
from buggy_uart.uart_interface import UartInterface

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'test')
HERE_UART = os.path.join(ROOT, 'actuation')
HERE_DET = os.path.join(ROOT, 'perception')

# ---- UART frames ------------------------------------------------------------------------
angles = [-30.0, -25.0, -20.0, -19.999, -10.5, -0.01, -0.004, 0.0, 0.004, 0.01, 3.3333, 5.5,
          12.345, 19.999, 20.0, 25.0, 1e9, -1e9]
with open(os.path.join(HERE_UART, 'golden_uart.txt'), 'w') as f:
    f.write('# angle yellow red b0 b1 b2 b3 b4 b5   (from UartInterface.create_message)\n')
    for a in angles:
        for y in (0, 1):
            for r in (0, 1):
                msg = UartInterface.create_message(None, bool(y), bool(r), a)
                f.write(f'{a!r} {y} {r} ' + ' '.join(str(b) for b in msg) + '\n')

# ---- clustering (the exact Python algorithm of LidarObstacleDetector) ------------------
def cluster(bx, by, bz, cell, min_pts):
    ix = np.floor((bx - bx.min()) / cell).astype(int)
    iy = np.floor((by - by.min()) / cell).astype(int)
    grid = np.zeros((ix.max() + 1, iy.max() + 1), dtype=bool)
    grid[ix, iy] = True
    labels, n = ndimage.label(grid, structure=np.ones((3, 3)))
    lab = labels[ix, iy]
    out = []
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() < min_pts:
            continue
        x0, x1, y0, y1 = bx[m].min(), bx[m].max(), by[m].min(), by[m].max()
        out.append((float((x0 + x1) / 2), float((y0 + y1) / 2), float(max(x1 - x0, 0.4)),
                    float(max(y1 - y0, 0.4)), float(bz[m].max()), int(m.sum())))
    return out

rng = np.random.default_rng(7)
blobs = [(8.0, 0.2, 0.5, 0.4, 1.2), (12.5, -0.8, 1.2, 0.6, 1.6), (18.0, 0.9, 0.4, 0.4, 0.8),
         (18.6, 1.0, 0.4, 0.3, 1.0), (22.0, -1.0, 2.0, 0.8, 1.4)]
pts = []
for cx, cy, sx, sy, h in blobs:
    n = int(rng.integers(30, 80))
    pts += list(zip(rng.normal(cx, sx / 2, n), rng.normal(cy, sy / 2, n), rng.uniform(0.25, h, n)))
pts += list(zip(rng.uniform(1, 24, 12), rng.uniform(-1.2, 1.2, 12), rng.uniform(0.25, 1.5, 12)))   # clutter
pts = np.array(pts)
# route: gentle curve, in the VEHICLE frame (the corridor), points every 0.5 m
xs = np.arange(0, 25, 0.5); corridor = np.column_stack([xs, 0.003 * xs ** 2])
tree_dist = np.min(np.hypot(pts[:, None, 0] - corridor[None, :, 0], pts[:, None, 1] - corridor[None, :, 1]), axis=1)
inside = tree_dist < 1.25
res = cluster(pts[inside, 0], pts[inside, 1], pts[inside, 2], 0.3, 5)
with open(os.path.join(HERE_DET, 'golden_cluster.txt'), 'w') as f:
    f.write('# POINTS x y z (base_link, height above ground)\n')
    for p in pts: f.write('P %r %r %r\n' % tuple(map(float, p)))
    f.write('# CORRIDOR x y\n')
    for c in corridor: f.write('C %r %r\n' % tuple(map(float, c)))
    f.write('# CLUSTERS cx cy lx ly h n  (Python: corridor 1.25 m, cell 0.3 m, min 5 pts)\n')
    for r in res: f.write('K %r %r %r %r %r %d\n' % r)

# ---- operation_manager: tracking maths, called on the real Python class without a node ----
from buggy_pcd_lanelet_map.operation_manager import OperationManager
om = OperationManager.__new__(OperationManager)
om.react, om.decel, om.lookahead, om.wheelbase, om.max_steer = 0.799, 0.694, 4.0, 1.6, 20.0
om.safe_half_width, om.avoid_hw = 1.25, 1.05
om.front_offset, om.detect_range = 2.2, 30.0
rng = np.random.default_rng(11)
t = np.arange(0, 140, 0.5)
om.path = [(float(30 * math.sin(a / 30.0)), float(a * 0.8 + 6 * math.sin(a / 11.0)), 0.5, 0.0, 1.0) for a in t]
om.seg = [0.0]
for a, b in zip(om.path, om.path[1:]):
    om.seg.append(om.seg[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
os.makedirs(os.path.join(ROOT, 'control'), exist_ok=True)
nq = nobj = 0
with open(os.path.join(ROOT, 'control', 'golden_tracking.txt'), 'w') as f:
    f.write('# R react decel lookahead wheelbase max_steer safe_hw avoid_hw\n')
    f.write('R %r %r %r %r %r %r %r\n' % (om.react, om.decel, om.lookahead, om.wheelbase, om.max_steer,
                                         om.safe_half_width, om.avoid_hw))
    for p in om.path: f.write('P %r %r %r\n' % p[:3])
    f.write('# Q rx ry ryaw idx0 v -> idx_after_match pure_pursuit cross_track stopping_distance\n')
    f.write('# O ox oy oyaw length width avoiding -> block_dist | none\n')
    for _ in range(60):
        i0 = int(rng.integers(0, len(om.path) - 60))
        base = om.path[i0 + int(rng.integers(0, 40))]
        om.rx, om.ry = float(base[0] + rng.normal(0, 0.6)), float(base[1] + rng.normal(0, 0.6))
        om.ryaw = float(rng.uniform(-math.pi, math.pi)); om.idx = i0
        v = float(rng.uniform(0, 2.0))
        om.match()
        f.write('Q %r %r %r %d %r %d %r %r %r\n' % (om.rx, om.ry, om.ryaw, i0, v, om.idx, om.pure_pursuit(),
                                                    om.cross_track(), om.stopping_distance(v)))
        nq += 1
        for _ in range(8):
            q = om.path[min(om.idx + int(rng.integers(0, 70)), len(om.path) - 1)]
            ox, oy = float(q[0] + rng.normal(0, 1.2)), float(q[1] + rng.normal(0, 1.2))
            oyaw, L, W = float(rng.uniform(-3, 3)), float(rng.uniform(0.3, 4)), float(rng.uniform(0.3, 2))
            for av in (False, True):
                om.avoiding = av
                d = om.block_dist((ox, oy, oyaw, L, W, 0))
                f.write('O %r %r %r %r %r %d %s\n' % (ox, oy, oyaw, L, W, int(av), 'none' if d is None else repr(d)))
                nobj += 1
        om.avoiding = False

# ---- ground segmentation ('segment' backend) and self-hit filter --------------------------
os.environ['SEGMENT_GROUND_METHOD'] = 'segment'
import types
import buggy_perception.segment_ground as sg
from buggy_perception.self_hit_filter import SelfHitFilter
from sensor_msgs.msg import PointCloud2, PointField
grng = np.random.default_rng(5)
scan = []
for az in np.arange(0, 360, 1.0):
    for el in np.radians(np.arange(-15, 16, 2.0)):
        c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
        # sensor 1 m above a gently sloped ground; beams below the horizon hit the ground
        slope = 0.02 * c
        if el < -0.02:
            r = (1.0 + slope * 0) / (-math.tan(el))
            if r > 30: continue
        else:
            r = grng.uniform(8, 30)
        x, y = r * c, r * s_
        z = -1.0 + slope * x * 0.5 + grng.normal(0, 0.015) if el < -0.02 else grng.uniform(-0.5, 1.5)
        scan.append((x, y, z))
for cx, cy, w, h in ((6.0, 1.0, 0.5, 1.2), (10.0, -2.0, 1.5, 0.8), (-5.0, 0.0, 0.6, 1.0)):      # objects on the ground
    for _ in range(60):
        scan.append((cx + grng.uniform(-w / 2, w / 2), cy + grng.uniform(-w / 2, w / 2), grng.uniform(-1.0, -1.0 + h)))
for _ in range(120):                                                                          # chassis / near points (self-hit)
    scan.append((grng.uniform(-3.0, 1.0), grng.uniform(-1.0, 1.0), grng.uniform(-1.1, 0.0)))
scan += [(0.2, 0.0, 0.0), (float('nan'), 0, 0)]
arr = np.array(scan, dtype=np.float32)
arr = arr[np.isfinite(arr).all(axis=1)]
f = arr[(np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2) < sg.MAX_DISTANCE) & (arr[:, 2] > sg.MIN_HEIGHT) & (arr[:, 2] < sg.MAX_HEIGHT)]
ground_cell, expected_z = sg.segment_linefit_ground(f)
signed = f[:, 2] - expected_z
ra = np.sqrt(f[:, 0] ** 2 + f[:, 1] ** 2)
nb = 0.08 * np.maximum(1.0 - ra / 3.0, 0.0)
adapt = np.minimum(sg.BASE_GROUND_THRESH + sg.DISTANCE_SCALE * ra + nb, sg.MAX_ADAPTIVE_THRESH)
gm_f = ground_cell & (np.abs(signed) < adapt) & (signed < sg.GROUND_MAX_Z_ABOVE) & (signed > -sg.GROUND_MAX_Z_BELOW)
keepmask = (np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2) < sg.MAX_DISTANCE) & (arr[:, 2] > sg.MIN_HEIGHT) & (arr[:, 2] < sg.MAX_HEIGHT)
gm = np.zeros(len(arr), dtype=bool); gm[keepmask] = gm_f
# the same through the real node method (fake publishers) - must agree with the replica above
import rclpy
rclpy.init()
node = sg.PointCloudProcessor()
class Cap:
    def __init__(s): s.m = None
    def publish(s, m): s.m = m
node.gpub, node.ngpub = Cap(), Cap()
m = PointCloud2(); m.height = 1; m.width = len(arr); m.point_step = 12; m.row_step = 12 * len(arr)
m.fields = [PointField(name=n_, offset=o, datatype=PointField.FLOAT32, count=1) for n_, o in (('x', 0), ('y', 4), ('z', 8))]
m.data = arr.tobytes()
node._segment_linefit(m, arr, len(arr), 0.0)
assert node.gpub.m.width == int(gm.sum()), (node.gpub.m.width, int(gm.sum()))
assert np.array_equal(np.frombuffer(node.gpub.m.data, dtype=np.float32).reshape(-1, 4)[:, :3], arr[gm])
# self-hit filter on the NON-ground points, through the node callback
shf = SelfHitFilter(); shf.pub = Cap()
ng = arr[~gm]
buf = np.zeros((len(ng), 4), dtype=np.float32); buf[:, :3] = ng
m2 = PointCloud2(); m2.height = 1; m2.width = len(ng); m2.point_step = 16; m2.row_step = 16 * len(ng)
m2.fields = [PointField(name=n_, offset=o, datatype=PointField.FLOAT32, count=1) for n_, o in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
m2.data = buf.tobytes()
shf.callback(m2)
kept = np.frombuffer(shf.pub.m.data, dtype=np.float32).reshape(-1, 4)
keep_flags, j = [], 0
for row in buf:
    if j < len(kept) and np.array_equal(kept[j], row): keep_flags.append(1); j += 1
    else: keep_flags.append(0)
assert j == len(kept)
with open(os.path.join(HERE_DET, 'golden_ground.txt'), 'w') as fo:
    fo.write('# G x y z ground(1/0)   - segment ground backend (python)\n')
    for p_, g_ in zip(arr, gm): fo.write('G %r %r %r %d\n' % (float(p_[0]), float(p_[1]), float(p_[2]), int(g_)))
    fo.write('# H x y z keep(1/0)   - self_hit_filter on the non-ground points (python)\n')
    for p_, k_ in zip(ng, keep_flags): fo.write('H %r %r %r %d\n' % (float(p_[0]), float(p_[1]), float(p_[2]), k_))
print('wrote golden_ground.txt (%d points: %d ground; self-hit keeps %d of %d)' % (len(arr), gm.sum(), sum(keep_flags), len(ng)))

# ---- wheel odometry: the real Python node fed through its callback, with a fake clock ----
from buggy_perception.wheel_odometry import WheelOdometry
from rclpy.time import Time
from ground_segmentation_msgs.msg import SteerSpeed
wo = WheelOdometry()
class FakeClock:
    def __init__(s): s.t = 0
    def now(s): return Time(nanoseconds=s.t)
fc = FakeClock(); wo.get_clock = lambda: fc
capt = []
wo.odom_pub = type('C', (), {'publish': lambda s, m: capt.append(m)})()
wo.tf_broadcaster = type('T', (), {'sendTransform': lambda s, m: None})()
orng = np.random.default_rng(3)
os.makedirs(os.path.join(ROOT, 'localization'), exist_ok=True)
rows = []
dts = []
for i in range(1200):
    dt = float(orng.choice([0.02, 0.05, 0.1, 0.0005, 0.2]))
    fc.t += int(round(dt * 1e9))
    steer = float(np.clip(orng.normal(0, 8), -20, 20)) if i % 50 < 40 else 0.0
    speed = float(max(0.0, 4.0 + orng.normal(0, 1.5))) if i % 97 > 8 else 0.0
    steer, speed = float(np.float32(steer)), float(np.float32(speed))   # what a received float32 message holds
    msg = SteerSpeed(); msg.steering, msg.speed = steer, speed
    before = len(capt); wo.feedback_callback(msg)
    if len(capt) > before:
        o = capt[-1]
        rows.append((dt_used := (fc.t - last_t) / 1e9 if i else 0.0, steer, speed, o))
    last_t = fc.t
with open(os.path.join(ROOT, 'localization', 'golden_odom.txt'), 'w') as fo:
    fo.write('# dt steering_deg speed_kmh -> x y qz qw v angular_z   (python WheelOdometry.feedback_callback)\n')
    for dt_, st, sp, o in rows:
        fo.write('%r %r %r %r %r %r %r %r %r\n' % (dt_, float(np.float32(st)), float(np.float32(sp)), o.pose.pose.position.x, o.pose.pose.position.y,
                 o.pose.pose.orientation.z, o.pose.pose.orientation.w, o.twist.twist.linear.x, o.twist.twist.angular.z))
print('wrote golden_odom.txt (%d odometry updates)' % len(rows))

# ---- route planner: the real RoutePlanner methods on every map in data/maps ---------------
from buggy_pcd_lanelet_map.route_planner import RoutePlanner, load_lanelets, project
data = os.environ.get('BUGGY_DATA_DIR', os.path.join(ROOT, '..', '..', '..', 'data'))
os.makedirs(os.path.join(ROOT, 'planning'), exist_ok=True)
prng = np.random.default_rng(21)
nplan = nok = nlap = 0
with open(os.path.join(ROOT, 'planning', 'golden_routes.txt'), 'w') as fo:
    fo.write('# MAP name | L id npts length cx0 cy0 cx1 cy1 | S id succ.. | P sx sy gx gy -> none | si gi chain.. | T n before after + points | A hx hy -> none | chain.. + n points\n')
    for name in sorted(os.listdir(os.path.join(data, 'maps'))):
        osm = os.path.join(data, 'maps', name, 'lanelet2_map.osm')
        if not os.path.exists(osm): continue
        rp = RoutePlanner.__new__(RoutePlanner)
        rp.get_logger = lambda: type('L', (), {'info': lambda s, m: None})()
        rp.lanelets = {l.id: l for l in load_lanelets(osm)}
        by_first = {}
        for l in rp.lanelets.values(): by_first.setdefault(l.first, []).append(l.id)
        rp.succ = {l.id: list(by_first.get(l.last, [])) for l in rp.lanelets.values()}
        rp.link_gap, rp.link_angle, rp.home_reach = 4.0, math.radians(60.0), 2.5
        rp.wheelbase, rp.steer_limit, rp.max_dev = 1.6, 20.0 * 0.9, 0.6
        rp.link_geometrically()
        fo.write('MAP %s\n' % name)
        allc = np.array([c[:2] for l in rp.lanelets.values() for c in l.center])
        for l in rp.lanelets.values():
            fo.write('L %s %d %r %r %r %r %r\n' % (l.id, len(l.center), l.length, l.center[0][0], l.center[0][1], l.center[-1][0], l.center[-1][1]))
        for lid, sc in rp.succ.items():
            fo.write('S %s %s\n' % (lid, ' '.join(sc)))
        lo, hi = allc.min(axis=0) - 8, allc.max(axis=0) + 8
        for _ in range(14):
            st, go = prng.uniform(lo, hi), prng.uniform(lo, hi)
            if prng.random() < 0.5:     # half of them close to the lanes
                st = allc[prng.integers(len(allc))] + prng.normal(0, 1.0, 2); go = allc[prng.integers(len(allc))] + prng.normal(0, 1.0, 2)
            res = rp.plan((float(st[0]), float(st[1])), (float(go[0]), float(go[1])))
            hdr = '%r %r %r %r' % (float(st[0]), float(st[1]), float(go[0]), float(go[1]))
            if res is None:
                fo.write('P %s none\n' % hdr); continue
            chain, si, gi = res
            fo.write('P %s %d %d %s\n' % (hdr, si, gi, ' '.join(chain)))
            raw = rp.chain_points(chain, si, gi)
            sm, b, a = rp.smooth_pts(raw)
            fo.write('T %d %r %r\n' % (len(sm), b, a))
            for p_ in sm: fo.write('%r %r %r\n' % p_)
            nok += 1
        for _ in range(3):
            h = allc[prng.integers(len(allc))] + prng.normal(0, 0.5, 2)
            lap = rp.lap_points((float(h[0]), float(h[1])))
            if lap is None:
                fo.write('A %r %r none\n' % (float(h[0]), float(h[1]))); continue
            chain, pts_ = lap
            fo.write('A %r %r %s\n' % (float(h[0]), float(h[1]), ' '.join(chain)))
            fo.write('%d %r\n' % (len(pts_), sum(math.dist(a_, b_) for a_, b_ in zip(pts_, pts_[1:]))))
            nlap += 1
        nplan += 14
print('wrote golden_routes.txt (%d plans, %d routed, %d laps)' % (nplan, nok, nlap))

# ---- avoidance: the real plan_detour on synthetic blocked-lane scenes ----------------------
import json
from buggy_pcd_lanelet_map.avoidance import FreeSpace, Params as APrm, plan_detour, free_space_cells
from buggy_pcd_lanelet_map.avoidance_planner import LaneModel
arng = np.random.default_rng(99)
nsc = nok_sc = 0
lanes = {}
with open(os.path.join(ROOT, 'planning', 'golden_avoid.jsonl'), 'w') as fo:
    for name in ('new_delhi_map', 'office_loop1'):
        osm = os.path.join(data, 'maps', name, 'lanelet2_map.osm')
        rp = RoutePlanner.__new__(RoutePlanner)
        rp.get_logger = lambda: type('L', (), {'info': lambda s, m: None})()
        rp.lanelets = {l.id: l for l in load_lanelets(osm)}
        bf = {}
        for l in rp.lanelets.values(): bf.setdefault(l.first, []).append(l.id)
        rp.succ = {l.id: list(bf.get(l.last, [])) for l in rp.lanelets.values()}
        rp.link_gap, rp.link_angle, rp.home_reach = 4.0, math.radians(60.0), 2.5
        rp.wheelbase, rp.steer_limit, rp.max_dev = 1.6, 18.0, 0.6
        rp.link_geometrically()
        lane = LaneModel(load_lanelets(osm))
        allc = np.array([c[:2] for l in rp.lanelets.values() for c in l.center])
        made = 0
        while made < 28:
            a = allc[arng.integers(len(allc))]; b = allc[arng.integers(len(allc))]
            res = rp.plan((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
            if res is None: continue
            raw = rp.chain_points(*res); sm, _, _ = rp.smooth_pts(raw)
            if len(sm) < 60: continue
            pts = np.array(sm[:130])
            obst = []
            for _ in range(int(arng.integers(1, 3))):
                k = int(arng.integers(12, min(len(pts) - 8, 80)))
                q = pts[k]; yaw = float(arng.uniform(-0.8, 0.8)) + math.atan2(pts[k + 1][1] - pts[k][1], pts[k + 1][0] - pts[k][0])
                kind = arng.random()
                L_, W_ = (float(arng.uniform(0.4, 1.2)), float(arng.uniform(0.4, 1.0))) if kind < 0.55 else \
                         (float(arng.uniform(1.5, 4.5)), float(arng.uniform(1.2, 2.2))) if kind < 0.85 else (0.8, float(arng.uniform(5.0, 9.0)))
                lat = float(arng.uniform(-0.9, 0.9))
                nrm = np.array([-(pts[k + 1][1] - pts[k][1]), pts[k + 1][0] - pts[k][0]]); nrm /= np.hypot(*nrm)
                obst.append((float(q[0] + lat * nrm[0]), float(q[1] + lat * nrm[1]), yaw, L_, W_))
            use_free = arng.random() < 0.6
            free_spec = None; free = None
            if use_free:
                res_ = 0.2; lo = pts[:, :2].min(axis=0) - 10.0; hi = pts[:, :2].max(axis=0) + 10.0
                shape = np.ceil((hi - lo) / res_).astype(int) + 1
                occ = np.zeros(shape, bool); gnd = np.zeros(shape, bool)
                occ_rects, gnd_rects, holes = [], [], []
                for (x, y, yaw, L_, W_) in obst:                    # solid cells where the obstacles are (+ sometimes a wall)
                    i0, j0 = np.floor((np.array([x, y]) - lo) / res_).astype(int)
                    r_ = int(max(L_, W_) / res_ / 2) + 1
                    occ_rects.append([int(i0 - r_), int(i0 + r_), int(j0 - r_), int(j0 + r_)])
                gnd_rects.append([0, int(shape[0]) - 1, 0, int(shape[1]) - 1])
                for _ in range(int(arng.integers(0, 4))):
                    i0 = int(arng.integers(0, shape[0] - 40)); j0 = int(arng.integers(0, shape[1] - 40))
                    holes.append([i0, i0 + int(arng.integers(10, 40)), j0, j0 + int(arng.integers(10, 40))])
                for i0, i1, j0, j1 in occ_rects: occ[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1] = True
                for i0, i1, j0, j1 in gnd_rects: gnd[i0:i1 + 1, j0:j1 + 1] = True
                for i0, i1, j0, j1 in holes: gnd[i0:i1 + 1, j0:j1 + 1] = False
                free = FreeSpace(lo, res_, occ, gnd)
                free_spec = {'origin': [float(lo[0]), float(lo[1])], 'res': res_, 'nx': int(shape[0]), 'ny': int(shape[1]),
                             'occ_rects': occ_rects, 'gnd_rects': gnd_rects, 'holes': holes}
            use_lane = arng.random() < 0.7
            r = plan_detour(pts, obst, lane if use_lane else None, free, APrm())
            zone_ = r.pop('zone', None)
            cells_ = free_space_cells(pts, zone_, free, lane if use_lane else None, APrm()) if (zone_ is not None and free is not None) else None
            fo.write(json.dumps({'map': name, 'points': pts.tolist(), 'obstacles': obst, 'lane': bool(use_lane),
                                 'free': free_spec, 'result': r,
                                 'zone': None if zone_ is None else {k_: float(v_) for k_, v_ in zone_.items()},
                                 'cells_n': None if cells_ is None else int(len(cells_)),
                                 'cells_sum': None if cells_ is None else [float(cells_[:, 0].sum()), float(cells_[:, 1].sum())] if len(cells_) else [0.0, 0.0]}) + '\n')
            nsc += 1; nok_sc += int(r['ok']); made += 1

# ---- operation_manager <- avoidance_planner handoff: the real Python on_avoid_result on a stub ----
from rclpy.clock import Clock
om2 = OperationManager.__new__(OperationManager)
om2.mono = lambda: 7.0
om2.get_logger = lambda: type('L', (), {'info': lambda s, m: None, 'warn': lambda s, m: None})()
om2.get_clock = lambda: Clock()
om2.detour_pub = type('D', (), {'publish': lambda s, m: None})()
om2.avoid_retry = 5.0
om2.avoid_pending_blockers, om2.probe_pending, om2.restored_until = [], None, 0.0
om2.live = True
nh = 0
os.makedirs(os.path.join(ROOT, 'control'), exist_ok=True)
with open(os.path.join(ROOT, 'planning', 'golden_avoid.jsonl')) as fi, open(os.path.join(ROOT, 'control', 'golden_handoff.jsonl'), 'w') as fh:
    for ln in fi:
        sc = json.loads(ln)
        if nh >= 40: break
        path = [(p_[0], p_[1], p_[2], 0.0, 1.0) for p_ in sc['points']]
        req_idx = int(len(path) * 0.0) + nh % 4
        om2.path = list(path); om2.seg = [0.0]
        for a_, b_ in zip(om2.path, om2.path[1:]): om2.seg.append(om2.seg[-1] + math.hypot(b_[0] - a_[0], b_[1] - a_[1]))
        om2.avoid_pending = (3, 1.0, req_idx); om2.avoid_tries = 0; om2.avoid_next = 0.0
        om2.avoiding, om2.avoid_end_idx = False, 0
        om2.hold_obstacle, om2.clear_since = True, 1.0
        om2.msg, om2.avoid_status, om2.avoid_note = '', 'planning', ''
        res = dict(sc['result']); res['id'] = 3; res['sensed'] = True
        om2.on_avoid_result(type('M', (), {'data': json.dumps(res)})())
        fh.write(json.dumps({'path': sc['points'], 'req_idx': req_idx, 'result': res,
                             'exp_path': [list(p_) for p_ in om2.path], 'exp_end_idx': int(om2.avoid_end_idx),
                             'exp_avoiding': bool(om2.avoiding), 'exp_tries': int(om2.avoid_tries),
                             'exp_status': om2.avoid_status, 'exp_msg': om2.msg, 'exp_hold': bool(om2.hold_obstacle)}) + '\n')
        nh += 1
print('wrote golden_handoff.jsonl (%d results applied)' % nh)

# ---- localization_manager: state machine + TF watchdog, real Python methods on a stub ----
from buggy_pcd_lanelet_map.localization_manager import LocalizationManager
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
lrng = np.random.default_rng(17)
nl = nstates = 0
os.makedirs(os.path.join(ROOT, 'localization'), exist_ok=True)
with open(os.path.join(ROOT, 'localization', 'golden_locstate.jsonl'), 'w') as fo:
    for scene in range(40):
        lm = LocalizationManager.__new__(LocalizationManager)
        T = [100.0]
        lm.now_s = lambda: T[0]
        lm.map_frame, lm.base_frame, lm.score_ok, lm.lost_timeout, lm.jump_m = 'map', 'base_link', 1.4, 3.0, 0.5
        lm.state, lm.msg, lm.activated = 'uninitialized', 'Give an initial pose (2D Pose Estimate)', False
        lm.score = lm.iters = lm.exe_ms = 0.0; lm.last_ndt = None
        lm.tf_parents, lm.jumps, lm.last_xy = {}, [], None
        pub = []
        lm.state_pub = type('P', (), {'publish': lambda s, m: pub.append(m.data)})()
        pose_box = [None]
        class B:
            def lookup_transform(s, a, b, t):
                if pose_box[0] is None: raise TransformException('no tf')
                x, y, yaw = pose_box[0]
                m = TransformStamped(); m.transform.translation.x, m.transform.translation.y = x, y
                m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2); return m
        lm.tf_buffer = B()
        lm.get_logger = lambda: type('L', (), {'error': lambda s, m: None, 'warn': lambda s, m: None, 'info': lambda s, m: None})()
        events = []
        x = y = 0.0
        lm.state, lm.msg, lm.activated = 'initialized', 'Seed sent, waiting for NDT', True; events.append(['set', 'initialized', 'Seed sent, waiting for NDT', True]); events.append(['reset'])
        for step in range(int(lrng.integers(25, 60))):
            T[0] += float(lrng.choice([0.05, 0.2, 0.5, 1.2, 2.5, 4.0]))
            r = lrng.random()
            if r < 0.35:
                edges = []
                for _ in range(int(lrng.integers(1, 4))):
                    kind = lrng.random()
                    if kind < 0.5:
                        x += float(lrng.normal(0, 0.3 if lrng.random() < 0.85 else 2.0)); y += float(lrng.normal(0, 0.3))
                        edges.append(['base_link', 'map', x, y])
                    elif kind < 0.7: edges.append(['base_link', 'odom', 0.0, 0.0])
                    elif kind < 0.85: edges.append(['velodyne', 'base_link', 1.73, 0.0])
                    else: edges.append(['base_link', 'old_map', 1.0, 1.0])
                m = TFMessage()
                for c, par, ex, ey in edges:
                    t = TransformStamped(); t.child_frame_id, t.header.frame_id = c, par
                    t.transform.translation.x, t.transform.translation.y = ex, ey; m.transforms.append(t)
                lm.on_tf(m); events.append(['tf', T[0], edges])
            elif r < 0.55:
                lm.on_ndt_pose(None); events.append(['ndt', T[0]])
            elif r < 0.65:
                v = float(np.float32(lrng.uniform(0.5, 3.0))); lm.score = v; events.append(['score', v])
            elif r < 0.68:
                v = float(lrng.integers(1, 30)); lm.iters = v; events.append(['iters', v])
            elif r < 0.71:
                v = float(np.float32(lrng.uniform(5, 90))); lm.exe_ms = v; events.append(['exe', v])
            elif r < 0.74:
                st = str(lrng.choice(['uninitialized', 'aligning', 'align_failed', 'initialized']))
                msg = 'm-' + st; act = bool(lrng.random() < 0.7)
                lm.state, lm.msg, lm.activated = st, msg, act; events.append(['set', st, msg, act])
                if st == 'initialized': lm.last_ndt = None; events.append(['reset'])
            else:
                if lrng.random() < 0.6: pose_box[0] = (float(lrng.uniform(-50, 50)), float(lrng.uniform(-50, 50)), float(lrng.uniform(-3, 3)))
                elif lrng.random() < 0.3: pose_box[0] = None
                lm.publish_state()
                events.append(['pub', T[0], list(pose_box[0]) if pose_box[0] else None, pub[-1]]); nstates += 1
        fo.write(json.dumps({'events': events}) + '\n'); nl += 1
print('wrote golden_locstate.jsonl (%d scenes, %d state texts)' % (nl, nstates))

# ---- uart_bridge: the real Python timer_callback fed with byte chunks (fake serial port) ----
import sys, types
try:
    import serial  # noqa
except ImportError:
    sys.modules['serial'] = types.SimpleNamespace(Serial=object, SerialException=Exception)
from buggy_uart.uart_bridge import UartBridge
brng = np.random.default_rng(23)
nb = nmsg = 0
with open(os.path.join(ROOT, 'actuation', 'golden_feedback.jsonl'), 'w') as fo:
    for scene in range(60):
        ub = UartBridge.__new__(UartBridge)
        ub.buf = bytearray(); got = []
        ub.get_logger = lambda: type('L', (), {k: (lambda s, m: None) for k in ('info', 'warn', 'error', 'debug', 'fatal')})()
        ub.get_clock = lambda: type('C', (), {'now': lambda s: type('N', (), {'to_msg': lambda s: __import__('builtin_interfaces.msg', fromlist=['Time']).Time()})()})()
        ub.pub = type('P', (), {'publish': lambda s, m: got.append([float(m.steering), float(m.speed)])})()
        chunks_hex = []; expected = []
        stream = bytearray()
        for _ in range(int(brng.integers(5, 40))):
            r = brng.random()
            if r < 0.7:      # a valid packet
                raw = int(brng.integers(-32768, 32768)) if brng.random() < 0.2 else int((brng.uniform(-24, 24) + 30) / 0.01)
                stream += bytes([40]) + int(raw).to_bytes(2, 'big', signed=True) + bytes([int(brng.integers(0, 12)), int(brng.integers(0, 256))]) + bytes([int(brng.integers(0, 256)) if brng.random() < 0.12 else 41])
            elif r < 0.85:   # garbage without a start byte
                stream += bytes(int(b) for b in brng.integers(0, 256, int(brng.integers(1, 9))) if int(b) != 40)
            else:            # garbage that may contain '('
                stream += bytes(int(b) for b in brng.integers(30, 50, int(brng.integers(1, 9))))
        pos = 0
        while pos < len(stream):
            k = int(brng.integers(1, 16)); chunk = bytes(stream[pos:pos + k]); pos += k
            data = [chunk]
            ub.ser = type('S', (), {'read': lambda s, n, d=data: d[0]})()
            before = len(got); ub.timer_callback()
            chunks_hex.append(chunk.hex()); expected.append(got[before:])
        fo.write(json.dumps({'chunks': chunks_hex, 'expected': expected}) + '\n'); nb += 1; nmsg += sum(len(e) for e in expected)
print('wrote golden_feedback.jsonl (%d streams, %d packets decoded)' % (nb, nmsg))

# ---- lidar_preprocess: the real Python on_cloud on synthetic scans --------------------------
from buggy_pcd_lanelet_map.lidar_preprocess import LidarPreprocess
prng2 = np.random.default_rng(31)
npre = 0
with open(os.path.join(ROOT, 'localization', 'golden_preprocess.jsonl'), 'w') as fo:
    for scene in range(12):
        lp = LidarPreprocess.__new__(LidarPreprocess)
        lp.leaf, lp.max_range, lp.min_range = [0.3, 0.3, 0.5, 0.2, 0.3, 1.0][scene % 6], 40.0, 0.5
        lp.box = (-2.5, 0.6, -0.9, 0.9, -1.2, 0.9); lp.min_period = 0.1; lp.last = None; lp.n = 0
        lp.get_logger = lambda: type('L', (), {'info': lambda s, m, **k: None, 'warn': lambda s, m, **k: None})()
        lp.get_clock = lambda: type('C', (), {'now': lambda s: type('N', (), {'nanoseconds': 10**12})()})()
        cap = []
        lp.pub = type('P', (), {'publish': lambda s, m: cap.append(m)})()
        n_ = int(prng2.integers(150, 6000)) if scene != 5 else 60       # scene 5: too few points
        ang = prng2.uniform(-math.pi, math.pi, n_); rad = prng2.uniform(0.2, 45, n_)
        P = np.column_stack([rad * np.cos(ang), rad * np.sin(ang), prng2.uniform(-1.3, 1.5, n_)]).astype(np.float32)
        P[::37] = np.nan; P[5::53, 1] = np.inf                       # no-return beams
        P[:20] = np.column_stack([prng2.uniform(-3, 1, 20), prng2.uniform(-1.1, 1.1, 20), prng2.uniform(-1.3, 1.0, 20)])   # the buggy itself
        m = PointCloud2(); m.height = 1; m.width = n_; m.point_step = 16; m.row_step = 16 * n_; m.is_dense = False
        m.fields = [PointField(name=n2, offset=o, datatype=PointField.FLOAT32, count=1) for n2, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
        buf = np.zeros((n_, 4), dtype=np.float32); buf[:, :3] = P; buf[:, 3] = prng2.uniform(0, 100, n_)
        m.data = buf.tobytes()
        lp.on_cloud(m)
        out = np.frombuffer(cap[0].data, dtype=np.float32).reshape(-1, 3) if cap else None
        fo.write(json.dumps({'voxel': lp.leaf, 'n': n_, 'points': [[(float(v) if math.isfinite(v) else ('nan' if math.isnan(v) else 'inf')) for v in r] for r in P.tolist()],
                             'out': None if out is None else [[float(v) for v in r] for r in out.tolist()]}) + '\n'); npre += 1
print('wrote golden_preprocess.jsonl (%d scans)' % npre)

# ---- ground-aware obstacle detector: the real Python on_ground / on_cloud ------------------
from buggy_pcd_lanelet_map.lidar_obstacle_detector import LidarObstacleDetector
from builtin_interfaces.msg import Time as BTime
grng = np.random.default_rng(41)
def mkcloud(P, stamp):
    m = PointCloud2(); n_ = len(P); m.height = 1; m.width = n_; m.point_step = 16; m.row_step = 16 * n_; m.is_dense = True
    m.fields = [PointField(name=k_, offset=o_, datatype=PointField.FLOAT32 if k_ != 'rgb' else PointField.UINT32, count=1) for k_, o_ in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
    b_ = np.zeros((n_, 4), np.float32); b_[:, :3] = P; m.data = b_.tobytes(); m.header.stamp = stamp; return m
def mkdet(aware, thin_low=True):
    d = LidarObstacleDetector.__new__(LidarObstacleDetector)
    d.sx, d.sz, d.min_h, d.max_h, d.half_width, d.length, d.cell, d.min_pts, d.max_hz = 1.73, 1.0, 0.2, 2.0, 1.25, 25.0, 0.3, 5, 10.0
    d.ground_min_h, d.ground_aware, d.reject_thin_low, d.thin_extent, d.low_height, d.ground_max_age = 0.30, aware, thin_low, 0.08, 0.5, 0.3
    d.last, d.path, d.prev, d.ground, d.rejected = 0.0, None, [], None, 0
    tick = [0]
    def nanos():
        tick[0] += 1
        return 10**12 + tick[0] * 10**9
    d.get_clock = lambda: type('C', (), {'now': lambda s: type('N', (), {'nanoseconds': nanos()})()})()
    d.pose = lambda: (0.0, 0.0, 0.0)
    d.got = []
    d.publish = lambda stamp, pose, cl: d.got.append(cl)
    return d
ngs = 0; n_confirmed = n_drop = n_aware_used = 0
with open(os.path.join(ROOT, 'perception', 'golden_groundaware.jsonl'), 'w') as fo:
    for scene in range(60):
        pitch = float(grng.uniform(-4, 6)); roll = float(grng.uniform(-2, 2))
        def gz_at(xb, yb): return -1.0 + math.tan(math.radians(pitch)) * xb + math.tan(math.radians(roll)) * yb
        xs_n = np.arange(2, 14, 0.5); xs_f = np.arange(14, 25, float(grng.choice([1.0, 2.5])))
        ys_ = np.arange(-2.0, 2.1, 0.4)
        Xn, Yn = np.meshgrid(np.r_[xs_n, xs_f], ys_)
        gX, gY = Xn.ravel(), Yn.ravel()
        gnd = np.column_stack([gX - 1.73, gY, gz_at(gX, gY) + grng.normal(0, float(grng.choice([0.01, 0.03])), len(gX))])
        ng = []
        if grng.random() < 0.6:     # ground leaked into the non-ground cloud (dense patch)
            x0_ = float(grng.uniform(8, 18)); LX, LY = np.meshgrid(np.arange(x0_, x0_ + float(grng.uniform(2, 7)), 0.2), np.arange(-1.6, 1.7, 0.2))
            LX, LY = LX.ravel(), LY.ravel(); ng.append(np.column_stack([LX - 1.73, LY, gz_at(LX, LY) + grng.normal(0, 0.02, len(LX)) + float(grng.uniform(0.0, 0.25))]))
        for _ in range(int(grng.integers(0, 3))):       # real objects
            cx = float(grng.uniform(5, 22)); cy = float(grng.uniform(-0.9, 0.9)); w = float(grng.uniform(0.3, 2.0)); h = float(grng.uniform(0.2, 1.8)); n_ = int(grng.integers(15, 90))
            ng.append(np.column_stack([cx - 1.73 + grng.uniform(-w / 2, w / 2, n_), cy + grng.uniform(-w / 2, w / 2, n_), gz_at(cx, cy) + grng.uniform(0.02, h, n_)]))
        if grng.random() < 0.4:       # a single thin ring (bump)
            cx = float(grng.uniform(10, 23)); n_ = int(grng.integers(6, 20))
            ng.append(np.column_stack([cx - 1.73 + grng.uniform(-0.4, 0.4, n_), grng.uniform(-0.8, 0.8, n_), gz_at(cx, 0) + float(grng.uniform(0.1, 0.45)) + grng.normal(0, 0.004, n_)]))
        ng = np.vstack(ng) if ng else np.zeros((0, 3))
        ng = ng.astype(np.float32); gnd = gnd.astype(np.float32)
        variant = scene % 4        # 0 aware, 1 aware but no ground cloud (stale), 2 not aware, 3 aware without thin/low rule
        aware = variant != 2; thin_low = variant != 3
        d = mkdet(aware, thin_low); t0_ = BTime(sec=10, nanosec=0)
        for _ in range(3):
            if aware and variant != 1: d.on_ground(mkcloud(gnd, t0_))
            d.on_cloud(mkcloud(ng, t0_))
        cl = d.got[-1]
        fo.write(json.dumps({'variant': variant, 'ground': [[float(v) for v in r_] for r_ in gnd.tolist()], 'nonground': [[float(v) for v in r_] for r_ in ng.tolist()],
                             'clusters': [[c['cx'], c['cy'], c['lx'], c['ly'], c['h'], c['n']] for c in cl], 'rejected': int(d.rejected) // 3}) + '\n')
        ngs += 1; n_confirmed += len(cl); n_drop += d.rejected
print('wrote golden_groundaware.jsonl (%d scenes, %d confirmed clusters)' % (ngs, n_confirmed))
print('wrote golden_avoid.jsonl (%d scenes, %d with a detour)' % (nsc, nok_sc))
print('wrote golden_tracking.txt (%d poses, %d object checks)' % (nq, nobj))
print('wrote golden_uart.txt (%d frames) and golden_cluster.txt (%d points -> %d clusters)'
      % (len(angles) * 4, len(pts), len(res)))
