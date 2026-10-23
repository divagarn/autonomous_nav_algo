"""Detour planning around an obstacle that blocks the route (pure Python/NumPy, no ROS).

Given the route ahead of the vehicle and the obstacles on it, build a smooth lateral detour that
leaves the lane centre line, passes the obstacle with a safe margin and rejoins the same route
afterwards, so the buggy can keep following its lanelet path. Forward driving only (the ECU has no
reverse): the detour starts from the vehicle's current pose and heading.

Geometry (all in Frenet coordinates of the route: s along it, e lateral, + left):
  offset profile  o(s) = 0 -> d (arc S-curve, radius R from the steering limit) -> hold d past the
                  obstacle -> d -> 0. Path point = route(s) + o(s) * left_normal(s).
  d               = (obstacle edge on the passing side) +/- (vehicle half width + side margin).
If the lane is blocked completely (a barrier across it, a wide vehicle), the detour leaves the lane:
with LiDAR data the drivable area outside the lane is whatever ground the LiDAR has actually seen and
found free (solid cells excluded), up to `max_offset` metres from the route; the extent of a barrier
is taken from the connected solid LiDAR cells, not only from the (corridor-clipped) detected boxes.
Candidate offsets are tried from the smallest that clears the obstacle outwards in `offset_step` steps.

A candidate is accepted only if its steering demand is within the limit and the whole swept body
footprint (rear axle frame: x in [-rear, front], y in [-half, half]) stays out of every obstacle
(inflated), out of occupied LiDAR cells, and inside the drivable area: the lane, or - up to
`offlane_allow` metres outside it - ground that the LiDAR has actually seen (if there is no LiDAR
data the lane + `offlane_allow` is assumed drivable, e.g. in the simulator).
"""
import math

import numpy as np

from buggy_pcd_lanelet_map.path_utils import max_steer_deg


class Params:
    wheelbase = 1.6
    max_steer_deg = 20.0
    steer_frac = 0.75          # plan arcs with this fraction of the steering limit
    check_steer_frac = 0.95    # the finished detour may use this fraction
    veh_half_width = 0.72
    side_margin = 0.35         # body-to-obstacle clearance
    rear = 0.7                 # body behind the rear axle
    front = 2.2                # body ahead of the rear axle
    footprint_margin = 0.15    # extra body margin used in the swept-area check
    avoid_half_width = 1.05    # lateral band around the route that counts as "blocking"
    lead = 1.0                 # the shift is complete this far before the bumper meets the obstacle
    tail = 0.6                 # rejoin starts this far behind the obstacle's far end (+ rear overhang)
    offlane_allow = 0.8        # drivable outside the lane by this much when there is no LiDAR data
    offlane_allow_sensed = 8.0  # ... by this much where the LiDAR has seen free ground
    max_offset = 6.5            # largest lateral shift from the route considered
    offset_step = 0.5
    ground_fraction = 0.65      # share of out-of-lane body samples that must lie on seen ground
    look_ahead = 40.0
    merge_gap = 8.0            # obstacles closer than this along the route form one zone

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Route:
    """Frenet helper for a polyline (N, 3)."""

    def __init__(self, pts):
        self.p = np.asarray(pts, dtype=float)
        d = np.hypot(*np.diff(self.p[:, :2], axis=0).T)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        t = np.gradient(self.p[:, :2], axis=0)
        n = np.hypot(t[:, 0], t[:, 1])
        n[n < 1e-9] = 1.0
        self.tx, self.ty = t[:, 0] / n, t[:, 1] / n
        self.nx, self.ny = -self.ty, self.tx          # left normal

    def to_frenet(self, x, y):
        """(s, e) of world points (arrays)."""
        from scipy.spatial import cKDTree
        if not hasattr(self, '_tree'):
            self._tree = cKDTree(self.p[:, :2])
        x, y = np.atleast_1d(x).astype(float), np.atleast_1d(y).astype(float)
        j = self._tree.query(np.column_stack([x, y]))[1]
        dx, dy = x - self.p[j, 0], y - self.p[j, 1]
        return (self.s[j] + dx * self.tx[j] + dy * self.ty[j],
                dx * self.nx[j] + dy * self.ny[j])


def box_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * dx - s * dy, y + s * dx + c * dy)
                     for dx, dy in ((-length / 2, -width / 2), (length / 2, -width / 2),
                                    (length / 2, width / 2), (-length / 2, width / 2))])


def zones_on_route(route, obstacles, prm):
    """Obstacle zones that block the route: list of dicts {s0, s1, lo, hi} (merged), nearest first."""
    raw = []
    for (x, y, yaw, length, width) in obstacles:
        cs, ce = route.to_frenet(*box_corners(x, y, yaw, length, width).T)
        s0, s1, lo, hi = cs.min(), cs.max(), ce.min(), ce.max()
        if s1 < 0.0 or s0 > prm.look_ahead:
            continue
        if hi < -prm.avoid_half_width or lo > prm.avoid_half_width:
            continue                       # beside the route, not blocking
        raw.append({'s0': s0, 's1': s1, 'lo': lo, 'hi': hi})
    raw.sort(key=lambda z: z['s0'])
    zones = []
    for z in raw:
        if zones and z['s0'] - zones[-1]['s1'] < prm.merge_gap:
            m = zones[-1]
            m['s1'], m['lo'], m['hi'] = max(m['s1'], z['s1']), min(m['lo'], z['lo']), max(m['hi'], z['hi'])
        else:
            zones.append(dict(z))
    return zones


def arc_shift(u, total, d, R):
    """Lateral offset after travelling u along an S-curve that shifts by d over length `total`."""
    sign = 1.0 if d >= 0 else -1.0
    half = total / 2.0
    a = np.clip(u, 0.0, total)
    first = R - np.sqrt(np.maximum(R * R - np.minimum(a, half) ** 2, 0.0))
    second = R - np.sqrt(np.maximum(R * R - np.maximum(total - a, 0.0) ** 2, 0.0))
    return sign * np.where(a <= half, first, abs(d) - second)


def shift_length(d, R):
    th = math.acos(max(-1.0, min(1.0, 1.0 - abs(d) / (2.0 * R))))
    return 2.0 * R * math.sin(th)


def build_detour(route, zone, d, R, prm):
    """(segment points, base points consumed) for lateral offset d, or None if it cannot be built."""
    L = shift_length(d, R)
    s_a0 = max(0.0, zone['s0'] - prm.front - prm.lead - L)
    s_hold_end = zone['s1'] + prm.rear + prm.tail
    s_end = s_hold_end + L
    if s_end >= route.s[-1] - 0.5:
        return None                         # route ends before the vehicle is back on it
    ib = int(np.searchsorted(route.s, s_end)) + 1
    ib = min(ib, len(route.s) - 1)
    s = route.s[:ib + 1]
    off = np.zeros_like(s)
    up = (s >= s_a0) & (s < s_a0 + L)
    off[up] = arc_shift(s[up] - s_a0, L, d, R)
    hold = (s >= s_a0 + L) & (s <= s_hold_end)
    off[hold] = d
    down = (s > s_hold_end) & (s < s_end)
    off[down] = d - arc_shift(s[down] - s_hold_end, L, d, R)
    xy = route.p[:ib + 1, :2] + off[:, None] * np.stack([route.nx[:ib + 1], route.ny[:ib + 1]], 1)
    seg = np.column_stack([xy, route.p[:ib + 1, 2]])
    return seg, ib + 1, L, s_a0


def footprint_samples(prm):
    xs = np.arange(-prm.rear, prm.front + 1e-6, 0.35)
    hw = prm.veh_half_width + prm.footprint_margin
    ys = np.linspace(-hw, hw, 7)
    return np.array([(x, y) for x in xs for y in ys])


class FreeSpace:
    """Optional LiDAR-derived grids (map frame): occupied cells and ground-seen cells."""

    def __init__(self, origin, res, occupied, ground):
        self.origin, self.res = np.asarray(origin, float), float(res)
        self.occ, self.gnd = occupied, ground

    def lookup(self, arr, xy):
        ij = np.floor((xy - self.origin) / self.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < arr.shape[0]) & (ij[:, 1] < arr.shape[1])
        val = np.zeros(len(xy), dtype=bool)
        val[ok] = arr[ij[ok, 0], ij[ok, 1]]
        return val, ok


def barrier_zones(route, free, prm):
    """Zones from the connected solid LiDAR cells that touch the route corridor: the true extent of a
    barrier (the detected boxes only cover the part inside the corridor)."""
    from scipy import ndimage
    labels, n = ndimage.label(free.occ, structure=np.ones((3, 3)))
    if n == 0:
        return []
    ij = np.argwhere(free.occ)
    xy = free.origin + (ij + 0.5) * free.res
    s, e = route.to_frenet(xy[:, 0], xy[:, 1])
    lab = labels[ij[:, 0], ij[:, 1]]
    ahead = (s >= 0.0) & (s <= prm.look_ahead + 10.0)
    touching = ahead & (np.abs(e) <= prm.avoid_half_width)
    zones = []
    for k in np.unique(lab[touching]):
        m = (lab == k) & ahead
        if m.sum() < 3:
            continue
        zones.append({'s0': float(s[m].min()), 's1': float(s[m].max()),
                      'lo': float(e[m].min()), 'hi': float(e[m].max())})
    return zones


def merge_zones(zones, prm):
    zones = sorted(zones, key=lambda z: z['s0'])
    out = []
    for z in zones:
        if out and z['s0'] - out[-1]['s1'] < prm.merge_gap:
            m = out[-1]
            m['s1'], m['lo'], m['hi'] = max(m['s1'], z['s1']), min(m['lo'], z['lo']), max(m['hi'], z['hi'])
        else:
            out.append(dict(z))
    return out


def swept_check(seg, obstacles, lane, free, prm):
    """(ok, reason). seg (N,3) detour points; lane = object with .inside(xy) and .dist(xy), or None."""
    pts = seg[:, :2]
    yaw = np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0]))
    F = footprint_samples(prm)
    c, s = np.cos(yaw), np.sin(yaw)
    wx = pts[:, 0, None] + c[:, None] * F[None, :, 0] - s[:, None] * F[None, :, 1]
    wy = pts[:, 1, None] + s[:, None] * F[None, :, 0] + c[:, None] * F[None, :, 1]
    W = np.stack([wx.ravel(), wy.ravel()], 1)
    for (x, y, oyaw, length, width) in obstacles:
        cc, ss = math.cos(oyaw), math.sin(oyaw)
        rx, ry = W[:, 0] - x, W[:, 1] - y
        lx, ly = cc * rx + ss * ry, -ss * rx + cc * ry
        if np.any((np.abs(lx) <= length / 2 + 0.1) & (np.abs(ly) <= width / 2 + 0.1)):
            return False, 'the detour would hit an obstacle'
    if free is not None:
        occ, known = free.lookup(free.occ, W)
        if occ.any():
            return False, 'LiDAR sees something solid beside the obstacle (no room to pass)'
    if lane is not None:
        inside = lane.inside(W)
        if not inside.all():
            out = W[~inside]
            d = lane.dist(out)
            sensed = free is not None and free.gnd is not None
            limit = prm.offlane_allow_sensed if sensed else prm.offlane_allow
            if (d > limit).any():
                return False, f'would leave the lane by more than {limit:.1f} m'
            if sensed:
                g, known = free.lookup(free.gnd, out)
                if (~g).mean() > 1.0 - prm.ground_fraction:
                    return False, 'the LiDAR has not seen free ground where the detour leaves the lane'
    return True, ''


def plan_detour(points, obstacles, lane=None, free=None, prm=None):
    """points: route ahead (N,3), starting at the vehicle's rear axle. obstacles: [(x,y,yaw,l,w)].
    Returns dict {ok, reason, ...}; on success includes 'segment' (list of [x,y,z]) and 'ib'."""
    prm = prm or Params()
    route = Route(points)
    zones = zones_on_route(route, obstacles, prm)
    if free is not None:
        zones = merge_zones(zones + barrier_zones(route, free, prm), prm)
    if not zones:
        return {'ok': False, 'reason': 'no obstacle on the route ahead'}
    z = zones[0]
    clr = prm.veh_half_width + prm.side_margin
    R0 = prm.wheelbase / math.tan(math.radians(prm.max_steer_deg * prm.steer_frac))
    # bumper-to-obstacle gap a forward-only shift around this zone needs (also reported when it works)
    need = shift_length(clr + (z['hi'] - z['lo']) / 2, R0) + prm.lead
    best, why = None, []
    for side, d0 in (('left', z['hi'] + clr), ('right', z['lo'] - clr)):
        last, sign = '', (1.0 if d0 > 0 else -1.0)
        # smallest shift that clears the obstacle first, then wider ones (a blocked lane may only have
        # free space further out); for each, the sharpest allowed arc first, then ever gentler ones
        # (turn the body less, but need more room before the obstacle)
        n_off = int(max(0.0, prm.max_offset - abs(d0)) / prm.offset_step) + 1
        for step in range(n_off):
            d = d0 + sign * prm.offset_step * step
            for k in (1.0, 1.4, 2.0, 2.8, 4.0):
                R = R0 * k
                built = build_detour(route, z, d, R, prm)
                if built is None:
                    last = last or 'the route ends before the vehicle could rejoin it'
                    break
                seg, ib, L, s_a0 = built
                if max_steer_deg(list(map(tuple, seg)), prm.wheelbase) > prm.max_steer_deg * prm.check_steer_frac:
                    last = 'needs more steering than the buggy has'
                    continue
                ok, reason = swept_check(seg, obstacles, lane, free, prm)
                if not ok:
                    last = reason
                    continue
                inside = lane.inside(seg[:, :2]) if lane is not None else np.ones(len(seg), dtype=bool)
                best_side = {'ok': True, 'side': side, 'offset': float(d), 'segment': seg.tolist(),
                             'ib': ib, 'length': float(route.s[ib - 1]), 'shift_length': float(L),
                             'radius': float(R), 'room_before': float(z['s0'] - prm.front),
                             'outside_lane': bool(not inside.all()),
                             'inside': [bool(v) for v in inside], 'zone': z, 'room_needed': float(need)}
                last = ''
                break
            if not last:
                if best is None or abs(d) < abs(best['offset']):
                    best = best_side
                break
            if built is None:
                break
        if last:
            why.append(f'{side}: {last}')
    if best is not None:
        return best
    # why too close? the shift needs `shift_length + lead` before the bumper reaches the obstacle
    have = z['s0'] - prm.front
    extra = f' (needs ~{need:.1f} m before the obstacle, have {have:.1f} m)' if have < need else ''
    return {'ok': False, 'reason': '; '.join(why) + extra if why else 'no feasible detour' + extra,
            'room_before': float(have), 'zone': z, 'room_needed': float(need)}


def free_space_cells(points, zone, free, lane, prm=None, stride=2, max_cells=4000):
    """Free ground outside the lane around a blocked zone, for the RViz display: cell centres (N, 2)
    of cells the LiDAR has seen as ground, that are not solid and not inside a lane, within
    max_offset of the route and 6 m before / after the zone. Every `stride`-th cell, raster order."""
    prm = prm or Params()
    if free is None or free.gnd is None:
        return np.zeros((0, 2))
    route = Route(points)
    ok = free.gnd & ~free.occ
    ij = np.argwhere(ok)
    ij = ij[(ij[:, 0] % stride == 0) & (ij[:, 1] % stride == 0)]
    if not len(ij):
        return np.zeros((0, 2))
    xy = free.origin + (ij + 0.5) * free.res
    near = (np.abs(xy[:, 0] - route.p[:, 0].mean()) < 200)      # cheap guard, the grid is local anyway
    xy = xy[near]
    s, e = route.to_frenet(xy[:, 0], xy[:, 1])
    m = (s >= zone['s0'] - 6.0) & (s <= zone['s1'] + 6.0) & (np.abs(e) <= prm.max_offset)
    xy = xy[m]
    if lane is not None and len(xy):
        xy = xy[~lane.inside(xy)]
    if len(xy) > max_cells:
        xy = xy[np.linspace(0, len(xy) - 1, max_cells).astype(int)]
    return xy
