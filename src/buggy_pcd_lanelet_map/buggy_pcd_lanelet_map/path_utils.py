"""Path helpers shared by route_planner (smoothing) and operation_manager
(pure-pursuit tracking). Pure Python, no ROS imports.

Vehicle limits come from the existing buggy stack: wheelbase 1.6 m and
max steering 20 deg (steering_calculator_node / buggy.urdf.xacro), so the
minimum turning radius is wheelbase / tan(max_steer) = ~4.4 m.
"""
import math


def resample(pts, step):
    """Resample a 3D polyline at ~constant arclength spacing (endpoints kept)."""
    d = [0.0]
    for a, b in zip(pts, pts[1:]):
        d.append(d[-1] + math.dist(a, b))
    total = d[-1]
    if total <= 0.0:
        return list(pts[:1]) * 2
    n = max(2, int(round(total / step)) + 1)
    out, j = [], 0
    for i in range(n):
        s = total * i / (n - 1)
        while j < len(d) - 2 and d[j + 1] < s:
            j += 1
        seg = d[j + 1] - d[j]
        t = 0.0 if seg <= 0 else (s - d[j]) / seg
        a, b = pts[j], pts[j + 1]
        out.append(tuple(a[k] + t * (b[k] - a[k]) for k in range(3)))
    return out


def curvatures(pts):
    """Signed curvature (+left) at every point, 3-point Menger formula."""
    k = [0.0] * len(pts)
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i - 1][0], pts[i - 1][1]
        bx, by = pts[i][0], pts[i][1]
        cx, cy = pts[i + 1][0], pts[i + 1][1]
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        den = math.hypot(bx - ax, by - ay) * math.hypot(cx - bx, cy - by) * \
            math.hypot(cx - ax, cy - ay)
        k[i] = 0.0 if den < 1e-9 else 2.0 * cross / den
    return k


def steer_deg_for_curvature(kappa, wheelbase):
    return math.degrees(math.atan(wheelbase * kappa))


def max_steer_deg(pts, wheelbase):
    ks = curvatures(pts)
    return max((abs(steer_deg_for_curvature(k, wheelbase)) for k in ks), default=0.0)


def smooth(pts, wheelbase, max_steer, max_dev, spacing=0.5, max_iter=3000):
    """Make a path drivable for the given steering limit.

    Resamples, then relaxes interior points toward their neighbours' mean
    (shrinks curvature) while keeping every point within max_dev metres of
    the original centreline (stay in lane). Endpoints are pinned. Stops as
    soon as the peak steering demand is within max_steer.

    Returns (smoothed_pts, peak_steer_before_deg, peak_steer_after_deg).
    """
    base = resample(pts, spacing)
    before = max_steer_deg(base, wheelbase)
    if len(base) < 3 or before <= max_steer:
        return base, before, before
    cur = [list(p) for p in base]
    for _ in range(max_iter):
        new = [c[:] for c in cur]
        for i in range(1, len(cur) - 1):
            for k in (0, 1):
                new[i][k] = cur[i][k] + 0.5 * (0.5 * (cur[i - 1][k] + cur[i + 1][k]) - cur[i][k])
            dx, dy = new[i][0] - base[i][0], new[i][1] - base[i][1]
            dist = math.hypot(dx, dy)
            if dist > max_dev:
                new[i][0] = base[i][0] + dx * max_dev / dist
                new[i][1] = base[i][1] + dy * max_dev / dist
        cur = new
        if max_steer_deg(cur, wheelbase) <= max_steer:
            break
    out = [tuple(p) for p in cur]
    return out, before, max_steer_deg(out, wheelbase)
