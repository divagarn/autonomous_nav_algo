#!/usr/bin/env python3
"""GNSS check + georeference of a map built from a bag (optional step, only for bags that have GNSS).

  python3 gnss_georef.py --bag BAG --traj MAP_trajectory.csv -o MAP_gps_origin.yaml [--topic /chcnav/devpvt]

The map itself is built from the LiDAR only (build_map_from_bag.sh). With GNSS in the bag this tool
  1. reads the fixes of /chcnav/devpvt (msg_interfaces/Hcinspvatzcb: latitude, longitude, stat[1] = GNSS state)
     and keeps the best ones (RTK fixed, then RTK float, then anything with a fix),
  2. fits the map trajectory (x, y, matched by bag record time) to the GNSS track in a local East-North frame,
  3. writes <name>_gps_origin.yaml: latitude / longitude of the map origin and the heading of the map's +x
     axis (degrees clockwise from north) - enough to turn any GNSS fix into map coordinates later
     (latlon_to_map below) - plus how well the two tracks agree,
  4. writes <name>_gnss_check.png (both tracks on top of each other).
The agreement is also a DRIFT CHECK of the LiDAR-only map: scale far from 1, or an RMS of several metres, means
the LiDAR odometry drifted (re-record slower / with a loop). Nothing here changes the PCD.
"""
import argparse
import math
import os
import sys

import numpy as np

A = 6378137.0                      # WGS84
F = 1 / 298.257223563
E2 = F * (2 - F)
RTK_FIXED, RTK_FLOAT = (4, 8), (5, 9)          # stat[1] values (see Hcinspvatzcb.msg)


# ---- geodesy ------------------------------------------------------------------------------------
def geodetic_to_ecef(lat, lon, alt=0.0):
    lat, lon = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.array([(n + alt) * np.cos(lat) * np.cos(lon), (n + alt) * np.cos(lat) * np.sin(lon),
                     (n * (1 - E2) + alt) * np.sin(lat)])


def ecef_to_geodetic(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - E2))
    for _ in range(6):
        n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def _enu_matrix(lat0, lon0):
    la, lo = math.radians(lat0), math.radians(lon0)
    return np.array([[-math.sin(lo), math.cos(lo), 0.0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def geodetic_to_enu(lat, lon, alt, ref):
    """Arrays of lat/lon/alt -> N x 3 East-North-Up metres relative to ref = (lat0, lon0, alt0)."""
    ecef0 = geodetic_to_ecef(*ref)
    m = _enu_matrix(ref[0], ref[1])
    out = []
    for la, lo, al in zip(np.atleast_1d(lat), np.atleast_1d(lon), np.atleast_1d(alt)):
        out.append(m @ (geodetic_to_ecef(la, lo, al) - ecef0))
    return np.array(out)


def enu_to_geodetic(e, n, u, ref):
    ecef = geodetic_to_ecef(*ref) + _enu_matrix(ref[0], ref[1]).T @ np.array([e, n, u])
    return ecef_to_geodetic(*ecef)


# ---- fit ----------------------------------------------------------------------------------------
def fit_2d(src, dst, with_scale=False):
    """Least-squares  dst ~ s * R(theta) @ src + t  (Umeyama). Returns (theta [rad, ccw], t, s)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    u, d, vt = np.linalg.svd(xd.T @ xs / len(src))
    sgn = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sgn[1, 1] = -1
    r = u @ sgn @ vt
    s = float(np.trace(np.diag(d) @ sgn) / (xs ** 2).sum() * len(src)) if with_scale else 1.0
    theta = math.atan2(r[1, 0], r[0, 0])
    t = mu_d - s * (r @ mu_s)
    return theta, t, s


def apply_fit(theta, t, s, pts):
    c, sn = math.cos(theta), math.sin(theta)
    return s * (pts @ np.array([[c, sn], [-sn, c]])) + t


def select_fixes(stat, lat, lon):
    """Indices of the best fixes: RTK fixed, else RTK fixed + float, else any fix. Returns (index array, label)."""
    stat = np.asarray(stat)
    ok = (np.abs(lat) > 1e-6) | (np.abs(lon) > 1e-6)
    for label, states, need in (('RTK fixed', RTK_FIXED, 20), ('RTK fixed/float', RTK_FIXED + RTK_FLOAT, 20)):
        idx = np.where(ok & np.isin(stat, states))[0]
        if len(idx) >= need:
            return idx, label
    idx = np.where(ok & (stat > 0))[0]
    return idx, 'any fix (no RTK: accuracy of metres, the georeference is only approximate)'


def georeference(t_gnss, lat, lon, alt, stat, t_traj, xy, max_gap_s=0.5, yaw_map=None, lidar_xy=(1.73, 0.0)):
    """Fit the map trajectory xy (at times t_traj) to the GNSS fixes. Returns the result dict.
    With yaw_map (LiDAR heading per pose) it also estimates the lever arm GNSS <-> rear axle (lidar_xy = LiDAR position
    relative to the rear axle, vehicle frame)."""
    idx, label = select_fixes(stat, lat, lon)
    if len(idx) < 10:
        raise RuntimeError('fewer than 10 usable GNSS fixes in the bag (stat[1] > 0): no georeference possible')
    t_g = np.asarray(t_gnss, dtype=np.float64)[idx]
    o = np.argsort(t_g)
    t_g, la, lo, al = t_g[o], np.asarray(lat)[idx][o], np.asarray(lon)[idx][o], np.asarray(alt)[idx][o]
    ref = (float(la[0]), float(lo[0]), float(al[0]))
    enu = geodetic_to_enu(la, lo, al, ref)
    t_tr = np.asarray(t_traj, dtype=np.float64)
    inside = (t_tr >= t_g[0]) & (t_tr <= t_g[-1])
    k = np.clip(np.searchsorted(t_g, t_tr), 1, len(t_g) - 1)
    gap = np.minimum(np.abs(t_g[k] - t_tr), np.abs(t_g[k - 1] - t_tr)) * 1e-9
    use = inside & (gap < max_gap_s)
    if use.sum() < 10:
        raise RuntimeError('the GNSS fixes and the LiDAR scans do not overlap in time (bag record times)')
    g_xy = np.column_stack([np.interp(t_tr[use], t_g, enu[:, 0]), np.interp(t_tr[use], t_g, enu[:, 1])])
    m_xy = np.asarray(xy)[use]
    theta, t, _ = fit_2d(m_xy, g_xy)
    theta_s, _, scale = fit_2d(m_xy, g_xy, with_scale=True)
    res = np.linalg.norm(apply_fit(theta, t, 1.0, m_xy) - g_xy, axis=1)
    lat0, lon0, _ = enu_to_geodetic(t[0], t[1], 0.0, ref)
    heading = (90.0 - math.degrees(theta)) % 360.0           # +x axis of the map, clockwise from north
    path_m = float(np.linalg.norm(np.diff(m_xy, axis=0), axis=1).sum())
    path_g = float(np.linalg.norm(np.diff(g_xy, axis=0), axis=1).sum())
    lever = None
    if yaw_map is not None and use.sum() >= 50:
        dx, dy, rms_l = estimate_lever(m_xy, np.asarray(yaw_map)[use], g_xy)
        lever = {'gnss_minus_lidar_x_m': round(dx, 3), 'gnss_minus_lidar_y_m': round(dy, 3),
                 'rms_with_lever_m': round(rms_l, 3),
                 'gnss_to_rear_axle_x': round(-lidar_xy[0] - dx, 3), 'gnss_to_rear_axle_y': round(-lidar_xy[1] - dy, 3)}
    result = {
        'map_origin': {'latitude': round(lat0, 9), 'longitude': round(lon0, 9)},
        'map_x_heading_deg_clockwise_from_north': round(heading, 3),
        'transform': {'reference_latitude': ref[0], 'reference_longitude': ref[1], 'reference_altitude': ref[2],
                      'theta_deg_ccw_map_to_east': round(math.degrees(theta), 4),
                      'east_of_map_origin_m': round(float(t[0]), 4), 'north_of_map_origin_m': round(float(t[1]), 4)},
        'agreement': {'rms_m': round(float(np.sqrt((res ** 2).mean())), 3), 'max_m': round(float(res.max()), 3),
                      'scale_lidar_vs_gnss': round(scale, 4),
                      'path_length_lidar_m': round(path_m, 1), 'path_length_gnss_m': round(path_g, 1),
                      'points_used': int(use.sum())},
        'gnss': {'fixes_used': int(len(idx)), 'quality': label,
                 'rtk_fixed_fraction': round(float(np.isin(stat, RTK_FIXED).mean()), 3),
                 'fixes_in_bag': int(len(stat))},
    }
    if lever:
        result['lever_arm'] = lever
    return result, g_xy, apply_fit(theta, t, 1.0, m_xy)


def estimate_lever(m_xy, yaw_map, g_xy, iters=30):
    """Where is the GNSS position relative to the LiDAR (vehicle frame, x forward, y left)?  Model
    g_i = R(theta) * (m_i + Rz(yaw_i) * d) + t, solved alternately for (theta, t) and d. Needs a drive with curves.
    Returns (dx, dy, rms_m)."""
    m_xy, g_xy, yaw = np.asarray(m_xy), np.asarray(g_xy), np.asarray(yaw_map)
    d = np.zeros(2)
    c, s = np.cos(yaw), np.sin(yaw)
    theta = 0.0
    for _ in range(iters):
        off = np.column_stack([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        theta, t, _ = fit_2d(m_xy + off, g_xy)
        r = g_xy - apply_fit(theta, t, 1.0, m_xy + off) + apply_fit(theta, np.zeros(2), 1.0, off)   # residual + the offset part
        ct, st = math.cos(theta), math.sin(theta)
        r_map = np.column_stack([ct * r[:, 0] + st * r[:, 1], -st * r[:, 0] + ct * r[:, 1]])      # back into the map frame
        d = np.array([np.mean(c * r_map[:, 0] + s * r_map[:, 1]), np.mean(-s * r_map[:, 0] + c * r_map[:, 1])])
    off = np.column_stack([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    theta, t, _ = fit_2d(m_xy + off, g_xy)
    rms = float(np.sqrt((np.linalg.norm(apply_fit(theta, t, 1.0, m_xy + off) - g_xy, axis=1) ** 2).mean()))
    return float(d[0]), float(d[1]), rms


def heading_check(t_gnss, heading_cw, stat, t_traj, yaw_map_rad, theta_deg, max_gap_s=0.5):
    """Compare the GNSS dual-antenna heading (devpvt.heading2, degrees clockwise from north) with the heading the LiDAR
    trajectory has in the East-North frame. Returns {median_offset_deg, std_deg, samples} or None. The median is what the
    GNSS heading must be corrected by: gnss_pose_initializer.heading_offset_deg. Near 0 = the convention and the antenna
    mounting agree; 180 = antennas reversed; 90 = baseline across the vehicle."""
    t_g = np.asarray(t_gnss, dtype=np.float64)
    ok = np.isin(stat, RTK_FIXED + RTK_FLOAT + (1, 2, 3)) & np.isfinite(heading_cw)
    if ok.sum() < 10:
        return None
    t_g, h = t_g[ok], np.asarray(heading_cw)[ok]
    o = np.argsort(t_g)
    t_g, h = t_g[o], h[o]
    t_tr = np.asarray(t_traj, dtype=np.float64)
    k = np.clip(np.searchsorted(t_g, t_tr), 1, len(t_g) - 1)
    near = np.where(np.abs(t_g[k] - t_tr) < np.abs(t_g[k - 1] - t_tr), k, k - 1)
    use = np.abs(t_g[near] - t_tr) * 1e-9 < max_gap_s
    if use.sum() < 10:
        return None
    pred = (90.0 - (np.degrees(np.asarray(yaw_map_rad)[use]) + theta_deg)) % 360.0     # what the LiDAR trajectory says
    diff = (h[near][use] - pred + 180.0) % 360.0 - 180.0                                  # GNSS - LiDAR, -180..180
    med = float(np.median(diff))
    return {'median_offset_deg': round(med, 2), 'std_deg': round(float(np.std(((diff - med) + 180) % 360 - 180)), 2),
            'samples': int(use.sum())}


def latlon_to_map(geo, lat, lon, alt=0.0):
    """(lat, lon) -> (x, y) in the map frame, from the dict written by this tool (georeference())."""
    tr = geo['transform']
    ref = (tr['reference_latitude'], tr['reference_longitude'], tr['reference_altitude'])
    e, n, _ = geodetic_to_enu(lat, lon, alt, ref)[0]
    th = math.radians(tr['theta_deg_ccw_map_to_east'])
    d = np.array([e - tr['east_of_map_origin_m'], n - tr['north_of_map_origin_m']])
    c, s = math.cos(th), math.sin(th)
    return float(c * d[0] + s * d[1]), float(-s * d[0] + c * d[1])


# ---- bag / files --------------------------------------------------------------------------------
def read_devpvt(bag, topic='/chcnav/devpvt'):
    try:
        import rosbag2_py
        from msg_interfaces.msg import Hcinspvatzcb
        from rclpy.serialization import deserialize_message
    except ImportError as e:
        raise RuntimeError(f'{e}: source buggy_unified/install/setup.bash first (msg_interfaces is needed)')
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag)) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t, lat, lon, alt, stat, hd = [], [], [], [], [], []
    while r.has_next():
        _, data, rec = r.read_next()
        m = deserialize_message(data, Hcinspvatzcb)
        t.append(rec); lat.append(m.latitude); lon.append(m.longitude); alt.append(m.altitude); stat.append(m.stat[1])
        hd.append(m.heading2)
    return (np.array(t, dtype=np.float64), np.array(lat), np.array(lon), np.array(alt), np.array(stat), np.array(hd))


def bag_topics(bag):
    import yaml
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))
    return {t['topic_metadata']['name']: t['message_count'] for t in meta['rosbag2_bagfile_information']['topics_with_message_count']}


def write_yaml(path, result, bag):
    import yaml
    out = {'bag': os.path.abspath(bag), **result,
           'note': 'x, y of the map frame -> latlon_to_map() in tools/mapping/gnss_georef.py does the inverse'}
    with open(path, 'w') as f:
        f.write('# GNSS georeference of the LiDAR map (tools/mapping/gnss_georef.py)\n')
        yaml.safe_dump(out, f, sort_keys=False)


def plot(path, g_xy, m_xy):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(g_xy[:, 0], g_xy[:, 1], 'g-', lw=2, label='GNSS (East, North)')
    ax.plot(m_xy[:, 0], m_xy[:, 1], 'r--', lw=1.2, label='LiDAR map trajectory (fitted)')
    ax.plot(*g_xy[0], 'ko'); ax.plot(*g_xy[-1], 'ks')
    ax.set_aspect('equal'); ax.legend(); ax.set_title('GNSS vs LiDAR trajectory (m)'); ax.grid(alpha=.3)
    fig.savefig(path, dpi=70)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--traj', required=True, help='<map>_trajectory.csv from rebuild_flat_map.py (x,y,z,yaw,t_rec_ns)')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--topic', default='/chcnav/devpvt')
    ap.add_argument('--lidar-x', type=float, default=1.73, help='LiDAR ahead of the rear axle [m] (velodyne_static_tf.x)')
    ap.add_argument('--lidar-y', type=float, default=0.0, help='LiDAR left of the rear axle [m]')
    a = ap.parse_args()
    traj = np.genfromtxt(a.traj, delimiter=',', names=True)
    if 't_rec_ns' not in traj.dtype.names:
        sys.exit('the trajectory has no t_rec_ns column: build the map again with the current rebuild_flat_map.py')
    t, lat, lon, alt, stat, hd = read_devpvt(a.bag, a.topic)
    print(f'{len(t)} GNSS messages on {a.topic}')
    if len(t) == 0:
        sys.exit(f'no messages on {a.topic} in the bag')
    res, g_xy, m_xy = georeference(t, lat, lon, alt, stat, traj['t_rec_ns'], np.column_stack([traj['x'], traj['y']]),
                                   yaw_map=traj['yaw'], lidar_xy=(a.lidar_x, a.lidar_y))
    hc = heading_check(t, hd, stat, traj['t_rec_ns'], traj['yaw'], res['transform']['theta_deg_ccw_map_to_east'])
    if hc:
        res['heading_check'] = hc
    write_yaml(a.out, res, a.bag)
    plot(os.path.splitext(a.out)[0].replace('_gps_origin', '') + '_gnss_check.png', g_xy, m_xy)
    ag, gn = res['agreement'], res['gnss']
    print(f"GNSS quality: {gn['quality']} ({gn['fixes_used']} fixes, RTK fixed {gn['rtk_fixed_fraction'] * 100:.0f} %)")
    print(f"map origin: lat {res['map_origin']['latitude']:.7f} lon {res['map_origin']['longitude']:.7f}, "
          f"+x heading {res['map_x_heading_deg_clockwise_from_north']:.1f} deg from north")
    print(f"agreement: RMS {ag['rms_m']} m, max {ag['max_m']} m, scale {ag['scale_lidar_vs_gnss']} "
          f"(path LiDAR {ag['path_length_lidar_m']} m / GNSS {ag['path_length_gnss_m']} m)")
    lv = res.get('lever_arm')
    if lv:
        print(f"GNSS position vs LiDAR (vehicle frame): {lv['gnss_minus_lidar_x_m']:+.2f} m forward, {lv['gnss_minus_lidar_y_m']:+.2f} m left "
              f"(RMS {ag['rms_m']} -> {lv['rms_with_lever_m']} m with it) -> gnss_pose_initializer.gnss_to_rear_axle_x: "
              f"{lv['gnss_to_rear_axle_x']}, gnss_to_rear_axle_y: {lv['gnss_to_rear_axle_y']}  (LiDAR assumed at {a.lidar_x}, {a.lidar_y} from the rear axle)")
    if hc:
        print(f"GNSS heading vs LiDAR heading: offset {hc['median_offset_deg']} deg (spread {hc['std_deg']} deg, {hc['samples']} samples)"
              f" -> gnss_pose_initializer.heading_offset_deg: {hc['median_offset_deg']}"
              + ('' if abs(hc['median_offset_deg']) < 5 and hc['std_deg'] < 5 else '   <-- check the antenna mounting / heading2 convention'))
    else:
        print('no usable GNSS heading in the bag: the automatic start pose needs devpvt.heading2')
    if ag['rms_m'] > 2.0 or abs(ag['scale_lidar_vs_gnss'] - 1) > 0.03:
        print('WARNING: the LiDAR trajectory does not match the GNSS track well - the map probably drifted')
    print('wrote', a.out)


if __name__ == '__main__':
    main()
