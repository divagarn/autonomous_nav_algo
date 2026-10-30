"""gnss_georef: geodesy round trip and recovery of a known georeference (pytest, no ROS)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gnss_georef as G  # noqa: E402

REF = (28.6139, 77.2090, 216.0)


def synthetic(theta_deg=37.0, origin_enu=(5.0, -3.0), n=400, noise=2e-8, stat=4):
    s = np.linspace(0, 60, n)
    r = 30.0
    m_xy = np.column_stack([r * np.sin(s / r), r * (1 - np.cos(s / r))])
    th = math.radians(theta_deg)
    c, sn = math.cos(th), math.sin(th)
    enu = m_xy @ np.array([[c, sn], [-sn, c]]) + np.array(origin_enu)
    ll = np.array([G.enu_to_geodetic(e, nn, 0.0, REF)[:2] for e, nn in enu])
    rng = np.random.default_rng(1)
    lat = ll[:, 0] + rng.normal(0, noise, n)
    lon = ll[:, 1] + rng.normal(0, noise, n)
    t = np.arange(n) * 1e8
    return t, lat, lon, np.full(n, 216.0), np.full(n, stat), m_xy


def test_geodetic_round_trip():
    e, n, u = G.geodetic_to_enu([28.6142], [77.2093], [220.0], REF)[0]
    lat, lon, alt = G.enu_to_geodetic(e, n, u, REF)
    assert (lat, lon) == pytest.approx((28.6142, 77.2093), abs=1e-9) and alt == pytest.approx(220.0, abs=1e-3)
    assert 20 < math.hypot(e, n) < 60                      # 0.0003 deg ~ 33 m, 0.0003 deg lon ~ 29 m


def test_recovers_heading_origin_and_has_zero_drift():
    t, lat, lon, alt, stat, m_xy = synthetic()
    res, _, _ = G.georeference(t, lat, lon, alt, stat, t, m_xy)
    assert res['map_x_heading_deg_clockwise_from_north'] == pytest.approx(53.0, abs=0.05)
    la, lo, _ = G.enu_to_geodetic(5.0, -3.0, 0.0, REF)
    assert (res['map_origin']['latitude'], res['map_origin']['longitude']) == pytest.approx((la, lo), abs=1e-7)
    assert res['agreement']['rms_m'] < 0.05 and res['agreement']['scale_lidar_vs_gnss'] == pytest.approx(1.0, abs=1e-3)


def test_latlon_to_map_inverts_the_fit():
    t, lat, lon, alt, stat, m_xy = synthetic()
    res, _, _ = G.georeference(t, lat, lon, alt, stat, t, m_xy)
    x, y = G.latlon_to_map(res, lat[123], lon[123])
    assert (x, y) == pytest.approx(tuple(m_xy[123]), abs=0.05)


def test_drift_shows_as_scale_and_rms():
    t, lat, lon, alt, stat, m_xy = synthetic()
    res, _, _ = G.georeference(t, lat, lon, alt, stat, t, m_xy * 0.9)       # LiDAR path 10 % too short
    assert res['agreement']['scale_lidar_vs_gnss'] == pytest.approx(1 / 0.9, abs=0.01)
    assert res['agreement']['rms_m'] > 0.5


def test_picks_the_best_fix_class_and_needs_overlap():
    t, lat, lon, alt, stat, m_xy = synthetic(stat=1)
    res, _, _ = G.georeference(t, lat, lon, alt, stat, t, m_xy)
    assert 'no RTK' in res['gnss']['quality']
    with pytest.raises(RuntimeError):
        G.georeference(t, lat, lon, alt, np.zeros(len(t)), t, m_xy)            # no fix at all
    with pytest.raises(RuntimeError):
        G.georeference(t, lat, lon, alt, stat, t + 1e12, m_xy)                 # other time span


def test_georeference_pcd_puts_the_cloud_in_east_north(tmp_path):
    import georeference_pcd as GP
    t, lat, lon, alt, stat, m_xy = synthetic()
    res, _, _ = G.georeference(t, lat, lon, alt, stat, t, m_xy)
    out = GP.rotate_to_enu(np.column_stack([m_xy, np.zeros(len(m_xy))]), res['transform']['theta_deg_ccw_map_to_east'])
    o = res['map_origin']
    ref = (o['latitude'], o['longitude'], 0.0)
    enu = G.geodetic_to_enu(lat, lon, np.zeros(len(lat)), ref)          # truth: GNSS track relative to the map origin
    assert np.abs(out[:, :2] - enu[:, :2]).max() < 0.05
    assert 'LocalCartesian' in GP.projector_yaml(res) and f"{o['latitude']:.9f}" in GP.projector_yaml(res)


def test_heading_check_finds_an_antenna_offset():
    t, lat, lon, alt, stat, m_xy = synthetic()
    yaw_map = np.arctan2(np.gradient(m_xy[:, 1]), np.gradient(m_xy[:, 0]))
    theta = 37.0
    true_cw = (90.0 - (np.degrees(yaw_map) + theta)) % 360.0
    ok = G.heading_check(t, true_cw, stat, t, yaw_map, theta)
    assert abs(ok['median_offset_deg']) < 0.1 and ok['std_deg'] < 0.1
    off = G.heading_check(t, (true_cw + 7.0) % 360.0, stat, t, yaw_map, theta)      # GNSS heading 7 deg too large
    assert off['median_offset_deg'] == pytest.approx(7.0, abs=0.1)
    rev = G.heading_check(t, (true_cw + 180.0) % 360.0, stat, t, yaw_map, theta)    # antennas the wrong way round
    assert abs(abs(rev['median_offset_deg']) - 180.0) < 0.1
    assert G.heading_check(t[:5], true_cw[:5], stat[:5], t[:5], yaw_map[:5], theta) is None


def test_lever_arm_is_estimated_from_a_curved_drive():
    # GNSS antenna 1.17 m behind and 0.42 m right of the LiDAR; an S-shaped drive gives the geometry for it
    s = np.linspace(0, 120, 800)
    m_xy = np.column_stack([s, 12 * np.sin(s / 14.0)])
    yaw = np.arctan2(np.gradient(m_xy[:, 1]), np.gradient(m_xy[:, 0]))
    d = np.array([-1.17, -0.42])
    ant = m_xy + np.column_stack([np.cos(yaw) * d[0] - np.sin(yaw) * d[1], np.sin(yaw) * d[0] + np.cos(yaw) * d[1]])
    th = math.radians(37.0)
    c, sn = math.cos(th), math.sin(th)
    enu = ant @ np.array([[c, sn], [-sn, c]]) + np.array([5.0, -3.0])
    ll = np.array([G.enu_to_geodetic(e, n, 0.0, REF)[:2] for e, n in enu])
    t = np.arange(len(s)) * 1e8
    res, _, _ = G.georeference(t, ll[:, 0], ll[:, 1], np.full(len(s), 216.0), np.full(len(s), 4), t, m_xy, yaw_map=yaw)
    lv = res['lever_arm']
    assert lv['gnss_minus_lidar_x_m'] == pytest.approx(-1.17, abs=0.05) and lv['gnss_minus_lidar_y_m'] == pytest.approx(-0.42, abs=0.05)
    assert lv['rms_with_lever_m'] < 0.05 < res['agreement']['rms_m'] * 10 or lv['rms_with_lever_m'] < res['agreement']['rms_m']
    # rear axle = LiDAR - (1.73, 0); relative to the antenna: (-1.73 + 1.17, +0.42)
    assert lv['gnss_to_rear_axle_x'] == pytest.approx(-1.73 + 1.17, abs=0.05) and lv['gnss_to_rear_axle_y'] == pytest.approx(0.42, abs=0.05)
