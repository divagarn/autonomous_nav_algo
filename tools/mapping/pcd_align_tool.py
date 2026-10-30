#!/usr/bin/env python3
"""
Interactive PCD map alignment tool.

  python3 scripts/pcd_align_tool.py /home/divagar/new_delhi_bag_3_map.pcd
  python3 scripts/pcd_align_tool.py map.pcd -o map_fixed.pcd

How it works
  * The map is shown top-down, coloured by capture order (point index) or height.
  * Drag a rectangle on the left map to select a region (every z inside it).
  * The points in the region are split into "passes": runs of consecutive
    capture indices. A place the buggy drove past twice (start/end of a loop)
    shows up as two passes. Right-hand views show the passes in top-down and
    side (x-z) view so you can SEE the misalignment.
  * Pick the reference pass (the one that is right), press "Align others".
    Each other pass is ICP-registered (point-to-plane) to the reference using
    only the region points. The resulting rigid correction is applied to that
    pass's whole capture-index range and tapers smoothly to zero over the
    "taper" amount of points before/after, so the fix stays local to that path
    and does not tear the rest of the map.
  * Undo / Reset / Save are available. Save writes a new PCD (original untouched).

Keys: u = undo, s = save, a = align others, c = toggle colour (order/height)
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R


# --------------------------------------------------------------------------- #
# PCD I/O
# --------------------------------------------------------------------------- #
def load_pcd(path):
    """Return Nx3 float32 xyz from an ascii or binary PCD."""
    with open(path, 'rb') as f:
        hdr = {}
        nlines = 0
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            nlines += 1
            if not line or line.startswith('#'):
                continue
            k, *v = line.split()
            hdr[k.upper()] = v
            if k.upper() == 'DATA':
                break
        fields = hdr['FIELDS']
        sizes = [int(s) for s in hdr['SIZE']]
        types = hdr['TYPE']
        counts = [int(c) for c in hdr.get('COUNT', ['1'] * len(fields))]
        npts = int(hdr['POINTS'][0])
        fmt = hdr['DATA'][0].lower()
        ix = [fields.index(a) for a in ('x', 'y', 'z')]
        if fmt == 'ascii':
            f.close()
            data = np.loadtxt(path, skiprows=nlines, usecols=ix, dtype=np.float32, ndmin=2)
        elif fmt == 'binary':
            tmap = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
                    ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
            dt = []
            for name, s, t, c in zip(fields, sizes, types, counts):
                dt.append((name, tmap[(t, s)]) if c == 1 else (name, tmap[(t, s)], (c,)))
            arr = np.frombuffer(f.read(), dtype=np.dtype(dt), count=npts)
            data = np.stack([arr[a].astype(np.float32) for a in ('x', 'y', 'z')], axis=1)
        else:
            raise SystemExit('Unsupported PCD DATA format: %s (use ascii or binary)' % fmt)
    data = data[np.isfinite(data).all(axis=1)]
    return np.ascontiguousarray(data)


def save_pcd(path, xyz):
    n = len(xyz)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
           "TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % (n, n))
    with open(path, 'w') as f:
        f.write(hdr)
        np.savetxt(f, xyz, fmt='%.4f')


# --------------------------------------------------------------------------- #
# Core geometry (no GUI dependency)
# --------------------------------------------------------------------------- #
def voxel_down(x, v):
    k = np.floor(x / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return x[i]


def split_passes(idx, gap):
    """idx: sorted point indices inside the region -> list of index arrays (one per pass)."""
    if len(idx) == 0:
        return []
    cuts = np.where(np.diff(idx) > gap)[0] + 1
    return [p for p in np.split(idx, cuts) if len(p) > 0]


def icp_point_to_plane(src, dst, voxel=0.2, yaw_only=False,
                       stages=((3.0, 10), (1.5, 15), (0.8, 20), (0.4, 20)), trim=0.85):
    """Rigid transform (Rm, t, c) so that  (p - c) @ Rm.T + t + c  maps src onto dst. None on failure."""
    c = dst.mean(0)
    S0 = voxel_down(src.astype(np.float64) - c, voxel)
    D = voxel_down(dst.astype(np.float64) - c, voxel)
    if len(S0) < 50 or len(D) < 50:
        return None
    tree = cKDTree(D)
    _, nb = tree.query(D, k=10)
    q = D[nb]
    q = q - q.mean(1, keepdims=True)
    cov = np.einsum('mki,mkj->mij', q, q)
    nrm = np.linalg.eigh(cov)[1][:, :, 0]
    Rm, t = np.eye(3), np.zeros(3)
    matches, resid = 0, np.nan
    for d, iters in stages:
        for _ in range(iters):
            S = S0 @ Rm.T + t
            dist, idx = tree.query(S, distance_upper_bound=d)
            m = np.isfinite(dist)
            if m.sum() < 30:
                return None
            m &= dist <= np.quantile(dist[m], trim)
            s, qd, nn = S[m], D[idx[m]], nrm[idx[m]]
            r = np.einsum('ij,ij->i', s - qd, nn)
            J = np.hstack([np.cross(s, nn), nn])
            if yaw_only:
                J = J[:, [2, 3, 4, 5]]
            x = np.linalg.lstsq(J.T @ J + 1e-6 * np.eye(J.shape[1]), -J.T @ r, rcond=None)[0]
            if yaw_only:
                x = np.array([0, 0, x[0], x[1], x[2], x[3]])
            dR = R.from_rotvec(x[:3]).as_matrix()
            Rm, t = dR @ Rm, dR @ t + x[3:]
            matches, resid = int(m.sum()), float(np.abs(r).mean())
    return Rm, t, c, matches, resid


def apply_local(P, a, b, taper, Rm, t, c):
    """Apply (Rm,t,c) to P[a..b] fully; smoothstep-fade to identity over `taper` points either side."""
    n = len(P)
    lo, hi = max(0, a - taper), min(n, b + 1 + taper)
    i = np.arange(lo, hi)
    w = np.ones(len(i))
    if taper > 0:
        L, Rr = i < a, i > b
        w[L] = (i[L] - lo + 0.5) / taper
        w[Rr] = (hi - i[Rr] - 0.5) / taper
        w = w * w * (3 - 2 * w)
    rv = R.from_matrix(Rm).as_rotvec()
    lv = np.round(w * 64).astype(int)
    for level in np.unique(lv):
        if level == 0:
            continue
        f = level / 64.0
        sel = i[lv == level]
        Rf = R.from_rotvec(rv * f).as_matrix()
        P[sel] = ((P[sel].astype(np.float64) - c) @ Rf.T + t * f + c).astype(np.float32)


def nn_stats(src, dst, gate=2.0, cap=20000):
    if len(src) == 0 or len(dst) == 0:
        return float('nan'), 0.0
    if len(src) > cap:
        src = src[:: len(src) // cap + 1]
    d, _ = cKDTree(dst).query(src, distance_upper_bound=gate)
    d = d[np.isfinite(d)]
    if len(d) == 0:
        return float('nan'), 0.0
    return float(np.median(d)), float(np.mean(d < 0.2))


# --------------------------------------------------------------------------- #
# Tool state + GUI
# --------------------------------------------------------------------------- #
PASS_COLORS = ['tab:red', 'tab:orange', 'tab:green', 'tab:purple', 'tab:brown', 'tab:pink']


class AlignTool:
    def __init__(self, pts, out_path, gap_frac=0.02, headless=False):
        import matplotlib
        if headless:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.widgets import RectangleSelector, Button, Slider, RadioButtons, CheckButtons
        self.plt, self.RadioButtons = plt, RadioButtons

        self.P = pts.copy()
        self.P0 = pts.copy()
        self.n = len(pts)
        self.out_path = out_path
        self.gap = max(1, int(gap_frac * self.n))
        self.undo = []
        self.region = None
        self.passes = []
        self.ref = 0
        self.color_mode = 'order'
        self.sub = np.arange(0, self.n, max(1, self.n // 400000))

        self.fig = plt.figure(figsize=(17, 9))
        self.fig.canvas.manager.set_window_title('PCD align tool') if hasattr(self.fig.canvas, 'manager') and self.fig.canvas.manager else None
        self.ax_map = self.fig.add_axes([0.04, 0.16, 0.44, 0.78])
        self.ax_top = self.fig.add_axes([0.53, 0.56, 0.27, 0.38])
        self.ax_side = self.fig.add_axes([0.53, 0.16, 0.27, 0.34])
        self.ax_radio = self.fig.add_axes([0.82, 0.58, 0.17, 0.36])
        self.ax_radio.set_title('Reference pass', fontsize=9)
        self.ax_radio.axis('off')
        self.status = self.fig.text(0.04, 0.02, 'Drag a rectangle on the left map to select a region.', fontsize=10)

        self.sc = self.ax_map.scatter(self.P[self.sub, 0], self.P[self.sub, 1], c=self.sub / self.n,
                                      s=0.2, cmap='jet', rasterized=True)
        self.ax_map.set_aspect('equal')
        self.ax_map.set_title('Map (colour = capture order). Drag to select region.')
        self.rect_patch = None
        self.selector = RectangleSelector(self.ax_map, self.on_select, useblit=False, button=[1],
                                          minspanx=2, minspany=2, spancoords='data', interactive=True)

        def mk(rect, label, cb):
            b = Button(self.fig.add_axes(rect), label)
            b.on_clicked(cb)
            return b
        self.btns = [
            mk([0.82, 0.48, 0.17, 0.04], 'Align others to reference (a)', lambda e: self.align()),
            mk([0.82, 0.43, 0.08, 0.04], 'Undo (u)', lambda e: self.do_undo()),
            mk([0.91, 0.43, 0.08, 0.04], 'Reset all', lambda e: self.reset()),
            mk([0.82, 0.38, 0.17, 0.04], 'Colour: order/height (c)', lambda e: self.toggle_color()),
            mk([0.82, 0.33, 0.17, 0.04], 'Save PCD (s)', lambda e: self.save()),
        ]
        self.chk = CheckButtons(self.fig.add_axes([0.82, 0.25, 0.17, 0.06]), ['yaw + xyz only (no roll/pitch)'], [False])
        self.sl_taper = Slider(self.fig.add_axes([0.84, 0.19, 0.13, 0.025]), 'Taper\n(% pts)', 0, 30, valinit=5, valstep=0.5)
        self.sl_gap = Slider(self.fig.add_axes([0.84, 0.14, 0.13, 0.025]), 'Pass gap\n(% pts)', 0.2, 10,
                             valinit=gap_frac * 100, valstep=0.2)
        self.sl_gap.on_changed(self.on_gap)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    # ---- helpers ----
    def say(self, msg):
        print(msg)
        self.status.set_text(msg)
        self.fig.canvas.draw_idle()

    def on_key(self, e):
        {'u': self.do_undo, 's': self.save, 'a': self.align, 'c': self.toggle_color}.get(e.key, lambda: None)()

    def on_gap(self, v):
        self.gap = max(1, int(v / 100.0 * self.n))
        if self.region is not None:
            self.analyze()

    def toggle_color(self):
        self.color_mode = 'height' if self.color_mode == 'order' else 'order'
        self.refresh_map()

    def refresh_map(self):
        self.sc.set_offsets(self.P[self.sub, :2])
        if self.color_mode == 'order':
            self.sc.set_array(self.sub / self.n)
            self.sc.set_cmap('jet')
        else:
            z = self.P[self.sub, 2]
            self.sc.set_array(z)
            self.sc.set_cmap('viridis')
        self.sc.autoscale()
        self.ax_map.set_title('Map (colour = capture %s). Drag to select region.' % self.color_mode)
        self.fig.canvas.draw_idle()

    # ---- selection / analysis ----
    def on_select(self, eclick, erelease):
        x0, x1 = sorted([eclick.xdata, erelease.xdata])
        y0, y1 = sorted([eclick.ydata, erelease.ydata])
        self.set_region(x0, x1, y0, y1)

    def set_region(self, x0, x1, y0, y1):
        self.region = (x0, x1, y0, y1)
        self.ref = 0
        self.analyze()

    def region_mask(self):
        x0, x1, y0, y1 = self.region
        P = self.P
        return (P[:, 0] >= x0) & (P[:, 0] <= x1) & (P[:, 1] >= y0) & (P[:, 1] <= y1)

    def analyze(self):
        idx = np.where(self.region_mask())[0]
        self.passes = split_passes(idx, self.gap)
        self.ref = min(self.ref, max(0, len(self.passes) - 1))
        self.update_radio()
        self.draw_region()
        if len(self.passes) == 0:
            self.say('No points in the selected region.')
        elif len(self.passes) == 1:
            self.say('Only 1 pass in region (%d pts) - nothing to align against. Try a larger gap/region.' % len(self.passes[0]))
        else:
            self.say('%d passes in region. Choose the reference pass, then press Align.' % len(self.passes))

    def update_radio(self):
        self.ax_radio.clear()
        self.ax_radio.set_title('Reference pass', fontsize=9)
        labels = ['#%d  idx %d-%d  (%d pts)' % (k, p[0], p[-1], len(p)) for k, p in enumerate(self.passes)]
        if not labels:
            self.ax_radio.axis('off')
            self.radio = None
            return
        self.radio = self.RadioButtons(self.ax_radio, labels, active=self.ref)
        for k, lab in enumerate(self.radio.labels):
            lab.set_fontsize(8)
            lab.set_color('tab:blue' if k == self.ref else PASS_COLORS[(k - 1) % len(PASS_COLORS)])
        self.radio.on_clicked(lambda lab: self.set_ref(int(lab.split()[0][1:])))

    def set_ref(self, k):
        self.ref = k
        self.update_radio()
        self.draw_region()

    def pass_color(self, k):
        if k == self.ref:
            return 'tab:blue'
        j = k if k < self.ref else k - 1
        return PASS_COLORS[j % len(PASS_COLORS)]

    def draw_region(self, title_extra=''):
        x0, x1, y0, y1 = self.region
        for ax in (self.ax_top, self.ax_side):
            ax.clear()
        for k, p in enumerate(self.passes):
            q = self.P[p[:: max(1, len(p) // 60000)]]
            col = self.pass_color(k)
            self.ax_top.scatter(q[:, 0], q[:, 1], s=0.5, c=col, alpha=0.5, label='#%d' % k, rasterized=True)
            self.ax_side.scatter(q[:, 0], q[:, 2], s=0.5, c=col, alpha=0.5, rasterized=True)
        self.ax_top.set_xlim(x0, x1)
        self.ax_top.set_ylim(y0, y1)
        self.ax_top.set_aspect('equal')
        self.ax_top.set_title('Region top-down (x-y)' + title_extra, fontsize=9)
        self.ax_side.set_xlim(x0, x1)
        self.ax_side.set_title('Region side view (x-z)', fontsize=9)
        if self.rect_patch is not None:
            self.rect_patch.remove()
        from matplotlib.patches import Rectangle
        self.rect_patch = Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='k', lw=1.2, ls='--')
        self.ax_map.add_patch(self.rect_patch)
        self.fig.canvas.draw_idle()

    # ---- actions ----
    def align(self):
        if self.region is None or len(self.passes) < 2:
            self.say('Select a region containing at least 2 passes first.')
            return
        taper = int(self.sl_taper.val / 100.0 * self.n)
        yaw_only = self.chk.get_status()[0]
        self.undo.append(self.P.copy())
        self.undo = self.undo[-8:]
        ref_idx = self.passes[self.ref]
        ref_pts = self.P[ref_idx].astype(np.float64)
        report = []
        for k, p in enumerate(list(self.passes)):
            if k == self.ref:
                continue
            src = self.P[p].astype(np.float64)
            before = nn_stats(src, ref_pts)
            res = icp_point_to_plane(src, ref_pts, yaw_only=yaw_only)
            if res is None:
                report.append('#%d: ICP failed (too little overlap)' % k)
                continue
            Rm, t, c, matches, resid = res
            apply_local(self.P, int(p[0]), int(p[-1]), taper, Rm, t, c)
            after = nn_stats(self.P[p].astype(np.float64), ref_pts)
            rv = np.degrees(R.from_matrix(Rm).as_rotvec())
            report.append('#%d: rot(deg)=[%.2f %.2f %.2f] shift(m)=[%.2f %.2f %.2f] median NN %.2f->%.2f m, <0.2m %.0f%%->%.0f%%'
                          % (k, rv[0], rv[1], rv[2], t[0], t[1], t[2], before[0], after[0], 100 * before[1], 100 * after[1]))
        self.refresh_map()
        self.analyze()
        self.say(' | '.join(report))

    def do_undo(self):
        if not self.undo:
            self.say('Nothing to undo.')
            return
        self.P = self.undo.pop()
        self.refresh_map()
        if self.region is not None:
            self.analyze()
        self.say('Undone.')

    def reset(self):
        self.undo.append(self.P.copy())
        self.P = self.P0.copy()
        self.refresh_map()
        if self.region is not None:
            self.analyze()
        self.say('Reset to the originally loaded map.')

    def save(self):
        save_pcd(self.out_path, self.P)
        self.say('Saved %d points to %s' % (self.n, self.out_path))

    def show(self):
        self.plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd')
    ap.add_argument('-o', '--out', help='output PCD (default: <input>_edited.pcd)')
    ap.add_argument('--gap-frac', type=float, default=0.02,
                    help='index gap (fraction of all points) that separates two passes (default 0.02)')
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.pcd)[0] + '_edited.pcd'
    if os.path.abspath(out) == os.path.abspath(a.pcd):
        sys.exit('Refusing to overwrite the input file.')
    print('Loading %s ...' % a.pcd)
    pts = load_pcd(a.pcd)
    print('%d points. Output will be %s' % (len(pts), out))
    AlignTool(pts, out, a.gap_frac).show()


if __name__ == '__main__':
    main()
