#!/usr/bin/env python3
"""Buggy Manager - a small desktop app to set the stops (home, A, B ...) of a map.   ./buggy app

  1. pick the map, click [Waypoint Set]: the map opens in RViz (./buggy view MAP)
  2. [A] / [B] (or any name + [Add stop]): then click RViz "2D Goal Pose" on the lane
     Home is OPTIONAL: by default it is simply where the buggy starts - set it in RViz with "2D Pose Estimate"
     before the mission starts (the old flow); the buggy returns there after the last stop. [Fixed home]
     stores a fixed place instead (RViz "2D Pose Estimate" click).
     the point is stored in <map>/mission.yaml at once and shown in RViz (/mission/markers)
  3. close the map; drive the stops with   ./buggy live|view MAP --mission   then   ./buggy mission start

The second tab, "Build map from bag", makes a PCD map from a recorded LiDAR bag (tools/mapping/build_map_from_bag.sh):
pick the bag folder, choose "Without GNSS" (LiDAR only) or "With GNSS" (the bag also has /chcnav/devpvt: adds the
map's latitude/longitude + heading and a drift check), press [Build map].

The ROS side is optional for the window: without it the stops can still be listed, reordered and deleted.
"""
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from buggy_pcd_lanelet_map import mission  # noqa: E402
from buggy_pcd_lanelet_map.paths import maps_dir, resolve_map  # noqa: E402

BUGGY = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'buggy'))
BG, PANEL, ACCENT, TEXT = '#27408B', '#1E3170', '#4169E1', '#ffffff'
SOURCES = {'initialpose': 'RViz "2D Pose Estimate"', 'goal_pose': 'RViz "2D Goal Pose"'}


BUILD_SCRIPT = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'tools', 'mapping', 'build_map_from_bag.sh'))
GNSS_TOPIC = '/chcnav/devpvt'


def bag_info(path):
    """What the bag folder holds, from its metadata.yaml: {ok, error, lidar, gnss, seconds, topics}."""
    import yaml
    info = {'ok': False, 'error': '', 'lidar': 0, 'gnss': 0, 'seconds': 0.0, 'topics': {}}
    meta = os.path.join(os.path.expanduser(path or ''), 'metadata.yaml')
    if not path or not os.path.isfile(meta):
        info['error'] = 'not a rosbag2 folder (no metadata.yaml)'
        return info
    try:
        b = yaml.safe_load(open(meta))['rosbag2_bagfile_information']
        info['topics'] = {t['topic_metadata']['name']: t['message_count'] for t in b['topics_with_message_count']}
        info['seconds'] = b['duration']['nanoseconds'] * 1e-9
    except Exception as e:
        info['error'] = f'cannot read metadata.yaml: {e}'
        return info
    info['lidar'] = info['topics'].get('/velodyne_points', 0)
    info['gnss'] = info['topics'].get(GNSS_TOPIC, 0)
    info['ok'] = info['lidar'] > 0
    if not info['ok']:
        info['error'] = 'the bag has no /velodyne_points'
    return info


def build_command(bag, out, gnss, rviz, max_scans=0, closure=True):
    cmd = ['bash', BUILD_SCRIPT, os.path.abspath(os.path.expanduser(bag))]
    if out:
        cmd.append(os.path.abspath(os.path.expanduser(out)))
    if gnss:
        cmd.append('--gnss')
    if not closure:
        cmd.append('--no-closure')
    if not rviz:
        cmd.append('--no-rviz')
    if max_scans:
        cmd += ['--max-scans', str(int(max_scans))]
    return cmd


class Editor:
    """The mission file of one map + the click that is being waited for (no GUI, no ROS: testable)."""

    def __init__(self, map_name):
        self.map_name = map_name
        self.path = mission.mission_path(resolve_map(map_name))
        self.data = mission.read_lenient(self.path)
        self.armed = None                   # (name, source, wait_sec or None)
        self.message = ''

    def stops(self):
        return self.data['stops']

    def default_source(self, name):
        return 'initialpose' if name.lower() == 'home' else 'goal_pose'

    def next_free_name(self):
        names = {s.get('name') for s in self.stops()}
        for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
            if c not in names:
                return c
        return f'stop{len(names) + 1}'

    def arm(self, name, source=None, wait_sec=None):
        name = name.strip()
        if not name or any(ch in name for ch in ':{},#\'"'):
            self.message = 'give the stop a simple name (letters, digits, - _)'
            return False
        self.armed = (name, source or self.default_source(name), wait_sec)
        self.message = f'click {SOURCES[self.armed[1]]} in RViz for "{name}" ...'
        return True

    def cancel(self):
        self.armed = None
        self.message = 'cancelled'

    def on_click(self, source, x, y):
        """A click arrived from RViz. True when it was stored."""
        if not self.armed or self.armed[1] != source:
            return False
        name, _, wait = self.armed
        mission.set_stop(self.data, name, x, y, wait)
        self.save()
        self.armed = None
        self.message = f'stop "{name}" set at ({x:.2f}, {y:.2f})'
        return True

    def save(self):
        mission.save(self.path, self.data)

    def delete(self, index):
        mission.delete_stop(self.data, index)
        self.save()

    def move(self, index, delta):
        j = mission.move_stop(self.data, index, delta)
        self.save()
        return j

    def set_wait(self, index, wait):
        self.stops()[index]['wait_sec'] = float(wait)
        self.save()

    def set_loop(self, loop):
        self.data['loop'] = bool(loop)
        self.save()

    def set_start_as_home(self, on):
        self.data['start_as_home'] = bool(on)
        self.save()

    def problems(self):
        if not self.stops():
            return ['no stops yet']
        if self.data.get('start_as_home') and any(s.get('name') == 'home' for s in self.stops()):
            return ['a fixed "home" stop exists: it is used, the RViz start pose is not added'] + mission.warnings(self.data)
        out = mission.validate(self.data)
        return out + mission.warnings(self.data)


class RosLink(threading.Thread):
    """/initialpose and /goal_pose clicks in, /mission/markers out. Optional: the app works without ROS."""

    def __init__(self, on_click, get_stops):
        super().__init__(daemon=True)
        self.on_click, self.get_stops, self.error = on_click, get_stops, ''

    def run(self):
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
            from rclpy.node import Node
            from visualization_msgs.msg import Marker, MarkerArray
            rclpy.init()
            node = Node('buggy_manager_app')
            node.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda m: self.on_click(
                'initialpose', m.pose.pose.position.x, m.pose.pose.position.y), 10)
            node.create_subscription(PoseStamped, '/goal_pose', lambda m: self.on_click(
                'goal_pose', m.pose.position.x, m.pose.position.y), 10)
            pub = node.create_publisher(MarkerArray, '/mission/markers', 1)

            def publish():
                arr = MarkerArray()
                clear = Marker()
                clear.action = Marker.DELETEALL
                arr.markers.append(clear)
                for i, s in enumerate(self.get_stops()):
                    m = Marker()
                    m.header.frame_id = 'map'
                    m.ns, m.id, m.type, m.action = 'mission_stop', i, Marker.CYLINDER, Marker.ADD
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(s['x']), float(s['y']), 0.1
                    m.pose.orientation.w = 1.0
                    m.scale.x = m.scale.y = 1.6
                    m.scale.z = 0.2
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.1, 0.85
                    arr.markers.append(m)
                    t = Marker()
                    t.header.frame_id = 'map'
                    t.ns, t.id, t.type, t.action = 'mission_label', i, Marker.TEXT_VIEW_FACING, Marker.ADD
                    t.pose.position.x, t.pose.position.y, t.pose.position.z = float(s['x']), float(s['y']), 2.0
                    t.pose.orientation.w = 1.0
                    t.scale.z = 0.9
                    t.color.r = t.color.g = t.color.b = t.color.a = 1.0
                    t.text = f'{i + 1}: {s["name"]}'
                    arr.markers.append(t)
                pub.publish(arr)

            node.create_timer(1.0, publish)
            rclpy.spin(node)
        except Exception as e:                       # no ROS in this shell: the window still works
            self.error = f'{type(e).__name__}: {e}'


class BuildProcess:
    """tools/mapping/build_map_from_bag.sh as a child process; its output lines go to a queue."""

    def __init__(self, cmd, lines):
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     start_new_session=True, cwd=os.path.dirname(BUILD_SCRIPT))
        threading.Thread(target=self._read, args=(lines,), daemon=True).start()

    def _read(self, lines):
        for line in self.proc.stdout:
            lines.put(line.rstrip())
        lines.put(f'[build finished, exit code {self.proc.wait()}]')

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.alive():
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass


class MapProcess:
    """`./buggy view MAP` as a child process (own process group, stopped with Ctrl+C semantics)."""

    def __init__(self, map_name, lines):
        self.proc = subprocess.Popen(
            ['bash', BUGGY, 'view', map_name, '--no-avoid'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        threading.Thread(target=self._read, args=(lines,), daemon=True).start()

    def _read(self, lines):
        for line in self.proc.stdout:
            lines.put(line.rstrip())
        lines.put(f'[map closed, exit code {self.proc.wait()}]')

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if not self.alive():
            return
        for sig, wait in ((signal.SIGINT, 6), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            t = time.time() + wait
            while time.time() < t and self.alive():
                time.sleep(0.2)
            if not self.alive():
                return


def run_gui():
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title('Buggy Manager')
    root.geometry('980x720')
    root.configure(bg=BG)
    st = ttk.Style(root)
    st.theme_use('clam')
    st.configure('.', background=BG, foreground=TEXT, fieldbackground=PANEL)
    st.configure('TLabel', background=BG, foreground=TEXT)
    st.configure('TButton', background=ACCENT, foreground=TEXT, padding=6)
    st.map('TButton', background=[('active', '#5A7FEA'), ('disabled', '#3A4F8F')])
    st.configure('Big.TButton', font=('TkDefaultFont', 13, 'bold'), padding=12)
    st.configure('TCheckbutton', background=BG, foreground=TEXT)
    st.configure('TRadiobutton', background=BG, foreground=TEXT)
    st.configure('TNotebook', background=BG)
    st.configure('TNotebook.Tab', background=PANEL, foreground=TEXT, padding=(14, 6))
    st.map('TNotebook.Tab', background=[('selected', ACCENT)])
    st.configure('TFrame', background=BG)
    st.configure('Treeview', background=PANEL, fieldbackground=PANEL, foreground=TEXT, rowheight=26)
    st.configure('Treeview.Heading', background=ACCENT, foreground=TEXT)
    st.map('Treeview', background=[('selected', ACCENT)])

    maps = sorted(d for d in os.listdir(maps_dir()) if os.path.isdir(os.path.join(maps_dir(), d))) \
        if os.path.isdir(maps_dir()) else []
    state = {'editor': None, 'proc': None}
    loading = {'on': False}
    closing = {'on': False}
    clicks, lines = queue.Queue(), queue.Queue()

    nb = ttk.Notebook(root)
    nb.pack(fill='both', expand=True)
    tab_wp = ttk.Frame(nb)
    tab_map = ttk.Frame(nb)
    nb.add(tab_wp, text='  Waypoints  ')
    nb.add(tab_map, text='  Build map from bag  ')

    top = ttk.Frame(tab_wp)
    top.pack(fill='x', padx=14, pady=10)
    ttk.Label(top, text='Map').pack(side='left')
    map_var = tk.StringVar(value=maps[0] if maps else '')
    map_box = ttk.Combobox(top, textvariable=map_var, values=maps, state='readonly', width=24)
    map_box.pack(side='left', padx=8)
    go = ttk.Button(top, text='Waypoint Set', style='Big.TButton')
    go.pack(side='left', padx=12)
    close_btn = ttk.Button(top, text='Close map')
    close_btn.pack(side='left')
    status = tk.StringVar(value='Pick a map and click "Waypoint Set": the map opens in RViz.')
    ttk.Label(tab_wp, textvariable=status, wraplength=940).pack(fill='x', padx=14)

    mid = ttk.Frame(tab_wp)
    mid.pack(fill='both', expand=True, padx=14, pady=8)
    left = ttk.Frame(mid)
    left.pack(side='left', fill='both', expand=True)
    tree = ttk.Treeview(left, columns=('n', 'name', 'x', 'y', 'wait'), show='headings', height=9)
    for c, t, w in (('n', '#', 40), ('name', 'Stop', 160), ('x', 'x (m)', 110), ('y', 'y (m)', 110), ('wait', 'Wait (s)', 90)):
        tree.heading(c, text=t)
        tree.column(c, width=w, anchor='center')
    tree.pack(fill='both', expand=True)
    loop_var = tk.BooleanVar(value=True)
    ttk.Checkbutton(left, text='Loop: after the last stop go to the first again', variable=loop_var).pack(anchor='w', pady=(4, 0))
    home_var = tk.BooleanVar(value=True)
    ttk.Checkbutton(left, text='Home = where the buggy starts (set it in RViz with 2D Pose Estimate before "mission start"); '
                    'it is the last stop', variable=home_var).pack(anchor='w', pady=(0, 4))

    right = ttk.Frame(mid)
    right.pack(side='left', fill='y', padx=(14, 0))
    ttk.Label(right, text='Set a stop, then click in RViz').pack(anchor='w')
    quick = ttk.Frame(right)
    quick.pack(fill='x', pady=4)
    btn_a = ttk.Button(quick, text='A\n(2D Goal Pose)')
    btn_b = ttk.Button(quick, text='B\n(2D Goal Pose)')
    btn_home = ttk.Button(quick, text='Fixed home (optional)\n(2D Pose Estimate)')
    for b in (btn_a, btn_b, btn_home):
        b.pack(fill='x', pady=2)
    ttk.Label(right, text='Other stop: name / click type / wait (s)').pack(anchor='w', pady=(10, 0))
    name_var, src_var, wait_var = tk.StringVar(), tk.StringVar(value='goal_pose'), tk.StringVar(value='')
    ttk.Entry(right, textvariable=name_var).pack(fill='x', pady=2)
    ttk.Combobox(right, textvariable=src_var, values=list(SOURCES), state='readonly').pack(fill='x', pady=2)
    ttk.Entry(right, textvariable=wait_var).pack(fill='x', pady=2)
    btn_add = ttk.Button(right, text='Add / replace stop')
    btn_add.pack(fill='x', pady=2)
    btn_cancel = ttk.Button(right, text='Cancel waiting')
    btn_cancel.pack(fill='x', pady=(2, 10))
    btn_up, btn_down = ttk.Button(right, text='Move up'), ttk.Button(right, text='Move down')
    btn_wait, btn_del = ttk.Button(right, text='Set wait of selected'), ttk.Button(right, text='Delete selected')
    for b in (btn_up, btn_down, btn_wait, btn_del):
        b.pack(fill='x', pady=2)

    log = tk.Text(tab_wp, height=9, bg='#101A40', fg='#cfd8ff', insertbackground='white')
    log.pack(fill='x', padx=14, pady=(0, 10))


    # ---- tab 2: build a PCD map from a LiDAR bag -------------------------------------------------------
    from tkinter import filedialog
    blines = queue.Queue()
    bstate = {'proc': None, 'info': None, 'auto_out': '', 'out': ''}
    bag_var, out_var, scans_var = tk.StringVar(), tk.StringVar(), tk.StringVar()
    gnss_var, rviz_var = tk.StringVar(value='without'), tk.BooleanVar(value=False)
    closure_var = tk.BooleanVar(value=True)
    binfo_var = tk.StringVar(value='Choose the bag folder (a rosbag2 folder with /velodyne_points).')
    bstatus = tk.StringVar(value='')

    r1 = ttk.Frame(tab_map)
    r1.pack(fill='x', padx=14, pady=(12, 4))
    ttk.Label(r1, text='Bag folder', width=14).pack(side='left')
    bag_entry = ttk.Entry(r1, textvariable=bag_var)
    bag_entry.pack(side='left', fill='x', expand=True, padx=6)
    ttk.Button(r1, text='Browse...', command=lambda: browse_bag()).pack(side='left')
    ttk.Label(tab_map, textvariable=binfo_var, wraplength=940).pack(fill='x', padx=14, pady=2)
    r2 = ttk.Frame(tab_map)
    r2.pack(fill='x', padx=14, pady=4)
    ttk.Label(r2, text='Output folder', width=14).pack(side='left')
    out_entry = ttk.Entry(r2, textvariable=out_var)
    out_entry.pack(side='left', fill='x', expand=True, padx=6)
    r3 = ttk.Frame(tab_map)
    r3.pack(fill='x', padx=14, pady=4)
    ttk.Label(r3, text='GNSS', width=14).pack(side='left')
    rb_without = ttk.Radiobutton(r3, text='Without GNSS (LiDAR only)', variable=gnss_var, value='without')
    rb_with = ttk.Radiobutton(r3, text='With GNSS (the bag has /chcnav/devpvt): map latitude / longitude / heading + drift check',
                              variable=gnss_var, value='with')
    rb_without.pack(side='left', padx=6)
    rb_with.pack(side='left', padx=6)
    r4 = ttk.Frame(tab_map)
    r4.pack(fill='x', padx=14, pady=4)
    ttk.Label(r4, text='Options', width=14).pack(side='left')
    ttk.Checkbutton(r4, text='Open RViz when done (the build keeps running until you stop it)', variable=rviz_var).pack(side='left', padx=6)
    ttk.Checkbutton(r4, text='Loop closure (the drive returns to its start)', variable=closure_var).pack(side='left', padx=6)
    ttk.Label(r4, text='   first N scans only (test):').pack(side='left')
    ttk.Entry(r4, textvariable=scans_var, width=7).pack(side='left', padx=6)
    r5 = ttk.Frame(tab_map)
    r5.pack(fill='x', padx=14, pady=8)
    build_btn = ttk.Button(r5, text='Build map', style='Big.TButton')
    build_btn.pack(side='left')
    stop_build_btn = ttk.Button(r5, text='Stop')
    stop_build_btn.pack(side='left', padx=8)
    folder_btn = ttk.Button(r5, text='Open output folder')
    folder_btn.pack(side='left')
    ttk.Label(tab_map, textvariable=bstatus, wraplength=940).pack(fill='x', padx=14)
    blog = tk.Text(tab_map, height=16, bg='#101A40', fg='#cfd8ff', insertbackground='white')
    blog.pack(fill='both', expand=True, padx=14, pady=(4, 10))

    def refresh_bag(*_):
        info = bag_info(bag_var.get().strip())
        bstate['info'] = info
        if not info['ok']:
            binfo_var.set(info['error'] if bag_var.get().strip() else 'Choose the bag folder (a rosbag2 folder with /velodyne_points).')
            rb_with.state(['disabled'] if not info['gnss'] else ['!disabled'])
            return
        binfo_var.set(f"{info['seconds']:.0f} s, {info['lidar']} LiDAR scans, GNSS: "
                      + (f"yes ({info['gnss']} messages on {GNSS_TOPIC})" if info['gnss'] else f'no ({GNSS_TOPIC} is not in the bag)'))
        rb_with.state(['!disabled'] if info['gnss'] else ['disabled'])
        gnss_var.set('with' if info['gnss'] else 'without')
        auto = os.path.join(os.path.expanduser('~'), 'maps', os.path.basename(os.path.normpath(bag_var.get().strip())))
        if out_var.get().strip() in ('', bstate['auto_out']):          # not changed by the user: follow the bag name
            out_var.set(auto)
        bstate['auto_out'] = auto

    def browse_bag():
        d = filedialog.askdirectory(initialdir=os.path.expanduser('~/bags'), title='Bag folder (rosbag2)')
        if d:
            bag_var.set(d)
            refresh_bag()

    bag_entry.bind('<FocusOut>', refresh_bag)
    bag_entry.bind('<Return>', refresh_bag)

    def start_build():
        if bstate['proc'] and bstate['proc'].alive():
            bstatus.set('A build is running already.')
            return
        refresh_bag()
        info = bstate['info']
        if not info or not info['ok']:
            bstatus.set('Cannot build: ' + (info['error'] if info else 'choose a bag folder first'))
            return
        gnss = gnss_var.get() == 'with'
        if gnss and not info['gnss']:
            bstatus.set('The bag has no GNSS: choose "Without GNSS".')
            return
        try:
            n = int(scans_var.get()) if scans_var.get().strip() else 0
        except ValueError:
            bstatus.set('"first N scans" must be a whole number (or empty = all).')
            return
        bstate['out'] = out_var.get().strip() or os.path.join(os.path.expanduser('~'), 'maps', os.path.basename(os.path.normpath(bag_var.get().strip())))
        blog.delete('1.0', 'end')
        cmd = build_command(bag_var.get().strip(), bstate['out'], gnss, rviz_var.get(), n, closure_var.get())
        bstatus.set('Building ' + ('with GNSS' if gnss else 'without GNSS') + ' ... (a long bag takes minutes; the log shows the steps)')
        build_btn.state(['disabled'])
        bstate['proc'] = BuildProcess(cmd, blines)

    def stop_build():
        if bstate['proc'] and bstate['proc'].alive():
            bstate['proc'].stop()
            bstatus.set('Stopping ...')

    def open_folder():
        d = bstate['out'] or out_var.get().strip()
        if d and os.path.isdir(d):
            subprocess.Popen(['xdg-open', d], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            bstatus.set('The output folder does not exist yet.')

    build_btn.configure(command=start_build)
    stop_build_btn.configure(command=stop_build)
    folder_btn.configure(command=open_folder)

    def poll_build():
        try:
            while True:
                line = blines.get_nowait()
                blog.insert('end', line + '\n')
                blog.see('end')
                if line.startswith('[build finished'):
                    build_btn.state(['!disabled'])
                    ok = line.endswith('code 0]')
                    bstatus.set(('Done. ' if ok else 'The build failed or was stopped (see the log). ')
                                + (f"Map: {bstate['out']}/*_map_flat.pcd. Next: draw the lanes in Vector Map Builder, then "
                                   "./buggy add-map NAME FILE.osm --pcd FILE.pcd" if ok else ''))
        except queue.Empty:
            pass
        root.after(200, poll_build)

    ros = RosLink(lambda src, x, y: clicks.put((src, x, y)), lambda: list(state['editor'].stops()) if state['editor'] else [])

    def ed():
        return state['editor']

    def refresh(select=None):
        tree.delete(*tree.get_children())
        if not ed():
            return
        default = ed().data.get('wait_sec', mission.DEFAULT_WAIT)
        for i, s in enumerate(ed().stops()):
            tree.insert('', 'end', iid=str(i), values=(i + 1, s['name'], f'{s["x"]:.2f}', f'{s["y"]:.2f}',
                                                      f'{s.get("wait_sec", default):g}'))
        if select is not None and tree.exists(str(select)):
            tree.selection_set(str(select))
        loading['on'] = True                          # showing the file's values must not write the file
        loop_var.set(bool(ed().data.get('loop', True)))
        home_var.set(bool(ed().data.get('start_as_home', False)))
        loading['on'] = False

    def say(text):
        status.set(text)

    def selected():
        sel = tree.selection()
        return int(sel[0]) if sel else None

    def open_map():
        if not map_var.get():
            say('No map folders in data/maps.')
            return
        if state['proc'] and state['proc'].alive():
            say('A map is open already: "Close map" first.')
            return
        state['editor'] = Editor(map_var.get())
        refresh()
        say(f'Opening {map_var.get()} in RViz ... (output below). Set the vehicle pose with 2D Pose Estimate if you like.')
        state['proc'] = MapProcess(map_var.get(), lines)
        if not ros.is_alive():
            ros.start()

    def close_map():
        if state['proc']:
            say('Closing the map ...')
            root.update_idletasks()
            closing['on'] = True
            state['proc'].stop()
            state['proc'] = None
            closing['on'] = False
            say('Map closed. Drive the stops with:  ./buggy live|view MAP --mission   then   ./buggy mission start')

    def arm(name, source=None):
        if not ed():
            say('Click "Waypoint Set" first.')
            return
        w = wait_var.get().strip()
        try:
            wait = float(w) if w else None
        except ValueError:
            say('Wait must be a number of seconds (or empty = the default).')
            return
        if ed().arm(name, source, wait):
            if not state['proc'] or not state['proc'].alive():
                say('The map is not open: click "Waypoint Set", then set the stop.')
                ed().cancel()
                return
        say(ed().message)

    def poll():
        try:
            while True:
                src, x, y = clicks.get_nowait()
                if ed() and ed().on_click(src, x, y):
                    refresh()
                    say(ed().message)
        except queue.Empty:
            pass
        try:
            while True:
                line = lines.get_nowait()
                log.insert('end', line + '\n')
                if line.startswith('[map closed') and not line.endswith('code 0]') and not closing['on']:
                    reason = [x for x in log.get('1.0', 'end').splitlines()[-12:-1] if x.strip()]
                    say('The map did not start or stopped (' + line.strip('[]') + '). Last lines: '
                        + ' | '.join(reason[-3:])[:420])
                    state['proc'] = None
                log.see('end')
                if int(log.index('end-1c').split('.')[0]) > 400:
                    log.delete('1.0', '100.0')
        except queue.Empty:
            pass
        if ros.error and not getattr(poll, 'warned', False):
            poll.warned = True
            say('ROS is not available in this shell (' + ros.error + '). Start the app with ./buggy app.')
        root.after(150, poll)

    def on_wait():
        i = selected()
        if i is None or not ed():
            return
        try:
            ed().set_wait(i, float(wait_var.get()))
        except ValueError:
            say('Type the wait in seconds in the "wait" box first.')
            return
        refresh(i)

    go.configure(command=open_map)
    close_btn.configure(command=close_map)
    btn_home.configure(command=lambda: arm('home', 'initialpose'))
    btn_a.configure(command=lambda: arm('A', 'goal_pose'))
    btn_b.configure(command=lambda: arm('B', 'goal_pose'))
    btn_add.configure(command=lambda: arm(name_var.get() or (ed().next_free_name() if ed() else ''), src_var.get()))
    btn_cancel.configure(command=lambda: (ed() and ed().cancel(), say('Cancelled.')))
    btn_wait.configure(command=on_wait)
    btn_del.configure(command=lambda: selected() is not None and ed() and (ed().delete(selected()), refresh()))
    btn_up.configure(command=lambda: selected() is not None and ed() and refresh(ed().move(selected(), -1)))
    btn_down.configure(command=lambda: selected() is not None and ed() and refresh(ed().move(selected(), 1)))
    home_var.trace_add('write', lambda *_: ed() and not loading['on'] and ed().set_start_as_home(home_var.get()))
    loop_var.trace_add('write', lambda *_: ed() and not loading['on'] and ed().set_loop(loop_var.get()))
    map_box.bind('<<ComboboxSelected>>', lambda e: (state.update(editor=Editor(map_var.get())), refresh())
                 if not (state['proc'] and state['proc'].alive()) else None)
    state['editor'] = Editor(map_var.get()) if map_var.get() else None
    refresh()

    def on_close():
        if bstate['proc'] and bstate['proc'].alive():
            if not messagebox.askyesno('Buggy Manager', 'A map build is running. Stop it and quit?'):
                return
            bstate['proc'].stop()
        if state['proc'] and state['proc'].alive():
            if not messagebox.askyesno('Buggy Manager', 'The map is still open. Close it and quit?'):
                return
            state['proc'].stop()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    poll()
    poll_build()
    root.mainloop()


if __name__ == '__main__':
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('no display: run ./buggy app on the desktop')
    run_gui()
