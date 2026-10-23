#!/usr/bin/env python3
"""End-to-end test of the C++ uart_writer against a pseudo-terminal (no hardware needed):
commands in -> bytes out, watchdog -> STOP, SIGINT -> final STOP.   python3 test_uart_pty.py [pkg]"""
import os, pty, select, signal, subprocess, sys, time, termios, tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
pkg = sys.argv[1] if len(sys.argv) > 1 else 'buggy_core'
master, slave = pty.openpty(); tty.setraw(master)
port = os.ttyname(slave)
proc = subprocess.Popen(['ros2', 'run', pkg, 'uart_writer', '--ros-args', '-p', f'port:={port}', '-p', 'loopback:=false'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
rclpy.init(); n = Node('uart_pty_test')
ps = n.create_publisher(Float32, '/steering_angle', 1); pr = n.create_publisher(Bool, '/final_red_detected', 1); py_ = n.create_publisher(Bool, '/final_yellow_detected', 1)
buf = bytearray()
def drain(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        if select.select([master], [], [], 0)[0]: buf.extend(os.read(master, 4096))
def frames():
    out = []
    while len(buf) >= 6:
        i = buf.find(b'(')
        if i < 0: buf.clear(); break
        del buf[:i]
        if len(buf) < 6: break
        out.append(bytes(buf[:6])); del buf[:6]
    return out
def decode(f): return dict(ok=(f[0] == 40 and f[5] == 41), yellow=f[1], red=f[2], angle=((f[3] << 8 | f[4]) * 0.01) - 30.0)
def command(red, yellow, steer, sec):
    end = time.time() + sec
    while time.time() < end:
        pr.publish(Bool(data=red)); py_.publish(Bool(data=yellow)); ps.publish(Float32(data=steer)); drain(0.05)
ok = True
def check(name, cond, extra=''):
    global ok; ok &= bool(cond); print(('PASS' if cond else 'FAIL'), name, extra)
time.sleep(1.5); drain(0.5); buf.clear()
command(False, True, 7.5, 1.0); fr = [decode(f) for f in frames()]
check('GO / 2 km/h preset / steer 7.5 deg frames arrive', len(fr) >= 6 and all(f['ok'] for f in fr[-5:]) and fr[-1] == dict(ok=True, yellow=1, red=0, angle=fr[-1]['angle']) and abs(fr[-1]['angle'] - 7.5) < 0.011, f'{len(fr)} frames, last {fr[-1] if fr else None}')
rate = len(fr) / 1.0; check('about 10 frames/s', 7 <= rate <= 13, f'{rate:.1f}/s')
command(True, False, -12.0, 0.6); fr = [decode(f) for f in frames()]
check('RED / 4 km/h / steer -12 deg', fr and fr[-1]['red'] == 1 and fr[-1]['yellow'] == 0 and abs(fr[-1]['angle'] + 12.0) < 0.011, str(fr[-1] if fr else None))
buf.clear(); drain(1.2); fr = [decode(f) for f in frames()]          # no commands: watchdog (0.5 s)
check('watchdog: stale commands -> STOP (red, angle 0)', fr and fr[-1]['red'] == 1 and abs(fr[-1]['angle']) < 0.011 and fr[-1]['yellow'] == 0, str(fr[-1] if fr else None))
command(False, False, 3.0, 0.8); fr = [decode(f) for f in frames()]
check('commands fresh again -> GO', fr and fr[-1]['red'] == 0, str(fr[-1] if fr else None))
buf.clear(); os.killpg(proc.pid, signal.SIGINT); t0 = time.time()
while proc.poll() is None and time.time() - t0 < 5: drain(0.05)
drain(0.2); fr = [decode(f) for f in frames()]
check('SIGINT: exits and the LAST frames are STOP', proc.poll() is not None and len(fr) >= 1 and fr[-1]['red'] == 1 and abs(fr[-1]['angle']) < 0.011, f'{len(fr)} frames, exit {proc.poll()}')
print('ALL PASS' if ok else 'SOME FAILED'); sys.exit(0 if ok else 1)
