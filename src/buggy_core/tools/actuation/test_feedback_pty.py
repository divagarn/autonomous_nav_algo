#!/usr/bin/env python3
"""End-to-end test of uart_bridge against a pseudo-terminal (no hardware): the test plays the ECU,
the bridge must publish /wheel_uart with the decoded steering / speed, survive garbage and a packet
split across writes, and keep running when the adapter disappears.
   python3 test_feedback_pty.py [buggy_core|buggy_uart]"""
import os, pty, signal, subprocess, sys, time, tty
import rclpy
from rclpy.node import Node
from ground_segmentation_msgs.msg import SteerSpeed
pkg = sys.argv[1] if len(sys.argv) > 1 else 'buggy_core'
master, slave = pty.openpty(); tty.setraw(master)
port = os.ttyname(slave)
proc = subprocess.Popen(['ros2', 'run', pkg, 'uart_bridge', '--ros-args', '-p', f'port:={port}'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
rclpy.init(); n = Node('feedback_pty_test'); got = []
n.create_subscription(SteerSpeed, '/wheel_uart', lambda m: got.append((round(m.steering, 2), round(m.speed, 1))), 50)
def spin(sec):
    end = time.time() + sec
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
def pkt(deg, speed): return bytes([40]) + int((deg + 30) / 0.01).to_bytes(2, 'big', signed=True) + bytes([speed, 0, 41])
ok = True
def check(name, cond, extra=''):
    global ok; ok &= bool(cond); print(('PASS' if cond else 'FAIL'), name, extra)
spin(2.0)
for deg, sp in ((0.0, 4), (-12.5, 4), (7.25, 2), (19.0, 0)):
    os.write(master, pkt(deg, sp)); spin(0.15)
check('four packets decoded', got == [(0.0, 4.0), (-12.5, 4.0), (7.25, 2.0), (19.0, 0.0)], str(got))
got.clear(); os.write(master, b'\x01\x02\xff' + pkt(3.0, 4)); spin(0.2)
check('garbage before a packet is skipped', got == [(3.0, 4.0)], str(got))
got.clear(); p = pkt(-5.0, 2); os.write(master, p[:3]); spin(0.2); os.write(master, p[3:]); spin(0.2)
check('packet split across writes', got == [(-5.0, 2.0)], str(got))
got.clear(); os.write(master, pkt(35.0, 4)); spin(0.2)
check('steering clamped to +20 deg', got == [(20.0, 4.0)], str(got))
got.clear()
for i in range(100): os.write(master, pkt(1.0 * (i % 10), 4)); spin(0.02)
check('100 packets at 50 Hz', len(got) == 100, f'{len(got)} received')
os.close(master); spin(1.5)
check('survives the adapter disappearing', proc.poll() is None)
os.killpg(proc.pid, signal.SIGINT); t0 = time.time()
while proc.poll() is None and time.time() - t0 < 5: time.sleep(0.05)
print('ALL PASS' if ok else 'SOME FAILED'); sys.exit(0 if ok else 1)
