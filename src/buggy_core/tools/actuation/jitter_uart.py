#!/usr/bin/env python3
"""Frame-interval jitter of a uart_writer (python|cpp) on a pseudo-terminal, optionally under CPU load.
   python3 jitter_uart.py <package> <seconds> <busy_processes>"""
import os, pty, select, signal, subprocess, sys, time, tty, statistics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
pkg, secs, busy = sys.argv[1], float(sys.argv[2]), int(sys.argv[3])
master, slave = pty.openpty(); tty.setraw(master)
hogs = [subprocess.Popen([sys.executable, '-c', 'while True: pass']) for _ in range(busy)]
proc = subprocess.Popen(['ros2', 'run', pkg, 'uart_writer', '--ros-args', '-p', f'port:={os.ttyname(slave)}', '-p', 'loopback:=false'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
rclpy.init(); n = Node('jit'); pr = n.create_publisher(Bool, '/final_red_detected', 1); py_ = n.create_publisher(Bool, '/final_yellow_detected', 1); ps = n.create_publisher(Float32, '/steering_angle', 1)
buf = bytearray(); stamps = []
t0 = time.time()
while time.time() - t0 < secs + 2:
    pr.publish(Bool(data=False)); py_.publish(Bool(data=False)); ps.publish(Float32(data=1.0))
    rclpy.spin_once(n, timeout_sec=0.02)
    if select.select([master], [], [], 0.02)[0]:
        data = os.read(master, 4096); now = time.time(); buf.extend(data)
        while len(buf) >= 6:
            i = buf.find(b'(')
            if i < 0: buf.clear(); break
            del buf[:i]
            if len(buf) < 6: break
            del buf[:6]
            if now - t0 > 2: stamps.append(now)
os.killpg(proc.pid, signal.SIGINT)
for h in hogs: h.kill()
iv = [b - a for a, b in zip(stamps, stamps[1:])]
iv.sort()
print(f'{pkg:24s} busy={busy:2d}  frames {len(stamps):3d}  interval ms: mean {1000*statistics.mean(iv):6.1f}  p99 {1000*iv[int(0.99*len(iv))]:6.1f}  max {1000*iv[-1]:6.1f}   (ideal 100)')
