#!/usr/bin/env python3
"""Checks the buggy hardware is reachable before the live launch.
Exit 0 = all good, 1 = something to fix (details printed)."""
import glob
import os
import socket
import subprocess
import sys

ok = True


def report(good, what, fix=''):
    global ok
    ok &= good
    print(f"  [{'OK' if good else 'FAIL'}] {what}" + ('' if good else f'\n         -> {fix}'))


print('Buggy live preflight')

# ECU serial
port = os.environ.get('FEEDBACK_PORT', '/dev/ttyUSB0')
exists = os.path.exists(port)
report(exists, f'ECU serial {port} present',
       'connect the ECU USB-serial cable; `ls /dev/ttyUSB*` (also check `dmesg | tail`)')
if exists:
    report(os.access(port, os.R_OK | os.W_OK), f'{port} readable/writable',
           'sudo usermod -aG dialout $USER  (then log out and in)')

# LiDAR ethernet
nic = 'enp2s0'
state = ''
try:
    state = open(f'/sys/class/net/{nic}/operstate').read().strip()
except OSError:
    pass
report(state == 'up', f'{nic} link up (LiDAR ethernet)',
       'plug in the LiDAR cable / power the VLP-16. VLP-16 defaults to 192.168.1.201 and '
       f'sends to 192.168.1.77, so give {nic} a static 192.168.1.77/24: '
       f'nmcli con add type ethernet ifname {nic} con-name vlp16 ipv4.method manual '
       'ipv4.addresses 192.168.1.77/24 && nmcli con up vlp16')
if state == 'up':
    addrs = subprocess.run(['ip', '-4', '-br', 'addr', 'show', nic], capture_output=True,
                           text=True).stdout
    report('192.168.1.' in addrs or '/' in addrs.split(None, 2)[-1] if addrs else False,
           f'{nic} has an IPv4 address ({addrs.strip().split(None, 2)[-1] if addrs else "none"})',
           f'set 192.168.1.77/24 on {nic} (see above)')
    # any UDP packet on 2368 within 3 s
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(3.0)
    try:
        s.bind(('', 2368))
        data, src = s.recvfrom(2000)
        report(True, f'VLP-16 packets on UDP 2368 from {src[0]} ({len(data)} B)')
    except OSError as e:
        report(False, f'VLP-16 packets on UDP 2368 ({e})',
               'LiDAR powered and spinning? destination IP must be this laptop; '
               'check with `sudo tcpdump -i enp2s0 -c 3 udp port 2368`')
    finally:
        s.close()

sys.exit(0 if ok else 1)
