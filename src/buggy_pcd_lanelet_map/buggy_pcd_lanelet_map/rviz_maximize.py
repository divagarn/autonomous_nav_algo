#!/usr/bin/env python3
"""Maximise the RViz window once it exists (the window stays resizable: restore with the title bar).

RViz2 has no command-line option for this and restores whatever size is saved in its .rviz file. This
sends the standard EWMH request (_NET_WM_STATE_MAXIMIZED_VERT/HORZ) to the window manager through libX11
(ctypes - no extra packages, no ROS). It works on X11 and for XWayland windows (RViz uses XWayland here).

  rviz_maximize.py [window title text = RViz] [max wait seconds = 40]
Exits quietly if there is no X display / libX11 / xwininfo - RViz then just keeps its saved size.
"""
import ctypes
import ctypes.util
import re
import subprocess
import sys
import time


def find_window(text):
    try:
        out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True,
                             timeout=3).stdout
    except Exception:
        return None
    m = re.search(r'(0x[0-9a-f]+) "[^"]*' + re.escape(text) + r'"', out)
    return int(m.group(1), 16) if m else None


def maximize(window_id):
    lib = ctypes.util.find_library('X11')
    if not lib:
        return False
    x11 = ctypes.CDLL(lib)
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long,
                               ctypes.c_void_p]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    d = x11.XOpenDisplay(None)
    if not d:
        return False
    atom = lambda n: x11.XInternAtom(d, n, 0)
    ev = (ctypes.c_long * 24)()            # XEvent (LP64): ints sit in the low half of 8-byte slots
    ev[0] = 33                             # ClientMessage
    ev[3] = d
    ev[4] = window_id
    ev[5] = atom(b'_NET_WM_STATE')
    ev[6] = 32
    ev[7], ev[8], ev[9], ev[10], ev[11] = (
        1, atom(b'_NET_WM_STATE_MAXIMIZED_VERT'), atom(b'_NET_WM_STATE_MAXIMIZED_HORZ'), 1, 0)
    mask = (1 << 20) | (1 << 19)           # SubstructureRedirect | SubstructureNotify
    x11.XSendEvent(d, x11.XDefaultRootWindow(d), 0, mask, ctypes.byref(ev))
    x11.XFlush(d)
    x11.XCloseDisplay(d)
    return True


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    text = argv[0] if argv else 'RViz'
    max_wait = float(argv[1]) if len(argv) > 1 else 40.0
    t0 = time.time()
    while time.time() - t0 < max_wait:
        wid = find_window(text)
        if wid:
            time.sleep(0.8)                # let RViz finish its own initial resize first
            print('[rviz_maximize] maximised' if maximize(wid) else '[rviz_maximize] not possible')
            return
        time.sleep(0.3)
    print('[rviz_maximize] RViz window not found')


if __name__ == '__main__':
    main()
