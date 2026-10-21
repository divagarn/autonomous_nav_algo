#!/usr/bin/env python3
"""Start-up splash window shown for a fixed time (default 5 s), then closed.

Borderless, always on top, centred on the screen at the image's own size, with a progress bar under
it. The launch starts RViz only AFTER this process exits (OnProcessExit), so nothing else is on screen
while the splash is up and the RViz main page appears when it closes. Esc skips it. No ROS needed.

  splash_screen.py <image.png> [seconds=5]
If Tk or the image is unavailable it exits at once (a splash must never break the launch); RViz then
simply starts immediately.
"""
import sys


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:                    # ros2 launch appends its own arguments
        argv = argv[:argv.index('--ros-args')]
    image = argv[0] if argv else ''
    seconds = float(argv[1]) if len(argv) > 1 else 5.0
    try:
        import tkinter as tk
        from PIL import Image, ImageTk
        root = tk.Tk()
        img = Image.open(image)
    except Exception as e:                      # no display / no Tk / bad image
        print(f'[splash] not shown: {e}')
        return
    w, h = img.size
    bar_h = 8
    root.overrideredirect(True)
    root.attributes('-topmost', True)
    root.geometry(f'{w}x{h + bar_h}+{(root.winfo_screenwidth() - w) // 2}+'
                  f'{(root.winfo_screenheight() - h - bar_h) // 2}')
    root.bind('<Escape>', lambda _e: root.destroy())
    canvas = tk.Canvas(root, width=w, height=h + bar_h, highlightthickness=0, bg='#1a2a6c')
    canvas.pack()
    photo = ImageTk.PhotoImage(img)
    canvas.create_image(0, 0, image=photo, anchor='nw')
    canvas.create_rectangle(0, h, w, h + bar_h, fill='#1a2a6c', outline='')
    bar = canvas.create_rectangle(0, h, 0, h + bar_h, fill='#8fb0ff', outline='')
    steps = max(int(seconds * 20), 1)

    def tick(i=0):
        canvas.coords(bar, 0, h, w * min(i / steps, 1.0), h + bar_h)
        canvas.tag_raise('all')
        root.lift()
        if i >= steps:
            root.destroy()
        else:
            root.after(50, tick, i + 1)
    tick()
    root.mainloop()


if __name__ == '__main__':
    main()
