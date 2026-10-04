#!/usr/bin/env python3
"""Optional helper: start a simulator round hands-free.

Finds the homework2026 X11 window, selects the difficulty (default
超大杯), optionally types a seed and clicks 开始. Lets reviewers replay
deterministic rounds without manual clicking.

Dependencies (only for this helper, not for the node):
    pip install python-xlib pillow

Usage:
    python3 scripts/auto_round.py [difficulty 0|1|2] [seed]
The game window must already be open (on the main menu).
"""
import sys
import time

from Xlib import display, X, XK
from Xlib.ext import xtest

GAME_TITLE = 'homework2026'
# Click positions are relative to the 1152x648 main menu layout.
POS_DIFFICULTY = (628, 452)
POS_OPTION_Y0 = 469       # first popup item centre; +34 px per item
POS_SEED = (679, 408)
POS_START = (575, 590)


def find_window(d):
    stack = [d.screen().root]
    while stack:
        w = stack.pop()
        try:
            name = w.get_wm_name()
        except Exception:
            name = None
        if name and GAME_TITLE in str(name):
            return w
        try:
            stack.extend(w.query_tree().children)
        except Exception:
            pass
    return None


def click(d, w, x, y):
    g = w.get_geometry()
    # window-relative -> absolute (game window is a direct child of root)
    xtest.fake_input(d, X.MotionNotify, x=g.x + int(x), y=g.y + int(y))
    d.sync()
    time.sleep(0.05)
    xtest.fake_input(d, X.ButtonPress, 1)
    d.sync()
    time.sleep(0.06)
    xtest.fake_input(d, X.ButtonRelease, 1)
    d.sync()


def type_text(d, text):
    for ch in text:
        kc = d.keysym_to_keycode(XK.string_to_keysym(ch))
        xtest.fake_input(d, X.KeyPress, kc)
        d.sync()
        time.sleep(0.02)
        xtest.fake_input(d, X.KeyRelease, kc)
        d.sync()
        time.sleep(0.04)


def main():
    difficulty = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    seed = sys.argv[2] if len(sys.argv) > 2 else None
    d = display.Display(':0')
    w = find_window(d)
    if w is None:
        sys.exit('homework2026 window not found (open the game first)')
    click(d, w, *POS_DIFFICULTY)
    time.sleep(0.4)
    click(d, w, POS_DIFFICULTY[0], POS_OPTION_Y0 + difficulty * 34)
    time.sleep(0.3)
    if seed:
        click(d, w, *POS_SEED)
        time.sleep(0.3)
        back = d.keysym_to_keycode(XK.string_to_keysym('BackSpace'))
        for _ in range(14):
            xtest.fake_input(d, X.KeyPress, back)
            d.sync()
            xtest.fake_input(d, X.KeyRelease, back)
            d.sync()
        type_text(d, seed)
    click(d, w, *POS_START)
    print('round started')


if __name__ == '__main__':
    main()
