"""The driving window across three side-by-side monitors (5760x1080 on three
1080p screens).

Two layouts (``--screen-layout``):

  - ``angled``: the side monitors are turned towards you (a sim-rig wrap).
    Three cameras at the driver's eye, one per screen, each covering a third
    of ``--triple-fov`` and yawed by that much, so the view is continuous
    across the bezels with no stretching.
  - ``flat``: the monitors stand in one line. One wide camera spans all three
    (a flat surface needs one flat projection).

The window is borderless and fullscreened across the three monitors with
the EWMH hint _NET_WM_FULLSCREEN_MONITORS (GNOME's top bar does not cover
it), sent through libX11 with ctypes; works on X11 desktops whose monitors
form one screen (here: two on the Intel GPU, one on the NVIDIA, reverse
PRIME).

運転ウインドウを横並び 3 画面に表示する。angled（左右を内側に向けた配置）は画面ごとに
カメラ 1 台、flat（一直線）は横長カメラ 1 台。ウインドウは 3 画面にまたがる全画面。
"""

import ctypes
import ctypes.util
import re
import subprocess

LAYOUTS = ("angled", "flat")


def monitors(text=None):
    """[(index, x, y, w, h)] from ``xrandr --listmonitors``. / モニター一覧。"""
    if text is None:
        try:
            text = subprocess.run(["xrandr", "--listmonitors"], capture_output=True,
                                  text=True, timeout=5).stdout
        except (OSError, subprocess.SubprocessError):
            return []
    out = []
    for m in re.finditer(r"^\s*(\d+):\s+\S+\s+(\d+)/\d+x(\d+)/\d+\+(-?\d+)\+(-?\d+)", text, re.M):
        i, w, h, x, y = (int(g) for g in m.groups())
        out.append((i, x, y, w, h))
    return out


def pick_three(mons):
    """The three monitors in a row (same size, same top, touching), left to
    right, or None. / 横一列に並んだ同じ大きさの 3 画面（左から）。"""
    by_x = sorted(mons, key=lambda m: m[1])
    for k in range(len(by_x) - 2):
        a, b, c = by_x[k:k + 3]
        if (a[3:] == b[3:] == c[3:] and a[2] == b[2] == c[2]
                and b[1] == a[1] + a[3] and c[1] == b[1] + b[3]):
            return [a, b, c]
    return None


def plan(layout, total_fov, screen_w, screen_h):
    """Cameras for the layout: [(image_w, image_h, fov, yaw_offset, screen_x)].
    / 配置ごとのカメラ（画像サイズ・画角・ヨー・画面上の x）。"""
    if layout == "flat":
        fov = min(float(total_fov), 170.0)
        return [(3 * screen_w, screen_h, fov, 0.0, 0)]
    each = float(total_fov) / 3.0
    return [(screen_w, screen_h, each, -each, 0),
            (screen_w, screen_h, each, 0.0, screen_w),
            (screen_w, screen_h, each, each, 2 * screen_w)]


def span_fullscreen(window_id, left, right, top=None, bottom=None):
    """Ask the window manager to fullscreen ``window_id`` across the monitors
    ``left``..``right`` (Xinerama indices). True if the request went out.
    ウインドウを複数モニターにまたがる全画面にするよう WM に依頼する。"""
    top = left if top is None else top
    bottom = left if bottom is None else bottom
    try:
        x11 = ctypes.cdll.LoadLibrary(ctypes.util.find_library("X11") or "libX11.so.6")
    except OSError:
        return False
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

    class ClientMessage(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                    ("display", ctypes.c_void_p), ("window", ctypes.c_ulong),
                    ("message_type", ctypes.c_ulong), ("format", ctypes.c_int),
                    ("l", ctypes.c_long * 5)]

    class XEvent(ctypes.Union):
        _fields_ = [("xclient", ClientMessage), ("pad", ctypes.c_long * 24)]

    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long,
                               ctypes.POINTER(XEvent)]
    dpy = x11.XOpenDisplay(None)
    if not dpy:
        return False
    try:
        root = x11.XDefaultRootWindow(dpy)

        def send(name, data):
            ev = XEvent()
            ev.xclient.type, ev.xclient.send_event = 33, 1          # ClientMessage
            ev.xclient.display, ev.xclient.window = dpy, int(window_id)
            ev.xclient.message_type = x11.XInternAtom(dpy, name, 0)
            ev.xclient.format = 32
            for i, v in enumerate(data):
                ev.xclient.l[i] = v
            # SubstructureRedirectMask | SubstructureNotifyMask
            x11.XSendEvent(dpy, root, 0, (1 << 20) | (1 << 19), ctypes.byref(ev))

        send(b"_NET_WM_FULLSCREEN_MONITORS", [top, bottom, left, right, 1])
        send(b"_NET_WM_STATE", [1, x11.XInternAtom(dpy, b"_NET_WM_STATE_FULLSCREEN", 0), 0, 1, 0])
        x11.XFlush(dpy)
        return True
    finally:
        x11.XCloseDisplay(dpy)
