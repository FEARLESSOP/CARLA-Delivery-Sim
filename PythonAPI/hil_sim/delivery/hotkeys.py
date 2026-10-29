"""Read key presses made in the CARLA window, from another process (X11, ctypes).

The CARLA server window belongs to the simulator, not to this script, so it
can't deliver key events to us. Instead this polls the X server: which keys
are down right now (XQueryKeymap, no grab, so other programs still get every
key) and which window has the focus. A key counts only while the CARLA
window is focused, so Q in the driving window (reverse) doesn't also cycle
the camera. Needs an X11 session; without one, ``ok`` is False and ``poll``
returns nothing.

CARLA ウィンドウで押されたキーを別プロセスから読む（X11・ctypes）。押下中の
キー（XQueryKeymap、他のアプリへの入力は奪わない）とフォーカス中のウィンドウ
を問い合わせ、CARLA ウィンドウにフォーカスがあるときだけキーを有効にする。
"""

import ctypes
import ctypes.util

CARLA_WINDOW_CLASS = "CarlaUE4-Linux-Shipping"
XA_WINDOW = 33


class _ClassHint(ctypes.Structure):
    _fields_ = [("res_name", ctypes.c_char_p), ("res_class", ctypes.c_char_p)]


# Xlib's default error handler EXITS the process on any error -- e.g. asking
# for the class of a window that just closed, or of window 0 when nothing has
# the focus. That killed the delivery sim once, so errors are ignored instead.
# Xlib の既定のエラーハンドラはプロセスを終了させる（閉じたウィンドウや
# フォーカス無し＝ウィンドウ 0 の問い合わせで）。そのためエラーを無視する。
_X_ERROR_HANDLER = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)
_ignore_x_errors = _X_ERROR_HANDLER(lambda display, event: 0)   # kept alive here


class Hotkeys(object):
    """``poll()`` -> keys newly pressed since the last call, e.g. ["q"]."""

    def __init__(self, keys=("q", "e"), window_class=CARLA_WINDOW_CLASS):
        self.ok = False
        self.window_class = window_class
        self._down = set()
        self._class_cache = {}
        name = ctypes.util.find_library("X11")
        if not name:
            return
        try:
            x = self.x = ctypes.cdll.LoadLibrary(name)
            x.XOpenDisplay.restype = ctypes.c_void_p
            x.XOpenDisplay.argtypes = [ctypes.c_char_p]
            x.XDefaultRootWindow.restype = ctypes.c_ulong
            x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
            x.XStringToKeysym.restype = ctypes.c_ulong
            x.XStringToKeysym.argtypes = [ctypes.c_char_p]
            x.XKeysymToKeycode.restype = ctypes.c_ubyte
            x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            x.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
            x.XInternAtom.restype = ctypes.c_ulong
            x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
            x.XGetWindowProperty.argtypes = [
                ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_long, ctypes.c_long,
                ctypes.c_int, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
                ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_ulong),
                ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_void_p)]
            x.XGetClassHint.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                        ctypes.POINTER(_ClassHint)]
            x.XFree.argtypes = [ctypes.c_void_p]
            x.XSetErrorHandler.argtypes = [_X_ERROR_HANDLER]
            x.XSetErrorHandler.restype = ctypes.c_void_p
            x.XSetErrorHandler(_ignore_x_errors)
            self.dpy = x.XOpenDisplay(None)
            if not self.dpy:
                return
            self.root = x.XDefaultRootWindow(self.dpy)
            self.active_atom = x.XInternAtom(self.dpy, b"_NET_ACTIVE_WINDOW", 0)
            self.codes = {k: x.XKeysymToKeycode(self.dpy, x.XStringToKeysym(k.encode()))
                          for k in keys}
            self.ok = all(self.codes.values())
        except (OSError, AttributeError):
            self.ok = False

    def _active_window(self):
        x = self.x
        typ, fmt = ctypes.c_ulong(), ctypes.c_int()
        n, after, data = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_void_p()
        if x.XGetWindowProperty(self.dpy, self.root, self.active_atom, 0, 1, 0, XA_WINDOW,
                                ctypes.byref(typ), ctypes.byref(fmt), ctypes.byref(n),
                                ctypes.byref(after), ctypes.byref(data)) != 0 or not data:
            return None
        win = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))[0] if n.value else None
        x.XFree(data)
        return win or None          # 0: nothing has the focus / 0 はフォーカス無し

    def _window_class(self, win):
        if win not in self._class_cache:
            hint = _ClassHint()
            cls = None
            if self.x.XGetClassHint(self.dpy, win, ctypes.byref(hint)):
                cls = (hint.res_class or b"").decode(errors="replace")
            self._class_cache[win] = cls
        return self._class_cache[win]

    def focused(self):
        """The CARLA window has the keyboard focus. / CARLA ウィンドウが前面か。"""
        win = self._active_window()
        return win is not None and self._window_class(win) == self.window_class

    def poll(self):
        if not self.ok:
            return []
        keymap = (ctypes.c_char * 32)()
        self.x.XQueryKeymap(self.dpy, keymap)
        raw = bytes(keymap)
        down = {k for k, c in self.codes.items() if raw[c // 8] >> (c % 8) & 1}
        new = down - self._down
        self._down = down
        if new and not self.focused():
            return []
        return sorted(new)
