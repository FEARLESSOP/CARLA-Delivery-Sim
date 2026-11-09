"""Start / stop the per-drone CARLA windows (drone_window.py) for the dashboard.

The Drones page's "Open CARLA view" posts to dashboard.py, which calls
``open_window(drone)`` here: drone_window.py starts as a detached process
(its own session, so it outlives a dashboard restart and never gets the
dashboard's signals) with the CARLA host / port and the X display. One window
per drone: a pid file per drone in ``data/drone_windows/<id>.json``. A second
open of the same drone brings that window to the front when wmctrl or xdotool
is installed, and otherwise replaces it (the new window opens on top).
``close_window`` sends SIGTERM (drone_window.py destroys its sensors on the
way out), then SIGKILL after a few seconds. No carla import: the dashboard
stays light and tests/test_drone_window.py runs without a server.

ドローンごとの CARLA ウィンドウ（drone_window.py）の起動・終了。1 機に 1 窓、
2 回目は前面へ（wmctrl / xdotool が無ければ開き直す）。
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PID_DIR = os.path.join(HERE, "data", "drone_windows")
SCRIPT = os.path.join(HERE, "drone_window.py")
TITLE = "CARLA view - Drone %d"
MAX_WINDOWS = 4             # each window is ~5 render passes on the server / 描画負荷の上限
_PROCS = {}                 # pid -> Popen we started (reaped here, no zombies) / 子プロセス


def _pid_file(drone, pid_dir=None):
    return os.path.join(pid_dir or PID_DIR, "%d.json" % int(drone))


def _alive(pid, drone):
    """The process exists and is drone_window.py for this drone. / 生存確認。"""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            args = f.read().split(b"\0")
    except OSError:
        return False
    try:
        with open("/proc/%d/stat" % pid) as f:
            if f.read().rsplit(")", 1)[1].split()[0] == "Z":      # zombie / ゾンビ
                return False
    except (OSError, IndexError):
        pass
    a = [x.decode(errors="replace") for x in args]
    return (any(x.endswith("drone_window.py") for x in a) and "--drone" in a
            and a[a.index("--drone") + 1:a.index("--drone") + 2] == [str(int(drone))])


def list_windows(pid_dir=None):
    """[{drone, pid, running, started}] of every recorded window (stale files removed)."""
    d = pid_dir or PID_DIR
    out = []
    for pid, p in list(_PROCS.items()):
        if p.poll() is not None:
            _PROCS.pop(pid, None)
    if not os.path.isdir(d):
        return out
    for name in sorted(os.listdir(d)):
        if not name.endswith(".json"):
            continue
        path = os.path.join(d, name)
        try:
            with open(path) as f:
                rec = json.load(f)
            drone, pid = int(rec["drone"]), int(rec["pid"])
        except (OSError, ValueError, KeyError, TypeError):
            _rm(path)
            continue
        if _alive(pid, drone):
            out.append({"drone": drone, "pid": pid, "running": True, "started": rec.get("started")})
        else:
            _rm(path)
    return out


def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass


def _focus(drone):
    """Raise the drone's window if a window tool is installed. / 窓を前面へ。"""
    title = TITLE % int(drone)
    for cmd in (["wmctrl", "-a", title], ["xdotool", "search", "--name", title, "windowactivate"]):
        if shutil.which(cmd[0]):
            try:
                return subprocess.run(cmd, timeout=3, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL).returncode == 0
            except (OSError, subprocess.SubprocessError):
                continue
    return False


def close_window(drone, pid_dir=None, wait=3.0):
    """SIGTERM the drone's window (SIGKILL if it hangs). / 窓を閉じる。"""
    path = _pid_file(drone, pid_dir)
    for w in list_windows(pid_dir):
        if w["drone"] != int(drone):
            continue
        try:
            os.killpg(w["pid"], signal.SIGTERM)       # its own session = its group / 自分のグループ
        except OSError:
            try:
                os.kill(w["pid"], signal.SIGTERM)
            except OSError:
                pass
        t_end = time.time() + wait
        while time.time() < t_end and _alive(w["pid"], drone):
            time.sleep(0.1)
        if _alive(w["pid"], drone):
            try:
                os.killpg(w["pid"], signal.SIGKILL)
            except OSError:
                pass
        _rm(path)
        return "closed"
    _rm(path)
    return "not_open"


def open_window(drone, host="127.0.0.1", port=2000, python=None, pid_dir=None,
                script=None, extra=(), env=None, focus=_focus):
    """Open (or bring forward / replace) the CARLA window of one drone.

    Returns (action, info): action "opened" | "focused" | "replaced" | "too_many".
    1 機の CARLA 窓を開く（既にあれば前面へ、できなければ開き直す）。
    """
    drone = int(drone)
    d = pid_dir or PID_DIR
    os.makedirs(d, exist_ok=True)
    running = list_windows(d)
    action = "opened"
    if any(w["drone"] == drone for w in running):
        if focus(drone):
            return "focused", {"drone": drone}
        close_window(drone, d)
        action = "replaced"
    elif len(running) >= MAX_WINDOWS:
        return "too_many", {"max": MAX_WINDOWS}
    e = dict(os.environ if env is None else env)
    e.setdefault("DISPLAY", ":1")
    cmd = [python or sys.executable, script or SCRIPT, "--drone", str(drone),
           "--host", str(host), "-p", str(int(port))] + list(extra)
    log = open(os.path.join(d, "%d.log" % drone), "ab")
    try:
        p = subprocess.Popen(cmd, cwd=HERE, env=e, stdin=subprocess.DEVNULL, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
    finally:
        log.close()
    _PROCS[p.pid] = p
    with open(_pid_file(drone, d), "w") as f:
        json.dump({"drone": drone, "pid": p.pid, "started": time.time(), "cmd": cmd}, f)
    return action, {"drone": drone, "pid": p.pid}
