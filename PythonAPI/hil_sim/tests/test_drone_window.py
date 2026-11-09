#!/usr/bin/env python3
"""The Drones page's CARLA pop-up: drone_windows.py (start / focus / replace /
close one detached window per drone) and drone_window.py's camera logic
(what the cameras follow, re-attaching, the pod, a crashed drone) with a fake
world. No CARLA server (needs carla + pygame in the venv).

  ../../carla-venv/bin/python tests/test_drone_window.py

ドローンの CARLA 窓の試験（サーバー不要）。
"""
import os
import sys
import tempfile
import time
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import drone_windows as dw  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("=== drone_windows: one detached window per drone / 1 機 1 窓 ===")
tmp = tempfile.mkdtemp(prefix="dronewin_")
fake = os.path.join(tmp, "drone_window.py")
with open(fake, "w") as f:
    f.write("import signal, sys, time\n"
            "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
            "open(sys.argv[-1] if sys.argv[-1].endswith('.mark') else '/dev/null', 'w').close()\n"
            "time.sleep(60)\n")
pid_dir = os.path.join(tmp, "pids")
no_focus = lambda d: False  # noqa: E731
act, info = dw.open_window(3, "10.0.0.5", 2001, python=sys.executable, pid_dir=pid_dir, script=fake,
                           env={"PATH": os.environ.get("PATH", "")}, focus=no_focus)
time.sleep(0.4)
ws = dw.list_windows(pid_dir)
check("open: started detached, listed as running", act == "opened" and [w["drone"] for w in ws] == [3]
      and ws[0]["running"], (act, ws))
with open("/proc/%d/cmdline" % info["pid"], "rb") as f:
    args = f.read().split(b"\0")
check("command line carries drone, host and port", [b"--drone", b"3"] == args[2:4] and b"10.0.0.5" in args
      and b"2001" in args, args)
with open("/proc/%d/environ" % info["pid"], "rb") as f:
    env = f.read()
check("DISPLAY passed (default :1 when the dashboard has none)", b"DISPLAY=:1" in env)
check("its own session (survives a dashboard restart)", os.getsid(info["pid"]) == info["pid"])
act2, info2 = dw.open_window(3, python=sys.executable, pid_dir=pid_dir, script=fake, focus=lambda d: True)
check("second open with a window tool: focused, same process", act2 == "focused" and
      dw.list_windows(pid_dir)[0]["pid"] == info["pid"])
act3, info3 = dw.open_window(3, python=sys.executable, pid_dir=pid_dir, script=fake, focus=no_focus)
time.sleep(0.4)
ws = dw.list_windows(pid_dir)
check("second open without one: replaced (old one stopped)", act3 == "replaced" and len(ws) == 1 and
      ws[0]["pid"] == info3["pid"] and not dw._alive(info["pid"], 3), ws)
dw.open_window(5, python=sys.executable, pid_dir=pid_dir, script=fake, focus=no_focus)
time.sleep(0.4)
check("another drone: its own window", sorted(w["drone"] for w in dw.list_windows(pid_dir)) == [3, 5])
check("close: SIGTERM, gone, pid file removed", dw.close_window(3, pid_dir) == "closed" and
      [w["drone"] for w in dw.list_windows(pid_dir)] == [5] and not os.path.exists(os.path.join(pid_dir, "3.json")))
check("close a closed window: not_open", dw.close_window(3, pid_dir) == "not_open")
with open(os.path.join(pid_dir, "7.json"), "w") as f:
    f.write('{"drone": 7, "pid": 1}')          # pid 1 is not drone_window.py / 別プロセス
check("stale / foreign pid file ignored and removed", 7 not in [w["drone"] for w in dw.list_windows(pid_dir)]
      and not os.path.exists(os.path.join(pid_dir, "7.json")))
old_max = dw.MAX_WINDOWS
dw.MAX_WINDOWS = 1
check("window limit", dw.open_window(8, python=sys.executable, pid_dir=pid_dir, script=fake,
                                     focus=no_focus)[0] == "too_many")
dw.MAX_WINDOWS = old_max
dw.close_window(5, pid_dir)
check("all closed", dw.list_windows(pid_dir) == [])

print("\n=== drone_window: what the cameras follow / 追従対象 ===")
import drone_window as win  # noqa: E402

vans = {2: {"id": 2, "actor": 154}}
check("flying: the body", win.target_of({"status": "delivering", "actor": 77, "van": 2}, vans) == ("body", 77))
check("docked: the pod car", win.target_of({"status": "docked", "actor": 77, "van": 2}, vans) == ("pod", 154))
check("docked, pod unknown: the body", win.target_of({"status": "docked", "actor": 77, "van": 9}, vans) == ("body", 77))
check("no body: nothing", win.target_of({"status": "crashed", "actor": None}, vans) == (None, None))
check("views cycle", win.next_view("rear") == "chase" and win.next_view("chase", -1) == "rear")
(pos, rot) = win.level_follow((10.0, 0.0, 5.0), 0.0)
check("level follow: behind and above, looking down, no roll", pos[0] < 10.0 and pos[2] > 5.0 and rot[0] < 0)
(pos, rot) = win.orbit_pose((0.0, 0.0, 0.0), 0.0)
check("orbit: on the circle, facing the drone", abs(pos[0] - win.ORBIT_R) < 1e-6 and abs(abs(rot[1]) - 180.0) < 1e-6)
w = win.warnings_of({"status": "crashed", "fail": "collision:Terrain",
                     "sensors": {"damage": {"parts": {"rotor1": {"level": "broken", "why": "collision"},
                                                      "shell": {"level": "damaged", "why": "collision"}}},
                                 "motors": {"health": "fail"}, "gnss": {"health": "ok"}}})
check("HUD warnings: down reason first, broken before damaged, faults",
      w[0].startswith("DOWN") and w[1].startswith("BROKEN ROTOR1") and w[2].startswith("DAMAGED SHELL")
      and "FAULT MOTORS" in w, w)

print("\n=== drone_window: cameras on a fake world / 擬似ワールドでのカメラ ===")
made, gone = [], []


class FakeCam(object):
    def __init__(self, world, size, fov, tf, parent, drone, tick=0.0):
        self.size, self.parent, self.tf, self.surface = size, parent.id if parent else None, tf, None
        self.moves = []
        made.append(self)

    def move(self, loc, rot):
        self.moves.append((loc, rot))

    def destroy(self, client):
        gone.append(self)


class Actor(object):
    def __init__(self, i):
        self.id, self.is_alive = i, True

    def get_location(self):
        return types.SimpleNamespace(x=1.0, y=2.0, z=3.0)

    def get_transform(self):
        return None


win.Cam = FakeCam
actors = {77: Actor(77), 78: Actor(78), 154: Actor(154)}
W = win.Window.__new__(win.Window)
W.args = types.SimpleNamespace(drone=3, no_strip=False, strip_fps=10.0)
W.big, W.tile, W.client, W.t0 = (1280, 640), (320, 160), None, time.time()
W.world = types.SimpleNamespace(get_actor=lambda i: actors.get(i))
W.view, W.main, W.main_key, W.strip, W.strip_body, W.status_line = "chase", None, None, {}, None, ""
W.vans = vans
W.drone = {"id": 3, "status": "delivering", "actor": 77, "van": 2, "yaw": 30.0}
kind = W.keep_cameras()
check("flying: chase camera rides on the body + 4 onboard cameras on it",
      kind == "body" and W.main.parent == 77 and len(W.strip) == 4 and all(c.parent == 77 for c in W.strip.values()))
n = len(made)
W.keep_cameras()
check("same body: nothing respawned", len(made) == n and not gone)
W.drone = dict(W.drone, actor=78)
W.keep_cameras()
check("body actor changed: everything re-attached to the new one", W.main.parent == 78 and
      all(c.parent == 78 for c in W.strip.values()) and len(gone) == 5)
W.view = "fpv"
W.keep_cameras()
check("FPV view: the big camera at the FPV mount on the body", W.main.parent == 78 and W.main_key[0] == "fpv")
W.drone = dict(W.drone, status="docked")
kind = W.keep_cameras()
check("docked: the big camera stays on the pod car, onboard cameras off",
      kind == "pod" and W.main.parent == 154 and W.main_key[0] == "chase" and not W.strip)
W.view = "chase"
W.drone = dict(W.drone, status="crashed", actor=78)
W.keep_cameras()
check("crashed (lying on its side): a free, level camera frames it", W.main.parent is None and W.main.moves)
W.view = "orbit"
W.keep_cameras()
check("orbit: a free camera moving round it", W.main.parent is None and W.main_key[0] == "orbit")
actors.pop(78)
W.keep_cameras()
check("body gone: all cameras destroyed", W.main is None and not W.strip)
W.close()

print("\n=== dashboard.py /drones/window route / ダッシュボードのルート ===")
import dashboard  # noqa: E402

calls = []
dashboard.DRONE_WIN.update(host="127.0.0.9", port=2042)
dashboard.drone_windows.open_window = lambda d, h, p: (calls.append(("open", d, h, p)) or ("opened", {"drone": d, "pid": 1}))
dashboard.drone_windows.close_window = lambda d: (calls.append(("close", d)) or "closed")
dashboard.drone_windows.list_windows = lambda: [{"drone": 4, "pid": 1, "running": True}]
c = dashboard.app.test_client()
r = c.post("/drones/window", json={"drone": 4})
check("POST open: launches with the dashboard's CARLA host / port", r.status_code == 200 and
      r.get_json()["action"] == "opened" and calls[-1] == ("open", 4, "127.0.0.9", 2042), r.get_json())
r = c.post("/drones/window", json={"drone": 4, "action": "close"})
check("POST close", r.get_json()["action"] == "closed" and calls[-1] == ("close", 4))
check("POST without a drone: 400", c.post("/drones/window", json={}).status_code == 400)
check("GET lists the windows", c.get("/drones/window").get_json()["windows"][0]["drone"] == 4)
check("old camera routes retired", c.get("/drones/cams?drone=1").status_code == 404)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
