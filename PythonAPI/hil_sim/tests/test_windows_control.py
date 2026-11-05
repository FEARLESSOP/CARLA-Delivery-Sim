"""Tests for opening / closing the optional windows from the dashboard
(windows_control.py, run_pipeline.py, dashboard.py /windows). No CARLA server:
fakes stand in for the processes, as in tests/test_map_switch.py.

1. the request file: write / merge / read / bad entries / unknown names;
2. the supervisor: launch state, open while running, close = SIGTERM of that
   one process only, user close (exit 0) is not a crash, a crash restarts,
   stale and replayed requests, an unavailable window, the status file;
3. the dashboard side: read_status live / stale / dead pipeline;
4. run_pipeline.main end to end with the user's command: open the sensor
   viewer and close the drone cameras from the dashboard, F5 -> the same
   windows are open on the new map (not the launch flags), a window closed with
   its own close button stays closed, a crash is restarted, no shutdown.

ダッシュボードからの任意ウィンドウ開閉の試験（CARLA サーバー不要。プロセスは
偽物。tests/test_map_switch.py と同じ方式）。
"""
import json
import os
import signal
import sys
import tempfile
import time
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import map_switch as ms  # noqa: E402
import run_pipeline as rp  # noqa: E402
import windows_control as wc  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


TMP = tempfile.mkdtemp(prefix="windows_control_test_")
CTRL = os.path.join(TMP, "pipeline_control.json")
STAT = os.path.join(TMP, "pipeline_status.json")
REQ = os.path.join(TMP, "map_request.json")

print("\n=== 1. the request file ===")
wc.write_request("drone_cams", "open", path=CTRL, now=100.0)
wc.write_request("sensor_viewer", "close", path=CTRL, now=100.0)
r = wc.read_requests(CTRL)
check("two windows' requests both kept (merge)",
      r == {"drone_cams": ("open", 100.0), "sensor_viewer": ("close", 100.0)}, str(r))
wc.write_request("drone_cams", "close", path=CTRL, now=100.0)
r = wc.read_requests(CTRL)
check("a second click wins and its t still increases",
      r["drone_cams"][0] == "close" and r["drone_cams"][1] > 100.0, str(r))
check("no temp file left (atomic write)", not os.path.exists(CTRL + ".tmp"))
for bad in (("nope", "open"), ("drone_cams", "explode"), (None, "open")):
    try:
        wc.write_request(bad[0], bad[1], path=CTRL)
        check("rejects %s" % (bad,), False)
    except ValueError:
        check("rejects %s" % (bad,), True)
with open(CTRL, "w") as f:
    json.dump({"drone_cams": {"action": "open", "t": "x"}, "manual_control": {"action": "close", "t": 1},
               "sensor_viewer": {"action": "open", "t": 5}}, f)
check("bad entries and non-optional windows (manual_control) are skipped",
      wc.read_requests(CTRL) == {"sensor_viewer": ("open", 5.0)}, str(wc.read_requests(CTRL)))
with open(CTRL, "w") as f:
    f.write("{broken")
check("a corrupt file reads as no requests", wc.read_requests(CTRL) == {})
check("the driving window is not an optional window", "manual_control" not in wc.WINDOWS)

print("\n=== 2. the supervisor ===")
LAUNCHED, STOPPED = [], []


class Proc(object):
    _pid = [70000]

    def __init__(self, name, cmd):
        Proc._pid[0] += 1
        self.pid, self.name, self.cmd, self.returncode = Proc._pid[0], name, cmd, None

    def poll(self):
        return self.returncode


def fake_launch(cmd, name):
    LAUNCHED.append(name)
    return Proc(name, cmd)


def fake_stop(procs):
    for p, n in procs:
        STOPPED.append(n)
        p.returncode = -signal.SIGTERM


CMDS = {"sensor_viewer": ["py", "sensor_viewer.py", "--res", "1440x720"],
        "drone_cams": ["py", "drone_viewer.py", "--res", "1280x720"]}
with open(CTRL, "w") as f:      # left over from an earlier run / 以前の実行の残骸
    json.dump({"sensor_viewer": {"action": "open", "t": time.time() - 5}}, f)
sup = wc.WindowSupervisor({"sensor_viewer": False, "drone_cams": True}, CMDS,
                          fake_launch, fake_stop, control_path=CTRL, status_path=STAT)
check("a leftover request file is cleared at start", not os.path.exists(CTRL))
children = [(Proc("traffic", []), "generate_traffic"), (Proc("mc", []), "manual_control")]
sup.start_wanted(children)
check("launch flags: drone cams open, sensor viewer closed", LAUNCHED == ["drone_cams"], str(LAUNCHED))
dc = sup.find(children, "drone_cams")
check("same command line as at launch", dc is not None and dc.cmd == CMDS["drone_cams"])

wc.write_request("drone_cams", "open", path=CTRL)
done = sup.poll_requests(children)
check("open while running does nothing", done == [("drone_cams", "already-open")]
      and LAUNCHED == ["drone_cams"], "%s %s" % (done, LAUNCHED))
check("the same request is not replayed", sup.poll_requests(children) == [])

wc.write_request("sensor_viewer", "open", path=CTRL)
done = sup.poll_requests(children)
check("open a closed window launches it once", done == [("sensor_viewer", "opened")]
      and LAUNCHED == ["drone_cams", "sensor_viewer"], "%s %s" % (done, LAUNCHED))

wc.write_request("drone_cams", "close", path=CTRL)
done = sup.poll_requests(children)
names = [n for _, n in children]
check("close stops only that process", done == [("drone_cams", "closed")]
      and STOPPED == ["drone_cams"], "%s %s" % (done, STOPPED))
check("closed window leaves children (nothing to restart)", "drone_cams" not in names, str(names))
check("traffic and the driving window untouched",
      "generate_traffic" in names and "manual_control" in names
      and all(p.returncode is None for p, n in children), str(names))
check("closed is remembered", sup.want == {"sensor_viewer": True, "drone_cams": False}, str(sup.want))
wc.write_request("drone_cams", "close", path=CTRL)
check("close a closed window does nothing", sup.poll_requests(children) == [("drone_cams", "already-closed")]
      and STOPPED == ["drone_cams"])

sv = sup.find(children, "sensor_viewer")
sv.returncode = 0           # user clicked the window's X / Esc / ウィンドウの × / Esc
check("exit 0 = closed by the user (handled, no restart)",
      sup.handle_exit(children, "sensor_viewer", 0) is True and sup.want["sensor_viewer"] is False
      and "sensor_viewer" not in [n for _, n in children])
check("a crash (rc 1 / -11) is not a user close",
      sup.handle_exit(children, "sensor_viewer", 1) is False
      and sup.handle_exit(children, "sensor_viewer", -11) is False)
check("non-window helpers are never handled as a close",
      sup.handle_exit(children, "generate_traffic", 0) is False)
left = [sup.take_restart("drone_cams") for _ in range(5)]
check("crash restarts are bounded", left == [True, True, True, False, False], str(left))
sup.start_wanted(children)
check("a new world (start_wanted) gives fresh restarts", sup.restart_left["drone_cams"] == wc.WINDOW_RESTARTS)

st = sup.write_status(children, "running", "OitaKosen")
disk = json.load(open(STAT))
check("status file written", disk["windows"].keys() == wc.WINDOWS.keys() and disk["map"] == "OitaKosen"
      and disk["pid"] == os.getpid(), str(disk))
wc.write_request("sensor_viewer", "open", path=CTRL)
sup.poll_requests(children)
st = sup.write_status(children, "running", "OitaKosen")
sv = sup.find(children, "sensor_viewer")
check("status: running + pid of an open window",
      st["windows"]["sensor_viewer"] == {"running": True, "pid": sv.pid, "wanted": True,
                                         "available": True, "reason": ""}, str(st["windows"]))
check("status: closed window", st["windows"]["drone_cams"]["running"] is False
      and st["windows"]["drone_cams"]["pid"] is None)

LAUNCHED[:] = []
sup2 = wc.WindowSupervisor({"sensor_viewer": True, "drone_cams": True}, CMDS, fake_launch, fake_stop,
                           unavailable={"drone_cams": "needs --delivery N"},
                           control_path=CTRL, status_path=STAT)
ch2 = []
sup2.start_wanted(ch2)
check("drone cams without --delivery are never launched", LAUNCHED == ["sensor_viewer"], str(LAUNCHED))
wc.write_request("drone_cams", "open", path=CTRL)
check("opening an unavailable window is refused",
      sup2.poll_requests(ch2) == [("drone_cams", "unavailable")] and LAUNCHED == ["sensor_viewer"])
st = sup2.status(ch2)
check("status says why", st["windows"]["drone_cams"]["available"] is False
      and "delivery" in st["windows"]["drone_cams"]["reason"])

print("\n=== 3. the dashboard side (read_status) ===")
sup.write_status(children, "running", "OitaKosen")
rs = wc.read_status(STAT)
check("fresh status from a live pid -> live", rs["live"] is True and rs["map"] == "OitaKosen")
row = {w["name"]: w for w in rs["windows"]}
check("every window listed with labels", list(row) == list(wc.WINDOWS)
      and row["sensor_viewer"]["ja"] == "センサービューア" and row["sensor_viewer"]["running"] is True)
check("stale status -> not live, nothing running",
      wc.read_status(STAT, now=time.time() + 60)["live"] is False
      and not any(w["running"] for w in wc.read_status(STAT, now=time.time() + 60)["windows"]))
d = json.load(open(STAT))
d["pid"] = 2 ** 22 + 12345          # no such process / 存在しない pid
json.dump(d, open(STAT, "w"))
check("status of a dead pipeline -> not live", wc.read_status(STAT)["live"] is False)
os.remove(STAT)
check("no status file -> not live, rows still listed",
      wc.read_status(STAT)["live"] is False and len(wc.read_status(STAT)["windows"]) == len(wc.WINDOWS))

print("\n=== 4. run_pipeline.main end to end, all processes faked ===")
E_LAUNCH, E_SIG, LOG = [], [], []
STEP = {"mc": 0, "poll": 0}
PROCS = []


def status():
    return json.load(open(STAT))["windows"]


def live(name):
    return [p for p in PROCS if p.name == name and p.returncode is None]


class FakeProc(object):
    _pid = [91000]

    def __init__(self, name, cmd):
        FakeProc._pid[0] += 1
        self.pid, self.name, self.cmd, self.returncode = FakeProc._pid[0], name, cmd, None
        if name == "manual_control":
            STEP["mc"] += 1
            STEP["poll"] = 0
        PROCS.append(self)

    def poll(self):
        if self.name == "manual_control" and self.returncode is None:
            STEP["poll"] += 1
            script = SCRIPTS.get(STEP["mc"], {})
            fn = script.get(STEP["poll"])
            if fn:
                fn(self)
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = -15
        return self.returncode


def f5(mc):
    path = mc.cmd[mc.cmd.index("--map-request") + 1]
    ms.write_request("OitaStation", hero="vehicle.mazda.rx7", current="OitaKosen", path=path)
    mc.returncode = ms.MAP_SWITCH_RC


def note(msg):
    def fn(mc):
        LOG.append(msg)
    return fn


def run1_open_sensor(mc):
    LOG.append(("run1 start", status()))
    wc.write_request("sensor_viewer", "open", path=CTRL)


def run1_close_drone(mc):
    LOG.append(("run1 after open", status(), len(live("sensor_viewer"))))
    wc.write_request("drone_cams", "close", path=CTRL)


def run1_check(mc):
    LOG.append(("run1 after close", status(), len(live("drone_cams")),
                [n for n, s in E_SIG if s == signal.SIGTERM]))


def run2_start(mc):
    LOG.append(("run2 start", status(), len(live("sensor_viewer")), len(live("drone_cams"))))
    live("sensor_viewer")[0].returncode = 0         # user clicks X / × を押す


def run2_after_user_close(mc):
    LOG.append(("run2 after X", status(), len(live("sensor_viewer"))))
    wc.write_request("drone_cams", "open", path=CTRL)


def run2_crash_drone(mc):
    LOG.append(("run2 drone opened", len(live("drone_cams"))))
    live("drone_cams")[0].returncode = -11          # segfault / クラッシュ


def run2_after_crash(mc):
    LOG.append(("run2 after crash", len(live("drone_cams")), status()))


def quit_(mc):
    mc.returncode = 0


SCRIPTS = {
    1: {1: run1_open_sensor, 2: run1_close_drone, 3: run1_check, 4: f5},
    2: {1: run2_start, 2: run2_after_user_close, 3: run2_crash_drone, 4: run2_after_crash, 5: quit_},
}

saved = {k: getattr(rp, k) for k in ("launch", "signal_group", "port_in_use", "world_map_name",
                                      "load_world_map", "reset_world_async", "subprocess")}
saved_sleep, saved_argv = rp.time.sleep, sys.argv
saved_paths = (wc.CONTROL_PATH, wc.STATUS_PATH, ms.REQUEST_PATH)
rp.launch = lambda cmd, cwd, name, env=None: (E_LAUNCH.append(name) or FakeProc(name, list(cmd)))
rp.signal_group = lambda proc, sig: (E_SIG.append((proc.name, sig)),
                                     setattr(proc, "returncode", -sig))
rp.port_in_use = lambda port: False
rp.world_map_name = lambda host, port, timeout=10.0: "OitaKosen"
rp.load_world_map = lambda host, port, name, timeout=120.0: True
rp.reset_world_async = lambda host, port, timeout=10.0: False
rp.subprocess = types.SimpleNamespace(
    call=lambda cmd, cwd=None: 0,
    run=lambda cmd, capture_output=True: types.SimpleNamespace(stdout=b""),
    TimeoutExpired=saved["subprocess"].TimeoutExpired)
rp.time.sleep = lambda s: None
wc.CONTROL_PATH, wc.STATUS_PATH, ms.REQUEST_PATH = CTRL, STAT, REQ
take = rp.take_map_request
rp.take_map_request = lambda rc, path=None, now=None: take(rc, REQ, now)
try:
    # the user's command / ユーザーの通常コマンド
    sys.argv = ["run_pipeline.py", "--no-sim", "--map", "OitaKosen", "--vehicles", "10",
                "--walkers", "10", "--delivery", "3", "--ffb", "--no-sensor-viewer",
                "--start-vehicle", "vehicle.mazda.rx7", "--res", "1080p", "--drone-cams",
                "--dashboard-port", "5999"]
    rc = rp.main()
finally:
    for k, v in saved.items():
        setattr(rp, k, v)
    rp.time.sleep, sys.argv = saved_sleep, saved_argv
    rp.take_map_request = take
    wc.CONTROL_PATH, wc.STATUS_PATH, ms.REQUEST_PATH = saved_paths

L = {e[0]: e for e in LOG if isinstance(e, tuple)}
check("pipeline ran both windows and exited 0 (no crash shutdown)", rc == 0 and STEP["mc"] == 2,
      "rc=%s runs=%s" % (rc, STEP["mc"]))
first = E_LAUNCH[:E_LAUNCH.index("manual_control") + 1]
check("launch flags honoured: drone cams yes, sensor viewer no",
      "drone_cams" in first and "sensor_viewer" not in first, str(first))
s = L["run1 start"][1]
check("status file shows the launch state",
      s["drone_cams"]["running"] and not s["sensor_viewer"]["running"]
      and s["drone_cams"]["pid"], str(s))
s = L["run1 after open"]
check("dashboard open -> sensor viewer running, once", s[1]["sensor_viewer"]["running"] and s[2] == 1,
      str(s))
sv_cmd = [p.cmd for p in PROCS if p.name == "sensor_viewer"][0]
check("opened with the launch command line (--res, --fps, --sensor-fps)",
      sv_cmd[sv_cmd.index("--res") + 1] == "1440x720" and "--sensor-fps" in sv_cmd, str(sv_cmd))
s = L["run1 after close"]
check("dashboard close -> drone cams stopped, SIGTERM to it alone",
      not s[1]["drone_cams"]["running"] and s[2] == 0 and s[3] == ["drone_cams"], str(s))
s = L["run2 start"]
check("after F5: user's choice kept (sensor viewer open, drone cams closed)",
      s[2] == 1 and s[3] == 0 and s[1]["sensor_viewer"]["running"]
      and not s[1]["drone_cams"]["running"], str(s))
second = E_LAUNCH[len(first):E_LAUNCH.index("manual_control", len(first)) + 1]
check("after F5: launch list follows the state, not the flags",
      "sensor_viewer" in second and "drone_cams" not in second, str(second))
s = L["run2 after X"]
check("window's own close button: not restarted, shown closed",
      s[2] == 0 and not s[1]["sensor_viewer"]["running"] and not s[1]["sensor_viewer"]["wanted"], str(s))
check("drone cams can be opened again after a map switch", L["run2 drone opened"][1] == 1)
s = L["run2 after crash"]
check("a crash (rc -11) is restarted", s[1] == 1 and s[2]["drone_cams"]["running"], str(s))
check("sensor viewer launched exactly twice (dashboard open + map-switch restore)",
      E_LAUNCH.count("sensor_viewer") == 2, str(E_LAUNCH))
check("traffic never restarted by any window action", E_LAUNCH.count("generate_traffic") == 2,
      str(E_LAUNCH))
check("status file removed at shutdown", not os.path.exists(STAT))
check("the real side-channel files were not touched",
      wc.CONTROL_PATH.startswith(HIL) and wc.STATUS_PATH.startswith(HIL))

print("\n%d checks failed" % len(FAILS))
for f in FAILS:
    print("  - " + f)
sys.exit(1 if FAILS else 0)
