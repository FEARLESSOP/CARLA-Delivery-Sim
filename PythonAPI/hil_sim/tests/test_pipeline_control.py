#!/usr/bin/env python3
"""Live control of a running pipeline from the dashboard (run_pipeline.py,
pipeline_control.py), end to end with every process faked: traffic counts,
pods off, a helper off, another car, a map switch, stop. No CARLA.

  ../../carla-venv/bin/python tests/test_pipeline_control.py

ダッシュボードからのライブ操作の試験（全プロセスは偽物）。
"""
import os
import sys
import tempfile
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import pipeline_control as pc  # noqa: E402
import run_pipeline as rp  # noqa: E402
import windows_control as wc  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


TMP = tempfile.mkdtemp(prefix="pipeline_control_test_")
LAUNCHED, STOPPED, LOADS = [], [], []


class FakeProc(object):
    _pid = [70000]

    def __init__(self, name, cmd):
        FakeProc._pid[0] += 1
        self.pid, self.name, self.cmd, self.returncode = FakeProc._pid[0], name, cmd, None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = -15
        return self.returncode


# the dashboard's requests, one per supervision tick (time.sleep(1.0))
# 監視ループの 1 秒ごとにダッシュボードの依頼を 1 件
SCRIPT = {
    2: ("traffic", {"vehicles": 25, "walkers": 4, "cyclists": 0}),
    4: ("delivery", {"vans": 0}),
    6: ("helper", {"name": "ffb", "on": False}),
    8: ("vehicle", {"vehicle": "vehicle.toyota.supra_a70"}),
    10: ("map", {"map": "Town03"}),
    14: ("stop", {}),
}
TICKS = {"n": 0}
STATES = []


def fake_sleep(s):
    if s != 1.0:
        return
    TICKS["n"] += 1
    if TICKS["n"] in SCRIPT:
        a, kw = SCRIPT[TICKS["n"]]
        pc.request(pc.CONTROL_PATH, a, **kw)
    st = pc.read_json(pc.STATE_PATH)
    if st:
        STATES.append(st)
    if TICKS["n"] > 40:
        raise SystemExit("runaway loop")


def fake_signal(proc, sig):
    STOPPED.append(proc.name)
    proc.returncode = -sig


saved = {k: getattr(rp, k) for k in ("launch", "signal_group", "port_in_use", "world_map_name",
                                      "load_world_map", "reset_world_async", "subprocess")}
saved_sleep, saved_argv = rp.time.sleep, sys.argv
rp.launch = lambda cmd, cwd, name, env=None: (LAUNCHED.append((name, list(cmd))) or FakeProc(name, list(cmd)))
rp.signal_group = fake_signal
rp.port_in_use = lambda port: False
rp.world_map_name = lambda host, port, timeout=10.0: "OitaKosen"
rp.load_world_map = lambda host, port, name, timeout=120.0: LOADS.append(name) or True
rp.reset_world_async = lambda host, port, timeout=10.0: False
rp.subprocess = types.SimpleNamespace(
    call=lambda cmd, cwd=None: 0,
    run=lambda cmd, capture_output=True: types.SimpleNamespace(stdout=b""),
    TimeoutExpired=saved["subprocess"].TimeoutExpired)
rp.time.sleep = fake_sleep
real = {k: getattr(pc, k) for k in ("CONTROL_PATH", "STATE_PATH")}
pc.CONTROL_PATH = os.path.join(TMP, "control.json")
pc.STATE_PATH = os.path.join(TMP, "state.json")
real_win = (wc.CONTROL_PATH, wc.STATUS_PATH)
wc.CONTROL_PATH, wc.STATUS_PATH = os.path.join(TMP, "win_c.json"), os.path.join(TMP, "win_s.json")
real_req = rp.map_switch.REQUEST_PATH
rp.map_switch.REQUEST_PATH = os.path.join(TMP, "map_request.json")
try:
    sys.argv = ["run_pipeline.py", "--no-sim", "--map", "OitaKosen", "--vehicles", "10", "--walkers", "10",
                "--delivery", "3", "--ffb", "--no-sensor-viewer", "--drone-cams",
                "--start-vehicle", "vehicle.mazda.rx7", "--external-dashboard", "--dashboard-port", "5999"]
    rc = rp.main()
finally:
    for k, v in saved.items():
        setattr(rp, k, v)
    rp.time.sleep, sys.argv = saved_sleep, saved_argv
    for k, v in real.items():
        setattr(pc, k, v)
    wc.CONTROL_PATH, wc.STATUS_PATH = real_win
    rp.map_switch.REQUEST_PATH = real_req

names = [n for n, _ in LAUNCHED]
cmds = lambda n: [c for m, c in LAUNCHED if m == n]
print("=== live control / ライブ操作 ===")
check("no dashboard started (the launcher owns it)", "dashboard" not in names, names)
tr = cmds("generate_traffic")
check("traffic restarted with the new counts", len(tr) >= 2 and tr[1][tr[1].index("-n") + 1] == "25"
      and tr[1][tr[1].index("-w") + 1] == "4", [c[c.index("-n") + 1] for c in tr])
check("the old traffic was stopped first", "generate_traffic" in STOPPED)
check("pods off: the delivery sim stopped, not restarted", "delivery" in STOPPED and len(cmds("delivery")) == 1)
check("pods off: the drone camera wall closed", "drone_cams" in STOPPED)
check("force feedback off", "ffb" in STOPPED and len(cmds("ffb")) == 1)
mcs = cmds("manual_control")
check("driving window restarted with the new car", len(mcs) >= 2
      and mcs[1][mcs[1].index("--start-vehicle") + 1] == "vehicle.toyota.supra_a70")
check("map switch to Town03 like F5", LOADS == ["Town03"], LOADS)
check("after the switch the new car is kept", mcs[-1][mcs[-1].index("--start-vehicle") + 1] == "vehicle.toyota.supra_a70")
check("after the switch traffic keeps the new counts", tr[-1][tr[-1].index("-n") + 1] == "25")
check("after the switch no delivery sim (pods 0)", len(cmds("delivery")) == 1)
check("stop: the pipeline returns 0", rc == 0, rc)
phases = [s.get("phase") for s in STATES]
check("state file: running, then switching", "running" in phases and "switching" in phases, sorted(set(phases)))
mid = [s for s in STATES if s.get("phase") == "running" and s.get("vehicles") == 25]
check("state file shows the live values", bool(mid) and mid[-1]["delivery"] == 0 and mid[-1]["ffb"] is False
      and mid[-1]["launcher"] is True, mid[-1] if mid else STATES[-1:])
print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
