"""Tests for the F5 map switch (map_switch.py, run_pipeline.py, the driving
window). No CARLA server: fakes stand in for the client, world and processes.

1. the map cycle: built from get_available_maps(), installed maps only, stable
   order, own maps (OitaKosen) included, _Opt / sublevels / test levels out;
2. next / previous map, wrap-around, a current map outside the cycle;
3. the request file: write / read / stale / corrupt / clear;
4. the pipeline side: which exit means "switch", stop order (traffic first,
   never CarlaUE4), the switch with its load_world -> CARLA-restart fallback;
5. the driving window: under the pipeline F5 writes the request (same hero
   model) and quits with MAP_SWITCH_RC instead of calling load_world;
   standalone it still loads the map itself.

F5 のマップ切替の試験（CARLA サーバー不要。クライアント・ワールド・プロセス
は偽物）。
"""
import importlib.util
import json
import os
import sys
import tempfile
import time
import types

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import map_switch as ms  # noqa: E402
import run_pipeline as rp  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


TMP = tempfile.mkdtemp(prefix="map_switch_test_")
REQ = os.path.join(TMP, "map_request.json")

# What CARLA 0.9.16 lists on this machine (Town06/07 are NOT installed).
# このマシンで CARLA が返す一覧（Town06/07 は未インストール）
AVAILABLE = [
    "/Game/Carla/Maps/Town10HD_Opt", "/Game/Carla/Maps/Town01", "/Game/Carla/Maps/Town01_Opt",
    "/Game/Carla/Maps/Town02", "/Game/Carla/Maps/Town02_Opt", "/Game/Carla/Maps/Town03",
    "/Game/Carla/Maps/Town03_Opt", "/Game/Carla/Maps/Town04", "/Game/Carla/Maps/Town05",
    "/Game/Carla/Maps/Town10HD", "/Game/Carla/Maps/OpenDriveMap",
    "/Game/Carla/Maps/TestMaps/EmptyMap",
    "/Game/Carla/Maps/Sublevels/Town10HD_Opt/T10HD_Props",
    "/Game/Japan/Maps/OitaStation/OitaStation", "/Game/Japan/Maps/OitaKosen/OitaKosen",
    "/Game/Singapore/Maps/PasirRis/PasirRis",
    "/Game/Circuits/Maps/SpaFrancorchamps/SpaFrancorchamps",
    "/Game/Circuits/Maps/MarinaBay/MarinaBay",
]
EXPECTED = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town10HD",
            "MarinaBay", "SpaFrancorchamps", "OitaKosen", "OitaStation", "PasirRis"]

print("\n=== 1. the map cycle ===")
cyc = ms.build_cycle(AVAILABLE)
check("cycle = installed towns, then own maps by package", cyc == EXPECTED, str(cyc))
check("OitaKosen is in the cycle", "OitaKosen" in cyc)
check("not-installed Town06/07 never appear", "Town06" not in cyc and "Town07" not in cyc)
check("order does not depend on the server's listing order",
      ms.build_cycle(list(reversed(AVAILABLE))) == cyc)
check("paths without /Game (client form) work too",
      ms.build_cycle([p[len("/Game/"):] for p in AVAILABLE]) == cyc)
check("empty listing -> empty cycle", ms.build_cycle([]) == [] and ms.build_cycle(None) == [])

fake_content = os.path.join(TMP, "Content")
for rel in ("Carla/Maps/Town02.umap", "Carla/Maps/Town02_Opt.umap",
            "Carla/Maps/Sublevels/Town10HD_Opt/T10HD_Props.umap",
            "Japan/Maps/OitaKosen/OitaKosen.umap", "Japan/Maps/OitaKosen/OitaKosen.uasset"):
    path = os.path.join(fake_content, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").close()
inst = ms.installed_maps(fake_content)
check("installed_maps lists .umap files as /Game paths",
      "/Game/Japan/Maps/OitaKosen/OitaKosen" in inst and "/Game/Carla/Maps/Town02" in inst, str(inst))
check("installed_maps feeds build_cycle", ms.build_cycle(inst) == ["Town02", "OitaKosen"],
      str(ms.build_cycle(inst)))
real_content = os.path.join(os.path.dirname(os.path.dirname(HIL)), "CarlaUE4", "Content")
if os.path.isdir(real_content):
    real = ms.build_cycle(ms.installed_maps(real_content))
    check("this machine's installed maps include OitaKosen", "OitaKosen" in real, str(real))

print("\n=== 2. next / previous ===")
check("F5 from OitaKosen -> OitaStation (no longer Town01)",
      ms.step(cyc, "OitaKosen", 1) == "OitaStation")
check("Shift+F5 from OitaKosen -> SpaFrancorchamps", ms.step(cyc, "OitaKosen", -1) == "SpaFrancorchamps")
check("F5 from the last map wraps to Town01", ms.step(cyc, "PasirRis", 1) == "Town01")
check("Shift+F5 from Town01 wraps to the last map", ms.step(cyc, "Town01", -1) == "PasirRis")
check("an _Opt / full path current map is recognised",
      ms.step(cyc, "Carla/Maps/Town10HD_Opt", 1) == "MarinaBay")
check("current map outside the cycle: F5 -> first, Shift+F5 -> last",
      ms.step(cyc, "EmptyMap", 1) == "Town01" and ms.step(cyc, "EmptyMap", -1) == "PasirRis")
check("empty cycle -> None", ms.step([], "Town01", 1) is None)

print("\n=== 3. the request file ===")
ms.write_request("OitaStation", hero="vehicle.mazda.rx7", current="OitaKosen", path=REQ)
r = ms.read_request(REQ)
check("request round-trips map, hero and from",
      r is not None and r["map"] == "OitaStation" and r["hero"] == "vehicle.mazda.rx7"
      and r["from"] == "OitaKosen", str(r))
check("no temp file left behind (atomic write)", not os.path.exists(REQ + ".tmp"))
check("a stale request is ignored", ms.read_request(REQ, now=time.time() + 600) is None)
with open(REQ, "w") as f:
    f.write("{not json")
check("a corrupt request is ignored", ms.read_request(REQ) is None)
with open(REQ, "w") as f:
    json.dump({"map": "  ", "time": time.time()}, f)
check("a request without a map is ignored", ms.read_request(REQ) is None)
ms.clear_request(REQ)
check("clear_request removes it", not os.path.exists(REQ) and ms.read_request(REQ) is None)
ms.clear_request(REQ)
check("clear_request on nothing is fine", True)

print("\n=== 4. the pipeline side ===")
check("normal quit (rc 0, no request) -> no switch", rp.take_map_request(0, REQ) is None)
check("MAP_SWITCH_RC without a request -> no switch (shut down)",
      rp.take_map_request(ms.MAP_SWITCH_RC, REQ) is None)
ms.write_request("Town03", hero="vehicle.tesla.model3", current="OitaKosen", path=REQ)
r = rp.take_map_request(ms.MAP_SWITCH_RC, REQ)
check("MAP_SWITCH_RC + request -> switch to its map with its hero",
      r is not None and r["map"] == "Town03" and r["hero"] == "vehicle.tesla.model3", str(r))
check("the request is consumed", not os.path.exists(REQ))
ms.write_request("Town03", path=REQ)
check("a fresh request counts even if the window then died (rc -11)",
      rp.take_map_request(-11, REQ) is not None)
ms.write_request("Town03", path=REQ)
check("a stale request does not switch",
      rp.take_map_request(ms.MAP_SWITCH_RC, REQ, now=time.time() + 600) is None
      and not os.path.exists(REQ))

names = ["CarlaUE4", "generate_traffic", "dashboard", "delivery", "drone_cams", "ffb",
         "traction_fx", "manual_control"]
groups = rp.switch_stop_groups(names)
check("stop order: traffic first, then the rest", groups[0] == ["generate_traffic"]
      and set(groups[1]) == set(names) - {"CarlaUE4", "generate_traffic"}, str(groups))
check("CarlaUE4 is never stopped by a switch", all("CarlaUE4" not in g for g in groups))
check("no traffic -> one group", rp.switch_stop_groups(["CarlaUE4", "dashboard"]) == [["dashboard"]])


def run_switch(load_ok, restart_ok):
    log = []
    how = rp.switch_map(
        "OitaStation",
        lambda: log.append("stop"),
        lambda: log.append("async") or False,
        lambda m: log.append("load:" + m) or load_ok,
        lambda m: log.append("restart:" + m) or restart_ok)
    return how, log


how, log = run_switch(True, True)
check("switch: stop -> async -> load, no CARLA restart",
      how == "loaded" and log == ["stop", "async", "load:OitaStation"], "%s %s" % (how, log))
how, log = run_switch(False, True)
check("load_world fails -> CARLA restarted on that map",
      how == "restarted" and log[-1] == "restart:OitaStation", "%s %s" % (how, log))
how, log = run_switch(False, False)
check("both fail -> 'failed'", how == "failed", how)

print("\n=== 5. the driving window ===")
import carla as real_carla  # noqa: E402
import pygame  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "mc", os.path.join(HIL, "manual_control_steeringwheel G923.py"))
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)


class FakeHUD(object):
    def __init__(self):
        self.msgs, self.errors = [], []

    def notification(self, text, seconds=2.0):
        self.msgs.append(text)

    def error(self, text):
        self.errors.append(text)

    def on_world_tick(self, ts):
        pass


class FakeCarlaWorld(object):
    def __init__(self, name):
        self.name = name

    def get_map(self):
        return types.SimpleNamespace(name=self.name)

    def on_tick(self, cb):
        pass


class FakeClient(object):
    def __init__(self, available):
        self.available = available
        self.loaded = []

    def get_available_maps(self):
        if self.available is None:
            raise RuntimeError("timeout")
        return list(self.available)

    def load_world(self, name):
        self.loaded.append(name)
        return FakeCarlaWorld("Carla/Maps/" + name)


def make_world(map_request, available=AVAILABLE, current="Japan/Maps/OitaKosen/OitaKosen"):
    w = object.__new__(mc.World)            # skip __init__ (it spawns a car) / 生成を省く
    w.client = FakeClient(available)
    w.world = FakeCarlaWorld(current)
    w.hud = FakeHUD()
    w.player = types.SimpleNamespace(type_id="vehicle.mazda.rx7")
    w._model_id = "vehicle.mazda.rx7"
    w.map_request = map_request
    w.exit_code = 0
    w.restarts = []
    w.restart = lambda step=1: w.restarts.append(step)
    return w


w = make_world(REQ)
quit_ = w.change_map(1)
r = ms.read_request(REQ)
check("pipeline mode: F5 returns True (close the window)", quit_ is True)
check("pipeline mode: exit code is MAP_SWITCH_RC", w.exit_code == ms.MAP_SWITCH_RC, str(w.exit_code))
check("pipeline mode: no load_world from the window", w.client.loaded == [], str(w.client.loaded))
check("pipeline mode: request = next map, same hero model",
      r is not None and r["map"] == "OitaStation" and r["hero"] == "vehicle.mazda.rx7"
      and r["from"] == "OitaKosen", str(r))
check("pipeline mode: 'Switching to ...' shown", any("Switching to OitaStation" in m for m in w.hud.msgs),
      str(w.hud.msgs))
ms.clear_request(REQ)

w = make_world(REQ)
w.change_map(-1)
check("Shift+F5 asks for the previous map", (ms.read_request(REQ) or {}).get("map") == "SpaFrancorchamps")
ms.clear_request(REQ)

w = make_world(REQ, available=None)
w.change_map(1)
check("server cannot list maps -> Content folder / built-in list still give a target",
      (ms.read_request(REQ) or {}).get("map") not in (None, "OitaKosen"), str(ms.read_request(REQ)))
ms.clear_request(REQ)

w = make_world(None)
quit_ = w.change_map(1)
check("standalone: loads the map itself and keeps running",
      w.client.loaded == ["OitaStation"] and quit_ is False and w.exit_code == 0
      and w.restarts == [0], "%s %s %s" % (w.client.loaded, quit_, w.restarts))
check("standalone: no request file written", not os.path.exists(REQ))

# F5 through the action dispatcher: True = quit the game loop
# 操作ディスパッチャ経由の F5：True でゲームループ終了
fake_ctrl = types.SimpleNamespace(_control=real_carla.VehicleControl())
fw = types.SimpleNamespace(camera_manager=None, change_map=lambda off: True)
check("_do_action('map_next') quits when change_map asks to",
      mc.DualControl._do_action(fake_ctrl, "map_next", fw) is True)
fw.change_map = lambda off: False
check("_do_action('map_prev') keeps running in standalone mode",
      mc.DualControl._do_action(fake_ctrl, "map_prev", fw) is False)
fw.change_map = lambda off: None
check("_do_action tolerates change_map returning None",
      mc.DualControl._do_action(fake_ctrl, "map_next", fw) is False)

check("--map-request is a driving-window option",
      "--map-request" in open(os.path.join(HIL, "manual_control_steeringwheel G923.py")).read())

print("\n=== 6. run_pipeline.main end to end, all processes faked ===")
# Nothing here touches the real CARLA, processes or ports: launch, signals,
# subprocess and the CARLA probes are all replaced. / 実プロセスには触れない
LAUNCHED, SIGNALS = [], []
MC_RUNS = {"n": 0}


class FakeProc(object):
    _pid = [90000]

    def __init__(self, name, cmd):
        FakeProc._pid[0] += 1
        self.pid, self.name, self.cmd = FakeProc._pid[0], name, cmd
        self.returncode = None
        if name == "manual_control":
            MC_RUNS["n"] += 1
            if MC_RUNS["n"] == 1:       # first window: driver presses F5 / F5 を押す
                path = cmd[cmd.index("--map-request") + 1]
                ms.write_request("OitaStation", hero="vehicle.mazda.rx7",
                                 current="OitaKosen", path=path)
                self.returncode = ms.MAP_SWITCH_RC
            else:                       # second window: driver quits / 終了
                self.returncode = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = -15
        return self.returncode


saved = {k: getattr(rp, k) for k in ("launch", "signal_group", "port_in_use", "world_map_name",
                                      "load_world_map", "reset_world_async", "subprocess")}
saved_sleep, saved_argv = rp.time.sleep, sys.argv
LOADS = []
rp.launch = lambda cmd, cwd, name, env=None: (LAUNCHED.append((name, list(cmd))) or
                                              FakeProc(name, list(cmd)))
rp.signal_group = lambda proc, sig: (SIGNALS.append((proc.name, sig)),
                                     setattr(proc, "returncode", -sig))
rp.port_in_use = lambda port: False
rp.world_map_name = lambda host, port, timeout=10.0: "OitaKosen"
rp.load_world_map = lambda host, port, name, timeout=120.0: LOADS.append(name) or True
rp.reset_world_async = lambda host, port, timeout=10.0: False
fake_sub = types.SimpleNamespace(
    call=lambda cmd, cwd=None: LOADS.append("config.py:" + cmd[-1]) or 0,
    run=lambda cmd, capture_output=True: types.SimpleNamespace(stdout=b""),
    TimeoutExpired=saved["subprocess"].TimeoutExpired)
rp.subprocess = fake_sub
rp.time.sleep = lambda s: None
real_req = ms.REQUEST_PATH
ms.REQUEST_PATH = REQ
# keep the dashboard Windows-card files of a live pipeline untouched
# 動作中のパイプラインのウィンドウ開閉ファイルに触れない
import windows_control as wc  # noqa: E402
real_win = (wc.CONTROL_PATH, wc.STATUS_PATH)
wc.CONTROL_PATH = os.path.join(TMP, "pipeline_control.json")
wc.STATUS_PATH = os.path.join(TMP, "pipeline_status.json")
try:
    sys.argv = ["run_pipeline.py", "--no-sim", "--map", "OitaKosen", "--vehicles", "10",
                "--walkers", "10", "--delivery", "3", "--ffb", "--no-sensor-viewer",
                "--start-vehicle", "vehicle.mazda.rx7", "--res", "1080p", "--drone-cams",
                "--dashboard-port", "5999"]
    # the default arg was bound at import; point the helper at the temp file
    # 既定値は import 時に固定されるため一時ファイルを指す版に差し替える
    take = rp.take_map_request
    rp.take_map_request = lambda rc, path=None, now=None: take(rc, REQ, now)
    rc = rp.main()
finally:
    for k, v in saved.items():
        setattr(rp, k, v)
    rp.time.sleep, sys.argv = saved_sleep, saved_argv
    rp.take_map_request = take
    ms.REQUEST_PATH = real_req
    wc.CONTROL_PATH, wc.STATUS_PATH = real_win

names = [n for n, _ in LAUNCHED]
first, second = names[:names.index("manual_control") + 1], names[names.index("manual_control") + 1:]
check("pipeline exits 0 after the second window quits", rc == 0, str(rc))
check("startup map via config.py, the switch via load_world_map",
      LOADS == ["config.py:OitaKosen", "OitaStation"], str(LOADS))
# the real signal timing runs only on maps with maps.json "signal_timing" (OitaKosen)
# 信号時間は maps.json に "signal_timing" があるマップ（OitaKosen）だけ
check("every client started again after the switch (same set, less the timing)",
      sorted(n for n in first if n != "signal_timing") == sorted(second), "%s | %s" % (first, second))
check("signal timing on OitaKosen only", "signal_timing" in first and "signal_timing" not in second)
check("CarlaUE4 never launched (no second server)", "CarlaUE4" not in names)
mcs = [c for n, c in LAUNCHED if n == "manual_control"]
check("both windows get --map-request", all("--map-request" in c for c in mcs))
check("hero model kept after the switch",
      mcs[1][mcs[1].index("--start-vehicle") + 1] == "vehicle.mazda.rx7")
dash = [c for n, c in LAUNCHED if n == "dashboard"]
check("dashboard keeps its port", len(dash) == 2 and dash[0] == dash[1], str(dash))
deliv = [c for n, c in LAUNCHED if n == "delivery"][1]
check('delivery_sim keeps --map "" and --passive after the switch',
      deliv[deliv.index("--map") + 1] == "" and "--passive" in deliv)
traffic = [c for n, c in LAUNCHED if n == "generate_traffic"]
check("left-hand OitaStation: traffic without --hybrid", "--hybrid" not in traffic[1])
stopped = [n for n, sig in SIGNALS if sig == rp.signal.SIGTERM]
check("switch stopped traffic before the other clients",
      stopped and stopped[0] == "generate_traffic", str(stopped))
check("request consumed", not os.path.exists(REQ))

pygame.quit()
print("\n%d checks failed" % len(FAILS))
for f in FAILS:
    print("  - " + f)
sys.exit(1 if FAILS else 0)
