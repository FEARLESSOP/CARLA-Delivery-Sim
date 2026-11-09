#!/usr/bin/env python3
"""Offline test of the launcher (launcher.py) with a stand-in pipeline: launch,
running state, stop, relaunch with new options, a crashed run, and the
options -> flags mapping (launch_options.py). No CARLA.

  ../../carla-venv/bin/python tests/test_launcher.py

ランチャーのオフラインテスト（代用のパイプラインで起動・停止・再起動）。
"""

import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import pipeline_control as pc  # noqa: E402

TMP = tempfile.mkdtemp()
for name in ("CONTROL_PATH", "STATE_PATH", "LAUNCH_PATH", "LAUNCHER_STATE_PATH", "LOG_PATH"):
    setattr(pc, name, os.path.join(TMP, os.path.basename(getattr(pc, name))))

import launch_options  # noqa: E402
launch_options.SAVED_PATH = os.path.join(TMP, "launch_options.json")
import launcher  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, (" | %s" % (detail,)) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


# a stand-in pipeline: writes the running state like run_pipeline.py, records its
# argv, exits on SIGINT (rc 0) or by itself with rc 3 when --map Crash
# 代用のパイプライン
FAKE = os.path.join(TMP, "fake_pipeline.py")
with open(FAKE, "w") as fh:
    fh.write('''import json, os, signal, sys, time
state, argv_out = %r, %r
open(argv_out, "a").write(" ".join(sys.argv[1:]) + "\\n")
stop = {"now": False}
signal.signal(signal.SIGINT, lambda *a: stop.__setitem__("now", True))
if any("Crash" in a for a in sys.argv):
    time.sleep(0.5); sys.exit(3)
while not stop["now"]:
    json.dump({"phase": "running", "pid": os.getpid(), "t": time.time()}, open(state + ".tmp", "w"))
    os.replace(state + ".tmp", state)
    time.sleep(0.2)
print("fake pipeline stopping"); sys.exit(0)
''' % (pc.STATE_PATH, os.path.join(TMP, "argv.txt")))


def run_until(L, cond, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        L.step()
        if cond():
            return True
        time.sleep(0.1)
    return False


L = launcher.Launcher(port=5999)
L.script = FAKE
st = lambda: pc.read_json(pc.LAUNCHER_STATE_PATH) or {}

print("1. launch / 起動")
L.step()
check("idle at start", st().get("phase") == "idle", st().get("phase"))
pc.request(pc.LAUNCH_PATH, "launch", options={"map": "Town03", "vehicles": 25, "ffb": False})
check("starting then running", run_until(L, lambda: st().get("phase") == "running"), st().get("phase"))
argv = open(os.path.join(TMP, "argv.txt")).read().split()
check("flags from the options", argv[argv.index("--map") + 1] == "Town03" and argv[argv.index("--vehicles") + 1] == "25"
      and "--ffb" not in argv and "--external-dashboard" in argv, " ".join(argv))
check("options saved for next time", launch_options.load_saved()["vehicles"] == 25)
check("the log has the command line", "run_pipeline.py" in open(pc.LOG_PATH).read())

print("2. a second launch while running is ignored / 実行中の起動依頼は無視")
pid = L.child.pid
pc.request(pc.LAUNCH_PATH, "launch", options={"map": "Town05"})
run_until(L, lambda: False, timeout=0.6)
check("same child", L.child is not None and L.child.pid == pid)

print("3. relaunch with new options / 新しい設定で再起動")
pc.request(pc.LAUNCH_PATH, "relaunch", options={"map": "Town04", "vehicles": 5})
check("a new child with the new map", run_until(L, lambda: L.child is not None and L.child.pid != pid
                                                 and st().get("phase") == "running"), st())
check("its flags", open(os.path.join(TMP, "argv.txt")).read().strip().splitlines()[-1].split("--map ")[1].startswith("Town04"))

print("4. stop / 停止")
pc.request(pc.LAUNCH_PATH, "stop")
check("back to idle, exit recorded", run_until(L, lambda: st().get("phase") == "idle"), st())
check("rc 0 after a clean stop", (st().get("last_exit") or {}).get("rc") == 0, st().get("last_exit"))

print("5. a run that crashes / 異常終了")
pc.request(pc.LAUNCH_PATH, "launch", options={"map": "Crash"})
check("idle again with its exit code", run_until(L, lambda: (st().get("last_exit") or {}).get("rc") == 3), st().get("last_exit"))

L.shutdown()
check("shutdown marks the launcher off", st().get("phase") == "off")

print("6. charging stations -> flags / 充電ステーションの引数")
d = launch_options.defaults()
check("defaults: random, 0 (off)", d["charge_mode"] == "random" and d["charge_stations"] == 0)
a = launch_options.to_argv({"charge_mode": "manual", "charge_stations": 5})
check("to_argv passes both to run_pipeline", a[a.index("--charge-mode") + 1] == "manual"
      and a[a.index("--charge-stations") + 1] == "5", " ".join(a))
c = launch_options.clean({"charge_mode": "nowhere", "charge_stations": "99"})
check("clean: unknown mode -> random, count capped at 12", c["charge_mode"] == "random" and c["charge_stations"] == 12, c)
import run_pipeline  # noqa: E402
ap = run_pipeline.main.__code__.co_consts      # the flags exist in run_pipeline's parser / 引数が定義済み
check("run_pipeline knows --charge-stations / --charge-mode",
      "--charge-stations" in ap and "--charge-mode" in ap)
print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
