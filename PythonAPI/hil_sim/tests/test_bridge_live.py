"""Live verification script for the RL CARLA Bridge on OitaKosen.

Launches a headless CARLA instance on the RTX 5080 (if not already running),
loads OitaKosen, runs delivery_sim.py with --policy-nav latest for 1 batch / 120s,
and verifies autonomous neural flight & landing in CARLA.
"""
import glob
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL_SIM = os.path.dirname(HERE) if os.path.basename(HERE) == "tests" else HERE
CARLA_ROOT = os.path.dirname(os.path.dirname(HIL_SIM))
PY = os.path.join(CARLA_ROOT, "carla-venv", "bin", "python")

if HIL_SIM not in sys.path:
    sys.path.insert(0, HIL_SIM)

from run_pipeline import load_world_map, nvidia_offload_env


def run_live_test(map_name="OitaKosen", duration_s=160.0, port=2000):
    env = dict(os.environ, **nvidia_offload_env("nvidia"))

    # Check if CARLA is already running on this port
    check_srv = subprocess.run(["pgrep", "-f", "CarlaUE4.*-carla-rpc-port=%d" % port],
                               stdout=subprocess.PIPE).stdout.strip()
    started_srv = False
    srv = None

    if not check_srv:
        print("[Live Bridge] Starting headless CARLA on port %d with RTX 5080 offload..." % port)
        srv_cmd = [
            os.path.join(CARLA_ROOT, "CarlaUE4.sh"),
            "-RenderOffScreen",
            "-nosound",
            "-carla-rpc-port=%d" % port
        ]
        srv = subprocess.Popen(srv_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        started_srv = True
        print("[Live Bridge] Waiting 12s for CARLA initialization...")
        time.sleep(12)
    else:
        print("[Live Bridge] CARLA server already running on port %d." % port)

    try:
        print("[Live Bridge] Loading map %s..." % map_name)
        load_world_map("127.0.0.1", port, map_name)

        before = set(glob.glob(os.path.join(HIL_SIM, "data", "delivery", "*")))
        cmd = [
            PY, "delivery_sim.py",
            "--host", "127.0.0.1",
            "--port", str(port),
            "--map", "",
            "--vans", "1",
            "--lmh", "0",
            "--vehicles", "0",
            "--walkers", "0",
            "--sync",
            "--speedup", "0",
            "--duration", str(duration_s),
            "--first-now",
            "--follow", "none",
            "--charge-stations", "1",
            "--policy-nav", "latest",
            "--policy-recovery", "ramppo"
        ]

        print("[Live Bridge] Running delivery_sim with --policy-nav latest:")
        print("  Command:", " ".join(cmd))
        t0 = time.time()
        p = subprocess.Popen(cmd, cwd=HIL_SIM, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        for line in p.stdout:
            sys.stdout.write("  [sim] " + line)
            sys.stdout.flush()

        rc = p.wait()
        wall_s = round(time.time() - t0, 1)
        print("[Live Bridge] delivery_sim exited with rc=%d in %.1f s" % (rc, wall_s))

        new = sorted(set(glob.glob(os.path.join(HIL_SIM, "data", "delivery", "*"))) - before)
        summary = None
        if new and os.path.exists(os.path.join(new[-1], "summary.json")):
            with open(os.path.join(new[-1], "summary.json")) as f:
                summary = json.load(f)
            print("[Live Bridge] Run summary:", json.dumps(summary.get("kpi", {}), indent=2))

        return rc == 0
    finally:
        if started_srv and srv is not None:
            print("[Live Bridge] Terminating test CARLA server...")
            srv.terminate()
            try:
                srv.wait(10)
            except subprocess.TimeoutExpired:
                srv.kill()
            subprocess.call(["pkill", "-9", "-f", "CarlaUE4-Linux-Shipping.*-carla-rpc-port=%d" % port])


if __name__ == "__main__":
    success = run_live_test()
    sys.exit(0 if success else 1)
