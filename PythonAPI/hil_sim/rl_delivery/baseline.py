"""Baseline: today's rule-based pods + drones on a map, fixed seeds, KPIs saved.

Starts its own headless CARLA (on the NVIDIA GPU), loads the map once, runs
delivery_sim.py once per seed and collects each run's summary["kpi"] into
rl_delivery/results/baseline_<Map>.json. Never run it while the demo pipeline
is up (two CARLAs crash the pipeline's server).

  python3 -m rl_delivery.baseline --map OitaKosen --seeds 1 2 3 4 5 --duration 1800

ルールベースの基準 KPI を固定シードで計測する（デモ停止中に実行）。
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
CARLA_ROOT = os.path.dirname(os.path.dirname(HIL))
PY = os.path.join(CARLA_ROOT, "carla-venv", "bin", "python")
sys.path.insert(0, HIL)
from run_pipeline import load_world_map, nvidia_offload_env  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="OitaKosen")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--duration", type=float, default=1800.0, help="sim seconds per run")
    ap.add_argument("--vans", type=int, default=3)
    ap.add_argument("--vehicles", type=int, default=9)
    ap.add_argument("--walkers", type=int, default=20)
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    env = dict(os.environ, **nvidia_offload_env("nvidia"))
    srv = subprocess.Popen([os.path.join(CARLA_ROOT, "CarlaUE4.sh"), "-RenderOffScreen", "-nosound",
                            "-carla-rpc-port=%d" % a.port], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    out = os.path.join(HERE, "results", "baseline_%s.json" % a.map)
    runs = []
    try:
        time.sleep(15)
        load_world_map("127.0.0.1", a.port, a.map)
        for seed in a.seeds:
            before = set(glob.glob(os.path.join(HIL, "data", "delivery", "*")))
            cmd = [PY, "delivery_sim.py", "--host", "127.0.0.1", "--port", str(a.port), "--map", "",
                   "--vans", str(a.vans), "--lmh", "0", "--vehicles", str(a.vehicles),
                   "--walkers", str(a.walkers), "--sync", "--speedup", "0", "--seed", str(seed),
                   "--duration", str(a.duration), "--first-now", "--follow", "none",
                   "--charge-stations", "4", "--charge-mode", "random"]
            t0 = time.time()
            rc = subprocess.call(cmd, cwd=HIL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            new = sorted(set(glob.glob(os.path.join(HIL, "data", "delivery", "*"))) - before)
            kpi = None
            if new and os.path.exists(os.path.join(new[-1], "summary.json")):
                with open(os.path.join(new[-1], "summary.json")) as f:
                    kpi = json.load(f).get("kpi")
            runs.append({"seed": seed, "rc": rc, "wall_s": round(time.time() - t0), "dir": new[-1] if new else None,
                         "kpi": kpi})
            print("seed %d rc %d in %.0f s" % (seed, rc, time.time() - t0), flush=True)
            with open(out, "w") as f:
                json.dump({"map": a.map, "duration_s": a.duration, "vans": a.vans, "runs": runs}, f, indent=1)
    finally:
        srv.terminate()
        try:
            srv.wait(30)
        except subprocess.TimeoutExpired:
            srv.kill()
        subprocess.call(["pkill", "-9", "-f", "CarlaUE4-Linux-Shipping.*-carla-rpc-port=%d" % a.port])
    print("saved", out)


if __name__ == "__main__":
    main()
