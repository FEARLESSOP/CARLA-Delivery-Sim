"""Nav brain before / after the no-fly detour training (2026-11-18): the 30 real CARLA
trips flown as LastMile missions (delivery/rl_bridge.py RLMission), 32 practice trips
that cross a no-fly zone and 32 normal practice trips, for each checkpoint given.

  ../../carla-venv/bin/python rl_delivery/research/nav_detour_check.py OLD.zip NEW.zip

飛行禁止区域の迂回訓練の前後比較（CARLA の実経路 30 本・迂回練習 32 本・通常 32 本）。
"""
import sys, os, json
from collections import Counter
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import warnings; warnings.filterwarnings("ignore")
from sb3_contrib import RecurrentPPO
from rl_delivery.env_nav import NavEnv
from delivery.rl_bridge import NavBridge

TRIPS = [((57.54, -488.5, 38.79), (261.12, -91.75)), ((57.54, -488.5, 38.79), (156.37, -235.12)),
         ((57.54, -488.5, 38.79), (125.54, -253.63)), ((404.32, 100.89, 58.83), (460.02, 223.2)),
         ((-435.98, 473.49, 84.89), (-473.35, 408.89)), ((-435.98, 473.49, 84.89), (-218.7, 566.01))]


def carla_trips(nav):
    c = Counter()
    for start, drop_xy in TRIPS:
        for rep in range(5):
            g = float(nav.heights.ground_at(*drop_xy)); mz = max(start[2] + 25.0, 70.0)
            f = nav.create_mission(start, (drop_xy[0], drop_xy[1], g), mz, start, mz, battery=100.0, drain=0.0667)
            t, res = 0.0, "timeout"
            while t < 900:
                evs = f.update(0.05); t += 0.05
                cr = [e for e in evs if e[0] == "crashed"]
                if cr: res = "crash_" + cr[0][1]; break
                if any(e[0] == "landed" for e in evs): res = "completed"; break
                if f.holding and f.land_fn is None: f.clear(lambda s=start: s)
            c[res] += 1
    return dict(c)


def practice(model, share, n=32):
    env = NavEnv("OitaKosen", wind_level=0.0, seed=99); env.curriculum_dist = 900.0; env.detour_share = share
    c = Counter()
    for k in range(n):
        obs, _ = env.reset(seed=500 + k)
        st, start = None, np.ones((1,), bool)
        while True:
            a, st = model.predict(obs, state=st, episode_start=start, deterministic=True); start = np.zeros((1,), bool)
            obs, r, term, trunc, info = env.step(a)
            if term or trunc: break
        s = env.stats
        c["completed" if s["reached"] else ("delivered_only" if s["delivered"] else "crash_%s" % (s["crash"] or "?"))] += 1
    return dict(c)


out = {}
for ck in sys.argv[1:]:
    nav = NavBridge(ck, map_name="OitaKosen", wind_level=1.0)
    out[os.path.basename(ck)] = {"carla_trips_30": carla_trips(nav), "detour_32": practice(nav.model, 1.0),
                                 "normal_32": practice(nav.model, 0.0)}
    print(os.path.basename(ck), json.dumps(out[os.path.basename(ck)]), flush=True)
json.dump(out, open(os.path.join(os.path.dirname(__file__), "nav_detour_check.json"), "w"), indent=1)
