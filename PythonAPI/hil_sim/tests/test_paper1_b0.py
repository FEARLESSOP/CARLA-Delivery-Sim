"""Tests of the Paper 1 world and baseline B0 (rl_delivery/paper1/). No CARLA.

1. B0 picks the feasible option with the lowest estimated ready time.
2. The feasibility check: battery reserve, a station out of service.
3. A full charger: the estimate includes a wait; an arriving drone queues.
4. One landing at a time on the pod's roof pad (a second drone waits).
5. Estimates never draw from the world's random generator (paired seeds).
6. Same seed -> identical episode (determinism).
7. Every scenario runs an episode with sensible results.

Paper 1 の世界と B0 のテスト（CARLA 不要）。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.paper1 import world as W1
from rl_delivery.paper1.methods import B0NearestFeasibleQueue
from rl_delivery.paper1.evaluate import run_episode


def fresh(scenario="S1_normal", seed=1000):
    w = W1.Paper1World(scenario=scenario, seed=seed)
    w.reset(seed=seed)
    return w


def airborne(w, batt=80.0):
    """Drone 0 just dropped a parcel half way between the pod and station 1."""
    d = w.drones[0]
    st = w.stations[1]["pads"][0]
    d.update(state=F.FLY, batt=batt, pod=0, pos=(w.pods[0]["pos"] + st) / 2.0)
    d["p1_drop_t"] = w.t
    return d


def test_b0_picks_lowest_ready_time():
    w = fresh()
    d = airborne(w)
    opt, info = B0NearestFeasibleQueue().decide(w, d)
    feas = [i for i, ok in enumerate(info["mask"]) if ok]
    scores = info["score_ready_s"]
    best = min(feas, key=lambda i: scores[i])
    assert info["candidates"][best] == ("pod" if opt[0] == "pod" else "station%d" % opt[1])
    assert not info["fallback"]
    print("1. B0 chose %s (scores %s)" % (info["candidates"][best], scores))


def test_feasibility():
    w = fresh("S4_station_outage")
    d = airborne(w, batt=16.0)
    est = [w.estimate(d, o) for o in w.options(d)]
    assert all(not e["feasible"] for e in est), "16 % cannot keep a 15 % reserve anywhere far"
    assert all(e["reason"] for e in est)
    w.t = 4000.0                                       # inside the outage window / 停止時間中
    d = airborne(w, batt=95.0)
    i = w.outage_station
    e = w.estimate(d, ("station", i))
    assert not e["feasible"] and e["reason"] == "station out of service"
    opt, info = B0NearestFeasibleQueue().decide(w, d)
    assert opt != ("station", i)
    print("2. reserve and outage masks: OK (outage station %d)" % i)


def test_charger_queue():
    w = fresh()
    st = w.stations[1]
    for _slot in range(len(st["slots"])):
        dd = w._new_drone(pod=w.pods[0])
        dd["batt"] = 0.0                                   # empty: a minute to charge / 充電に約 1 分
        w._join_station(dd, 1)
    d = airborne(w)
    d["pos"] = st["pads"][0].copy() + 5.0                  # right next to the station / 局のすぐ横
    full = w.estimate(d, ("station", 1))["wait_s"]
    assert full > 30.0, full
    w._join_station(d, 1)
    assert d["state"] == W1.STN_QUEUE and d["id"] in st["queue"] and w.p1["charger_conflicts"] == 1
    first = w.drones[st["slots"][0]]
    w._leave_station(first)
    assert d["state"] == W1.STN_CHARGE and d["id"] in st["slots"]
    print("3. full charger: queued, then took the freed pad (estimated wait %.0f s): OK" % full)


def test_one_landing_at_a_time():
    w = fresh()
    pod = w.pods[0]
    pod["drive"] = None
    a, b = w.drones[0], w.drones[1]
    for d in (a, b):
        d.update(state=F.HOVER, pod=0, pos=pod["pos"].copy(), hover_t=0.0, batt=60.0)
    w._drone_step(a)
    w._drone_step(b)
    assert a["state"] == F.DOCKED and b["state"] == F.HOVER and w.p1["pad_conflicts"] == 1
    w.t += F.LAND_S
    w._drone_step(b)
    assert b["state"] == F.DOCKED
    print("4. roof pad: one landing at a time: OK")


def test_estimate_does_not_draw_random():
    w = fresh()
    d = airborne(w)
    before = w.rng.bit_generator.state
    for o in w.options(d):
        w.estimate(d, o)
    B0NearestFeasibleQueue().decide(w, d)
    assert w.rng.bit_generator.state == before
    print("5. estimates leave the world's random generator untouched: OK")


def test_determinism():
    r1, _ = run_episode("B0", "S1_normal", 1003, log_decisions=False)
    r2, _ = run_episode("B0", "S1_normal", 1003, log_decisions=False)
    keys = ("orders", "delivered", "lead_mean_s", "ready_time_mean_s", "to_station", "recovery_failures")
    assert all(r1[k] == r2[k] for k in keys), [(k, r1[k], r2[k]) for k in keys]
    print("6. same seed -> same episode: OK (%d delivered)" % r1["delivered"])


def test_all_scenarios_run():
    for s in W1.SCENARIOS:
        row, decs = run_episode("B0", s, 1001)
        assert row["orders"] > 20 and row["delivered"] > 0 and row["recoveries"] == len(decs)
        assert all(r["chosen"] for r in decs)
        assert 0.0 <= row["recovery_failure_rate"] <= 1.0
    print("7. all %d scenarios run: OK" % len(W1.SCENARIOS))


if __name__ == "__main__":
    test_b0_picks_lowest_ready_time()
    test_feasibility()
    test_charger_queue()
    test_one_landing_at_a_time()
    test_estimate_does_not_draw_random()
    test_determinism()
    test_all_scenarios_run()
    print("ALL PAPER 1 / B0 TESTS PASSED")
