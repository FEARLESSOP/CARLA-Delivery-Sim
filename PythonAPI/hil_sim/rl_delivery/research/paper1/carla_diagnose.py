"""Why does B4 choose chargers so often in CARLA? Reads a delivery run's
paper1_decisions.json (each decision with its snapshot and what B4 saw), compares
charger and pod decisions, and replays the saved snapshots offline with B0 / B2 / B4.

  ../../carla-venv/bin/python rl_delivery/research/paper1/carla_diagnose.py data/delivery/<run>

CARLA で B4 が充電器を多く選ぶ理由を、保存した判断（スナップショット付き）から調べる。
"""
import json
import math
import os
import statistics as st
import sys
import warnings

HIL = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, HIL)
warnings.filterwarnings("ignore")


def feats(r):
    s = r["snapshot"]
    d = next(x for x in s["drones"] if x["state"] == "dropped")
    pod = s["pod"]
    meet = pod.get("meet") or pod["pos"]
    return {"drone_batt": d["batt"], "to_meet_m": math.dist(d["pos"], meet),
            "to_next_m": math.dist(d["pos"], pod["next_stop"]) if pod.get("next_stop") else None,
            "pod_batt": pod["batt"], "pod_driving": pod.get("driving_to") is not None,
            "orders": len(s.get("orders", [])), "docked": sum(x["state"] == "docked" for x in s["drones"]),
            "away": sum(x["state"] in ("deliver", "return", "hover") for x in s["drones"]) - 0,
            "t_min": s["t"] / 60.0}


def summ(rows, key):
    v = [r[key] for r in rows if r.get(key) is not None]
    if not v:
        return "-"
    if isinstance(v[0], bool):
        return "%d/%d" % (sum(v), len(v))
    return "%.1f (%.1f-%.1f)" % (st.median(v), min(v), max(v))


def main():
    run = sys.argv[1]
    recs = [r for r in json.load(open(os.path.join(run, "paper1_decisions.json"))) if r.get("snapshot")]
    print("%d decisions with snapshots in %s" % (len(recs), run))
    groups = {}
    for r in recs:
        groups.setdefault(r["choice"], []).append(dict(feats(r), rec=r))
    keys = ["drone_batt", "to_meet_m", "to_next_m", "pod_batt", "pod_driving", "orders", "docked", "away", "t_min"]
    print("\nmedian (min-max) per choice:")
    print("  %-10s %-4s " % ("choice", "n") + " ".join("%-18s" % k for k in keys))
    for c, rows in sorted(groups.items(), key=lambda x: str(x[0])):
        print("  %-10s %-4d " % (c, len(rows)) + " ".join("%-18s" % summ(rows, k) for k in keys))
    print("\nwhat B4 saw (first 8 charger decisions):")
    for f in groups.get("station", [])[:8]:
        r = f["rec"]
        sn = r.get("seen") or {}
        print("  t=%.0f D%s batt %.0f%% pod %.0f%% | options %s | feasible %s | est. ready %s | prior %s | q %s" % (
            r["t"], r["drone"], f["drone_batt"], f["pod_batt"], sn.get("candidates"),
            sn.get("mask"), sn.get("score_ready_s"), sn.get("prior"), r.get("q")))
    # offline replay / オフライン再生
    from rl_delivery.paper1.snapshot import SnapshotPlanner
    planners = {}
    agree = {}
    for m in ("B0", "B2", "B4"):
        for f in [x for rows in groups.values() for x in rows]:
            r = f["rec"]
            z = r["van"] if r["van"] in (1, 2, 3) else 1
            key = (m, z)
            if key not in planners:
                planners[key] = SnapshotPlanner(m, 0, zone=z)
            a = planners[key].decide(r["snapshot"])
            agree.setdefault(m, []).append((r["choice"], a["kind"]))
    print("\noffline replay of the CARLA snapshots (choice counts):")
    for m, pairs in agree.items():
        from collections import Counter
        c = Counter(k for _c, k in pairs)
        same = sum(1 for a, b in pairs if a == b)
        print("  %s: %s | same as live B4: %d/%d" % (m, dict(c), same, len(pairs)))


if __name__ == "__main__":
    main()
