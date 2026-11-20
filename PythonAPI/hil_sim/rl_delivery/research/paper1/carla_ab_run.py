"""Paper 1 in CARLA: one run with a recovery method, one with the sim's own rule, same
settings (OitaKosen, 3 LastMile pods, --drone-prob 0.9 --seed 7), back to back.

  ../../carla-venv/bin/python rl_delivery/research/paper1/carla_ab_run.py --minutes 45

Relaunches the pipeline through the launcher, switches the recovery option live
(only the delivery helper restarts; CARLA keeps running), ends each run by the next
switch (the run writes summary.json on exit) and finally sets the pods to 0. Writes
carla_ab_<date>.json here with both runs' numbers and the drones' ready times.

CARLA で手法あり・既定ルールの 2 回を同じ設定で続けて走らせ、結果を比べる。
"""
import argparse
import csv
import glob
import json
import os
import re
import subprocess
import sys
import time

HIL = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, HIL)
import launch_options as L          # noqa: E402
import pipeline_control as pc       # noqa: E402

DELIV = os.path.join(HIL, "data", "delivery")
EXTRA = "--drone-prob 0.9 --seed 7"
# "typical" drone hardware of the Paper 1 v2 world (rl_delivery/paper1/world.py PROFILES):
# 25 min of level flight, station 2C (30 min), in-pod 1C (60 min) / 練習世界 v2 の機体
TYPICAL = "--battery-drain 0.0667 --charge-fast 0.0556 --charge-slow 0.0278"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def delivery_procs():
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout
    return [(int(l.split(None, 1)[0]), l.split(None, 1)[1]) for l in out.splitlines()[1:]
            if "delivery_sim.py" in l and "ps -eo" not in l]


def wait_for(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = pred()
        if v:
            return v
        time.sleep(3)
    raise SystemExit("timed out waiting for %s / 待機がタイムアウト" % what)


def newest_run(after):
    """The run folder created after ``after``, by the time in its name (a folder's
    mtime changes when the previous run writes its summary on exit).
    名前の時刻で選ぶ（前の実行が終了時に書くと mtime が変わるため）。"""
    runs = []
    for d in glob.glob(os.path.join(DELIV, "2*")):
        try:
            t = time.mktime(time.strptime(os.path.basename(d), "%Y%m%d_%H%M%S"))
        except ValueError:
            continue
        if t >= after - 5:
            runs.append((t, d))
    return max(runs)[1] if runs else None


def switch(method, after_pids):
    req = dict(vans=3, lmh=0, charge_stations=4, charge_mode="random", paper1_recovery=method, paper1_seed="0")
    pc.request(pc.CONTROL_PATH, "delivery", **req)
    t = time.time()
    want = (lambda a: "--paper1-recovery" not in a) if method == "off" else (lambda a: "--paper1-recovery %s" % method in a)
    procs = wait_for(lambda: [p for p in delivery_procs() if p[0] not in after_pids and want(p[1]) and EXTRA.split()[0] in p[1]],
                     240, "the delivery helper with %s" % method)
    time.sleep(20)
    return procs[0][0], newest_run(t)


def ready_times(run):
    """Drop -> landed in a pod, per drone delivery, from events.csv. / 投下から帰還まで"""
    order_drone, drop_t, pend, out = {}, {}, {}, []
    with open(os.path.join(run, "events.csv")) as f:
        for r in csv.DictReader(f):
            t, ev, det = float(r["sim_t"]), r["event"], r.get("detail", "")
            if ev == "drone_launch" and r.get("order"):
                m = re.match(r"D(\d+)", det)
                if m:
                    order_drone[r["order"]] = m.group(1)
            elif ev == "delivered" and r.get("kind") == "drone" and r.get("order") in order_drone:
                pend[order_drone[r["order"]]] = t
            elif ev == "drone_home":
                m = re.match(r"D(\d+)", det)
                if m and m.group(1) in pend:
                    out.append(t - pend.pop(m.group(1)))
    return out


def summarise(run):
    s = json.load(open(os.path.join(run, "summary.json")))
    rt = ready_times(run)
    h = max(1e-9, float(s.get("sim_seconds", 0.0)) / 3600.0)
    return {"run": os.path.basename(run), "sim_minutes": round(h * 60, 1),
            "delivered": s.get("delivered"), "delivered_per_h": round(s.get("delivered", 0) / h, 2),
            "delivered_drone": s.get("delivered_drone"), "drone_per_h": round((s.get("delivered_drone") or 0) / h, 2),
            "failed": s.get("failed"), "crashes": s.get("crashes"), "charge_trips": s.get("charge_trips"),
            "mean_wait_s": s.get("mean_wait_s"), "ready_n": len(rt),
            "ready_mean_s": round(sum(rt) / len(rt), 1) if rt else None,
            "paper1": s.get("paper1_recovery"), "kpi": s.get("kpi")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=45.0)
    ap.add_argument("--method", default="B4")
    ap.add_argument("--hardware", default="carla", choices=("carla", "typical"),
                    help="carla = the sim's own drone battery; typical = the hardware the Paper 1 methods learned on")
    ap.add_argument("--seed", type=int, default=7, help="the delivery sim's order seed (repeats use 8, 9, ...) / 注文の乱数シード")
    a = ap.parse_args()
    global EXTRA
    EXTRA = EXTRA.replace("--seed 7", "--seed %d" % a.seed)
    if a.hardware == "typical":
        EXTRA = EXTRA + " " + TYPICAL
    o = L.load_saved()
    o.update(map="OitaKosen", delivery=3, minipod_hubs=0, charge_mode="random", charge_stations=4,
             policy_nav="off", policy_drive="off", delivery_args=EXTRA)
    o = L.clean(o)
    old = {p for p, _ in delivery_procs()}
    pc.request(pc.LAUNCH_PATH, "relaunch", options=o)
    log("relaunch requested (3 pods, %s)" % EXTRA)
    wait_for(lambda: [p for p in delivery_procs() if p[0] not in old and EXTRA.split()[0] in p[1]], 600, "the relaunch")
    time.sleep(30)
    runs = {}
    for method in (a.method, "off"):
        pid, run = switch(method, {p for p, _ in delivery_procs()})
        log("run %s started: pid %d, folder %s" % (method, pid, run))
        t_end = time.time() + a.minutes * 60
        while time.time() < t_end:
            time.sleep(30)
            if pid not in {p for p, _ in delivery_procs()}:
                log("WARNING: the %s delivery helper stopped early" % method)
                break
        runs[method] = run
    pc.request(pc.CONTROL_PATH, "delivery", vans=0, lmh=0, charge_stations=4, charge_mode="random",
               paper1_recovery="off", paper1_seed="0")
    wait_for(lambda: not [p for p in delivery_procs() if EXTRA.split()[0] in p[1]], 240, "the last run to end")
    time.sleep(5)
    res = {m: summarise(r) for m, r in runs.items() if r and os.path.exists(os.path.join(r, "summary.json"))}
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "carla_ab_%s_%s%s.json" % (a.hardware, time.strftime("%Y%m%d_%H%M"),
                                                                                         "" if a.seed == 7 else "_seed%d" % a.seed))
    json.dump({"settings": {"map": "OitaKosen", "pods": 3, "extra": EXTRA, "hardware": a.hardware, "minutes": a.minutes,
                            "seed": a.seed},
               "runs": res},
              open(out, "w"), indent=1)
    log("DONE ->", out)
    for m, r in res.items():
        log(m, {k: r[k] for k in ("sim_minutes", "delivered_per_h", "drone_per_h", "failed", "crashes", "charge_trips",
                                  "ready_n", "ready_mean_s", "mean_wait_s")})


if __name__ == "__main__":
    main()
