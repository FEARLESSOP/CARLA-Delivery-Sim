"""Record one episode of a brain for watch mode (dashboard Training page).

  python3 -m rl_delivery.watch --brain flight --checkpoint latest --wind 1.0 --seed 7
  python3 -m rl_delivery.watch --brain flight --checkpoint pid      # the PID autopilot

Writes rl_delivery/results/watch/<name>.json: the path, every 0.1 s the drone
position, the hidden wind (for the viewer only), the brain's command, the
path error and guard overrides, plus the episode KPIs. Same seed = same path
and same wind, so checkpoints can be compared on one flight.

ウォッチモード用に 1 エピソードを記録する（同じシードなら同じ経路と風）。
"""
import argparse
import json
import math
import os

import numpy as np

from rl_delivery.train import CKPT, RES, latest

OUT = os.path.join(RES, "watch")


def record_flight(checkpoint, wind, seed, map_name="OitaKosen"):
    from rl_delivery.env_flight import FlightEnv, PidPilot
    env = FlightEnv(map_name, wind_level=wind, seed=seed)
    obs, _ = env.reset(seed=seed)
    env.log = []
    if checkpoint == "pid":
        pilot, model = PidPilot(env), None
    else:
        from sb3_contrib import RecurrentPPO
        path = latest("flight") if checkpoint in ("latest", "", None) else checkpoint
        if not path:
            raise SystemExit("no flight checkpoint yet")
        model = RecurrentPPO.load(path, device="cpu")
        checkpoint = path
    state, start = None, np.ones((1,), bool)
    while True:
        if model is None:
            act = pilot.act(obs)
        else:
            act, state = model.predict(obs, state=state, episode_start=start, deterministic=True)
            start = np.zeros((1,), bool)
        obs, _r, te, tr, info = env.step(act)
        if te or tr:
            break
    frames = env.log[1::2]                      # 10 Hz / 10 Hz に間引き
    xs = [f["p"][0] for f in frames] + env.path[:, 0].tolist()
    ys = [f["p"][1] for f in frames] + env.path[:, 1].tolist()
    box = [min(xs) - 120, min(ys) - 120, max(xs) + 120, max(ys) + 120]
    near = [b for b in env.m["buildings"] if box[0] <= b["x"] <= box[2] and box[1] <= b["y"] <= box[3]]
    return {"brain": "flight", "checkpoint": os.path.basename(str(checkpoint)), "wind_level": wind,
            "seed": seed, "path": env.path.round(2).tolist(), "frames": frames, "kpi": info["kpi"],
            "box": box, "nofly": env.nofly, "zones": env.m["zones"]["pods"],
            "buildings": [[b["x"], b["y"], b["len"], b["wid"], b["yaw"], b["h"]] for b in near]}


def record_mission(checkpoint, seed, map_name="OitaKosen", pod=None):
    """One hour of one pod zone: pod, 4 drones, orders, wind and rain every 10 s."""
    from rl_delivery.env_mission import MissionEnv, RuleMission
    env = MissionEnv(map_name, seed=seed, pod=pod)
    obs, _ = env.reset(seed=seed)
    env.log = []
    if checkpoint == "rule":
        pol, model = RuleMission(env), None
    else:
        from sb3_contrib import MaskablePPO
        path = latest("mission") if checkpoint in ("latest", "", None) else checkpoint
        if not path:
            raise SystemExit("no mission checkpoint yet")
        model = MaskablePPO.load(path, device="cpu")
        checkpoint = path
    events = []
    while True:
        pend = env.pending
        if model is None:
            act = pol.act(obs)
        else:
            act, _ = model.predict(obs, action_masks=env.action_masks(), deterministic=True)
        if pend is not None:
            names = ["launch low", "launch medium", "launch high", "wait", "return: pod here",
                     "return: next stop", "return: stop after"]
            events.append({"t": env.t, "drone": pend[0]["id"], "what": names[int(act)],
                           "batt": round(pend[0]["batt"], 1)})
        obs, _r, te, tr, info = env.step(act)
        if te or tr:
            break
    z = env.zone
    box = [z["x0"] - 150, z["y0"] - 150, z["x1"] + 150, z["y1"] + 150]
    lines = [l for l in env.m["lines"] if any(box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3] for p in l)]
    return {"brain": "mission", "checkpoint": os.path.basename(str(checkpoint)), "seed": seed, "pod": env.vid,
            "frames": env.log, "events": events, "kpi": info["kpi"], "box": box, "zone": z,
            "nofly": env.m["zones"]["nofly"], "zones": env.m["zones"]["pods"],
            "lines": [[[round(x), round(y)] for x, y in l[::2] + [l[-1]]] for l in lines]}


def record_nav(checkpoint, wind, seed, map_name="OitaKosen", dist=900.0):
    """One delivery flight of the navigation brain, with the height map around it."""
    from rl_delivery.env_nav import NavEnv, ScriptedPilot
    env = NavEnv(map_name, wind_level=wind, seed=seed)
    env.curriculum_dist = dist
    obs, _ = env.reset(seed=seed)
    env.log = []
    if checkpoint == "scripted":
        pol, model = ScriptedPilot(env), None
    else:
        from sb3_contrib import RecurrentPPO
        path = latest("nav") if checkpoint in ("latest", "", None) else checkpoint
        if not path:
            raise SystemExit("no nav checkpoint yet")
        model = RecurrentPPO.load(path, device="cpu")
        checkpoint = path
        # the trip length this checkpoint practised (curriculum) / このチェックポイントの練習距離
        meta = path[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                env.curriculum_dist = float(json.load(f).get("dist", dist))
            obs, _ = env.reset(seed=seed)
    state, start = None, np.ones((1,), bool)
    start_p = env.drone.p.copy()
    while True:
        if model is None:
            act = pol.act(obs)
        else:
            act, state = model.predict(obs, state=state, episode_start=start, deterministic=True)
            start = np.zeros((1,), bool)
        obs, _r, te, tr, info = env.step(act)
        if te or tr:
            break
    frames = env.log
    xs = [f["p"][0] for f in frames] + [start_p[0], env.drop_goal[0], env.meet_goal[0]]
    ys = [f["p"][1] for f in frames] + [start_p[1], env.drop_goal[1], env.meet_goal[1]]
    # a square view of at least 300 m round the flight / 飛行を囲む正方形（300 m 以上）
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    half = max(150.0, (max(xs) - min(xs)) / 2.0 + 60.0, (max(ys) - min(ys)) / 2.0 + 60.0)
    box = [cx - half, cy - half, cx + half, cy + half]
    # the height map around the flight (4 m cells): ground and the top of anything on it
    # 飛行範囲の高さ地図（4 m 格子）
    H, cell = env.H, 4.0
    gx = np.arange(box[0], box[2], cell)
    gy = np.arange(box[1], box[3], cell)
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    top, gnd = H.top_at(X, Y), H.ground_at(X, Y)
    return {"brain": "nav", "checkpoint": os.path.basename(str(checkpoint)), "wind_level": wind, "seed": seed,
            "start": start_p.round(2).tolist(), "goal": env.drop_goal.round(2).tolist(),
            "meet": env.meet_goal.round(2).tolist(), "frames": frames,
            "kpi": info["kpi"], "box": box, "nofly": env.nofly, "zones": env.m["zones"]["pods"],
            "cell": cell, "nx": len(gx), "ny": len(gy),
            "above": np.clip(np.round(top - gnd), 0, 99).astype(int).ravel().tolist(),
            "ground": np.round(gnd, 1).ravel().tolist()}


def record_fleet(pod_ck, drone_ck, seed, map_name="OitaKosen", hours=1.0):
    """One hour of the whole fleet: 3 pods, every drone (standbys too), orders,
    stations, lockers, every 5 s, plus the event log. / フリート全体の 1 時間"""
    from sb3_contrib import MaskablePPO
    from rl_delivery.env_fleet import PodView
    from rl_delivery.fleet_rules import rule_pod

    def load(ck, brain):
        if ck == "rule":
            return None, "rule"
        path = latest(brain) if ck in ("latest", "", None) else ck
        if not path:
            return None, "rule (no %s checkpoint yet)" % brain
        return MaskablePPO.load(path, device="cpu"), os.path.basename(path)
    pm, pname = load(pod_ck, "fleet_pod")
    dm, dname = load(drone_ck, "fleet_drone")
    other = (lambda o, m: int(dm.predict(o, action_masks=m, deterministic=False)[0])) if dm else None
    env = PodView(map_name, seed=seed, other=other)
    obs, _ = env.reset(seed=seed)
    W = env.W
    W.log, W.events = [], []
    while W.t < hours * 3600.0:
        if pm is not None:
            a = int(pm.predict(obs, action_masks=env.action_masks(), deterministic=False)[0])
        else:
            a = rule_pod(W, env.cur[1])
        obs, _r, te, tr, info = env.step(a)
        if te or tr:
            break
    m = W.m
    return {"brain": "fleet", "pod_checkpoint": pname, "drone_checkpoint": dname, "seed": seed,
            "frames": W.log, "events": W.events[-2000:], "kpi": W.kpi(), "bounds": m["bounds"],
            "zones": m["zones"]["pods"], "nofly": m["zones"]["nofly"],
            "stations": [[round(float(st["xy"][0]), 1), round(float(st["xy"][1]), 1)] for st in W.stations],
            "lockers": [[round(float(L["xy"][0]), 1), round(float(L["xy"][1]), 1)] for L in W.lockers],
            "lines": [[[round(x), round(y)] for x, y in l[::3] + [l[-1]]] for l in m["lines"]],
            "checkpoint": pname}


def record_drive(checkpoint, seed, map_name="OitaKosen"):
    """One driving run: the route with its lane edges (kerb / lane / centre line),
    stop lines and lights, the other cars and the pod, every 0.2 s.
    運転 1 回：経路・縁の種類・停止線と信号・他車・ポッド"""
    from rl_delivery.env_drive import DriveEnv, ScriptedDriver
    env = DriveEnv(map_name, seed=seed)
    model = None
    if checkpoint == "scripted":
        name = "scripted"
    else:
        from stable_baselines3 import SAC
        path = latest("drive") if checkpoint in ("latest", "", None) else checkpoint
        if not path:
            raise SystemExit("no drive checkpoint yet")
        model = SAC.load(path, device="cpu")
        name = os.path.basename(path)
        meta = path[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                j = json.load(f)
            env.route_len, env.traffic = float(j.get("route_len", 150.0)), int(j.get("traffic", 0))
    obs, _ = env.reset(seed=seed)
    env.log = []
    pol = ScriptedDriver(env) if model is None else None
    while True:
        act = pol.act(obs) if model is None else model.predict(obs, deterministic=True)[0]
        obs, _r, te, tr, info = env.step(act)
        if te or tr:
            break
    P = env.P
    edges = []
    for q in P[::2]:
        h, half = q[2], q[3] / 2.0
        nx_, ny_ = -math.sin(h), math.cos(h)
        edges.append([round(q[0] + nx_ * half, 2), round(q[1] + ny_ * half, 2), int(q[4]),
                      round(q[0] - nx_ * half, 2), round(q[1] - ny_ * half, 2), int(q[5]), int(q[7])])
    marks = []
    for m in env.marks:
        q = env._at(m[0])
        marks.append({"xy": [round(float(q[0]), 2), round(float(q[1]), 2)], "yaw": round(float(q[2]), 3),
                      "w": round(float(q[3]), 2), "kind": m[1]})
    frames = []
    for f in env.log:
        f = dict(f)
        f["cars"] = [[round(float(env._at(cs)[0]), 2), round(float(env._at(cs)[1]), 2),
                      round(float(env._at(cs)[2]), 3)] for cs in f["cars"]]
        frames.append(f)
    return {"brain": "drive", "checkpoint": name, "seed": seed, "route_len": env.route_len,
            "traffic": env.traffic, "frames": frames, "kpi": info["kpi"], "edges": edges, "marks": marks,
            "limit": [round(float(q[6]), 0) for q in P[::5]]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brain", default="flight", choices=["flight", "mission", "nav", "fleet", "drive"])
    ap.add_argument("--drone-checkpoint", default="latest")
    ap.add_argument("--checkpoint", default="latest")
    ap.add_argument("--wind", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--map", default="OitaKosen")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    if a.brain == "fleet":
        rec = record_fleet(a.checkpoint, a.drone_checkpoint, a.seed, a.map)
    elif a.brain == "drive":
        rec = record_drive(a.checkpoint, a.seed, a.map)
    elif a.brain == "mission":
        rec = record_mission(a.checkpoint, a.seed, a.map)
    elif a.brain == "nav":
        rec = record_nav(a.checkpoint, a.wind, a.seed, a.map)
    else:
        rec = record_flight(a.checkpoint, a.wind, a.seed, a.map)
    os.makedirs(OUT, exist_ok=True)
    path = a.out or os.path.join(OUT, "%s_%s_w%.1f_s%d.json" % (
        a.brain, rec["checkpoint"].replace(".zip", ""), a.wind, a.seed))
    with open(path, "w") as f:
        # numpy bools / floats from the env -> plain JSON / numpy 値を JSON 用に変換
        json.dump(rec, f, separators=(",", ":"),
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(json.dumps({"file": path, "frames": len(rec["frames"])}))


if __name__ == "__main__":
    main()
