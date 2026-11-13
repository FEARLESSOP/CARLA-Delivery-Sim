"""Train a brain in the pretend world.

  python3 -m rl_delivery.train --brain flight --map OitaKosen --hours 3
  python3 -m rl_delivery.train --brain flight --resume          # from the latest checkpoint

flight: RecurrentPPO (LSTM memory: it must feel the hidden wind from its own
motion), 16 parallel worlds, wind curriculum (calm -> full as it improves).
Checkpoints every ~100k steps in rl_delivery/checkpoints/<brain>/, progress
(reward + KPIs per evaluation) in rl_delivery/results/<brain>_progress.csv.
mission / pod: added next (docs/RL_DELIVERY.md).

疑似環境でブレインを学習する。チェックポイントと進捗 CSV を保存。
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")       # one thread per world process / 1 プロセス 1 スレッド
os.environ.setdefault("MKL_NUM_THREADS", "1")
import argparse
import csv
import glob
import json
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CKPT = os.path.join(HERE, "checkpoints")
RES = os.path.join(HERE, "results")
STATE = os.path.join(RES, "train_state.json")      # read by the dashboard / ダッシュボード用


def make_flight(map_name, seed):
    def _f():
        from rl_delivery.env_flight import FlightEnv
        return FlightEnv(map_name, wind_level=0.0, seed=seed)
    return _f


def evaluate_flight(model, map_name, level, n=12, seed=12345):
    """KPIs of the current policy over n flights at a wind level. / 評価"""
    from rl_delivery.env_flight import FlightEnv
    env = FlightEnv(map_name, wind_level=level, seed=seed)
    res = []
    for k in range(n):
        obs, _ = env.reset(seed=seed + k)
        state, start = None, np.ones((1,), bool)
        while True:
            act, state = model.predict(obs, state=state, episode_start=start, deterministic=True)
            start = np.zeros((1,), bool)
            obs, _r, te, tr, info = env.step(act)
            if te or tr:
                res.append(info["kpi"])
                break
    return {"wind_level": level, "reached": float(np.mean([r["reached"] for r in res])),
            "crash": float(np.mean([r["crash"] for r in res])),
            "xtrack_mean": float(np.mean([r["xtrack_mean"] for r in res])),
            "xtrack_max": float(np.max([r["xtrack_max"] for r in res])),
            "guard_steps": int(sum(r["guard_steps"] for r in res)),
            "battery_used": float(np.mean([r["battery_used"] for r in res]))}


NAV_ALGOS = ("recurrent_ppo", "ppo", "sac", "td3")
DRIVE_ALGOS = ("sac", "ppo", "td3", "recurrent_ppo")


def algo_class(algo):
    from sb3_contrib import RecurrentPPO as _R
    from stable_baselines3 import PPO as _P, SAC as _S, TD3 as _T
    return {"recurrent_ppo": _R, "ppo": _P, "sac": _S, "td3": _T}[algo]


def new_model(algo, venv, seed, gamma):
    """The algorithms compared in the paper, with their settings here in one place
    (also saved in each run's config). / 比較するアルゴリズムと設定"""
    cls = algo_class(algo)
    common = dict(device="cpu", verbose=0, seed=seed, gamma=gamma)
    if algo == "recurrent_ppo":
        return cls("MlpLstmPolicy", venv, n_steps=512, batch_size=2048, n_epochs=5, learning_rate=3e-4,
                   gae_lambda=0.95, ent_coef=0.0, policy_kwargs={"lstm_hidden_size": 192, "net_arch": [256, 256]},
                   **common)
    if algo == "ppo":
        return cls("MlpPolicy", venv, n_steps=1024, batch_size=2048, n_epochs=8, learning_rate=3e-4,
                   gae_lambda=0.95, ent_coef=0.0, policy_kwargs={"net_arch": [256, 256]}, **common)
    if algo == "sac":
        return cls("MlpPolicy", venv, buffer_size=1_000_000, batch_size=256, learning_starts=20000, train_freq=1,
                   gradient_steps=1, tau=0.005, learning_rate=3e-4, policy_kwargs={"net_arch": [256, 256]}, **common)
    if algo == "td3":
        return cls("MlpPolicy", venv, buffer_size=1_000_000, batch_size=256, learning_starts=20000, train_freq=1,
                   gradient_steps=1, learning_rate=3e-4, policy_kwargs={"net_arch": [256, 256]}, **common)
    raise ValueError(algo)


def latest(brain):
    files = glob.glob(os.path.join(CKPT, brain, "step_*.zip"))
    return max(files, key=os.path.getmtime) if files else None


def write_state(**kw):
    os.makedirs(RES, exist_ok=True)
    kw["t"] = time.time()
    with open(STATE + ".tmp", "w") as f:
        json.dump(kw, f)
    os.replace(STATE + ".tmp", STATE)


def train_flight(a):
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    import torch
    torch.set_num_threads(4)                          # the learner itself / 学習本体
    os.makedirs(os.path.join(CKPT, "flight"), exist_ok=True)
    venv = VecMonitor(SubprocVecEnv([make_flight(a.map, 1000 + i) for i in range(a.envs)]))
    resume = latest("flight") if a.resume else None
    level = 0.0
    if resume:
        model = RecurrentPPO.load(resume, env=venv, device="cpu")
        meta = resume[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                level = json.load(f).get("wind_level", 0.0)
    else:
        model = RecurrentPPO("MlpLstmPolicy", venv, n_steps=512, batch_size=2048, n_epochs=5,
                             learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, ent_coef=0.0,
                             clip_range=0.2, device="cpu", verbose=0,
                             policy_kwargs={"lstm_hidden_size": 128, "net_arch": [128, 128]})
    venv.env_method("__setattr__", "wind_level", level)
    prog = os.path.join(RES, "flight_progress.csv")
    new_file = not os.path.exists(prog) or not resume
    pf = open(prog, "w" if new_file else "a", newline="")
    pw = csv.writer(pf)
    if new_file:
        pw.writerow(["steps", "wall_s", "wind_level", "reached", "crash", "xtrack_mean", "xtrack_max",
                     "guard_steps", "battery_used", "ep_reward"])
    t0 = time.time()
    deadline = t0 + a.hours * 3600.0

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = model.num_timesteps + a.eval_every

        def _on_step(self):
            nonlocal level
            if self.num_timesteps >= self.next_eval:
                self.next_eval += a.eval_every
                k = evaluate_flight(self.model, a.map, level)
                rew = [e["r"] for e in self.model.ep_info_buffer] if self.model.ep_info_buffer else [0]
                pw.writerow([self.num_timesteps, round(time.time() - t0), round(level, 2), k["reached"], k["crash"],
                             round(k["xtrack_mean"], 2), round(k["xtrack_max"], 1), k["guard_steps"],
                             round(k["battery_used"], 1), round(float(np.mean(rew)), 1)])
                pf.flush()
                path = os.path.join(CKPT, "flight", "step_%09d.zip" % self.num_timesteps)
                self.model.save(path)
                with open(path[:-4] + ".json", "w") as f:
                    json.dump(dict(k, steps=self.num_timesteps), f)
                # curriculum: more wind once it flies well / 上達したら風を強く
                if k["reached"] >= 0.8 and k["xtrack_mean"] < 3.0 and level < 1.0:
                    level = min(1.0, level + 0.2)
                    self.training_env.env_method("__setattr__", "wind_level", level)
                write_state(brain="flight", running=True, steps=self.num_timesteps, wind_level=level,
                            last=k, checkpoint=path, started=t0, deadline=deadline)
                print("[train] %d steps  wind %.1f  reached %.0f%%  xtrack %.2f m  crash %.0f%%" % (
                    self.num_timesteps, level, 100 * k["reached"], k["xtrack_mean"], 100 * k["crash"]), flush=True)
            return time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP"))

    write_state(brain="flight", running=True, steps=model.num_timesteps, wind_level=level, started=t0,
                deadline=deadline)
    try:
        model.learn(total_timesteps=10 ** 10, callback=Cb(), reset_num_timesteps=not resume)
    finally:
        path = os.path.join(CKPT, "flight", "step_%09d.zip" % model.num_timesteps)
        model.save(path)
        with open(path[:-4] + ".json", "w") as f:
            json.dump({"steps": model.num_timesteps, "wind_level": level}, f)
        write_state(brain="flight", running=False, steps=model.num_timesteps, wind_level=level, checkpoint=path)
        pf.close()
        venv.close()


def make_nav(map_name, seed):
    def _f():
        from rl_delivery.env_nav import NavEnv
        return NavEnv(map_name, wind_level=0.0, seed=seed)
    return _f


def evaluate_nav(model, map_name, level, dist, n=16, seed=4242):
    """Reach / crash rates over n deliveries at a wind level and distance cap. / 航法の評価"""
    from rl_delivery.env_nav import NavEnv
    env = NavEnv(map_name, wind_level=level, seed=seed)
    env.curriculum_dist = dist
    res = []
    for k in range(n):
        obs, _ = env.reset(seed=seed + k)
        state, start = None, np.ones((1,), bool)
        while True:
            act, state = model.predict(obs, state=state, episode_start=start, deterministic=True)
            start = np.zeros((1,), bool)
            obs, _r, te, tr, info = env.step(act)
            if te or tr:
                res.append(info["kpi"])
                break
    crash = [r["crash"] for r in res]
    return {"wind_level": level, "dist": dist, "reached": float(np.mean([r["reached"] for r in res])),
            "delivered": float(np.mean([r.get("delivered", False) for r in res])),
            "crash_pod": float(np.mean([c == "pod" for c in crash])),
            "crash": float(np.mean([bool(c) and c != "timeout" for c in crash])),
            "crash_obstacle": float(np.mean([c == "obstacle" for c in crash])),
            "crash_nofly": float(np.mean([c == "nofly" for c in crash])),
            "crash_ceiling": float(np.mean([c == "ceiling" for c in crash])),
            "crash_battery": float(np.mean([c == "battery" for c in crash])),
            "timeout": float(np.mean([c == "timeout" for c in crash])),
            "min_clear": float(np.mean([r["min_clear"] for r in res])),
            "battery_used": float(np.mean([r["battery_used"] for r in res])),
            "time_s": float(np.mean([r["time_s"] for r in res]))}


NAV_COLS = ["steps", "wall_s", "wind_level", "dist", "reached", "delivered", "crash_pod", "crash", "crash_obstacle", "crash_nofly",
            "crash_ceiling", "crash_battery",
            "timeout", "min_clear", "battery_used", "time_s", "ep_reward"]


def train_nav(a):
    import torch
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    torch.set_num_threads(4)
    os.makedirs(os.path.join(CKPT, "nav"), exist_ok=True)
    venv = VecMonitor(SubprocVecEnv([make_nav(a.map, 3000 + 1000 * a.seed + i) for i in range(a.envs)]))
    resume = latest("nav") if a.resume else None
    level, dist = 0.0, 80.0
    algo = a.algo or "recurrent_ppo"
    if algo not in NAV_ALGOS:
        raise SystemExit("nav --algo: %s" % "/".join(NAV_ALGOS))
    if resume:
        model = algo_class(algo).load(resume, env=venv, device="cpu")
        meta = resume[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                j = json.load(f)
            level, dist = j.get("wind_level", 0.0), j.get("dist", 200.0)
    else:
        model = new_model(algo, venv, a.seed, gamma=0.995)
    if getattr(a, "dist", 0.0) > 0.0:
        dist = float(a.dist)
    if getattr(a, "wind_level", None) is not None:
        level = float(a.wind_level)

    def setc():
        venv.env_method("__setattr__", "wind_level", level)
        venv.env_method("__setattr__", "curriculum_dist", dist)
    setc()
    prog = os.path.join(RES, "nav_progress.csv")
    new_file = not os.path.exists(prog) or not resume
    pf = open(prog, "w" if new_file else "a", newline="")
    pw = csv.writer(pf)
    if new_file:
        pw.writerow(NAV_COLS)
    t0 = time.time()
    deadline = t0 + a.hours * 3600.0
    sfile = os.path.join(RES, "train_state_nav.json")

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = model.num_timesteps + a.eval_every

        def _on_step(self):
            nonlocal level, dist
            if self.num_timesteps >= self.next_eval:
                self.next_eval += a.eval_every
                k = evaluate_nav(self.model, a.map, level, dist)
                rew = [e["r"] for e in self.model.ep_info_buffer] if self.model.ep_info_buffer else [0]
                pw.writerow([self.num_timesteps, round(time.time() - t0)] + [round(k[c], 3) for c in NAV_COLS[2:-1]]
                            + [round(float(np.mean(rew)), 1)])
                pf.flush()
                path = os.path.join(CKPT, "nav", "step_%09d.zip" % self.num_timesteps)
                self.model.save(path)
                with open(path[:-4] + ".json", "w") as f:
                    json.dump(dict(k, steps=self.num_timesteps), f)
                # curriculum: longer trips first, then wind / まず距離、次に風
                if k["reached"] >= 0.7:
                    if dist < 900.0:
                        dist = min(900.0, dist + (60.0 if dist < 200 else 150.0))
                    elif level < 1.0:
                        level = min(1.0, level + 0.2)
                    setc()
                with open(sfile, "w") as f:
                    json.dump({"brain": "nav", "running": True, "steps": self.num_timesteps, "wind_level": level,
                               "dist": dist, "last": k, "checkpoint": path, "started": t0, "deadline": deadline,
                               "t": time.time()}, f)
                print("[nav] %d steps  dist %.0f  wind %.1f  delivered %.0f%%  landed on pod %.0f%%  crash %.0f%% (obstacle %.0f%%, no-fly %.0f%%, "
                      "ceiling %.0f%%, battery %.0f%%)  time-out %.0f%%" % (self.num_timesteps, dist, level, 100 * k["delivered"], 100 * k["reached"],
                                                           100 * k["crash"], 100 * k["crash_obstacle"],
                                                           100 * k["crash_nofly"], 100 * k["crash_ceiling"],
                                                           100 * k["crash_battery"], 100 * k["timeout"]), flush=True)
            return time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP_nav"))

    try:
        model.learn(total_timesteps=10 ** 10, callback=Cb(), reset_num_timesteps=not resume)
    finally:
        path = os.path.join(CKPT, "nav", "step_%09d.zip" % model.num_timesteps)
        model.save(path)
        with open(path[:-4] + ".json", "w") as f:
            json.dump({"steps": model.num_timesteps, "wind_level": level, "dist": dist}, f)
        with open(sfile, "w") as f:
            json.dump({"brain": "nav", "running": False, "steps": model.num_timesteps, "checkpoint": path,
                       "t": time.time()}, f)
        pf.close()
        venv.close()


def make_drive(map_name, seed):
    def _f():
        from rl_delivery.env_drive import DriveEnv
        return DriveEnv(map_name, seed=seed)
    return _f


DRIVE_COLS = ["steps", "wall_s", "route_len", "traffic", "reached", "kerb", "collision", "stop_violations",
              "red_violations", "centre_s", "speeding_s", "speed", "p10_speed", "ep_reward"]


def evaluate_drive(model, map_name, route_len, traffic, n=16, seed=8080):
    from rl_delivery.env_drive import DriveEnv
    env = DriveEnv(map_name, seed=seed)
    env.route_len, env.traffic = route_len, traffic
    res = []
    for k in range(n):
        obs, _ = env.reset(seed=seed + k)
        while True:
            act, _ = model.predict(obs, deterministic=True)
            obs, _r, te, tr, info = env.step(act)
            if te or tr:
                res.append(info["kpi"])
                break
    return {"route_len": route_len, "traffic": traffic,
            "reached": float(np.mean([r["reached"] for r in res])),
            "kerb": float(np.mean([r["end"] == "kerb" for r in res])),
            "collision": float(np.mean([r["end"] == "collision" for r in res])),
            "stalls": float(np.mean([r.get("stalls", 0) > 0 for r in res])),
            "stop_violations": float(np.mean([r["stop_violation"] for r in res])),
            "red_violations": float(np.mean([r["red_violation"] for r in res])),
            "centre_s": float(np.mean([r["centre_s"] for r in res])),
            "speeding_s": float(np.mean([r["speeding_s"] for r in res])),
            "speed": float(np.mean([r["progress"] / max(r["time_s"], 1.0) for r in res])),
            "p10_speed": float(np.mean([r.get("p10_speed", r["progress"] / max(r["time_s"], 1.0)) for r in res]))}


def train_drive(a):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    from stable_baselines3 import SAC
    torch.set_num_threads(2)
    os.makedirs(os.path.join(CKPT, "drive"), exist_ok=True)
    venv = VecMonitor(SubprocVecEnv([make_drive(a.map, 7000 + 1000 * a.seed + i) for i in range(a.envs)]))
    resume = latest("drive") if a.resume else None
    route_len, traffic = 150.0, 0
    algo = a.algo or "sac"
    if algo not in DRIVE_ALGOS:
        raise SystemExit("drive --algo: %s" % "/".join(DRIVE_ALGOS))
    if resume:
        model = algo_class(algo).load(resume, env=venv, device="cpu")
        meta = resume[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                j = json.load(f)
            route_len, traffic = j.get("route_len", 150.0), j.get("traffic", 0)
    else:
        # SAC: tanh-squashed actions (PPO's clipped Gaussian pinned at full brake)
        # SAC（PPO は出力が端に張り付いた）
        model = new_model(algo, venv, a.seed, gamma=0.99)

    def setc():
        venv.env_method("__setattr__", "route_len", route_len)
        venv.env_method("__setattr__", "traffic", traffic)
    setc()
    prog = os.path.join(RES, "drive_progress.csv")
    new_file = not os.path.exists(prog) or not resume
    pf = open(prog, "w" if new_file else "a", newline="")
    pw = csv.writer(pf)
    if new_file:
        pw.writerow(DRIVE_COLS)
    t0 = time.time()
    deadline = t0 + a.hours * 3600.0
    sfile = os.path.join(RES, "train_state_drive.json")

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = model.num_timesteps + a.eval_every

        def _on_step(self):
            nonlocal route_len, traffic
            if self.num_timesteps >= self.next_eval:
                self.next_eval += a.eval_every
                k = evaluate_drive(self.model, a.map, route_len, traffic)
                rew = [e["r"] for e in self.model.ep_info_buffer] if self.model.ep_info_buffer else [0]
                pw.writerow([self.num_timesteps, round(time.time() - t0)] + [round(k[c], 3) for c in DRIVE_COLS[2:-1]]
                            + [round(float(np.mean(rew)), 1)])
                pf.flush()
                path = os.path.join(CKPT, "drive", "step_%09d.zip" % self.num_timesteps)
                self.model.save(path)
                with open(path[:-4] + ".json", "w") as f:
                    json.dump(dict(k, steps=self.num_timesteps), f)
                # curriculum: longer routes, then more traffic, only when lawful and safe / 合法かつ安全なら難しく
                clean_runs = k["reached"] >= 0.75
                safe_runs = (k.get("kerb", 0.0) + k.get("collision", 0.0) + k.get("stalls", 0.0)) <= 0.10
                lawful_runs = (k["stop_violations"] + k["red_violations"]) < 0.10
                if clean_runs and safe_runs and lawful_runs:
                    if route_len < 800.0:
                        route_len = min(800.0, route_len + 100.0)
                    elif traffic < 6:
                        traffic += 1
                    setc()
                with open(sfile, "w") as f:
                    json.dump({"brain": "drive", "running": True, "steps": self.num_timesteps, "route_len": route_len,
                               "traffic": traffic, "last": k, "checkpoint": path, "started": t0,
                               "deadline": deadline, "t": time.time()}, f)
                print("[drive] %d steps  route %.0f m  traffic %d  reached %.0f%%  kerb %.0f%%  collision %.0f%%  "
                      "stop viol %.2f  red viol %.2f  centre %.1f s  speed %.1f m/s" % (
                          self.num_timesteps, route_len, traffic, 100 * k["reached"], 100 * k["kerb"],
                          100 * k["collision"], k["stop_violations"], k["red_violations"], k["centre_s"],
                          k["speed"]), flush=True)
            return time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP_drive"))

    try:
        model.learn(total_timesteps=10 ** 10, callback=Cb(), reset_num_timesteps=not resume)
    finally:
        path = os.path.join(CKPT, "drive", "step_%09d.zip" % model.num_timesteps)
        model.save(path)
        with open(path[:-4] + ".json", "w") as f:
            json.dump({"steps": model.num_timesteps, "route_len": route_len, "traffic": traffic}, f)
        with open(sfile, "w") as f:
            json.dump({"brain": "drive", "running": False, "steps": model.num_timesteps, "t": time.time()}, f)
        pf.close()
        venv.close()


def make_fleet(me, map_name, seed):
    def _f():
        from sb3_contrib.common.wrappers import ActionMasker
        from rl_delivery.env_fleet import PodView, DroneView
        other = os.path.join(CKPT, "fleet_drone" if me == "pod" else "fleet_pod")
        env = (PodView if me == "pod" else DroneView)(map_name, seed=seed, other_dir=other)
        return ActionMasker(env, lambda e: e.action_masks())
    return _f


FLEET_KEYS = ("deliveries_per_h", "f2f_per_h", "drone_per_h", "lead_mean_s", "crashes_per_100", "landing_min",
              "hover_s_per_h", "swaps", "recoveries", "replacements", "stranded", "charges", "takeoff_crash",
              "waiting", "deliveries_at_30", "waiting_at_30", "stranded_at_30", "deliveries_at_72", "waiting_at_72",
              "stranded_at_72")


def evaluate_fleet(pod_path=None, drone_path=None, map_name="OitaKosen", n=4, seed=555, rate=47.0):
    """Whole-fleet KPIs with given brains (None = rules) over n 3-hour episodes at a
    fixed order rate (47/h = the CARLA delivery sim). / 固定の注文数で評価"""
    from sb3_contrib import MaskablePPO
    from rl_delivery.env_fleet import PodView
    from rl_delivery.fleet_rules import rule_pod
    pm = MaskablePPO.load(pod_path, device="cpu") if pod_path else None
    dm = MaskablePPO.load(drone_path, device="cpu") if drone_path else None
    # sampled, like in training: early on the single most likely action is not the
    # brain's behaviour / 学習時と同じく確率的に選ぶ
    other = (lambda o, m: int(dm.predict(o, action_masks=m, deterministic=False)[0])) if dm else None
    res = []
    for k in range(n):
        env = PodView(map_name, seed=seed + k, other=other)
        env.W.rate_fixed = rate
        obs, _ = env.reset(seed=seed + k)
        while True:
            if pm is not None:
                a = int(pm.predict(obs, action_masks=env.action_masks(), deterministic=False)[0])
            else:
                a = rule_pod(env.W, env.cur[1])
            obs, _r, te, tr, info = env.step(a)
            if te or tr:
                res.append(info["kpi"])
                break
    out = {}
    for key in FLEET_KEYS:
        v = [r[key] for r in res if r.get(key) is not None]
        out[key] = float(np.mean(v)) if v else None
    return out


def train_fleet(a):
    """Pod and drone brains in turns: each round trains one brain for
    --round-steps against the other's newest checkpoint (rules at first).
    ポッドとドローンを交互に学習（相手は最新のチェックポイント）。"""
    import torch
    from sb3_contrib import MaskablePPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    torch.set_num_threads(2)
    t0 = time.time()
    deadline = t0 + a.hours * 3600.0
    prog = os.path.join(RES, "fleet_progress.csv")
    new_file = not os.path.exists(prog)
    pf = open(prog, "a", newline="")
    pw = csv.writer(pf)
    if new_file:
        base = evaluate_fleet(map_name=a.map)
        with open(os.path.join(RES, "fleet_rule_baseline.json"), "w") as f:
            json.dump(base, f, indent=1)
        pw.writerow(["round", "brain", "steps", "wall_s"] + list(FLEET_KEYS))
        pf.flush()
    sfile = os.path.join(RES, "train_state_fleet.json")
    rnd = 0
    retries = 0
    try:
        while time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP_fleet")):
            me = "pod" if rnd % 2 == 0 else "drone"
            d = os.path.join(CKPT, "fleet_" + me)
            os.makedirs(d, exist_ok=True)
            venv = None
            try:
                venv = VecMonitor(SubprocVecEnv([make_fleet(me, a.map, 5000 + 1000 * a.seed + 100 * rnd + i)
                                                 for i in range(a.envs)]))
                last = latest("fleet_" + me)
                if last:
                    model = MaskablePPO.load(last, env=venv, device="cpu")
                else:
                    model = MaskablePPO("MlpPolicy", venv, n_steps=512, batch_size=1024, n_epochs=6, seed=a.seed,
                                        learning_rate=3e-4, gamma=0.995, gae_lambda=0.95, ent_coef=0.01,
                                        device="cpu", verbose=0, policy_kwargs={"net_arch": [256, 256]})
                # status at the start of every round too (the dashboard showed "stopped",
                # 0 steps and no time left until the first round ended) / ラウンド開始時も状態を書く
                with open(sfile, "w") as f:
                    json.dump({"brain": "fleet", "running": True, "round": rnd, "training": me,
                               "steps": model.num_timesteps, "started": t0, "deadline": deadline, "t": time.time()}, f)
                from stable_baselines3.common.callbacks import BaseCallback

                class Stop(BaseCallback):
                    """Stops on the deadline / STOP_fleet; updates the status file about every
                    20k steps so the dashboard's step count moves during a round.
                    期限・停止ファイルで停止、約 2 万ステップごとに状態を更新"""
                    next_w = 0

                    def _on_step(self):
                        if self.num_timesteps >= self.next_w:
                            self.next_w = self.num_timesteps + 20000
                            with open(sfile, "w") as f:
                                json.dump({"brain": "fleet", "running": True, "round": rnd, "training": me,
                                           "steps": self.num_timesteps, "started": t0, "deadline": deadline,
                                           "t": time.time()}, f)
                        return time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP_fleet"))
                model.learn(total_timesteps=a.round_steps, reset_num_timesteps=False, callback=Stop())
                path = os.path.join(d, "step_%09d.zip" % model.num_timesteps)
                model.save(path)
                venv.close()
                venv = None
                k = evaluate_fleet(latest("fleet_pod"), latest("fleet_drone"), a.map)
                for rr in (30.0, 72.0):
                    kk = evaluate_fleet(latest("fleet_pod"), latest("fleet_drone"), a.map, n=2, rate=rr)
                    k["deliveries_at_%d" % rr] = kk["deliveries_per_h"]
                    k["waiting_at_%d" % rr] = kk["waiting"]
                    k["stranded_at_%d" % rr] = kk["stranded"]
                pw.writerow([rnd, me, model.num_timesteps, round(time.time() - t0)]
                            + [None if k.get(x) is None else round(k[x], 2) for x in FLEET_KEYS])
                pf.flush()
                with open(sfile, "w") as f:
                    json.dump({"brain": "fleet", "running": True, "round": rnd, "training": me, "steps": model.num_timesteps,
                               "last": k, "started": t0, "deadline": deadline, "t": time.time()}, f)
                print("[fleet] round %d (%s, %d steps): %.1f deliveries/h, crashes %.1f/100, lead %.0f s, "
                      "stranded %.1f, hover %.0f s/h" % (rnd, me, model.num_timesteps, k["deliveries_per_h"] or 0,
                                                        k["crashes_per_100"] or 0, k["lead_mean_s"] or 0,
                                                        k["stranded"] or 0, k["hover_s_per_h"] or 0), flush=True)
                rnd += 1
                retries = 0
            except Exception as exc:
                retries += 1
                print("[fleet] worker/round error in round %d (%s): %s (retry %d/3) / ワーカのエラー"
                      % (rnd, me, exc, retries), flush=True)
                if venv is not None:
                    try:
                        venv.close()
                    except Exception:
                        pass
                if retries >= 3:
                    print("[fleet] exceeded 3 retries, stopping / リトライ上限に達したため停止", flush=True)
                    raise
                time.sleep(2.0)
    finally:
        reason = "deadline" if time.time() >= deadline else (
            "stop_file" if os.path.exists(os.path.join(RES, "STOP_fleet")) else "interrupted"
        )
        print("[fleet] stopped (reason: %s, round: %d) / 停止" % (reason, rnd), flush=True)
        with open(sfile, "w") as f:
            json.dump({"brain": "fleet", "running": False, "round": rnd, "reason": reason, "t": time.time()}, f)
        pf.close()


def make_mission(map_name, seed):
    def _f():
        from sb3_contrib.common.wrappers import ActionMasker
        from rl_delivery.env_mission import MissionEnv
        return ActionMasker(MissionEnv(map_name, seed=seed), lambda e: e.action_masks())
    return _f


MISSION_KEYS = ("deliveries", "lead_mean_s", "crashes_per_100", "landing_min", "hold_s", "meet_wait_mean_s",
                "queue_end")


def evaluate_mission(model, map_name, n=6, seed=777, rule=False):
    """Mean KPIs over n one-hour episodes (fixed seeds). / ミッションの評価"""
    from rl_delivery.env_mission import MissionEnv, RuleMission
    env = MissionEnv(map_name, seed=seed)
    res = []
    for k in range(n):
        obs, _ = env.reset(seed=seed + k)
        pol = RuleMission(env) if rule else None
        while True:
            if rule:
                act = pol.act(obs)
            else:
                act, _ = model.predict(obs, action_masks=env.action_masks(), deterministic=True)
            obs, _r, te, tr, info = env.step(act)
            if te or tr:
                res.append(info["kpi"])
                break
    out = {}
    for key in MISSION_KEYS:
        v = [r[key] for r in res if r[key] is not None]
        out[key] = float(np.mean(v)) if v else None
    out["crash_why"] = {}
    for r in res:
        for w, c in r["crash_why"].items():
            out["crash_why"][w] = out["crash_why"].get(w, 0) + c
    return out


def train_mission(a):
    import torch
    from sb3_contrib import MaskablePPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    torch.set_num_threads(2)
    os.makedirs(os.path.join(CKPT, "mission"), exist_ok=True)
    venv = VecMonitor(SubprocVecEnv([make_mission(a.map, 2000 + i) for i in range(a.envs)]))
    resume = latest("mission") if a.resume else None
    if resume:
        model = MaskablePPO.load(resume, env=venv, device="cpu")
    else:
        model = MaskablePPO("MlpPolicy", venv, n_steps=1024, batch_size=1024, n_epochs=8, learning_rate=3e-4,
                            gamma=0.995, gae_lambda=0.95, ent_coef=0.01, device="cpu", verbose=0,
                            policy_kwargs={"net_arch": [128, 128]})
    prog = os.path.join(RES, "mission_progress.csv")
    new_file = not os.path.exists(prog) or not resume
    pf = open(prog, "w" if new_file else "a", newline="")
    pw = csv.writer(pf)
    if new_file:
        base = evaluate_mission(None, a.map, rule=True)
        with open(os.path.join(RES, "mission_rule_baseline.json"), "w") as f:
            json.dump(base, f, indent=1)
        pw.writerow(["steps", "wall_s"] + list(MISSION_KEYS) + ["ep_reward"])
    t0 = time.time()
    deadline = t0 + a.hours * 3600.0
    sfile = os.path.join(RES, "train_state_mission.json")

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = model.num_timesteps + a.eval_every

        def _on_step(self):
            if self.num_timesteps >= self.next_eval:
                self.next_eval += a.eval_every
                k = evaluate_mission(self.model, a.map)
                rew = [e["r"] for e in self.model.ep_info_buffer] if self.model.ep_info_buffer else [0]
                pw.writerow([self.num_timesteps, round(time.time() - t0)]
                            + [None if k[x] is None else round(k[x], 2) for x in MISSION_KEYS]
                            + [round(float(np.mean(rew)), 1)])
                pf.flush()
                path = os.path.join(CKPT, "mission", "step_%09d.zip" % self.num_timesteps)
                self.model.save(path)
                with open(path[:-4] + ".json", "w") as f:
                    json.dump(dict(k, steps=self.num_timesteps), f)
                with open(sfile, "w") as f:
                    json.dump({"brain": "mission", "running": True, "steps": self.num_timesteps, "last": k,
                               "checkpoint": path, "started": t0, "deadline": deadline, "t": time.time()}, f)
                print("[mission] %d steps  deliveries %.1f/h  crashes %.1f/100  lead %.0f s" % (
                    self.num_timesteps, k["deliveries"] or 0, k["crashes_per_100"] or 0, k["lead_mean_s"] or 0),
                    flush=True)
            return time.time() < deadline and not os.path.exists(os.path.join(RES, "STOP_mission"))

    try:
        model.learn(total_timesteps=10 ** 10, callback=Cb(), reset_num_timesteps=not resume)
    finally:
        path = os.path.join(CKPT, "mission", "step_%09d.zip" % model.num_timesteps)
        model.save(path)
        with open(sfile, "w") as f:
            json.dump({"brain": "mission", "running": False, "steps": model.num_timesteps, "checkpoint": path,
                       "t": time.time()}, f)
        pf.close()
        venv.close()


ENV_MODULE = {"fleet": "rl_delivery.env_fleet", "drive": "rl_delivery.env_drive", "nav": "rl_delivery.env_nav",
              "mission": "rl_delivery.env_mission", "flight": "rl_delivery.env_flight"}


def _record_run(a, phase, run_id=None, started=None):
    """Research record (rl_delivery/research/runs/<run_id>/ + runs_live.csv): the
    config snapshot at the start (arguments, reward / world constants of the env
    module, git commit), the progress file and end time at the end.
    研究記録：開始時の設定（報酬定数・コミット）と終了時の進捗。"""
    import importlib
    import subprocess
    rdir = os.path.join(HERE, "research", "runs") if not getattr(a, "tag", "") else os.path.dirname(a.run_dir)
    os.makedirs(rdir, exist_ok=True)
    if phase == "start":
        run_id = os.path.basename(a.run_dir) if getattr(a, "tag", "") else \
            "%s_%s" % (a.brain, time.strftime("%Y%m%d_%H%M%S"))
        consts = {}
        try:
            mod = importlib.import_module(ENV_MODULE[a.brain])
            consts = {k: v for k, v in vars(mod).items()
                      if k.isupper() and isinstance(v, (int, float, str, tuple, list))}
        except Exception as exc:                      # never block training / 学習は止めない
            consts = {"error": str(exc)}
        try:
            commit = subprocess.run(["git", "-C", HERE, "rev-parse", "--short", "HEAD"], capture_output=True,
                                    text=True, timeout=5).stdout.strip()
            dirty = bool(subprocess.run(["git", "-C", HERE, "status", "--porcelain", "--", "."], capture_output=True,
                                        text=True, timeout=5).stdout.strip())
        except Exception:
            commit, dirty = "", None
        os.makedirs(os.path.join(rdir, run_id), exist_ok=True)
        with open(os.path.join(rdir, run_id, "config.json"), "w") as f:
            json.dump({"run_id": run_id, "args": vars(a), "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                       "git_commit": commit, "uncommitted_changes": dirty, "env_constants": consts}, f, indent=1,
                      default=str)
        return run_id, time.time()
    prog = os.path.join(RES, "%s_progress.csv" % a.brain)
    if os.path.exists(prog):
        import shutil
        shutil.copy(prog, os.path.join(rdir, run_id, "progress.csv"))
    live = os.path.join(HERE, "research", "runs_live.csv")
    new = not os.path.exists(live)
    with open(live, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["run_id", "brain", "resume", "hours", "envs", "started", "ended", "wall_h", "config"])
        w.writerow([run_id, a.brain, a.resume, a.hours, a.envs,
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started)),
                    time.strftime("%Y-%m-%d %H:%M:%S"), round((time.time() - started) / 3600.0, 2),
                    "runs/%s/config.json" % run_id])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brain", choices=["flight", "mission", "nav", "fleet", "drive"], default="flight")
    ap.add_argument("--round-steps", type=int, default=200000)
    ap.add_argument("--map", default="OitaKosen")
    ap.add_argument("--hours", type=float, default=3.0)
    ap.add_argument("--envs", type=int, default=16)
    ap.add_argument("--eval-every", type=int, default=100000)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--algo", default="", help="nav: %s; drive: %s; fleet: maskable_ppo" % (
        "/".join(NAV_ALGOS), "/".join(DRIVE_ALGOS)))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dist", type=float, default=0.0, help="Override curriculum distance (m)")
    ap.add_argument("--wind-level", type=float, default=None, help="Override wind level (0.0-1.0)")
    ap.add_argument("--tag", default="", help="experiment name: the run gets its own folder "
                    "research/exp/<tag>/<run_id>/ (checkpoints, progress, state, STOP file)")
    a = ap.parse_args()
    if a.tag:
        # an experiment run never touches the main brains' checkpoints / 本番のチェックポイントに触れない
        global CKPT, RES, STATE
        rid = "%s_%s_s%d_%s" % (a.brain, a.algo or "default", a.seed, time.strftime("%Y%m%d_%H%M%S"))
        base = os.path.join(HERE, "research", "exp", a.tag, rid)
        CKPT, RES = os.path.join(base, "checkpoints"), base
        STATE = os.path.join(RES, "train_state.json")
        os.makedirs(CKPT, exist_ok=True)
        a.run_dir = base
    stop = os.path.join(RES, "STOP" if a.brain == "flight" else "STOP_" + a.brain)
    if os.path.exists(stop):
        os.remove(stop)
    run_id, started = _record_run(a, "start")
    try:
        _dispatch(a)
    finally:
        _record_run(a, "end", run_id, started)


def _dispatch(a):
    if a.brain == "fleet":
        train_fleet(a)
    elif a.brain == "drive":
        train_drive(a)
    elif a.brain == "mission":
        train_mission(a)
    elif a.brain == "nav":
        train_nav(a)
    else:
        train_flight(a)


if __name__ == "__main__":
    main()
