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


def latest(brain):
    files = glob.glob(os.path.join(CKPT, brain, "step_*.zip"))
    return max(files, key=lambda p: int(os.path.basename(p)[5:-4])) if files else None


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
    venv = VecMonitor(SubprocVecEnv([make_nav(a.map, 3000 + i) for i in range(a.envs)]))
    resume = latest("nav") if a.resume else None
    level, dist = 0.0, 80.0
    if resume:
        model = RecurrentPPO.load(resume, env=venv, device="cpu")
        meta = resume[:-4] + ".json"
        if os.path.exists(meta):
            with open(meta) as f:
                j = json.load(f)
            level, dist = j.get("wind_level", 0.0), j.get("dist", 200.0)
    else:
        model = RecurrentPPO("MlpLstmPolicy", venv, n_steps=512, batch_size=2048, n_epochs=5, learning_rate=3e-4,
                             gamma=0.995, gae_lambda=0.95, ent_coef=0.0, device="cpu", verbose=0,
                             policy_kwargs={"lstm_hidden_size": 192, "net_arch": [256, 256]})

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brain", choices=["flight", "mission", "nav"], default="flight")
    ap.add_argument("--map", default="OitaKosen")
    ap.add_argument("--hours", type=float, default=3.0)
    ap.add_argument("--envs", type=int, default=16)
    ap.add_argument("--eval-every", type=int, default=100000)
    ap.add_argument("--resume", action="store_true")
    a = ap.parse_args()
    stop = os.path.join(RES, "STOP" if a.brain == "flight" else "STOP_" + a.brain)
    if os.path.exists(stop):
        os.remove(stop)
    if a.brain == "mission":
        train_mission(a)
    elif a.brain == "nav":
        train_nav(a)
    else:
        train_flight(a)


if __name__ == "__main__":
    main()
