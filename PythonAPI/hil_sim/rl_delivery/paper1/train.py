"""Train the learned Paper 1 baselines B1 (PPO), B2 (MaskablePPO), B3 (MaskablePPO
+ forecasts) on the recovery decision (rl_delivery/paper1/env.py).

  ../../carla-venv/bin/python -m rl_delivery.paper1.train --method B2 --seed 0 --steps 300000
  touch rl_delivery/research/paper1/STOP_<method>_s<seed>        # stop (saves first)

Every ``--eval-every`` steps the policy is checked on validation episodes
(scenarios S1 / S3 / S5, seeds 2000..; never the evaluation seeds 1000-1029)
and a checkpoint is saved to rl_delivery/checkpoints/paper1/<method>/s<seed>/.
Progress: rl_delivery/research/paper1/train_<method>_s<seed>.csv.

Paper 1 の学習型ベースライン B1/B2/B3 を回収判断で学習する。検証は S1/S3/S5 の別シード。
"""
import argparse
import csv
import json
import os
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RL = os.path.dirname(HERE)
CKPT = os.path.join(RL, "checkpoints", "paper1")
OUT = os.path.join(RL, "research", "paper1")
VAL_SCENARIOS = ("S1_normal", "S3_pod_delay", "S5_high_wind")
VAL_SEED0, VAL_N = 2000, 4
SPEC = {"B1": ("ppo", "base"), "B2": ("maskable_ppo", "base"), "B3": ("maskable_ppo", "forecast")}
COLS = ["steps", "wall_s", "ep_reward", "val_delivered_per_h", "val_ontime30_per_h", "val_ready_mean_s",
        "val_recovery_failure_rate", "val_to_station", "val_infeasible_picks", "val_fallbacks"]


def make_env(obs_mode, seed, pads=None, profile="v1", world="v1"):
    def _f():
        from rl_delivery.paper1.env import Paper1RecoveryEnv
        return Paper1RecoveryEnv(obs_mode=obs_mode, seed=seed, pads=pads, profile=profile, world=world)
    return _f


def validate(method_name, model, pads=None, profile="v1", world="v1"):
    """Deterministic episodes on the validation seeds. / 検証エピソード"""
    from rl_delivery.paper1.evaluate import run_episode
    from rl_delivery.paper1.methods import PolicyMethod
    m = PolicyMethod(method_name, model)
    rows = [run_episode(method_name, s, VAL_SEED0 + k, log_decisions=False, method=m, pads=pads,
                        profile=profile, world=world)[0]
            for s in VAL_SCENARIOS for k in range(VAL_N)]

    def mean(key):
        v = [r[key] for r in rows if r.get(key) is not None]
        return float(np.mean(v)) if v else float("nan")
    return {"val_delivered_per_h": mean("delivered_per_h"), "val_ontime30_per_h": mean("ontime30_per_h"),
            "val_ready_mean_s": mean("ready_time_mean_s"), "val_recovery_failure_rate": mean("recovery_failure_rate"),
            "val_to_station": mean("to_station"), "val_infeasible_picks": mean("infeasible_picks"),
            "val_fallbacks": mean("fallbacks")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", required=True, choices=sorted(SPEC))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--eval-every", type=int, default=30_000)
    ap.add_argument("--pads", type=int, default=0, help="charging pads per station (0 = all) / 局あたりのパッド数")
    ap.add_argument("--profile", default="v1", help="hardware profile: v1 | typical / 機体プロファイル")
    ap.add_argument("--world", default="v1", help="Paper 1 world version: v1 | v2 / 世界のバージョン")
    a = ap.parse_args()
    import torch
    torch.set_num_threads(2)
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
    algo, obs_mode = SPEC[a.method]
    pads = a.pads or None
    variant = "_".join(v for v in ("" if a.world == "v1" else a.world, ("pad%d" % pads) if pads else "",
                                    "" if a.profile == "v1" else a.profile) if v)
    run = "%s_s%d" % (a.method, a.seed) + ("_" + variant if variant else "")
    cdir = os.path.join(CKPT, variant, a.method, "s%d" % a.seed) if variant else \
        os.path.join(CKPT, a.method, "s%d" % a.seed)
    os.makedirs(cdir, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    stop = os.path.join(OUT, "STOP_" + run)
    if os.path.exists(stop):
        os.remove(stop)
    venv = VecMonitor(SubprocVecEnv([make_env(obs_mode, 7919 * (a.seed + 1) + i, pads, a.profile, a.world)
                                     for i in range(a.envs)]))
    kw = dict(n_steps=256, batch_size=512, n_epochs=10, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
              ent_coef=0.01, clip_range=0.2, seed=a.seed, device="cpu", verbose=0,
              policy_kwargs={"net_arch": [128, 128]})
    if algo == "ppo":
        from stable_baselines3 import PPO
        model = PPO("MlpPolicy", venv, **kw)
    else:
        from sb3_contrib import MaskablePPO
        model = MaskablePPO("MlpPolicy", venv, **kw)
    with open(os.path.join(cdir, "config.json"), "w") as f:
        from rl_delivery.paper1.env import REWARD, TRAIN_SCENARIOS
        json.dump({"method": a.method, "algo": algo, "obs_mode": obs_mode, "seed": a.seed, "steps": a.steps,
                   "pads_per_station": pads, "profile": a.profile, "world": a.world,
                   "envs": a.envs, "hyper": {k: v for k, v in kw.items() if k != "policy_kwargs"},
                   "net_arch": [128, 128], "reward": REWARD, "train_scenarios": TRAIN_SCENARIOS,
                   "val_scenarios": VAL_SCENARIOS, "val_seeds": [VAL_SEED0, VAL_SEED0 + VAL_N - 1]}, f, indent=1)
    prog = os.path.join(OUT, "train_%s.csv" % run)
    pf = open(prog, "w", newline="")
    pw = csv.writer(pf)
    pw.writerow(COLS)
    t0 = time.time()

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = a.eval_every

        def _on_step(self):
            if self.num_timesteps >= self.next_eval:
                self.next_eval += a.eval_every
                v = validate(a.method, self.model, pads, a.profile, a.world)
                rew = [e["r"] for e in self.model.ep_info_buffer] if self.model.ep_info_buffer else [0.0]
                row = [self.num_timesteps, round(time.time() - t0), round(float(np.mean(rew)), 2)] + \
                      [round(v[c], 3) for c in COLS[3:]]
                pw.writerow(row)
                pf.flush()
                self.model.save(os.path.join(cdir, "step_%07d.zip" % self.num_timesteps))
                print("[paper1 %s] %d steps  ep_reward %.1f | val: delivered %.1f/h  on-time30 %.1f/h  "
                      "ready %.0f s  failures %.3f  to-station %.1f  infeasible %.1f" % (
                          run, self.num_timesteps, row[2], v["val_delivered_per_h"], v["val_ontime30_per_h"],
                          v["val_ready_mean_s"], v["val_recovery_failure_rate"], v["val_to_station"],
                          v["val_infeasible_picks"]), flush=True)
            return not os.path.exists(stop)

    try:
        model.learn(total_timesteps=a.steps, callback=Cb())
    finally:
        model.save(os.path.join(cdir, "final.zip"))
        pf.close()
        venv.close()
        print("[paper1 %s] done: %s (%.0f s)" % (run, os.path.join(cdir, "final.zip"), time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
