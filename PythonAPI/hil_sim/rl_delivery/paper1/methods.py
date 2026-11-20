"""Paper 1 recovery methods (docs/PAPER1_BASELINES.md section 3).

A method is an object with ``name`` and ``decide(world, drone) -> (option,
info)``; ``option`` is one of ``world.options(drone)``, ``info`` is logged.
Only B0 is implemented so far; B1-B5 follow the same interface.

Paper 1 の回収判断手法。decide(world, drone) が (選択, 記録用情報) を返す。現在は B0 のみ。
"""
import math

from rl_delivery.paper1.world import Paper1World


class B0NearestFeasibleQueue(object):
    """B0: nearest feasible recovery option + deterministic queue score.

    Among the options the feasibility check allows (battery reserve on
    arrival, charger in service, pod not stranded), pick the one with the
    lowest estimated time until the drone is ready again: flight + wait (pod
    arrival, roof-pad queue, charger queue) + charge + the empty flight back
    from a charger. No learning, no lookahead: the transparent floor every
    learned method must beat. If no option is feasible, fall back to the
    charger with the most battery left on arrival (counted as a fallback).
    B0：実行可能な回収先のうち「再び準備完了になるまでの見積り時間」が最小のものを選ぶ。
    学習も先読みもしない基準手法。実行可能な先が無ければ到着時電池が最大の充電器へ。
    """
    name = "B0"

    def decide(self, world, d):
        opts = world.options(d)
        est = [world.estimate(d, o) for o in opts]
        feas = [i for i, e in enumerate(est) if e["feasible"]]
        fallback = not feas
        if feas:
            best = min(feas, key=lambda i: (est[i]["ready_s"], i))
        else:
            stations = [i for i, o in enumerate(opts) if o[0] == "station"]
            best = max(stations, key=lambda i: est[i].get("batt_arrive", -math.inf))
            world.p1["fallbacks"] += 1
        info = {"mask": [e["feasible"] for e in est], "mask_reasons": [e["reason"] for e in est],
                "candidates": [e["option"] for e in est],
                "score_ready_s": [round(e["ready_s"], 1) if "ready_s" in e else None for e in est],
                "fallback": fallback, "predicted_ready_s": round(est[best].get("ready_s", float("nan")), 1)}
        return opts[best], info


class PolicyMethod(object):
    """B1 / B2 / B3: a trained policy choosing the recovery option.
    B1 = PPO without the mask (an infeasible pick is carried out and counted);
    B2 = MaskablePPO; B3 = MaskablePPO with the forecast inputs.
    B1/B2/B3：学習済み方策が回収先を選ぶ（B1 はマスクなし、不可能な選択も実行して数える）。"""
    SPEC = {"B1": ("ppo", "base", False), "B2": ("maskable_ppo", "base", True),
            "B3": ("maskable_ppo", "forecast", True)}
    _cache = {}

    def __init__(self, name, model):
        self.name = name
        algo, self.obs_mode, self.masked = self.SPEC[name]
        if isinstance(model, str):
            if model not in self._cache:
                if algo == "ppo":
                    from stable_baselines3 import PPO as cls
                else:
                    from sb3_contrib import MaskablePPO as cls
                self._cache[model] = cls.load(model, device="cpu")
            model = self._cache[model]
        self.model = model

    def decide(self, world, d):
        import numpy as np
        from rl_delivery.paper1.env import make_obs
        opts = world.options(d)
        est = [world.estimate(d, o) for o in opts]
        mask = np.array([e["feasible"] for e in est], bool)
        fallback = not mask.any()
        if fallback:
            stn = [k for k, o in enumerate(opts) if o[0] == "station"]
            mask[stn[int(np.argmax([est[k].get("batt_arrive", -1e9) for k in stn]))]] = True
            world.p1["fallbacks"] += 1
        obs = make_obs(world, d, self.obs_mode)
        if self.masked:
            a = int(self.model.predict(obs, action_masks=mask, deterministic=True)[0])
        else:
            a = int(self.model.predict(obs, deterministic=True)[0])
        infeasible = not est[a]["feasible"]
        if infeasible and not self.masked:
            world.p1["infeasible_picks"] = world.p1.get("infeasible_picks", 0) + 1
        info = {"mask": [e["feasible"] for e in est], "mask_reasons": [e["reason"] for e in est],
                "candidates": [e["option"] for e in est],
                "score_ready_s": [round(e["ready_s"], 1) if "ready_s" in e else None for e in est],
                "fallback": fallback, "infeasible_pick": infeasible,
                "predicted_ready_s": round(est[a].get("ready_s", float("nan")), 1)}
        return opts[a], info


class AlwaysPodDiagnostic(object):
    """Diagnostic only (not a baseline): the pod whenever feasible, else B0's
    fallback. Shows whether the recovery choice matters in a world: if B0 and this
    tie, it does not. / 診断用：可能な限りポッド。回収判断が効く世界かを確かめる。"""
    name = "POD"

    def decide(self, world, d):
        opts = world.options(d)
        e0 = world.estimate(d, opts[0])
        if e0["feasible"]:
            return opts[0], {"chosen_rule": "pod", "predicted_ready_s": round(e0["ready_s"], 1)}
        return B0NearestFeasibleQueue().decide(world, d)


METHODS = {"B0": B0NearestFeasibleQueue, "POD": AlwaysPodDiagnostic}
LEARNED = ("B1", "B2", "B3")
ROLLOUT = ("B4", "B5")


def make_method(name, checkpoint=None, world="v1"):
    """B0 / POD by name; B1-B3 need a checkpoint (path or model); B4 needs a B2
    checkpoint, B5 none. / 手法を作る"""
    if name in ROLLOUT:
        from rl_delivery.paper1.rollout import make_rollout_method
        return make_rollout_method(name, checkpoint, world_name=world)
    if name in LEARNED:
        if checkpoint is None:
            raise ValueError("%s needs --checkpoint / チェックポイントが必要" % name)
        return PolicyMethod(name, checkpoint)
    return METHODS[name]()
