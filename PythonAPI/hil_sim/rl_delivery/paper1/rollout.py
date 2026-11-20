"""Rollout-assisted recovery for Paper 1: B4 (proposed) and B5 (no learning).

At a drone's recovery decision the method copies the world, plays each candidate
option a few minutes forward in the copy, scores the imagined futures with the
training reward (rl_delivery/paper1/env.py REWARD) and picks the best one.

- **B4** (rollout + critic + prior): candidates = the top-K options of the trained
  B2 policy (its prior), the other drones' later decisions in the copy are made by
  B2 (the continuation), and the end of each rollout is scored by B2's critic V.
  If the rollouts do not finish within the deadline, B4 falls back to B2's choice.
- **B5** (fixed continuation, no network): every feasible option is a candidate, the
  continuation is the B0 rule, no critic (a longer horizon instead). B4 vs B5 shows
  whether the learned parts are needed at all.

Honesty rules for the imagined futures (they must not see the real future):
each copy gets fresh random seeds (orders, wind gusts, crash draws), the next order
arrival is re-drawn, and crash outcomes already drawn for flights in the air are
removed (assumed none). Seeds come from (decision time, drone, sample), so a run is
reproducible, and every candidate sees the same samples (common random numbers).

Paper 1 のロールアウト手法：B4（提案：事前分布＋継続方策＋評価関数に B2 を使う）と
B5（学習なし：継続は B0 ルール）。判断時に世界を複製し、各候補を数分先まで再生して
報酬で採点する。複製は本当の未来を知らない（乱数を新たに与え、飛行中の墜落結果は消す）。
"""
import copy
import math
import time
import types
import zlib

import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.paper1 import env as E

# attributes that change during an episode: copied for each rollout; everything
# else (map, roads, buildings, route caches, settings) is shared with the real world
# エピソード中に変わる属性（複製する）。地図・道路・経路キャッシュなどは共有する。
DYNAMIC = ("drones", "pods", "orders", "stations", "k", "p1", "next_order", "oid", "requests",
           "decisions", "events", "reward", "rng", "t", "wind", "wind_hist", "rain", "rain_target",
           "rate", "rate_fixed", "outage_station", "log")


def _rebind(obj, memo):
    """Copy a value; a function (an arrival callback) is rebuilt with its captured
    world / pod / order replaced by their copies. / 関数は複製先に結び直す"""
    if isinstance(obj, types.FunctionType):
        key = id(obj)
        if key in memo:
            return memo[key]
        cells = None
        if obj.__closure__:
            cells = tuple(types.CellType(_rebind(c.cell_contents, memo)) for c in obj.__closure__)
        f = types.FunctionType(obj.__code__, obj.__globals__, obj.__name__,
                               _rebind(obj.__defaults__, memo) if obj.__defaults__ else None, cells)
        f.__kwdefaults__ = obj.__kwdefaults__
        memo[key] = f
        return f
    if isinstance(obj, types.MethodType):
        return types.MethodType(obj.__func__, _rebind(obj.__self__, memo))
    if isinstance(obj, tuple):
        return tuple(_rebind(x, memo) for x in obj)
    return copy.deepcopy(obj, memo)


def clone_world(w, seed):
    """An independent copy of the world for imagining, with fresh randomness.
    想像用の独立した複製（乱数は新しく与える）。"""
    c = copy.copy(w)                                     # shares the static parts / 静的部分は共有
    memo = {id(w): c}
    if getattr(w, "wind", None) is not None and getattr(w.wind, "grid", None) is not None:
        memo[id(w.wind.grid)] = w.wind.grid
    for name in DYNAMIC:
        if hasattr(w, name):
            setattr(c, name, copy.deepcopy(getattr(w, name), memo))
    for d in c.drones:                                   # arrival callbacks -> the copy / 到着処理を結び直す
        for k, v in list(d.items()):
            if callable(v):
                d[k] = _rebind(v, memo)
        d.pop("pending_crash", None)                     # hidden: not known in advance / 墜落は事前に分からない
    for r in c.requests:
        for k, v in list(r.items()):
            if callable(v):
                r[k] = _rebind(v, memo)
    c.log, c.events = None, []
    c.rng = np.random.default_rng(seed)
    if getattr(c, "wind", None) is not None and hasattr(c.wind, "rng"):
        c.wind.rng = np.random.default_rng(seed + 7919)
    burst = int(c.sc.get("burst", 1))
    for z in list(c.next_order):                         # the next arrival is not known / 次の注文時刻は未知
        c.next_order[z] = c.t + burst * float(c.rng.uniform(*c.order_gap))
    return c


def _seed(t, drone, j, base):
    return zlib.crc32(("%d|%d|%d|%d" % (int(round(t * 10)), drone, j, base)).encode()) & 0x7FFFFFFF


class RolloutMethod(object):
    """B4 / B5. ``model`` = a trained B2 (MaskablePPO) for B4, None for B5.
    B4/B5 のロールアウト手法。"""

    def __init__(self, name, model=None, top_k=3, samples=4, horizon_s=600.0, gamma=0.99,
                 deadline_s=20.0, base_seed=0, world_name="v2", max_steps=None, margin_se=None,
                 fixed_time=False, prior_gate=None):
        self.name = name
        self.model = model
        self.learned = model is not None
        self.top_k, self.samples, self.horizon_s = top_k, samples, horizon_s
        self.gamma, self.deadline_s, self.base_seed = gamma, deadline_s, base_seed
        self.world_name = world_name
        # max_steps: cap on decisions per rollout (incl. this one); margin_se: B4 keeps
        # B2's choice unless another candidate is better by more than margin_se
        # standard errors of the paired difference. / 打ち切り判断数と信頼性の条件
        self.max_steps, self.margin_se = max_steps, margin_se
        # fixed_time: every option's rollout ends at exactly t + horizon and the critic
        # scores the state then (no uneven look-ahead); prior_gate: keep B2's choice
        # without rollouts when B2 gives it at least this probability
        # fixed_time：全候補を同じ時刻で打ち切る。prior_gate：B2 が十分確かなら想像しない
        self.fixed_time, self.prior_gate = fixed_time, prior_gate
        self._env = None
        if self.learned:
            import torch
            torch.set_num_threads(1)                       # one core per evaluation worker / 1 ワーカー 1 コア

    # -- the learned parts (B4) / 学習済みの部分
    def _policy_parts(self, obs, mask):
        import torch
        with torch.no_grad():
            t = torch.as_tensor(obs[None, :], dtype=torch.float32)
            dist = self.model.policy.get_distribution(t, action_masks=mask[None, :])
            probs = dist.distribution.probs[0].cpu().numpy()
        return probs

    def _value(self, obs):
        import torch
        with torch.no_grad():
            t = torch.as_tensor(obs[None, :], dtype=torch.float32)
            return float(self.model.policy.predict_values(t)[0, 0])

    def _continue(self, env):
        """The continuation's choice at a later decision inside a rollout. / 継続方策"""
        mask = env.action_masks()
        if self.learned:
            obs = env._obs()
            return int(self.model.predict(obs, action_masks=mask, deterministic=True)[0])
        from rl_delivery.paper1.methods import B0NearestFeasibleQueue
        w, d = env.world, env.cur[1]
        opt, _info = B0NearestFeasibleQueue().decide(w, d)  # counts only inside the copy / 複製内のみ
        return w.options(d).index(opt)

    def _rollout(self, world, d, action, seed):
        """Imagined return of choosing ``action`` now. / この選択の想像上の収益"""
        if self._env is None:
            self._env = E.Paper1RecoveryEnv(obs_mode="base", world=self.world_name)
        env = self._env
        c = clone_world(world, seed)
        cd = c.drones[d["id"]]
        env.world, env.cur, env._acc_not_ready = c, ("drone", cd, F.D_AFTER), 0.0
        t_end = c.t + self.horizon_s
        if self.fixed_time:
            c.t_stop = min(t_end, F.EP_S)
        _obs, r, term, _tr, _info = env.step(action)
        ret, disc, n = r, self.gamma, 1
        while not term and env.cur is not None and c.t < t_end and (self.max_steps is None or n < self.max_steps):
            n += 1
            _obs, r, term, _tr, _info = env.step(self._continue(env))
            ret += disc * r
            disc *= self.gamma
        if self.fixed_time:
            if self.learned and c.t >= c.t_stop - 1e-6 and c.t < F.EP_S:
                # stopped at the common end time: the critic scores the state from the
                # deciding drone's view / 共通の終了時刻で、判断したドローンの視点から評価
                ret += disc * self._value(E.make_obs(c, cd, "base"))
            return ret, c.t
        if self.learned and not term and env.cur is not None:
            ret += disc * self._value(env._obs())          # the critic scores the rest / 残りは評価関数
        return ret, c.t

    # -- the method interface (evaluate.py) / 評価ハーネス用
    def decide(self, world, d):
        t0 = time.perf_counter()
        opts = world.options(d)
        est = [world.estimate(d, o) for o in opts]
        mask = np.array([e["feasible"] for e in est], bool)
        fallback = not mask.any()
        if fallback:
            stn = [k for k, o in enumerate(opts) if o[0] == "station"]
            mask[stn[int(np.argmax([est[k].get("batt_arrive", -1e9) for k in stn]))]] = True
            world.p1["fallbacks"] += 1
        obs = E.make_obs(world, d, "base")
        prior = None
        if self.learned:
            prior = self._policy_parts(obs, mask)
            ranked = [int(a) for a in np.argsort(-prior) if mask[a]]
            cands = ranked[:self.top_k]
            policy_choice = ranked[0]
        else:
            cands = [int(a) for a in np.flatnonzero(mask)]
            # B5's base choice = B0's (shortest estimated time to ready among the
            # feasible options), used only by the confident-override variant
            # B5 の基準選択 = B0 の選択（信頼性条件つきの変種でのみ使う）
            policy_choice = min(cands, key=lambda k: (est[k].get("ready_s", math.inf), k))
        q = {}
        timed_out = False
        gated = bool(self.learned and self.prior_gate is not None and prior[policy_choice] >= self.prior_gate)
        if gated:
            cands = [policy_choice]                         # B2 is sure: no rollouts / B2 が確かなら想像しない
        if len(cands) > 1:
            for j in range(self.samples):
                seed = _seed(world.t, d["id"], j, self.base_seed)
                for a in cands:
                    if time.perf_counter() - t0 > self.deadline_s:
                        timed_out = True
                        break
                    q.setdefault(a, []).append(self._rollout(world, d, a, seed)[0])
                if timed_out:
                    break
        complete = [a for a in cands if len(q.get(a, [])) == len(q.get(cands[0], [])) and q.get(a)]
        if len(cands) == 1:
            a = cands[0]
        elif timed_out:
            a = policy_choice                               # deadline: B2's own choice / 期限切れは B2 の選択
        else:
            a = max(complete, key=lambda k: (float(np.mean(q[k])), -cands.index(k)))
            if self.margin_se is not None and a != policy_choice and policy_choice in complete:
                # override B2 only when the rollouts are confident (paired samples)
                # B2 の選択を覆すのはロールアウトが確かな場合だけ
                diff = np.array(q[a]) - np.array(q[policy_choice])
                se = float(diff.std(ddof=1) / math.sqrt(len(diff))) if len(diff) > 1 else float("inf")
                if not float(diff.mean()) > self.margin_se * se:
                    a = policy_choice
        if timed_out:
            world.p1["rollout_timeouts"] = world.p1.get("rollout_timeouts", 0) + 1
        ms = (time.perf_counter() - t0) * 1000.0
        info = {"mask": [bool(m) for m in mask], "mask_reasons": [e["reason"] for e in est],
                "candidates": [e["option"] for e in est], "rollout_set": [est[k]["option"] for k in cands],
                "q": {est[k]["option"]: round(float(np.mean(v)), 3) for k, v in q.items()},
                "prior": None if prior is None else [round(float(p), 3) for p in prior],
                "policy_choice": est[policy_choice]["option"],
                "agrees_with_policy": bool(a == policy_choice),
                "fallback": fallback, "timed_out": timed_out, "gated": gated, "rollout_ms": round(ms, 1),
                "score_ready_s": [round(e["ready_s"], 1) if "ready_s" in e else None for e in est],
                "predicted_ready_s": round(est[a].get("ready_s", float("nan")), 1)}
        return opts[a], info


def make_rollout_method(name, checkpoint=None, world_name="v2", **kw):
    """B4 needs a B2 checkpoint; B5 needs none. / B4 は B2 のチェックポイントが必要"""
    if name == "B4":
        if checkpoint is None:
            raise ValueError("B4 needs --checkpoint (a B2 model) / B2 のモデルが必要")
        from sb3_contrib import MaskablePPO
        model = MaskablePPO.load(checkpoint, device="cpu") if isinstance(checkpoint, str) else checkpoint
        # chosen on validation days only (seeds 2020-2029, 7 situations; docs/PAPER1_BASELINES.md):
        # 10 min look-ahead, 4 samples, top-3, override B2 only when 2 standard errors better
        # 検証日のみで選んだ設定（B2 を覆すのは 2 標準誤差以上良い場合だけ）
        kw.setdefault("margin_se", 2.0)
        # 2026-11-18 (validation days only, after the CARLA diagnosis): every option's
        # look-ahead ends at the same time, and B2's choice is kept without rollouts when
        # B2 is at least 95 % sure (the previous setting is "B4conf" in the results)
        # 全候補を同じ時刻で打ち切り、B2 が 95 % 以上確かならその選択を保つ（旧設定は B4conf）
        kw.setdefault("fixed_time", True)
        kw.setdefault("prior_gate", 0.95)
        return RolloutMethod("B4", model=model, world_name=world_name, **kw)
    # chosen on the same validation days as B4 (seeds 2020-2029, 7 situations): no critic,
    # so 20 min look-ahead; keep B0's choice unless 2 standard errors better
    # B4 と同じ検証日で選んだ設定：評価関数なしで 20 分先まで、B0 の選択を覆すのは確かな場合だけ
    kw.setdefault("horizon_s", 1200.0)
    kw.setdefault("margin_se", 2.0)
    return RolloutMethod("B5", model=None, world_name=world_name, **kw)
