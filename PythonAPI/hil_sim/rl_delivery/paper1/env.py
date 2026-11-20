"""Training view of the Paper 1 world for the learned baselines B1-B3.

One step = one recovery decision of a drone that has just dropped its parcel
(the same decision B0 makes). Between decisions the world runs on the frozen
rules (the pod rule, medium launch height, the hover rule), exactly as in the
evaluation harness (rl_delivery/paper1/evaluate.py).

- action: Discrete(1 + n_stations): 0 = the pod's meeting point, 1.. = charger i.
- action_masks(): the same feasibility as B0 (Paper1World.estimate): battery
  reserve on arrival, charger in service, pod not stranded (B2, B3). B1 ignores
  it: an infeasible choice is carried out and the world punishes it (a battery
  crash, waiting at a dead charger).
- observation: ``obs_mode="base"`` (B1, B2) = the raw state; ``"forecast"`` (B3) =
  base + the per-option forecasts that B0 / the rollouts use (flight, wait,
  charge, return and ready time, battery on arrival, feasible).
- reward (fleet level, guide section 05), summed over the world between this
  decision and the next one of any drone (REWARD weights below).
- episodes: a scenario drawn from TRAIN_SCENARIOS (S1-S6; S7 stays unseen) and a
  seed from 10^6 up (never the evaluation seeds 1000-1029).

Paper 1 世界の学習用ビュー（B1〜B3）。1 ステップ = 投下直後のドローン 1 機の回収判断。
行動はポッドの合流地点か充電器 i。B2/B3 は B0 と同じ実行可能性マスク、B1 はマスクなし。
観測は base（B1/B2）と、予測付き forecast（B3）。報酬は艦隊全体（ガイド 05 章）。
"""
import math

import gymnasium as gym
import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.fleet_rules import rule_pod
from rl_delivery.paper1 import world as W1

TRAIN_SCENARIOS = [s for s in W1.SCENARIOS if not s.startswith("S7")]
TRAIN_SEED0 = 1_000_000
N_STN = 3

# fleet-level reward per decision interval (guide 05; docs/PAPER1_BASELINES.md 5.2) / 報酬の重み
REWARD = {
    "w_d": 1.0,      # per parcel delivered within 30 min / 30 分以内の配達 1 件
    "w_l": 0.05,     # per minute a delivered parcel was later than 30 min / 遅れ 1 分
    "w_r": 0.02,     # per drone-minute not ready after its drop (any drone) / 未準備のドローン分
    "w_e": 0.002,    # per Wh of drone energy / エネルギー 1 Wh
    "w_q": 0.1,      # per roof-pad or charger conflict / 競合 1 回
    "w_f": 5.0,      # per recovery failure (lost between drop and ready) / 回収失敗 1 回
}
ON_TIME_S = 1800.0


def obs_base(world, d):
    """The raw state at a recovery decision (no forecasts); v2 adds the pod's next
    stop. / 生の状態（予測なし）。v2 は次の停車地を追加"""
    pod = world.pods[0]
    tgt = world._pod_target(pod)
    others = [o for o in world.drones if o is not d]
    pend = [o for o in world.orders.values() if o["state"] != "done"]
    ws = float(np.mean(world.wind_hist)) if world.wind_hist else 0.0
    v = [d["batt"] / 100.0,
         math.dist(d["pos"], tgt) / 1000.0,
         float(pod["drive"] is not None),
         (math.dist(pod["pos"], tgt) / 1000.0),
         pod["batt"] / 100.0,
         float(world.t < pod.get("pad_free_t", 0.0)),
         sum(o["state"] == F.DOCKED for o in others) / 4.0,
         sum(o["state"] == F.FLY for o in others) / 4.0,
         sum(o["state"] == F.HOVER for o in others) / 4.0,
         sum(o["state"] in (W1.STN_CHARGE, W1.STN_QUEUE) for o in others) / 4.0,
         min([o["batt"] for o in others], default=100.0) / 100.0,
         len(pend) / 10.0,
         (max(world.t - o["t0"] for o in pend) / 1800.0) if pend else 0.0,
         ws / 10.0,
         world.t / F.EP_S]
    for i in range(N_STN):
        st = world.stations[i]
        v += [math.dist(d["pos"], st["xy"]) / 1000.0,
              math.dist(st["xy"], tgt) / 1000.0,
              sum(s is not None for s in st["slots"]) / max(1, len(st["slots"])),
              len(st["queue"]) / 3.0,
              float(world.station_out(i))]
    if world.meet_choice:
        n = world._next_stop(pod)
        nxy = world.xy[n] if n is not None else tgt
        v += [float(n is not None), math.dist(d["pos"], nxy) / 1000.0, math.dist(pod["pos"], nxy) / 1000.0]
    return v


def obs_forecast(world, d):
    """Per-option forecasts (B3): what B0 and the rollouts compute. / 選択肢ごとの予測"""
    v = []
    for o in world.options(d):
        e = world.estimate(d, o)
        v += [e.get("flight_s", 0.0) / 600.0, e.get("wait_s", 0.0) / 600.0, e.get("charge_s", 0.0) / 600.0,
              e.get("return_s", 0.0) / 600.0, e.get("ready_s", 0.0) / 1200.0,
              e.get("batt_arrive", 0.0) / 100.0, float(e["feasible"])]
    return v


def feasibility_mask(world, d):
    return np.array([world.estimate(d, o)["feasible"] for o in world.options(d)], bool)


def make_obs(world, d, obs_mode):
    v = obs_base(world, d) + (obs_forecast(world, d) if obs_mode == "forecast" else [])
    return np.clip(np.array(v, np.float32), -10.0, 10.0)


def n_options(world_name="v1"):
    return (2 if W1.WORLDS[world_name].get("meet_choice") else 1) + N_STN


def obs_size(obs_mode, world_name="v1"):
    extra = 3 if W1.WORLDS[world_name].get("meet_choice") else 0
    return 15 + 5 * N_STN + extra + (7 * n_options(world_name) if obs_mode == "forecast" else 0)


def frozen_step(world, p):
    """Answer a decision that is not the method's (frozen rules). / 固定ルールで答える"""
    from rl_delivery.paper1.evaluate import hover_rule
    if p[0] == "pod":
        world.pod_act(p[1], world.pod_rule(p[1]))
        return
    d, kind = p[1], p[2]
    if kind == F.D_HOVER:
        opt = hover_rule(world, d)
        if opt is None:
            d["ask"] = None
        else:
            world.recover(d, opt, by_method=False)
    else:
        world.drone_act(d, kind, 1)


class Paper1RecoveryEnv(gym.Env):
    """Gymnasium env: the recovery decision in the Paper 1 world. / 回収判断の学習環境"""
    metadata = {"render_modes": []}

    def __init__(self, obs_mode="base", seed=0, scenarios=None, pads=None, profile="v1", world="v1"):
        self.pads, self.profile, self.world_name = pads, profile, world
        self.obs_mode = obs_mode
        self.scenarios = list(scenarios or TRAIN_SCENARIOS)
        self.rng = np.random.default_rng(seed)
        self.worlds = {}
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (obs_size(obs_mode, world),), np.float32)
        self.action_space = gym.spaces.Discrete(n_options(world))
        self.world, self.cur = None, None

    def _world(self, scenario):
        if scenario not in self.worlds:
            self.worlds[scenario] = W1.Paper1World(scenario=scenario, pads=self.pads, profile=self.profile,
                                                   world=self.world_name)
        return self.worlds[scenario]

    def _snap(self):
        """World totals used for the reward. / 報酬用の累計"""
        w = self.world
        lead = w.k["lead"]
        not_ready = sum(1 for d in w.drones if d.get("p1_drop_t") is not None)
        return {"n_lead": len(lead), "fail": w.p1["fail"],
                "conf": w.p1["pad_conflicts"] + w.p1["charger_conflicts"],
                "energy": w.p1["energy_pct"] / 100.0 * w.P["drone_wh"], "t": w.t, "not_ready": not_ready}

    def _advance(self):
        """Run frozen decisions until the next recovery decision (or the end);
        accumulates not-ready drone-time on the way. / 次の回収判断まで進める"""
        w = self.world
        while True:
            t_before = w.t
            p = w.advance()
            dt = w.t - t_before
            self._acc_not_ready += dt * sum(1 for d in w.drones if d.get("p1_drop_t") is not None)
            if p is None:
                return None
            if p[0] == "drone" and p[2] == F.D_AFTER:
                return p
            frozen_step(w, p)

    def _reward(self, s0):
        w = self.world
        new_lead = w.k["lead"][s0["n_lead"]:]
        on_time = sum(1 for x in new_lead if x <= ON_TIME_S)
        late_min = sum(max(0.0, x - ON_TIME_S) for x in new_lead) / 60.0
        s1 = self._snap()
        r = (REWARD["w_d"] * on_time - REWARD["w_l"] * late_min
             - REWARD["w_r"] * self._acc_not_ready / 60.0
             - REWARD["w_e"] * (s1["energy"] - s0["energy"])
             - REWARD["w_q"] * (s1["conf"] - s0["conf"])
             - REWARD["w_f"] * (s1["fail"] - s0["fail"]))
        return float(r)

    def _obs(self):
        if self.cur is None:
            return np.zeros(self.observation_space.shape, np.float32)
        return make_obs(self.world, self.cur[1], self.obs_mode)

    def action_masks(self):
        if self.cur is None:
            m = np.zeros(self.action_space.n, bool)
            m[0] = True
            return m
        m = feasibility_mask(self.world, self.cur[1])
        if not m.any():
            # nothing keeps the reserve: the charger with the most battery on arrival
            # (B0's fallback), so the masked methods always have a choice / 全不可時は B0 と同じ退避先
            opts = self.world.options(self.cur[1])
            stn = [k for k, o in enumerate(opts) if o[0] == "station"]
            est = [self.world.estimate(self.cur[1], opts[k]) for k in stn]
            m[stn[int(np.argmax([e.get("batt_arrive", -1e9) for e in est]))]] = True
        return m

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        scen = self.scenarios[int(self.rng.integers(len(self.scenarios)))]
        ep_seed = TRAIN_SEED0 + int(self.rng.integers(1_000_000))
        self.world = self._world(scen)
        self.world.reset(seed=ep_seed)
        self._acc_not_ready = 0.0
        self.cur = self._advance()
        return self._obs(), {"scenario": scen, "seed": ep_seed}

    def step(self, action):
        w = self.world
        s0 = self._snap()
        self._acc_not_ready = 0.0
        if self.cur is not None:
            d = self.cur[1]
            w.recover(d, w.options(d)[int(action)])
        self.cur = self._advance()
        r = self._reward(s0)
        term = self.cur is None
        info = {"kpi": w.p1_kpi()} if term else {}
        return self._obs(), r, term, False, info
