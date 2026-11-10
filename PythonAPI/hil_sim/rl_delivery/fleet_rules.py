"""Rule version of the user's delivery flow: the baseline the pod and drone
brains must beat, and the other brain's stand-in early in training. Never in
control of a trained fleet. / ユーザーの配送手順のルール版（比較用）
"""
import math

from rl_delivery import env_fleet as F


def rule_pod(W, pod):
    m = W.pod_mask(pod)
    spots, lks, meets = W.pod_options(pod)
    mine = [d for d in W.drones if d["pod"] == pod["id"]]
    docked = [d for d in mine if d["state"] == F.DOCKED]
    out = [d for d in mine if d["state"] in (F.FLY, F.HOVER)]
    hover = [d for d in mine if d["state"] == F.HOVER]
    P = W.P
    # charge: low pod -> nearest station; stay until full / 充電
    if pod["charging"] is not None and pod["batt"] < P["pod_charge_to"] - 1.0 and not out:
        return F.A_WAIT
    if pod["batt"] < 30.0 and not out:
        s = min(range(len(W.stations)), key=lambda i: W.route(pod["node"], W.stations[i]["node"]))
        return F.A_CHARGE + s
    # a tired docked drone -> standby swap / 低電池機を交換
    if m[F.A_SWAP] and min((d["batt"] for d in docked), default=100.0) < 40.0 and not out:
        return F.A_SWAP
    if out:
        # drones in the air: F2F drops while they last, then the meeting point / 飛行中
        low = min((d["batt"] for d in hover), default=100.0)
        go_time = W.route(pod["node"], pod["meet_node"]) / W.pod_speed() if pod["meet_node"] is not None else 0.0
        if hover and (low - F.HOVER_PCT_S * (go_time + 60.0) < 25.0 or not any(L is not None for L in lks)):
            return F.A_GOMEET
        if m[F.A_LOCK] and lks[0] is not None and not hover:
            return F.A_LOCK
        if m[F.A_GOMEET]:
            return F.A_GOMEET
        return F.A_WAIT
    dro = W.pod_orders(pod, "drone")
    usable = [d for d in docked if d["batt"] > 45.0]
    if dro and usable:
        if pod["node"] == spots[0]:
            n = min(len(usable), len(dro), F.N_PER_POD)
            a = F.A_LAUNCH + (n - 1) * F.K_MEET + 1          # meet near the middle / 中央付近で合流
            return a if m[a] else F.A_WAIT
        return F.A_SPOT
    if lks and lks[0] is not None:
        return F.A_LOCK
    return F.A_WAIT


def rule_drone(W, d, kind):
    if kind == F.D_LAUNCH:
        return 1                                              # medium height / 中高度
    if kind == F.D_TASK:
        return 0 if d["batt"] > 50.0 else 1
    pod = W.pods[d["pod"]] if d["pod"] is not None else None
    tgt = pod["meet"] if pod is not None and pod["meet"] is not None else d["pos"]
    need = F.HOVER_PCT_S * (math.dist(d["pos"], tgt) / W.P["drone_speed"] * 1.4 + 120.0) + 10.0
    if kind == F.D_AFTER:
        return 0 if d["batt"] > need else 1
    return 0 if d["batt"] > 20.0 else 1
