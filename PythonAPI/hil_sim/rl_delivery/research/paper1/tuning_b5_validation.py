import sys, os, math, multiprocessing as mp
sys.path.insert(0, os.getcwd())
V = {"B0": None, "h600": {"horizon_s": 600.0}, "h1200": {"horizon_s": 1200.0},
     "h600_conf": {"horizon_s": 600.0, "margin_se": 2.0}, "h1200_conf": {"horizon_s": 1200.0, "margin_se": 2.0}}
SC = ["S1_normal", "S2_rush_hour", "S3_pod_delay", "S4_station_outage", "S5_high_wind", "S6_burst_returns", "S7_unseen_customers"]
DAYS = range(2020, 2030)      # the validation days B4 was tuned on / B4 と同じ検証日
def job(a):
    v, scen, seed = a
    import warnings; warnings.filterwarnings("ignore")
    from rl_delivery.paper1.evaluate import run_episode
    if v == "B0":
        row, _ = run_episode("B0", scen, seed, log_decisions=False, world="v2")
    else:
        from rl_delivery.paper1.rollout import make_rollout_method
        row, _ = run_episode("B5", scen, seed, log_decisions=False, world="v2", method=make_rollout_method("B5", None, world_name="v2", **V[v]))
    return v, scen, seed, row
if __name__ == "__main__":
    jobs = [(v, s, d) for v in V for s in SC for d in DAYS]
    jobs.sort(key=lambda j: j[0] == "B0")          # slow ones first / 遅いものから
    with mp.get_context("fork").Pool(12) as p:
        res = p.map(job, jobs, chunksize=1)
    R = {(v, s, d): r for v, s, d, r in res}
    keys = [(s, d) for s in SC for d in DAYS]
    print("B5 validation days (7 situations x seeds 2020-2029), paired against B0")
    for v in V:
        g = lambda k, f: float(R[(v,) + k][f])
        line = "%-11s on-time %6.2f ready %4.0f s next %4.1f stn %4.1f" % (v, sum(g(k, "ontime30_per_h") for k in keys) / len(keys),
               sum(g(k, "ready_time_mean_s") for k in keys) / len(keys), sum(g(k, "to_next") for k in keys) / len(keys), sum(g(k, "to_station") for k in keys) / len(keys))
        for f, nm in (("ontime30_per_h", "on-time"), ("ready_time_mean_s", "ready")):
            d = [g(k, f) - float(R[("B0",) + k][f]) for k in keys]; mu = sum(d) / len(d)
            se = (sum((x - mu) ** 2 for x in d) / (len(d) - 1)) ** 0.5 / math.sqrt(len(d))
            line += " | %s %+.2f ± %.2f" % (nm, mu, 1.99 * se)
        print(line)
