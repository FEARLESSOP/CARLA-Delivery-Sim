import sys, os, math, multiprocessing as mp
sys.path.insert(0, os.getcwd())
CK = "rl_delivery/checkpoints/paper1/v2/B2/s0/final.zip"
V = {"B2": None, "C_confident": {"margin_se": 2.0}, "equal_time": {"margin_se": 2.0, "fixed_time": True}, "gate95": {"margin_se": 2.0, "prior_gate": 0.95}, "gate99": {"margin_se": 2.0, "prior_gate": 0.99}, "equal_gate95": {"margin_se": 2.0, "fixed_time": True, "prior_gate": 0.95}}
SC = ["S1_normal", "S2_rush_hour", "S3_pod_delay", "S4_station_outage", "S5_high_wind", "S6_burst_returns", "S7_unseen_customers"]
DAYS = range(2020, 2030)
def job(a):
    v, scen, seed = a
    import warnings; warnings.filterwarnings("ignore")
    from rl_delivery.paper1.evaluate import run_episode
    if v == "B2":
        row, _ = run_episode("B2", scen, seed, log_decisions=False, world="v2", checkpoint=CK)
    elif v == "B3":
        row, _ = run_episode("B3", scen, seed, log_decisions=False, world="v2", checkpoint=CK.replace("/B2/", "/B3/"))
    else:
        from rl_delivery.paper1.rollout import make_rollout_method
        m = make_rollout_method("B4", CK, world_name="v2", **V[v])
        row, _ = run_episode("B4", scen, seed, log_decisions=False, world="v2", method=m)
    return v, scen, seed, row
if __name__ == "__main__":
    jobs = [(v, s, d) for v in V for s in SC for d in DAYS]
    with mp.get_context("fork").Pool(10) as p:
        res = p.map(job, jobs, chunksize=1)
    R = {(v, s, d): r for v, s, d, r in res}
    keys = [(s, d) for s in SC for d in DAYS]
    print("validation days (7 situations x seeds 2020-2029)")
    for v in V:
        on = [float(R[(v,) + k]["ontime30_per_h"]) for k in keys]; rd = [float(R[(v,) + k]["ready_time_mean_s"]) for k in keys]
        dif = [float(R[(v,) + k]["ontime30_per_h"]) - float(R[("B2",) + k]["ontime30_per_h"]) for k in keys]
        dr = [float(R[(v,) + k]["ready_time_mean_s"]) - float(R[("B2",) + k]["ready_time_mean_s"]) for k in keys]
        mu = sum(dif) / len(dif); se = (sum((x - mu) ** 2 for x in dif) / (len(dif) - 1)) ** 0.5 / math.sqrt(len(dif))
        mr = sum(dr) / len(dr); ser = (sum((x - mr) ** 2 for x in dr) / (len(dr) - 1)) ** 0.5 / math.sqrt(len(dr))
        print("%-14s on-time %6.2f  ready %4.0f s  | vs B2: on-time %+.2f ± %.2f, ready %+.0f ± %.0f s" % (v, sum(on) / len(on), sum(rd) / len(rd), mu, 2.09 * se, mr, 2.09 * ser))
