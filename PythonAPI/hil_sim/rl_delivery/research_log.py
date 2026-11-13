"""Build the research record of the RL work (for the paper) in rl_delivery/research/.

  python3 -m rl_delivery.research_log        # rebuild / update; safe to run any time

Writes (all plain text, tracked in git, readable by pandas / R / Excel):
- runs.csv        one row per training run: brain, algorithm, settings, world /
                  reward version, start / end, steps, status, outcome, why it
                  ended, its progress file
- changes.csv     every change to worlds, rewards, curricula, algorithms and
                  every bug found, in time order, with the reason and evidence
- progress/<run_id>.csv   the run's evaluation rows (copied from results/ and
                  the archived checkpoints, or parsed from its log)
- baselines/      the baselines (CARLA rule fleet, rule flow, scripted
                  drivers / pilots) as JSON + one flat baselines.csv
- runs_live.csv   rows appended automatically by train.py for every new run
                  (config snapshot incl. reward constants and git commit)

研究用の記録（実行一覧・変更履歴・進捗・基準）を research/ に作る。
"""
import csv
import glob
import json
import os
import re
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
OLD = os.path.join(HERE, "checkpoints", "_old")
OUT = os.path.join(HERE, "research")

# Every training run so far, as recorded during the work (2026-11-09 .. 11-10, JST).
# progress = source of its evaluation rows ("log:<file>" = parsed from a log;
# "" = rows were deleted when the run was restarted fresh: outcome from the notes).
RUNS = [
    dict(run_id="flight_v1", brain="drone_flight", algorithm="RecurrentPPO (MlpLstmPolicy, lstm 128, net 128x128)",
         n_envs=12, world="env_flight v1: follow a given 3-D route in hidden wind; no-fly safety guard",
         reward="progress 0.1/m, cross-track -0.04/m -0.004/m2, speed error, smoothness; reached +20; crash -30; guard -1/step",
         curriculum="wind level +0.2 when >=80 % reached and x-track < 3 m",
         started="2026-11-09 13:30", ended="2026-11-09 15:56", steps=4000008, status="superseded",
         outcome="full wind, 100 % reached, 0.72 m mean cross-track, 0 % crash (12-flight tests); beat PID (90 %, 1.38 m, 10 % crash)",
         ended_why="user: no route may be given (all behaviour RL) -> replaced by drone navigation",
         progress="log:train_flight.log"),
    dict(run_id="mission_v1", brain="drone_mission", algorithm="MaskablePPO (MlpPolicy 128x128)", n_envs=5,
         world="env_mission v1: one pod zone, rule pod round, 4 drones, quick flight stand-in",
         reward="delivery +10 - 0.004*lead; crash -40 (only at decisions); queue -0.002/order/decision",
         curriculum="none", started="2026-11-09 14:58", ended="2026-11-09 16:34", steps=5850000, status="changed",
         outcome="19-24 deliveries/h vs rule 11.5, but 7.6-13.7 crashes/100 vs rule 4.2",
         ended_why="audit: crashes while hovering/chasing were free; crash too cheap -> v2 rewards", progress=""),
    dict(run_id="mission_v2", brain="drone_mission", algorithm="MaskablePPO (resumed from mission_v1)", n_envs=5,
         world="env_mission v2", reward="crash -150 everywhere; landing < 20 % battery -2 per %",
         curriculum="none", started="2026-11-09 16:36", ended="2026-11-10 12:21", steps=18500000, status="retired",
         outcome="~13-19 deliveries/h, 0-6 crashes/100 (rule 11.5 / 4.2), lead ~4-5 min (rule 15.5 min)",
         ended_why="replaced by the fleet world (user's full flow); a later run was started from the dashboard",
         progress="results/mission_progress.csv"),
    dict(run_id="nav_v1_climb", brain="drone_navigation", algorithm="RecurrentPPO (lstm 192, net 256x256)", n_envs=12,
         world="env_nav v1: one leg to the drop, solid buildings/trees (2 m height map), pure RL",
         reward="progress 0.05/m (new best only), time -0.01, closeness < 3 m cost, crash -50, reached +50",
         curriculum="dist 200 -> +150 to 900 at >=70 % reached, then wind", started="2026-11-09 15:20",
         ended="2026-11-09 15:51", steps=800004, status="exploit",
         outcome="0 % reached; climbed to ~1,770 m until the battery died", ended_why="reward hack: climbing avoids every obstacle",
         progress="old:nav_climbing/nav_progress.csv"),
    dict(run_id="nav_v2_hover", brain="drone_navigation", algorithm="RecurrentPPO", n_envs=12,
         world="env_nav v2: + 150 m ceiling (failure), height cost, battery running cost",
         reward="v1 + ceiling -50, height > 60 m cost, battery -0.02/% drained", curriculum="as v1",
         started="2026-11-09 15:56", ended="2026-11-09 16:05", steps=300000, status="exploit",
         outcome="12 % crash, 0 % reached, 88 % hovered until the time limit", ended_why="reward hack: hover safely until time-out",
         progress=""),
    dict(run_id="nav_v3_circle", brain="drone_navigation", algorithm="RecurrentPPO", n_envs=12,
         world="env_nav v3: time-out = failure", reward="v2 + time-out -50", curriculum="as v1",
         started="2026-11-09 16:06", ended="2026-11-09 16:17", steps=1300008, status="changed",
         outcome="0 % reached at 200 m trips; seen circling the drop in watch mode",
         ended_why="weak progress signal; short trips needed", progress="old:nav_hover/nav_progress.csv"),
    dict(run_id="nav_v4_oneleg", brain="drone_navigation", algorithm="RecurrentPPO", n_envs=12,
         world="env_nav v4", reward="potential-based closeness (0.3/m + 25*exp(-d/15)), circling cost 0.02*tangential speed, reached +100",
         curriculum="dist 80 -> +60 to 200 -> +150 to 900 at >=70 %, then wind", started="2026-11-09 16:25",
         ended="2026-11-09 16:50", steps=1400004, status="superseded",
         outcome="69 % reached on 140 m trips", ended_why="user: drones must also fly to the meeting point and land on the pod (two legs)",
         progress="old:nav_oneleg/nav_progress.csv"),
    dict(run_id="nav_v5_twoleg", brain="drone_navigation", algorithm="RecurrentPPO", n_envs=12,
         world="env_nav v5: drop hover (+60) then land on the pod at a meeting point (+100); leg flag in observation; 480 s",
         reward="v4 per leg; pod crash -50", curriculum="as v4, on landings",
         started="2026-11-09 16:51", ended="(running)", steps=None, status="running",
         outcome="see progress; 41 % fallback-trip bug fixed 11-10 10:2x; landing rule loosened 11-10 11:4x; "
                 "at 6.3 M steps: 200 m trips, 88 % delivered, 69 % landed",
         ended_why="", progress="results/nav_progress.csv"),
    dict(run_id="fleet_v1_idle", brain="fleet (pod+drone)", algorithm="MaskablePPO (net 256x256), turns of 200k steps",
         n_envs=6, world="env_fleet v1: 3 pods, 4 drones each, standbys, lockers, stations, EV batteries, crashes, recovery",
         reward="delivery +10 - 0.004*lead; crash -150; stranded -200; queue -0.0005/order/decision; wait -0.2",
         curriculum="none", started="2026-11-10 09:47", ended="2026-11-10 10:04", steps=202752, status="exploit",
         outcome="pod brain waited until all pods were flat (0 deliveries)", ended_why="reward hack: idling",
         progress="old:idle/fleet_progress.csv"),
    dict(run_id="fleet_v2", brain="fleet (pod+drone)", algorithm="MaskablePPO, turns", n_envs=6,
         world="env_fleet v2", reward="delivery +20; waiting parcel -0.05/parcel/min (world time); stranded -500",
         curriculum="none", started="2026-11-10 10:19", ended="2026-11-10 12:13", steps=1061412, status="changed",
         outcome="deterministic tests showed 'always wait' (artefact; sampled: 29 deliveries/h, 0 stranded early); "
                 "later rounds: pods still stranded, ~7-12 deliveries/h",
         ended_why="credit problem: per-decision rewards made 30 s waits look cheap", progress="results/fleet_progress.csv"),
    dict(run_id="fleet_v3", brain="fleet (pod+drone)", algorithm="MaskablePPO, turns; fresh pod brain, drone brain kept",
         n_envs=6, world="env_fleet v3", reward="v2 + low pod battery cost below 25 %; waiting with work available -3 (else -0.2)",
         curriculum="none", started="2026-11-10 12:15", ended="2026-11-10 14:2x", steps=None, status="changed",
         outcome="first rounds froze / restarted; no reliable result", ended_why="order rate randomised (fleet_v4)",
         progress="old:fleet_fixed_rate/fleet_progress.csv"),
    dict(run_id="fleet_v4_randrate", brain="fleet (pod+drone)", algorithm="MaskablePPO, turns; fresh pod brain (obs changed), drone kept",
         n_envs=6, world="env_fleet v4: order rate random 30-90/h per episode, rate observed",
         reward="as fleet_v3", curriculum="domain randomisation of the order rate", started="2026-11-10 14:2x",
         ended="(running)", steps=None, status="running", outcome="see progress (KPIs at 47/h + 30 and 72/h)",
         ended_why="", progress="results/fleet_progress.csv"),
    dict(run_id="drive_v1_ppo_idle", brain="pod_driving", algorithm="PPO (net 256x256)", n_envs=6,
         world="env_drive v1: real lanes, kerb/lane/centre edges, 74 stop lines, 29 signal heads, IDM traffic",
         reward="progress 0.1/m; kerb -40; centre -0.5/step; stop -25; red -40; collision -50; reached +30; time-out -20",
         curriculum="route 150 -> +100 to 800, then traffic +1 to 6, at >=75 % reached and lawful",
         started="2026-11-10 09:59", ended="2026-11-10 10:13", steps=4800000, status="exploit",
         outcome="speed 0: stood still", ended_why="reward hack: time-out cheaper than a kerb strike",
         progress="old:idle/drive_progress.csv"),
    dict(run_id="drive_v2_ppo", brain="pod_driving", algorithm="PPO", n_envs=6, world="env_drive v2",
         reward="time-out -40; standing still without a reason -0.05/step", curriculum="as v1",
         started="2026-11-10 10:16", ended="2026-11-10 10:44", steps=7200000, status="failed",
         outcome="actions pinned at full brake / full lock (clipped Gaussian saturated)", ended_why="switch to SAC",
         progress="old:drive_ppo/drive_progress.csv"),
    dict(run_id="drive_v3_sac", brain="pod_driving", algorithm="SAC (net 256x256, buffer 1M, batch 256)", n_envs=6,
         world="env_drive v2", reward="as drive_v2; from 11-10 12:2x progress 0.3/m (was 0.1: it crawled)",
         curriculum="as v1", started="2026-11-10 10:45", ended="2026-11-10 13:0x", steps=2353974, status="changed",
         outcome="never got moving: 0.08 m/s deterministic, 12 % reached, all time-outs",
         ended_why="asymmetric pedal (2.5 forward / 6.0 brake): random exploration braked more than it accelerated",
         progress="old:drive_sac_asym/drive_progress.csv"),
    dict(run_id="drive_v4_sac_sym", brain="pod_driving", algorithm="SAC", n_envs=6,
         world="env_drive v3: symmetric pedal +-3 m/s2", reward="as drive_v3 (progress 0.3/m)",
         curriculum="as v1", started="2026-11-10 13:0x", ended="(running)", steps=None, status="running",
         outcome="see progress", ended_why="", progress="results/drive_progress.csv"),
]

# Changes in time order (worlds, rewards, curricula, algorithms, bugs, evaluation).
CHANGES = [
    ("2026-11-09 13:2x", "drone_flight", "world", "PID baseline: corner braking (v_ref) and forward-only path projection",
     "PID overshot corners by 6-10 m and deadlocked at hairpins", "PID calm: 29/30 reached, 1.25 m"),
    ("2026-11-09 13:2x", "all", "infrastructure", "OMP_NUM_THREADS=1 per env worker; one trainer per brain",
     "two trainers + thread oversubscription made training crawl", "~1.3k -> ~8k steps/s"),
    ("2026-11-09 14:3x", "mission", "world", "battery swap in the pod, crashed drones replaced after 15 min, drones chase a departed pod",
     "rule baseline lost all 4 drones every episode (flat batteries)", "rule: 10 deliveries/h, 4 crashes/100"),
    ("2026-11-09 15:4x", "nav", "world", "2 m height map of buildings, tree trunks/canopies, poles from the map meshes",
     "user: drones must not hit trees or buildings", "ground matches CARLA roads within 0.15 m"),
    ("2026-11-09 15:5x", "nav", "reward", "150 m ceiling (failure), height > 60 m cost, battery running cost",
     "nav_v1 climbed to ~1,770 m", ""),
    ("2026-11-09 16:0x", "mission", "reward", "crash -150 wherever it happens; low-battery landing cost",
     "crashes while hovering/chasing were free; 1-in-4 crash risk still paid", "audit: 22 crashes / 10 h, 15 battery"),
    ("2026-11-09 16:0x", "nav", "reward", "time-out = failure (-50)", "nav_v2 hovered until time-out (88 %)", ""),
    ("2026-11-09 16:2x", "nav", "reward", "potential-based closeness reward + circling cost; reached +100",
     "user saw circling at the drop", ""),
    ("2026-11-09 16:2x", "nav", "curriculum", "start at 80 m trips (+60 m steps to 200)", "0 % reached on 200 m trips", ""),
    ("2026-11-09 16:5x", "nav", "world", "two legs: drop, then land on the pod at a meeting point; leg flag in the observation",
     "user: second destination like the delivery sim", "scripted: 19/20 landed"),
    ("2026-11-09 17:xx", "dashboard", "infrastructure", "CARLA helper threads skip CARLA while its port is closed",
     "a failed connect held the GIL: every page froze 5-25 s", "40 requests ~1.5 ms"),
    ("2026-11-10 09:2x", "fleet", "world", "fleet world built from the user's flow; sites from charge_sites/boxes; 4 drones/pod",
     "user's full delivery flow", "rule flow 41.5 deliveries/h, 15.3 crashes/100"),
    ("2026-11-10 09:3x", "fleet", "bug", "drone charging in the pod drew 100x too much from the pod battery", "all pods stranded in the rule flow", ""),
    ("2026-11-10 09:4x", "drive", "world", "lane edges classified geometrically (netconvert: each direction its own road); routes resampled at 1 m",
     "99 % 'kerb' edges; xodr headings off at piece ends", "scripted: 31/40 reached"),
    ("2026-11-10 10:0x", "fleet", "reward", "delivery +20, waiting parcel -0.05/min, stranded -500", "fleet_v1 idled", "always-wait -2617 vs rule -715"),
    ("2026-11-10 10:0x", "drive", "reward", "time-out -40; idle without reason -0.05/step", "drive_v1 stood still", "standing still -195"),
    ("2026-11-10 10:2x", "nav", "bug", "trip picker searched 40 random nodes and fell back to a fake trip (drop inside a building)",
     "41 % of 80 m practice trips were broken", "0 % fallback after the fix"),
    ("2026-11-10 10:4x", "fleet", "evaluation", "tests and the other brain sample actions instead of argmax",
     "deterministic test showed 'always wait' though sampled behaviour delivered 29/h", ""),
    ("2026-11-10 10:4x", "drive", "algorithm", "PPO -> SAC", "PPO outputs pinned at full brake/lock", ""),
    ("2026-11-10 11:4x", "nav", "world", "landing rule 1.0/1.0/0.8 -> 1.3 m / 1.5 m/s / 1.2 m/s",
     "0 % landed for ~2.6 M steps", "56 % landed one check later"),
    ("2026-11-10 12:1x", "fleet", "reward", "low pod battery cost below 25 %", "pods never charged (3 stranded)", "never-charge -2587 vs rule -757"),
    ("2026-11-10 12:1x", "fleet", "reward", "waiting with work available -3 (else -0.2); fresh pod brain",
     "pod waited 328/359 decisions (per-decision credit made waits cheap)", "always-wait -3821 vs rule -774"),
    ("2026-11-10 12:2x", "drive", "reward", "progress 0.1 -> 0.3 per metre", "crawled at 0.6-0.9 m/s, 12-19 % reached", ""),
    ("2026-11-10 12:2x", "research", "infrastructure", "research record, automatic run configs, --algo/--seed/--tag, experiment queue, compare",
     "user: research paper comparing algorithms and changes", ""),
    ("2026-11-10 13:0x", "drive", "world", "pedal made symmetric: +-3.0 m/s2 (was +2.5 / -6.0)",
     "SAC never got moving (0.08 m/s): random exploration braked more than it accelerated",
     "random actions now average 1.6 m/s; scripted 19/20 reached"),
    ("2026-11-10 13:3x", "fleet", "bug", "trainer froze: waited forever on vanished worker processes", "no checkpoint for 1 h", "killed by PID, restarted"),
    ("2026-11-10 14:2x", "fleet", "world", "order rate random per episode 30-90/h (domain randomisation); rate in the pod observation; tests at 30 / 47 (CARLA) / 72 per h",
     "user: train at a high rate, deploy to CARLA's lower rate; brains learn the conditions they practise in",
     "rule flow @30/47/72: 26.8/36.0/36.3 deliveries/h, waiting 12/33.5/105"),
    ("2026-11-10 14:2x", "roles", "infrastructure", "Antigravity runs the checks, Claude Code starts/restarts trainers (HANDOFF.md)", "two agents managing the same trainers", ""),
]

BASELINES = [
    ("carla_rule_fleet", "today's rule fleet in CARLA, OitaKosen, 5 seeds x 30 min, 3 pods, 9 cars, 20 walkers",
     "pods 23.6 deliveries/h, lead 10.3 min; drones 23.6/h, 5.2 crashes/100, lowest landing 66 %", "baseline_OitaKosen.json"),
    ("fleet_rule_flow", "fleet_rules.py in the fleet world (4 x 3 h) at 47 orders/h (CARLA rate); earlier fixed ~72/h gave 41.5/h, 15.3 crashes",
     "47/h: 36.0 deliveries/h, 10.5 crashes/100, 0 stranded, 33.5 waiting; 30/h: 26.8/h, 12 waiting; 72/h: 36.3/h, 105 waiting",
     "fleet_rule_baseline.json"),
    ("mission_rule", "RuleMission in env_mission (6 x 1 h)", "11.5 deliveries/h, 4.2 crashes/100, 15.5 min lead, lowest landing 47 %",
     "mission_rule_baseline.json"),
    ("flight_pid", "PidPilot in env_flight (30 flights per wind level)", "calm 97 % / 1.25 m; full wind 90 % / 1.38 m / 10 % crash", ""),
    ("nav_scripted", "ScriptedPilot in env_nav (climb 45 m, straight, descend; no avoidance)",
     "one leg: 65 % reached, 30 % obstacle, 5 % no-fly; two legs <= 200 m: 19/20 landed", ""),
    ("drive_scripted", "ScriptedDriver in env_drive (40 routes, 400 m)", "31/40 reached, 0 stop/red violations, 3 kerb, 2.4 m/s", ""),
]


def flight_from_log(path, out):
    rows = []
    for line in open(path):
        m = re.match(r"\[train\] (\d+) steps\s+wind ([\d.]+)\s+reached (\d+)%\s+xtrack ([\d.]+) m\s+crash (\d+)%", line)
        if m:
            rows.append([int(m.group(1)), float(m.group(2)), int(m.group(3)) / 100.0, float(m.group(4)),
                         int(m.group(5)) / 100.0])
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["steps", "wind_level", "reached", "xtrack_mean", "crash"])
        w.writerows(rows)
    return len(rows)


def main():
    os.makedirs(os.path.join(OUT, "progress"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "baselines"), exist_ok=True)
    for r in RUNS:
        src, dst = r["progress"], os.path.join(OUT, "progress", r["run_id"] + ".csv")
        r["progress_rows"] = 0
        if src.startswith("log:"):
            r["progress_rows"] = flight_from_log(os.path.join(RES, src[4:]), dst)
        elif src.startswith("old:"):
            p = os.path.join(OLD, src[4:])
            if os.path.exists(p):
                shutil.copy(p, dst)
                r["progress_rows"] = sum(1 for _ in open(p)) - 1
        elif src:
            p = os.path.join(HERE, src)
            if os.path.exists(p):
                shutil.copy(p, dst)
                r["progress_rows"] = sum(1 for _ in open(p)) - 1
        r["progress_file"] = ("progress/%s.csv" % r["run_id"]) if r["progress_rows"] else "(deleted at a fresh restart)"
    cols = ["run_id", "brain", "algorithm", "n_envs", "world", "reward", "curriculum", "started", "ended", "steps",
            "status", "outcome", "ended_why", "progress_file", "progress_rows"]
    with open(os.path.join(OUT, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(RUNS)
    with open(os.path.join(OUT, "changes.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["when_jst", "brain", "type", "change", "reason", "evidence"])
        w.writerows(CHANGES)
    with open(os.path.join(OUT, "baselines.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["baseline", "what", "result", "file"])
        w.writerows(BASELINES)
    for b in BASELINES:
        if b[3] and os.path.exists(os.path.join(RES, b[3])):
            shutil.copy(os.path.join(RES, b[3]), os.path.join(OUT, "baselines", b[3]))
    print("research/: %d runs (%d with progress rows), %d changes, %d baselines" % (
        len(RUNS), sum(1 for r in RUNS if r["progress_rows"]), len(CHANGES), len(BASELINES)))


if __name__ == "__main__":
    main()
