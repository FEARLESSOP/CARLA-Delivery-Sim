# Research record of the pod + drone RL (for the paper)

Findings so far (what the data shows, and which controlled comparisons would
back each claim): `docs/RL_DELIVERY.md` section 10.

Plain CSV / JSON, tracked in git, readable by pandas, R, Excel, MATLAB.
Rebuild the curated files with `python3 -m rl_delivery.research_log`
(from `hil_sim`); new runs are recorded automatically by `train.py`.

| File | What | One row = |
|---|---|---|
| `runs.csv` | every training run so far: brain, algorithm, world / reward version, curriculum, start / end (JST), steps, status (running, superseded, exploit, failed, changed, retired), outcome, why it ended, its progress file | one run |
| `changes.csv` | every change to worlds, rewards, curricula, algorithms, evaluation, and every bug, in time order, with the reason and the evidence | one change |
| `progress/<run_id>.csv` | the run's evaluation rows during training (columns per brain, see below) | one evaluation |
| `baselines.csv` + `baselines/*.json` | the scores to beat (CARLA rule fleet, rule flow, scripted driver / pilot, PID) | one baseline |
| `runs/<run_id>/config.json` | automatic, every run since 2026-11-10: arguments, all upper-case world / reward constants of the env module, git commit, uncommitted-changes flag | one run |
| `runs/<run_id>/progress.csv`, `runs_live.csv` | automatic: the run's progress copied at its end; one summary row per run | one run |

Progress columns (one evaluation on fixed test seeds):
- nav: steps, wall_s, wind_level, dist, reached (= landed on the pod), delivered, crash_pod, crash, crash_obstacle, crash_nofly, crash_ceiling, crash_battery, timeout, min_clear, battery_used, time_s, ep_reward
- fleet: round, brain (pod / drone trained that round), steps, wall_s, deliveries_per_h, f2f_per_h, drone_per_h, lead_mean_s, crashes_per_100, landing_min, hover_s_per_h, swaps, recoveries, replacements, stranded, charges, takeoff_crash, waiting
- drive: steps, wall_s, route_len, traffic, reached, kerb, collision, stop_violations, red_violations, centre_s, speeding_s, speed, ep_reward
- mission / flight: see the header row of each file.

Caveats to state in a paper:
- Evaluations use 12-16 test episodes per point (fleet: 4 x 3 h): noisy; for
  claims use several seeds per configuration and confidence intervals.
- Some early runs (nav_v2_hover, mission_v1) had their rows deleted at a fresh
  restart; their outcomes come from the notes in `runs.csv` / `changes.csv`.
- `fleet_v2` and `fleet_v3` share one progress file (rows appended); split by time.
- Several runs were resumed after a reward change: the checkpoint carried over,
  the reward did not stay the same (see `changes.csv`).

## Running experiments (queue)

1. Add rows to `queue.csv` (one row = one run; same `tag` = one comparison):

   ```
   id,tag,brain,algo,seed,hours,envs,args,status,started,ended,note
   n1,nav_algo,nav,recurrent_ppo,1,6,8,,pending,,,LSTM
   n2,nav_algo,nav,ppo,1,6,8,,pending,,,no memory
   n3,nav_algo,nav,sac,1,6,8,,pending,,,off-policy
   ```
   algorithms: nav `recurrent_ppo/ppo/sac/td3`, drive `sac/ppo/td3/recurrent_ppo`,
   fleet `maskable_ppo`; `args` = extra trainer flags (e.g. `--eval-every 50000`).
2. Start the runner: `systemctl --user start rl-queue` (keeps going without
   the agent; `--parallel 3`, nice 5) or by hand
   `python3 -m rl_delivery.queue_runner --parallel 3`. Stop taking new runs:
   `touch research/STOP_queue`; stop the service: `systemctl --user stop rl-queue`.
3. Each run lives in `exp/<tag>/<brain>_<algo>_s<seed>_<time>/` (config,
   progress; models git-ignored), never touching the main brains.
4. Compare: `python3 -m rl_delivery.compare --tag nav_algo` ->
   `exp/<tag>/summary.csv, table.csv, table.md, curves.png` (mean ± std over seeds).
