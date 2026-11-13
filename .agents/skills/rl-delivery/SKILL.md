---
name: rl-delivery
description: "Reinforcement learning for the LastMile pods and drones in hil_sim/rl_delivery: the fleet world (pod + drone decision brains trained in turns), pod driving (kerbs, 止まれ, signals), drone navigation (find the way, deliver, land on the pod), trainers, the dashboard AI training page and watch mode, baselines and the reward tricks found so far. Use for any RL training, checking, tuning or watching work."
---

# RL delivery (Antigravity entry point)

1. Read `/home/inouelab/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/docs/RL_DELIVERY.md`
   completely: the user's flow, the rules (nothing hardcoded, pure RL, no geofence), brains,
   files, commands, baselines, reward tricks, curricula, traps, next steps.
2. KPIs: `PythonAPI/hil_sim/docs/DELIVERY_KPI.md`. Putting brains into CARLA (plan, how-to,
   tests): `PythonAPI/hil_sim/docs/RL_CARLA_BRIDGE.md`.
3. ROLES (user decision 2026-11-10): **Antigravity does the 30-minute checks; Claude Code
   starts, stops and restarts the trainers.** Antigravity must NOT start, stop, kill or
   restart a trainer itself. When a trainer is dead, frozen, duplicated, or needs a restart
   after an env / reward change, write the request (brain, what you saw, the change made,
   `--resume` or fresh, why) at the top of
   `/home/inouelab/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/rl_delivery/HANDOFF.md` and tell
   the user; Claude Code carries it out. Env / reward code changes are fine to make (pure RL,
   quick-test them) but take effect only after Claude restarts that trainer.
4. Checking the trainings (the user wants it every 30 min): exactly one trainer per brain
   (`ps -eo pid,etime,args | grep "[r]l_delivery.train"`), read `rl_delivery/results/train_<brain>.log`
   and `<brain>_progress.csv`, compare with the baselines, watch for a failure type rising while
   the success number stays flat (a reward trick). Fix with world rules or rewards only, test the
   env quickly, then hand the restart to Claude via HANDOFF.md (see 3). Report in plain
   English per brain.
5. Method, checks and traps of the whole rig: `/home/inouelab/Downloads/CARLA_0.9.16/AGENTS.md`.
