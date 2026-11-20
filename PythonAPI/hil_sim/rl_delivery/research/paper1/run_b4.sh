#!/bin/bash
# B4 (rollout + B2 prior / continuation / critic, confident override) in world v2:
# full evaluation on the 7 scenarios x 30 paired test days, once per B2 training seed.
# Tags B4_s<seed>_v2. / B4 を世界 v2 で評価（B2 の各学習シードごと）。
cd "$(dirname "$0")/../../.."          # hil_sim
PY=../../carla-venv/bin/python
for seed in 0 1 2; do
  $PY -W ignore -m rl_delivery.paper1.evaluate --method B4 --world v2 \
      --checkpoint rl_delivery/checkpoints/paper1/v2/B2/s$seed/final.zip --seeds 30 --workers 12 --tag B4_s${seed}_v2
done
echo "ALL B4 RUNS DONE $(date '+%F %H:%M')"
