#!/bin/bash
# Paper 1 world v2 (docs/PAPER1_BASELINES.md 3b): wait for the v1 run (pid 1832687) to
# finish, then train B1-B3 (3 seeds, 3 methods in parallel per seed) in world v2 and
# evaluate every final model on the 7 scenarios x 30 paired seeds (tags <M>_s<seed>_v2).
# B0 / always-pod v2 results already exist (tags B0_v2, POD_v2).
# v1 の学習の終了を待ち、世界 v2 で B1〜B3 を学習・評価する。
cd "$(dirname "$0")/../../.."          # hil_sim
PY=../../carla-venv/bin/python
while kill -0 1832687 2>/dev/null; do sleep 60; done
echo "v1 run finished $(date '+%F %H:%M'); starting world v2"
for seed in 0 1 2; do
  for m in B1 B2 B3; do
    $PY -W ignore -m rl_delivery.paper1.train --method $m --seed $seed --steps 300000 --envs 6 --eval-every 30000 --world v2 &
  done
  wait
  for m in B1 B2 B3; do
    $PY -W ignore -m rl_delivery.paper1.evaluate --method $m --world v2 \
        --checkpoint rl_delivery/checkpoints/paper1/v2/$m/s$seed/final.zip --seeds 30 --workers 12 --tag ${m}_s${seed}_v2
  done
done
echo "ALL V2 RUNS DONE $(date '+%F %H:%M')"
