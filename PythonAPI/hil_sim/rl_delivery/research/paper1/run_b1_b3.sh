#!/bin/bash
# Train B1-B3 (3 seeds each, 3 methods in parallel per seed), then evaluate every
# final model on the 7 scenarios x 30 paired seeds. Logs: research/paper1/run_b1_b3.log
# B1〜B3 を学習（各 3 シード）し、全シナリオ x 30 シードで評価する。
cd "$(dirname "$0")/../../.."          # hil_sim
PY=../../carla-venv/bin/python
for seed in 0 1 2; do
  for m in B1 B2 B3; do
    $PY -W ignore -m rl_delivery.paper1.train --method $m --seed $seed --steps 300000 --envs 6 --eval-every 30000 &
  done
  wait
  for m in B1 B2 B3; do
    $PY -W ignore -m rl_delivery.paper1.evaluate --method $m --checkpoint rl_delivery/checkpoints/paper1/$m/s$seed/final.zip \
        --seeds 30 --workers 12 --tag ${m}_s$seed
  done
done
echo "ALL B1-B3 RUNS DONE"
