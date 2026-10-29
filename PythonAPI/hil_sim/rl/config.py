"""Study-wide knobs you may want to change -- edit HERE, nowhere else.

Every RL script reads its defaults from this file, so switching the town
the study is run and analysed on is one edit. The command-line flags
(`--map`, `--eval-traffic`) still override per call.

研究全体の設定はここだけを編集する。各スクリプトはここから既定値を読む
ので、対象の町を変えるのは1行の変更で済む。コマンドライン引数
（`--map`、`--eval-traffic`）で呼び出し毎の上書きもできる。
"""

# The CARLA town the study is trained, evaluated and analysed on. Must be
# a map the server has (`Town01` ... `Town07`, `Town10HD_Opt`, ...) and,
# for arm B and the human reference lines, one with human sessions under
# data/ (rl.extract_scenarios writes per-map profiles; rl.plot_town_events
# and the human reference use the same map name).
# 学習・評価・分析の対象となる町。サーバに存在するマップ名で、アームBと
# 人間の参照線には data/ にその町の人間セッションが必要。
STUDY_MAP = 'Town02'

# Traffic every checkpoint is evaluated in (rl/run_arm.py EVAL_TRAFFIC):
# 'human_x1' = measured human error rates, 'default' = plain TrafficManager,
# 'human_x2' = doubled rates. Plots only compare rows with the same label.
# 全チェックポイントの評価交通。プロットは同じラベルの行同士だけ比較する。
EVAL_TRAFFIC = 'human_x1'
