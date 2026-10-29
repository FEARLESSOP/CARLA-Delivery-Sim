"""One place for where the RL layer writes its results.

Everything a training or evaluation run produces lands under
`rl/results/` so a run can be archived, diffed or wiped as one folder:

    rl/results/
      models/<arm>_<map>_s<seed>/   SAC checkpoints + final model + run.json
      tb/<arm>_s<seed>_N/           tensorboard scalars (rl/train_sac.py)
      eval_history.csv              one row per evaluated checkpoint
      figures/                      progress.png, results.png, results_table.md,
                                    <map>_events.png, <map>_metrics.png
      logs/                         per-run console log + machine-health
                                    snapshot (rl/health.py) -- written from
                                    the FIRST second, so a run that freezes
                                    the machine still leaves a trace

Only README.md in that folder is tracked by git (see .gitignore).

RL 層の出力先を1か所で定義する。学習・評価の成果物はすべて
`rl/results/` 配下に置き、1フォルダ単位で保存・比較・削除できる。
models/ はチェックポイントと run.json、tb/ は tensorboard、
eval_history.csv は評価行、figures/ は図表、logs/ は各 run のコンソール
ログと機器の健康状態スナップショット（開始直後に書くので、途中で PC が
固まっても痕跡が残る）。git が追跡するのは README.md のみ。
"""

import os

HIL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(HIL_DIR, 'rl', 'results')
MODELS_DIR = os.path.join(RESULTS_DIR, 'models')
TB_DIR = os.path.join(RESULTS_DIR, 'tb')
FIGURES_DIR = os.path.join(RESULTS_DIR, 'figures')
LOGS_DIR = os.path.join(RESULTS_DIR, 'logs')
EVAL_CSV = os.path.join(RESULTS_DIR, 'eval_history.csv')


def ensure_dirs():
    """Create the results tree (idempotent). / 出力ツリーを作る。"""
    for d in (RESULTS_DIR, MODELS_DIR, TB_DIR, FIGURES_DIR, LOGS_DIR):
        os.makedirs(d, exist_ok=True)
    return RESULTS_DIR


def run_name(arm, map_name, seed):
    """`A_Town02_s0` -- the folder name of one study run. / 1 run の名前。"""
    return '%s_%s_s%d' % (arm, map_name, seed)


def model_dir(arm, map_name, seed):
    return os.path.join(MODELS_DIR, run_name(arm, map_name, seed))
