#!/usr/bin/env python
"""Train the ego agent with SAC on CarlaHILEnv (docs/RL_DESIGN.md §5).

Requires a running CARLA server, plus stable-baselines3:
    pip install stable-baselines3

Usage / 使い方 (from hil_sim/, CARLAを起動してから):
    python -m rl.train_sac --timesteps 200000
    python -m rl.train_sac --resume rl/results/models/sac_ego.zip --timesteps 100000
    python -m rl.train_sac --arm B --seed 1      # tensorboard run "B_s1"

Outputs go under rl/results/ (rl/paths.py): models/ and tb/.
出力は rl/results/ 配下（models/ と tb/、rl/paths.py 参照）。

For the study's matched arms (traffic + map + seeds handled for you) use
`python -m rl.run_arm` instead; it calls train() below.
研究のアーム比較（交通・マップ・シードの一括管理）には
`python -m rl.run_arm` を使う。内部で下の train() を呼ぶ。

Populate traffic first for interaction learning (generate_traffic.py with
driver-profile error parameters — see rl/extract_scenarios.py output).
交通と相互作用させる学習では先に generate_traffic.py で交通を生成する
（rl/extract_scenarios.py が出力するドライバープロファイルのエラー
パラメータを使うと「人間らしい」交通になる）。
"""

import argparse
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl.carla_env import CarlaHILEnv  # noqa: E402
from rl import paths  # noqa: E402


def train(env, timesteps, save, seed=0, resume=None, run_name=None,
          ckpt_every=20_000, tb_dir=None, torch_threads=4):
    """Train SAC on `env`; checkpoints next to `save`, scalars in tb_dir.

    `run_name` is the tensorboard run directory (e.g. "B_s1"): the arm
    letter in front is how rl/plot_progress.py groups seeds into arms, so
    keep the "<arm>_s<seed>" shape when the run belongs to the study.
    `torch_threads` caps torch's intra-op parallelism: this workstation is
    suspected of CPU instability under heavy parallel load (see
    docs/TROUBLESHOOTING.md), and an MLP this small gains nothing from 20
    threads anyway. Every checkpoint also saves the replay buffer (only
    the newest is kept) so `resume` continues faithfully after a crash or
    a machine freeze. Returns the trained model.

    `torch_threads` は torch の並列度上限。この PC は並列高負荷で不安定
    という疑いがあり（docs/TROUBLESHOOTING.md）、この規模の MLP に 20
    スレッドは不要。チェックポイント毎にリプレイバッファも保存（最新のみ
    保持）し、クラッシュや PC フリーズ後の `resume` を忠実にする。

    `env` で SAC を学習する。チェックポイントは `save` と同じ場所、
    スカラーは tb_dir に出る。`run_name` は tensorboard の run ディレクトリ
    名（例 "B_s1"）。先頭のアーム文字で rl/plot_progress.py がシードを
    アームにまとめるため、研究の run では "<アーム>_s<シード>" の形を保つ。
    """
    import torch
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import CheckpointCallback

    if torch_threads and torch_threads > 0:
        torch.set_num_threads(int(torch_threads))
        print('[train] torch threads = %d' % torch.get_num_threads())
    if tb_dir is None:
        tb_dir = paths.TB_DIR
    save_dir = os.path.dirname(save) or '.'
    os.makedirs(save_dir, exist_ok=True)
    ckpt = _LatestBufferCheckpoint(save_freq=ckpt_every, save_path=save_dir,
                                   name_prefix='sac_ego_ckpt',
                                   save_replay_buffer=True)
    if resume:
        model = SAC.load(resume, env=env)
        buf = _buffer_path_for(resume)
        if buf and os.path.exists(buf):
            model.load_replay_buffer(buf)
            print('[train] resumed from %s + replay buffer (%d transitions)'
                  % (resume, model.replay_buffer.size()))
        else:
            print('[train] resumed from %s (no replay buffer found; the '
                  'first %d steps refill it) / バッファ無しで再開'
                  % (resume, model.learning_starts))
    else:
        model = SAC('MlpPolicy', env, verbose=1, seed=seed,
                    learning_starts=2_000, train_freq=1,
                    tensorboard_log=tb_dir)
    try:
        # log_interval=1: SB3 logs rollout stats every N *episodes* (4 by
        # default). Episodes here run up to 1800 steps, so a short run
        # would leave the tensorboard curve empty.
        # log_interval=1：SB3 は N エピソード毎（既定4）にしか記録しない。
        # 1エピソード最大1800ステップのため、短い run では曲線が空になる。
        model.learn(total_timesteps=timesteps, callback=ckpt,
                    reset_num_timesteps=not resume,
                    tb_log_name=run_name or 'SAC', log_interval=1)
    finally:
        model.save(save)
        print('[train] saved -> %s' % save)
    return model


def _buffer_path_for(ckpt_zip):
    """`.../sac_ego_ckpt_40000_steps.zip` -> its replay-buffer pickle, as
    named by SB3's CheckpointCallback. / SB3 の命名規則で対応する pkl。"""
    import re
    m = re.match(r'(.*?)_(\d+)_steps\.zip$', ckpt_zip)
    if not m:
        return None
    return '%s_replay_buffer_%s_steps.pkl' % (m.group(1), m.group(2))


def _latest_checkpoint_callback_base():
    from stable_baselines3.common.callbacks import CheckpointCallback
    return CheckpointCallback


class _LatestBufferCheckpoint(_latest_checkpoint_callback_base()):
    """CheckpointCallback that keeps only the NEWEST replay-buffer pickle.

    The buffer is the big file (hundreds of MB); one copy is enough to
    resume, and keeping every one would fill the disk over a study.
    / リプレイバッファは巨大なので最新の1つだけ残す。
    """

    def _on_step(self):
        prev = None
        if self.save_replay_buffer and self.n_calls % self.save_freq == 0:
            import glob
            prev = sorted(glob.glob(os.path.join(
                self.save_path, self.name_prefix + '_replay_buffer_*_steps.pkl')))
        out = super()._on_step()
        if prev:
            newest = self._checkpoint_path('replay_buffer_', extension='pkl')
            for old in prev:
                if os.path.abspath(old) != os.path.abspath(newest):
                    try:
                        os.remove(old)
                    except OSError:
                        pass
        return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--timesteps', type=int, default=200_000)
    p.add_argument('--save',
                   default=os.path.join(paths.MODELS_DIR, 'sac_ego.zip'))
    p.add_argument('--resume', default=None,
                   help='existing model .zip to continue / 続きから学習')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--arm', default=None,
                   help='study arm letter; names the tensorboard run '
                        '"<arm>_s<seed>" / 研究アーム。tensorboard の run 名')
    p.add_argument('--torch-threads', type=int, default=4,
                   help='torch intra-op threads (low = stable on this CPU) '
                        '/ torch スレッド数（少なめが安定）')
    args = p.parse_args()
    paths.ensure_dirs()

    try:
        import stable_baselines3  # noqa: F401
    except ImportError:
        print('stable-baselines3 is required: pip install stable-baselines3\n'
              'stable-baselines3 が必要です: pip install stable-baselines3')
        return 1

    env = CarlaHILEnv(host=args.host, port=args.port, seed=args.seed)
    try:
        train(env, args.timesteps, args.save, seed=args.seed,
              resume=args.resume,
              run_name=('%s_s%d' % (args.arm, args.seed)) if args.arm else None,
              torch_threads=args.torch_threads)
    finally:
        env.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
