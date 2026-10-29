#!/usr/bin/env python
"""Run one study arm end to end: map -> traffic -> train -> evaluate.

The research protocol (docs/RESEARCH_PROTOCOL.md §3) compares two learned
arms that differ ONLY in their training traffic:

    A   default TrafficManager traffic
    B   human-error traffic: TrafficManager driven by the km-weighted
        profile mined from this rig's human sessions
        (data/scenarios/map_profiles.json, rl/extract_scenarios.py)

Everything else is matched here by construction: same map, same SAC
settings (rl/train_sac.py), same reward (rl/rules.py), same timestep
budget, same seed list, and -- for the evaluation -- the SAME traffic for
both arms, so later rows of rl/results/eval_history.csv are comparable
across arms. rl/plot_progress.py turns that CSV into the result graph.

Usage / 使い方 (CARLA must be running, e.g. `CarlaUE4.sh -RenderOffScreen`):
    python -m rl.run_arm --arm A --seed 0 --timesteps 50000
    python -m rl.run_arm --arm B --seed 0 --timesteps 50000
    python -m rl.run_arm --arm A --seed 0 --eval-only     # re-evaluate ckpts
    python -m rl.run_arm --arm A --seed 0 --resume        # continue after a crash
    python -m rl.plot_progress                             # -> rl/results/figures/

Results / 出力: everything goes under rl/results/ (rl/paths.py) --
models/<arm>_<map>_s<seed>/ (checkpoints, run.json), tb/, eval_history.csv,
logs/<run>_<time>.log (console, flushed line by line) and
logs/<run>_<time>.health.json (machine state at start, rl/health.py).

Machine safety / 機器の保護: on 2026-09-14 a run froze the whole
workstation (hard lockup in the NVIDIA modeset kernel thread, see
docs/TROUBLESHOOTING.md). A user process cannot stop a kernel driver from
locking up, so this script does what it can:
  * refuses to start if this boot already shows GPU faults / lockups
    (--allow-faults overrides), and records a health snapshot first;
  * runs the world in no_rendering_mode (the env has no camera; physics,
    TrafficManager, collision and lane sensors are unaffected), so the
    GPU is nearly idle during training -- --render turns that off;
  * caps torch at --torch-threads (default 4) and nices itself (--nice):
    this CPU is suspected of Raptor Lake instability under parallel load;
  * checkpoints every --ckpt-every steps with the replay buffer, so
    --resume continues from the last checkpoint instead of starting over.

Tick ownership / ティックの所有: this process is the synchronous tick
master AND the TrafficManager server. generate_traffic.py is started with
--passive so it registers its vehicles with the TrafficManager here and
never ticks; CARLA only advances a synchronous TrafficManager from the
process that owns it, so the ticking process must own it.

PC 全体が固まった事故（2026-09-14、NVIDIA modeset カーネルスレッドの
ハードロックアップ）を受け、開始前の健康チェック（障害があれば拒否）、
即時 flush されるログと機器スナップショット、no_rendering_mode（env に
カメラは無い）、torch スレッド上限と nice、リプレイバッファ付き
チェックポイントからの --resume を備える。出力は rl/results/ 配下。

研究プロトコル（docs/RESEARCH_PROTOCOL.md §3）の1アームを一括実行する：
マップ読込 → 交通生成 → 学習 → チェックポイント評価。アームAは既定の
TrafficManager 交通、アームBは人間セッションから抽出した距離加重
プロファイル（data/scenarios/map_profiles.json）で運転する人間エラー交通。
マップ・SAC設定・報酬・ステップ数・シードは両アームで一致させ、評価は
両アームとも同一の交通で行うため、eval_history.csv の行はアーム間で
比較できる。本プロセスが同期ティックマスタかつ TrafficManager サーバで、
generate_traffic.py は --passive で車両登録のみ行う（同期 TrafficManager
はそれを所有するプロセスのティックでしか進まないため）。
"""

import argparse
import glob
import json
import os
import re
import signal
import subprocess
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl.carla_env import CarlaHILEnv, FIXED_DT  # noqa: E402
from rl import config, health, paths  # noqa: E402
import carla  # noqa: E402  (path set up by rl.carla_env)

HIL_DIR = paths.HIL_DIR
PROFILE_PATH = os.path.join(HIL_DIR, 'data', 'scenarios', 'map_profiles.json')
HEARTBEAT_PATH = os.path.join(HIL_DIR, 'traffic_heartbeat.json')
EVAL_CSV = paths.EVAL_CSV

# Set by main() once the log is open; child processes write into it too.
# main() でログを開いた後に設定。子プロセスの出力もここへ。
LOG = None


class Tee(object):
    """stdout/stderr -> terminal AND a line-buffered file.

    Line-buffered so the last line before a machine freeze is on disk.
    / 行バッファなので PC が固まる直前の行までディスクに残る。
    """

    def __init__(self, path):
        self.fh = open(path, 'a', buffering=1)
        self.term = sys.__stdout__

    def write(self, s):
        self.term.write(s)
        self.fh.write(s)

    def flush(self):
        self.term.flush()
        self.fh.flush()

    def fileno(self):
        return self.fh.fileno()

    def isatty(self):
        return False

# Traffic per arm: (error scale, label). Arm A is "profile off"; arm B is
# the measured human rates. Evaluation traffic is the same for every arm
# (docs/RESEARCH_PROTOCOL.md §4 secondary endpoint): human-error traffic
# at measured rates, i.e. "what the agent will meet around real humans".
# アーム毎の交通：（エラー係数, ラベル）。Aはプロファイル無し、Bは実測の
# 人間エラー率。評価交通は全アーム共通（RESEARCH_PROTOCOL §4 の副次評価）
# で、実測率の人間エラー交通＝「本物の人間の周りで出会う交通」。
ARMS = {
    'A': (0.0, 'default'),
    'B': (1.0, 'human_x1'),
}
EVAL_TRAFFIC = {
    'human_x1': 1.0,
    'default': 0.0,
    'human_x2': 2.0,
}


def ensure_map(client, map_name, timeout_s=120.0):
    """Load `map_name` unless it is already the current world.

    Reloading a map that is already up would destroy every actor and
    re-stream the town for nothing. / 既にロード済みなら再読込しない。
    """
    world = client.get_world()
    current = world.get_map().name.rsplit('/', 1)[-1]
    if current == map_name:
        print('[arm] map %s already loaded / 読込済み' % map_name)
        return world
    print('[arm] loading %s (was %s) / マップ読込' % (map_name, current))
    client.set_timeout(timeout_s)
    world = client.load_world(map_name)
    client.set_timeout(20.0)
    return world


def set_sync(world, on, no_rendering=False):
    """Synchronous fixed-step on/off; optionally no_rendering_mode.

    no_rendering_mode stops the server rendering anything at all while
    physics, TrafficManager, traffic lights and the collision / lane
    sensors keep working -- the env has no camera, so training loses
    nothing and the GPU sits almost idle. Always pass False on the way
    out so the next human session gets its picture back.
    / no_rendering_mode は描画だけを止め、物理・TrafficManager・信号・
    衝突/車線センサは動き続ける。env にカメラは無いので学習に影響なく
    GPU はほぼ遊ぶ。終了時は必ず False に戻す。
    """
    s = world.get_settings()
    s.synchronous_mode = on
    s.fixed_delta_seconds = FIXED_DT if on else None
    s.no_rendering_mode = bool(no_rendering)
    world.apply_settings(s)


def tick_until(world, predicate, timeout_s, label):
    """Tick the world at FIXED_DT until predicate() or timeout.

    While generate_traffic.py --passive is spawning, someone must keep
    the synchronous world moving. / 受動側の交通生成中もワールドを
    進め続ける必要がある。
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        world.tick()
        if predicate is not None and predicate():
            return True
        time.sleep(FIXED_DT)
    if predicate is not None:
        print('[arm] timed out waiting for %s / 待機タイムアウト: %s'
              % (label, label))
    return False


def traffic_alive(proc):
    try:
        with open(HEARTBEAT_PATH) as fh:
            hb = json.load(fh)
        return hb.get('pid') == proc.pid and time.time() - hb['wall_time'] < 5
    except (OSError, ValueError, KeyError):
        return False


def start_traffic(args, world, error_scale, seed):
    """Spawn NPC traffic as a --passive child and wait until it is in.

    Returns the Popen (None if --vehicles 0). / 受動モードで交通を生成し、
    登録完了まで待つ。
    """
    if args.vehicles <= 0:
        return None
    cmd = [sys.executable, '-u', os.path.join(HIL_DIR, 'generate_traffic.py'),
           '--host', args.host, '--port', str(args.port),
           '--tm-port', str(args.tm_port),
           '-n', str(args.vehicles), '-w', '0',
           '--passive', '--safe', '--seed', str(seed),
           '--spawn-chunk', str(args.spawn_chunk)]
    if error_scale > 0.0:
        cmd += ['--driver-profile', args.profile, '--error-scale',
                str(error_scale)]
    print('[arm] traffic: %s' % ' '.join(cmd[1:]))
    try:
        os.remove(HEARTBEAT_PATH)
    except OSError:
        pass
    out = LOG.fh if LOG is not None else None
    proc = subprocess.Popen(cmd, cwd=HIL_DIR, start_new_session=True,
                            stdout=out, stderr=subprocess.STDOUT if out else None)
    ok = tick_until(world, lambda: traffic_alive(proc) or proc.poll() is not None,
                    timeout_s=args.vehicles * 2.0 + 60.0, label='traffic')
    if proc.poll() is not None:
        raise RuntimeError('generate_traffic.py exited with %d '
                           '/ 交通生成が終了コード %d で落ちた'
                           % (proc.returncode, proc.returncode))
    if not ok:
        raise RuntimeError('traffic never reported a heartbeat '
                           '/ 交通のハートビートが来ない')
    # Let the TrafficManager settle its first plans before the ego
    # appears. / ego 出現前に TrafficManager の初期計画を落ち着かせる。
    tick_until(world, None, timeout_s=2.0, label='settle')
    n = len(world.get_actors().filter('vehicle.*'))
    print('[arm] traffic ready: %d vehicle(s) in world / 交通準備完了'
          % n)
    return proc


def stop_traffic(proc, world):
    """SIGTERM the traffic child and tick while it tears down.

    Its cleanup is chunked (GPU burst protection) and needs the world to
    keep advancing. / 破棄はチャンク化されワールドの進行が必要。
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except OSError:
        return
    tick_until(world, lambda: proc.poll() is not None, timeout_s=60.0,
               label='traffic teardown')
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGKILL)


def run_dir(args):
    return paths.model_dir(args.arm, args.map, args.seed)


def write_run_json(args, status, **extra):
    """run.json is written at START (status "running") and rewritten at
    the end, so a run that took the machine down is still identifiable.
    / 開始時に "running" で書き、終了時に上書き。固まった run も判別可。"""
    path = os.path.join(run_dir(args), 'run.json')
    doc = {}
    if os.path.exists(path):
        try:
            with open(path) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            doc = {}
    error_scale, label = ARMS[args.arm]
    doc.update({'arm': args.arm, 'seed': args.seed, 'map': args.map,
                'train_traffic': label, 'error_scale': error_scale,
                'vehicles': args.vehicles, 'timesteps': args.timesteps,
                'no_rendering': not args.render,
                'torch_threads': args.torch_threads,
                'status': status, 'wall_time': time.time(),
                'iso_time': time.strftime('%Y-%m-%dT%H:%M:%S')})
    doc.update(extra)
    os.makedirs(run_dir(args), exist_ok=True)
    with open(path, 'w') as fh:
        json.dump(doc, fh, indent=1)
        fh.flush()
        os.fsync(fh.fileno())


def checkpoint_steps(path):
    m = re.search(r'_(\d+)_steps\.zip$', path)
    return int(m.group(1)) if m else 0


def do_train(args, client, world):
    error_scale, label = ARMS[args.arm]
    save = os.path.join(run_dir(args), 'sac_ego.zip')
    steps = args.timesteps
    resume = None
    if args.resume:
        ckpts = sorted(glob.glob(os.path.join(run_dir(args),
                                              'sac_ego_ckpt_*_steps.zip')),
                       key=checkpoint_steps)
        if ckpts:
            resume = ckpts[-1]
            steps = args.timesteps - checkpoint_steps(resume)
            print('[arm] resuming from %s (%d steps left) / 再開'
                  % (os.path.basename(resume), steps))
            if steps <= 0:
                print('[arm] budget already reached, skipping training '
                      '/ 学習済みステップ数が予算に達している')
                return
        else:
            print('[arm] --resume but no checkpoint yet; starting fresh '
                  '/ チェックポイントが無いので最初から')
    print('[arm] === TRAIN arm %s (%s traffic) seed %d, %d steps ==='
          % (args.arm, label, args.seed, steps))
    write_run_json(args, 'running', resumed_from=resume)
    traffic = start_traffic(args, world, error_scale, seed=1000 + args.seed)
    env = None
    try:
        env = CarlaHILEnv(host=args.host, port=args.port, seed=args.seed)
        from rl.train_sac import train
        train(env, steps, save, seed=args.seed, resume=resume,
              run_name='%s_s%d' % (args.arm, args.seed),
              ckpt_every=args.ckpt_every, tb_dir=paths.TB_DIR,
              torch_threads=args.torch_threads)
    finally:
        if env is not None:
            env._destroy_actors()
        stop_traffic(traffic, world)
    write_run_json(args, 'trained')


def do_eval(args, client, world):
    from rl.evaluate import evaluate, CSV_FIELDS
    from stable_baselines3 import SAC
    import csv

    ckpts = sorted(glob.glob(os.path.join(run_dir(args),
                                          'sac_ego_ckpt_*_steps.zip')),
                   key=checkpoint_steps)
    final = os.path.join(run_dir(args), 'sac_ego.zip')
    if os.path.exists(final):
        ckpts.append(final)
    if not ckpts:
        print('[arm] nothing to evaluate in %s / 評価対象なし' % run_dir(args))
        return
    if args.max_ckpts and len(ckpts) > args.max_ckpts:
        # Keep an even spread plus the last one. / 等間隔＋最後を残す。
        idx = [round(i * (len(ckpts) - 1) / (args.max_ckpts - 1))
               for i in range(args.max_ckpts)]
        ckpts = [ckpts[i] for i in sorted(set(idx))]

    for label in args.eval_traffic:
        scale = EVAL_TRAFFIC[label]
        print('[arm] === EVAL arm %s seed %d on "%s" traffic, %d ckpt(s) ==='
              % (args.arm, args.seed, label, len(ckpts)))
        # Fixed seed per traffic label: every arm and checkpoint meets
        # the same NPC fleet. / 交通ラベル毎に固定シード：全アーム・全
        # チェックポイントが同じ NPC 集団に会う。
        traffic = start_traffic(args, world, scale,
                                seed=7 + sorted(EVAL_TRAFFIC).index(label))
        env = None
        try:
            env = CarlaHILEnv(host=args.host, port=args.port,
                              seed=args.eval_seed)
            for ck in ckpts:
                model = SAC.load(ck)
                # Same routes for every checkpoint. / 全 ckpt で同じ経路。
                env.rng.seed(args.eval_seed)
                stats = evaluate(model, env, args.episodes)
                row = {'wall_time': int(time.time()), 'model': ck,
                       'train_timesteps': model.num_timesteps,
                       'episodes': args.episodes,
                       'arm': args.arm, 'seed': args.seed,
                       'eval_traffic': label, 'map': args.map}
                row.update({k: round(v, 3) for k, v in stats.items()})
                new_file = not os.path.exists(EVAL_CSV)
                with open(EVAL_CSV, 'a', newline='') as fh:
                    w = csv.DictWriter(fh, fieldnames=CSV_FIELDS,
                                       extrasaction='ignore')
                    if new_file:
                        w.writeheader()
                    w.writerow(row)
                print('[arm] %s @%d: reward %.1f, coll/ep %.2f, '
                      'near-miss/ep %.2f, done %.0f%%'
                      % (label, model.num_timesteps, stats['mean_reward'],
                         stats['collisions_per_ep'],
                         stats['near_miss_per_ep'],
                         100 * stats['completion_rate']))
        finally:
            if env is not None:
                env._destroy_actors()
            stop_traffic(traffic, world)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', required=True, choices=sorted(ARMS))
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--map', default=config.STUDY_MAP,
                   help='CARLA town (default from rl/config.py) / 対象の町')
    p.add_argument('--timesteps', type=int, default=50_000)
    p.add_argument('--ckpt-every', type=int, default=10_000)
    p.add_argument('--vehicles', type=int, default=20,
                   help='NPC vehicles during training and evaluation '
                        '/ 学習・評価中の NPC 車両数')
    p.add_argument('--spawn-chunk', type=int, default=5)
    p.add_argument('--profile', default=PROFILE_PATH)
    p.add_argument('--eval-only', action='store_true')
    p.add_argument('--no-eval', action='store_true')
    p.add_argument('--eval-traffic', nargs='+', default=[config.EVAL_TRAFFIC],
                   choices=sorted(EVAL_TRAFFIC))
    p.add_argument('--episodes', type=int, default=5,
                   help='evaluation episodes per checkpoint / ckpt 毎の評価数')
    p.add_argument('--eval-seed', type=int, default=42)
    p.add_argument('--max-ckpts', type=int, default=6,
                   help='evaluate at most this many checkpoints (0 = all) '
                        '/ 評価するチェックポイントの上限（0 = 全て）')
    p.add_argument('--resume', action='store_true',
                   help='continue from the newest checkpoint of this run '
                        '/ この run の最新チェックポイントから続ける')
    p.add_argument('--render', action='store_true',
                   help='keep server rendering on (default: no_rendering_mode '
                        'while this script owns the world) / 描画を切らない')
    p.add_argument('--torch-threads', type=int, default=4,
                   help='torch intra-op threads; low keeps this CPU stable '
                        '/ torch のスレッド数（少なめが安定）')
    p.add_argument('--nice', type=int, default=5,
                   help='os.nice() increment for this process / 優先度を下げる')
    p.add_argument('--allow-faults', action='store_true',
                   help='start even if this boot already logged NVRM Xid / '
                        'lockups / 既に障害ログがあっても開始する')
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--tm-port', type=int, default=8000)
    args = p.parse_args()

    global LOG
    paths.ensure_dirs()
    stamp = time.strftime('%Y%m%d-%H%M%S')
    base = os.path.join(paths.LOGS_DIR, '%s_%s' % (
        paths.run_name(args.arm, args.map, args.seed), stamp))
    LOG = Tee(base + '.log')
    sys.stdout = sys.stderr = LOG
    print('[arm] log -> %s.log' % base)

    # --- machine preflight (see rl/health.py) --------------------------------
    ok, warns, _ = health.preflight(allow_faults=args.allow_faults)
    for w in warns:
        print('[arm] WARNING: %s' % w)
    health.write_snapshot(base + '.health.json',
                          extra={'argv': sys.argv[1:], 'phase': 'start'})
    if not ok:
        print('[arm] refusing to start: kernel log shows GPU faults or '
              'lockups in this boot. Reboot, or pass --allow-faults.\n'
              '[arm] 開始を中止：このブートに GPU 障害/ロックアップの記録が'
              'ある。再起動するか --allow-faults を付ける。')
        return 2

    # --- keep our own load modest --------------------------------------------
    # OMP/MKL must be set before torch is imported (inside train_sac).
    # torch の import 前に環境変数を設定する必要がある。
    for var in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
        os.environ.setdefault(var, str(max(1, args.torch_threads)))
    if args.nice > 0:
        try:
            os.nice(args.nice)
        except OSError:
            pass

    if ARMS[args.arm][0] > 0.0 or any(EVAL_TRAFFIC[t] > 0 for t in args.eval_traffic):
        if not os.path.exists(args.profile):
            print('[arm] %s missing: run `python -m rl.extract_scenarios` '
                  'first / が無い。先に rl.extract_scenarios を実行'
                  % args.profile)
            return 1

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = ensure_map(client, args.map)
    # Own the TrafficManager BEFORE the traffic child connects, so the
    # child becomes a TM client of this process (see module docstring).
    # 交通の子プロセスより先に TrafficManager を所有し、子をこちらの TM
    # クライアントにする（モジュール冒頭を参照）。
    tm = client.get_trafficmanager(args.tm_port)
    tm.set_synchronous_mode(True)
    set_sync(world, True, no_rendering=not args.render)
    if not args.render:
        print('[arm] no_rendering_mode ON: GPU idle during the run '
              '(--render to disable) / 描画停止中（--render で解除）')
    os.makedirs(run_dir(args), exist_ok=True)
    status = 'failed'
    try:
        if not args.eval_only:
            do_train(args, client, world)
        if not args.no_eval:
            do_eval(args, client, world)
        status = 'finished'
    except KeyboardInterrupt:
        status = 'interrupted'
        print('[arm] interrupted / 中断')
    except RuntimeError as exc:
        # carla.Client raises RuntimeError on time-out: the server died or
        # the GPU went away. Say so plainly instead of a stack trace.
        # サーバ死亡や GPU 消失は time-out の RuntimeError で来る。
        print('[arm] CARLA error: %s' % exc)
        print('[arm] check `journalctl -k | grep -E "Xid|lockup"` before '
              'retrying / 再試行前にカーネルログを確認')
        status = 'failed: %s' % str(exc)[:120]
    finally:
        # Hand the server back asynchronous and rendering, exactly like
        # generate_traffic does, or the next human session on this
        # CarlaUE4 freezes / shows nothing.
        # generate_traffic と同様に非同期・描画ありへ戻す。さもないと次の
        # 人間セッションが停止する／何も映らない。
        try:
            tm.set_synchronous_mode(False)
            set_sync(world, False, no_rendering=False)
        except RuntimeError as exc:
            print('[arm] could not restore async mode: %s' % exc)
        try:
            write_run_json(args, status)
            health.write_snapshot(base + '.health.json',
                                  extra={'argv': sys.argv[1:],
                                         'phase': 'end', 'status': status})
        except Exception as exc:   # noqa: BLE001 -- never mask the run result
            print('[arm] could not write run.json/health: %s' % exc)
    print('[arm] %s -> %s' % (status, run_dir(args)))
    return 0 if status == 'finished' else 1


if __name__ == '__main__':
    sys.exit(main())
