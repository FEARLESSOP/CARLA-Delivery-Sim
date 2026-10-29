#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# 本成果物は MIT ライセンスの条件の下で提供される。
# 全文は <https://opensource.org/licenses/MIT> を参照。

"""Example script to generate traffic in the simulation

シミュレーション内に交通を生成するサンプルスクリプト
"""

import glob
import json
import os
import signal
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

# How long to wait for a tick from another client before concluding that
# nobody is stepping this synchronous world / 他クライアントによるティック
# をどれだけ待って「誰もこの同期ワールドを進めていない」と判断するか
MASTER_PROBE_S = 3.0
PASSIVE_TIMEOUT_S = 10.0
# Consecutive world.tick() RPC failures tolerated before giving up
# world.tick() の RPC 連続失敗をどこまで許容するか
MAX_TICK_FAILURES = 3
# Liveness file polled by run_pipeline.py's freeze watchdog
# run_pipeline.py のフリーズ監視が mtime を見る死活ファイル
HEARTBEAT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'traffic_heartbeat.json')
HEARTBEAT_PERIOD_S = 0.5


def apply_batch_chunked(client, batch, sync, chunk, pause, label=''):
    """Apply a command batch in small chunks, pausing between them.

    Spawning every vehicle in one `apply_batch_sync` makes the server load
    all of their meshes, materials and shaders in a single burst. On this
    rig (RTX 5080 / nvidia-driver-595.84-open) that burst reliably takes the
    GPU down with an NVRM Xid fault, which reaches UE4 as
    `VK_ERROR_DEVICE_LOST` and kills the server with signal 11 -- measured
    3 crashes in 3 attempts with a 60-vehicle batch, and 0 in 3 with this
    chunking. Spreading the uploads out is a workaround for a driver bug,
    not a fix for one: see docs/TROUBLESHOOTING.md.

    1回の apply_batch_sync で全車両を生成すると、サーバがメッシュ・
    マテリアル・シェーダを一気に読み込む。本機（RTX 5080 /
    nvidia-driver-595.84-open）ではこの集中が NVRM Xid 障害を確実に誘発し、
    UE4 には VK_ERROR_DEVICE_LOST として伝わりシグナル11でサーバが落ちる
    （60台一括で3回中3回クラッシュ、チャンク化で3回中0回）。これは
    ドライバ側のバグに対する回避策であって修正ではない。
    詳細は docs/TROUBLESHOOTING.md を参照。

    Returns the concatenated responses, so callers read it exactly as they
    read apply_batch_sync. / 応答を連結して返すため、呼び出し側は
    apply_batch_sync と同じように扱える。
    """
    if chunk <= 0 or len(batch) <= chunk:
        return client.apply_batch_sync(batch, sync)
    responses = []
    for i in range(0, len(batch), chunk):
        responses.extend(client.apply_batch_sync(batch[i:i + chunk], sync))
        if i + chunk < len(batch) and pause > 0.0:
            time.sleep(pause)
    if label:
        print('[traffic] spawned %d %s in chunks of %d / %d体の%sを%d体ずつ生成'
              % (len(responses), label, chunk, len(responses), label, chunk))
    return responses


def destroy_chunked(client, actor_ids, chunk, pause):
    """Destroy actors a few at a time, mirroring apply_batch_chunked.

    Tearing 60 vehicles down in one apply_batch frees all of their GPU
    resources at once -- the same burst shape that crashes the server on
    spawn. Measured: SIGTERM during a 60-vehicle session killed CarlaUE4
    with signal 11 every time while the teardown was one batch, and stopped
    doing so once it was chunked.

    60台を1回の apply_batch で破棄すると GPU リソースの解放が一気に集中し、
    生成時にサーバを落とすのと同じ形の負荷になる。実測では、破棄が一括の
    間は60台セッション中の SIGTERM で毎回 CarlaUE4 がシグナル11で落ちたが、
    チャンク化すると発生しなくなった。
    """
    if not actor_ids:
        return
    if chunk <= 0:
        chunk = len(actor_ids)
    for i in range(0, len(actor_ids), chunk):
        client.apply_batch(
            [carla.command.DestroyActor(x) for x in actor_ids[i:i + chunk]])
        if i + chunk < len(actor_ids) and pause > 0.0:
            time.sleep(pause)



def load_driver_profile(path, map_name):
    """Return (profile_map_name, traffic_manager params) from map_profiles.json.

    `rl/extract_scenarios.py` writes one km-weighted profile per map; the
    caller normally wants the entry for the map that is loaded right now.
    If that map has no profile the first available one is used with a
    warning -- a profile from another town is still measured human error,
    which beats default TrafficManager traffic for arm B, but the mismatch
    must be visible in the log.

    map_profiles.json（rl/extract_scenarios.py が出力するマップ毎の距離
    加重プロファイル）から (使用したマップ名, TrafficManager パラメータ)
    を返す。通常は現在ロード中のマップの項目を使う。該当マップの項目が
    無い場合は警告付きで最初の項目を使う。他の町の実測エラーでも既定の
    TrafficManager 交通よりはアームBの目的に適うが、不一致はログで
    分かるようにしておく。
    """
    with open(path) as fh:
        profiles = json.load(fh)
    if not profiles:
        raise ValueError('%s holds no map profiles / にマッププロファイルが無い'
                         % path)
    short = map_name.rsplit('/', 1)[-1]
    for key in (map_name, short):
        if key in profiles:
            return key, profiles[key]['traffic_manager']
    key = sorted(profiles)[0]
    logging.warning('no driver profile for %s; using %s '
                    '/ %s のプロファイルが無いため %s を使用',
                    short, key, short, key)
    return key, profiles[key]['traffic_manager']


def apply_driver_profile(traffic_manager, vehicles, tm_params, scale, rng):
    """Make the TrafficManager drive like the measured humans (study arm B).

    Per-vehicle parameters, so the fleet is a *population* of imperfect
    drivers rather than 60 copies of one:
      ignore_lights_percentage      measured red-light rate x scale
      distance_to_leading_vehicle   measured following distance / scale
      vehicle_lane_offset           ~N(0, SDLP x scale) per vehicle, i.e.
                                    each car carries its own lateral bias
                                    (`rng` is a numpy RandomState: this
                                    module's `random` IS numpy.random)
      vehicle_percentage_speed_difference
                                    measured value x scale (negative =
                                    faster than the limit)
    `scale` is the curriculum knob of docs/RL_DESIGN.md §4: 0 = default
    traffic (the caller skips this function), 1 = measured rates,
    2 = overshoot for stress testing.

    TrafficManager を実測した人間のように運転させる（研究のアームB）。
    車両毎にパラメータを与え、同じ運転者のコピー60台ではなく「不完全な
    運転者の集団」にする。`scale` は docs/RL_DESIGN.md §4 のカリキュラム
    係数：0 = 既定交通（呼び出し側がこの関数を飛ばす）、1 = 実測値、
    2 = 過酷試験用の2倍。
    """
    lights = min(100.0, max(0.0, tm_params['ignore_lights_percentage'] * scale))
    lead = max(1.0, tm_params['distance_to_leading_vehicle'] / max(scale, 1e-3))
    offset_std = max(0.0, tm_params['vehicle_lane_offset_std'] * scale)
    speed_diff = tm_params['vehicle_percentage_speed_difference'] * scale
    for v in vehicles:
        traffic_manager.ignore_lights_percentage(v, lights)
        traffic_manager.distance_to_leading_vehicle(v, lead)
        # Clamp so a 2x overshoot cannot push a car into the next lane
        # by itself. / 2倍でも単独で隣車線へ出ない程度に制限する。
        offset = float(max(-1.0, min(1.0, rng.normal(0.0, offset_std))))
        traffic_manager.vehicle_lane_offset(v, offset)
        traffic_manager.vehicle_percentage_speed_difference(v, speed_diff)
    print('[traffic] driver profile x%.1f on %d vehicle(s): lights %.1f%%, '
          'lead %.1f m, lane offset sd %.3f m, speed diff %.1f%% '
          '/ プロファイル x%.1f を %d 台に適用'
          % (scale, len(vehicles), lights, lead, offset_std, speed_diff,
             scale, len(vehicles)))


def tick_is_flowing(world, timeout=MASTER_PROBE_S):
    """True if some client is already stepping this synchronous world.

    In synchronous mode the server advances only when a client calls
    `world.tick()`, and in this rig that client is this script -- the
    driving window, dashboard and sensor viewer are all passive readers.
    So `settings.synchronous_mode` being True does NOT prove a tick master
    exists: a master that was killed without unwinding leaves the flag set
    with nobody stepping the world. Trusting the flag makes this instance
    demote itself to a passive waiter, and the simulation stays frozen
    forever. Probe for an actual tick instead.

    同期モードではクライアントが `world.tick()` を呼んだときだけサーバが
    進む。本環境でそれを呼ぶのは本スクリプトのみで、運転ウィンドウ・
    ダッシュボード・センサービューアはいずれも受動的な読み手である。
    したがって `settings.synchronous_mode` が True でもティックマスタが
    存在するとは限らない。後始末をせずに落ちたマスタはフラグだけを残す。
    フラグを信用するとこのインスタンスも受動側に回り、シミュレーションは
    永久に停止したままになる。フラグではなく実際のティックを確認する。
    """
    try:
        world.wait_for_tick(timeout)
        return True
    except RuntimeError:
        return False


def become_tick_master(world, args):
    """Put the world into synchronous mode stepped by this process.

    ワールドを同期モードにし、本プロセスがステップを進める状態にする。
    """
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = (
        1.0 / args.sim_fps if args.sim_fps > 0 else 0.05)
    world.apply_settings(settings)
    return True


def write_heartbeat(frame, master):
    """Record that the world advanced, for the pipeline's freeze watchdog.

    Written via a temp file + rename so a reader never sees a half-written
    record, matching the other JSON side-channels in this rig.

    ワールドが進んだことをパイプラインのフリーズ監視向けに記録する。
    本環境の他のJSON連携と同様、一時ファイル+renameで書き込み、読み手が
    書きかけの内容を見ないようにする。
    """
    tmp = HEARTBEAT_PATH + '.tmp'
    try:
        with open(tmp, 'w') as fh:
            json.dump({'frame': int(frame), 'wall_time': time.time(),
                       'master': bool(master), 'pid': os.getpid()}, fh)
        os.replace(tmp, HEARTBEAT_PATH)
    except OSError:
        pass


def install_shutdown_handlers():
    """Make SIGTERM/SIGINT unwind through main()'s cleanup.

    run_pipeline.py stops this script with SIGTERM. Python's default
    SIGTERM disposition kills the process outright, so the `finally` block
    that hands the world back to asynchronous mode never runs -- and the
    server is left in synchronous mode with no tick master. Any later
    session that reuses the same CarlaUE4 then freezes on startup. Raising
    SystemExit instead unwinds normally. Further signals are ignored so a
    second one cannot abort the cleanup halfway (the pipeline escalates to
    SIGKILL on its own if we take too long).

    run_pipeline.py は本スクリプトを SIGTERM で停止する。Python の既定
    動作ではプロセスが即座に終了するため、ワールドを非同期モードへ戻す
    `finally` が実行されず、サーバはティックマスタ不在の同期モードのまま
    残る。その CarlaUE4 を再利用した以降のセッションは起動時に停止する。
    代わりに SystemExit を送出して正常に巻き戻す。後続のシグナルは無視し、
    2回目のシグナルが後始末を中断しないようにする（時間がかかりすぎた
    場合はパイプライン側が SIGKILL に切り替える）。
    """
    shutdown_signals = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)

    def bail(signum, _frame):
        # Ignore *every* shutdown signal from here on, not just this one.
        # Ignoring only `signum` meant a SIGINT followed by a SIGTERM (or
        # the pipeline's SIGTERM after an operator's Ctrl-C) raised a second
        # SystemExit inside the cleanup block, aborting it midway through
        # destroying actors and leaving the world half torn down.
        # 以降はこのシグナルだけでなく、全ての終了シグナルを無視する。
        # `signum` だけを無視していたため、SIGINT の後に SIGTERM が来ると
        # （操作者の Ctrl-C の後にパイプラインが SIGTERM を送る場合など）
        # 後始末の最中に2度目の SystemExit が発生し、アクターの破棄を
        # 途中で中断してワールドを半壊状態で残していた。
        for s in shutdown_signals:
            try:
                signal.signal(s, signal.SIG_IGN)
            except (ValueError, OSError, AttributeError):
                pass
        raise SystemExit(128 + signum)

    for sig in shutdown_signals:
        try:
            signal.signal(sig, bail)
        except (ValueError, OSError, AttributeError):
            pass


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    # フィルタ結果がブループリント1件のみの場合はそれが目的のものと見なし、
    # 世代 (generation) 指定を無視する
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations / 指定世代が有効な世代に含まれるか確認
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--sim-fps',
        default=30.0,
        type=float,
        help='Simulation steps per second when this script is the '
             'synchronous tick master (default: 30). The loop paces itself '
             'to real time, so this is also the server frame rate. '
             '0 = free-running (old behaviour) '
             '/ 本スクリプトが同期ティックマスタのときのシミュレーション'
             'ステップ数（既定 30）。ループは実時間に合わせて動作するため、'
             'これはサーバのフレームレートでもある。0 で無制限（従来動作）')
    argparser.add_argument(
        '--spawn-chunk',
        default=5,
        type=int,
        help='Spawn vehicles/walkers this many at a time instead of in one '
             'batch (default: 5). A single apply_batch_sync of 60 vehicles '
             'uploads every mesh, material and shader in one burst, which '
             'reliably triggers an NVRM Xid GPU fault on this rig. 0 = one '
             'batch (old behaviour) '
             '/ 車両・歩行者を一度に全部ではなく、この台数ずつ生成する'
             '（既定 5）。60台を1回の apply_batch_sync で生成すると'
             'メッシュ・マテリアル・シェーダの転送が一気に集中し、本機では'
             'NVRM Xid GPU 障害を確実に誘発する。0 で一括生成（従来動作）')
    argparser.add_argument(
        '--spawn-pause',
        default=0.5,
        type=float,
        help='Seconds to wait between spawn chunks (default: 0.5), giving '
             'the GPU time to absorb each batch of asset uploads '
             '/ 生成チャンク間の待機秒数（既定 0.5）。GPU がアセット転送を'
             '処理しきる時間を与える')
    argparser.add_argument(
        '--driver-profile',
        metavar='JSON',
        default=None,
        help='Human-error traffic (study arm B): apply the TrafficManager '
             'parameters mined from human sessions by rl/extract_scenarios.py '
             '(data/scenarios/map_profiles.json) to every spawned vehicle '
             '/ 人間エラー交通（研究アームB）。rl/extract_scenarios.py が'
             '人間セッションから抽出した TrafficManager パラメータ'
             '（data/scenarios/map_profiles.json）を全車両に適用する')
    argparser.add_argument(
        '--profile-map',
        default=None,
        help='Which map entry of --driver-profile to use (default: the map '
             'currently loaded) / --driver-profile のどのマップ項目を使うか'
             '（既定：現在ロード中のマップ）')
    argparser.add_argument(
        '--error-scale',
        default=1.0,
        type=float,
        help='Curriculum multiplier on the --driver-profile error rates: '
             '0 = default traffic, 1 = measured human rates, 2 = overshoot '
             '(docs/RL_DESIGN.md section 4) / --driver-profile のエラー率に'
             '掛けるカリキュラム係数：0 = 既定交通、1 = 実測値、2 = 2倍')
    argparser.add_argument(
        '--passive',
        action='store_true',
        default=False,
        help='Never become the synchronous tick master; another client '
             '(e.g. the RL training env) steps the world. Waits instead of '
             'taking over when ticks stop / 同期ティックマスタにならない。'
             '他のクライアント（RL学習環境など）がワールドを進める。'
             'ティックが止まっても引き継がず待機する')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')

    argparser.add_argument(
        '--cyclists', type=int, default=0,
        help='bicycles with riders, on top of -n, driven by the traffic manager '
             'a little slower than cars (default: 0) / 自転車の台数（-n に加えて）')
    argparser.add_argument(
        '--wheelchair-share', type=float, default=0.0,
        help='share of walkers in a wheelchair, 0..1 (default: 0) / 車椅子の歩行者の割合')
    argparser.add_argument(
        '--runners', type=float, default=0.0,
        help='share of walkers jogging, 0..1 (default: 0) / ジョギングする歩行者の割合')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if args.passive:
                # The tick master is someone else by contract (the RL env
                # steps the world during training). If it is not ticking
                # yet, the spawn below still works -- apply_batch_sync
                # answers without a tick -- and the main loop waits.
                # 契約上ティックマスタは別のクライアント（学習中は RL 環境が
                # ワールドを進める）。まだ進んでいなくても下の生成は動く
                # （apply_batch_sync はティック無しで応答する）ので、
                # メインループで待つ。
                synchronous_master = False
                print('[traffic] --passive: leaving the tick to another '
                      'client / ティックは他クライアントに任せる')
            elif not settings.synchronous_mode:
                synchronous_master = True
            elif tick_is_flowing(world):
                # Another client is stepping the world; stay passive.
                # 他のクライアントがワールドを進めている。受動側に回る。
                synchronous_master = False
            else:
                # Synchronous mode is on but the world is not advancing:
                # the previous tick master died without restoring async.
                # Take over, otherwise nothing ever steps the simulation.
                # 同期モードなのにワールドが進んでいない。前のティック
                # マスタが非同期へ戻さずに落ちた状態。ここで引き継がない
                # と誰もシミュレーションを進めなくなる。
                print('[traffic] world is in synchronous mode but nothing is '
                      'stepping it -> taking over as tick master '
                      '/ 同期モードだが誰も進めていないため'
                      'ティックマスタを引き継ぐ')
                synchronous_master = True
            if synchronous_master:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = (
                    1.0 / args.sim_fps if args.sim_fps > 0 else 0.05)
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")

        # the delivery pods are driven by delivery_sim.py, never random traffic
        # 配送ポッドは delivery_sim.py が動かす（一般交通には出さない）
        blueprints = [x for x in blueprints if not x.id.startswith('vehicle.delivery.')]
        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
        bikes = [x for x in blueprints if x.has_attribute('base_type')
                 and x.get_attribute('base_type').as_str() == 'bicycle']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly. / @todo これらは直接インポートできない。
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles / 車両を生成
        # --------------
        batch = []
        hero = args.hero
        n_cyclists = args.cyclists if bikes else 0
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles + n_cyclists:
                break
            # the first --cyclists spawn points get a bicycle / 先頭から自転車
            blueprint = random.choice(bikes if n < n_cyclists else blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            # 車両を生成し、オートパイロットとライト状態をまとめて設定
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in apply_batch_chunked(
                client, batch, synchronous_master,
                args.spawn_chunk, args.spawn_pause, 'vehicles'):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)
        # cyclists ride ~40 % under the limit and keep their distance
        # 自転車は制限速度の約 4 割減、車間を広めに
        n_bikes = 0
        for actor in world.get_actors(vehicles_list):
            if actor.attributes.get('base_type') == 'bicycle':
                traffic_manager.vehicle_percentage_speed_difference(actor, 40.0)
                traffic_manager.distance_to_leading_vehicle(actor, 3.0)
                n_bikes += 1
        if n_cyclists:
            print('[traffic] %d cyclists among the vehicles / 自転車 %d 台' % (n_bikes, n_bikes))

        if args.driver_profile and args.error_scale > 0.0 and vehicles_list:
            prof_key, tm_params = load_driver_profile(
                args.driver_profile,
                args.profile_map or world.get_map().name)
            print('[traffic] driver profile: %s (%s) / プロファイル'
                  % (prof_key, args.driver_profile))
            apply_driver_profile(
                traffic_manager, world.get_actors(vehicles_list), tm_params,
                args.error_scale,
                random.RandomState(args.seed if args.seed is not None else 0))
        elif args.driver_profile:
            print('[traffic] --error-scale 0: default traffic, profile not '
                  'applied / 係数0のため既定交通（プロファイル未適用）')

        # Set automatic vehicle lights update if specified / 指定があれば車両ライトの自動更新を有効化
        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers / 歩行者を生成
        # -------------
        # some settings / 各種設定
        percentagePedestriansRunning = max(0.0, min(1.0, args.runners))  # jogging / 走る歩行者の割合
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 道路を横断する歩行者の割合
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
        # 1. take all the random locations to spawn / 1. スポーン用のランダムな位置をすべて取得
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object / 2. 歩行者オブジェクトを生成
        batch = []
        walker_speed = []
        chairs = [b for b in blueprintsWalkers if b.has_attribute('can_use_wheelchair')
                  and b.get_attribute('can_use_wheelchair').as_bool()]
        n_chairs = 0
        for spawn_point in spawn_points:
            in_chair = bool(chairs) and random.random() < args.wheelchair_share
            walker_bp = random.choice(chairs if in_chair else blueprintsWalkers)
            # set as not invincible / 無敵状態を解除
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # CARLA 0.9.16's wheelchair: the same people, seated / 車椅子
            if walker_bp.has_attribute('use_wheelchair'):
                walker_bp.set_attribute('use_wheelchair', 'true' if in_chair else 'false')
                n_chairs += in_chair
            # set the max speed / 最高速度を設定
            if walker_bp.has_attribute('speed'):
                if in_chair or random.random() > percentagePedestriansRunning:
                    # walking / 歩行
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running / 走行
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        # Walker bodies carry skeletal meshes, so they get the same chunking
        # as vehicles. The controllers spawned below are pure logic actors
        # with no assets to upload, so they stay in one batch.
        # 歩行者本体はスケルタルメッシュを伴うため車両と同様にチャンク化する。
        # 下で生成するコントローラはアセットを持たない論理アクターのため
        # 一括のままでよい。
        results = apply_batch_chunked(
            client, batch, True, args.spawn_chunk, args.spawn_pause, 'walkers')
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        if n_chairs or args.runners:
            print('[traffic] walkers: %d in a wheelchair, ~%.0f %% jogging / 車椅子 %d 人'
                  % (n_chairs, percentagePedestriansRunning * 100, n_chairs))
        # 3. we spawn the walker controller / 3. 歩行者コントローラを生成
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        # 4. 歩行者とコントローラのIDをまとめ、IDからオブジェクトを取得できるようにする
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        # 直前に生成した歩行者の最新Transformをクライアントが確実に受け取れるよう1ティック待つ
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        # 5. 各コントローラを初期化し、歩行先の目標地点を設定（リストは [コントローラ, アクター, ...] の順）
        # 道路を横断できる歩行者の割合を設定
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker / 歩行者を開始
            all_actors[i].start()
            # set walk to random point / ランダムな地点へ歩行させる
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed / 最高速度
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # Example of how to use Traffic Manager parameters / Traffic Manager パラメータの使用例
        traffic_manager.global_percentage_speed_difference(30.0)

        # Real-time pacing / 実時間ペーシング
        #
        # world.tick() returns as soon as the server has stepped, so an
        # unthrottled loop drives the simulator as hard as the hardware
        # allows -- measured at ~500 steps/s on an empty Town02, each one a
        # full render pass, for ~3.3 CPU cores and 140 W of GPU. It also
        # means simulated time races ahead of the wall clock, which is wrong
        # for a human-in-the-loop study. Sleeping out the remainder of each
        # step pins the simulation to real time and hands the rest of the
        # machine back.
        #
        # world.tick() はサーバが1ステップ進めた時点で戻るため、制限のない
        # ループはハードウェアの限界までシミュレータを回してしまう。空の
        # Town02 で毎秒約500ステップ、各ステップが完全な描画パスとなり、
        # CPU約3.3コアとGPU 140W を消費していた。さらにシミュレーション時間
        # が実時間より先行するため、人間参加型の実験としては不正である。
        # 各ステップの残り時間をスリープすることで、シミュレーションを実時間
        # に固定し、余った計算資源を他に回す。
        # --sim-fps 0 restores the old free-running behaviour.
        # --sim-fps 0 で従来の無制限動作に戻る。
        step = (world.get_settings().fixed_delta_seconds or 0.05
                if args.sim_fps > 0 else 0.0)
        next_tick = time.time()
        next_beat = 0.0
        tick_failures = 0
        frame = 0
        while True:
            if not args.asynch and synchronous_master:
                try:
                    frame = world.tick()
                    tick_failures = 0
                except RuntimeError as exc:
                    # One slow frame (map streaming, a GPU hitch) can blow
                    # the RPC timeout. Dying on it would leave the world
                    # synchronous with nobody stepping it, freezing every
                    # other component, so retry before giving up.
                    # 重いフレーム（マップのストリーミングやGPUの詰まり）
                    # 1回で RPC がタイムアウトしうる。ここで終了すると
                    # ワールドが同期モードのまま誰も進めなくなり他の全
                    # コンポーネントが停止するため、諦める前に再試行する。
                    tick_failures += 1
                    if tick_failures > MAX_TICK_FAILURES:
                        raise
                    print('[traffic] world.tick() failed (%d/%d): %s'
                          % (tick_failures, MAX_TICK_FAILURES, exc))
                    next_tick = time.time()
                    continue
                if step > 0.0:
                    next_tick += step
                    slack = next_tick - time.time()
                    if slack > 0.0:
                        time.sleep(slack)
                    elif slack < -1.0:
                        # Fell more than a second behind (a heavy frame, a
                        # map load): resync rather than sprint to catch up.
                        # 1秒以上遅れた場合（重いフレームやマップ読込）は、
                        # 取り戻そうと全速で回さず基準時刻を引き直す。
                        next_tick = time.time()
            else:
                try:
                    frame = world.wait_for_tick(PASSIVE_TIMEOUT_S).frame
                except RuntimeError:
                    if args.passive:
                        # By contract the master is pausing (model save,
                        # checkpoint evaluation) -- keep waiting.
                        # 契約上マスタが一時停止中（モデル保存・評価）
                        # なので待ち続ける。
                        continue
                    # The client that was stepping this world has gone
                    # away. Take over instead of blocking forever.
                    # ワールドを進めていたクライアントが消えた。永久に
                    # 待ち続けず、ここで引き継ぐ。
                    print('[traffic] no tick for %.0fs -> taking over as '
                          'synchronous tick master / %.0f秒ティックが無い'
                          'ためティックマスタを引き継ぐ'
                          % (PASSIVE_TIMEOUT_S, PASSIVE_TIMEOUT_S))
                    synchronous_master = become_tick_master(world, args)
                    next_tick = time.time()

            now = time.time()
            if now >= next_beat:
                write_heartbeat(frame, synchronous_master)
                next_beat = now + HEARTBEAT_PERIOD_S

    finally:

        # Hand the world back to asynchronous mode. Skipping this is what
        # freezes the *next* session: the server keeps synchronous_mode
        # with no client stepping it, and every component of a run that
        # reuses this CarlaUE4 blocks on a world that never advances. The
        # restore must not be able to fail loudly either, or the actor
        # cleanup below would be skipped as well.
        # ワールドを非同期モードへ戻す。これを飛ばすと*次回*のセッションが
        # 停止する。サーバは誰も進めない同期モードのまま残り、その
        # CarlaUE4 を再利用した実行では全コンポーネントが進まないワールド
        # を待ち続ける。ここで例外を投げると後続のアクター破棄も飛ぶため、
        # 復旧処理自体が失敗しても握りつぶす。
        if not args.asynch and synchronous_master:
            try:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.no_rendering_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
            except (RuntimeError, NameError) as exc:
                print('[traffic] could not restore asynchronous mode: %s '
                      '/ 非同期モードへ戻せなかった: %s' % (exc, exc))
        try:
            os.remove(HEARTBEAT_PATH)
        except OSError:
            pass

        print('\ndestroying %d vehicles' % len(vehicles_list))
        destroy_chunked(client, vehicles_list,
                        args.spawn_chunk, args.spawn_pause)

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        # 歩行者コントローラを停止（リストは [コントローラ, アクター, ...] の順）
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        destroy_chunked(client, all_id, args.spawn_chunk, args.spawn_pause)

        time.sleep(0.5)

if __name__ == '__main__':

    install_shutdown_handlers()
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
