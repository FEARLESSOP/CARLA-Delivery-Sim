#!/usr/bin/env python
"""Run CARLA + traffic + G923 manual control + telemetry dashboard in one go.

Example:
    python run_pipeline.py --map Town03 --vehicles 40 --walkers 15
    python run_pipeline.py --no-sim            # assume CarlaUE4 is already running
    python run_pipeline.py --no-traffic --no-dashboard
    python run_pipeline.py --map Town05 --delivery 3   # + 3 delivery vans with drones
    python run_pipeline.py --ffb                       # + G923 force feedback

Press Ctrl+C once to shut everything down cleanly.

CARLA・交通生成・G923手動運転・テレメトリダッシュボードを一括起動する。

例:
    python run_pipeline.py --map Town03 --vehicles 40 --walkers 15
    python run_pipeline.py --no-sim            # CarlaUE4 が既に起動している前提
    python run_pipeline.py --no-traffic --no-dashboard
    python run_pipeline.py --map Town05 --delivery 3   # 配送車 3 台（ドローン付き）も起動
    python run_pipeline.py --ffb                       # G923 のフォースフィードバックも起動

Ctrl+C を一度押すと全プロセスをクリーンに終了する。
"""

import argparse
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent          # .../PythonAPI/hil_sim / 本スクリプトの所在ディレクトリ
CARLA_ROOT = BASE_DIR.parent.parent
VENV_PY = CARLA_ROOT / "carla-venv" / "bin" / "python"
UTIL_DIR = CARLA_ROOT / "PythonAPI" / "util"
CONTROL_SCRIPT = BASE_DIR / "manual_control_steeringwheel G923.py"
FFB_SCRIPT = BASE_DIR / "carla_ffb.py"
# F5 map switch: request file + exit code shared with the driving window
# F5 のマップ切替：運転ウィンドウと共有する依頼ファイルと終了コード
sys.path.insert(0, str(BASE_DIR))
import map_switch  # noqa: E402
# Dashboard Windows card: open / close the optional viewer windows
# ダッシュボードの「ウィンドウ」カード：任意ビューアの開閉
import windows_control  # noqa: E402
DATA_DIR = BASE_DIR / "data"                        # simulation database / シミュレーションデータの保存先


def map_is_left_hand(map_name):
    """True for your own left-hand-traffic maps (maps/maps.json). / 左側通行の自作マップ。"""
    try:
        maps = json.load(open(BASE_DIR / "maps" / "maps.json"))
    except (OSError, ValueError):
        return False
    # maps.json writes "keep_left" (make_osm_city.sh); older entries "left_hand_traffic"
    return any(isinstance(m, dict) and m.get("name") == map_name
               and (m.get("keep_left") or m.get("left_hand_traffic"))
               for m in maps.values())


def map_signal_timing(map_name):
    """The signal-timing profile of one of your own maps (maps.json
    "signal_timing", relative to hil_sim), or None. / 信号時間プロファイル。"""
    try:
        maps = json.load(open(BASE_DIR / "maps" / "maps.json"))
    except (OSError, ValueError):
        return None
    for m in maps.values():
        if isinstance(m, dict) and m.get("name") == map_name and m.get("signal_timing"):
            path = BASE_DIR / m["signal_timing"]
            return path if path.exists() else None
    return None


def nvidia_offload_env(mode):
    """Env for running CarlaUE4 on the discrete NVIDIA GPU under PRIME.

    On a hybrid Intel iGPU + NVIDIA laptop/desktop with
    `prime-select on-demand`, the default Vulkan/GL device is the *Intel*
    iGPU, so CarlaUE4 renders there and the run crawls at single-digit FPS
    while nvidia-smi shows the dGPU idle. These three variables push the
    process onto the NVIDIA card. `mode` is "auto" (only when on-demand
    PRIME is detected), "nvidia" (always) or "off" (never).

    PRIME 環境で CarlaUE4 を NVIDIA GPU 上で動かすための環境変数。
    Intel iGPU と NVIDIA GPU が同居し `prime-select on-demand` の場合、
    Vulkan/GL の既定デバイスは *Intel* 側になる。そのため CarlaUE4 が
    iGPU で描画され、nvidia-smi 上で dGPU が遊んでいるのに FPS が一桁
    まで落ちる。以下の3変数でプロセスを NVIDIA 側に寄せる。mode は
    "auto"（on-demand を検出した時のみ）/"nvidia"（常に）/"off"（無効）。
    """
    if mode == "off":
        return None
    if mode == "auto" and not _prime_on_demand():
        return None
    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["__VK_LAYER_NV_optimus"] = "NVIDIA_only"
    return env


def _prime_on_demand():
    """True if `prime-select query` reports on-demand (hybrid) mode.

    `prime-select query` が on-demand（ハイブリッド）を返せば True。
    """
    try:
        out = subprocess.run(["prime-select", "query"],
                             capture_output=True, timeout=5.0)
        return b"on-demand" in out.stdout
    except Exception:
        return False


def wait_for_port(host, port, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def pick_free_port(preferred, tries=10):
    for p in range(preferred, preferred + tries):
        if not port_in_use(p):
            return p
    return preferred


def carla_alive(host, port, timeout=6.0):
    """True if a CARLA server answers an RPC version request on the port.

    ポート上の CARLA サーバーが RPC のバージョン要求に応答すれば True。
    """
    code = ("import carla\n"
            "c = carla.Client('%s', %d)\n"
            "c.set_timeout(%.1f)\n"
            "c.get_server_version()\n"
            "print('carla-ok')\n" % (host, port, timeout))
    try:
        out = subprocess.run([str(VENV_PY), "-c", code],
                             capture_output=True, timeout=timeout + 10.0)
        return b"carla-ok" in out.stdout
    except Exception:
        return False


def reset_world_async(host, port, timeout=10.0):
    """Clear a stale synchronous-mode world left behind by a dead master.

    In synchronous mode the CARLA server advances only when a client calls
    `world.tick()`; in this rig that client is generate_traffic.py alone.
    If it was killed without unwinding, the server keeps
    `synchronous_mode=True` with nobody stepping it. Every component of the
    next session -- driving window, dashboard, sensor viewer -- then waits
    on a world that never advances: the simulation "starts" and is frozen,
    and telemetry.csv stays empty. Because a healthy CarlaUE4 on the port
    is reused rather than restarted, that state survives across runs until
    the server is killed. So reuse always begins by forcing async mode.

    死んだティックマスタが残した同期モードのワールドを解除する。
    同期モードの CARLA サーバはクライアントが `world.tick()` を呼んだとき
    だけ進み、本環境でそれを呼ぶのは generate_traffic.py だけである。後始末
    をせずに落ちると、サーバは誰も進めないまま `synchronous_mode=True` を
    保持する。次のセッションの全コンポーネント（運転ウィンドウ、ダッシュ
    ボード、センサービューア）は進まないワールドを待ち続け、シミュレーション
    は「起動」しても停止したまま、telemetry.csv も空のままになる。ポート上の
    健全な CarlaUE4 は再起動せず再利用するため、この状態はサーバを終了する
    まで実行をまたいで残る。よって再利用時は必ず非同期モードへ戻す。
    """
    code = ("import carla\n"
            "c = carla.Client('%s', %d)\n"
            "c.set_timeout(%.1f)\n"
            "w = c.get_world()\n"
            "s = w.get_settings()\n"
            "if s.synchronous_mode:\n"
            "    s.synchronous_mode = False\n"
            "    s.fixed_delta_seconds = None\n"
            "    w.apply_settings(s)\n"
            "    print('reset-sync')\n" % (host, port, timeout))
    try:
        out = subprocess.run([str(VENV_PY), "-c", code],
                             capture_output=True, timeout=timeout + 10.0)
        return b"reset-sync" in out.stdout
    except Exception:
        return False


def world_map_name(host, port, timeout=10.0):
    """Short name of the map the server has loaded ('OitaKosen'), or None.

    サーバが読み込んでいるマップの短い名前（取得できなければ None）。
    """
    code = ("import carla\n"
            "c = carla.Client('%s', %d)\n"
            "c.set_timeout(%.1f)\n"
            "print('map=' + c.get_world().get_map().name)\n" % (host, port, timeout))
    try:
        out = subprocess.run([str(VENV_PY), "-c", code],
                             capture_output=True, timeout=timeout + 10.0)
    except Exception:
        return None
    for line in out.stdout.decode("utf-8", "replace").splitlines():
        if line.startswith("map="):
            return map_switch.short_name(line[4:].strip())
    return None


def load_world_map(host, port, name, timeout=120.0):
    """Load a map in a throwaway client and confirm it is the one loaded.

    util/config.py gives load_world only 10 s; a big own map (OitaKosen)
    takes longer, so config.py reports a failure while the server is still
    loading fine. Here the client waits up to ``timeout``, and a timeout is
    followed by polling the loaded map's name before calling it a failure.
    The throwaway process also keeps a libcarla abort out of the pipeline.

    使い捨てクライアントでマップを読み込み、読み込まれたことを確認する。
    util/config.py の load_world は 10 秒しか待たず、大きな自作マップ
    （OitaKosen）では読込中なのに失敗を返す。ここでは ``timeout`` まで待ち、
    タイムアウト後も読込済みマップ名を確認してから失敗と判断する。別プロセス
    なので libcarla の abort もパイプラインに及ばない。
    """
    code = ("import carla\n"
            "c = carla.Client('%s', %d)\n"
            "c.set_timeout(%.1f)\n"
            "w = c.load_world(%r)\n"
            "print('map=' + w.get_map().name)\n" % (host, port, timeout, name))
    try:
        subprocess.run([str(VENV_PY), "-c", code],
                       capture_output=True, timeout=timeout + 20.0)
    except Exception as exc:
        print("[pipeline] load_world(%s): %s" % (name, exc))
    deadline = time.time() + 30.0
    while True:
        if world_map_name(host, port) == map_switch.short_name(name):
            return True
        if time.time() > deadline:
            return False
        time.sleep(3.0)


def stop_procs(procs, grace):
    """SIGTERM a list of (proc, name) together, SIGKILL whatever outlives
    ``grace`` seconds. / まとめて SIGTERM し、猶予後も残るものは SIGKILL。"""
    for proc, name in procs:
        if proc.poll() is None:
            print("[pipeline] stopping %s (pid=%d)" % (name, proc.pid))
            signal_group(proc, signal.SIGTERM)
    deadline = time.time() + grace
    for proc, name in procs:
        try:
            proc.wait(timeout=max(0.0, deadline - time.time()))
        except subprocess.TimeoutExpired:
            print("[pipeline] killing %s (pid=%d)" % (name, proc.pid))
            signal_group(proc, signal.SIGKILL)
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass


# Order in which a map switch stops the world clients: the tick master and
# TrafficManager server first, so the world is handed back asynchronous and
# no TM keeps driving cars that are about to vanish; then everyone else.
# マップ切替でワールドのクライアントを止める順：tick マスタ兼 TM サーバを
# 最初に（ワールドを非同期に戻し、消える車を TM が動かし続けないように）、
# 次に残り全部。
SWITCH_STOP_FIRST = ("generate_traffic", "agent")


def switch_stop_groups(names):
    """Split client names into the stop stages of a map switch (CarlaUE4 is
    never stopped). / マップ切替時の停止段階に分ける（CarlaUE4 は止めない）。"""
    names = [n for n in names if n != "CarlaUE4"]
    first = [n for n in SWITCH_STOP_FIRST if n in names]
    rest = [n for n in names if n not in first]
    return [g for g in (first, rest) if g]


def take_map_request(rc, path=map_switch.REQUEST_PATH, now=None):
    """After the driving window exited with ``rc``: the map-switch request to
    carry out, or None (a normal quit). The file is removed either way. A
    fresh request counts even if the window then died with another code; a
    MAP_SWITCH_RC without a readable request is reported and treated as quit.

    運転ウィンドウ終了後に実行するマップ切替依頼（通常終了なら None）。
    ファイルはどちらの場合も消す。新しい依頼があれば終了コードが違っても
    実行し、MAP_SWITCH_RC なのに依頼が読めない場合は通常終了として扱う。
    """
    req = map_switch.read_request(path, now=now)
    map_switch.clear_request(path)
    if req is None and rc == map_switch.MAP_SWITCH_RC:
        print("[pipeline] manual_control asked for a map switch but %s is missing "
              "or stale -> shutting down / マップ切替の依頼ファイルが無いか古い"
              % path)
    return req


def switch_map(target, stop_clients, reset_async, load_map, restart_carla):
    """The orderly map switch, with the side effects passed in (testable):
    stop every world client, make sure the world is asynchronous, load the
    map; if that fails, restart CarlaUE4 on it. Returns "loaded",
    "restarted" or "failed".

    順序立てたマップ切替（副作用は引数で受け取る）：全クライアント停止 →
    非同期化 → マップ読込。失敗したら CarlaUE4 ごと再起動して読み込む。
    戻り値は "loaded" / "restarted" / "failed"。
    """
    stop_clients()
    if reset_async():
        print("[pipeline] cleared a leftover synchronous mode before the map load "
              "/ マップ読込前に残っていた同期モードを解除した")
    print("[pipeline] loading map %s / マップ %s を読み込み中" % (target, target))
    if load_map(target):
        return "loaded"
    print("[pipeline] ERROR: load_world(%s) failed -> restarting CarlaUE4 on %s "
          "/ マップ読込に失敗したため CarlaUE4 を %s で再起動する"
          % (target, target, target))
    if restart_carla(target):
        return "restarted"
    print("[pipeline] ERROR: could not bring CARLA up on %s / %s で CARLA を"
          "起動できなかった" % (target, target))
    return "failed"


def recent_gpu_faults(minutes=10):
    """Kernel Xid lines blaming CarlaUE4, if the journal is readable.

    CarlaUE4 on this rig has repeatedly taken the GPU down with an Xid
    fault (DMA_DRAM_ACCESS_OUT_OF_BOUNDS, Class Error), which surfaces to
    the engine as VK_ERROR_DEVICE_LOST and kills it with signal 11. The
    only visible symptom is the driving window freezing, so surface the
    kernel's version of events rather than leaving the user guessing.

    CarlaUE4 が Xid 障害（DMA_DRAM_ACCESS_OUT_OF_BOUNDS、Class Error）で
    GPU を落とすことが本環境では繰り返し起きている。エンジン側には
    VK_ERROR_DEVICE_LOST として現れ、signal 11 で終了する。表に見える
    症状は運転ウィンドウの停止だけなので、カーネル側の記録を提示して
    原因を推測させないようにする。
    """
    try:
        out = subprocess.run(
            ["journalctl", "-k", "--no-pager", "--since", "-%dmin" % minutes],
            capture_output=True, timeout=15.0)
        lines = [l for l in out.stdout.decode("utf-8", "replace").splitlines()
                 if "NVRM: Xid" in l]
        return lines[-4:]
    except Exception:
        return []


def heartbeat_age(path):
    """Seconds since the traffic tick master last reported a tick, or None.

    交通ティックマスタが最後にティックを報告してからの秒数（無ければ None）。
    """
    try:
        return time.time() - os.path.getmtime(str(path))
    except OSError:
        return None


def launch(cmd, cwd, name, env=None):
    print("[pipeline] start %s: %s" % (name, " ".join(str(c) for c in cmd)))
    # start_new_session puts the child in its own process group, so we can
    # kill the whole group on shutdown. Crucial for CarlaUE4.sh: it is a
    # wrapper script whose child (CarlaUE4-Linux-Shipping) would otherwise
    # survive terminate() and keep holding port 2000.
    # start_new_session により子プロセスを独立したプロセスグループに置き、
    # 終了時にグループごと kill できるようにする。CarlaUE4.sh はラッパー
    # スクリプトのため、これがないと子 (CarlaUE4-Linux-Shipping) が
    # terminate() 後も生き残り、ポート 2000 を掴み続ける。
    return subprocess.Popen(cmd, cwd=str(cwd), env=env,
                            start_new_session=True)


def signal_group(proc, sig):
    """Send a signal to the child's whole process group (fallback: child).

    子のプロセスグループ全体へシグナルを送る（失敗時は子プロセスのみに送る）。
    """
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.send_signal(sig)
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--tm-port", type=int, default=8000)
    p.add_argument("--map", default=None,
                   help="Load this map before starting (e.g. Town03)")
    p.add_argument("--vehicles", type=int, default=30)
    p.add_argument("--walkers", type=int, default=10)
    p.add_argument("--cyclists", type=int, default=3,
                   help="bicycles with riders in the traffic, on top of --vehicles (default: 3) "
                        "/ 自転車の台数")
    p.add_argument("--wheelchair-share", type=float, default=0.15,
                   help="share of walkers in a wheelchair (default: 0.15) / 車椅子の歩行者の割合")
    p.add_argument("--runners", type=float, default=0.1,
                   help="share of walkers jogging (default: 0.1) / ジョギングの割合")
    p.add_argument("--no-traffic", action="store_true")
    p.add_argument("--no-dashboard", action="store_true")
    p.add_argument("--no-traction-fx", action="store_true",
                   help="no skid marks (traction_fx.py)")
    p.add_argument("--dashboard-port", type=int, default=5000)
    p.add_argument("--no-sensor-viewer", action="store_true",
                   help="Start without the multi-sensor viewer window (open it "
                        "later from the dashboard's Windows card) / センサー"
                        "ビューアを開かずに起動（後でダッシュボードから開ける）")
    p.add_argument("--sensor-viewer-res", default="1440x720",
                   help="Sensor viewer window size, WxH (default 1440x720)")
    p.add_argument("--vehicle-filter", default="vehicle.*",
                   help='Hero vehicle blueprint filter, e.g. '
                        '"vehicle.tesla.model3" (default vehicle.*)')
    p.add_argument("--start-vehicle", default="",
                   help="first hero car, e.g. vehicle.mazda.rx7 (Backspace cycles "
                        "through --vehicle-filter)")
    p.add_argument("--control-res", "--res", dest="control_res", default="1080p",
                   help="Driving window resolution: WxH or 720p / 1080p / 1440p / "
                        "4k (default 1080p)")
    p.add_argument("--fov", type=float, default=100.0,
                   help="Driver-view field of view in degrees (default 100)")
    p.add_argument("--autopilot", action="store_true",
                   help="Start the hero vehicle in autopilot")
    p.add_argument("--agent", default=None, metavar="POLICY",
                   help='Also launch an autonomous ego vehicle beside the '
                        'human (HIL co-simulation, docs/RL_DESIGN.md): '
                        '"autopilot" or "model:PATH". '
                        '/ 人間と同一ワールドに自動運転ego車を追加起動 '
                        '(HIL共走)。"autopilot" か "model:PATH" を指定。')
    p.add_argument("--delivery", type=int, default=0, metavar="N",
                   help="Also run N car + drone delivery vans (delivery_sim.py, "
                        "docs/DELIVERY.md); each order goes to exactly one van "
                        "/ 車+ドローンの配送車を N 台起動（各注文は 1 台のみが担当）")
    p.add_argument("--delivery-args", default="",
                   help='Extra options for delivery_sim.py, e.g. '
                        '"--first-now --drone-prob 0.7" '
                        '/ delivery_sim.py への追加オプション')
    p.add_argument("--drone-cams", action="store_true",
                   help="open the drone camera wall (drone_viewer.py) at start; needs "
                        "--delivery (without this flag it can still be opened from the "
                        "dashboard's Windows card) / ドローンカメラ一覧を起動時に開く")
    p.add_argument("--drone-cams-res", default="1280x720",
                   help="drone camera wall window size WxH (default 1280x720)")
    p.add_argument("--ffb", action="store_true",
                   help="Also run carla_ffb.py (G923 force feedback on the hero) "
                        "/ carla_ffb.py（hero の G923 フォースフィードバック）も起動")
    p.add_argument("--ffb-strength", type=float, default=None,
                   help="carla_ffb.py --strength (overall force multiplier) "
                        "/ フォースフィードバック全体の強さ")
    p.add_argument("--keyboard", action="store_true",
                   help="Drive with W/A/S/D instead of the G923 wheel "
                        "(automatic anyway when no wheel is detected) "
                        "/ G923ホイールの代わりにW/A/S/Dで運転する"
                        "（ホイール未検出時は自動で有効）")
    p.add_argument("--no-log", action="store_true",
                   help="Disable dashboard CSV session logging")
    p.add_argument("--no-sim", action="store_true",
                   help="Skip CarlaUE4.sh launch (assume it's already running)")
    p.add_argument("--fps", type=int, default=60,
                   help="Driving window frame cap (default: 60) "
                        "/ 運転ウィンドウの上限FPS（既定 60）")
    p.add_argument("--mirror-fps", type=float, default=30.0,
                   help="Rear/left/right mirror camera rate (default: 30). "
                        "Each mirror is an extra render pass; 0 = uncapped "
                        "/ リア・左右ミラーカメラのレート（既定 30）。"
                        "ミラー1枚が追加の描画パス。0 で無制限")
    p.add_argument("--viewer-fps", type=int, default=30,
                   help="Sensor viewer window frame cap (default: 30) "
                        "/ センサービューアの上限FPS（既定 30）")
    p.add_argument("--sensor-fps", type=float, default=20.0,
                   help="Sensor viewer panel sensor rate (default: 20); "
                        "0 = uncapped "
                        "/ センサービューア各パネルのレート（既定 20）。"
                        "0 で無制限")
    p.add_argument("--no-hybrid", action="store_true",
                   help="Give every vehicle in the town full physics instead "
                        "of only those within 70 m of the hero (much more "
                        "CPU) / hero から70m以内だけでなく町中の全車両に"
                        "物理演算を行う（CPU負荷は大幅に増加）")
    p.add_argument("--carla-args", default="",
                   help='Extra args for CarlaUE4.sh, e.g. "-quality-level=Low"')
    p.add_argument("--gpu", choices=("auto", "nvidia", "off"), default="auto",
                   help="Render CarlaUE4 on the NVIDIA dGPU via PRIME offload: "
                        "auto = only when prime-select reports on-demand "
                        "/ PRIME オフロードで CarlaUE4 を NVIDIA GPU で描画: "
                        "auto は on-demand 検出時のみ")
    p.add_argument("--boot-timeout", type=float, default=45.0,
                   help="Seconds to wait for CARLA RPC port")
    p.add_argument("--stall-timeout", type=float, default=20.0,
                   help="Restart the traffic tick master if the world stops "
                        "advancing for this many seconds; 0 disables the "
                        "freeze watchdog (default: 20) "
                        "/ ワールドがこの秒数だけ進まなければ交通ティック"
                        "マスタを再起動する。0 でフリーズ監視を無効化"
                        "（既定 20）")
    p.add_argument("--stall-grace", type=float, default=90.0,
                   help="Seconds after starting the traffic tick master "
                        "before the freeze watchdog arms; spawning a full "
                        "town of traffic takes a while (default: 90) "
                        "/ 交通ティックマスタ起動後、フリーズ監視を有効化"
                        "するまでの秒数。町全体の交通生成には時間がかかる"
                        "（既定 90）")
    args = p.parse_args()

    if not VENV_PY.exists():
        print("[pipeline] ERROR: venv python not found at %s" % VENV_PY)
        return 1

    children = []  # list of (proc, name) / (proc, name) のリスト
    # True if the user runs their own CARLA (--no-sim): then we never kill
    # it. Auto-reused leftovers from a previous run ARE killed on shutdown.
    # ユーザー自身が CARLA を管理する場合（--no-sim 明示指定）は殺さない。
    # 前回実行の残骸を自動再利用した場合は終了時に必ず殺す。
    user_owns_carla = args.no_sim
    carla_proc = None      # our CarlaUE4 child, if we started it
                           # 自前で起動した CarlaUE4 の子プロセス

    def terminate_all():
        # the dashboard's Windows card must not offer controls of a dead
        # pipeline / 終了したパイプラインの操作をダッシュボードに出さない
        windows_control.clear(windows_control.STATUS_PATH)
        for proc, name in reversed(children):
            if proc.poll() is None:
                print("[pipeline] terminating %s (pid=%d)" % (name, proc.pid))
                signal_group(proc, signal.SIGTERM)
        deadline = time.time() + 5.0
        for proc, name in reversed(children):
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                print("[pipeline] killing %s (pid=%d)" % (name, proc.pid))
                signal_group(proc, signal.SIGKILL)
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    pass
        # Final sweep: CarlaUE4 sometimes survives (it ignores SIGTERM, or
        # it was reused from a previous run and is not our child). Make
        # sure port 2000 is actually released before we return.
        # 最終処理：CarlaUE4 は SIGTERM を無視したり、前回実行から再利用
        # されて子プロセスでない場合に生き残ることがある。終了前に
        # ポート2000が確実に解放されたことを確認する。
        if not user_owns_carla:
            for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
                try:
                    subprocess.call(["pkill", "-9", "-f", pat])
                except Exception:
                    pass
            deadline = time.time() + 8.0
            while time.time() < deadline and port_in_use(args.port):
                time.sleep(0.5)
            if port_in_use(args.port):
                print("[pipeline] WARNING: port %d is still in use "
                      "(check: lsof -i:%d)" % (args.port, args.port))
            else:
                print("[pipeline] port %d released" % args.port)

    def on_signal(signum, _frame):
        # Ignore further Ctrl+C while cleaning up: a second one used to
        # abort terminate_all halfway, leaving CarlaUE4 holding port 2000.
        # 後始末中は追加の Ctrl+C を無視する。以前は2回目の Ctrl+C が
        # terminate_all を中断し、CarlaUE4 がポート2000を掴んだままになった。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print("\n[pipeline] signal %d -> shutting down" % signum)
        terminate_all()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        # A CARLA server already listening on the RPC port means a previous
        # instance is still alive. Starting a second CarlaUE4 makes the two
        # fight over the GPU (typically a signal-11 crash), so reuse it.
        # RPCポートで既にCARLAサーバが待ち受けていれば、前のインスタンスが
        # まだ生きている。2つ目のCarlaUE4を起動するとGPUを取り合って
        # クラッシュする（典型的には signal-11）ため、既存のものを再利用する。
        if not args.no_sim and port_in_use(args.port):
            if carla_alive(args.host, args.port):
                print("[pipeline] port %d already in use -> reusing the "
                      "healthy running CARLA (it will be shut down when "
                      "this pipeline exits)" % args.port)
                args.no_sim = True
                if reset_world_async(args.host, args.port):
                    print("[pipeline] cleared a stale synchronous-mode world "
                          "left by a previous run (nothing was stepping it) "
                          "/ 前回実行が残した同期モード（誰も進めていない"
                          "状態）を解除した")
            else:
                # A leftover CARLA is holding the port but not answering:
                # kill it and start fresh.
                # 応答しない CARLA の残骸がポートを掴んでいる：
                # 強制終了して新規に起動する。
                print("[pipeline] port %d held by an unresponsive CARLA "
                      "-> killing stale processes" % args.port)
                for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
                    subprocess.call(["pkill", "-9", "-f", pat])
                deadline = time.time() + 10.0
                while time.time() < deadline and port_in_use(args.port):
                    time.sleep(0.5)
                if port_in_use(args.port):
                    print("[pipeline] ERROR: port %d is still in use by a "
                          "non-CARLA process (check: lsof -i:%d)"
                          % (args.port, args.port))
                    return 1

        def start_carla():
            """Launch CarlaUE4 and wait for its RPC port. / CarlaUE4 を起動しポートを待つ。"""
            nonlocal carla_proc
            sh = CARLA_ROOT / "CarlaUE4.sh"
            if not sh.exists():
                print("[pipeline] ERROR: %s not found" % sh)
                return False
            cmd = [str(sh)]
            # The rig has no audio; the UE4 mixer thread is pure CPU.
            # 本環境は音声を使わない。UE4のミキサースレッドはCPUのみを消費する。
            cmd.append("-nosound")
            if args.carla_args:
                cmd += args.carla_args.split()
            gpu_env = nvidia_offload_env(args.gpu)
            if gpu_env is not None:
                print("[pipeline] NVIDIA PRIME offload enabled for CarlaUE4 "
                      "/ CarlaUE4 を NVIDIA GPU で描画します")
            carla_proc = launch(cmd, CARLA_ROOT, "CarlaUE4", gpu_env)
            children.insert(0, (carla_proc, "CarlaUE4"))
            print("[pipeline] waiting for %s:%d (timeout %.0fs)..."
                  % (args.host, args.port, args.boot_timeout))
            if not wait_for_port(args.host, args.port, args.boot_timeout):
                print("[pipeline] CARLA did not open RPC port in time")
                return False
            print("[pipeline] CARLA ready")
            time.sleep(2.0)
            return True

        def load_startup_map(name):
            cmd = [str(VENV_PY), str(UTIL_DIR / "config.py"),
                   "--host", args.host, "--port", str(args.port),
                   "--map", name]
            print("[pipeline] loading map %s" % name)
            rc = subprocess.call(cmd, cwd=str(UTIL_DIR))
            if rc != 0:
                print("[pipeline] map load failed (rc=%d), continuing anyway" % rc)
            time.sleep(3.0)

        if not args.no_sim:
            if not start_carla():
                terminate_all()
                return 1

        if args.map:
            load_startup_map(args.map)

        # A request left by an earlier run must not switch this one.
        # 以前の実行が残した依頼で今回の実行が切り替わらないようにする。
        map_switch.clear_request()

        # Helper processes we can relaunch if they crash mid-session.
        # セッション中にクラッシュした場合に再起動できる補助プロセス。
        helper_cmds = {}   # name -> cmd list / 名前 -> コマンドリスト
        # generate_traffic is the synchronous tick master: a restart used
        # to be near-useless because the fresh instance saw the dead one's
        # leftover synchronous_mode flag and demoted itself to a passive
        # waiter. It now probes for an actual tick and reclaims mastery, so
        # restarting it genuinely revives a frozen world -- allow a few.
        # generate_traffic は同期ティックマスタである。以前は再起動しても
        # 新しいインスタンスが死んだ側の synchronous_mode フラグを見て受動
        # 側に回るため、ほぼ無意味だった。現在は実際のティックを確認して
        # マスタを取り戻すため、再起動で停止したワールドが実際に復帰する。
        # よって回数に余裕を持たせる。
        restart_left = {}
        heartbeat = BASE_DIR / "traffic_heartbeat.json"
        watchdog_armed_at = float("inf")

        # The dashboard keeps its port across map switches, so an open
        # browser tab reconnects to the same URL.
        # ダッシュボードはマップ切替後も同じポートを使う（ブラウザのタブが
        # 同じ URL に再接続できるように）。
        dport = None
        if not args.no_dashboard:
            dport = pick_free_port(args.dashboard_port)
            if dport != args.dashboard_port:
                print("[pipeline] dashboard port %d busy -> using %d"
                      % (args.dashboard_port, dport))

        # The optional windows, with the command line they get at launch and
        # when the dashboard opens them later (windows_control.py).
        # 任意ウィンドウと、その起動コマンド（起動時もダッシュボードから
        # 後で開く時も同じ）。
        window_cmds = {
            "sensor_viewer": [str(VENV_PY), "sensor_viewer.py",
                              "--host", args.host, "-p", str(args.port),
                              "--res", args.sensor_viewer_res,
                              "--fps", str(args.viewer_fps),
                              "--sensor-fps", str(args.sensor_fps)],
            # CCTV-style wall of drone cameras; a camera exists only while
            # its drone flies. / ドローンカメラ一覧（飛行中のみ描画）
            "drone_cams": [str(VENV_PY), "drone_viewer.py",
                           "--host", args.host, "-p", str(args.port),
                           "--res", args.drone_cams_res],
        }
        unavailable = {}
        if args.delivery <= 0:
            unavailable["drone_cams"] = "needs --delivery N (no drones without vans)"
        windows = windows_control.WindowSupervisor(
            wanted={"sensor_viewer": not args.no_sensor_viewer,
                    "drone_cams": args.drone_cams},
            cmds=window_cmds,
            launch=lambda cmd, name: launch(cmd, BASE_DIR, name),
            stop=lambda procs: stop_procs(procs, 8.0),
            unavailable=unavailable,
            control_path=windows_control.CONTROL_PATH,
            status_path=windows_control.STATUS_PATH)

        def start_clients(map_name, start_vehicle):
            """Start every client of the world (all but CarlaUE4) for
            ``map_name``: at startup, and again after each F5 map switch with
            the same options. Returns the driving window's process.
            ワールドの全クライアント（CarlaUE4 以外）を ``map_name`` 用に起動
            する。起動時と、F5 のマップ切替の後に同じオプションで再度呼ぶ。
            運転ウィンドウのプロセスを返す。"""
            nonlocal watchdog_armed_at
            helper_cmds.clear()
            restart_left.clear()
            # (the optional windows restart through `windows` instead)
            # （任意ウィンドウの再起動は `windows` が扱う）
            restart_left.update({"generate_traffic": 3, "dashboard": 3,
                                 "agent": 3, "delivery": 3,
                                 "ffb": 3, "traction_fx": 3,
                                 "signal_timing": 3})
            watchdog_armed_at = float("inf")

            if not args.no_traffic:
                cmd = [str(VENV_PY), "generate_traffic.py",
                       "--host", args.host, "--port", str(args.port),
                       "--tm-port", str(args.tm_port),
                       "-n", str(args.vehicles),
                       "-w", str(args.walkers),
                       "--cyclists", str(args.cyclists if args.vehicles or args.walkers else 0),
                       "--wheelchair-share", str(args.wheelchair_share),
                       "--runners", str(args.runners)]
                lht = map_is_left_hand(map_name)
                if lht and not args.no_hybrid:
                    # In hybrid mode the traffic manager moves far-away cars along
                    # the lane in the right-hand direction: on a left-hand map
                    # they step back and forth on the spot. Full physics for all.
                    # 左側通行マップではハイブリッドの遠方車両が往復して止まるため無効化。
                    print("[pipeline] %s is left-hand traffic -> hybrid physics off (all "
                          "cars fully simulated) / 左側通行のためハイブリッド無効" % map_name)
                if not args.no_hybrid and not lht:
                    # Full wheeled-vehicle physics for every car in the town runs
                    # on the simulator's CPU whether or not anyone can see it.
                    # Hybrid mode keeps physics only within 70 m of the hero and
                    # moves the rest along their waypoints, which is by far the
                    # largest single CPU saving in this rig. Traffic the driver
                    # can actually interact with is unaffected.
                    # 町中の全車両の四輪物理演算は、見えているかどうかに関係なく
                    # シミュレータのCPUで実行される。ハイブリッドモードでは
                    # hero から70m以内のみ物理演算を行い、残りはウェイポイントに
                    # 沿って移動させる。本環境で最も効果の大きいCPU削減であり、
                    # ドライバーが実際に関わる範囲の交通は影響を受けない。
                    cmd.append("--hybrid")
                helper_cmds["generate_traffic"] = cmd
                try:
                    heartbeat.unlink()
                except OSError:
                    pass
                children.append((launch(cmd, BASE_DIR, "generate_traffic"),
                                 "generate_traffic"))
                watchdog_armed_at = time.time() + args.stall_grace
                time.sleep(2.0)

            if dport is not None:
                # after a map switch: give the old dashboard's socket a moment
                # マップ切替後：旧ダッシュボードのソケット解放を少し待つ
                deadline = time.time() + 5.0
                while time.time() < deadline and port_in_use(dport):
                    time.sleep(0.5)
                cmd = [str(VENV_PY), "dashboard.py",
                       "--host", args.host, "--port", str(args.port),
                       "--dashboard-port", str(dport),
                       "--log-dir", str(DATA_DIR)]
                if args.no_log:
                    cmd.append("--no-log")
                helper_cmds["dashboard"] = cmd
                children.append((launch(cmd, BASE_DIR, "dashboard"),
                                 "dashboard"))
                print("[pipeline] dashboard will be at http://localhost:%d/"
                      % dport)

            if args.agent:
                # Autonomous ego vehicle sharing the world with the human hero
                # (HIL co-simulation). / 人間のhero車と同一ワールドを共有する
                # 自動運転ego車（HIL共走）。
                cmd = [str(VENV_PY), "agent_drive.py",
                       "--host", args.host, "--port", str(args.port),
                       "--tm-port", str(args.tm_port),
                       "--policy", args.agent]
                helper_cmds["agent"] = cmd
                children.append((launch(cmd, BASE_DIR, "agent"), "agent"))

            if args.delivery > 0:
                # Car + drone delivery vans. With the traffic script running it
                # owns the synchronous tick, so the vans only follow it
                # (--passive); without traffic nobody steps the world at a fixed
                # rate, so delivery_sim becomes the tick master itself.
                # The map is already loaded (also after an F5 switch), so
                # --map "" keeps it: delivery_sim reads the map's buildings,
                # addresses and yard from the loaded world itself.
                # 車+ドローンの配送車。交通スクリプトが同期 tick を持つ場合は追従
                # のみ（--passive）。交通なしでは誰も固定レートで進めないため、
                # delivery_sim 自身が tick マスタになる。マップは読込済み（F5
                # 切替後も同様）なので --map "" で現在のマップを使う。建物・住所・
                # ヤードは delivery_sim が読込済みワールドから取得する。
                cmd = [str(VENV_PY), "delivery_sim.py",
                       "--host", args.host, "--port", str(args.port),
                       "--map", "", "--vans", str(args.delivery),
                       "--vehicles", "0", "--walkers", "0",
                       "--tm-port", str(args.tm_port)]
                if not args.no_traffic:
                    cmd.append("--passive")
                # start delivering at once instead of after a 20-90 s wait
                # 20〜90 秒待たずにすぐ配送を始める
                cmd.append("--first-now")
                if args.delivery_args:
                    cmd += shlex.split(args.delivery_args)
                helper_cmds["delivery"] = cmd
                children.append((launch(cmd, BASE_DIR, "delivery"), "delivery"))
            elif args.drone_cams:
                print("[pipeline] --drone-cams needs --delivery N (no drones without vans) "
                      "/ --drone-cams には --delivery が必要")

            if args.ffb:
                # carla_ffb.py only reads the world and restarts itself on errors.
                # A second copy would fight the first for the wheel's force
                # feedback, so reuse one that is already running.
                # carla_ffb.py はワールドを読むだけで、エラー時は自身で再起動する。
                # 2 つ目が動くとホイールの FFB を奪い合うため、既存のものを使う。
                running = subprocess.run(["pgrep", "-f", "carla_ffb.py"],
                                         capture_output=True).stdout.split()
                if running:
                    print("[pipeline] carla_ffb.py is already running (pid %s) -> not "
                          "starting a second copy / carla_ffb.py は既に起動中のため"
                          "2 つ目は起動しない"
                          % ", ".join(r.decode() for r in running))
                else:
                    cmd = [str(VENV_PY), str(FFB_SCRIPT),
                           "--host", args.host, "--port", str(args.port)]
                    if args.ffb_strength is not None:
                        cmd += ["--strength", str(args.ffb_strength)]
                    helper_cmds["ffb"] = cmd
                    children.append((launch(cmd, BASE_DIR, "ffb"), "ffb"))

            if not args.no_traction_fx:
                # Skid marks behind the hero's sliding tyres. It only reads the
                # world, like the dashboard.
                # hero の滑ったタイヤのスキッドマーク（読むだけ）。
                cmd = [str(VENV_PY), "traction_fx.py",
                       "--host", args.host, "--port", str(args.port)]
                helper_cmds["traction_fx"] = cmd
                children.append((launch(cmd, BASE_DIR, "traction_fx"), "traction_fx"))

            profile = map_signal_timing(map_name)
            if profile is not None:
                # Real signal timing (maps.json "signal_timing"): cycle and
                # green/yellow/all-red per 5-minute slot, only on that map.
                # 実際の信号時間（そのマップのみ、5 分毎に再設定）。
                cmd = [str(VENV_PY), "signal_timing.py", "apply", "--watch",
                       "--host", args.host, "--port", str(args.port),
                       "--profile", str(profile), "--only-map", map_name]
                helper_cmds["signal_timing"] = cmd
                children.append((launch(cmd, BASE_DIR, "signal_timing"), "signal_timing"))

            # The optional viewer windows (sensor viewer, drone cameras): the
            # ones that are open now -- the launch flags at startup, after an
            # F5 switch whatever the user opened or closed meanwhile.
            # 任意のビューアウィンドウ：今開いているもの（起動時は起動
            # オプション、F5 切替後はそれまでにユーザーが開閉した状態）。
            windows.start_wanted(children)

            cmd = [str(VENV_PY), str(CONTROL_SCRIPT),
                   "--host", args.host, "--port", str(args.port),
                   "--res", args.control_res,
                   "--filter", args.vehicle_filter,
                   "--start-vehicle", start_vehicle,
                   "--fov", str(args.fov),
                   "--fps", str(args.fps),
                   "--mirror-fps", str(args.mirror_fps),
                   # F5 asks us to switch the map instead of loading it there
                   # F5 はマップを自分で読まず、パイプラインに切替を依頼する
                   "--map-request", map_switch.REQUEST_PATH]
            if args.autopilot:
                cmd.append("--autopilot")
            # The hero's autopilot (--autopilot or the P key) gets its own TM:
            # set_autopilot() would otherwise create the traffic TM server in
            # manual_control before generate_traffic starts, and the traffic
            # cars would join a TM that is never stepped (they stand still).
            # hero の自動運転は別ポートの TM を使う（交通車両が動かなくなるため）。
            cmd += ["--tm-port", str(args.tm_port + 1)]
            if args.keyboard:
                cmd.append("--keyboard")
            mc = launch(cmd, BASE_DIR, "manual_control")
            children.append((mc, "manual_control"))
            return mc

        def stop_clients():
            """Stop every world client but CarlaUE4, tick master first.
            CarlaUE4 以外の全クライアントを止める（tick マスタが最初）。"""
            live = [(p_, n) for p_, n in children if n != "CarlaUE4"]
            for group in switch_stop_groups([n for _, n in live]):
                procs = [(p_, n) for p_, n in live if n in group]
                # the tick master destroys its cars in chunks; give it time
                # tick マスタは車両を分割して破棄するため猶予を長めに
                stop_procs(procs, 20.0 if "generate_traffic" in group else 10.0)
            children[:] = [(p_, n) for p_, n in children if n == "CarlaUE4"]
            try:
                heartbeat.unlink()
            except OSError:
                pass

        def restart_carla(map_name):
            """Fallback when load_world fails: kill CarlaUE4, start one fresh
            (never a second one beside it) and load the map.
            load_world 失敗時の代替：CarlaUE4 を終了してから新しく起動し
            （2 つ目を並べて起動はしない）、マップを読み込む。"""
            nonlocal carla_proc
            if user_owns_carla:
                print("[pipeline] --no-sim: this CARLA is yours, not restarting it "
                      "/ --no-sim のため CARLA は再起動しない")
                return False
            for p_, n in [c for c in children if c[1] == "CarlaUE4"]:
                stop_procs([(p_, n)], 10.0)
            children[:] = [c for c in children if c[1] != "CarlaUE4"]
            carla_proc = None
            for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
                subprocess.call(["pkill", "-9", "-f", pat])
            deadline = time.time() + 15.0
            while time.time() < deadline and port_in_use(args.port):
                time.sleep(0.5)
            if port_in_use(args.port):
                print("[pipeline] ERROR: port %d still in use, not starting CARLA "
                      "(check: lsof -i:%d)" % (args.port, args.port))
                return False
            args.no_sim = False
            if not start_carla():
                return False
            return load_world_map(args.host, args.port, map_name)

        current_map = args.map or world_map_name(args.host, args.port)
        start_vehicle = args.start_vehicle
        while True:
            mc = start_clients(current_map, start_vehicle)
            windows.write_status(children, "running", current_map)

            # Supervise: wait for manual_control, restart crashed helpers.
            # 監視: manual_control の終了を待ち、落ちた補助プロセスを再起動する。
            while mc.poll() is None:
                time.sleep(1.0)

                # The simulator itself is not restartable: every actor, sensor
                # and the hero vehicle die with it. When it goes, the driving
                # window just stops updating -- which is what "the simulation
                # froze" looks like from the driver's seat -- so say plainly
                # what happened instead of supervising a world that no longer
                # exists.
                # シミュレータ本体は再起動できない。全アクター・センサー・
                # hero 車両が道連れになるためである。落ちると運転ウィンドウ
                # が更新を止めるだけで、運転席からは「シミュレーションが
                # 固まった」ようにしか見えない。存在しないワールドを監視し
                # 続けず、何が起きたかを明示する。
                if carla_proc is not None and carla_proc.poll() is not None:
                    print("[pipeline] CarlaUE4 exited (rc=%s) -> the world is "
                          "gone, shutting down / CarlaUE4 が終了したため"
                          "ワールドが消失。シャットダウンする"
                          % carla_proc.returncode)
                    faults = recent_gpu_faults()
                    if faults:
                        print("[pipeline] the kernel logged a GPU fault -- this "
                              "is a driver/GPU crash, not a CARLA bug "
                              "/ カーネルが GPU 障害を記録している。CARLA の"
                              "不具合ではなくドライバ/GPU のクラッシュである:")
                        for line in faults:
                            print("[pipeline]   %s" % line)
                    else:
                        print("[pipeline] check %s/CarlaUE4.sh output above, and "
                              "`journalctl -k | grep Xid` for a GPU fault "
                              "/ 上記の CarlaUE4.sh の出力と、GPU 障害について"
                              "は `journalctl -k | grep Xid` を確認すること"
                              % CARLA_ROOT)
                    terminate_all()
                    return 1

                # Dashboard Windows card: open / close requests, then the
                # status it shows. / ダッシュボードの開閉依頼と状態ファイル。
                windows.poll_requests(children)

                # An optional window that exited by itself: exit code 0 is the
                # user closing it (close button / Esc / Q) -> it stays closed;
                # anything else is a crash -> a bounded number of restarts.
                # 自分で終了した任意ウィンドウ：終了コード 0 はユーザーが閉じた
                # （閉じたまま）、それ以外はクラッシュ（回数制限付きで再起動）。
                for proc, name in [c for c in children
                                   if c[1] in windows_control.WINDOWS
                                   and c[0].poll() is not None]:
                    if windows.handle_exit(children, name, proc.returncode):
                        continue
                    children[:] = [c for c in children if c[0] is not proc]
                    if windows.take_restart(name):
                        print("[pipeline] %s died (rc=%s) -> restarting "
                              "(%d restart(s) left)"
                              % (name, proc.returncode, windows.restart_left[name]))
                        windows.open(children, name, reason="restart")
                    else:
                        print("[pipeline] %s died (rc=%s), no restarts left "
                              "(open it again from the dashboard) / 再起動回数を"
                              "使い切った" % (name, proc.returncode))
                windows.write_status(children, "running", current_map)

                for i, (proc, name) in enumerate(children):
                    if proc is mc or proc.poll() is None:
                        continue
                    if name in helper_cmds and restart_left.get(name, 0) > 0:
                        restart_left[name] -= 1
                        print("[pipeline] %s died (rc=%s) -> restarting "
                              "(%d restart(s) left)"
                              % (name, proc.returncode, restart_left[name]))
                        if name == "generate_traffic":
                            try:
                                heartbeat.unlink()
                            except OSError:
                                pass
                            watchdog_armed_at = time.time() + args.stall_grace
                        children[i] = (
                            launch(helper_cmds[name], BASE_DIR, name), name)

                # Freeze watchdog. A tick master that is alive but no longer
                # stepping the world -- a wedged RPC, a stuck render thread --
                # leaves every other component staring at a still frame while
                # poll() reports a perfectly healthy process, so the loop above
                # never notices. generate_traffic.py touches
                # traffic_heartbeat.json on every tick; a stale heartbeat means
                # the world has stopped advancing, whatever the process table
                # says. Restarting the master is what unfreezes it.
                # フリーズ監視。ティックマスタがプロセスとしては生きているのに
                # ワールドを進めなくなると（RPCの詰まり、描画スレッドの停止）、
                # 他の全コンポーネントは静止画を見続けるが poll() は正常と報告
                # するため、上のループでは検知できない。generate_traffic.py は
                # 毎ティック traffic_heartbeat.json を更新するので、その停止は
                # プロセス表の内容に関わらずワールドが進んでいないことを示す。
                # マスタを再起動することで停止が解ける。
                if ("generate_traffic" in helper_cmds and args.stall_timeout > 0
                        and time.time() > watchdog_armed_at):
                    age = heartbeat_age(heartbeat)
                    if age is None or age > args.stall_timeout:
                        for i, (proc, name) in enumerate(children):
                            if name != "generate_traffic":
                                continue
                            if restart_left.get(name, 0) <= 0:
                                print("[pipeline] WARNING: simulation frozen (no "
                                      "tick for %s s) and no traffic restarts "
                                      "left / シミュレーションが停止したが交通の"
                                      "再起動回数を使い切った"
                                      % ("?" if age is None else "%.0f" % age))
                                watchdog_armed_at = float("inf")
                                break
                            restart_left[name] -= 1
                            print("[pipeline] simulation frozen: no tick for %s s "
                                  "-> restarting the traffic tick master "
                                  "(%d restart(s) left) / ワールドが %s 秒進んで"
                                  "いないため交通ティックマスタを再起動する"
                                  % ("?" if age is None else "%.0f" % age,
                                     restart_left[name],
                                     "?" if age is None else "%.0f" % age))
                            signal_group(proc, signal.SIGTERM)
                            try:
                                proc.wait(timeout=5.0)
                            except subprocess.TimeoutExpired:
                                signal_group(proc, signal.SIGKILL)
                                try:
                                    proc.wait(timeout=2.0)
                                except subprocess.TimeoutExpired:
                                    pass
                            try:
                                heartbeat.unlink()
                            except OSError:
                                pass
                            children[i] = (
                                launch(helper_cmds[name], BASE_DIR, name), name)
                            watchdog_armed_at = time.time() + args.stall_grace
                            break
            rc = mc.returncode

            # F5 / Shift+F5: the driving window asked for another map and
            # closed. Switch it for every client while CarlaUE4 keeps running,
            # then start them all again with the same options and hero car.
            # F5 / Shift+F5：運転ウィンドウが別マップを依頼して終了した。
            # CarlaUE4 は動かしたまま全クライアントごと切り替え、同じ
            # オプション・同じ自車で全て再起動する。
            req = take_map_request(rc)
            if req is None:
                break
            target = req["map"]
            print("[pipeline] map switch %s -> %s requested by manual_control "
                  "(rc=%d, hero %s) / マップ切替の依頼"
                  % (req.get("from") or current_map, target, rc,
                     req["hero"] or start_vehicle or "-"))
            windows.write_status(children, "switching", target)
            how = switch_map(
                target, stop_clients,
                lambda: reset_world_async(args.host, args.port),
                lambda m: load_world_map(args.host, args.port, m),
                restart_carla)
            if how == "failed":
                terminate_all()
                return 1
            print("[pipeline] map %s %s -> restarting the clients / マップ切替"
                  "完了、クライアントを再起動" % (
                      target, "loaded" if how == "loaded" else
                      "loaded after a CARLA restart"))
            time.sleep(2.0)
            current_map = target
            start_vehicle = req["hero"] or start_vehicle

        print("[pipeline] manual_control exited rc=%d -> shutting down" % rc)
        terminate_all()
        return rc

    except Exception as exc:
        print("[pipeline] error: %s" % exc)
        terminate_all()
        return 1


if __name__ == "__main__":
    sys.exit(main())
