#!/usr/bin/env python
"""Live telemetry dashboard for the CARLA hero vehicle.

Connects to a running CARLA simulator, looks for the actor whose role_name
is 'hero', attaches GNSS / IMU / collision sensors to it, and serves a
small Flask web page with live vehicle state.

Run:
    python dashboard.py --host 127.0.0.1 --port 2000 --dashboard-port 5000

Then open http://localhost:5000 in a browser.

CARLA のヒーロー車両向けライブテレメトリダッシュボード。

起動中の CARLA シミュレータに接続し、role_name が 'hero' のアクターを探して
GNSS / IMU / 衝突センサーを取り付け、車両状態をライブ表示する小さな
Flask ウェブページを提供する。

実行方法:
    python dashboard.py --host 127.0.0.1 --port 2000 --dashboard-port 5000

その後、ブラウザで http://localhost:5000 を開く。
"""

import argparse
import csv
import glob
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from collections import deque

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from flask import Flask, Response, jsonify, request

from delivery import live as delivery_live
import ffb_settings
import vehicle_tuning


STATE_LOCK = threading.Lock()
STATE = {
    "connected": False,
    "hero_found": False,
    "map": "",
    "speed_kmh": 0.0,
    "speed_limit_kmh": 0.0,
    "speed_delta_kmh": 0.0,
    "throttle": 0.0,
    "brake": 0.0,
    "steer": 0.0,
    "reverse": False,
    "hand_brake": False,
    "gear": 0,
    "location": {"x": 0.0, "y": 0.0, "z": 0.0},
    "heading_deg": 0.0,
    "compass_deg": 0.0,
    "gnss": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
    "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0},
    # Derived dynamics / 派生的な車両運動量
    "lat_accel": 0.0,
    "lon_accel": 0.0,
    "vert_accel": 0.0,
    "yaw_rate_deg": 0.0,
    "jerk": 0.0,
    "g_force": 0.0,
    # Safety / environment / 安全・環境関連
    "traffic_light": "None",
    "at_traffic_light": False,
    "front_distance": -1.0,
    "ttc": -1.0,
    "hard_brake_count": 0,
    "collision_count": 0,
    # Lane / road / 車線・道路情報
    "on_road": False,
    "lane_width": 0.0,
    "lane_offset": 0.0,   # signed: + = right of lane centre / 符号付き：+ は車線中心より右側
    "lane_id": 0,
    "road_id": 0,
    "at_junction": False,
    # Driver-performance metrics (human-in-the-loop research) / 運転パフォーマンス指標（HIL研究用）
    "thw": -1.0,                  # time headway to lead vehicle (s) / 先行車への車間時間（秒）
    "sdlp": 0.0,                  # std dev of lateral lane position, 60 s (m)
    # SDLP：直近60秒の車線内横位置の標準偏差（m）
    "steer_reversal_rate": 0.0,   # steering reversals per minute / 1分あたりのステアリング反転回数
    "lane_invasions": 0,
    "red_light_violations": 0,
    "speed_compliance": 100.0,    # % of moving time at/below limit / 制限速度以下で走行した時間の割合（%）
    "jerk_rms": 0.0,
    "events": [],                 # recent driving events (newest first) / 最近の運転イベント（新しい順）
    "log_file": "",
    # Trip / 走行記録
    "trip_km": 0.0,
    "trip_sec": 0.0,
    "avg_kmh": 0.0,
    # Collision / 衝突
    "collision_peak": 0.0,
    "collision_recent": [],
    "collision_hits": [],   # [{age, f, r, i, o}] impact points, vehicle frame
                            # 車両ローカル座標系での衝突位置（前方f・右方r、-1..1）
    # Context / 周辺状況
    "nearby_vehicles": 0,
    "nearby_list": [],      # [{fx, rx, d, yaw, kmh, kind}, ...] relative to hero / ヒーロー車基準の相対値
    # All sensors attached to the hero in the sim (any script)
    # シミュレーション内でヒーロー車に取り付けられた全センサー（他スクリプト含む）
    "sensors": [],          # [{type, f, r}] mounting position in vehicle frame / 車両座標系での取付位置
    "ultrasonic_m": -1.0,   # front obstacle (ultrasonic-style) sensor distance / 前方障害物（超音波型）センサー距離
    "bb": {"x": 2.4, "y": 1.0},  # hero bounding-box half extents (m) / 自車バウンディングボックス半径（m）
    "trail": [],
    "timestamp": 0.0,
    # Weather / environment / 天候・環境
    "weather": {
        "cloudiness": 0.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 0.0,
        "sun_azimuth": 0.0,
        "sun_altitude": 0.0,
        "fog_density": 0.0,
        "wetness": 0.0,
    },
    # Autonomous-driving indicators / 自動運転関連の指標
    "abs_active": False,      # approximate: hard brake + high slip signature
    # 近似値：急ブレーキ＋大スリップの兆候
    "tcs_active": False,      # approximate: high throttle + low accel / 近似値：高スロットル＋低加速度
    "stability": 1.0,         # 0..1, 1 = stable / 0〜1、1 = 安定
    "slip_angle_deg": 0.0,    # angle between heading and velocity / 車体方位と速度ベクトルの角度
    "curvature": 0.0,         # 1/m of current trajectory / 現在の軌跡の曲率（1/m）
    # Vehicle physical / 車両の物理特性
    "vehicle_type": "",
    "num_wheels": 4,
    "mass_kg": 0.0,
    "engine_rpm": 0.0,
    # Turn signal / ウインカー
    "turn_signal": "off",
    # Simulation meta / シミュレーションのメタ情報
    "sim_fps": 0.0,
    "sim_time_sec": 0.0,
}

TRAIL = deque(maxlen=400)
GNSS_DATA = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
IMU_DATA = {"ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0, "compass": 0.0}
# Front obstacle ("ultrasonic") sensor; stale after 0.6 s without an event.
# 前方障害物（超音波）センサー。0.6秒イベントが無ければ無効扱い。
OBST_DATA = {"t": 0.0, "d": -1.0}

# Turn-signal state, shared with the G923 wheel script via turn_signal.json
# (wheel button 11 = left, 10 = right; the dashboard buttons POST /signal).
# While a signal is on, crossing a broken lane marking counts as a lane
# change instead of a lane invasion.
# ウインカー状態。turn_signal.json を介して G923 スクリプトと共有
# （ホイールのボタン11=左・10=右、ダッシュボードのボタンは /signal へPOST）。
# 点灯中に破線を跨いだ場合は車線逸脱ではなく車線変更として扱う。
SIGNAL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "turn_signal.json")
SIGNAL = {"side": "off", "mtime": 0.0}


def _load_signal():
    """Pick up turn-signal toggles written by the wheel script.

    ホイールスクリプトが書き込んだウインカー切替を取り込む。
    """
    try:
        mtime = os.path.getmtime(SIGNAL_PATH)
    except OSError:
        return
    if mtime == SIGNAL["mtime"]:
        return
    SIGNAL["mtime"] = mtime
    try:
        with open(SIGNAL_PATH) as fh:
            side = json.load(fh).get("side", "off")
        SIGNAL["side"] = side if side in ("left", "right") else "off"
    except Exception:
        pass


def _save_signal(side):
    SIGNAL["side"] = side
    try:
        tmp = SIGNAL_PATH + ".tmp"
        with open(tmp, "w") as fh:
            json.dump({"side": side}, fh)
        os.replace(tmp, SIGNAL_PATH)
        SIGNAL["mtime"] = os.path.getmtime(SIGNAL_PATH)
    except Exception:
        pass
COLL_EVENTS = deque(maxlen=200)
# Distinct crash impact points for the vehicle diagram / 車両図用の衝突位置
COLL_HITS = deque(maxlen=20)
EVENTS = deque(maxlen=80)  # (wall_time, type, message) / （実時刻, 種別, メッセージ）

# CSV session log (one per dashboard run), stored under
# <log-root>/<YYYY-MM-DD>/<HH-MM-SS>/telemetry.csv with a summary.json
# next to it that is refreshed every few seconds.
# CSV セッションログ（ダッシュボード実行ごとに1つ）。
# <log-root>/<YYYY-MM-DD>/<HH-MM-SS>/telemetry.csv に保存され、
# 隣に置かれる summary.json は数秒ごとに更新される。
LOG = {"fh": None, "writer": None, "path": "", "dir": "", "root": "",
       "summary_path": "", "last_flush": 0.0, "last_summary": 0.0,
       "date": "", "time": ""}
LOG_COLS = [
    "wall_time", "sim_time", "x", "y", "z", "speed_kmh", "speed_limit_kmh",
    "throttle", "brake", "steer", "gear", "lon_accel", "lat_accel", "jerk",
    "yaw_rate_deg", "slip_angle_deg", "lane_offset", "lane_width", "on_road",
    "front_distance", "ttc", "thw", "sdlp", "steer_reversal_rate",
    "hard_brake_count", "lane_invasions", "red_light_violations",
    "speed_compliance", "collision_peak", "collision_count",
]


def add_event(etype, msg):
    EVENTS.append((time.time(), etype, msg))

# Per-hero derived state (reset on hero change). / ヒーロー車ごとの派生状態（車両変更時にリセット）
DRV_STATE = {}


def _reset_drv_state():
    DRV_STATE.clear()
    DRV_STATE.update({
        "prev_accel": None,
        "prev_t": None,
        "last_pos": None,
        "trip_m": 0.0,
        "hard_brake_count": 0,
        "in_hard_brake": False,
        "trip_start": None,
        # Driver-performance state / 運転パフォーマンス関連の状態
        "lane_invasions": 0,
        "red_light_violations": 0,
        "collision_count": 0,
        "last_coll_t": None,
        "last_red_tl": None,          # traffic-light actor while at a light / 信号待ち中の信号機アクター
        "moving_time": 0.0,
        "speeding_time": 0.0,
        "last_loop_t": None,
        "offset_hist": deque(maxlen=1200),   # (t, signed lane offset) / （時刻, 符号付き車線オフセット）
        "jerk_hist": deque(maxlen=300),
        "steer_prev": None,
        "steer_dir": 0,
        "steer_extreme": 0.0,
        "reversal_times": deque(maxlen=600),
    })
    EVENTS.clear()


_reset_drv_state()


def find_hero(world):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "hero":
            return actor
    return None


def attach_sensors(world, hero):
    bp_lib = world.get_blueprint_library()
    sensors = []

    gnss_bp = bp_lib.find("sensor.other.gnss")
    gnss = world.spawn_actor(
        gnss_bp,
        carla.Transform(carla.Location(x=1.0, z=2.8)),
        attach_to=hero,
    )
    gnss.listen(lambda ev: GNSS_DATA.update(
        {"lat": ev.latitude, "lon": ev.longitude}))
    sensors.append(gnss)

    imu_bp = bp_lib.find("sensor.other.imu")
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=hero)

    def on_imu(ev):
        IMU_DATA.update({
            "ax": ev.accelerometer.x,
            "ay": ev.accelerometer.y,
            "az": ev.accelerometer.z,
            "gx": ev.gyroscope.x,
            "gy": ev.gyroscope.y,
            "gz": ev.gyroscope.z,
            "compass": math.degrees(ev.compass),
        })

    imu.listen(on_imu)
    sensors.append(imu)

    coll_bp = bp_lib.find("sensor.other.collision")
    coll = world.spawn_actor(coll_bp, carla.Transform(), attach_to=hero)

    def on_coll(ev):
        i = ev.normal_impulse
        intensity = math.sqrt(i.x * i.x + i.y * i.y + i.z * i.z)
        now = time.time()
        COLL_EVENTS.append((now, intensity))
        # A single crash produces a burst of contact events; count a new
        # crash only after 2 s without contact.
        # 1回の衝突では接触イベントが連続して発生するため、接触が
        # 2秒間途切れた後にのみ新しい衝突としてカウントする。
        last = DRV_STATE.get("last_coll_t")
        DRV_STATE["last_coll_t"] = now
        if last is None or now - last > 2.0:
            DRV_STATE["collision_count"] = DRV_STATE.get(
                "collision_count", 0) + 1
            try:
                other = ev.other_actor.type_id
            except Exception:
                other = "unknown"
            # Impact point in the vehicle frame (forward f, right r),
            # normalised by the bounding-box half extents to -1..1 so the
            # UI can pin it on the car outline.  Prefer the other actor's
            # position; fall back to the impulse direction for walls etc.
            # 車両ローカル座標系での衝突位置（前方f・右方r）。バウンディング
            # ボックス半径で -1..1 に正規化。相手アクターの位置を優先し、
            # 壁などではインパルス方向から推定する。
            fn, rn = 0.0, 0.0
            try:
                ht = hero.get_transform()
                fwd, right = _yaw_to_forward_right(ht.rotation.yaw)
                ext = hero.bounding_box.extent
                dx = dy = 0.0
                try:
                    ol = ev.other_actor.get_transform().location
                    dx = ol.x - ht.location.x
                    dy = ol.y - ht.location.y
                except Exception:
                    pass
                if abs(dx) < 1e-3 and abs(dy) < 1e-3 and intensity > 0:
                    dx, dy = -i.x / intensity, -i.y / intensity
                fu = (dx * fwd[0] + dy * fwd[1]) / max(ext.x, 0.1)
                ru = (dx * right[0] + dy * right[1]) / max(ext.y, 0.1)
                m = max(abs(fu), abs(ru), 1.0)
                fn, rn = fu / m, ru / m
            except Exception:
                pass
            COLL_HITS.append({
                "t": now, "f": round(fn, 3), "r": round(rn, 3),
                "i": intensity,
                "o": other.split(".", 1)[-1] if other else "unknown"})
            add_event("collision",
                      "with %s (impulse %.0f)" % (other, intensity))

    coll.listen(on_coll)
    sensors.append(coll)

    lane_bp = bp_lib.find("sensor.other.lane_invasion")
    lane = world.spawn_actor(lane_bp, carla.Transform(), attach_to=hero)

    def on_lane(ev):
        kinds = sorted(set(str(m.type).split(".")[-1]
                           for m in ev.crossed_lane_markings))
        # With the turn signal on, crossing broken markings is a legal,
        # signalled lane change — don't count it as a violation.
        # ウインカー点灯中の破線跨ぎは合法な車線変更なので違反に数えない。
        legal = (SIGNAL["side"] != "off" and kinds
                 and all(k in ("Broken", "BrokenBroken") for k in kinds))
        if legal:
            add_event("lane_change",
                      "signalled lane change (%s)" % SIGNAL["side"])
        else:
            DRV_STATE["lane_invasions"] = DRV_STATE.get(
                "lane_invasions", 0) + 1
            add_event("lane", "crossed %s" % ", ".join(kinds))

    lane.listen(on_lane)
    sensors.append(lane)

    # Ultrasonic-style front obstacle sensor (ray-cast proximity probe).
    # 超音波型の前方障害物センサー（レイキャストによる近接プローブ）。
    try:
        obs_bp = bp_lib.find("sensor.other.obstacle")
        obs_bp.set_attribute("distance", "20")
        obs_bp.set_attribute("hit_radius", "0.4")
        obs_bp.set_attribute("only_dynamics", "false")
        ext = hero.bounding_box.extent
        obs = world.spawn_actor(
            obs_bp,
            carla.Transform(carla.Location(x=float(ext.x) + 0.3, z=0.6)),
            attach_to=hero,
        )

        def on_obs(ev):
            OBST_DATA.update({"t": time.time(), "d": float(ev.distance)})

        obs.listen(on_obs)
        sensors.append(obs)
    except Exception as exc:
        print("[dashboard] obstacle sensor unavailable: %s" % exc,
              file=sys.stderr)

    return sensors


def _classify_actor(actor):
    """Rough class of a nearby actor: walker / bike / truck / car.

    周辺アクターの大まかな分類：walker（歩行者）/ bike（二輪）/
    truck（トラック等）/ car（乗用車）。
    """
    tid = actor.type_id
    if tid.startswith("walker."):
        return "walker"
    attrs = actor.attributes
    base = attrs.get("base_type", "")
    if base in ("bicycle", "motorcycle") or \
            attrs.get("number_of_wheels", "4") == "2":
        return "bike"
    if base in ("truck", "van", "bus"):
        return "truck"
    return "car"


def _yaw_to_forward_right(yaw_deg):
    """Return (forward, right) unit 2D vectors for given yaw (degrees, CARLA).

    指定ヨー角（度、CARLA座標系）に対する（前方, 右方）の2D単位ベクトルを返す。
    """
    yr = math.radians(yaw_deg)
    fx, fy = math.cos(yr), math.sin(yr)
    # CARLA/UE4 is left-handed with X forward, Y RIGHT, Z up, so the right
    # vector is forward rotated +90° about Z (matches get_right_vector()).
    # CARLA/UE4 は左手系（X前方・Y右・Z上）なので、右方向ベクトルは
    # forward を Z 軸まわりに +90度 回転したもの（get_right_vector() と一致）。
    rx, ry = math.cos(yr + math.pi / 2.0), math.sin(yr + math.pi / 2.0)
    return (fx, fy), (rx, ry)


def _front_obstacle(hero, others, max_dist=60.0, half_cone_deg=20.0):
    """Nearest actor ahead of hero inside a cone. Return (dist, ttc, actor).

    `others` carries walkers as well as vehicles -- a pedestrian stepping out
    is exactly the front obstacle a driving study wants TTC for.

    ヒーロー車前方のコーン内で最も近いアクターを探し、(距離, TTC, アクター)
    を返す。`others` には車両だけでなく歩行者も含む。飛び出してくる歩行者
    こそ、運転研究で TTC を取りたい前方障害物である。
    """
    t = hero.get_transform()
    (fx, fy), _ = _yaw_to_forward_right(t.rotation.yaw)
    hv = hero.get_velocity()
    hx, hy = t.location.x, t.location.y
    h_fwd_speed = hv.x * fx + hv.y * fy  # m/s along forward / 前方方向の速度（m/s）
    best = (max_dist, -1.0, None)
    cos_lim = math.cos(math.radians(half_cone_deg))
    for a in others:
        al = a.get_location()
        dx, dy = al.x - hx, al.y - hy
        d = math.hypot(dx, dy)
        if d < 0.5 or d > max_dist:
            continue
        # angle check / 角度の確認
        cos_a = (dx * fx + dy * fy) / d
        if cos_a < cos_lim:
            continue
        av = a.get_velocity()
        other_fwd_speed = av.x * fx + av.y * fy
        closing = h_fwd_speed - other_fwd_speed  # m/s positive = approaching / 正なら接近中（m/s）
        ttc = (d / closing) if closing > 0.5 else -1.0
        if d < best[0]:
            best = (d, ttc, a)
    return best


def _open_log(log_dir):
    """Open a session log under <log_dir>/<YYYY-MM-DD>/<HH-MM-SS>/.

    <log_dir>/<YYYY-MM-DD>/<HH-MM-SS>/ 配下にセッションログを開く。
    """
    try:
        day = time.strftime("%Y-%m-%d")
        tod = time.strftime("%H-%M-%S")
        folder = os.path.join(log_dir, day, tod)
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "telemetry.csv")
        fh = open(path, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(LOG_COLS)
        # Flush the header immediately. Rows are flushed only every 2 s, and
        # a session that dies before the first flush -- which is what a
        # CarlaUE4 GPU crash plus the pipeline's SIGKILL sweep looks like --
        # otherwise leaves a 0-byte telemetry.csv with no header, so the run
        # cannot even be identified afterwards.
        # ヘッダは即座にフラッシュする。行のフラッシュは2秒ごとのため、
        # 最初のフラッシュ前にセッションが死ぬと（CarlaUE4 の GPU クラッシュと
        # パイプラインの SIGKILL 掃除がまさにこれ）ヘッダすら無い0バイトの
        # telemetry.csv が残り、後から実行の識別すらできなくなる。
        fh.flush()
        LOG.update({"fh": fh, "writer": writer, "path": path,
                    "dir": folder, "root": log_dir,
                    "summary_path": os.path.join(folder, "summary.json"),
                    "date": day, "time": tod})
        print("[dashboard] logging session to %s" % path)
    except Exception as exc:
        print("[dashboard] could not open log: %s" % exc, file=sys.stderr)


def _write_summary(now):
    """Refresh summary.json for the running session (called every ~5 s).

    実行中セッションの summary.json を更新する（約5秒ごとに呼ばれる）。
    """
    if not LOG.get("summary_path"):
        return
    if now - LOG.get("last_summary", 0.0) < 5.0:
        return
    LOG["last_summary"] = now
    with STATE_LOCK:
        summary = {
            "date": LOG["date"],
            "time": LOG["time"],
            "map": STATE["map"],
            "vehicle": STATE["vehicle_type"],
            "crashes": STATE["collision_count"],
            "out_of_lane": STATE["lane_invasions"],
            "red_light_violations": STATE["red_light_violations"],
            "hard_brakes": STATE["hard_brake_count"],
            "trip_km": round(STATE["trip_km"], 3),
            "trip_sec": round(STATE["trip_sec"], 1),
            "avg_kmh": round(STATE["avg_kmh"], 1),
            "speed_compliance": round(STATE["speed_compliance"], 1),
            "sdlp": round(STATE["sdlp"], 3),
        }
    try:
        tmp = LOG["summary_path"] + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(summary, fh, indent=1)
        os.replace(tmp, LOG["summary_path"])
    except Exception:
        pass


def _rpc_port_open(host, port, timeout=0.5):
    """True if something is accepting TCP on the simulator's RPC port.

    Python's socket calls release the GIL; the carla client's RPC calls do
    not. That difference is the whole point of this probe -- see
    _wait_for_server below.

    シミュレータのRPCポートがTCP接続を受け付けていれば True。

    Python の socket 呼び出しは GIL を解放するが、carla クライアントの RPC
    呼び出しは解放しない。この差こそが、下の _wait_for_server が存在する
    理由である。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            return s.connect_ex((host, port)) == 0
    except OSError:
        return False


def _wait_for_server(host, port):
    """Block until the CARLA RPC port answers, without holding the GIL.

    Every carla client call is a C++ RPC that keeps the GIL for its whole
    duration, timeout included. So a single `client.get_world()` against a
    simulator that is not up freezes *the entire process* for the client
    timeout -- including the Flask thread, and including the main thread
    sitting in `Thread.start()`. The dashboard would then never bind its
    port at all and the browser got "connection refused" rather than a page
    saying "disconnected". Probing with a plain socket first keeps that
    waiting in GIL-releasing code, so the web UI comes up and stays up
    whether or not the simulator is there.

    CARLA の RPC ポートが応答するまで、GIL を保持せずに待機する。

    carla クライアントの呼び出しはすべて C++ の RPC であり、タイムアウトを
    含む実行中ずっと GIL を保持する。そのため、起動していないシミュレータに
    対する `client.get_world()` 1回で、Flask スレッドも `Thread.start()` で
    待機中のメインスレッドも含めた*プロセス全体*がクライアントタイムアウトの
    間フリーズする。結果としてダッシュボードはポートを bind すらできず、
    ブラウザには「切断」表示ではなく接続拒否が返っていた。先に素の socket で
    確認することで待機を GIL 解放側のコードに閉じ込め、シミュレータの有無に
    関わらず Web UI が起動し、動き続ける。
    """
    if _rpc_port_open(host, port):
        return
    with STATE_LOCK:
        STATE["connected"] = False
        STATE["hero_found"] = False
    print("[dashboard] waiting for CARLA at %s:%d "
          "(web UI is already serving)" % (host, port))
    while not _rpc_port_open(host, port):
        time.sleep(1.0)
    print("[dashboard] CARLA RPC port is up")


def telemetry_worker(host, port, log_dir=None):
    # 5 s rather than 10 s: while the simulator is reachable but wedged
    # (a synchronous world with no tick master), this timeout is how long a
    # single RPC holds the GIL and so how long the web UI stops answering.
    # The unreachable case is handled by _wait_for_server, not by this.
    # 10秒ではなく5秒。シミュレータには到達できるが停止している場合
    # （ティックマスタ不在の同期ワールド）、この時間だけ1回の RPC が GIL を
    # 保持し、その間 Web UI が応答しなくなる。到達不能な場合はこの値ではなく
    # _wait_for_server が処理する。
    _wait_for_server(host, port)
    client = carla.Client(host, port)
    client.set_timeout(5.0)
    if log_dir:
        _open_log(log_dir)
    current_hero_id = None
    current_map = ""
    sensors = []
    last_fps_t = time.time()
    frame_count = 0
    sim_fps = 0.0
    carla_map = None
    map_name = ""
    actor_cache = None
    next_actor_scan = 0.0

    # world.get_map() parses the town's OpenDRIVE description. It was called
    # twice per pass of this 10 Hz loop; the map only changes when the town
    # is reloaded, which also invalidates the hero, so fetch it there.
    # world.get_map() は町の OpenDRIVE 定義を解析する。この10Hzループで
    # 毎回2度呼ばれていたが、マップが変わるのは町の再読み込み時だけで、
    # そのとき hero も無効になるため、そちらで取得する。
    while True:
        try:
            world = client.get_world()
            if carla_map is None:
                carla_map = world.get_map()
                map_name = carla_map.name.split("/")[-1]
            hero = find_hero(world)

            # Rough wall-clock fps for the telemetry loop.
            # テレメトリループの実時間ベースの概算FPS。
            frame_count += 1
            tnow = time.time()
            if tnow - last_fps_t >= 1.0:
                sim_fps = frame_count / (tnow - last_fps_t)
                frame_count = 0
                last_fps_t = tnow

            if hero is None:
                with STATE_LOCK:
                    STATE["connected"] = True
                    STATE["hero_found"] = False
                    STATE["map"] = map_name
                for s in sensors:
                    try:
                        s.destroy()
                    except Exception:
                        pass
                sensors = []
                current_hero_id = None
                carla_map = None      # town may be reloading / 町の再読込中かも
                _reset_drv_state()
                time.sleep(0.5)
                continue

            if hero.id != current_hero_id or map_name != current_map:
                actor_cache = None
                carla_map = world.get_map()
                map_name = carla_map.name.split("/")[-1]
                for s in sensors:
                    try:
                        s.destroy()
                    except Exception:
                        pass
                sensors = attach_sensors(world, hero)
                current_hero_id = hero.id
                current_map = map_name
                TRAIL.clear()
                COLL_EVENTS.clear()
                COLL_HITS.clear()
                _reset_drv_state()

            # Turn signals: pick up toggles (wheel script or /signal) and
            # drive the hero's blinker lights accordingly.
            # ウインカー：切替（ホイールまたは /signal）を取り込み、
            # 車両のブリンカーライトに反映する。
            _load_signal()
            if DRV_STATE.get("signal_applied") != SIGNAL["side"]:
                DRV_STATE["signal_applied"] = SIGNAL["side"]
                try:
                    blink = (int(carla.VehicleLightState.LeftBlinker)
                             | int(carla.VehicleLightState.RightBlinker))
                    ls = int(hero.get_light_state()) & ~blink
                    if SIGNAL["side"] == "left":
                        ls |= int(carla.VehicleLightState.LeftBlinker)
                    elif SIGNAL["side"] == "right":
                        ls |= int(carla.VehicleLightState.RightBlinker)
                    hero.set_light_state(carla.VehicleLightState(ls))
                except Exception:
                    pass

            t = hero.get_transform()
            v = hero.get_velocity()
            a_vec = hero.get_acceleration()
            c = hero.get_control()
            speed_ms = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            speed = 3.6 * speed_ms
            TRAIL.append([t.location.x, t.location.y])

            # Body-frame decomposition of accel & velocity. / 加速度と速度を車体座標系に分解。
            (fx, fy), (rx, ry) = _yaw_to_forward_right(t.rotation.yaw)
            lon_accel = a_vec.x * fx + a_vec.y * fy
            lat_accel = a_vec.x * rx + a_vec.y * ry
            vert_accel = a_vec.z
            g_force = math.sqrt(lon_accel ** 2 + lat_accel ** 2) / 9.81

            # Slip angle: angle between velocity vector and heading.
            # スリップ角：速度ベクトルと車体方位のなす角度。
            if speed_ms > 0.5:
                v_heading = math.degrees(math.atan2(v.y, v.x))
                slip = ((v_heading - t.rotation.yaw + 540.0) % 360.0) - 180.0
            else:
                slip = 0.0

            # Jerk. / 加加速度（ジャーク）。
            now = time.time()
            jerk = 0.0
            if DRV_STATE["prev_accel"] is not None and DRV_STATE["prev_t"] is not None:
                dt = max(1e-3, now - DRV_STATE["prev_t"])
                jerk = (lon_accel - DRV_STATE["prev_accel"]) / dt
            DRV_STATE["prev_accel"] = lon_accel
            DRV_STATE["prev_t"] = now

            # Trip distance + time. / 走行距離と走行時間。
            if DRV_STATE["trip_start"] is None:
                DRV_STATE["trip_start"] = now
            if DRV_STATE["last_pos"] is not None:
                lp = DRV_STATE["last_pos"]
                DRV_STATE["trip_m"] += math.hypot(
                    t.location.x - lp[0], t.location.y - lp[1])
            DRV_STATE["last_pos"] = (t.location.x, t.location.y)
            trip_km = DRV_STATE["trip_m"] / 1000.0
            trip_sec = now - DRV_STATE["trip_start"]
            avg_kmh = (trip_km / (trip_sec / 3600.0)) if trip_sec > 1.0 else 0.0

            # Hard brake detection (rising edge at brake > 0.7 & decel > 4 m/s²).
            # 急ブレーキ検出（brake > 0.7 かつ減速度 > 4 m/s² の立ち上がりエッジ）。
            hard_now = (c.brake > 0.7 and lon_accel < -4.0)
            if hard_now and not DRV_STATE["in_hard_brake"]:
                DRV_STATE["hard_brake_count"] += 1
                add_event("hard_brake",
                          "decel %.1f m/s² at %.0f km/h" % (-lon_accel, speed))
            DRV_STATE["in_hard_brake"] = hard_now

            # Curvature from yaw rate and speed. / ヨーレートと速度から曲率を算出。
            yaw_rate = IMU_DATA.get("gz", 0.0)  # rad/s / 単位：rad/s
            curvature = (yaw_rate / speed_ms) if speed_ms > 1.0 else 0.0

            # Speed limit + traffic light. / 制限速度と信号。
            try:
                speed_limit = float(hero.get_speed_limit())
            except Exception:
                speed_limit = 0.0
            try:
                at_tl = bool(hero.is_at_traffic_light())
                tl_state = "None"
                tl_actor = None
                if at_tl:
                    tl_actor = hero.get_traffic_light()
                    if tl_actor is not None:
                        tl_state = str(tl_actor.get_state()).split(".")[-1]
            except Exception:
                at_tl, tl_state, tl_actor = False, "None", None

            # Red-light violation: we were inside a light's trigger volume and
            # left it while the light is still red and the car is moving.
            # 信号無視：信号のトリガー領域内にいて、信号がまだ赤のまま
            # 走行状態で領域を出た場合に違反とみなす。
            if at_tl and tl_actor is not None:
                DRV_STATE["last_red_tl"] = tl_actor
            elif DRV_STATE.get("last_red_tl") is not None:
                try:
                    exit_state = str(
                        DRV_STATE["last_red_tl"].get_state()).split(".")[-1]
                except Exception:
                    exit_state = "None"
                if exit_state == "Red" and speed_ms > 2.0:
                    DRV_STATE["red_light_violations"] += 1
                    add_event("red_light",
                              "ran red light at %.0f km/h" % speed)
                DRV_STATE["last_red_tl"] = None

            # Lane / waypoint info. / 車線・ウェイポイント情報。
            try:
                wp = carla_map.get_waypoint(
                    t.location, project_to_road=True,
                    lane_type=carla.LaneType.Driving)
                wp_off = carla_map.get_waypoint(
                    t.location, project_to_road=False)
                on_road = wp_off is not None
                lane_width = float(wp.lane_width) if wp else 0.0
                lane_id = int(wp.lane_id) if wp else 0
                road_id = int(wp.road_id) if wp else 0
                at_junction = bool(wp.is_junction) if wp else False
                if wp is not None:
                    # Signed lateral offset: project the vector from lane
                    # centre to the vehicle onto the lane's right vector.
                    # 符号付き横方向オフセット：車線中心から車両への
                    # ベクトルを車線の右方向ベクトルに射影する。
                    wl = wp.transform.location
                    rv = wp.transform.get_right_vector()
                    lane_offset = ((t.location.x - wl.x) * rv.x
                                   + (t.location.y - wl.y) * rv.y)
                else:
                    lane_offset = 0.0
            except Exception:
                on_road, lane_width, lane_offset = False, 0.0, 0.0
                lane_id, road_id, at_junction = 0, 0, False

            # --- Driver-performance metrics (HIL research) ---------------
            # SDLP: std dev of signed lane position over the last 60 s,
            # only while driving on-road at a meaningful speed.
            # --- 運転パフォーマンス指標（HIL研究用） ---------------
            # SDLP：直近60秒の符号付き車線内位置の標準偏差。
            # 道路上を十分な速度で走行中のみ算出する。
            if on_road and speed_ms > 2.0:
                DRV_STATE["offset_hist"].append((now, lane_offset))
            recent_off = [o for ts, o in DRV_STATE["offset_hist"]
                          if now - ts <= 60.0]
            if len(recent_off) >= 10:
                mo = sum(recent_off) / len(recent_off)
                sdlp = math.sqrt(
                    sum((o - mo) ** 2 for o in recent_off) / len(recent_off))
            else:
                sdlp = 0.0

            # Steering reversal rate: direction changes of the steering
            # signal larger than a small gap, per minute (rolling 60 s).
            # ステアリング反転率：一定のギャップを超えるステアリング信号の
            # 方向転換の1分あたり回数（直近60秒のローリング集計）。
            st = c.steer
            prev_st = DRV_STATE["steer_prev"]
            if prev_st is not None and abs(st - prev_st) > 1e-4:
                dirn = 1 if st > prev_st else -1
                if DRV_STATE["steer_dir"] == 0:
                    DRV_STATE["steer_dir"] = dirn
                    DRV_STATE["steer_extreme"] = prev_st
                elif dirn != DRV_STATE["steer_dir"]:
                    # ~0.01 of full lock ≈ a few degrees of wheel angle.
                    # フルロックの約0.01 ≈ ハンドル角にして数度。
                    if abs(prev_st - DRV_STATE["steer_extreme"]) >= 0.01:
                        DRV_STATE["reversal_times"].append(now)
                    DRV_STATE["steer_dir"] = dirn
                    DRV_STATE["steer_extreme"] = prev_st
            DRV_STATE["steer_prev"] = st
            srr = float(len([ts for ts in DRV_STATE["reversal_times"]
                             if now - ts <= 60.0]))

            # Jerk RMS (rolling window). / ジャークRMS（ローリング窓）。
            DRV_STATE["jerk_hist"].append(jerk)
            jh = DRV_STATE["jerk_hist"]
            jerk_rms = math.sqrt(sum(j * j for j in jh) / len(jh)) if jh else 0.0

            # Speed compliance: % of moving time at/below the limit (+3 km/h).
            # 速度遵守率：走行時間のうち制限速度（+3 km/h）以下だった割合（%）。
            last_lt = DRV_STATE["last_loop_t"]
            DRV_STATE["last_loop_t"] = now
            if (last_lt is not None and now - last_lt < 1.0
                    and speed > 1.0 and speed_limit > 0):
                DRV_STATE["moving_time"] += now - last_lt
                if speed > speed_limit + 3.0:
                    DRV_STATE["speeding_time"] += now - last_lt
            if DRV_STATE["moving_time"] > 1.0:
                speed_compliance = 100.0 * (
                    1.0 - DRV_STATE["speeding_time"] / DRV_STATE["moving_time"])
            else:
                speed_compliance = 100.0

            # Weather. / 天候。
            try:
                w = world.get_weather()
                weather = {
                    "cloudiness": float(w.cloudiness),
                    "precipitation": float(w.precipitation),
                    "precipitation_deposits": float(w.precipitation_deposits),
                    "wind_intensity": float(w.wind_intensity),
                    "sun_azimuth": float(w.sun_azimuth_angle),
                    "sun_altitude": float(w.sun_altitude_angle),
                    "fog_density": float(getattr(w, "fog_density", 0.0)),
                    "wetness": float(getattr(w, "wetness", 0.0)),
                }
            except Exception:
                weather = dict(STATE["weather"])

            # Nearby vehicles / walkers + front obstacle / TTC.
            # 周辺車両・歩行者・前方障害物・TTC。
            #
            # world.get_actors() is an RPC that rebuilds a Python object for
            # every actor in the town; with 60 vehicles + 15 walkers that was
            # ~750 constructions a second at this loop's 10 Hz. The roster
            # only changes when traffic spawns or despawns, while the actor
            # objects themselves keep reading live poses out of the current
            # world snapshot -- so rescan at 1 Hz and reuse the handles.
            # world.get_actors() は町の全アクター分の Python オブジェクトを
            # 作り直すRPCで、車60台＋歩行者15人ではこの10Hzループで毎秒
            # 約750個を生成していた。名簿が変わるのは交通の出入り時だけで、
            # アクターオブジェクト自体は最新のワールドスナップショットから
            # 姿勢を読むため、走査は1Hzにして参照を使い回す。
            if actor_cache is None or now >= next_actor_scan:
                next_actor_scan = now + 1.0
                all_actors = world.get_actors()
                actor_cache = (
                    all_actors,
                    [a for a in all_actors.filter("vehicle.*")
                     if a.id != hero.id]
                    + list(all_actors.filter("walker.pedestrian.*")))
            all_actors, moving_actors = actor_cache

            nearby_list = []
            for a in moving_actors:
                try:
                    al = a.get_location()
                except RuntimeError:
                    # Despawned since the last scan / 前回の走査以降に消滅
                    continue
                dx, dy = al.x - t.location.x, al.y - t.location.y
                if abs(dx) > 80 or abs(dy) > 80:
                    continue
                d = math.hypot(dx, dy)
                if d > 80.0:
                    continue
                av = a.get_velocity()
                # project into hero body frame for mini-map / ミニマップ用にヒーロー車体座標系へ射影
                bx = dx * fx + dy * fy       # forward / 前方
                by = dx * rx + dy * ry       # right / 右方
                try:
                    ayaw = a.get_transform().rotation.yaw
                except Exception:
                    ayaw = t.rotation.yaw
                rel_yaw = ((ayaw - t.rotation.yaw + 540.0) % 360.0) - 180.0
                nearby_list.append({
                    "fx": round(bx, 2),
                    "rx": round(by, 2),
                    "d": round(d, 1),
                    "yaw": round(rel_yaw, 1),
                    "kmh": round(3.6 * math.hypot(av.x, av.y), 1),
                    "kind": _classify_actor(a),
                })
            nearby_list.sort(key=lambda v: v["d"])

            # All sensors attached to the hero (this script or any other).
            # ヒーロー車に取り付けられた全センサー（本スクリプト・他スクリプト問わず）。
            hero_sensors = []
            try:
                for a in all_actors.filter("sensor.*"):
                    par = a.parent
                    if par is None or par.id != hero.id:
                        continue
                    sl = a.get_transform().location
                    sdx, sdy = sl.x - t.location.x, sl.y - t.location.y
                    hero_sensors.append({
                        "type": a.type_id,
                        "f": round(sdx * fx + sdy * fy, 2),
                        "r": round(sdx * rx + sdy * ry, 2),
                    })
                hero_sensors.sort(key=lambda x: (-x["f"], x["r"], x["type"]))
            except Exception:
                pass

            # Ultrasonic reading is stale after 0.6 s without an event.
            # 超音波の測定値はイベントが0.6秒無ければ無効とする。
            ultra = OBST_DATA["d"] if (time.time() - OBST_DATA["t"]) < 0.6 \
                else -1.0

            try:
                ext = hero.bounding_box.extent
                bb = {"x": round(max(float(ext.x), 0.5), 2),
                      "y": round(max(float(ext.y), 0.3), 2)}
            except Exception:
                bb = dict(STATE["bb"])

            front_d, ttc, _fa = _front_obstacle(hero, moving_actors)
            # Time headway: gap to lead vehicle divided by own speed.
            # 車間時間（THW）：先行車との車間距離を自車速度で割った値。
            thw = (front_d / speed_ms) if (0.0 < front_d < 60.0
                                           and speed_ms > 0.5) else -1.0

            # Stability proxy: penalize high slip, high lat accel, low speed irrelevant.
            # 安定性の代理指標：大きなスリップや横加速度を減点。低速時は実質無関係。
            stability = max(0.0, 1.0 - min(1.0,
                abs(slip) / 20.0 + abs(lat_accel) / 8.0))
            abs_active = c.brake > 0.6 and abs(lat_accel) > 3.0
            tcs_active = c.throttle > 0.7 and speed_ms < 2.0 and lon_accel < 1.0

            # Vehicle physical properties. / 車両の物理特性。
            try:
                phys = hero.get_physics_control()
                mass_kg = float(phys.mass)
                num_wheels = len(phys.wheels)
            except Exception:
                mass_kg, num_wheels = 0.0, 4
            try:
                sim_time = float(world.get_snapshot().timestamp.elapsed_seconds)
            except Exception:
                sim_time = 0.0

            recent = [i for ts, i in COLL_EVENTS if now - ts < 10.0]
            peak = max(recent) if recent else 0.0

            events_out = [
                {"t": time.strftime("%H:%M:%S", time.localtime(ts)),
                 "type": et, "msg": msg}
                for ts, et, msg in list(EVENTS)[-25:]][::-1]

            with STATE_LOCK:
                STATE.update({
                    "connected": True,
                    "hero_found": True,
                    "map": map_name,
                    "speed_kmh": speed,
                    "speed_limit_kmh": speed_limit,
                    "speed_delta_kmh": speed - speed_limit if speed_limit > 0 else 0.0,
                    "throttle": c.throttle,
                    "brake": c.brake,
                    "steer": c.steer,
                    "reverse": c.reverse,
                    "hand_brake": c.hand_brake,
                    "gear": c.gear,
                    "location": {"x": t.location.x,
                                 "y": t.location.y,
                                 "z": t.location.z},
                    "heading_deg": t.rotation.yaw,
                    "compass_deg": IMU_DATA["compass"],
                    "gnss": dict(GNSS_DATA),
                    "imu": {k: IMU_DATA[k] for k in
                            ("ax", "ay", "az", "gx", "gy", "gz")},
                    "lat_accel": lat_accel,
                    "lon_accel": lon_accel,
                    "vert_accel": vert_accel,
                    "yaw_rate_deg": math.degrees(yaw_rate),
                    "jerk": jerk,
                    "g_force": g_force,
                    "slip_angle_deg": slip,
                    "curvature": curvature,
                    "traffic_light": tl_state,
                    "at_traffic_light": at_tl,
                    "front_distance": front_d if front_d < 60.0 else -1.0,
                    "ttc": ttc,
                    "thw": thw,
                    "sdlp": sdlp,
                    "steer_reversal_rate": srr,
                    "lane_invasions": DRV_STATE["lane_invasions"],
                    "red_light_violations": DRV_STATE["red_light_violations"],
                    "speed_compliance": speed_compliance,
                    "jerk_rms": jerk_rms,
                    "events": events_out,
                    "log_file": LOG["path"],
                    "hard_brake_count": DRV_STATE["hard_brake_count"],
                    "collision_count": DRV_STATE["collision_count"],
                    "on_road": on_road,
                    "lane_width": lane_width,
                    "lane_offset": lane_offset,
                    "lane_id": lane_id,
                    "road_id": road_id,
                    "at_junction": at_junction,
                    "trip_km": trip_km,
                    "trip_sec": trip_sec,
                    "avg_kmh": avg_kmh,
                    "collision_peak": peak,
                    "collision_recent": recent[-60:],
                    "collision_hits": [
                        {"age": round(now - h["t"], 1), "f": h["f"],
                         "r": h["r"], "i": round(h["i"]), "o": h["o"]}
                        for h in list(COLL_HITS) if now - h["t"] < 120.0],
                    # Count of what the minimap actually shows: actors
                    # within 80 m. The chip is labelled "Nearby" /
                    # 「周辺車両」, so the town-wide total would be
                    # both useless and near-constant.
                    # ミニマップに実際に表示される範囲（80m以内）の
                    # 台数。チップの表記は "Nearby"／「周辺車両」で
                    # あり、町全体の総数ではほぼ一定で意味を成さない。
                    "nearby_vehicles": len(nearby_list),
                    "nearby_list": nearby_list[:50],
                    "sensors": hero_sensors[:24],
                    "ultrasonic_m": round(ultra, 2),
                    "bb": bb,
                    "turn_signal": SIGNAL["side"],
                    "trail": list(TRAIL),
                    "weather": weather,
                    "abs_active": abs_active,
                    "tcs_active": tcs_active,
                    "stability": stability,
                    "vehicle_type": hero.type_id,
                    "num_wheels": num_wheels,
                    "mass_kg": mass_kg,
                    "sim_fps": sim_fps,
                    "sim_time_sec": sim_time,
                    "timestamp": now,
                })

            if LOG["writer"] is not None:
                try:
                    LOG["writer"].writerow([
                        round(now, 3), round(sim_time, 3),
                        round(t.location.x, 2), round(t.location.y, 2),
                        round(t.location.z, 2),
                        round(speed, 2), round(speed_limit, 1),
                        round(c.throttle, 3), round(c.brake, 3),
                        round(c.steer, 4), c.gear,
                        round(lon_accel, 3), round(lat_accel, 3),
                        round(jerk, 3),
                        round(math.degrees(yaw_rate), 2), round(slip, 2),
                        round(lane_offset, 3), round(lane_width, 2),
                        int(on_road),
                        round(front_d, 2) if front_d < 60.0 else -1,
                        round(ttc, 2), round(thw, 2), round(sdlp, 3), srr,
                        DRV_STATE["hard_brake_count"],
                        DRV_STATE["lane_invasions"],
                        DRV_STATE["red_light_violations"],
                        round(speed_compliance, 1), round(peak, 1),
                        DRV_STATE["collision_count"]])
                    if now - LOG["last_flush"] > 2.0:
                        LOG["fh"].flush()
                        LOG["last_flush"] = now
                except Exception:
                    pass
            _write_summary(now)
        except Exception as exc:
            with STATE_LOCK:
                STATE["connected"] = False
                STATE["hero_found"] = False
            print("[dashboard] telemetry worker error: %s" % exc, file=sys.stderr)
            time.sleep(1.0)
            # If the simulator went away (a GPU crash takes CarlaUE4 with it),
            # retrying straight into get_world() would burn the client timeout
            # with the GIL held, once per second, and the page would go dead
            # instead of showing "disconnected". Wait on the socket instead,
            # and rebuild the client so it reconnects cleanly.
            # シミュレータが消えた場合（GPUクラッシュは CarlaUE4 ごと落とす）、
            # そのまま get_world() を再試行すると毎秒クライアントタイムアウト
            # 分だけ GIL を保持し続け、「切断」表示どころかページ自体が無反応に
            # なる。代わりに socket で待ち、クライアントを作り直して確実に
            # 再接続する。
            if not _rpc_port_open(host, port):
                _wait_for_server(host, port)
                client = carla.Client(host, port)
                client.set_timeout(5.0)
                carla_map = None
                current_hero_id = None
                sensors = []
            continue

        time.sleep(0.1)


app = Flask(__name__)


@app.route("/telemetry")
def telemetry():
    with STATE_LOCK:
        return jsonify(STATE)


# Delivery drones: delivery_sim.py writes its live state to files in this
# folder and reads wind commands appended to commands.jsonl (delivery/live.py).
# 配送ドローン：delivery_sim.py がこのフォルダに状態を書き出し、
# commands.jsonl に追記された風コマンドを読む（delivery/live.py）。
DELIVERY_DIR = delivery_live.DEFAULT_DIR
DELIVERY_STALE_S = 5.0      # no update for this long: the sim isn't running
WIND_LIMITS = {"speed": (0.0, 40.0), "duration": (0.5, 60.0), "value": (0.0, 20.0)}


@app.route("/delivery/state")
def delivery_state():
    s = delivery_live.read_json(os.path.join(DELIVERY_DIR, delivery_live.STATE))
    if not s:
        return jsonify({"available": False, "running": False})
    s["available"] = True
    s["age"] = round(time.time() - s.get("wall_time", 0.0), 1)
    if s["age"] > DELIVERY_STALE_S:
        s["running"] = False
    return jsonify(s)


@app.route("/delivery/map")
def delivery_map():
    m = delivery_live.read_json(os.path.join(DELIVERY_DIR, delivery_live.MAP))
    if not m:
        return jsonify({"error": "no map yet"}), 404
    return jsonify(m)


@app.route("/delivery/pod", methods=["POST"])
def delivery_pod():
    """Queue a LastMile pod command for delivery_sim: the shell view, or the
    showcase (take a pod over, scrub / play its animation, release it)."""
    d = request.get_json(force=True, silent=True) or {}
    cmd = d.get("cmd")
    clean = {"cmd": cmd}
    try:
        if cmd == "shell":
            if d.get("mode") not in ("solid", "cutaway", "xray", "frame"):
                raise ValueError("unknown shell mode")
            clean["mode"] = d["mode"]
        elif cmd == "pod_takeover":
            clean.update(van=int(d["van"]), t=max(float(d.get("t", 0.0)), 0.0),
                         speed=min(max(float(d.get("speed", 1.0)), 0.1), 4.0))
        elif cmd == "pod_time":
            clean["t"] = max(float(d["t"]), 0.0)
        elif cmd == "tooltips":
            clean["on"] = bool(d.get("on"))
        elif cmd == "pod_play":
            clean.update(playing=bool(d.get("playing", True)),
                         speed=min(max(float(d.get("speed", 1.0)), 0.1), 4.0))
        elif cmd != "pod_release":
            raise ValueError("unknown command")
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    delivery_live.send_command(DELIVERY_DIR, clean)
    return jsonify({"ok": True, "sent": clean})


@app.route("/delivery/wind", methods=["POST"])
def delivery_wind():
    """Queue a wind command for delivery_sim: gust, wind, calm or gust_rate."""
    d = request.get_json(force=True, silent=True) or {}
    cmd = d.get("cmd")
    if cmd not in ("gust", "wind", "calm", "gust_rate"):
        return jsonify({"ok": False, "error": "unknown command"}), 400
    clean = {"cmd": cmd}
    try:
        for k, (lo, hi) in WIND_LIMITS.items():
            if d.get(k) is not None:
                clean[k] = min(max(float(d[k]), lo), hi)
        if d.get("dir") is not None:
            clean["dir"] = float(d["dir"]) % 360.0
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "bad number"}), 400
    delivery_live.send_command(DELIVERY_DIR, clean)
    return jsonify({"ok": True, "sent": clean})


# Pedal tuning shared with the G923 control script, which watches this
# file's mtime and applies changes live while driving.
# G923 制御スクリプトと共有するペダル調整設定。同スクリプトがこの
# ファイルの更新時刻を監視し、運転中に変更をライブ適用する。
TUNING_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pedal_tuning.json")
TUNING_DEFAULTS = {"throttle_max": 0.8, "throttle_curve": 1.8,
                   "brake_max": 1.0, "brake_curve": 2.2}
TUNING_LIMITS = {"throttle_max": (0.1, 1.0), "throttle_curve": (0.5, 3.0),
                 "brake_max": (0.1, 1.0), "brake_curve": (0.5, 3.0)}


@app.route("/tuning", methods=["GET", "POST"])
def tuning():
    data = dict(TUNING_DEFAULTS)
    try:
        with open(TUNING_PATH) as fh:
            saved = json.load(fh)
        data.update({k: float(saved[k]) for k in TUNING_DEFAULTS
                     if k in saved})
    except Exception:
        pass
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        for k, (lo, hi) in TUNING_LIMITS.items():
            if k in body:
                try:
                    data[k] = max(lo, min(hi, float(body[k])))
                except (TypeError, ValueError):
                    pass
        try:
            tmp = TUNING_PATH + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(data, fh, indent=1)
            os.replace(tmp, TUNING_PATH)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    return jsonify(data)


# Force-feedback settings shared with carla_ffb.py, which re-reads
# ffb_tuning.json twice a second (schema and defaults in ffb_settings.py).
# carla_ffb.py と共有する FFB 設定（ffb_tuning.json を毎秒 2 回読み直す）。
@app.route("/ffb", methods=["GET", "POST"])
def ffb():
    values = ffb_settings.load()
    error = None
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        if body.get("preset"):
            chosen = ffb_settings.preset(str(body["preset"]))
            if chosen is not None:
                values = chosen
        for k, v in (body.get("values") or {}).items():
            if k in values:
                try:
                    values[k] = ffb_settings.clamp_value(k, v)
                except (TypeError, ValueError):
                    pass
        try:
            values = ffb_settings.save(values)
            if body.get("save_preset") is not None:
                try:
                    ffb_settings.save_preset(body["save_preset"], values)
                except ValueError as exc:
                    error = str(exc)
            if body.get("delete_preset"):
                ffb_settings.delete_preset(str(body["delete_preset"]))
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    status = ffb_settings.read_status()
    if status is not None:
        status["age"] = round(time.time() - status.get("wall_time", 0), 1)
    return jsonify({
        "values": values, "defaults": ffb_settings.DEFAULTS,
        "params": [{"key": p[0], "min": p[2], "max": p[3], "step": p[4], "group": p[5],
                    "en": p[6], "ja": p[7], "help": p[8]} for p in ffb_settings.PARAMS],
        "groups": [{"key": g[0], "en": g[1], "ja": g[2]} for g in ffb_settings.GROUPS],
        "presets": list(ffb_settings.PRESETS),
        "user_presets": sorted(ffb_settings.user_presets()), "error": error,
        "active": next((n for n in list(ffb_settings.PRESETS) + sorted(ffb_settings.user_presets())
                        if ffb_settings.preset(n) == values), None),
        "status": status})


# Vehicle performance (brakes, power, top speed) shared with carla_ffb.py; the
# thread below applies vehicle_tuning.json to the cars.
# 車両性能の設定。下のスレッドが vehicle_tuning.json を各車に適用する。
@app.route("/vehicle_tuning", methods=["GET", "POST"])
def vehicle_tuning_route():
    values = vehicle_tuning.load()
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        if body.get("reset"):
            values = dict(vehicle_tuning.DEFAULTS)
        if body.get("neutral"):
            values.update(vehicle_tuning.NEUTRAL)
        for k, v in (body.get("values") or {}).items():
            if k in values:
                try:
                    values[k] = vehicle_tuning.clamp_value(k, v)
                except (TypeError, ValueError):
                    pass
        try:
            values = vehicle_tuning.save(values)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
    return jsonify({
        "values": values, "defaults": vehicle_tuning.DEFAULTS,
        "params": [{"key": p[0], "min": p[2], "max": p[3], "step": p[4], "group": p[5],
                    "en": p[6], "ja": p[7], "help": p[8]} for p in vehicle_tuning.PARAMS],
        "groups": [{"key": g[0], "en": g[1], "ja": g[2]} for g in vehicle_tuning.GROUPS],
        "status": vehicle_tuning.read_status()})


def vehicle_tuning_worker(host, port):
    """Keep every car's physics in line with vehicle_tuning.json.

    Applies when the settings change (once they've been still for 0.8 s, so
    dragging a slider doesn't re-apply every step) and to each new car.
    Pods in a scripted move (physics off) wait until they drive again. The
    hero's tyre grip comes from the FFB settings while carla_ffb.py runs.
    A car's state is unknown when first seen (it may carry an older tuning
    from before a dashboard restart), so every car gets one apply then.
    設定変更時（0.8 秒変化が無くなってから）と新しい車に適用する。物理オフの
    ポッドは走行再開まで待つ。自車のタイヤグリップは FFB 設定に従う。
    """
    applied = {}            # actor id -> what it has / 適用済みの内容
    shape = {}              # type id -> physics control (for gears, wheels)
    client = world = None
    mtime, changed_at = None, 0.0
    while True:
        try:
            if world is None:
                client = carla.Client(host, port)
                client.set_timeout(5.0)
                world = client.get_world()
                applied.clear()
            world.wait_for_tick(5.0)
            try:
                m = os.path.getmtime(vehicle_tuning.TUNING_PATH)
            except OSError:
                m = None
            if m != mtime:
                mtime, changed_at = m, time.time()
            if time.time() - changed_at < 0.8:
                time.sleep(0.2)
                continue
            values = vehicle_tuning.load()
            ffb = ffb_settings.read_status()
            ffb_grip = (ffb_settings.load()["tire_friction"]
                        if ffb and time.time() - ffb.get("wall_time", 0) < 5 else None)
            state = delivery_live.read_json(
                os.path.join(DELIVERY_DIR, delivery_live.STATE)) or {}
            vans = {v.get("actor"): v for v in state.get("vans", []) if "actor" in v}
            seen, cars = set(), []
            for a in world.get_actors().filter("vehicle.*"):
                if a.attributes.get("role_name") == vehicle_tuning.PROBE_ROLE:
                    continue
                seen.add(a.id)
                is_hero = a.attributes.get("role_name") == "hero"
                sc = vehicle_tuning.scales(values, is_hero)
                grip = ffb_grip if is_hero else None
                want = (tuple(sorted(sc.items())), grip)
                van = vans.get(a.id)
                waiting = a.attributes.get("role_name") == "delivery" and \
                    (van is None or van.get("kinematic"))
                fac = vehicle_tuning.factory(world, a.type_id)
                have = applied.get(a.id)        # None: unknown / 未知
                if want != have and not waiting:
                    vehicle_tuning.apply(client, a, sc, grip)
                    applied[a.id] = have = want
                now = dict(have[0]) if have else vehicle_tuning.NEUTRAL
                if a.type_id not in shape:      # gears, wheel size: never tuned / 不変
                    shape[a.type_id] = a.get_physics_control()
                brake0, torque0 = fac["brake"][0], max(y for _, y in fac["torque_curve"])
                top0 = vehicle_tuning.top_speed_kmh(shape[a.type_id], fac["max_rpm"])
                cars.append({
                    "id": a.id, "type": a.type_id, "hero": is_hero,
                    "applied": have == want, "waiting": waiting and want != have,
                    "brake_nm": round(brake0 * now["brake_scale"]), "brake_base": round(brake0),
                    "torque_nm": round(torque0 * now["power_scale"]), "torque_base": round(torque0),
                    "top_kmh": round(top0 * now["top_speed_scale"]), "top_base": round(top0),
                    "grip": grip})
            for gone in set(applied) - seen:
                del applied[gone]
            cars.sort(key=lambda c: (not c["hero"], c["type"], c["id"]))
            vehicle_tuning.write_status({"wall_time": time.time(), "cars": cars})
            time.sleep(0.5)
        except Exception as exc:        # server gone / RPC timeout: reconnect
            print("[dashboard] vehicle tuning: %s" % exc)
            world = None
            time.sleep(3.0)


def _shutdown_worker():
    """Stop the whole simulation stack and free its ports.

    First kill the manual-control window: when launched via run_pipeline.py
    its supervisor then terminates every child cleanly (traffic, viewers,
    CarlaUE4, and this dashboard) and all ports are released. If we are
    still alive after that grace period (standalone use), kill CARLA
    directly and exit this process too.

    シミュレーション一式を停止しポートを解放する。まず運転ウィンドウを
    終了させる：run_pipeline.py 経由なら監視プロセスが全ての子（交通・
    ビューア・CarlaUE4・本ダッシュボード）をクリーンに終了させ、ポートも
    解放される。猶予時間を過ぎても生きていれば（単体起動の場合）、CARLA を
    直接終了させ、本プロセスも終了する。
    """
    time.sleep(0.3)   # let the HTTP response flush / レスポンス送信を待つ
    for pat in ("manual_control_steeringwheel", "sensor_viewer.py",
                "generate_traffic.py", "agent_drive.py"):
        try:
            subprocess.call(["pkill", "-f", pat])
        except Exception:
            pass
    # Grace period for run_pipeline's terminate_all (it will kill us too).
    # run_pipeline の terminate_all の猶予（本プロセスも終了させてくれる）。
    time.sleep(6.0)
    # Still alive -> standalone use: kill CARLA ourselves. UE4 often
    # ignores SIGTERM, so escalate to SIGKILL and verify port 2000 is
    # actually released.
    # まだ生きている＝単体起動：CARLA を自ら終了させる。UE4 は SIGTERM を
    # 無視しがちなので SIGKILL へ段階的に強め、ポート2000の解放を確認する。
    for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
        try:
            subprocess.call(["pkill", "-f", pat])
        except Exception:
            pass
    time.sleep(3.0)
    for pat in ("CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
        try:
            subprocess.call(["pkill", "-9", "-f", pat])
        except Exception:
            pass
    for _ in range(10):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", 2000)) != 0:
                print("[dashboard] port 2000 released")
                break
        time.sleep(0.5)
    print("[dashboard] shutdown requested from the web UI - exiting")
    os._exit(0)


@app.route("/shutdown", methods=["POST"])
def shutdown():
    threading.Thread(target=_shutdown_worker, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/signal", methods=["GET", "POST"])
def signal():
    """Toggle / read the turn signal. POST {"side": "left"|"right"} toggles
    that side (same side again switches it off).

    ウインカーの切替・取得。POST {"side": "left"|"right"} でその側を
    トグルする（同じ側をもう一度送ると消灯）。
    """
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        side = body.get("side")
        if side in ("left", "right"):
            _save_signal("off" if SIGNAL["side"] == side else side)
        elif side == "off":
            _save_signal("off")
    return jsonify({"side": SIGNAL["side"]})


@app.route("/sessions")
def sessions():
    """List saved sessions: [{date, time, ...summary}, ...] newest first.

    保存済みセッションの一覧を返す：[{date, time, ...summary}, ...]（新しい順）。
    """
    root = LOG.get("root") or "logs"
    out = []
    try:
        for day in sorted(os.listdir(root), reverse=True):
            day_dir = os.path.join(root, day)
            if not os.path.isdir(day_dir):
                continue
            for tod in sorted(os.listdir(day_dir), reverse=True):
                sess_dir = os.path.join(day_dir, tod)
                spath = os.path.join(sess_dir, "summary.json")
                if not os.path.isfile(spath):
                    continue
                try:
                    with open(spath) as fh:
                        s = json.load(fh)
                except Exception:
                    continue
                s["date"] = s.get("date", day)
                s["time"] = s.get("time", tod)
                s["current"] = (sess_dir == LOG.get("dir"))
                out.append(s)
    except OSError:
        pass
    return jsonify(out[:200])


@app.route("/download")
def download():
    from flask import send_file
    if not LOG["path"]:
        return Response("no session log active (run without --no-log)",
                        status=404, mimetype="text/plain")
    try:
        LOG["fh"].flush()
    except Exception:
        pass
    return send_file(os.path.abspath(LOG["path"]), as_attachment=True)


@app.route("/")
def index():
    return Response(HTML, mimetype="text/html")


HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CARLA HIL Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Fira+Code:wght@400;500;600;700&family=Fira+Sans:wght@300;400;500;600;700&display=swap');
  :root {
    --bg:#090c12; --panel:#141a24; --panel2:#0c1119; --line:#222c3a;
    --ink:#e8eef6; --sec:#9daabd; --mute:#818da2;
    --blue:#3987e5; --green:#199e70; --yellow:#c98500; --red:#e66767;
    --violet:#9085e9; --good:#0ca30c; --warn:#fab219; --serious:#ec835a;
    --crit:#d03b3b; --radius:14px; --accent:var(--blue);
    --font:'Fira Sans', system-ui, -apple-system, "Segoe UI", "Noto Sans JP",
           "Hiragino Sans", "Yu Gothic UI", sans-serif;
    --mono:'Fira Code', ui-monospace, "SF Mono", "Cascadia Code", Menlo,
           Consolas, monospace;
  }
  * { box-sizing:border-box; }
  body { margin:0; color:var(--ink); font-size:14px; line-height:1.5;
         font-family:var(--font); -webkit-font-smoothing:antialiased;
         background:
           radial-gradient(1100px 560px at 80% -12%, #17324e33, transparent 60%),
           radial-gradient(820px 460px at -5% -5%, #1a2a3e2e, transparent 55%),
           var(--bg);
         background-attachment:fixed; }
  header { position:sticky; top:0; z-index:20; display:flex;
           align-items:center; gap:14px; flex-wrap:wrap; padding:11px 18px;
           background:rgba(9,12,18,.82); backdrop-filter:blur(10px);
           border-bottom:1px solid var(--line);
           box-shadow:0 6px 24px rgba(0,0,0,.35); }
  header h1 { margin:0; font-family:var(--mono); font-size:14px; font-weight:700;
              letter-spacing:1.6px; text-transform:uppercase; }
  header h1 span { color:var(--blue); text-shadow:0 0 12px #3987e588; }
  .dot { width:9px; height:9px; border-radius:50%; background:var(--good);
         display:inline-block; margin-right:6px; box-shadow:0 0 6px var(--good); }
  .dot.bad { background:var(--crit); box-shadow:0 0 6px var(--crit); }
  #status { color:var(--sec); font-size:12px; }
  nav.tabs { display:flex; background:var(--panel); border:1px solid var(--line);
             border-radius:9px; padding:3px; gap:2px; }
  nav.tabs button { border:0; background:none; color:var(--sec); cursor:pointer;
                    font:inherit; font-size:12.5px; font-weight:600;
                    padding:5px 14px; border-radius:7px; }
  nav.tabs button:hover { color:var(--ink); }
  nav.tabs button.active { background:var(--blue); color:#fff;
                           box-shadow:0 0 14px #3987e555; }
  .push { margin-left:auto; display:flex; gap:10px; align-items:center; }
  .lang { display:flex; border:1px solid var(--line); border-radius:8px;
          overflow:hidden; }
  .lang button { border:0; background:var(--panel); color:var(--sec);
                 font:inherit; font-size:12px; padding:5px 10px; cursor:pointer; }
  .lang button.active { background:#1f6feb33; color:var(--ink); font-weight:700; }
  a.dl { color:var(--ink); font-size:12px; text-decoration:none;
         border:1px solid var(--line); background:var(--panel);
         padding:5px 12px; border-radius:8px; }
  a.dl:hover { border-color:var(--blue); }
  button.dl { font:inherit; font-size:12px; cursor:pointer; }
  .dl.shut { color:#ff9c9c; border-color:#d03b3b55; }
  .dl.shut:hover { border-color:var(--crit); background:#d03b3b1f; }
  #shutmask { position:fixed; inset:0; z-index:100; display:flex;
              align-items:center; justify-content:center; text-align:center;
              background:rgba(9,12,18,.93); backdrop-filter:blur(6px);
              color:var(--ink); font-size:17px; line-height:1.8;
              padding:24px; }
  .chips { display:flex; gap:8px; flex-wrap:wrap; padding:12px 18px 0; }
  .chip { background:var(--panel); border:1px solid var(--line);
          border-radius:999px; padding:3px 12px; font-size:12px;
          color:var(--mute); }
  .chip b { color:var(--ink); font-weight:600; margin-left:5px;
            font-variant-numeric:tabular-nums; }
  .grid { display:grid; grid-template-columns:repeat(4,1fr); gap:12px;
          padding:14px 18px 26px; }
  @media (max-width:1500px){ .grid{grid-template-columns:repeat(3,1fr);} }
  @media (max-width:1050px){ .grid{grid-template-columns:repeat(2,1fr);} }
  @media (max-width:660px) { .grid{grid-template-columns:1fr;} }
  .card { background:linear-gradient(180deg, #ffffff05, transparent 42%),
                     var(--panel);
          border:1px solid var(--line); border-radius:var(--radius);
          padding:14px 16px; min-width:0; position:relative;
          box-shadow:0 1px 2px rgba(0,0,0,.4), inset 0 1px 0 #ffffff08;
          transition:border-color .18s, box-shadow .18s; }
  .card:hover { border-color:#324356;
                box-shadow:0 8px 24px rgba(0,0,0,.45), inset 0 1px 0 #ffffff0d; }
  .card.wide { grid-column:span 2; }
  @media (max-width:660px){ .card.wide{grid-column:span 1;} }
  .card h3 { margin:0 0 12px; font-family:var(--mono); font-size:12px;
             font-weight:600; text-transform:uppercase; letter-spacing:1.6px;
             color:var(--sec); display:flex; justify-content:space-between;
             align-items:center; gap:8px; }
  .big { font-family:var(--mono); font-size:48px; font-weight:600;
         text-align:center; line-height:1.02; font-variant-numeric:tabular-nums;
         letter-spacing:-1px; text-shadow:0 0 22px #3987e533; }
  .mid { font-family:var(--mono); font-size:22px; font-weight:600;
         font-variant-numeric:tabular-nums; }
  .kv .v, .inlbl b, .chip b, .tile .tv, .sub b, .legend b,
  table.sess td.num, #tooltip, #tooltip .tv { font-family:var(--mono); }
  .unit { font-size:12px; color:var(--mute); margin-left:4px; font-weight:500;
          letter-spacing:0; }
  .sub { text-align:center; color:var(--mute); font-size:12px; margin-top:4px; }
  .sub b { color:var(--sec); font-variant-numeric:tabular-nums; }
  .kv { display:flex; justify-content:space-between; padding:3px 0;
        font-size:12.5px; color:var(--sec); }
  .kv .v { font-variant-numeric:tabular-nums; color:var(--ink); }
  .bar { background:var(--panel2); border-radius:3px; height:7px;
         overflow:hidden; margin:5px 0 10px; position:relative;
         box-shadow:inset 0 0 0 1px #ffffff08; }
  .bar > i { display:block; height:100%; border-radius:3px;
             background:linear-gradient(90deg, #147a57, var(--green));
             box-shadow:0 0 8px #199e7066; transition:width .12s; }
  .bar.red > i { background:linear-gradient(90deg, #b34a4a, var(--red));
                 box-shadow:0 0 8px #e6676766; }
  .bar.steer > i { background:var(--yellow); position:absolute; top:0; bottom:0; }
  .inlbl { display:flex; justify-content:space-between; font-size:12.5px;
           color:var(--sec); }
  .inlbl b { color:var(--ink); font-variant-numeric:tabular-nums; }
  input[type=range] { width:100%; margin:6px 0 10px; accent-color:var(--blue);
                      height:4px; cursor:pointer; }
  .rbtn { border:1px solid var(--line); background:var(--panel2);
          color:var(--sec); font:inherit; font-size:12px; font-weight:600;
          border-radius:6px; padding:4px 10px; min-height:26px; cursor:pointer;
          text-transform:none; letter-spacing:.3px; }
  .rbtn:hover { border-color:var(--blue); color:var(--ink); }
  .chartwrap { position:relative; }
  canvas { background:var(--panel2); border-radius:8px; display:block;
           width:100%; height:140px; }
  canvas.tall { height:200px; }
  .xline { position:absolute; top:0; bottom:0; width:1px; display:none;
           background:rgba(230,237,243,.28); pointer-events:none; }
  #alertbar { position:fixed; top:66px; left:50%; z-index:40;
              transform:translateX(-50%) translateY(-16px);
              padding:10px 26px; border-radius:10px; font-size:15px;
              font-weight:700; letter-spacing:.4px; opacity:0;
              pointer-events:none; transition:opacity .15s, transform .15s;
              box-shadow:0 8px 24px rgba(0,0,0,.5); }
  #alertbar.show { opacity:1; transform:translateX(-50%) translateY(0); }
  #alertbar.warn { background:#4a3d1a; color:#ffd77a;
                   border:1px solid rgba(250,178,25,.4); }
  #alertbar.danger::before, #alertbar.warn::before {
    content:""; display:inline-block; width:18px; height:18px; margin-right:8px;
    vertical-align:-3px; background:currentColor;
    -webkit-mask:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='black' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpath d='M12 3 2 20h20L12 3z'/%3E%3Cpath d='M12 10v4M12 17.5v.5'/%3E%3C/svg%3E") center/contain no-repeat;
            mask:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='black' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpath d='M12 3 2 20h20L12 3z'/%3E%3Cpath d='M12 10v4M12 17.5v.5'/%3E%3C/svg%3E") center/contain no-repeat; }
  #alertbar.danger { background:#5c1a1a; color:#ffb3b3;
                     border:1px solid rgba(208,59,59,.6); }
  #tooltip { position:fixed; z-index:50; display:none; pointer-events:none;
             background:#1c2330; border:1px solid var(--line);
             border-radius:8px; padding:8px 10px; font-size:12px;
             box-shadow:0 6px 18px rgba(0,0,0,.45);
             font-variant-numeric:tabular-nums; max-width:260px; }
  #tooltip .tr { display:flex; align-items:center; gap:6px; padding:1px 0;
                 color:var(--sec); }
  #tooltip i { width:8px; height:8px; border-radius:2px; display:inline-block;
               flex:none; }
  #tooltip .tv { margin-left:auto; padding-left:12px; color:var(--ink); }
  #tooltip .th { color:var(--ink); font-weight:700; padding-bottom:2px; }
  .legend { font-size:12px; color:var(--mute); margin-top:7px; display:flex;
            flex-wrap:wrap; gap:4px 14px; }
  .legend span.it { display:inline-flex; align-items:center; gap:5px; }
  .legend i { width:9px; height:9px; border-radius:2px; display:inline-block; }
  .legend b { color:var(--sec); font-weight:600;
              font-variant-numeric:tabular-nums; }
  .ind { display:inline-block; padding:2px 8px; border-radius:6px;
         background:var(--panel2); margin:2px 3px 0 0; font-size:12px;
         font-weight:700; letter-spacing:1px; color:var(--mute); }
  .ind.on { background:var(--crit); color:#fff; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px;
          font-size:12px; font-weight:700; background:var(--panel2);
          color:var(--sec); letter-spacing:.5px; }
  .pill.red { background:#d03b3b26; color:#ff9c9c; }
  .pill.green { background:#0ca30c26; color:#5fd45f; }
  .pill.yellow { background:#fab21926; color:#fac95e; }
  .events { font-size:12px; max-height:190px; overflow-y:auto; }
  .events .ev { padding:3px 4px; border-bottom:1px solid var(--panel2);
                display:flex; gap:8px; align-items:baseline; }
  .events .ev .t { color:var(--mute); flex:none;
                   font-variant-numeric:tabular-nums; }
  .events .ev .ty { flex:none; min-width:84px; font-weight:700; }
  .events .ev .m { color:var(--sec); }
  .ev-collision .ty { color:var(--crit); }
  .ev-red_light .ty { color:var(--serious); }
  .ev-hard_brake .ty { color:var(--warn); }
  .ev-lane .ty { color:var(--blue); }
  .sect { padding:16px 18px 0; font-size:12px; font-weight:700;
          letter-spacing:1.2px; text-transform:uppercase; color:var(--mute); }
  .tiles { display:grid; grid-template-columns:repeat(4,1fr); gap:12px;
           padding:12px 18px 0; }
  @media (max-width:1050px){ .tiles{grid-template-columns:repeat(2,1fr);} }
  .tile { background:var(--panel); border:1px solid var(--line);
          border-radius:var(--radius); padding:16px 18px; position:relative;
          overflow:hidden; }
  .tile::before { content:""; position:absolute; left:0; top:0; bottom:0;
                  width:3px; background:var(--ac,var(--blue)); }
  .tile .tl { font-size:12px; font-weight:700; letter-spacing:1.1px;
              text-transform:uppercase; color:var(--mute); }
  .tile .tv { font-size:38px; font-weight:600; margin-top:6px; line-height:1;
              font-variant-numeric:tabular-nums; letter-spacing:-1px;
              text-shadow:0 0 20px #3987e522; }
  .tile .tu { font-size:13px; color:var(--mute); font-weight:500;
              margin-left:4px; letter-spacing:0; }
  .statwrap { padding:12px 18px 26px; display:grid; gap:12px; }
  table.sess { width:100%; border-collapse:collapse; font-size:12.5px; }
  table.sess th { text-align:left; color:var(--mute); font-size:12px;
                  text-transform:uppercase; letter-spacing:1px; padding:6px 10px;
                  border-bottom:1px solid var(--line); white-space:nowrap; }
  table.sess td { padding:6px 10px; border-bottom:1px solid var(--panel2);
                  color:var(--sec); font-variant-numeric:tabular-nums;
                  white-space:nowrap; }
  table.sess td.num { color:var(--ink); }
  table.sess tr.day td { background:var(--panel2); color:var(--ink);
                         font-weight:700; }
  table.sess tr.live td { background:#1f6feb14; }
  .badge { display:inline-block; padding:1px 8px; border-radius:999px;
           background:#0ca30c26; color:#5fd45f; font-size:12px;
           font-weight:700; letter-spacing:.6px; }
  .warnnum { color:var(--serious) !important; font-weight:700; }
  .tblwrap { overflow-x:auto; }
  /* ------------------------------------------------ vehicle view */
  .vehwrap { display:grid; grid-template-columns:300px minmax(0,1fr) 300px;
             gap:12px; padding:14px 18px 26px; align-items:start; }
  @media (max-width:1150px){
    .vehwrap { grid-template-columns:1fr 1fr; }
    .vstage-card { grid-column:1 / -1; order:-1; }
  }
  @media (max-width:660px){ .vehwrap { grid-template-columns:1fr; } }
  .vcol { display:grid; gap:12px; }
  .vpill { display:inline-block; font-family:var(--mono); font-size:13px;
           font-weight:700; letter-spacing:2px; padding:7px 22px;
           border-radius:999px; border:1px solid var(--line);
           transition:background .25s, color .25s, box-shadow .25s; }
  .vpill.ok { background:#0ca30c1c; color:#5fd45f; border-color:#0ca30c55;
              box-shadow:0 0 18px #0ca30c33; }
  .vpill.warn { background:#fab2191c; color:#fac95e; border-color:#fab21955;
                box-shadow:0 0 18px #fab21933; }
  .vpill.crash { background:#d03b3b26; color:#ff9c9c; border-color:#d03b3b88;
                 box-shadow:0 0 26px #d03b3b66;
                 animation:vpulse 1s ease-in-out infinite; }
  @keyframes vpulse { 50% { box-shadow:0 0 40px #d03b3baa; } }
  .vstage { position:relative; }
  #vsvg { width:100%; max-width:560px; max-height:calc(100vh - 210px);
          min-height:420px; display:block; margin:0 auto; }
  .rings circle { fill:none; stroke:#3987e5; stroke-opacity:.10;
                  stroke-width:1; }
  .rings circle.r2 { stroke-dasharray:3 6; }
  #vcar { transition:filter .4s; }
  #vcar.ok { filter:drop-shadow(0 0 16px rgba(57,135,229,.40)); }
  #vcar.warn { filter:drop-shadow(0 0 20px rgba(250,178,25,.50)); }
  #vcar.crash { filter:drop-shadow(0 0 26px rgba(208,59,59,.80));
                animation:vshake .35s ease-in-out; }
  @keyframes vshake { 25%{transform:translate(2px,-1px)}
                      50%{transform:translate(-2px,1px)}
                      75%{transform:translate(1px,-2px)} }
  .taillight { fill:var(--red); opacity:.12; transition:opacity .1s;
               filter:drop-shadow(0 0 6px #e66767); }
  .headlight { fill:#cfe6ff; opacity:.35; filter:drop-shadow(0 0 5px #9ecbff); }
  .revlight { fill:#e8eef6; opacity:0; transition:opacity .15s;
              filter:drop-shadow(0 0 6px #ffffff); }
  .smark { transform-box:fill-box; transform-origin:center; }
  .smark .core { stroke:#090c12; stroke-width:1.5; }
  .smark .halo { fill:none; stroke-width:1.5; opacity:.8;
                 transform-box:fill-box; transform-origin:center;
                 animation:shalo 2.4s ease-out infinite; }
  @keyframes shalo { 0%{transform:scale(1); opacity:.8}
                     75%{transform:scale(3.2); opacity:0}
                     100%{transform:scale(3.2); opacity:0} }
  .slabel { font-family:var(--mono); font-size:12px; fill:var(--sec);
            letter-spacing:.5px; }
  .sleader { stroke:#33445c; stroke-width:1; stroke-dasharray:2 3; }
  /* park-distance (PDC) margin bands, modern-EV style
     モダンEV風のパーキング距離（PDC）マージンバンド */
  .pdc { fill:none; stroke-width:5; stroke-linecap:round; opacity:.07;
         transition:opacity .2s; }
  .pdc.lit { opacity:.95; filter:drop-shadow(0 0 7px currentColor); }
  .mtext { font-family:var(--mono); font-size:12.5px; font-weight:600;
           fill:var(--ink); text-anchor:middle; paint-order:stroke;
           stroke:#090c12; stroke-width:3px;
           font-variant-numeric:tabular-nums; }
  /* click-to-locate sensors / クリックでセンサー位置を表示 */
  #sensor-rows .srow { cursor:pointer; border-radius:7px;
                       padding:6px 6px; }
  #sensor-rows .srow:hover { background:#ffffff08; }
  #sensor-rows .srow.sel { background:#1f6feb1f; }
  #vsens.dim .smark { opacity:.25; }
  #vsens .smark.sel { opacity:1; }
  .slocate { fill:none; stroke:#fff; stroke-opacity:.9; stroke-width:1.5;
             transform-box:fill-box; transform-origin:center;
             animation:shalo 1.4s ease-out infinite; }
  .sfocus { font-family:var(--mono); font-size:12px; font-weight:600;
            fill:#fff; text-anchor:middle; letter-spacing:.6px;
            paint-order:stroke; stroke:#090c12; stroke-width:3.5px; }
  /* turn signals / ウインカー */
  .sigbtn { border:1px solid var(--line); background:var(--panel2);
            color:var(--mute); font-size:15px; line-height:1;
            padding:8px 26px; border-radius:9px; cursor:pointer; }
  .sigbtn:hover { border-color:#31d15888; color:var(--ink); }
  .sigbtn.on { color:#31d158; border-color:#31d15888;
               text-shadow:0 0 10px #31d158aa;
               animation:sigblink .9s steps(1) infinite; }
  @keyframes sigblink { 50% { color:#1b3a24; text-shadow:none; } }
  #vsig-l rect, #vsig-r rect { fill:#ffb020; opacity:0; }
  #vsig-l.on rect, #vsig-r.on rect {
    animation:lampblink .9s steps(1) infinite;
    filter:drop-shadow(0 0 6px #ffb020); }
  @keyframes lampblink { 0%,49% { opacity:.95 } 50%,100% { opacity:.06 } }
  .sigchev { fill:none; stroke:#31d158; stroke-width:5; stroke-linecap:round;
             stroke-linejoin:round; opacity:0;
             filter:drop-shadow(0 0 6px #31d15877); }
  .on > .sigchev { animation:chev 1s ease-out infinite; }
  .on > .sigchev:nth-child(2) { animation-delay:.15s; }
  .on > .sigchev:nth-child(3) { animation-delay:.3s; }
  @keyframes chev { 0%,60%,100% { opacity:0 } 15%,45% { opacity:.95 } }
  /* own-lane guide / 自車線ガイド */
  .laneline { stroke:#8b98a9; stroke-opacity:.4; stroke-width:2;
              stroke-dasharray:9 9; }
  .ev-lane_change .ty { color:var(--green); }
  .rlabel { font-family:var(--mono); font-size:12px; fill:#69758a;
            opacity:.9; }
  .nearg { transition:opacity .3s; }
  .nbody { stroke:#090c12; stroke-width:1; }
  .ndist { font-family:var(--mono); font-size:12px; fill:var(--ink);
           text-anchor:middle; paint-order:stroke; stroke:#090c12;
           stroke-width:3px; font-variant-numeric:tabular-nums; }
  #vultra { fill:none; stroke-width:3.5; stroke-linecap:round;
            filter:drop-shadow(0 0 6px currentColor); }
  .streak { stroke:#3987e5; stroke-width:2; stroke-linecap:round;
            stroke-dasharray:16 70;
            animation:vstreak linear infinite;
            animation-duration:var(--sd,1.2s); }
  #vstreaks { transition:opacity .4s; }
  @keyframes vstreak { to { stroke-dashoffset:-86; } }
  .hitcore { fill:var(--crit); stroke:#ffb3b3; stroke-width:1.5;
             filter:drop-shadow(0 0 8px #d03b3b); }
  .hitring { fill:none; stroke:var(--crit); stroke-width:2;
             transform-box:fill-box; transform-origin:center;
             animation:hitring 1.6s ease-out infinite; }
  @keyframes hitring { 0%{transform:scale(1); opacity:.9}
                       100%{transform:scale(4); opacity:0} }
  .hitlabel { font-family:var(--mono); font-size:12px; fill:#ff9c9c; }
  .srow { display:flex; align-items:center; gap:9px; padding:6px 0;
          border-bottom:1px solid var(--panel2); font-size:12.5px;
          color:var(--sec); }
  .srow:last-child { border-bottom:0; }
  .sdot { width:8px; height:8px; border-radius:50%; flex:none;
          background:var(--c,#69758a); box-shadow:0 0 8px var(--c,#0000); }
  .srow .sval { margin-left:auto; font-family:var(--mono); color:var(--ink);
                font-variant-numeric:tabular-nums; font-size:12px;
                text-align:right; }
  #vh-hits { overflow-x:hidden; }
  #vh-hits .ev { flex-wrap:wrap; }
  #vh-hits .ev .m { word-break:break-all; min-width:0; }
  .vzone { text-align:center; padding:6px 0 2px; }
  .vzone .zn { font-family:var(--mono); font-size:26px; font-weight:600;
               color:#ff9c9c; text-shadow:0 0 16px #d03b3b66; }
  .vzone .za { color:var(--mute); font-size:12px; margin-top:2px; }
  button, a.dl, .rbtn, input[type=range] { cursor:pointer; }
  button:focus-visible, a:focus-visible, input:focus-visible, select:focus-visible,
  [tabindex]:focus-visible, .rbtn:focus-visible { outline:2px solid var(--blue); outline-offset:2px;
                        border-radius:6px; }
  @media (prefers-reduced-motion: reduce) {
    *, *::before, *::after { transition:none !important; animation:none !important; }
    body { background-attachment:scroll; }
  }
  /* ---- drones view ---- */
  .dgrid { display:grid; grid-template-columns:minmax(0,2.1fr) minmax(310px,1fr);
           gap:12px; padding:14px 18px 26px; align-items:start; }
  @media (max-width:1150px){ .dgrid{grid-template-columns:1fr;} }
  .dside { display:flex; flex-direction:column; gap:12px; min-width:0; }
  .dwide { grid-column:1 / -1; }
  .dpair { grid-column:1 / -1; display:grid; gap:12px;
           grid-template-columns:minmax(0,1fr) minmax(0,1.3fr); }
  @media (max-width:1150px){ .dpair{grid-template-columns:1fr;} }
  .dmapwrap { position:relative; }
  canvas#dmap { height:620px; cursor:grab; }
  canvas#dmap.drag { cursor:grabbing; }
  canvas#dwind { height:170px; cursor:crosshair; }
  #dmap-tip { position:absolute; display:none; pointer-events:none; z-index:5;
              background:#1c2330; border:1px solid var(--line); border-radius:8px;
              padding:6px 9px; font-size:12px; color:var(--ink); white-space:nowrap;
              box-shadow:0 6px 18px rgba(0,0,0,.5); font-family:var(--mono); }
  .dlegend { display:flex; flex-wrap:wrap; gap:6px 14px; margin-top:9px;
             font-size:12px; color:var(--sec); }
  .dlegend i { display:inline-block; width:10px; height:10px; margin-right:5px;
               vertical-align:-1px; border-radius:2px; }
  .dlegend i.c { border-radius:50%; }
  .dctl { display:grid; grid-template-columns:auto 1fr 52px; gap:2px 10px;
          align-items:center; font-size:12px; color:var(--sec); margin-top:8px; }
  .dctl input[type=range] { margin:4px 0; }
  .dctl b { color:var(--ink); font-family:var(--mono); text-align:right;
            font-variant-numeric:tabular-nums; }
  .dbtns { display:flex; gap:8px; flex-wrap:wrap; margin-top:10px; }
  .dbtn { border:1px solid var(--line); background:var(--panel2); color:var(--ink);
          font:inherit; font-size:12px; font-weight:600; border-radius:8px;
          padding:6px 12px; cursor:pointer; }
  .dbtn:hover { border-color:var(--blue); }
  .dbtn.gust { background:#3987e526; border-color:#3987e577; }
  .dbtn.gust:hover { box-shadow:0 0 14px #3987e555; }
  .dbtn:disabled { opacity:.45; cursor:not-allowed; }
  .dmsg { font-size:12px; color:var(--mute); margin-top:6px; min-height:16px; }
  .batt { width:84px; height:8px; background:var(--panel2); border-radius:4px;
          overflow:hidden; display:inline-block; vertical-align:middle;
          margin-right:7px; box-shadow:inset 0 0 0 1px #ffffff10; }
  .batt > i { display:block; height:100%; border-radius:4px; }
  .stp { display:inline-block; padding:1px 9px; border-radius:999px; font-size:12px;
         font-weight:700; letter-spacing:.3px; }
  table.dt { width:100%; border-collapse:collapse; font-size:12.5px; }
  table.dt th { text-align:left; color:var(--mute); font-size:12px;
                text-transform:uppercase; letter-spacing:1px; padding:5px 8px;
                border-bottom:1px solid var(--line); white-space:nowrap; }
  table.dt td { padding:5px 8px; border-bottom:1px solid var(--panel2);
                color:var(--sec); font-variant-numeric:tabular-nums; white-space:nowrap; }
  table.dt td.num { color:var(--ink); font-family:var(--mono); }
  table.dt td.note { white-space:normal; color:var(--mute); }
  .dempty { color:var(--mute); font-size:12.5px; padding:10px 2px; }
  .ev-failed .ty, .ev-drone_crash .ty { color:var(--crit); }
  .ev-standby_sent .ty, .ev-drone_arrived .ty { color:var(--violet); }
  .ev-drone_to_station .ty, .ev-drone_charging .ty,
  .ev-drone_charging_in_van .ty { color:var(--warn); }
  .ev-wind .ty, .ev-wind_hold .ty { color:var(--blue); }
  .ev-to_charger .ty, .ev-ev_charging .ty, .ev-ev_charged .ty, .ev-batch_declined .ty,
  .ev-waiting_for_charger .ty { color:#ffd228; }
  .ev-pod_stranded .ty, .ev-orders_returned .ty, .ev-pod_towed .ty { color:var(--serious); }
  .ev-delivered .ty, .ev-drone_repaired .ty { color:var(--green); }
  /* ---- force feedback view ---- */
  .fgrid { display:grid; grid-template-columns:repeat(auto-fill, minmax(340px, 1fr));
           gap:12px; padding:14px 18px 26px; align-items:start; }
  .fwide { grid-column:1 / -1; }
  .frow { display:grid; grid-template-columns:minmax(0,1fr) 64px 22px; gap:0 8px;
          align-items:center; margin-top:6px; }
  .frow label { font-size:12.5px; color:var(--sec); }
  .frow label.chg { color:var(--ink); }
  .frow b { color:var(--ink); font-family:var(--mono); font-size:12px; text-align:right;
            font-variant-numeric:tabular-nums; }
  .frow input[type=range] { grid-column:1 / -1; margin:3px 0 4px; }
  .frow .fr { border:0; background:none; color:var(--mute); cursor:pointer; padding:0;
              min-width:24px; min-height:24px; border-radius:6px;
              font-size:13px; visibility:hidden; }
  .frow .fr.on { visibility:visible; }
  .frow .fr:hover { color:var(--blue); }
  .fhelp { grid-column:1 / -1; font-size:12px; color:var(--mute); line-height:1.35; }
  canvas#fcurve { height:200px; }
  /* ---- settings: section switcher ---- */
  .setnav { display:flex; flex-wrap:wrap; align-items:center; gap:6px; padding:14px 18px 0; }
  .setnav button { border:1px solid var(--line); background:var(--panel2); color:var(--sec);
                   font:inherit; font-size:13px; font-weight:600; border-radius:999px;
                   padding:6px 14px; cursor:pointer; }
  .setnav button:hover { color:var(--ink); border-color:var(--blue); }
  .setnav button.active { background:var(--blue); border-color:var(--blue); color:#fff; }
  svg.ico { width:15px; height:15px; vertical-align:-3px; margin-right:5px; fill:none;
            stroke:currentColor; stroke-width:2; stroke-linecap:round; stroke-linejoin:round; }
  .setnote { margin-left:auto; font-size:12px; color:var(--mute); }
  #view-settings .chips { padding-top:10px; }
  #set-driving .card:has(#vt-rows) { grid-column: span 2; }
  @media (max-width:900px){ #set-driving .card:has(#vt-rows) { grid-column: auto; } }
  h2.sect { margin:10px 2px -2px; font-size:12px; letter-spacing:1.4px; text-transform:uppercase;
            color:var(--mute); font-weight:600; }
  /* ---- pod view ---- */
  .pd-sel { background:var(--panel2); border:1px solid var(--line); color:var(--ink); font:inherit;
            font-size:12px; border-radius:8px; padding:5px 8px; }
  .pd-transport { display:grid; grid-template-columns:auto minmax(0,1fr) auto auto; gap:10px;
                  align-items:center; margin:6px 0 4px; }
  .pd-scrubwrap { position:relative; }
  .pd-scrubwrap input { width:100%; margin:10px 0 6px; accent-color:var(--blue); }
  #pd-ticks { position:absolute; left:0; right:0; top:0; height:6px; pointer-events:none; }
  #pd-ticks span { position:absolute; top:0; width:2px; height:7px; border-radius:1px; }
  .pd-time { font-family:var(--mono); font-size:12.5px; color:var(--sec); white-space:nowrap; }
  .pd-steps { display:grid; grid-template-columns:1fr 1fr; gap:14px; margin-top:8px; }
  @media (max-width:900px){ .pd-steps{grid-template-columns:1fr;} }
  .pd-steps h4 { margin:4px 0 6px; font-size:12px; letter-spacing:1px; text-transform:uppercase; color:var(--mute); }
  .pd-step { display:grid; grid-template-columns:22px 1fr auto; gap:6px; width:100%; text-align:left;
             background:none; border:0; border-radius:6px; padding:5px 6px; color:var(--sec);
             font:inherit; font-size:12.5px; cursor:pointer; }
  .pd-step:hover { background:#ffffff08; }
  .pd-step b { color:var(--mute); font-family:var(--mono); font-weight:500; }
  .pd-step i { font-style:normal; color:var(--mute); font-family:var(--mono); font-size:12px; }
  .pd-step.on { background:#3987e522; color:var(--ink); }
  .pd-step.on b { color:var(--blue); }
  [data-tooltips][aria-pressed="true"], [data-shell][aria-pressed="true"] { border-color:var(--blue); box-shadow:0 0 0 1px var(--blue) inset; }
  .pd-chip { display:inline-flex; align-items:center; gap:5px; margin:2px 10px 2px 0; font-size:12px; }
  .pd-chip .batt { width:44px; margin:0; }
  .pd-pack { display:inline-block; min-width:38px; text-align:center; padding:1px 4px; margin-right:4px;
             border:1px solid var(--line); border-radius:4px; font-family:var(--mono); font-size:12px; color:var(--ink); }
  .pd-pack.empty { color:var(--mute); }
  .ffsave { display:flex; gap:8px; align-items:center; flex-wrap:wrap; margin:-4px 0 12px; }
  .ffsave input { background:var(--panel2); border:1px solid var(--line); color:var(--ink);
                  font:inherit; font-size:12.5px; border-radius:8px; padding:6px 10px; width:200px; }
  .ffsave input:focus { outline:none; border-color:var(--blue); }
  .dbtn.on { border-color:var(--blue); box-shadow:0 0 0 1px var(--blue) inset; }
  .dbtn.user { background:#9085e91f; border-color:#9085e966; }
  .dbtn .px { margin-left:7px; color:var(--mute); font-weight:400; }
  .dbtn .px:hover { color:var(--red); }
  .fmeter { height:10px; background:var(--panel2); border-radius:5px; position:relative;
            overflow:hidden; box-shadow:inset 0 0 0 1px #ffffff10; margin:4px 0 10px; }
  .fmeter > i { position:absolute; top:0; bottom:0; background:var(--blue); }
  .fmeter > s { position:absolute; left:50%; top:0; bottom:0; width:1px; background:var(--mute); }
</style></head>
<body>
<header>
  <h1>CARLA <span>HIL</span></h1>
  <nav class="tabs" role="tablist" aria-label="Views">
    <button id="tab-dash" role="tab" aria-controls="view-dash" aria-selected="true" class="active" data-i18n="nav_dash">Drive</button>
    <button id="tab-veh" role="tab" aria-controls="view-veh" data-i18n="nav_veh">Vehicle</button>
    <button id="tab-drone" role="tab" aria-controls="view-drone" data-i18n="nav_drone">Delivery</button>
    <button id="tab-stats" role="tab" aria-controls="view-stats" data-i18n="nav_stats">Statistics</button>
    <button id="tab-settings" role="tab" aria-controls="view-settings" data-i18n="nav_settings">Settings</button>
  </nav>
  <div><span id="status-dot" class="dot bad"></span>
       <span id="status" role="status" aria-live="polite">connecting…</span></div>
  <div class="push">
    <div class="lang">
      <button id="lang-en" class="active">EN</button>
      <button id="lang-ja">日本語</button>
    </div>
    <a class="dl" href="/download"><svg class="ico" viewBox="0 0 24 24" aria-hidden="true" focusable="false"><path d="M12 4v11m0 0-5-5m5 5 5-5M5 20h14"/></svg><span data-i18n="dl_csv">CSV log</span></a>
    <button id="shutdown" class="dl shut"><svg class="ico" viewBox="0 0 24 24" aria-hidden="true" focusable="false"><path d="M12 3v9M6.3 7.3a8 8 0 1 0 11.4 0"/></svg><span data-i18n="btn_shutdown">Shutdown</span></button>
  </div>
</header>

<div id="alertbar" class="warn" role="status" aria-live="polite"></div>

<div class="chips">
  <span class="chip"><span data-i18n="chip_map">Map</span><b id="map">-</b></span>
  <span class="chip"><span data-i18n="chip_vehicle">Vehicle</span><b id="vtype">-</b></span>
  <span class="chip"><span data-i18n="chip_mass">Mass (kg)</span><b id="mass">-</b></span>
  <span class="chip"><span data-i18n="chip_nearby">Nearby</span><b id="nearby">0</b></span>
  <span class="chip">FPS<b id="fps">0</b></span>
  <span class="chip"><span data-i18n="chip_simt">Sim time (s)</span><b id="simt">0</b></span>
</div>

<main id="view-dash">
<div class="grid">

  <div class="card">
    <h3><span data-i18n="card_speed">Speed</span>
        <span class="pill" id="spd-pill">OK</span></h3>
    <div class="big"><span id="speed">0</span><span class="unit">km/h</span></div>
    <div class="sub"><span data-i18n="lbl_limit">limit</span>
      <b id="splimit">-</b> km/h · <span data-i18n="lbl_avg">avg</span>
      <b id="avgkmh">0</b></div>
    <div style="display:flex; gap:14px; margin-top:10px">
      <div style="flex:1">
        <div class="kv"><span data-i18n="lbl_gear">Gear</span>
          <span class="v" id="gear">N</span></div>
        <div class="kv"><span data-i18n="lbl_heading">Heading</span>
          <span class="v" id="heading">0°</span></div>
      </div>
      <div style="text-align:right">
        <span id="i-rev" class="ind">REV</span>
        <span id="i-hb" class="ind">HB</span>
        <span id="i-abs" class="ind">ABS</span>
        <span id="i-tcs" class="ind">TCS</span>
      </div>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_inputs">Driver Inputs</h3>
    <div class="inlbl"><span data-i18n="lbl_throttle">Throttle</span><b id="t-val">0%</b></div>
    <div class="bar"><i id="t-bar" style="width:0%"></i></div>
    <div class="inlbl"><span data-i18n="lbl_brake">Brake</span><b id="b-val">0%</b></div>
    <div class="bar red"><i id="b-bar" style="width:0%"></i></div>
    <div class="inlbl"><span data-i18n="lbl_steer">Steer</span><b id="s-val">0.00</b></div>
    <div class="bar steer"><i id="s-bar" style="width:0%; left:50%"></i></div>
    <div class="chartwrap"><canvas id="ctrlhist"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#199e70"></i><span data-i18n="lbl_throttle">throttle</span></span>
      <span class="it"><i style="background:#e66767"></i><span data-i18n="lbl_brake">brake</span></span>
      <span class="it"><i style="background:#c98500"></i><span data-i18n="lbl_steer">steer</span></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_safety">Safety / ADAS</h3>
    <div style="display:flex; gap:14px">
      <div style="flex:1">
        <div class="legend" style="margin:0"><span data-i18n="lbl_fdist">Front distance</span></div>
        <div class="mid"><span id="fdist">-</span><span class="unit">m</span></div>
      </div>
      <div style="flex:1">
        <div class="legend" style="margin:0">TTC</div>
        <div class="mid"><span id="ttc">-</span><span class="unit">s</span></div>
      </div>
      <div style="flex:1">
        <div class="legend" style="margin:0">THW</div>
        <div class="mid"><span id="thw">-</span><span class="unit">s</span></div>
      </div>
    </div>
    <div class="kv"><span data-i18n="lbl_crashes">Crashes</span>
      <span class="v" id="collcnt">0</span></div>
    <div class="kv"><span data-i18n="lbl_hardbrakes">Hard brakes</span>
      <span class="v" id="hbcnt">0</span></div>
    <div class="kv"><span data-i18n="lbl_tl">Traffic light</span>
      <span class="v" id="tl">None</span></div>
    <div class="kv"><span data-i18n="lbl_stability">Stability</span>
      <span class="v" id="stab">100%</span></div>
    <div class="bar"><i id="stab-bar" style="width:100%"></i></div>
    <div class="chartwrap"><canvas id="ttchist"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#e66767"></i>TTC</span>
      <span class="it"><i style="background:#3987e5"></i>THW</span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_perf">Driver Performance</h3>
    <div class="kv"><span data-i18n="lbl_sdlp">SDLP (lane keeping)</span>
      <span class="v" id="sdlp">0.00 m</span></div>
    <div class="kv"><span data-i18n="lbl_srr">Steer reversals</span>
      <span class="v" id="srr">0 /min</span></div>
    <div class="kv"><span data-i18n="lbl_jerkrms">Jerk RMS (comfort)</span>
      <span class="v" id="jerkrms">0.00 m/s³</span></div>
    <div class="kv"><span data-i18n="lbl_laneinv">Lane invasions</span>
      <span class="v" id="laneinv">0</span></div>
    <div class="kv"><span data-i18n="lbl_rlv">Red-light violations</span>
      <span class="v" id="rlv">0</span></div>
    <div class="kv"><span data-i18n="lbl_comp">Speed compliance</span>
      <span class="v" id="comp">100%</span></div>
    <div class="bar"><i id="comp-bar" style="width:100%"></i></div>
  </div>

  <div class="card wide">
    <h3 data-i18n="card_lanepos">Lane Position (SDLP window)</h3>
    <div class="chartwrap"><canvas id="lanehist" class="tall"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#3987e5"></i><span data-i18n="lg_laneoff">offset from centre</span><b id="lv-laneoff">0.00 m</b></span>
      <span class="it"><i style="background:#e66767"></i><span data-i18n="lg_laneedge">lane edges</span></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_events">Driving Events</h3>
    <div id="events" class="events"></div>
  </div>

  <div class="card wide">
    <h3 data-i18n="card_speedhist">Speed History vs Limit</h3>
    <div class="chartwrap"><canvas id="speedhist" class="tall"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#3987e5"></i><span data-i18n="lg_speed">speed</span><b id="lv-speed">0</b></span>
      <span class="it"><i style="background:#c98500"></i><span data-i18n="lg_limit">limit</span><b id="lv-limit">–</b></span>
      <span class="it"><i style="background:#199e70"></i><span data-i18n="lg_avg">avg</span><b id="lv-avg">0</b></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_gg">G-G Diagram</h3>
    <canvas id="gg" class="tall"></canvas>
    <div class="legend"><span data-i18n="gg_hint">X: lateral · Y: longitudinal · rings = 0.5 g</span></div>
  </div>

  <div class="card">
    <h3 data-i18n="card_acc">Acceleration</h3>
    <div class="chartwrap"><canvas id="acchist" class="tall"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#3987e5"></i><span data-i18n="lg_lon">lon</span><b id="lv-lon">0.0</b></span>
      <span class="it"><i style="background:#e66767"></i><span data-i18n="lg_lat">lat</span><b id="lv-lat">0.0</b></span>
      <span class="it"><i style="background:#c98500"></i><span data-i18n="lg_jerk">jerk</span><b id="lv-jerk">0.0</b></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_dyn">Vehicle Dynamics</h3>
    <div class="kv"><span data-i18n="lbl_lon">Lon accel</span><span class="v" id="lonA">0.00</span></div>
    <div class="kv"><span data-i18n="lbl_lat">Lat accel</span><span class="v" id="latA">0.00</span></div>
    <div class="kv"><span data-i18n="lbl_vert">Vert accel</span><span class="v" id="verA">0.00</span></div>
    <div class="kv"><span data-i18n="lbl_jerk">Jerk</span><span class="v" id="jerk">0.00</span></div>
    <div class="kv"><span data-i18n="lbl_g">G-force</span><span class="v" id="gfr">0.00</span></div>
    <div class="kv"><span data-i18n="lbl_yawrate">Yaw rate</span><span class="v" id="yaw">0.00°/s</span></div>
    <div class="kv"><span data-i18n="lbl_slip">Slip angle</span><span class="v" id="slip">0.00°</span></div>
    <div class="kv"><span data-i18n="lbl_curv">Curvature</span><span class="v" id="curv">0.0000</span></div>
  </div>

  <div class="card">
    <h3 data-i18n="card_yaw">Yaw Rate &amp; Steering</h3>
    <div class="chartwrap"><canvas id="yawhist" class="tall"></canvas><div class="xline"></div></div>
    <div class="legend">
      <span class="it"><i style="background:#3987e5"></i><span data-i18n="lg_yaw">yaw °/s</span><b id="lv-yaw">0.0</b></span>
      <span class="it"><i style="background:#c98500"></i><span data-i18n="lg_steer100">steer ×100</span><b id="lv-steer100">0</b></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_lane">Lane / Road</h3>
    <div class="kv"><span data-i18n="lbl_onroad">On road</span>
      <span class="v" id="onroad">-</span></div>
    <div class="kv"><span data-i18n="lbl_roadlane">Road / Lane</span>
      <span class="v"><span id="road">-</span> / <span id="lane">-</span></span></div>
    <div class="kv"><span data-i18n="lbl_lanew">Lane width</span>
      <span class="v" id="lw">0.0 m</span></div>
    <div class="kv"><span data-i18n="lbl_laneoff">Lane offset</span>
      <span class="v" id="lo">0.00 m</span></div>
    <div class="kv"><span data-i18n="lbl_junction">Junction</span>
      <span class="v" id="junc">no</span></div>
    <canvas id="lanevis"></canvas>
  </div>

  <div class="card">
    <h3 data-i18n="card_weather">Weather</h3>
    <div class="kv"><span data-i18n="lbl_cloud">Cloud</span>
      <span class="v"><span id="wcl">0</span>%</span></div>
    <div class="bar"><i id="wcl-bar" style="width:0%"></i></div>
    <div class="kv"><span data-i18n="lbl_rain">Rain</span>
      <span class="v"><span id="wrn">0</span>%</span></div>
    <div class="bar red"><i id="wrn-bar" style="width:0%"></i></div>
    <div class="kv"><span data-i18n="lbl_wet">Wetness</span>
      <span class="v"><span id="wwt">0</span>%</span></div>
    <div class="bar"><i id="wwt-bar" style="width:0%"></i></div>
    <div class="kv"><span data-i18n="lbl_fog">Fog</span>
      <span class="v"><span id="wfg">0</span>%</span></div>
    <div class="bar"><i id="wfg-bar" style="width:0%; background:var(--yellow)"></i></div>
    <div class="kv"><span data-i18n="lbl_wind">Wind</span>
      <span class="v"><span id="wwd">0</span>%</span></div>
    <div class="kv"><span data-i18n="lbl_sunalt">Sun altitude</span>
      <span class="v" id="wsa">0°</span></div>
  </div>

  <div class="card">
    <h3 data-i18n="card_trip">Trip</h3>
    <div class="big"><span id="tripkm">0.00</span><span class="unit">km</span></div>
    <div class="sub"><span data-i18n="lbl_elapsed">elapsed</span> <b id="tripsec">00:00</b></div>
    <div class="kv" style="margin-top:8px"><span data-i18n="lbl_avgspeed">Avg speed</span>
      <span class="v" id="avgkmh2">0 km/h</span></div>
    <div class="kv"><span data-i18n="lbl_crashes">Crashes</span>
      <span class="v" id="collcnt2">0</span></div>
    <div class="kv"><span data-i18n="lbl_hardbrakes">Hard brakes</span>
      <span class="v" id="hbcnt2">0</span></div>
    <div class="kv"><span data-i18n="lbl_collpeak">Collision peak</span>
      <span class="v" id="cpeak">0.0</span></div>
  </div>

  <div class="card">
    <h3 data-i18n="card_coll">Collision Intensity (10 s)</h3>
    <canvas id="collchart" class="tall"></canvas>
    <div class="legend"><span data-i18n="coll_hint">Normal impulse from collision sensor</span></div>
  </div>

  <div class="card wide">
    <h3 data-i18n="card_map">Situational Awareness (80 m)</h3>
    <canvas id="minimap" class="tall" style="height:260px"></canvas>
    <div class="legend">
      <span class="it"><i style="background:#fab219"></i><span data-i18n="lg_hero">hero</span></span>
      <span class="it"><i style="background:#e66767"></i><span data-i18n="lg_nearby">nearby vehicle</span></span>
      <span class="it"><i style="background:#9085e9"></i><span data-i18n="lg_bike">bike</span></span>
      <span class="it"><i style="background:#22d3ee"></i><span data-i18n="lg_walker">pedestrian</span></span>
      <span class="it"><i style="background:#3987e5"></i><span data-i18n="lg_cone">front cone</span></span>
    </div>
  </div>

  <div class="card">
    <h3 data-i18n="card_trail">World Trail</h3>
    <canvas id="trail" class="tall"></canvas>
    <div class="legend"><span data-i18n="trail_hint">World X/Y path history</span></div>
  </div>

  <div class="card">
    <h3 data-i18n="card_gnss">GNSS / IMU</h3>
    <div class="kv"><span data-i18n="lbl_pos">X / Y / Z</span>
      <span class="v"><span id="x">0</span>/<span id="y">0</span>/<span id="z">0</span></span></div>
    <div class="kv"><span data-i18n="lbl_lat2">Latitude</span><span class="v" id="lat">0</span></div>
    <div class="kv"><span data-i18n="lbl_lon2">Longitude</span><span class="v" id="lon">0</span></div>
    <div class="kv"><span data-i18n="lbl_accel">Accel x/y/z</span>
      <span class="v"><span id="ax">0</span>/<span id="ay">0</span>/<span id="az">0</span></span></div>
    <div class="kv"><span data-i18n="lbl_gyro">Gyro x/y/z</span>
      <span class="v"><span id="gx">0</span>/<span id="gy">0</span>/<span id="gz">0</span></span></div>
    <div class="kv"><span data-i18n="lbl_compass">Compass</span><span class="v" id="compass">0°</span></div>
  </div>

</div>
</main>

<main id="view-stats" hidden>
  <div class="sect" data-i18n="stats_session">Current session</div>
  <div class="tiles">
    <div class="tile" style="--ac:var(--crit)">
      <div class="tl" data-i18n="tile_crash">Crashes</div>
      <div class="tv"><span id="st-crash">0</span></div></div>
    <div class="tile" style="--ac:var(--blue)">
      <div class="tl" data-i18n="tile_lane">Out of lane</div>
      <div class="tv"><span id="st-lane">0</span></div></div>
    <div class="tile" style="--ac:var(--serious)">
      <div class="tl" data-i18n="tile_rlv">Red lights run</div>
      <div class="tv"><span id="st-rlv">0</span></div></div>
    <div class="tile" style="--ac:var(--warn)">
      <div class="tl" data-i18n="tile_hb">Hard brakes</div>
      <div class="tv"><span id="st-hb">0</span></div></div>
    <div class="tile" style="--ac:var(--green)">
      <div class="tl" data-i18n="tile_dist">Distance</div>
      <div class="tv"><span id="st-dist">0.00</span><span class="tu">km</span></div></div>
    <div class="tile" style="--ac:var(--violet)">
      <div class="tl" data-i18n="tile_dur">Duration</div>
      <div class="tv"><span id="st-dur">00:00</span></div></div>
    <div class="tile" style="--ac:var(--blue)">
      <div class="tl" data-i18n="tile_avg">Avg speed</div>
      <div class="tv"><span id="st-avg">0</span><span class="tu">km/h</span></div></div>
    <div class="tile" style="--ac:var(--good)">
      <div class="tl" data-i18n="tile_comp">Compliance</div>
      <div class="tv"><span id="st-comp">100</span><span class="tu">%</span></div></div>
  </div>

  <div class="statwrap">
    <div class="card">
      <h3><span data-i18n="stats_history">Session history</span>
          <span style="text-transform:none; letter-spacing:0" data-i18n="hist_hint">crashes &amp; out-of-lane per session (latest 20)</span></h3>
      <div class="chartwrap"><canvas id="histchart" class="tall"></canvas></div>
      <div class="legend">
        <span class="it"><i style="background:#e66767"></i><span data-i18n="tile_crash">Crashes</span></span>
        <span class="it"><i style="background:#3987e5"></i><span data-i18n="tile_lane">Out of lane</span></span>
      </div>
    </div>
    <div class="card">
      <h3 data-i18n="stats_sessions">Saved sessions</h3>
      <div class="tblwrap">
      <table class="sess">
        <thead><tr>
          <th data-i18n="th_time">Time</th><th data-i18n="th_map">Map</th>
          <th data-i18n="th_dur">Duration</th><th data-i18n="th_dist">Distance (km)</th>
          <th data-i18n="th_crash">Crashes</th><th data-i18n="th_lane">Out of lane</th>
          <th data-i18n="th_rlv">Red lights</th><th data-i18n="th_hb">Hard brakes</th>
          <th data-i18n="th_comp">Compliance</th>
        </tr></thead>
        <tbody id="sess-body"></tbody>
      </table>
      </div>
      <div id="sess-empty" class="sub" style="display:none; padding:14px"
           data-i18n="stats_empty">No saved sessions yet</div>
    </div>
  </div>
</main>

<main id="view-veh" hidden>
<div class="vehwrap">

  <div class="vcol">
    <div class="card">
      <h3 data-i18n="veh_status">Vehicle Status</h3>
      <div style="text-align:center; margin:4px 0 12px">
        <span id="vh-pill" class="vpill ok">OK</span></div>
      <div class="kv"><span data-i18n="lbl_speed">Speed</span>
        <span class="v"><span id="vh-speed">0</span> km/h</span></div>
      <div class="kv"><span data-i18n="lbl_gear">Gear</span>
        <span class="v" id="vh-gear">N</span></div>
      <div class="kv"><span data-i18n="lbl_steer">Steer</span>
        <span class="v" id="vh-steer">0.0°</span></div>
      <div class="kv"><span data-i18n="lbl_heading">Heading</span>
        <span class="v" id="vh-head">0°</span></div>
      <div class="kv"><span data-i18n="lbl_stability">Stability</span>
        <span class="v" id="vh-stab">100%</span></div>
      <div class="bar"><i id="vh-stab-bar" style="width:100%"></i></div>
      <div style="text-align:center; margin-top:10px">
        <span id="vi-rev" class="ind">REV</span>
        <span id="vi-hb" class="ind">HB</span>
        <span id="vi-abs" class="ind">ABS</span>
        <span id="vi-tcs" class="ind">TCS</span></div>
      <div style="display:flex; justify-content:center; gap:12px;
                  margin-top:12px">
        <button id="sig-l" class="sigbtn" aria-label="left turn signal">
          ◀</button>
        <button id="sig-r" class="sigbtn" aria-label="right turn signal">
          ▶</button>
      </div>
      <div class="legend" style="justify-content:center; margin-top:7px"
           data-i18n="sig_hint">turn signal · wheel button 11 (L) / 10 (R)</div>
    </div>
    <div class="card">
      <h3><span data-i18n="veh_sensors">Sensors</span>
          <span class="pill" id="vh-sensn">0</span></h3>
      <div id="sensor-rows"></div>
      <div class="legend" style="margin-top:9px"
           data-i18n="veh_sn_hint">click a sensor to show its mounting position</div>
    </div>
  </div>

  <div class="card vstage-card">
    <h3><span data-i18n="veh_diagram">Sensor &amp; Impact Map</span>
        <span class="pill" id="vh-mini">–</span></h3>
    <div class="vstage">
    <svg id="vsvg" viewBox="0 0 300 600" role="img"
         aria-label="Top-down vehicle diagram with sensor positions and collision impact points">

      <!-- nearby-object icons, public-domain top-view drawings from
           publicdomainvectors.org (body tinted via currentColor)
           周辺オブジェクト用アイコン：publicdomainvectors.org のパブリック
           ドメイン素材（車体色は currentColor で着色） -->
      <defs>
        <symbol id="ic-car" viewBox="0 0 960 476">
<title
 id="nctitle3968">Red Car - Top View</title>
 <defs
 id="ncdefs4">
 <linearGradient
 id="nclinearGradient3759">
 <stop
 id="ncstop3761"
 style="stop-color:#1a1a1a;stop-opacity:1"
 offset="0" />
 <stop
 id="ncstop3763"
 style="stop-color:#000000;stop-opacity:0"
 offset="1" />
 </linearGradient>
 <linearGradient
 x1="871.33002"
 y1="842.29999"
 x2="848.15997"
 y2="834.67999"
 id="nclinearGradient4149"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="translate(8.3085,-2.6518)" />
 <linearGradient
 x1="879.90002"
 y1="537.5"
 x2="812.14001"
 y2="533.5"
 id="nclinearGradient4153"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.89332,0,0,0.89332,80.349,365.15)" />
 <linearGradient
 x1="879.90002"
 y1="537.5"
 x2="815.82001"
 y2="531.90997"
 id="nclinearGradient4155"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="translate(27.625,296.49)" />
 <linearGradient
 x1="871.33002"
 y1="842.29999"
 x2="848.15997"
 y2="834.67999"
 id="nclinearGradient4185"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(1,0,0,-1,8.3085,1452)" />
 <linearGradient
 x1="887.90002"
 y1="528.35999"
 x2="876.14001"
 y2="528.41998"
 id="nclinearGradient4187"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(1,0,0,-1,21.438,1151.5)" />
 <linearGradient
 x1="879.90002"
 y1="537.5"
 x2="815.82001"
 y2="531.90997"
 id="nclinearGradient4189"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(1,0,0,-1,27.625,1152.9)" />
 <linearGradient
 x1="897.21997"
 y1="542.40002"
 x2="883.76001"
 y2="535.37"
 id="nclinearGradient4191"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(1,0,0,-1,21.438,1151.5)" />
 <linearGradient
 x1="880.71002"
 y1="552.04999"
 x2="835.98999"
 y2="501.07999"
 id="nclinearGradient4193"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(1,0,0,-1,21.438,1151.5)" />
 <linearGradient
 x1="887.90002"
 y1="528.35999"
 x2="805.28998"
 y2="529.60999"
 id="nclinearGradient4195"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.89332,0,0,-0.89332,94.156,1087.8)" />
 <linearGradient
 x1="879.90002"
 y1="537.5"
 x2="812.14001"
 y2="533.5"
 id="nclinearGradient4197"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.89332,0,0,-0.89332,80.349,1084.2)" />
 <linearGradient
 x1="229.7"
 y1="873.14001"
 x2="205.59"
 y2="867.67999"
 id="nclinearGradient4199"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.99043,-0.13799,-0.13799,-0.99043,146.05,1483.3)" />
 <linearGradient
 x1="238.83"
 y1="873.06"
 x2="216.56"
 y2="872.65002"
 id="nclinearGradient4201"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.99043,-0.13799,-0.13799,-0.99043,156.31,1482)" />
 <linearGradient
 x1="887.90002"
 y1="528.35999"
 x2="876.14001"
 y2="528.41998"
 id="nclinearGradient4203"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="translate(21.438,297.91)" />
 <linearGradient
 x1="897.21997"
 y1="542.40002"
 x2="883.76001"
 y2="535.37"
 id="nclinearGradient4205"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="translate(21.438,297.91)" />
 <linearGradient
 x1="880.71002"
 y1="552.04999"
 x2="835.98999"
 y2="501.07999"
 id="nclinearGradient4207"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="translate(21.438,297.91)" />
 <linearGradient
 x1="887.90002"
 y1="528.35999"
 x2="805.28998"
 y2="529.60999"
 id="nclinearGradient4209"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.89332,0,0,0.89332,94.156,361.57)" />
 <linearGradient
 x1="229.7"
 y1="873.14001"
 x2="205.59"
 y2="867.67999"
 id="nclinearGradient4211"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.99043,0.13799,-0.13799,0.99043,146.05,-33.885)" />
 <linearGradient
 x1="238.83"
 y1="873.06"
 x2="216.56"
 y2="872.65002"
 id="nclinearGradient4213"
 xlink:href="#nclinearGradient3759"
 gradientUnits="userSpaceOnUse"
 gradientTransform="matrix(0.99043,0.13799,-0.13799,0.99043,156.31,-32.603)" />
 </defs>
 <g
 transform="translate(-52.937,-486.69)"
 id="nclayer1">
 <g
 id="ncg3890">
 <path
 d="m 610.52,493.69 c -1.5086,0.009 -4.7211,0.30491 -6.4687,0.9375 l -3.5,1.5 8.6562,35.938 -124.81,0.28125 c -2.4363,0.005 -4.8876,-0.014 -7.3437,-0.0312 -4.912,-0.0343 -9.8649,-0.10455 -14.844,-0.21875 -7.2926,-0.16728 -14.669,-0.41288 -22.062,-0.71875 -0.39591,-0.0164 -0.79137,-0.0457 -1.1875,-0.0625 -14.932,-0.63018 -30.007,-1.4917 -45.031,-2.4375 -20.326,-1.2827 -40.52,-2.7074 -60.124,-3.875 -14.528,-0.8653 -28.732,-1.5796 -42.375,-2 -9.3692,-0.28873 -18.464,-0.45655 -27.25,-0.40625 -4.3158,0.0247 -8.5678,0.0933 -12.719,0.21875 -4.1508,0.12546 -8.3801,0.3553 -12.656,0.65625 -2.138,0.15047 -4.2778,0.31024 -6.4374,0.5 -6.4689,0.5684 -13.045,1.3214 -19.594,2.1875 -0.0101,10e-4 -0.0211,-10e-4 -0.0312,0 -4.3725,0.57858 -8.7185,1.2255 -13.062,1.9062 -4.3439,0.68073 -8.6734,1.3978 -12.937,2.1562 -4.2537,0.75662 -8.4704,1.5654 -12.594,2.375 -0.01,0.002 -0.0212,-0.002 -0.0312,0 -2.0664,0.40586 -4.0697,0.80417 -6.0937,1.2188 -6.0614,1.2416 -11.934,2.516 -17.5,3.7812 -0.0101,0.002 -0.0211,-0.002 -0.0312,0 -3.7165,0.8449 -7.2872,1.6752 -10.719,2.5 -6.8638,1.6484 -13.115,3.2346 -18.531,4.6562 -8.1236,2.1325 -14.382,3.9272 -18.094,5 -2.4736,0.71578 -3.8125,1.125 -3.8125,1.125 l -13.687,3.75 c -0.9024,0.24873 -1.7781,0.6929 -2.625,1.3125 -0.28771,0.2105 -0.56234,0.43593 -0.84374,0.6875 -1.0996,0.9796 -2.151,2.2707 -3.1562,3.8438 -0.0046,0.007 0.0046,0.0243 0,0.0312 -0.50036,0.78473 -1.0225,1.6405 -1.5,2.5625 -0.0042,0.008 0.0042,0.0232 0,0.0312 -0.47757,0.9237 -0.95114,1.9142 -1.4062,2.9688 -0.0037,0.009 0.0037,0.0222 0,0.0312 -3.667,8.5095 -6.62,21.131 -8.9374,36.219 -0.0015,0.01 0.0015,0.0212 0,0.0312 -0.03595,0.23416 -0.0581,0.48342 -0.09375,0.71875 -0.50462,3.3315 -0.99204,6.7982 -1.4375,10.344 -0.03048,0.24265 -0.06355,0.47519 -0.09375,0.71875 -0.50157,4.0432 -0.94672,8.2025 -1.375,12.469 -0.001,0.01 9.99e-4,0.0211 0,0.0312 -0.21354,2.1284 -0.42912,4.2948 -0.62499,6.4688 -9.21e-4,0.01 9.21e-4,0.0209 0,0.0312 -0.19581,2.1743 -0.38397,4.3492 -0.56249,6.5625 -8.26e-4,0.01 8.25e-4,0.0209 0,0.0312 -0.35775,4.4376 -0.67817,8.9458 -0.96874,13.5 -6.56e-4,0.0101 6.55e-4,0.0209 0,0.0312 -0.8726,13.684 -1.4596,27.789 -1.7812,41.562 -2.45e-4,0.0104 2.44e-4,0.0207 0,0.0312 -0.21443,9.1894 -0.3126,18.213 -0.3126,26.907 0,0.96763 0.02755,1.9629 0.03125,2.9375 -0.0037,0.97462 -0.03125,1.9699 -0.03125,2.9375 0,8.6932 0.09816,17.717 0.3125,26.906 2.44e-4,0.0105 -2.45e-4,0.0208 0,0.0312 0.32165,13.774 0.90864,27.878 1.7812,41.562 6.55e-4,0.0103 -6.56e-4,0.0211 0,0.0312 0.29057,4.5542 0.61099,9.0624 0.96874,13.5 8.25e-4,0.0103 -8.26e-4,0.0211 0,0.0312 0.17852,2.2133 0.36668,4.3882 0.56249,6.5625 9.21e-4,0.0103 -9.21e-4,0.0211 0,0.0312 0.19588,2.174 0.41145,4.3404 0.62499,6.4688 9.99e-4,0.0102 -0.001,0.0211 0,0.0312 0.42826,4.2663 0.87342,8.4256 1.375,12.469 0.0302,0.24356 0.06327,0.4761 0.09375,0.71875 0.44545,3.5456 0.93287,7.0123 1.4375,10.344 0.03565,0.23533 0.0578,0.48459 0.09375,0.71875 0.0015,0.01 -0.0015,0.0215 0,0.0312 2.3174,15.087 5.2704,27.709 8.9374,36.219 0.0037,0.009 -0.0037,0.0226 0,0.0312 0.45509,1.0546 0.92866,2.045 1.4062,2.9688 0.0042,0.008 -0.0042,0.0231 0,0.0312 0.47753,0.92204 0.99962,1.7778 1.5,2.5625 0.0046,0.007 -0.0046,0.0242 0,0.0312 1.0052,1.5731 2.0566,2.8642 3.1562,3.8438 0.2814,0.25157 0.55603,0.477 0.84374,0.6875 0.84686,0.6196 1.7226,1.0638 2.625,1.3125 l 13.687,3.75 c 0,0 1.3388,0.40922 3.8125,1.125 3.7111,1.0728 9.97,2.8675 18.094,5 5.4157,1.4217 11.667,3.0078 18.531,4.6562 3.4314,0.82484 7.0022,1.6551 10.719,2.5 0.0102,0.002 0.0211,-0.002 0.0312,0 5.5661,1.2652 11.438,2.5396 17.5,3.7812 2.024,0.41458 4.0273,0.81289 6.0937,1.2188 0.0101,0.002 0.0212,-0.002 0.0312,0 4.1232,0.80965 8.3399,1.6184 12.594,2.375 4.264,0.75843 8.5935,1.4755 12.937,2.1562 4.3439,0.68073 8.6898,1.3277 13.062,1.9062 0.0102,10e-4 0.0211,-10e-4 0.0312,0 6.5486,0.86611 13.125,1.6191 19.594,2.1875 2.1597,0.18976 4.2994,0.34953 6.4374,0.5 4.276,0.30095 8.5053,0.53079 12.656,0.65625 4.1508,0.12546 8.4028,0.19403 12.719,0.21875 8.7859,0.0503 17.881,-0.11752 27.25,-0.40625 13.642,-0.42043 27.847,-1.1347 42.375,-2 19.604,-1.1676 39.798,-2.5923 60.124,-3.875 15.024,-0.94578 30.098,-1.8073 45.031,-2.4375 0.39612,-0.0168 0.79158,-0.0461 1.1875,-0.0625 7.3934,-0.30587 14.77,-0.55147 22.062,-0.71875 4.9787,-0.1142 9.9316,-0.18447 14.844,-0.21875 2.456,-0.0172 4.9073,-0.0365 7.3437,-0.0312 l 124.81,0.28125 -8.6562,35.938 3.5,1.5 c 1.7476,0.63259 4.9601,0.92851 6.4687,0.9375 0.8486,0.005 1.7551,-0.0741 2.6875,-0.25 0.31069,-0.0585 0.62352,-0.14105 0.93749,-0.21875 0.30459,-0.0756 0.63258,-0.15668 0.93749,-0.25 0.62844,-0.19306 1.2635,-0.42156 1.875,-0.6875 1.8215,-0.79351 3.5342,-1.9314 4.7812,-3.3125 0.01,-0.0107 0.0217,-0.0206 0.0312,-0.0312 0.6248,-0.6979 1.1123,-1.4512 1.4687,-2.2812 l 12.156,-31.312 109.94,0.25 c 0.7469,0.3739 1.4761,0.74511 2.2187,1.0938 2.9792,1.3995 5.911,2.597 8.8436,3.625 1.4623,0.51259 2.9178,0.97549 4.375,1.4062 5.1,1.5075 10.188,2.5515 15.344,3.25 2.9462,0.39932 5.9259,0.69388 8.9374,0.90625 1.5057,0.1061 3.034,0.18011 4.5624,0.25 3.057,0.13994 6.1386,0.21283 9.3124,0.25 6.3475,0.0741 12.979,0 20,0 10.41,0 20.322,-0.53122 29.781,-1.5625 3.7835,-0.41251 7.4911,-0.89749 11.125,-1.4688 7.2676,-1.1425 14.228,-2.6189 20.906,-4.375 1.6694,-0.43903 3.3357,-0.89874 4.9687,-1.375 4.899,-1.4288 9.6128,-3.0254 14.187,-4.7812 1.5248,-0.58527 3.042,-1.1605 4.5312,-1.7812 2.9783,-1.2414 5.882,-2.557 8.7186,-3.9375 1.4183,-0.69027 2.8355,-1.4006 4.2187,-2.125 2.7664,-1.4488 5.4668,-2.98 8.0937,-4.5625 1.3135,-0.79125 2.5961,-1.6135 3.875,-2.4375 6.3943,-4.1201 12.363,-8.6255 17.906,-13.531 4.4286,-3.9195 8.6015,-8.0798 12.5,-12.469 0.005,-0.006 -0.005,-0.0254 0,-0.0312 0.97044,-1.0929 1.937,-2.1916 2.875,-3.3125 0.005,-0.006 -0.005,-0.0251 0,-0.0312 0.93773,-1.1209 1.8757,-2.2578 2.7812,-3.4062 0.005,-0.006 -0.005,-0.0249 0,-0.0312 2.7256,-3.4582 5.3132,-7.0189 7.7499,-10.719 0.005,-0.007 -0.005,-0.024 0,-0.0312 0.80882,-1.2286 1.6289,-2.4644 2.4062,-3.7188 0.005,-0.008 -0.005,-0.0238 0,-0.0312 2.3405,-3.7781 4.5668,-7.6525 6.6249,-11.656 0.004,-0.008 -0.004,-0.0231 0,-0.0312 0.68299,-1.3292 1.3477,-2.6781 2,-4.0312 0.004,-0.008 -0.004,-0.0229 0,-0.0312 0.65204,-1.3533 1.316,-2.7169 1.9375,-4.0938 0.004,-0.009 -0.004,-0.0227 0,-0.0312 1.2464,-2.7625 2.438,-5.5517 3.5625,-8.4062 0.004,-0.009 -0.004,-0.0224 0,-0.0312 0.56023,-1.4229 1.0948,-2.8674 1.625,-4.3125 0.003,-0.009 -0.003,-0.0222 0,-0.0312 0.53002,-1.4452 1.0623,-2.908 1.5625,-4.375 0.49999,-1.4671 0.99842,-2.949 1.4687,-4.4375 0.003,-0.009 -0.003,-0.022 0,-0.0312 0.47013,-1.4886 0.93435,-2.9905 1.375,-4.5 0.003,-0.01 -0.003,-0.0217 0,-0.0312 0.44045,-1.5096 0.87009,-3.0324 1.2812,-4.5625 0.003,-0.01 -0.003,-0.0217 0,-0.0312 1.2381,-4.6101 2.3684,-9.306 3.34372,-14.094 v -0.0312 c 0.3236,-1.5894 0.6112,-3.2044 0.9063,-4.8125 v -0.0312 c 2.0747,-11.318 3.4621,-23.062 4.1562,-35.219 5e-4,-0.0103 -6e-4,-0.0211 0,-0.0312 0.098,-1.7279 0.1787,-3.4757 0.25,-5.2188 4e-4,-0.0103 -4e-4,-0.0211 0,-0.0312 0.1426,-3.4969 0.2467,-7.0072 0.2812,-10.562 0,-0.0103 -10e-5,-0.0211 0,-0.0312 0.017,-1.783 0.01,-3.578 0,-5.375 0,-0.6358 -0.022,-1.2714 -0.031,-1.9062 0.01,-0.63485 0.031,-1.2704 0.031,-1.9062 0.01,-1.797 0.017,-3.592 0,-5.375 -10e-5,-0.0102 0,-0.0209 0,-0.0312 -0.034,-3.5553 -0.1386,-7.0656 -0.2812,-10.562 -4e-4,-0.0102 4e-4,-0.0209 0,-0.0312 -0.071,-1.7431 -0.152,-3.4908 -0.25,-5.2188 -6e-4,-0.0102 5e-4,-0.0209 0,-0.0312 -0.6941,-12.157 -2.0815,-23.9 -4.1562,-35.219 v -0.0312 c -0.2951,-1.6081 -0.5827,-3.2231 -0.9063,-4.8125 v -0.0312 c -0.97539,-4.7878 -2.10562,-9.4836 -3.34372,-14.094 -0.003,-0.01 0.003,-0.0213 0,-0.0312 -0.41115,-1.5301 -0.84079,-3.0529 -1.2812,-4.5625 -0.003,-0.01 0.003,-0.0213 0,-0.0312 -0.44063,-1.5095 -0.90485,-3.0114 -1.375,-4.5 -0.003,-0.009 0.003,-0.0223 0,-0.0312 -0.47032,-1.4885 -0.96875,-2.9704 -1.4687,-4.4375 -0.50017,-1.467 -1.0325,-2.9298 -1.5625,-4.375 -0.003,-0.009 0.003,-0.0223 0,-0.0312 -0.53021,-1.4451 -1.0648,-2.8896 -1.625,-4.3125 -0.004,-0.009 0.004,-0.0223 0,-0.0312 -1.1244,-2.8546 -2.3161,-5.6438 -3.5625,-8.4062 -0.004,-0.008 0.004,-0.0223 0,-0.0312 -0.62148,-1.3768 -1.2854,-2.7405 -1.9375,-4.0938 -0.004,-0.008 0.004,-0.0233 0,-0.0312 -0.65224,-1.3532 -1.317,-2.7021 -2,-4.0312 -0.004,-0.008 0.004,-0.0233 0,-0.0312 -2.0582,-4.0038 -4.2844,-7.8782 -6.6249,-11.656 -0.005,-0.007 0.005,-0.0233 0,-0.0312 -0.77733,-1.2543 -1.5974,-2.4902 -2.4062,-3.7188 -0.005,-0.007 0.005,-0.0242 0,-0.0312 -2.4367,-3.6999 -5.0243,-7.2606 -7.7499,-10.719 -0.005,-0.006 0.005,-0.0253 0,-0.0312 -0.90549,-1.1485 -1.8435,-2.2853 -2.7812,-3.4062 -0.005,-0.006 0.005,-0.0253 0,-0.0312 -0.93801,-1.1209 -1.9045,-2.2196 -2.875,-3.3125 -0.005,-0.006 0.005,-0.0253 0,-0.0312 -3.8983,-4.3889 -8.0712,-8.5493 -12.5,-12.469 -5.543,-4.9058 -11.512,-9.4112 -17.906,-13.531 -1.2789,-0.82402 -2.5615,-1.6462 -3.875,-2.4375 -2.6269,-1.5825 -5.3273,-3.1137 -8.0937,-4.5625 -1.3832,-0.72439 -2.8004,-1.4347 -4.2187,-2.125 -2.8367,-1.3806 -5.7403,-2.6961 -8.7186,-3.9375 -1.4892,-0.62071 -3.0064,-1.196 -4.5312,-1.7812 -4.5746,-1.7558 -9.2883,-3.3525 -14.187,-4.7812 -1.633,-0.47626 -3.2993,-0.93597 -4.9687,-1.375 -6.6777,-1.7561 -13.638,-3.2325 -20.906,-4.375 -3.6338,-0.57126 -7.3414,-1.0562 -11.125,-1.4688 -9.4586,-1.0326 -19.371,-1.5638 -29.781,-1.5638 -7.0204,0 -13.652,-0.0741 -20,0 -3.1738,0.0372 -6.2554,0.11006 -9.3124,0.25 -1.5285,0.0699 -3.0568,0.1439 -4.5624,0.25 -3.0115,0.21237 -5.9912,0.50693 -8.9374,0.90625 -5.1556,0.69849 -10.244,1.7425 -15.344,3.25 -1.4572,0.43076 -2.9127,0.89366 -4.375,1.4062 -2.9327,1.028 -5.8645,2.2255 -8.8436,3.625 -0.74267,0.34864 -1.4718,0.71985 -2.2187,1.0938 l -109.94,0.25 -12.156,-31.312 c -0.35644,-0.83 -0.84393,-1.5834 -1.4687,-2.2812 -0.01,-0.0105 -0.0213,-0.0206 -0.0312,-0.0312 -1.247,-1.3811 -2.9597,-2.519 -4.7812,-3.3125 -0.61146,-0.26594 -1.2465,-0.49444 -1.875,-0.6875 -0.30491,-0.0933 -0.6329,-0.1744 -0.93749,-0.25 -0.31402,-0.0777 -0.62685,-0.16025 -0.93754,-0.21875 -0.9324,-0.1759 -1.8389,-0.255 -2.6875,-0.25 z"
 id="ncpath3855"
 style="fill-opacity:0.99607999;stroke:#191919;stroke-width:14" />
 <path
 d="m 610.52,493.69 c -1.5086,0.009 -4.7211,0.30611 -6.4687,0.9375 l -3.5,1.5 8.6562,35.844 -124.81,0.28125 c -77.963,0.1654 -166.52,-11.504 -232.93,-9.5 -66.412,2.0037 -152.12,28 -152.12,28 l -13.687,3.7812 c -19.251,5.2963 -25.718,97.367 -25.718,166.78 0,1.113 0.02665,2.2531 0.03125,3.375 -0.0046,1.1219 -0.03125,2.262 -0.03125,3.375 0,69.414 6.4673,161.48 25.718,166.78 l 13.687,3.7812 c 0,0 85.711,25.996 152.12,28 66.412,2.0037 154.97,-9.6654 232.93,-9.5 l 124.81,0.28125 -8.6562,35.844 3.5,1.5 c 1.7476,0.63139 4.96,0.92854 6.4687,0.9375 0.84859,0.005 1.7551,-0.0744 2.6875,-0.25 0.3107,-0.0584 0.62352,-0.14119 0.93749,-0.21875 0.30459,-0.0754 0.63259,-0.15686 0.93749,-0.25 0.62844,-0.19269 1.2635,-0.42206 1.875,-0.6875 1.8215,-0.79201 3.5342,-1.9028 4.7812,-3.2812 0.01,-0.0107 0.0217,-0.0208 0.0312,-0.0312 0.6248,-0.69658 1.1123,-1.4528 1.4687,-2.2812 l 12.156,-31.25 109.94,0.25 c 23.9,11.942 45.511,10.719 73.593,10.719 133.25,0 187.63002,-86.586 187.00002,-201.38 0,-0.7802 -0.019,-1.5656 -0.031,-2.3438 0.012,-0.77811 0.031,-1.5636 0.031,-2.3438 0.6282,-114.79 -53.74902,-201.38 -187.00002,-201.38 -28.082,0 -49.693,-1.2236 -73.593,10.719 l -109.94,0.25 -12.156,-31.25 c -0.35645,-0.82844 -0.84393,-1.5847 -1.4687,-2.2812 -0.01,-0.0104 -0.0213,-0.0206 -0.0312,-0.0312 -1.247,-1.3785 -2.9597,-2.4892 -4.7812,-3.2812 -0.61148,-0.26544 -1.2465,-0.49481 -1.875,-0.6875 -0.3049,-0.0931 -0.6329,-0.1746 -0.93749,-0.25 -0.31397,-0.0776 -0.62679,-0.16035 -0.93749,-0.21875 -0.93239,-0.1756 -1.8389,-0.255 -2.6875,-0.25 z"
 id="ncpath2853"
 style="fill:currentColor;fill-opacity:0.99607999" />
 <path
 d="m 400.34,855.24 c -33.364,0 -65.307,1.8 -94.811,5.0625 25.66,48.714 97.985,30.265 205.56,31.531 49.686,0.58471 89.543,1.8793 121.53,2.375 -47.16,-23.334 -133.53,-38.969 -232.28,-38.969 z"
 id="ncpath3643"
 style="opacity:0.9;fill:#262626;fill-opacity:0.99607999" />
 <path
 d="m 400.34,855.24 c -3.2064,0 -6.3831,0.0295 -9.5624,0.0625 0.81825,16.171 6.4281,30.257 14.594,38.844 4.6714,-0.0756 9.4951,-0.19655 14.437,-0.34375 -8.5657,-8.1923 -14.593,-22.228 -15.719,-38.562 -1.2512,-0.005 -2.4947,0 -3.75,0 z"
 id="ncpath3658"
 style="opacity:0.5;fill-opacity:0.99607999" />
 <path
 d="m 989.02,827.5 -5.0937,0.59375 c -21.545,2.5127 -37.688,25.979 -39.281,54.531 l -0.37499,7.125 5.2499,-4.8438 c 15.889,-14.68 28.303,-32.507 37.406,-52.75 l 2.09,-4.65 z"
 id="ncpath3707"
 style="text-indent:0;text-transform:none;block-progression:tb;color:#000000;fill:#212121;stroke:#191919;stroke-width:5" />
 <path
 d="m 783.47,838.5 c 0,0 79.677,-22.596 105.38,-31.982 26.839,-9.8018 98.859,-39.146 98.859,-39.146 0,0 -8.7409,42.47 -30.483,57.918 -77.23,54.87 -232.69,53.85 -232.69,53.85"
 id="ncpath3715"
 style="opacity:0.5;fill:none;stroke:#292929;stroke-width:6;stroke-linecap:round;stroke-linejoin:round" />
 <path
 d="m 869.97,817.84 -4.4374,2.3438 c 0.98912,1.1568 1.7955,2.4286 2.375,3.8438 4.7979,11.717 -10.736,29.236 -26.875,35.781 -0.51675,0.20958 -1.8129,0.84066 -3.4062,1.6562 l 13.625,-3.875 c 17.306,-8.4576 27.47,-23.082 23,-34 -0.91615,-2.2373 -2.3752,-4.1661 -4.2812,-5.75 z"
 id="ncpath3757"
 style="fill:url(#nclinearGradient4149)" />
 <path
 d="m 878.55,813.38 -4.4375,2.3438 c 0.98913,1.1568 1.7955,2.4286 2.375,3.8438 4.7979,11.717 -10.736,29.236 -26.875,35.781 -0.51676,0.20958 -1.8129,0.84066 -3.4062,1.6562 l 13.625,-3.875 c 17.306,-8.4576 27.47,-23.082 23,-34 -0.91615,-2.2373 -2.3752,-4.1661 -4.2812,-5.75 z"
 id="ncpath3787"
 style="fill:url(#nclinearGradient4203)" />
 <path
 d="m 884.74,811.96 -4.4374,2.3438 c 0.98913,1.1568 1.7955,2.4286 2.375,3.8438 4.7979,11.717 -10.736,29.236 -26.875,35.781 -0.51675,0.20958 -1.8129,0.84066 -3.4062,1.6562 l 13.625,-3.875 c 17.306,-8.4576 27.47,-23.082 23,-34 -0.91615,-2.2373 -2.3752,-4.1661 -4.2812,-5.75 z"
 id="ncpath3752"
 style="fill:url(#nclinearGradient4155)" />
 <path
 d="m 901.65,807.69 -6.1874,1.8438 c 0.96015,1.7128 1.6545,3.5323 2.0312,5.4688 3.1194,16.034 -20.962,34.284 -43.031,38.5 -3.395,0.64864 -28.884,8.576 -32.158,8.8044 v 4.125 l 41.439,-12.148 c 26.285,-5.4963 44.949,-22.448 41.875,-38.25 -0.59564,-3.0616 -1.956,-5.8595 -3.9687,-8.3438 z"
 id="ncpath3735"
 style="fill:url(#nclinearGradient4205)" />
 <path
 d="m 901.65,807.69 -6.1874,1.8438 c 0.96015,1.7128 1.6545,3.5323 2.0312,5.4688 3.1194,16.034 -20.962,34.284 -43.031,38.5 -3.395,0.64864 -28.884,8.576 -32.158,8.8044 v 4.125 l 41.439,-12.148 c 26.285,-5.4963 44.949,-22.448 41.875,-38.25 -0.59564,-3.0616 -1.956,-5.8595 -3.9687,-8.3438 z"
 id="ncpath3783"
 style="fill:url(#nclinearGradient4207)" />
 <path
 d="m 857.12,822.46 -3.9641,2.0937 c 0.88361,1.0334 1.604,2.1696 2.1216,3.4337 4.2861,10.467 -9.5906,26.117 -24.008,31.964 -0.46163,0.18723 -1.6195,0.75098 -3.0428,1.4796 l 12.171,-3.4616 c 15.46,-7.5554 24.54,-20.62 20.546,-30.373 -0.81842,-1.9987 -2.1218,-3.7216 -3.8245,-5.1366 z"
 id="ncpath3799"
 style="fill:url(#nclinearGradient4209)" />
 <path
 d="m 843.32,826.03 -3.9641,2.0937 c 0.88361,1.0334 1.604,2.1696 2.1216,3.4337 4.2861,10.467 -9.5906,26.117 -24.008,31.964 -0.46162,0.18723 -1.6195,0.75098 -3.0428,1.4796 l 12.171,-3.4616 c 15.46,-7.5554 24.54,-20.62 20.546,-30.373 -0.81842,-1.9987 -2.1218,-3.7216 -3.8245,-5.1366 z"
 id="ncpath3803"
 style="fill:url(#nclinearGradient4153)" />
 <path
 d="m 233.27,845.72 c 8.293,-2.0234 15.486,-1.4788 19.797,5.7872 l -2.4934,17.897 c -6.8751,6.1732 -13.75,4.9509 -20.625,0.15543 l 3.3212,-23.839 z"
 id="ncrect3861"
 style="fill:url(#nclinearGradient4211)" />
 <path
 d="m 253.54,848.99 c 8.1502,-1.2102 15.167,-0.5728 18.843,5.5081 l -2.3731,17.034 c -6.4839,2.9748 -12.983,5.2096 -19.631,0.14793 l 3.1611,-22.69 z"
 id="ncpath3864"
 style="fill:url(#nclinearGradient4213)" />
 <path
 d="m 400.34,852.75 c -33.454,0 -65.492,1.7894 -95.093,5.0625 l -3.6562,0.40625 1.7187,3.25 c 6.6711,12.664 16.562,21.113 29.062,26.438 12.501,5.3241 27.572,7.6126 45.093,8.4375 35.042,1.6498 79.954,-2.6312 133.59,-2 49.659,0.58438 89.508,1.8787 121.53,2.375 l 1.125,-4.75 c -47.84,-23.68 -134.34,-39.22 -233.36,-39.22 z m 0,5 c 91.169,0 171.75,13.479 220.09,33.719 -29.952,-0.58241 -65.212,-1.606 -109.31,-2.125 -53.937,-0.63473 -98.976,3.6522 -133.4,2.0312 -17.214,-0.81046 -31.767,-3.1054 -43.406,-8.0625 -10.453,-4.4521 -18.485,-11.154 -24.5,-20.906 28.307,-2.9831 58.735,-4.6562 90.53,-4.6562 z"
 id="ncpath4025"
 style="text-indent:0;text-transform:none;block-progression:tb;opacity:0.9;color:#000000;fill:#191919" />
 <path
 d="m 260.5,607.38 -77.749,12.469 c -27.15,4.3542 -48.947,48.773 -50.999,104.84 2.0523,56.071 23.849,100.49 50.999,104.84 l 77.749,12.469 c 13.296,0 24,-10.704 24,-24 v -186.62 c 0,-13.296 -10.704,-24 -24,-24 z"
 id="ncrect2864"
 style="opacity:0.9;fill:#262626;fill-opacity:0.99607999;stroke:#191919;stroke-width:5" />
 <path
 d="m 691.96,573.16 c -2.9692,0 -5.8933,0.33215 -8.7812,0.96875 -0.0104,-0.01 -0.0208,-0.021 -0.0312,-0.0312 l -63.843,12.312 c -17.728,6.6047 -32,14.272 -32,32 v 212.56 c 0,17.728 14.272,25.395 32,32 l 63.843,12.312 c 0.0105,-0.0102 0.0208,-0.0211 0.0312,-0.0312 2.8879,0.6366 5.812,0.96875 8.7812,0.96875 45.395,0 82.198,-57.363 82.312,-151.53 -0.11408,-94.169 -36.916,-151.53 -82.312,-151.53 z"
 id="ncpath3703"
 style="opacity:0.9;fill:#262626;fill-opacity:0.99607999;stroke:#191919;stroke-width:5" />
 <path
 d="m 400.34,594.15 c -33.364,0 -65.307,-1.8 -94.811,-5.0625 25.66,-48.714 97.985,-30.265 205.56,-31.531 49.686,-0.58471 89.543,-1.8793 121.53,-2.375 -47.16,23.334 -133.53,38.969 -232.28,38.969 z"
 id="ncpath4157"
 style="opacity:0.9;fill:#262626;fill-opacity:0.99607999" />
 <path
 d="m 400.34,594.15 c -3.2064,0 -6.3831,-0.0295 -9.5624,-0.0625 0.81825,-16.171 6.4281,-30.257 14.594,-38.844 4.6714,0.0756 9.4951,0.19655 14.437,0.34375 -8.5657,8.1923 -14.593,22.228 -15.719,38.562 -1.2512,0.005 -2.4947,0 -3.75,0 z"
 id="ncpath4159"
 style="opacity:0.5;fill-opacity:0.99607999" />
 <path
 d="m 989.02,621.89 -5.0937,-0.59375 c -21.545,-2.5127 -37.688,-25.979 -39.281,-54.531 l -0.37499,-7.125 5.2499,4.8438 c 15.889,14.68 28.303,32.507 37.406,52.75 l 2.0937,4.6562 z"
 id="ncpath4161"
 style="text-indent:0;text-transform:none;block-progression:tb;color:#000000;fill:#212121;stroke:#191919;stroke-width:5" />
 <path
 d="m 783.47,610.89 c 0,0 79.677,22.596 105.38,31.982 26.839,9.8018 98.859,39.146 98.859,39.146 0,0 -8.7409,-42.47 -30.483,-57.918 -77.23,-54.87 -232.69,-53.86 -232.69,-53.86"
 id="ncpath4163"
 style="opacity:0.5;fill:none;stroke:#292929;stroke-width:6;stroke-linecap:round;stroke-linejoin:round" />
 <path
 d="m 869.97,631.55 -4.4374,-2.3438 c 0.98912,-1.1568 1.7955,-2.4286 2.375,-3.8438 4.7979,-11.717 -10.736,-29.236 -26.875,-35.781 -0.51675,-0.20958 -1.8129,-0.84066 -3.4062,-1.6562 l 13.625,3.875 c 17.306,8.4576 27.47,23.082 23,34 -0.91615,2.2373 -2.3752,4.1661 -4.2812,5.75 z"
 id="ncpath4165"
 style="fill:url(#nclinearGradient4185)" />
 <path
 d="m 878.55,636.01 -4.4375,-2.3438 c 0.98913,-1.1568 1.7955,-2.4286 2.375,-3.8438 4.7979,-11.717 -10.736,-29.236 -26.875,-35.781 -0.51676,-0.20958 -1.8129,-0.84066 -3.4062,-1.6562 l 13.625,3.875 c 17.306,8.4576 27.47,23.082 23,34 -0.91615,2.2373 -2.3752,4.1661 -4.2812,5.75 z"
 id="ncpath4167"
 style="fill:url(#nclinearGradient4187)" />
 <path
 d="m 884.74,637.42 -4.4374,-2.3438 c 0.98913,-1.1568 1.7955,-2.4286 2.375,-3.8438 4.7979,-11.717 -10.736,-29.236 -26.875,-35.781 -0.51675,-0.20958 -1.8129,-0.84066 -3.4062,-1.6562 l 13.625,3.875 c 17.306,8.4576 27.47,23.082 23,34 -0.91615,2.2373 -2.3752,4.1661 -4.2812,5.75 z"
 id="ncpath4169"
 style="fill:url(#nclinearGradient4189)" />
 <path
 d="m 901.65,641.7 -6.1874,-1.8438 c 0.96015,-1.7128 1.6545,-3.5323 2.0312,-5.4688 3.1194,-16.034 -20.962,-34.284 -43.031,-38.5 -3.395,-0.64864 -28.884,-8.576 -32.158,-8.8044 v -4.125 l 41.439,12.148 c 26.285,5.4963 44.949,22.448 41.875,38.25 -0.59564,3.0616 -1.956,5.8595 -3.9687,8.3438 z"
 id="ncpath4171"
 style="fill:url(#nclinearGradient4191)" />
 <path
 d="m 901.65,641.7 -6.1874,-1.8438 c 0.96015,-1.7128 1.6545,-3.5323 2.0312,-5.4688 3.1194,-16.034 -20.962,-34.284 -43.031,-38.5 -3.395,-0.64864 -28.884,-8.576 -32.158,-8.8044 v -4.125 l 41.439,12.148 c 26.285,5.4963 44.949,22.448 41.875,38.25 -0.59564,3.0616 -1.956,5.8595 -3.9687,8.3438 z"
 id="ncpath4173"
 style="fill:url(#nclinearGradient4193)" />
 <path
 d="m 857.12,626.93 -3.9641,-2.0937 c 0.88361,-1.0334 1.604,-2.1696 2.1216,-3.4337 4.2861,-10.467 -9.5906,-26.117 -24.008,-31.964 -0.46163,-0.18723 -1.6195,-0.75098 -3.0428,-1.4796 l 12.171,3.4616 c 15.46,7.5554 24.54,20.62 20.546,30.373 -0.81842,1.9987 -2.1218,3.7216 -3.8245,5.1366 z"
 id="ncpath4175"
 style="fill:url(#nclinearGradient4195)" />
 <path
 d="m 843.32,623.36 -3.9641,-2.0937 c 0.88361,-1.0334 1.604,-2.1696 2.1216,-3.4337 4.2861,-10.467 -9.5906,-26.117 -24.008,-31.964 -0.46162,-0.18723 -1.6195,-0.75098 -3.0428,-1.4796 l 12.171,3.4616 c 15.46,7.5554 24.54,20.62 20.546,30.373 -0.81842,1.9987 -2.1218,3.7216 -3.8245,5.1366 z"
 id="ncpath4177"
 style="fill:url(#nclinearGradient4197)" />
 <path
 d="m 233.27,603.66 c 8.293,2.0234 15.486,1.4788 19.797,-5.7872 l -2.4934,-17.897 c -6.8751,-6.1732 -13.75,-4.9509 -20.625,-0.15543 l 3.3212,23.839 z"
 id="ncpath4179"
 style="fill:url(#nclinearGradient4199)" />
 <path
 d="m 253.54,600.4 c 8.1502,1.2102 15.167,0.5728 18.843,-5.5081 l -2.3731,-17.034 c -6.4839,-2.9748 -12.983,-5.2096 -19.631,-0.14793 l 3.1611,22.69 z"
 id="ncpath4181"
 style="fill:url(#nclinearGradient4201)" />
 <path
 d="m 400.34,596.64 c -33.454,0 -65.492,-1.7894 -95.093,-5.0625 l -3.6562,-0.40625 1.7187,-3.25 c 6.6711,-12.664 16.562,-21.113 29.062,-26.438 12.501,-5.3241 27.572,-7.6126 45.093,-8.4375 35.042,-1.6498 79.954,2.6312 133.59,2 49.659,-0.58438 89.508,-1.8787 121.53,-2.375 l 1.125,4.75 c -47.849,23.675 -134.36,39.219 -233.37,39.219 z m 0,-5 c 91.169,0 171.75,-13.479 220.09,-33.719 -29.952,0.58241 -65.212,1.606 -109.31,2.125 -53.937,0.63473 -98.976,-3.6522 -133.4,-2.0312 -17.214,0.81046 -31.767,3.1054 -43.406,8.0625 -10.453,4.4521 -18.485,11.154 -24.5,20.906 28.307,2.9831 58.735,4.6562 90.53,4.6562 z"
 id="ncpath4183"
 style="text-indent:0;text-transform:none;block-progression:tb;opacity:0.9;color:#000000;fill:#191919" />
 </g>
 </g>
        </symbol>
        <symbol id="ic-bike" viewBox="0 0 354.33 885.83">
<defs id="nbdefs3">
 <linearGradient id="nblinearGradient3270" y2="274.25" gradientUnits="userSpaceOnUse" x2="1290" gradientTransform="translate(-522.24 -9.2108)" y1="314.78" x1="1288.4">
 <stop id="nbstop3081" style="stop-color:#999999" offset="0"/>
 <stop id="nbstop3083" style="stop-color:#999999;stop-opacity:0" offset="1"/>
 </linearGradient>
 </defs>
 <g id="nblayer1" transform="translate(0 -166.54)">
 <g id="nbg3237" transform="matrix(2.507 0 0 2.507 -1748.3 -408.39)">
 <rect id="nbrect3065" ry="2.8426" style="stroke:#2e3436;stroke-linecap:square;stroke-width:.83782;fill:#333333" transform="rotate(-17.908)" height="5.6852" width="26.631" y="534.91" x="580.04"/>
 <rect id="nbrect3057" style="stroke:#2e3436;stroke-linecap:square;fill:#2e3436" ry="10.607" height="55.306" width="19.193" y="260.11" x="759.64"/>
 <path id="nbpath3075" d="m741.71 334.36 2.2728 3.5355 9.3439 2.5254 6.0609 6.0609 2.778 3.0305 3.5355 2.2728 7.5761-0.50508 2.778-1.7678 2.7779-4.0406 5.5558-4.7982 9.849-1.7678-0.2525-4.2931-1.7678-25.506-5.0508-16.92-17.425-11.869h-4.0406l-17.678 13.385-5.0507 16.415z" style="stroke:#000000;stroke-width:1px;fill:#1a1a1a"/>
 <path id="nbpath3077" d="m744.61 315.42s8.7125-4.9245 20.708-5.0508c11.996-0.12626 26.769 4.9245 26.769 4.9245l-1.7677-14.395-6.8186-11.996-15.405-8.5863-17.804 11.996-6.1872 9.849z" style="fill:url(#nblinearGradient3270)"/>
 <path id="nbpath3033" style="stroke:#000000;stroke-width:1px;fill:currentColor" d="m734.68 307.21s1.2627-13.385 5.5559-17.678c4.2931-4.2932 28.158-15.657 28.158-15.657l0.6313 7.1973s-17.804 9.7227-19.824 15.405c-2.0203 5.6821-3.7881 11.996-4.1669 15.91-0.3788 3.9143-1.2627 22.728-1.2627 22.728l-7.5761-2.9042 0.6313-15.784-1.894-7.4499z"/>
 <path id="nbpath3035" style="stroke:#000000;stroke-width:1px;fill:#0e0e0e" d="m743.97 310.94-5.0507-0.25254s-9.4702-1.1996-13.953 0.69448c-4.4825 1.894-11.49 10.102-11.49 10.102l0.6314 2.336s9.0914 2.5885 13.826 0.69448c4.7351-1.894 8.902-7.0711 8.902-7.0711l0.2526-2.0834 5.8715 0.18941s0.8207-0.63135 1.1995-2.336-0.1894-2.2728-0.1894-2.2728z"/>
 <path id="nbpath3037" style="stroke:#000000;stroke-width:1px;fill:currentColor" d="m742.04 338.1s-4.2857 11.964-4.8214 18.393c-0.5357 6.4286-0.3571 27.321-0.3571 27.321l8.2142 12.321 2.8572-13.214s12.678 26.071 20.536 26.429c3.408 0.15491 0.016-57.679 0.016-57.679s-7.5164-0.71429-8.2307-3.5714c-0.7143-2.8572-0.8928-2.8572-0.8928-2.8572s-5.7143-5.5357-8.9286-6.0714c-3.2143-0.53572-8.3929-1.0714-8.3929-1.0714z"/>
 <path id="nbpath3039" style="stroke:#000000;stroke-width:1px;fill:#1a181b" d="m745.43 395.24s4.1071 3.2143 4.1071 8.5714c0 5.3572 2.3215 13.036 2.3215 13.036s-4.4643 6.0714-3.9286 13.036c0.5357 6.9643 8.5714 16.964 8.5714 16.964l11.607 1.9643 0.7142-40.357s-8.75-3.9286-10.357-7.8571c-1.6071-3.9286-10.357-17.857-10.357-17.857l-2.1429 8.3929z"/>
 <rect id="nbrect3059" ry="7.5" style="stroke:#2e3436;stroke-linecap:square;fill:#888a85" transform="rotate(7.011)" height="40.179" width="15" y="371.6" x="801.26"/>
 <path id="nbpath3041" style="stroke:#000000;stroke-width:1px;fill:currentColor" d="m767.93 449.17v54.643s-3.5714-3.9286-5.1786-6.25c-1.6071-2.3214-5.3571-4.6428-6.0714-9.2857s-9.8214-15.179-12.5-21.25c-2.6786-6.0714-4.4643-14.107-2.5-23.393 1.9643-9.2857 6.7857-9.2857 6.7857-9.2857l6.25 10 4.4643 3.9286z"/>
 <rect id="nbrect3067" style="stroke:#2e3436;stroke-linecap:square;stroke-width:.83782;fill:#333333" transform="matrix(-.95155 -.30749 -.30749 .95155 0 0)" ry="2.8426" height="5.6852" width="26.631" y="62.161" x="-882.12"/>
 <path id="nbpath3043" d="m802.46 307.21s-1.2627-13.385-5.5559-17.678c-4.2931-4.2932-28.284-15.657-28.284-15.657l0.3788 7.5762s16.92 9.3439 18.94 15.026c2.0203 5.6821 3.7881 11.996 4.1669 15.91 0.3788 3.9143 1.2627 22.728 1.2627 22.728l7.5761-2.9042-0.6313-15.784 1.894-7.4499z" style="stroke:#000000;stroke-width:1px;fill:currentColor"/>
 <path id="nbpath3045" d="m793.18 310.94 5.0507-0.25254s9.4702-1.1996 13.953 0.69448c4.4825 1.894 11.49 10.102 11.49 10.102l-0.6314 2.336s-9.0914 2.5885-13.826 0.69448c-4.7351-1.894-8.902-7.0711-8.902-7.0711l-0.2526-2.0834-5.8715 0.18941s-0.8207-0.63135-1.1995-2.336 0.1894-2.2728 0.1894-2.2728z" style="stroke:#000000;stroke-width:1px;fill:#0e0e0e"/>
 <path id="nbpath3047" d="m795.1 338.1s4.2857 11.964 4.8214 18.393c0.5357 6.4286 0.3571 27.321 0.3571 27.321l-8.2142 12.321-2.8572-13.214s-9.4643 26.786-19.464 27.143c-3.4094 0.12176-1.5928-58.519-1.5928-58.519s8.0214-0.58802 8.7357-3.4452 0.8928-2.8572 0.8928-2.8572 5.7143-5.5357 8.9286-6.0714c3.2143-0.53572 8.3929-1.0714 8.3929-1.0714z" style="stroke:#000000;stroke-width:1px;fill:currentColor"/>
 <path id="nbpath3049" d="m791.71 395.24s-4.1071 3.2143-4.1071 8.5714c0 5.3572-2.3215 13.036-2.3215 13.036s4.4643 6.0714 3.9286 13.036c-0.5357 6.9643-8.5714 16.964-8.5714 16.964l-11.607 1.9643 0.2679-40.268s7.7679-4.0179 9.375-7.9464c1.6071-3.9286 10.357-17.857 10.357-17.857l2.1429 8.3929z" style="stroke:#000000;stroke-width:1px;fill:#1a181b"/>
 <rect id="nbrect3093" style="stroke:#2e3436;stroke-linecap:square;fill:#888a85" transform="matrix(-.99252 .12206 .12206 .99252 0 0)" ry="7.5" height="40.179" width="15" y="559.21" x="-724.33"/>
 <path id="nbpath3051" d="m767.6 449.17 0.3571 54.643s4.8214-3.9286 6.4286-6.25c1.6071-2.3214 5.3571-4.6428 6.0714-9.2857s9.8214-15.179 12.5-21.25c2.6786-6.0714 4.4643-14.107 2.5-23.393-1.9643-9.2857-6.7857-9.2857-6.7857-9.2857l-6.25 10-4.4643 3.9286z" style="stroke:#000000;stroke-width:1px;fill:currentColor"/>
 <path id="nbpath3101" d="m789.01 439.35c0.1338 5.4692-2.3214 10.84-2.3214 15.893 0 9.0869-8.9286 16.982-8.9286 26.25 0 5.2368-6.25 6.8602-6.25 10.714 0 1.1294-1.0714 4.3437-1.0714 3.2143 0-1.8616 34.205-46.021 18.571-56.071z" style="fill:#d40000"/>
 <path id="nbpath3099" style="fill:currentColor" d="m750.37 365.87c0.5489 2.0548 1.7707 3.9022 2.4107 5.9822 1.0301 3.348 3.0845 6.4452 3.9285 9.8214 3.6998 14.799-6.6719-7.4878-9.1964-8.75-0.064-0.0323-5.0841 11.518-4.9107 11.518 1.0463 0-0.9144-5.6004-1.0714-6.0714-1.3074-3.922-0.5646-8.9942 0-12.946 0.3644-2.5509-1.1359-21.875 1.6964-21.875 5.7352 0 1.6594 20.265 7.1429 22.321z"/>
 <path id="nbpath3097" d="m785.61 365.87c-0.5489 2.0548-1.7707 3.9022-2.4107 5.9822-1.0301 3.348-3.0845 6.4452-3.9285 9.8214-3.6998 14.799 6.6719-7.4878 9.1964-8.75 0.064-0.0323 5.0841 11.518 4.9107 11.518-1.0463 0 0.9144-5.6004 1.0714-6.0714 1.3074-3.922 0.5646-8.9942 0-12.946-0.3644-2.5509 1.1359-21.875-1.6964-21.875-5.7352 0-1.6594 20.265-7.1429 22.321z" style="fill:#ff2a2a"/>
 <rect id="nbrect3063" style="stroke:#2e3436;stroke-linecap:square;fill:#333333" ry="7.5" height="57.806" width="13.485" y="334.7" x="715.12"/>
 <path id="nbpath3103" style="fill:currentColor" d="m747.56 439.88c-0.1338 5.4692 2.3214 10.84 2.3214 15.893 0 9.0869 8.9286 16.982 8.9286 26.25 0 5.2368 6.25 6.8602 6.25 10.714 0 1.1294 1.0714 4.3437 1.0714 3.2143 0-1.8616-34.205-46.021-18.571-56.071z"/>
 <rect id="nbrect3107" style="stroke:#2e3436;stroke-linecap:square;stroke-width:1.0214;fill:#1a1a1a" transform="matrix(-.029133 -.99958 .99950 -.031709 0 0)" ry="15.561" height="54.772" width="32.468" y="728.29" x="-463.45"/>
 <path id="nbpath3053" style="stroke:#2e3436;stroke-linecap:square;fill:#edd400" d="m1311.2 370.09c0 13.529-8.2538 24.496-18.435 24.496-10.182 0-18.435-10.967-18.435-24.496s8.2537-24.496 18.435-24.496c10.182 0 18.435 10.967 18.435 24.496z" transform="translate(-523.76 16.548)"/>
 <rect id="nbrect3069" ry="7.5" style="stroke:#2e3436;stroke-linecap:square;fill:#333333" transform="rotate(-47.246)" height="45.357" width="15.329" y="792.36" x="194.94"/>
 <rect id="nbrect3071" ry="7.5" style="stroke:#2e3436;stroke-linecap:square;fill:#333333" transform="scale(-1,1)" height="57.806" width="13.485" y="334.34" x="-820.13"/>
 <rect id="nbrect3073" style="stroke:#2e3436;stroke-linecap:square;fill:#333333" transform="matrix(-.67885 -.73428 -.73428 .67885 0 0)" ry="7.5" height="45.357" width="15.329" y="-335.19" x="-846.99"/>
 <path id="nbpath3055" d="m755.35 378.93s4.4195-9.0914 13.763-8.8388c12.008 0.32454 14.647 7.8287 14.647 7.8287l1.2627 10.48s-3.5356-9.9753-14.268-9.4702c-10.733 0.50508-15.91 9.3439-15.91 9.3439z" style="stroke:#000000;stroke-width:1px;fill:#2e3436"/>
 <path id="nbpath3087" d="m743.6 329.31s14.774 4.7982 15.152 5.3033c0.3788 0.50507-0.1263 2.7779 2.1466 3.283 2.2728 0.50508 3.788-1.894 5.9346-1.894s4.672 2.5254 7.3236 2.1466c2.6517-0.37881 3.0305-3.0305 3.0305-3.0305l16.036-4.9245-0.6314-4.2932-15.405 5.8084s-2.5254-1.0102-5.3033-1.0102-4.4194 1.7678-4.4194 1.7678-2.1466-1.389-4.0406-1.894c-1.8941-0.50508-4.5457 1.2627-4.5457 1.2627l-14.647-6.3135z" style="stroke:#000000;stroke-width:1px;fill:#333333"/>
 <path id="nbpath3089" style="stroke:#000000;stroke-linecap:square;fill:#333333" d="m1288.3 328.16c0 2.9987-3.7594 5.4296-8.3969 5.4296s-8.3969-2.4309-8.3969-5.4296 3.7594-5.4296 8.3969-5.4296 8.3969 2.4309 8.3969 5.4296z" transform="matrix(.96467 .26345 -.26345 .96467 -390.2 -332.66)"/>
 <path id="nbpath3091" style="stroke:#000000;stroke-linecap:square;fill:#333333" d="m1288.3 328.16c0 2.9987-3.7594 5.4296-8.3969 5.4296s-8.3969-2.4309-8.3969-5.4296 3.7594-5.4296 8.3969-5.4296 8.3969 2.4309 8.3969 5.4296z" transform="matrix(-.96467 .26345 .26345 .96467 1927.4 -332.66)"/>
 </g>
 </g>
        </symbol>
        <symbol id="ic-walk" viewBox="0 0 1133.26 1813.493">
          <g fill="currentColor">
<path d="M259.994,1627.544l-134.066-69.672c-27.294-14.184-37.892-48.437-22.318-74.962c0.117-0.198,0.222-0.404,0.352-0.595
	c20.072-29.311,47.127-30.98,75.615-10.802l338.467-598.919l-24.81-229.762c-0.877-8.125-11.646-10.368-15.694-3.269
	l-58.965,103.403c-7.068,12.395-18.329,21.861-31.756,26.692l-256.006,92.119c-9.814,3.531-20.587,4.376-30.549,1.287
	c-15.305-4.747-24.502-16.306-29.908-32.478c-5.347-15.997-5.884-31.553,5.704-46.489c6.626-8.541,15.846-14.703,25.997-18.417
	l209.078-76.492c17.329-6.34,31.784-18.729,40.7-34.885c0,0,74.19-150.04,107.397-194.591
	c15.611-20.943,38.374-29.697,61.212-28.806l-2.401-80.416c-70.036-34.903-101.958-89.017-80.416-159.632
	c27.907-91.48,138.027-100.22,193.238-64.813c67.748,43.447,83.524,123.689,38.408,189.638
	c-12.46,18.213-27.464,31.654-45.609,40.808l7.201,63.613c46.681,6.615,83.8,31.068,115.223,70.814
	c40.899,51.733,170.645,201.158,170.645,201.158c6.97,8.217,11.994,17.902,14.697,28.332l81.11,313.018
	c3.367,22.27-10.635,39.001-38.408,45.609c-26.61,6.331-49.21,3.301-59.813-29.767c-0.138-0.163-0.228-0.347-0.284-0.553
	l-73.074-270.483c-4.773-17.666-13.585-33.981-25.742-47.657l-92.69-104.276c-6.991-7.865-19.94-1.908-18.516,8.518l32.472,237.778
	l330.066,778.955l-165.633,54.011c-31.006,5.601-47.469-9.32-55.211-30.006c-14.296-38.198,1.855-64.681,37.207-74.415
	l-229.245-536.506c-15.403-32.206-47.409-39.608-64.813-9.602L259.994,1627.544z"/>
          </g>
        </symbol>
        <symbol id="ic-truck" viewBox="0 0 100 220">
          <rect x="10" y="6" width="80" height="64" rx="12"
                fill="currentColor"/>
          <rect x="17" y="42" width="66" height="16" rx="4" fill="#10151d"
                opacity=".8"/>
          <rect x="6" y="76" width="88" height="138" rx="6"
                fill="currentColor"/>
          <rect x="14" y="84" width="72" height="122" rx="3" fill="#10151d"
                opacity=".35"/>
        </symbol>
      </defs>
      <!-- radar grid: sqrt distance scale, r = 240*sqrt(d/50)
           レーダーグリッド：√距離スケール（r = 240*sqrt(d/50)） -->
      <g class="rings">
        <circle cx="150" cy="300" r="75.9"/>
        <circle cx="150" cy="300" r="131.5" class="r2"/>
        <circle cx="150" cy="300" r="185.9" class="r2"/>
        <circle cx="150" cy="300" r="240"/>
      </g>
      <g>
        <text class="rlabel" x="154" y="227">5 m</text>
        <text class="rlabel" x="154" y="172">15 m</text>
        <text class="rlabel" x="154" y="117">30 m</text>
        <text class="rlabel" x="154" y="63">50 m</text>
      </g>

      <!-- own-lane guide from the lane sensor (JS) / 車線ガイド -->
      <g id="vlane" style="display:none">
        <rect id="vlane-fill" x="120" y="130" width="60" height="340"
              fill="#ffffff" opacity="0.03"/>
        <line id="vlane-l" class="laneline" x1="120" y1="130"
              x2="120" y2="470"/>
        <line id="vlane-r" class="laneline" x1="180" y1="130"
              x2="180" y2="470"/>
      </g>
      <line x1="150" y1="18" x2="150" y2="582" stroke="#22314a"
            stroke-opacity=".4" stroke-dasharray="2 6"/>
      <line x1="8" y1="300" x2="292" y2="300" stroke="#22314a"
            stroke-opacity=".4" stroke-dasharray="2 6"/>
      <path d="M150,30 l-7,11 h14 Z" fill="#69758a"/>
      <text class="slabel" x="150" y="24" text-anchor="middle">FRONT</text>
      <text class="slabel" x="150" y="592" text-anchor="middle">REAR</text>

      <!-- car block drawn at 50% so ring distances stay honest
           車両まわりは距離感を保つため50%縮小で描画 -->
      <g id="vscale"
         transform="translate(150,300) scale(0.5) translate(-150,-300)">
      <!-- speed streaks -->
      <g id="vstreaks" style="opacity:0; --sd:1.2s">
        <line class="streak" x1="70" y1="130" x2="70" y2="490"/>
        <line class="streak" x1="84" y1="110" x2="84" y2="500"
              style="animation-delay:-.4s"/>
        <line class="streak" x1="216" y1="110" x2="216" y2="500"
              style="animation-delay:-.8s"/>
        <line class="streak" x1="230" y1="130" x2="230" y2="490"
              style="animation-delay:-.2s"/>
      </g>

      <!-- car: public-domain top-view drawing from openclipart.org
           ("car topview", CC0), recoloured to match the display theme.
           車両：openclipart.org のパブリックドメイン上面図（配色変更済み） -->
      <g id="vcar" class="ok">
        <g transform="translate(86.35,160) scale(0.35471)">
          <defs>
            <linearGradient id="gCarPaint">
              <stop stop-color="#e7edf5" offset="0"/>
              <stop stop-color="#8b9cb3" offset="1"/>
            </linearGradient>
            <radialGradient id="gCar1" href="#gCarPaint"
              gradientUnits="userSpaceOnUse" cy="819.9" cx="378.79"
              gradientTransform="matrix(1.1693 0 0 .93776 -273.1 -147.55)"
              r="115.39"/>
            <radialGradient id="gCar2" href="#gCarPaint"
              gradientUnits="userSpaceOnUse" cy="598.63" cx="294.43"
              gradientTransform="matrix(1.5041 0 0 .74199 -264.04 -146.71)"
              r="91.234"/>
            <radialGradient id="gCar3" href="#gCarPaint"
              gradientUnits="userSpaceOnUse" cy="153.65" cx="897.24"
              gradientTransform="matrix(-.39086 0 0 3.6334 703.38 -167.47)"
              r="15.849"/>
            <radialGradient id="gCar4" href="#gCarPaint"
              gradientUnits="userSpaceOnUse" cy="153.65" cx="897.24"
              gradientTransform="matrix(.39086 0 0 3.6334 -343 -162.47)"
              r="15.849"/>
            <radialGradient id="gCar5" href="#gCarPaint"
              gradientUnits="userSpaceOnUse" cy="319.37" cx="657.76"
              gradientTransform="matrix(.65803 0 0 1.5197 -252.14 -111.82)"
              r="1269.3"/>
          </defs>
          <rect rx="8.6" ry="8.6" height="78.7" width="27.8" y="623"
                x="16.3" fill="#151a23"/>
          <rect rx="8.6" ry="8.6" height="78.7" width="27.8" y="613"
                x="311.3" fill="#151a23"/>
          <rect id="vwfr" rx="8.6" ry="8.6" height="78.7" width="27.8"
                y="98" x="318.8" fill="#151a23"/>
          <rect id="vwfl" rx="8.6" ry="8.6" height="78.7" width="27.8"
                y="101.1" x="8.6" fill="#151a23"/>
          <path d="m178.73 782.98c-113.07 2.362-130.4-17.92-147.11-21.261-16.705-38.776-19.877-365.73-9.855-392.46 7.493-60.54-4.936-70.565-8.687-143.53-7.14-85.213 9.815-37.829-4.439-124.48 21.658-90.216-19.136-92.053 168.52-100.63 172.21 2.401 147.96 10.415 169.61 100.63-14.254 86.652 2.701 39.268-4.439 124.48-3.751 72.961-16.18 82.986-8.687 143.53 10.022 26.727 6.85 353.68-9.855 392.46-26.153 15.153-95.459 21.261-145.07 21.261z"
                fill-rule="evenodd" stroke="#10151d" stroke-width="2"
                fill="url(#gCar5)"/>
          <path d="m41.537 281.88s21.534 82.442 21.534 82.442 0 154.58-3.0758 157.15c-3.0771 2.5759-27.686 46.372-27.686 46.372s6.1517-280.81 9.2288-285.97z"
                fill-rule="evenodd" fill="url(#gCar4)"/>
          <path d="m318.84 276.88s-21.534 82.442-21.534 82.442 0 154.58 3.0758 157.15c3.0771 2.5759 27.686 46.372 27.686 46.372s-6.1517-280.81-9.2288-285.97z"
                fill-rule="evenodd" fill="url(#gCar3)"/>
          <path d="m37.198 44.521c-11.667 18.667-10.816 196.22 7.851 210.22 18.03-14.851 122.48-28.646 142.34-27.364 20.288-1.492 99.694 8.055 124.09 22.697 6.577 2.13 19.727-205.55 1.059-219.55-58.34-16.344-252.01-16.344-275.34 13.99z"
                stroke-opacity=".45" fill-rule="evenodd" stroke="#0c1118"
                stroke-width="1.5" fill="#d5dde8"/>
          <path d="m41.764 257.39s65.53-26.897 138.64-27.585c65.833-0.64088 95.565 9.623 135.61 25.019-4.8534 48.756-9.7078 94.943-21.843 110.34-111.64-28.23-118.92-28.23-230.56 0-2.43-15.39-24.273-105.21-21.846-107.77z"
                fill-rule="evenodd" fill="url(#gCar2)"/>
          <path d="m45.51 260.2s63.783-24.762 134.95-25.395c64.078-0.59 93.017 8.859 132 23.033-4.724 44.885-9.449 87.405-21.261 101.58-108.67-25.986-115.75-25.985-224.42 0.001-2.362-14.174-23.623-96.856-21.261-99.218z"
                fill-rule="evenodd" fill="#060a12" fill-opacity=".78"
                stroke="#0c1118" stroke-width="1.5"/>
          <path d="m75.697 531.43c-12.369 59.368-22.263 173.16-22.263 173.16 19.79 17.316 121.21 24.737 123.69 24.737 7.4212 0 108.84-7.4212 126.16-32.158 0-14.842-9.8956-121.21-19.79-168.21-86.579 12.368-202.84 7.4212-207.79 2.4734z"
                fill-rule="evenodd" fill="url(#gCar1)"/>
          <path d="m80.945 536.59c-11.812 56.695-21.261 165.36-21.261 165.36 18.899 16.536 115.75 23.623 118.12 23.623 7.087 0 103.94-7.087 120.48-30.71 0-14.174-9.45-115.75-18.899-160.64-82.681 11.811-193.71 7.087-198.44 2.362z"
                fill-rule="evenodd" fill="#060a12" fill-opacity=".82"
                stroke="#0c1118" stroke-width="1.5"/>
          <path d="m321.9 279.09l28.348 2.362s14.174 14.174 4.725 21.261c-9.45 7.087-33.073-2.362-33.073-2.362v-21.261z"
                stroke-opacity=".55" fill-rule="evenodd" stroke="#0c1118"
                stroke-width="1.5" fill="#d5dde8"/>
          <path d="m36.946 283.82l-28.348 2.362s-14.174 14.174-4.725 21.261c9.45 7.087 33.073-2.362 33.073-2.362v-21.261z"
                stroke-opacity=".55" fill-rule="evenodd" stroke="#0c1118"
                stroke-width="1.5" fill="#d5dde8"/>
          <path d="m52.582 17.025c-9.023 1.573-19.32 18.998-14.584 21.902 11.281-5.689 33.327-13.506 54.984-15.684 4.286-2.3 10.138-9.356 10.358-10.929-14.886-0.847-40.915 2.775-50.758 4.711z"
                stroke-opacity=".25" fill-rule="evenodd" fill-opacity=".6"
                stroke="#0c1118" stroke-width="1.2" fill="#26374d"/>
          <path d="m298.7 12.115c9.0231 1.5731 19.32 14.633 14.584 17.537-11.28-5.6886-33.327-7.5051-54.984-9.6837-4.2855-2.2999-10.134-10.992-10.36-12.565 14.888-0.84719 40.917 2.7754 50.76 4.7115z"
                stroke-opacity=".25" fill-rule="evenodd" fill-opacity=".6"
                stroke="#0c1118" stroke-width="1.2" fill="#26374d"/>
          <path d="m112.29 10.38c37.098-6.547 99.837-6.002 126.57-2.729"
                stroke-opacity=".4" stroke="#0c1118" stroke-width="1.5" fill="none"/>
          <path d="m112.29 13.108c37.098-6.547 99.837-6.002 126.57-2.729"
                stroke-opacity=".4" stroke="#0c1118" stroke-width="1.5" fill="none"/>
          <path d="m112.29 16.381c37.098-6.547 99.837-6.002 126.57-2.729"
                stroke-opacity=".4" stroke="#0c1118" stroke-width="1.5" fill="none"/>
          <path d="m112.29 20.2c37.098-6.001 98.2-4.911 124.39-3.275"
                stroke-opacity=".4" stroke="#0c1118" stroke-width="1.5" fill="none"/>
          <path d="m43.148 295.63s16.536 75.595 16.536 75.595 0 141.74-2.362 144.1c-2.363 2.362-21.261 42.521-21.261 42.521s4.724-257.49 7.087-262.22z"
                fill-rule="evenodd" fill="#060a12" fill-opacity=".7"
                stroke="#0c1118" stroke-width="1.5"/>
          <path d="m36.35 310.53c-2.333 25.667-14.991 443.45 4.666 443.34 69.856 26.467 221.62 23.584 278.52-7.0003 17.53-0.0711 6.999-417.68 4.666-443.34"
                stroke="#10151d" stroke-width="1.5" fill="none"/>
          <path d="m317.18 290.91s-16.536 75.595-16.536 75.595 0 141.74 2.362 144.1c2.363 2.362 21.261 42.521 21.261 42.521s-4.724-257.49-7.087-262.22z"
                fill-rule="evenodd" fill="#060a12" fill-opacity=".75"
                stroke="#0c1118" stroke-width="1.5"/>
        </g>
        <rect class="headlight" x="107" y="164" width="24" height="6" rx="3"/>
        <rect class="headlight" x="169" y="164" width="24" height="6" rx="3"/>
        <rect class="taillight" id="vtl-l" x="110" y="429" width="26"
              height="6" rx="3"/>
        <rect class="taillight" id="vtl-r" x="164" y="429" width="26"
              height="6" rx="3"/>
        <rect class="revlight" id="vrev" x="143" y="429" width="14"
              height="5" rx="2.5"/>
        <!-- blinker lamps / ウインカーランプ -->
        <g id="vsig-l">
          <rect x="99" y="170" width="7" height="6" rx="2"/>
          <rect x="100" y="427" width="7" height="6" rx="2"/>
        </g>
        <g id="vsig-r">
          <rect x="194" y="170" width="7" height="6" rx="2"/>
          <rect x="193" y="427" width="7" height="6" rx="2"/>
        </g>
      </g>

      <!-- attached sensor markers (JS, from live CARLA actor list)
           取付センサーマーカー（JS・CARLAの実アクター一覧から生成） -->
      <g id="vsens"></g>

      <!-- collision impact markers (JS) -->
      <g id="vhits"></g>
      </g><!-- /vscale -->

      <!-- park-distance margin bands (modern-EV style): inner red lights
           at <0.9 m, amber <1.8 m, green <3.0 m clearance
           EV風パーキング距離バンド：余裕 <0.9m で赤、<1.8m で黄、<3.0m で緑 -->
      <g id="vpdc">
        <path id="pdc-f-0" class="pdc" style="stroke:#d03b3b; color:#d03b3b"
              d="M130.1,225.3 A26,26 0 0 1 169.9,225.3"/>
        <path id="pdc-f-1" class="pdc" style="stroke:#fab219; color:#fab219"
              d="M120.9,217.6 A38,38 0 0 1 179.1,217.6"/>
        <path id="pdc-f-2" class="pdc" style="stroke:#0ca30c; color:#0ca30c"
              d="M111.7,209.9 A50,50 0 0 1 188.3,209.9"/>
        <path id="pdc-b-0" class="pdc" style="stroke:#d03b3b; color:#d03b3b"
              d="M169.9,374.7 A26,26 0 0 1 130.1,374.7"/>
        <path id="pdc-b-1" class="pdc" style="stroke:#fab219; color:#fab219"
              d="M179.1,382.4 A38,38 0 0 1 120.9,382.4"/>
        <path id="pdc-b-2" class="pdc" style="stroke:#0ca30c; color:#0ca30c"
              d="M188.3,390.1 A50,50 0 0 1 111.7,390.1"/>
        <line id="pdc-l-0" class="pdc" style="stroke:#d03b3b; color:#d03b3b"
              x1="116" y1="272" x2="116" y2="328"/>
        <line id="pdc-l-1" class="pdc" style="stroke:#fab219; color:#fab219"
              x1="105" y1="272" x2="105" y2="328"/>
        <line id="pdc-l-2" class="pdc" style="stroke:#0ca30c; color:#0ca30c"
              x1="94" y1="272" x2="94" y2="328"/>
        <line id="pdc-r-0" class="pdc" style="stroke:#d03b3b; color:#d03b3b"
              x1="184" y1="272" x2="184" y2="328"/>
        <line id="pdc-r-1" class="pdc" style="stroke:#fab219; color:#fab219"
              x1="195" y1="272" x2="195" y2="328"/>
        <line id="pdc-r-2" class="pdc" style="stroke:#0ca30c; color:#0ca30c"
              x1="206" y1="272" x2="206" y2="328"/>
        <text id="pdct-f" class="mtext" x="150" y="200"
              style="display:none"></text>
        <text id="pdct-b" class="mtext" x="150" y="408"
              style="display:none"></text>
        <text id="pdct-l" class="mtext" x="100" y="348"
              style="display:none"></text>
        <text id="pdct-r" class="mtext" x="200" y="348"
              style="display:none"></text>
      </g>

      <!-- lane-change direction animation while a signal is on
           ウインカー点灯中の車線変更方向アニメーション -->
      <g id="vsigarr-l">
        <path class="sigchev" d="M96,284 L80,300 L96,316"/>
        <path class="sigchev" d="M78,284 L62,300 L78,316"/>
        <path class="sigchev" d="M60,284 L44,300 L60,316"/>
      </g>
      <g id="vsigarr-r">
        <path class="sigchev" d="M204,284 L220,300 L204,316"/>
        <path class="sigchev" d="M222,284 L238,300 L222,316"/>
        <path class="sigchev" d="M240,284 L256,300 L240,316"/>
      </g>

      <!-- ultrasonic front-distance arc / 超音波の前方距離アーク -->
      <path id="vultra" style="display:none"/>

      <!-- nearby cars / bikes / pedestrians (JS) / 周辺の車・二輪・歩行者 -->
      <g id="vnear"></g>

      <!-- sensor leader labels, unscaled (JS) / センサー名ラベル（非縮小） -->
      <g id="vsenslabels"></g>
    </svg>
    </div>
  </div>

  <div class="vcol">
    <div class="card">
      <h3><span data-i18n="veh_nearby">Nearby Objects</span>
          <span class="pill" id="vh-nearn">0</span></h3>
      <div id="vh-near" class="events" style="max-height:250px"></div>
      <div class="legend" data-i18n="veh_range">objects within 50 m ·
        sorted by distance</div>
    </div>
    <div class="card">
      <h3 data-i18n="veh_last">Last Impact</h3>
      <div class="vzone">
        <div class="zn" id="vh-zone">–</div>
        <div class="za" id="vh-zage" data-i18n="veh_none">no impacts yet</div>
      </div>
    </div>
    <div class="card">
      <h3 data-i18n="veh_impacts">Impact Log</h3>
      <div id="vh-hits" class="events" style="max-height:340px"></div>
    </div>
  </div>

</div>
</main>

<main id="view-drone" hidden>
<div class="chips">
  <span class="chip"><span data-i18n="dr_chip_run">Delivery sim</span><b id="dr-run">-</b></span>
  <span class="chip"><span data-i18n="dr_chip_busy">Busy pods</span><b id="dr-busy">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_drones">Drones</span><b id="dr-n">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_flying">Flying</span><b id="dr-fly">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_vans">In vans</span><b id="dr-van">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_station">At station</span><b id="dr-st">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_down">Down</span><b id="dr-down">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_deliv">Delivered</span><b id="dr-deliv">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_fail">Failed</span><b id="dr-fail">0</b></span>
  <span class="chip"><span data-i18n="dr_chip_wind">Wind</span><b id="dr-wind">-</b></span>
  <span class="chip"><span data-i18n="chip_simt">Sim time (s)</span><b id="dr-t">0</b></span>
  <span class="chip"><span data-i18n="pd_chip_shell">Shell</span><b id="pd-shell">-</b></span>
  <span class="chip"><span data-i18n="pd_chip_on">Showcase</span><b id="pd-on">—</b></span>
  <span class="chip"><span data-i18n="pd_chip_scen">Scenario</span><b id="pd-scen">—</b></span>
  <span class="chip"><span data-i18n="pd_chip_step">Step</span><b id="pd-step">—</b></span>
  <b id="pd-run" hidden></b>
</div>
<div class="dgrid">

  <div class="card">
    <h3><span data-i18n="dr_map">City map</span>
      <span><span class="pill" id="dr-map-name">-</span>
      <button class="rbtn" id="dmap-fit" data-i18n="dr_fit">Fit</button></span></h3>
    <div class="dmapwrap"><canvas id="dmap"></canvas><div id="dmap-tip"></div></div>
    <div class="dlegend">
      <span><i style="background:#3987e5"></i><span data-i18n="dr_lg_pod">delivery pod</span></span>
      <span><i style="background:#8a96aa"></i><span data-i18n="dr_lg_car">other car</span></span>
      <span><i class="c" style="background:#199e70"></i><span data-i18n="dr_lg_drone">drone (colour = battery)</span></span>
      <span><i style="background:#78ff3c"></i><span data-i18n="dr_lg_station">charge station</span></span>
      <span><i style="background:#ffd228"></i><span data-i18n="dr_lg_ev">EV charger (pods)</span></span>
      <span><i class="c" style="background:#ff8c00"></i><span data-i18n="dr_lg_f2f">face-to-face order</span></span>
      <span><i class="c" style="background:#00a0ff"></i><span data-i18n="dr_lg_dr">drone order</span></span>
      <span><i style="background:#dc00dc"></i><span data-i18n="dr_lg_hub">drone hub</span></span>
      <span><i class="c" style="background:#0ca30c"></i><span data-i18n="dr_lg_done">delivered</span></span>
      <span><i class="c" style="background:#d03b3b"></i><span data-i18n="dr_lg_fail">failed / crash</span></span>
    </div>
    <div class="legend" style="margin-top:4px" data-i18n="dr_map_hint">wheel: zoom · drag: pan · double-click: fit</div>
  </div>

  <div class="dside">
    <div class="card">
      <h3 data-i18n="dr_stats">Deliveries &amp; failures</h3>
      <div id="dr-stats"></div>
    </div>
  </div>

  <div class="card dwide">
    <h3><span data-i18n="dr_drones">Drones</span><span class="pill" id="dr-n2">0</span></h3>
    <div style="overflow-x:auto"><table class="dt" id="dr-table"></table></div>
  </div>

  <div class="dpair">
    <div class="card">
      <h3 data-i18n="dr_vans">Delivery pods</h3>
      <div style="overflow-x:auto"><table class="dt" id="dr-vans"></table></div>
    </div>
    <div class="card">
      <h3 data-i18n="dr_events">Drone events</h3>
      <div class="events" id="dr-events" style="max-height:260px"></div>
    </div>
  </div>


  <h2 class="sect dwide" data-i18n="sect_pod">LastMile pod</h2>
  <div class="card dwide" id="pd-none" hidden><h3 data-i18n="pd_title">LastMile pod</h3>
    <div class="dempty" data-i18n="pd_none">No LastMile pod in the delivery sim (it runs the earlier pod, or isn't running).</div></div>
  <div id="pd-body" style="display:contents">
  <div class="card dwide">
    <h3><span data-i18n="pd_anim">Animation</span>
      <span class="dbtns" style="margin:0">
        <select id="pd-van" class="pd-sel"></select>
        <button class="dbtn gust" id="pd-take" data-i18n="pd_take">Take over</button>
        <button class="dbtn" id="pd-release" data-i18n="pd_release" hidden>Release</button>
      </span></h3>
    <div class="pd-transport">
      <button class="dbtn" id="pd-play" data-playing="1" aria-label="Play / pause"><svg class="ico" viewBox="0 0 24 24" aria-hidden="true"><path d="M7 4v16l13-8z" fill="currentColor" stroke="none"/></svg></button>
      <div class="pd-scrubwrap"><div id="pd-ticks"></div>
        <input type="range" id="pd-scrub" min="0" max="121" step="0.05" value="0" aria-label="Animation time (s)"></div>
      <span id="pd-time" class="pd-time">—</span>
      <select id="pd-speed" class="pd-sel"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
    </div>
    <div class="legend"><span class="it"><i style="background:#3987e5"></i><span data-i18n="pd_scen_drone">Drone delivery</span></span>
      <span class="it"><i style="background:#ff8c00"></i><span data-i18n="pd_scen_direct">Direct delivery</span></span></div>
    <div class="dmsg" id="pd-msg"></div>
    <div class="pd-steps"><div><h4 data-i18n="pd_scen_drone">Drone delivery</h4><div id="pd-steps-drone"></div></div>
      <div><h4 data-i18n="pd_scen_direct">Direct delivery</h4><div id="pd-steps-direct"></div></div></div>
  </div>
  <div class="card dwide">
    <h3 data-i18n="pd_deck_title">Drones on deck and battery magazine</h3>
    <div style="overflow-x:auto"><table class="dt" id="pd-deck"></table></div>
  </div>
  </div>
</div>
</main>

<main id="view-settings" hidden>
<div class="setnav" role="tablist" aria-label="Settings sections">
  <button id="set-btn-driving" role="tab" aria-controls="set-driving" aria-selected="true" data-sec="driving" class="active" data-i18n="set_driving">Driving</button>
  <button id="set-btn-ffb" role="tab" aria-controls="set-ffb" data-sec="ffb" data-i18n="set_ffb">Force feedback</button>
  <button id="set-btn-delivery" role="tab" aria-controls="set-delivery" data-sec="delivery" data-i18n="set_delivery">Delivery sim</button>
  <span class="setnote" data-i18n="set_note">Every setting applies live and is saved for the next run.</span>
</div>
<section id="set-driving">
<div class="fgrid">
<div class="card">
    <h3><span data-i18n="card_tuning">Pedal Tuning</span>
        <button id="tune-reset" class="rbtn" data-i18n="tune_reset">reset</button></h3>
    <div class="inlbl"><span data-i18n="tune_bmax">Brake strength</span><b id="v-bmax">100%</b></div>
    <input type="range" id="r-bmax" min="0.1" max="1" step="0.05" value="1">
    <div class="inlbl"><span data-i18n="tune_bcurve">Brake response curve</span><b id="v-bcurve">2.2</b></div>
    <input type="range" id="r-bcurve" min="0.5" max="3" step="0.1" value="2.2">
    <div class="inlbl"><span data-i18n="tune_tmax">Throttle strength</span><b id="v-tmax">80%</b></div>
    <input type="range" id="r-tmax" min="0.1" max="1" step="0.05" value="0.8">
    <div class="inlbl"><span data-i18n="tune_tcurve">Throttle response curve</span><b id="v-tcurve">1.8</b></div>
    <input type="range" id="r-tcurve" min="0.5" max="3" step="0.1" value="1.8">
    <canvas id="tunecurve" style="height:96px; margin-top:6px"></canvas>
    <div class="legend">
      <span class="it"><i style="background:#199e70"></i><span data-i18n="lbl_throttle">Throttle</span></span>
      <span class="it"><i style="background:#e66767"></i><span data-i18n="lbl_brake">Brake</span></span>
      <span data-i18n="tune_hint">pedal press → output · higher curve = gentler at light press</span>
    </div>
  </div>
<div class="card">
    <h3><span data-i18n="vt_card">Vehicle Performance</span>
      <span><button id="vt-stock" class="rbtn" data-i18n="vt_stock">stock</button>
      <button id="vt-reset" class="rbtn" data-i18n="tune_reset">reset</button></span></h3>
    <div id="vt-rows"></div>
    <div style="overflow-x:auto; margin-top:8px"><table class="dt" id="vt-cars"></table></div>
    <div class="legend" data-i18n="vt_hint">Applied ~1 s after you stop moving a slider; the car keeps its speed (a small dip).</div>
  </div>
</div>
</section>
<section id="set-ffb" hidden>
<div class="chips">
  <span class="chip"><span data-i18n="ff_chip_state">FFB</span><b id="ff-state">-</b></span>
  <span class="chip"><span data-i18n="chip_vehicle">Vehicle</span><b id="ff-hero">-</b></span>
  <span class="chip">km/h<b id="ff-kmh">-</b></span>
  <span class="chip"><span data-i18n="ff_chip_surface">Surface</span><b id="ff-surf">-</b></span>
  <span class="chip"><span data-i18n="ff_chip_sat">Front grip used</span><b id="ff-sat">-</b></span>
</div>
<div class="fgrid">
  <div class="card fwide">
    <h3><span data-i18n="ff_card_main">Force feedback</span>
      <span class="dbtns" style="margin:0" id="ff-presets"></span></h3>
    <div class="ffsave">
      <input id="ff-pname" maxlength="40" data-i18n-ph="ff_pname_ph" placeholder="preset name">
      <button class="dbtn" id="ff-psave" data-i18n="ff_psave">Save preset</button>
      <span class="dmsg" id="ff-pmsg" style="margin:0"></span>
    </div>
    <div class="dpair" style="grid-column:auto">
      <div>
        <canvas id="fcurve"></canvas>
        <div class="legend">
          <span class="it"><i style="background:#3987e5"></i><span data-i18n="ff_lg_now">centring torque at current speed</span></span>
          <span class="it"><i style="background:#69758a"></i><span data-i18n="ff_lg_still">at standstill</span></span>
          <span class="it"><i style="background:#c98500"></i><span data-i18n="ff_lg_wheel">wheel position</span></span>
        </div>
      </div>
      <div>
        <div class="inlbl"><span data-i18n="ff_m_wheel">Wheel position</span><b id="ff-v-wheel">-</b></div>
        <div class="fmeter"><s></s><i id="ff-m-wheel"></i></div>
        <div class="inlbl"><span data-i18n="ff_m_out">Motor force</span><b id="ff-v-out">-</b></div>
        <div class="fmeter"><s></s><i id="ff-m-out"></i></div>
        <div class="inlbl"><span data-i18n="ff_m_damper">Damper</span><b id="ff-v-damper">-</b></div>
        <div class="fmeter"><i id="ff-m-damper" style="left:0"></i></div>
        <div class="inlbl"><span data-i18n="ff_m_friction">Friction</span><b id="ff-v-friction">-</b></div>
        <div class="fmeter"><i id="ff-m-friction" style="left:0"></i></div>
        <div class="dmsg" id="ff-msg" data-i18n="ff_hint">Changes reach the wheel within half a second. Settings are saved in ffb_tuning.json.</div>
      </div>
    </div>
  </div>
  <div id="ff-groups" style="display:contents"></div>
</div>
</section>
<section id="set-delivery" hidden>
<div class="fgrid">
<div class="card">
      <h3><span data-i18n="dr_wind">Wind</span><span class="pill" id="dr-gust">-</span></h3>
      <canvas id="dwind"></canvas>
      <div class="kv"><span data-i18n="dr_wind_now">Now</span>
        <span class="v" id="dw-now">-</span></div>
      <div class="kv"><span data-i18n="dr_wind_base">Base wind</span>
        <span class="v" id="dw-base">-</span></div>
      <div class="kv"><span data-i18n="dr_wind_rate">Random gusts</span>
        <span class="v" id="dw-rate">-</span></div>
      <div class="kv"><span data-i18n="dr_wind_hold">Take-off hold above</span>
        <span class="v" id="dw-hold">-</span></div>
      <div class="dctl">
        <span data-i18n="dr_ctl_speed">Gust speed</span>
        <input type="range" id="dc-speed" min="4" max="25" step="0.5" value="14"><b id="dc-speed-v">14</b>
        <span data-i18n="dr_ctl_dir">Direction</span>
        <input type="range" id="dc-dir" min="0" max="359" step="1" value="0"><b id="dc-dir-v">0°</b>
        <span data-i18n="dr_ctl_dur">Duration</span>
        <input type="range" id="dc-dur" min="2" max="20" step="1" value="6"><b id="dc-dur-v">6 s</b>
      </div>
      <div class="dbtns">
        <button class="dbtn gust" id="dc-gust" data-i18n="dr_btn_gust">Gust now</button>
        <button class="dbtn" id="dc-follow" data-i18n="dr_btn_follow">Use wind direction</button>
      </div>
      <div class="dctl">
        <span data-i18n="dr_ctl_base">Base wind</span>
        <input type="range" id="dc-base" min="0" max="15" step="0.5" value="3"><b id="dc-base-v">3</b>
        <span data-i18n="dr_ctl_rate">Gusts / min</span>
        <input type="range" id="dc-rate" min="0" max="6" step="0.25" value="0.5"><b id="dc-rate-v">0.5</b>
      </div>
      <div class="dbtns">
        <button class="dbtn" id="dc-setbase" data-i18n="dr_btn_base">Set base wind</button>
        <button class="dbtn" id="dc-setrate" data-i18n="dr_btn_rate">Set gust rate</button>
        <button class="dbtn" id="dc-calm" data-i18n="dr_btn_calm">Calm</button>
      </div>
      <div class="dmsg" id="dc-msg"></div>
      <div class="legend" style="margin-top:2px" data-i18n="dr_wind_hint">click the compass to pick a direction · drones hold against 11 m/s</div>
    </div>
<div class="card">
    <h3 data-i18n="pd_view">Shell view</h3>
    <div class="dbtns">
      <button class="dbtn" data-shell="solid" data-i18n="pd_shell_solid">Solid</button>
      <button class="dbtn" data-shell="cutaway" data-i18n="pd_shell_cutaway">Cutaway</button>
      <button class="dbtn" data-shell="xray" data-i18n="pd_shell_xray">X-ray</button>
      <button class="dbtn" data-shell="frame" data-i18n="pd_shell_frame">Frame</button>
    </div>
    <div class="legend" style="margin-top:8px" data-i18n="pd_view_hint">For every LastMile pod. X in the CARLA window cycles it too.</div>
  </div>
<div class="card">
    <h3 id="tt-title" data-i18n="tt_title">CARLA tooltips</h3>
    <div class="dbtns" role="group" aria-labelledby="tt-title">
      <button class="dbtn" data-tooltips="0" data-i18n="tt_off">Off</button>
      <button class="dbtn" data-tooltips="1" data-i18n="tt_on">On</button>
    </div>
    <div class="legend" style="margin-top:8px" data-i18n="tt_hint">Order and hub markers, station boxes, labels over drones and pods, and drone flight trails in the CARLA window.</div>
  </div>
</div>
</section>
</main>

<div id="tooltip"></div>

<script>
const $ = id => document.getElementById(id);
const fmt = (n, d=2) => (Number(n)||0).toFixed(d);
const clamp = (v,a,b) => Math.max(a, Math.min(b, v));

const C = {blue:'#3987e5', green:'#199e70', yellow:'#c98500', red:'#e66767',
           violet:'#9085e9', warn:'#fab219', crit:'#d03b3b', serious:'#ec835a',
           good:'#0ca30c', grid:'#222937', axis:'#39414f', mute:'#8b949e',
           ink:'#e6edf3'};

/* ------------------------------------------------------------------ i18n */
const I18N = {
 en: {
  nav_dash:'Drive', nav_stats:'Statistics', nav_veh:'Vehicle',
  nav_drone:'Delivery', nav_settings:'Settings', nav_ffb:'Force Feedback', nav_pod:'Pod',
  set_driving:'Driving', set_ffb:'Force feedback', set_delivery:'Delivery sim',
  set_note:'Every setting applies live and is saved for the next run.', sect_pod:'LastMile pod',
  pd_title:'LastMile pod', pd_none:"No LastMile pod in the delivery sim (it runs the earlier pod, or isn't running).",
  pd_chip_shell:'Shell', pd_chip_on:'Showcase', pd_chip_scen:'Scenario', pd_chip_step:'Step',
  pd_anim:'Animation', pd_take:'Take over', pd_release:'Release', pd_play:'Play', pd_pause:'Pause',
  pd_scen_drone:'Drone delivery', pd_scen_direct:'Direct delivery', pd_pod:'Pod', pd_busy:'busy',
  pd_hint:'Take a pod over to play the whole cycle on it (its deliveries pause until you release it). The camera locks onto it.',
  pd_take_first:'take a pod over first', pd_view:'Shell view',
  pd_view_hint:'For every LastMile pod. X in the CARLA window cycles it too.',
  tt_title:'CARLA tooltips', tt_off:'Off', tt_on:'On',
  tt_hint:'Order and hub markers, station boxes, labels over drones and pods, and drone flight trails in the CARLA window.',
  pd_shell_solid:'Solid', pd_shell_cutaway:'Cutaway', pd_shell_xray:'X-ray', pd_shell_frame:'Frame',
  pd_deck_title:'Drones on deck and battery magazine', pd_deck:'Deck (lift + bays)', pd_mag:'Magazine', pd_lift:'lift',
  vt_card:'Vehicle Performance', vt_stock:'stock',
  vt_hint:'Applied ~1 s after you stop moving a slider; the car keeps its speed (a small dip).',
  vt_car:'Vehicle', vt_brake:'Brake / wheel', vt_torque:'Peak torque', vt_top:'Top speed (est.)',
  vt_me:'my car', vt_wait:'waiting', vt_wait_tip:'Applied once the pod drives again (it is in a scripted move, or the delivery sim predates this feature)', vt_none:'no vehicles yet',
  ff_chip_state:'FFB', ff_chip_surface:'Surface', ff_chip_sat:'Front grip used',
  ff_card_main:'Force feedback', ff_lg_now:'centring torque at current speed',
  ff_lg_still:'at standstill', ff_lg_wheel:'wheel position',
  ff_m_wheel:'Wheel position', ff_m_out:'Motor force', ff_m_damper:'Damper',
  ff_m_friction:'Friction', ff_running:'running', ff_offline:'not running',
  ff_stale:'idle', ff_reset:'reset to default',
  ff_hint:'Changes reach the wheel within half a second. Settings are saved in ffb_tuning.json.',
  ff_saved:'saved', ff_p_forza_horizon:'Forza Horizon', ff_p_previous:'Previous',
  ff_p_light:'Light', ff_p_heavy:'Heavy',
  ff_psave:'Save preset', ff_pname_ph:'preset name', ff_psaved:'saved as',
  ff_pdeleted:'deleted', ff_pdel_q:'Delete preset', ff_pname_need:'type a name first',
  ff_preplace_q:'Replace the saved preset',
  dr_chip_run:'Delivery sim', dr_chip_drones:'Drones', dr_chip_flying:'Flying',
  dr_chip_vans:'In pods', dr_chip_station:'At station', dr_chip_down:'Down',
  dr_chip_deliv:'Delivered', dr_chip_fail:'Failed', dr_chip_wind:'Wind',
  dr_map:'City map', dr_fit:'Fit', dr_map_hint:'wheel: zoom · drag: pan · double-click: fit · hover for details',
  dr_lg_pod:'delivery pod', dr_lg_car:'other car', dr_lg_drone:'drone (fill = battery, ring = status)',
  dr_lg_station:'charge station', dr_lg_f2f:'face-to-face order', dr_lg_dr:'drone order',
  dr_lg_hub:'drone hub', dr_lg_done:'delivered', dr_lg_fail:'failed / crashed',
  dr_wind:'Wind', dr_wind_now:'Now', dr_wind_base:'Base wind', dr_wind_rate:'Random gusts',
  dr_ctl_speed:'Gust speed (m/s)', dr_ctl_dir:'Direction', dr_ctl_dur:'Duration',
  dr_ctl_base:'Base wind (m/s)', dr_ctl_rate:'Gusts / min',
  dr_btn_gust:'Gust now', dr_btn_follow:'Use wind direction', dr_btn_base:'Set base wind',
  dr_btn_rate:'Set gust rate', dr_btn_calm:'Calm',
  dr_wind_hint:'click the compass to pick a direction · red ring = 11 m/s, the most a drone can hold against',
  dr_stats:'Deliveries & failures', dr_drones:'Drones', dr_vans:'Delivery pods', dr_events:'Drone events',
  dr_nomap:'Waiting for delivery_sim.py… (start it with run_pipeline.py --delivery N)',
  dr_waiting:'No delivery simulation running.', dr_noevents:'No events yet.',
  dr_none:'not running', dr_live:'live', dr_stopped:'stopped', dr_steady:'steady', dr_sent:'sent',
  dr_grounded:'GROUNDED', dr_wind_hold:'Take-off hold above',
  dr_lg_ev:'EV charger (pods)', dr_chip_busy:'Busy pods', dr_th_podbatt:'Pod battery',
  dr_s_evch:'Pod EV charges', dr_s_podkm:'Pod distance', dr_s_strand:'Pods stranded',
  dr_nodrone:'no drone',
  dr_th_drone:'Drone', dr_th_status:'Status', dr_th_batt:'Battery', dr_th_van:'Pod', dr_th_order:'Order',
  dr_th_alt:'Altitude', dr_th_drift:'Wind drift', dr_th_trips:'Trips', dr_th_crash:'Crashes',
  dr_th_note:'Note', dr_th_state:'State', dr_th_pend:'Pending',
  dr_s_deliv:'Delivered', dr_s_bydrone:'by drone', dr_s_fail:'Failed', dr_s_trips:'Drone trips',
  dr_s_crash:'Crashes', dr_s_swaps:'Standby swaps', dr_s_charge:'Station charges', dr_s_repair:'Repairs',
  dr_s_gusts:'Gusts', dr_s_km:'Distance flown', dr_s_queued:'Orders waiting',
  st_docked:'in pod', st_reloading:'loading parcel', st_delivering:'delivering', st_to_station:'to charge',
  st_to_van:'to pod', st_charging:'charging', st_standby:'standby', st_falling:'falling', st_crashed:'crashed',
  ost_queued:'waiting', ost_assigned:'assigned', ost_in_flight:'in flight', ost_delivered:'delivered',
  ost_failed:'failed',
  fr_collision:'collision', fr_blown_off_course:'blown off course', fr_battery_depleted:'battery died',
  fr_motor_failure:'motor failure', fr_parcel_lost:'parcel torn off', fr_no_drone:'no drone available',
  fr_no_safe_drop:'no safe place to drop',
  veh_status:'Vehicle Status', veh_sensors:'Sensors',
  veh_diagram:'Sensor & Impact Map', veh_impacts:'Impact Log',
  veh_last:'Last Impact', veh_ok:'ALL SYSTEMS OK', veh_warn:'CAUTION',
  veh_crash:'COLLISION', veh_none:'no impacts yet', veh_ago:'s ago',
  veh_impulse:'impulse', lbl_speed:'Speed',
  veh_sn_hint:'click a sensor to show its mounting position',
  veh_nearby:'Nearby Objects',
  veh_range:'objects within 50 m · sorted by distance',
  veh_none_near:'nothing in sensor range',
  veh_no_sensors:'no sensors attached',
  kind_car:'CAR', kind_truck:'TRUCK', kind_bike:'BIKE', kind_walker:'PED',
  sn_ultra:'Ultrasonic', sn_lidar:'LiDAR', sn_semlidar:'Sem. LiDAR',
  sn_radar:'Radar', sn_cam:'Camera', sn_other:'Sensor', sn_active:'active',
  lg_bike:'bike / motorcycle', lg_walker:'pedestrian',
  sig_hint:'turn signal \u00b7 wheel button 11 (L) / 10 (R)',
  ev_lane_change:'lane change',
  btn_shutdown:'Shutdown',
  shutdown_confirm:'Shut down the whole simulation? This stops CARLA, traffic, the driving window and this dashboard, and frees their ports.',
  shutdown_done:'Simulation is shutting down \u2014 all ports will be released. You can close this tab.',
  zone_front:'FRONT', zone_rear:'REAR', zone_left:'LEFT',
  zone_right:'RIGHT', zone_front_left:'FRONT-LEFT',
  zone_front_right:'FRONT-RIGHT', zone_rear_left:'REAR-LEFT',
  zone_rear_right:'REAR-RIGHT', zone_center:'CENTER',
  sn_gnss:'GPS / GNSS', sn_imu:'IMU', sn_coll:'Impact sensor (airbag)',
  sn_lane:'Lane camera (LDW)',
  st_connecting:'connecting…', st_conn_hero:'connected · hero found',
  st_conn_wait:'connected · waiting for hero', st_disconn:'disconnected',
  chip_map:'Map', chip_vehicle:'Vehicle', chip_mass:'Mass (kg)',
  chip_nearby:'Nearby', chip_simt:'Sim time (s)', dl_csv:'CSV log',
  card_speed:'Speed', lbl_limit:'limit', lbl_avg:'avg',
  lbl_gear:'Gear', lbl_heading:'Heading',
  card_inputs:'Driver Inputs', lbl_throttle:'Throttle', lbl_brake:'Brake',
  lbl_steer:'Steer',
  card_tuning:'Pedal Tuning', tune_reset:'reset',
  tune_bmax:'Brake strength', tune_bcurve:'Brake response curve',
  tune_tmax:'Throttle strength', tune_tcurve:'Throttle response curve',
  tune_hint:'pedal press → output · higher curve = gentler at light press',
  al_lane:'Lane departure!', al_crash:'Collision!',
  al_redlight:'Red light violation!', al_hardbrake:'Hard braking',
  al_fcw:'BRAKE! Vehicle ahead', al_speeding:'Speeding',
  al_test:'Alert test',
  card_safety:'Safety / ADAS', lbl_fdist:'Front distance',
  lbl_hardbrakes:'Hard brakes', lbl_crashes:'Crashes',
  lbl_tl:'Traffic light', lbl_stability:'Stability',
  card_perf:'Driver Performance', lbl_sdlp:'SDLP (lane keeping)',
  lbl_srr:'Steer reversals', lbl_jerkrms:'Jerk RMS (comfort)',
  lbl_laneinv:'Lane invasions', lbl_rlv:'Red-light violations',
  lbl_comp:'Speed compliance',
  card_lanepos:'Lane Position (SDLP window)',
  lg_laneoff:'offset from centre', lg_laneedge:'lane edges',
  card_events:'Driving Events', ev_none:'none yet',
  ev_collision:'collision', ev_lane:'lane', ev_hard_brake:'hard brake',
  ev_red_light:'red light',
  card_speedhist:'Speed History vs Limit', lg_speed:'speed', lg_limit:'limit',
  lg_avg:'avg',
  card_gg:'G-G Diagram', gg_hint:'X: lateral · Y: longitudinal · rings = 0.5 g',
  card_acc:'Acceleration', lg_lon:'lon', lg_lat:'lat', lg_jerk:'jerk',
  card_dyn:'Vehicle Dynamics', lbl_lon:'Lon accel', lbl_lat:'Lat accel',
  lbl_vert:'Vert accel', lbl_jerk:'Jerk', lbl_g:'G-force',
  lbl_yawrate:'Yaw rate', lbl_slip:'Slip angle', lbl_curv:'Curvature',
  card_yaw:'Yaw Rate & Steering', lg_yaw:'yaw °/s', lg_steer100:'steer ×100',
  card_lane:'Lane / Road', lbl_onroad:'On road', lbl_roadlane:'Road / Lane',
  lbl_lanew:'Lane width', lbl_laneoff:'Lane offset', lbl_junction:'Junction',
  yes:'yes', no:'no', offroad:'OFF ROAD',
  card_weather:'Weather', lbl_cloud:'Cloud', lbl_rain:'Rain', lbl_wet:'Wetness',
  lbl_fog:'Fog', lbl_wind:'Wind', lbl_sunalt:'Sun altitude',
  card_trip:'Trip', lbl_elapsed:'elapsed', lbl_avgspeed:'Avg speed',
  lbl_collpeak:'Collision peak',
  card_coll:'Collision Intensity (10 s)',
  coll_hint:'Normal impulse from collision sensor',
  card_map:'Situational Awareness (80 m)', lg_hero:'hero',
  lg_nearby:'nearby vehicle', lg_cone:'front cone',
  card_trail:'World Trail', trail_hint:'World X/Y path history',
  card_gnss:'GNSS / IMU', lbl_pos:'X / Y / Z', lbl_lat2:'Latitude',
  lbl_lon2:'Longitude', lbl_accel:'Accel x/y/z', lbl_gyro:'Gyro x/y/z',
  lbl_compass:'Compass',
  pill_ok:'OK', pill_over:'OVER',
  tl_None:'None', tl_Red:'Red', tl_Yellow:'Yellow', tl_Green:'Green',
  tile_crash:'Crashes', tile_lane:'Out of lane', tile_rlv:'Red lights run',
  tile_hb:'Hard brakes', tile_dist:'Distance', tile_dur:'Duration',
  tile_avg:'Avg speed', tile_comp:'Compliance',
  stats_session:'Current session', stats_history:'Session history',
  hist_hint:'crashes & out-of-lane per session (latest 20)',
  stats_sessions:'Saved sessions', stats_empty:'No saved sessions yet',
  th_time:'Time', th_map:'Map', th_dur:'Duration', th_dist:'Distance (km)',
  th_crash:'Crashes', th_lane:'Out of lane', th_rlv:'Red lights',
  th_hb:'Hard brakes', th_comp:'Compliance', badge_live:'LIVE',
 },
 ja: {
  nav_dash:'運転', nav_stats:'統計', nav_veh:'車両状態',
  nav_drone:'配送', nav_settings:'設定', nav_ffb:'フォースフィードバック', nav_pod:'ポッド',
  set_driving:'運転', set_ffb:'フォースフィードバック', set_delivery:'配送シミュ',
  set_note:'設定はすべて即時に反映され、次回も保持されます。', sect_pod:'LastMile ポッド',
  pd_title:'LastMile ポッド', pd_none:'配送シミュに LastMile ポッドがありません（以前のポッドを使用中、または停止中）。',
  pd_chip_shell:'外殻', pd_chip_on:'ショーケース', pd_chip_scen:'シナリオ', pd_chip_step:'手順',
  pd_anim:'アニメーション', pd_take:'乗っ取る', pd_release:'解放', pd_play:'再生', pd_pause:'一時停止',
  pd_scen_drone:'ドローン配達', pd_scen_direct:'直接配達', pd_pod:'ポッド', pd_busy:'作業中',
  pd_hint:'ポッドを乗っ取ると全手順を再生できます（解放するまで配達は一時停止）。カメラが追従します。',
  pd_take_first:'先にポッドを乗っ取ってください', pd_view:'外殻表示',
  pd_view_hint:'全 LastMile ポッドに適用。CARLA ウィンドウの X キーでも切替。',
  tt_title:'CARLA ツールチップ', tt_off:'オフ', tt_on:'オン',
  tt_hint:'CARLA ウィンドウ内の注文・ハブのマーカー、ステーションの枠、ドローンとポッドのラベル、飛行軌跡。',
  pd_shell_solid:'ソリッド', pd_shell_cutaway:'カットアウェイ', pd_shell_xray:'X 線', pd_shell_frame:'フレーム',
  pd_deck_title:'甲板の機体と電池マガジン', pd_deck:'甲板（リフト＋区画）', pd_mag:'マガジン', pd_lift:'リフト',
  vt_card:'車両性能', vt_stock:'標準',
  vt_hint:'スライダーを止めて約 1 秒後に適用。車速は保たれます（わずかに落ちる）。',
  vt_car:'車両', vt_brake:'ブレーキ/輪', vt_torque:'最大トルク', vt_top:'最高速 (概算)',
  vt_me:'自車', vt_wait:'待機中', vt_wait_tip:'ポッドが走行を再開したら適用（スクリプト移動中、または配送シミュが旧版）', vt_none:'車両なし',
  ff_chip_state:'FFB', ff_chip_surface:'路面', ff_chip_sat:'前輪グリップ使用率',
  ff_card_main:'フォースフィードバック', ff_lg_now:'現在の速度でのセンタリングトルク',
  ff_lg_still:'停車時', ff_lg_wheel:'ハンドル位置',
  ff_m_wheel:'ハンドル位置', ff_m_out:'モーター出力', ff_m_damper:'ダンパー',
  ff_m_friction:'摩擦', ff_running:'動作中', ff_offline:'停止中',
  ff_stale:'待機中', ff_reset:'既定値に戻す',
  ff_hint:'変更は 0.5 秒以内にハンドルへ反映されます。設定は ffb_tuning.json に保存されます。',
  ff_saved:'保存しました', ff_p_forza_horizon:'Forza Horizon', ff_p_previous:'以前の設定',
  ff_p_light:'軽め', ff_p_heavy:'重め',
  ff_psave:'プリセット保存', ff_pname_ph:'プリセット名', ff_psaved:'保存しました:',
  ff_pdeleted:'削除しました', ff_pdel_q:'プリセットを削除', ff_pname_need:'名前を入力してください',
  ff_preplace_q:'保存済みプリセットを上書き',
  dr_chip_run:'配送シミュ', dr_chip_drones:'機体', dr_chip_flying:'飛行中',
  dr_chip_vans:'ポッド内', dr_chip_station:'充電所', dr_chip_down:'墜落',
  dr_chip_deliv:'配達済', dr_chip_fail:'失敗', dr_chip_wind:'風',
  dr_map:'市街地図', dr_fit:'全体', dr_map_hint:'ホイール: ズーム・ドラッグ: 移動・ダブルクリック: 全体・ポイントで詳細',
  dr_lg_pod:'配送ポッド', dr_lg_car:'他の車', dr_lg_drone:'ドローン（塗り=電池、枠=状態）',
  dr_lg_station:'充電ステーション', dr_lg_f2f:'対面注文', dr_lg_dr:'ドローン注文',
  dr_lg_hub:'ドローン発進地点', dr_lg_done:'配達済', dr_lg_fail:'失敗・墜落',
  dr_wind:'風', dr_wind_now:'現在', dr_wind_base:'基本風', dr_wind_rate:'ランダム突風',
  dr_ctl_speed:'突風 (m/s)', dr_ctl_dir:'方向', dr_ctl_dur:'継続時間',
  dr_ctl_base:'基本風 (m/s)', dr_ctl_rate:'突風 / 分',
  dr_btn_gust:'突風', dr_btn_follow:'風向に合わせる', dr_btn_base:'基本風を設定',
  dr_btn_rate:'突風頻度を設定', dr_btn_calm:'無風',
  dr_wind_hint:'コンパスをクリックで方向を選択・赤い円 = 11 m/s（ドローンが抗える上限）',
  dr_stats:'配達と失敗', dr_drones:'ドローン', dr_vans:'配送ポッド', dr_events:'ドローンのイベント',
  dr_nomap:'delivery_sim.py を待っています…（run_pipeline.py --delivery N で起動）',
  dr_waiting:'配送シミュレーションは動いていません。', dr_noevents:'イベントはまだありません。',
  dr_none:'停止中', dr_live:'稼働中', dr_stopped:'終了', dr_steady:'安定', dr_sent:'送信',
  dr_grounded:'離陸禁止', dr_wind_hold:'離陸禁止の風速',
  dr_lg_ev:'EV 充電器（ポッド）', dr_chip_busy:'稼働ポッド', dr_th_podbatt:'ポッド電池',
  dr_s_evch:'ポッドの充電回数', dr_s_podkm:'ポッド走行距離', dr_s_strand:'電池切れポッド',
  dr_nodrone:'機体なし',
  dr_th_drone:'機体', dr_th_status:'状態', dr_th_batt:'電池', dr_th_van:'ポッド', dr_th_order:'注文',
  dr_th_alt:'高度', dr_th_drift:'風で流され', dr_th_trips:'往復', dr_th_crash:'墜落',
  dr_th_note:'メモ', dr_th_state:'状態', dr_th_pend:'未配達',
  dr_s_deliv:'配達済', dr_s_bydrone:'ドローン', dr_s_fail:'失敗', dr_s_trips:'ドローン往復',
  dr_s_crash:'墜落', dr_s_swaps:'待機機との交換', dr_s_charge:'充電所での充電', dr_s_repair:'修理',
  dr_s_gusts:'突風', dr_s_km:'飛行距離', dr_s_queued:'待機中の注文',
  st_docked:'ポッド内', st_reloading:'積み込み中', st_delivering:'配達中', st_to_station:'充電へ',
  st_to_van:'ポッドへ', st_charging:'充電中', st_standby:'待機', st_falling:'落下中', st_crashed:'墜落',
  ost_queued:'待機', ost_assigned:'割当済', ost_in_flight:'飛行中', ost_delivered:'配達済',
  ost_failed:'失敗',
  fr_collision:'衝突', fr_blown_off_course:'風でロスト', fr_battery_depleted:'電池切れ',
  fr_motor_failure:'モーター故障', fr_parcel_lost:'荷物が外れた', fr_no_drone:'機体なし',
  fr_no_safe_drop:'安全な投下場所なし',
  veh_status:'車両ステータス', veh_sensors:'センサー',
  veh_diagram:'センサー・衝突マップ', veh_impacts:'衝突ログ',
  veh_last:'直近の衝突', veh_ok:'全システム正常', veh_warn:'注意',
  veh_crash:'衝突検出', veh_none:'衝突はまだありません', veh_ago:'秒前',
  veh_impulse:'インパルス', lbl_speed:'速度',
  veh_sn_hint:'センサーをクリックすると取付位置を表示します',
  veh_nearby:'周辺オブジェクト',
  veh_range:'50m以内のオブジェクト・距離順に表示',
  veh_none_near:'検知範囲内にありません',
  veh_no_sensors:'センサー未接続',
  kind_car:'車', kind_truck:'トラック', kind_bike:'二輪', kind_walker:'歩行者',
  sn_ultra:'超音波', sn_lidar:'LiDAR', sn_semlidar:'セマンティックLiDAR',
  sn_radar:'レーダー', sn_cam:'カメラ', sn_other:'センサー', sn_active:'稼働中',
  lg_bike:'二輪車', lg_walker:'歩行者',
  sig_hint:'ウインカー・ホイールボタン 11（左）／10（右）',
  ev_lane_change:'車線変更',
  btn_shutdown:'終了',
  shutdown_confirm:'シミュレーション全体を終了しますか？ CARLA・交通・運転ウィンドウ・このダッシュボードを停止し、使用中のポートを解放します。',
  shutdown_done:'シミュレーションを終了しています。全てのポートが解放されます。このタブは閉じて構いません。',
  zone_front:'前部', zone_rear:'後部', zone_left:'左側',
  zone_right:'右側', zone_front_left:'左前', zone_front_right:'右前',
  zone_rear_left:'左後', zone_rear_right:'右後', zone_center:'中央',
  sn_gnss:'GPS / GNSS', sn_imu:'IMU', sn_coll:'衝撃センサー（エアバッグ）',
  sn_lane:'車線カメラ（LDW）',
  st_connecting:'接続中…', st_conn_hero:'接続済み・車両検出',
  st_conn_wait:'接続済み・車両待機中', st_disconn:'切断されました',
  chip_map:'マップ', chip_vehicle:'車両', chip_mass:'質量 (kg)',
  chip_nearby:'周辺車両', chip_simt:'SIM時間 (秒)', dl_csv:'CSVログ',
  card_speed:'速度', lbl_limit:'制限', lbl_avg:'平均',
  lbl_gear:'ギア', lbl_heading:'方位',
  card_inputs:'運転操作', lbl_throttle:'アクセル', lbl_brake:'ブレーキ',
  lbl_steer:'ステア',
  card_tuning:'ペダル調整', tune_reset:'リセット',
  tune_bmax:'ブレーキ強度', tune_bcurve:'ブレーキ応答カーブ',
  tune_tmax:'アクセル強度', tune_tcurve:'アクセル応答カーブ',
  tune_hint:'踏込量→出力・カーブが大きいほど軽い踏込みで穏やか',
  al_lane:'車線逸脱！', al_crash:'衝突！',
  al_redlight:'信号無視！', al_hardbrake:'急ブレーキ',
  al_fcw:'ブレーキ！前方車両', al_speeding:'速度超過',
  al_test:'警告テスト',
  card_safety:'安全・ADAS', lbl_fdist:'前方距離',
  lbl_hardbrakes:'急ブレーキ', lbl_crashes:'衝突回数',
  lbl_tl:'信号', lbl_stability:'安定性',
  card_perf:'運転パフォーマンス', lbl_sdlp:'SDLP（車線維持）',
  lbl_srr:'ステア反転', lbl_jerkrms:'ジャークRMS（快適性）',
  lbl_laneinv:'車線逸脱', lbl_rlv:'信号無視',
  lbl_comp:'速度遵守率',
  card_lanepos:'車線内位置（SDLP）',
  lg_laneoff:'中心からのオフセット', lg_laneedge:'車線端',
  card_events:'運転イベント', ev_none:'まだありません',
  ev_collision:'衝突', ev_lane:'車線逸脱', ev_hard_brake:'急ブレーキ',
  ev_red_light:'信号無視',
  card_speedhist:'速度履歴と制限速度', lg_speed:'速度', lg_limit:'制限',
  lg_avg:'平均',
  card_gg:'G-G線図', gg_hint:'X:横G · Y:前後G · リング0.5G刻み',
  card_acc:'加速度', lg_lon:'前後', lg_lat:'横', lg_jerk:'ジャーク',
  card_dyn:'車両挙動', lbl_lon:'前後加速度', lbl_lat:'横加速度',
  lbl_vert:'上下加速度', lbl_jerk:'ジャーク', lbl_g:'G',
  lbl_yawrate:'ヨーレート', lbl_slip:'スリップ角', lbl_curv:'曲率',
  card_yaw:'ヨーレートとステア', lg_yaw:'ヨー°/s', lg_steer100:'ステア×100',
  card_lane:'車線・道路', lbl_onroad:'車線上', lbl_roadlane:'道路/車線ID',
  lbl_lanew:'車線幅', lbl_laneoff:'横オフセット', lbl_junction:'交差点',
  yes:'はい', no:'いいえ', offroad:'車線外',
  card_weather:'天候', lbl_cloud:'雲量', lbl_rain:'雨', lbl_wet:'路面湿潤',
  lbl_fog:'霧', lbl_wind:'風', lbl_sunalt:'太陽高度',
  card_trip:'走行', lbl_elapsed:'経過時間', lbl_avgspeed:'平均速度',
  lbl_collpeak:'衝突ピーク',
  card_coll:'衝突強度（10秒）',
  coll_hint:'衝突センサの法線インパルス',
  card_map:'周辺状況（80m）', lg_hero:'自車',
  lg_nearby:'周辺車両', lg_cone:'前方コーン',
  card_trail:'走行軌跡', trail_hint:'ワールドX/Y軌跡',
  card_gnss:'GNSS・IMU', lbl_pos:'X / Y / Z', lbl_lat2:'緯度',
  lbl_lon2:'経度', lbl_accel:'加速度 x/y/z', lbl_gyro:'角速度 x/y/z',
  lbl_compass:'コンパス',
  pill_ok:'OK', pill_over:'超過',
  tl_None:'なし', tl_Red:'赤', tl_Yellow:'黄', tl_Green:'青',
  tile_crash:'衝突回数', tile_lane:'車線逸脱回数', tile_rlv:'信号無視',
  tile_hb:'急ブレーキ', tile_dist:'走行距離', tile_dur:'走行時間',
  tile_avg:'平均速度', tile_comp:'速度遵守率',
  stats_session:'現在のセッション', stats_history:'セッション履歴',
  hist_hint:'セッション毎の衝突・車線逸脱（最新20件）',
  stats_sessions:'保存済みセッション',
  stats_empty:'保存されたセッションはまだありません',
  th_time:'時刻', th_map:'マップ', th_dur:'時間', th_dist:'距離 (km)',
  th_crash:'衝突', th_lane:'車線逸脱', th_rlv:'信号無視',
  th_hb:'急ブレーキ', th_comp:'遵守率', badge_live:'走行中',
 }
};
let LANG = new URLSearchParams(location.search).get('lang')
        || localStorage.getItem('carla_lang') || 'en';
if (!I18N[LANG]) LANG = 'en';
const t = k => (I18N[LANG] && I18N[LANG][k]) || I18N.en[k] || k;

function labelRanges() {
  let k = 0;
  document.querySelectorAll('input[type=range]').forEach(r => {
    if (r.getAttribute('aria-label') || r.getAttribute('aria-labelledby') ||
        document.querySelector('label[for="' + r.id + '"]')) return;
    let lab = r.previousElementSibling;                     // wind: label span before it
    if (!lab || lab.tagName !== 'SPAN') {                   // pedals: .inlbl row above
      const row = r.previousElementSibling;
      lab = row && row.classList.contains('inlbl') ? row.querySelector('span') : null;
    }
    if (!lab) return;
    if (!lab.id) lab.id = 'rl-' + (k++);
    r.setAttribute('aria-labelledby', lab.id);
  });
}
function applyLang() {
  document.documentElement.lang = LANG;
  document.querySelectorAll('[data-i18n]').forEach(el => {
    el.textContent = t(el.dataset.i18n);
  });
  document.querySelectorAll('[data-i18n-ph]').forEach(el => {
    el.placeholder = t(el.dataset.i18nPh);
  });
  $('lang-en').classList.toggle('active', LANG === 'en');
  $('lang-ja').classList.toggle('active', LANG === 'ja');
  renderSessTable(); renderHist(); if (FF.params) ffBuild(); if (VT.params) vtBuild();
}
function setLang(l) { LANG = l; localStorage.setItem('carla_lang', l); applyLang(); }
$('lang-en').onclick = () => setLang('en');
$('lang-ja').onclick = () => setLang('ja');

/* ------------------------------------------------------------------ tabs */
// Tabs: Drive (live telemetry), Vehicle, Delivery (map, fleet, pods), Statistics
// and Settings -- every control, in three sections. Displays and controls are
// kept apart. / 表示タブと設定タブを分ける（操作はすべて設定タブに集約）。
const VIEWS = ['dash', 'veh', 'drone', 'stats', 'settings'];
const SECTIONS = ['driving', 'ffb', 'delivery'];
let SECTION = 'driving';
function secVisible(name) { return !$('view-settings').hidden && SECTION === name; }
function showSection(name) {
  if (!SECTIONS.includes(name)) name = 'driving';
  SECTION = name;
  SECTIONS.forEach(n => {
    $('set-' + n).hidden = n !== name;
    $('set-btn-' + n).classList.toggle('active', n === name);
    $('set-btn-' + n).setAttribute('aria-selected', n === name ? 'true' : 'false');
  });
  if (name === 'ffb') setTimeout(ffTick, 0);
  if (name === 'delivery') { setTimeout(drTick, 0); setTimeout(podTick, 0); }
  if (name === 'driving') { setTimeout(vtTick, 0); setTimeout(renderTuneCurve, 0); }
}
function showView(which, section) {
  if (!VIEWS.includes(which)) which = 'dash';
  VIEWS.forEach(v => {
    $('view-' + v).hidden = v !== which;
    $('tab-' + v).classList.toggle('active', v === which);
    $('tab-' + v).setAttribute('aria-selected', v === which ? 'true' : 'false');
  });
  if (which === 'settings') showSection(section || SECTION);
  if (history.replaceState)
    history.replaceState(null, '', which === 'dash' ? '#' :
      '#' + which + (which === 'settings' ? '-' + SECTION : ''));
  if (which === 'stats') loadSessions();
  if (which === 'drone') { setTimeout(drTick, 0); setTimeout(podTick, 0); }
}
VIEWS.forEach(v => $('tab-' + v).onclick = () => showView(v));
SECTIONS.forEach(n => $('set-btn-' + n).onclick = () => showView('settings', n));

/* ---------------------------------------------------------- vehicle view */
const escV = s => String(s).replace(/[<>&]/g,
  c => ({'<':'&lt;', '>':'&gt;', '&':'&amp;'}[c]));
function vzone(f, r) {
  const fz = f > 0.4 ? 'front' : f < -0.4 ? 'rear' : '';
  const rz = r > 0.4 ? 'right' : r < -0.4 ? 'left' : '';
  return t('zone_' + (fz && rz ? fz + '_' + rz : (fz || rz || 'center')));
}
const fmtAgo = a => a < 60 ? fmt(a, 0) + t('veh_ago')
                          : fmt(a / 60, 0) + 'min';
const VHITS = new Map();       // hit key -> SVG node, so ripples keep looping
let VH_LAST_N = -1;
function updVehHits(hits) {
  const layer = $('vhits'), live = new Set();
  hits.forEach(h => {
    const key = h.f + '|' + h.r + '|' + h.i;
    live.add(key);
    let el = VHITS.get(key);
    if (!el) {
      const x = fmt(150 + h.r * 50, 1), y = fmt(300 - h.f * 140, 1);
      el = document.createElementNS('http://www.w3.org/2000/svg', 'g');
      el.innerHTML =
        '<circle class="hitring" cx="' + x + '" cy="' + y + '" r="7"/>' +
        '<circle class="hitring" style="animation-delay:.55s" cx="' + x +
        '" cy="' + y + '" r="7"/>' +
        '<circle class="hitcore" cx="' + x + '" cy="' + y + '" r="5"/>';
      layer.appendChild(el);
      VHITS.set(key, el);
    }
    el.style.opacity = clamp(1 - h.age / 150, 0.25, 1);
  });
  [...VHITS.keys()].forEach(k => {
    if (!live.has(k)) { VHITS.get(k).remove(); VHITS.delete(k); }
  });
}
/* Attached sensors: card rows + SVG markers, built from the live CARLA
   actor list. / 取付センサー：CARLAの実アクター一覧からカードとSVGを生成。 */
const SN_META = [
  ['sensor.other.gnss',              'sn_gnss',     '#22d3ee'],
  ['sensor.other.imu',               'sn_imu',      '#9085e9'],
  ['sensor.other.collision',         'sn_coll',     '#e66767'],
  ['sensor.other.lane_invasion',     'sn_lane',     '#3987e5'],
  ['sensor.other.obstacle',          'sn_ultra',    '#fab219'],
  ['sensor.lidar.ray_cast_semantic', 'sn_semlidar', '#199e70'],
  ['sensor.lidar',                   'sn_lidar',    '#199e70'],
  ['sensor.other.radar',             'sn_radar',    '#ec835a'],
  ['sensor.camera',                  'sn_cam',      '#c98500'],
];
function snMeta(type) {
  for (const [p, k, c] of SN_META) if (type.startsWith(p)) return [k, c];
  return ['sn_other', '#69758a'];
}
function snValue(s, type) {
  if (type.startsWith('sensor.other.gnss'))
    return fmt(s.gnss.lat, 4) + ', ' + fmt(s.gnss.lon, 4);
  if (type.startsWith('sensor.other.imu'))
    return fmt(s.imu.ax, 1) + ' / ' + fmt(s.imu.ay, 1) + ' m/s²';
  if (type.startsWith('sensor.other.collision')) return s.collision_count || 0;
  if (type.startsWith('sensor.other.lane_invasion')) return s.lane_invasions || 0;
  if (type.startsWith('sensor.other.obstacle'))
    return s.ultrasonic_m > 0 ? fmt(s.ultrasonic_m, 1) + ' m' : '—';
  return t('sn_active');
}
/* Mounting-zone name from the vehicle-frame offset (reuses zone_* i18n).
   車両座標系オフセットから取付位置名を求める（zone_* の翻訳を再利用）。 */
function snPos(x, bb) {
  const f = x.f / Math.max(bb.x, 0.1), r = x.r / Math.max(bb.y, 0.1);
  const fz = f > 0.35 ? 'front' : f < -0.35 ? 'rear' : '';
  const rz = r > 0.35 ? 'right' : r < -0.35 ? 'left' : '';
  return t('zone_' + (fz && rz ? fz + '_' + rz : (fz || rz || 'center')));
}
/* Cameras (and any duplicated sensor type) get the position appended.
   カメラ（および同種が複数あるセンサー）には取付位置名を付ける。 */
function snName(x, bb, dupKeys) {
  const [k] = snMeta(x.type);
  return (k === 'sn_cam' || dupKeys.has(k))
    ? t(k) + ' · ' + snPos(x, bb) : t(k);
}
const VSCALE = 0.5;              // must match #vscale in the SVG
let SENS_SIG = '', SEL_SENS = -1;
let LAST_SENS = {sens: [], bb: {x: 2.4, y: 1.0}, pts: []};
const sensPos = (x, bb) => [150 + clamp(x.r / bb.y, -1.15, 1.15) * 49,
                            300 - clamp(x.f / bb.x, -1.15, 1.15) * 140];
/* Sensors mounted at the same spot are fanned out sideways so every
   marker stays visible. / 同じ位置のセンサーは横にずらして全て見えるように。 */
function spreadPts(sens, bb) {
  const pts = sens.map(x => sensPos(x, bb));
  const groups = {};
  pts.forEach((p, i) => {
    const k = Math.round(p[0] / 10) + ':' + Math.round(p[1] / 10);
    (groups[k] = groups[k] || []).push(i);
  });
  Object.values(groups).forEach(g => g.forEach((idx, j) => {
    pts[idx][0] += (j - (g.length - 1) / 2) * 15;
  }));
  return pts;
}
function sensDupKeys(sens) {
  const cnt = {};
  sens.forEach(x => { const k = snMeta(x.type)[0]; cnt[k] = (cnt[k] || 0) + 1; });
  return new Set(Object.keys(cnt).filter(k => cnt[k] > 1));
}
/* Show the clicked sensor's location: highlight its marker, pulse ring,
   leader label. Click again to hide.
   クリックしたセンサーの位置を表示：マーカー強調＋パルスリング＋ラベル。
   もう一度クリックで解除。 */
function applySensorSel() {
  const {sens, bb, pts} = LAST_SENS;
  if (SEL_SENS >= sens.length) SEL_SENS = -1;
  const rows = document.querySelectorAll('#sensor-rows .srow');
  rows.forEach((el, i) => el.classList.toggle('sel', i === SEL_SENS));
  $('vsens').classList.toggle('dim', SEL_SENS >= 0);
  document.querySelectorAll('#vsens .smark').forEach((el, i) =>
    el.classList.toggle('sel', i === SEL_SENS));
  if (SEL_SENS < 0) { $('vsenslabels').innerHTML = ''; return; }
  const x = sens[SEL_SENS], dup = sensDupKeys(sens);
  const [px, py] = pts[SEL_SENS] || sensPos(x, bb);
  const mx = 150 + (px - 150) * VSCALE, my = 300 + (py - 300) * VSCALE;
  const lx = clamp(mx, 72, 228);
  const ly = my < 300 ? my - 17 : my + 25;
  $('vsenslabels').innerHTML =
    '<circle class="slocate" cx="' + fmt(mx, 1) + '" cy="' + fmt(my, 1)
    + '" r="8"/>'
    + '<text class="sfocus" x="' + fmt(lx, 1) + '" y="' + fmt(ly, 1) + '">'
    + escV(snName(x, bb, dup)).toUpperCase() + '</text>';
}
$('sensor-rows').addEventListener('click', e => {
  const row = e.target.closest('#sensor-rows .srow');
  if (!row || row.dataset.i === undefined) return;
  SEL_SENS = (SEL_SENS === +row.dataset.i) ? -1 : +row.dataset.i;
  applySensorSel();
});

/* Turn-signal buttons: POST toggles the server-side state (shared with
   the wheel's buttons 11/10). / ウインカーボタン：POST でサーバー側の
   状態をトグル（ホイールのボタン11／10と共有）。 */
function postSignal(side) {
  fetch('/signal', {method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({side})}).catch(() => {});
}
$('sig-l').onclick = () => postSignal('left');
$('sig-r').onclick = () => postSignal('right');

/* Shutdown: stops CARLA + all helper processes and frees their ports.
   シャットダウン：CARLAと全補助プロセスを停止しポートを解放する。 */
$('shutdown').onclick = async () => {
  if (!confirm(t('shutdown_confirm'))) return;
  try { await fetch('/shutdown', {method: 'POST'}); } catch (e) {}
  const mask = document.createElement('div');
  mask.id = 'shutmask';
  mask.textContent = t('shutdown_done');
  document.body.appendChild(mask);
};
function renderSensors(s) {
  const sens = s.sensors || [], on = !!s.hero_found;
  const bb = s.bb || {x: 2.4, y: 1.0};
  LAST_SENS = {sens, bb, pts: spreadPts(sens, bb)};
  const sig = LANG + '|' + (on ? 1 : 0) + '|' + bb.x + ',' + bb.y + '|'
            + sens.map(x => x.type + ',' + x.f + ',' + x.r).join(';');
  if (sig !== SENS_SIG) {
    SENS_SIG = sig;
    $('vh-sensn').textContent = sens.length;
    const dup = sensDupKeys(sens);
    $('sensor-rows').innerHTML = sens.length ? sens.map((x, i) => {
      const [, c] = snMeta(x.type);
      return '<div class="srow" data-i="' + i + '"><i class="sdot" style="--c:'
           + (on ? c : '#69758a') + '"></i><span>' + escV(snName(x, bb, dup))
           + '</span><span class="sval" id="snv-' + i + '">–</span></div>';
    }).join('') : '<div class="srow"><span>' + t('veh_no_sensors')
                + '</span></div>';
    // markers on the (scaled) car outline / （縮小表示の）車両上のマーカー
    $('vsens').innerHTML = sens.map((x, i) => {
      const [, c] = snMeta(x.type), [px, py] = LAST_SENS.pts[i];
      return '<g class="smark"><circle class="halo" cx="' + fmt(px, 1)
           + '" cy="' + fmt(py, 1) + '" r="5" stroke="' + c + '"/>'
           + '<circle class="core" cx="' + fmt(px, 1) + '" cy="' + fmt(py, 1)
           + '" r="4.5" fill="' + c + '"/></g>';
    }).join('');
    applySensorSel();
  }
  sens.forEach((x, i) => {
    const el = $('snv-' + i);
    if (el) el.textContent = snValue(s, x.type);
  });
}

/* Park-distance margins per zone, from ground-truth nearby objects.
   Margin = object distance minus the car body extent.
   周辺オブジェクトからゾーンごとの余裕距離を算出（車体寸法を差し引く）。 */
const PDC_THR = [0.9, 1.8, 3.0];
function updatePdc(list, bb) {
  const zones = {f: Infinity, b: Infinity, l: Infinity, r: Infinity};
  list.forEach(v => {
    const m = Math.hypot(Math.max(0, Math.abs(v.fx) - bb.x),
                         Math.max(0, Math.abs(v.rx) - bb.y));
    const th = Math.atan2(v.rx, v.fx) * 180 / Math.PI;
    const z = Math.abs(th) <= 45 ? 'f' : Math.abs(th) >= 135 ? 'b'
            : th > 0 ? 'r' : 'l';
    if (m < zones[z]) zones[z] = m;
  });
  Object.keys(zones).forEach(z => {
    const m = zones[z];
    PDC_THR.forEach((thr, i) => {
      const el = $('pdc-' + z + '-' + i);
      if (el) el.classList.toggle('lit', m <= thr);
    });
    const tx = $('pdct-' + z);
    if (!tx) return;
    if (m <= 3.0) {
      tx.textContent = fmt(m, 1) + ' m';
      tx.style.fill = m <= 0.9 ? '#ff9c9c' : m <= 1.8 ? '#fac95e' : '#5fd45f';
      tx.style.display = 'block';
    } else tx.style.display = 'none';
  });
}

/* Nearby cars / bikes / pedestrians on the radar, with distances.
   レーダー上の周辺車両・二輪・歩行者と距離表示。 */
const KIND_C = {car: '#e66767', truck: '#ec835a',
                bike: '#9085e9', walker: '#22d3ee'};
const distPix = d => 240 * Math.sqrt(clamp(d, 0, 50) / 50);
const fmtDist = d => (d < 10 ? fmt(d, 1) : fmt(d, 0)) + ' m';
/* Downloaded top-view icons (see <defs> in the SVG). The car drawing is
   horizontal (nose to the right), so it gets a -90\u00b0 base rotation; the
   walking-person pictogram is never rotated.
   ダウンロードした上面図アイコン（SVG の <defs> 参照）。車は横向き
   （前が右）なので -90度 の基準回転を掛ける。歩行者ピクトグラムは
   回転させない。 */
function nearShape(kind, c) {
  if (kind === 'walker')
    return '<use href="#ic-walk" x="-4.7" y="-7.5" width="9.4" height="15"'
         + ' style="color:' + c + '"/>';
  if (kind === 'bike')
    return '<use href="#ic-bike" x="-4.4" y="-11" width="8.8" height="22"'
         + ' style="color:' + c + '"/>';
  if (kind === 'truck')
    return '<use href="#ic-truck" x="-7.7" y="-17" width="15.5" height="34"'
         + ' style="color:' + c + '"/>';
  return '<g transform="rotate(-90)"><use href="#ic-car" x="-15" y="-7.4"'
       + ' width="30" height="14.9" style="color:' + c + '"/></g>';
}
const CAR_A = 78, CAR_B = 34;   // hero silhouette half extents on stage (px)
function renderNear(list) {
  $('vnear').innerHTML = list.map(v => {
    const c = KIND_C[v.kind] || C.red;
    const th = Math.atan2(v.rx, v.fx);
    // keep very close objects just outside the car silhouette (ellipse),
    // like a real parking display / 至近距離の対象は実車のパーキング表示の
    // ように車両シルエットのすぐ外側に描く
    const rmin = CAR_A * CAR_B
               / Math.hypot(CAR_B * Math.cos(th), CAR_A * Math.sin(th)) + 8;
    const rd = distPix(v.d), r = Math.max(rd, rmin);
    const px = 150 + Math.sin(th) * r, py = 300 - Math.cos(th) * r;
    // objects pushed out to the silhouette get a sideways label so it
    // stays clear of the car and the margin readouts
    // シルエット位置に寄せた対象はラベルを横に出して重なりを避ける
    const clamped = rd < rmin;
    const ty = clamped ? 4 : (v.kind === 'walker' ? 16 : 24);
    const tattr = clamped
      ? ' x="' + (px >= 150 ? 16 : -16) + '" style="text-anchor:'
        + (px >= 150 ? 'start' : 'end') + '"'
      : '';
    return '<g class="nearg" transform="translate(' + fmt(px, 1) + ','
         + fmt(py, 1) + ')" opacity="' + fmt(clamp(1.15 - v.d / 60, .4, 1), 2)
         + '"><g transform="rotate('
         + fmt(v.kind === 'walker' ? 0 : (v.yaw || 0), 0) + ')">'
         + nearShape(v.kind, c) + '</g><text class="ndist" y="' + ty + '"'
         + tattr + '>' + fmtDist(v.d) + '</text></g>';
  }).join('');
}
function renderNearList(list) {
  $('vh-nearn').textContent = list.length;
  $('vh-near').innerHTML = list.length ? list.slice(0, 10).map(v =>
    '<div class="ev"><span class="ty" style="color:'
    + (KIND_C[v.kind] || C.red) + '">' + t('kind_' + v.kind)
    + '</span><span class="m">' + fmtDist(v.d) + ' · ' + fmt(v.kmh, 0)
    + ' km/h</span></div>').join('')
    : '<div class="ev"><span class="m">' + t('veh_none_near')
    + '</span></div>';
}

function renderVeh(s) {
  const hits = s.collision_hits || [];
  const fresh = hits.some(h => h.age < 5);
  const warn = (s.hero_found && !s.on_road) || (s.stability ?? 1) < 0.6 ||
               s.abs_active || s.tcs_active;
  const st = fresh ? 'crash' : warn ? 'warn' : 'ok';
  const pill = $('vh-pill');
  pill.className = 'vpill ' + st;
  pill.textContent = t(st === 'crash' ? 'veh_crash'
                     : st === 'warn' ? 'veh_warn' : 'veh_ok');
  const car = $('vcar');
  car.classList.remove('ok', 'warn', 'crash');
  if (st === 'crash' && hits.length !== VH_LAST_N)
    car.getBoundingClientRect();          // force reflow → replay shake
  car.classList.add(st);
  VH_LAST_N = hits.length;

  // readouts
  $('vh-speed').textContent = fmt(s.speed_kmh, 0);
  $('vh-gear').textContent = s.reverse ? 'R'
                           : (s.gear || 0) === 0 ? 'N' : s.gear;
  const sa = (s.steer || 0) * 35;         // approx. road-wheel angle
  $('vh-steer').textContent = fmt(sa, 1) + '°';
  $('vh-head').textContent = fmt(((s.heading_deg || 0) + 360) % 360, 0) + '°';
  $('vh-stab').textContent = fmt((s.stability ?? 1) * 100, 0) + '%';
  $('vh-stab-bar').style.width = ((s.stability ?? 1) * 100) + '%';
  $('vi-rev').classList.toggle('on', !!s.reverse);
  $('vi-hb').classList.toggle('on', !!s.hand_brake);
  $('vi-abs').classList.toggle('on', !!s.abs_active);
  $('vi-tcs').classList.toggle('on', !!s.tcs_active);

  // car animation: steering wheels, lights, speed streaks
  // (wheel pivots are in the embedded car drawing's local coordinates)
  $('vwfl').setAttribute('transform', 'rotate(' + fmt(sa, 1) + ' 22.5 140.5)');
  $('vwfr').setAttribute('transform', 'rotate(' + fmt(sa, 1) + ' 332.7 137.4)');
  const brk = s.brake > 0.03 ? 0.25 + 0.75 * s.brake : 0.12;
  $('vtl-l').style.opacity = brk;
  $('vtl-r').style.opacity = brk;
  $('vrev').style.opacity = s.reverse ? 0.9 : 0;
  const spd = s.speed_kmh || 0, streaks = $('vstreaks');
  streaks.style.opacity = clamp((spd - 4) / 40, 0, 0.75);
  streaks.style.setProperty('--sd',
    clamp(60 / Math.max(spd, 1), 0.25, 3) + 's');

  // sensors: live list from the sim (lidar, ultrasonic, cameras, ...)
  // センサー：シミュレーションの実取付一覧（LiDAR・超音波・カメラ等）
  renderSensors(s);

  // nearby cars / bikes / pedestrians within 50 m, with distance
  // 50m以内の周辺車両・二輪・歩行者と距離
  const near = (s.nearby_list || []).filter(v => (v.d || 0) <= 50);
  renderNear(near);
  renderNearList(near);
  updatePdc(s.nearby_list || [], s.bb || {x: 2.4, y: 1.0});

  // ultrasonic front arc / 超音波の前方距離アーク
  const du = s.ultrasonic_m, ua = $('vultra');
  if (du > 0) {
    const r = distPix(du), a = 18 * Math.PI / 180;
    const xw = Math.sin(a) * r, y = fmt(300 - Math.cos(a) * r, 1);
    ua.setAttribute('d', 'M' + fmt(150 - xw, 1) + ',' + y + ' A' + fmt(r, 1)
      + ',' + fmt(r, 1) + ' 0 0 1 ' + fmt(150 + xw, 1) + ',' + y);
    const uc = du < 3 ? C.crit : du < 8 ? C.warn : C.green;
    ua.style.stroke = uc; ua.style.color = uc;
    ua.style.display = 'block';
  } else ua.style.display = 'none';

  // turn signal: buttons, blinker lamps, lane-change chevrons
  // ウインカー：ボタン・ランプ・車線変更シェブロン
  const sig = s.turn_signal || 'off';
  $('sig-l').classList.toggle('on', sig === 'left');
  $('sig-r').classList.toggle('on', sig === 'right');
  $('vsig-l').classList.toggle('on', sig === 'left');
  $('vsig-r').classList.toggle('on', sig === 'right');
  $('vsigarr-l').classList.toggle('on', sig === 'left');
  $('vsigarr-r').classList.toggle('on', sig === 'right');

  // own-lane guide lines at their true lateral offsets
  // 自車線ガイド線（実際の横方向オフセット位置に表示）
  const lg = $('vlane');
  if (s.on_road && (s.lane_width || 0) > 0.5) {
    const lo = s.lane_offset || 0, lw2 = s.lane_width / 2;
    const ex = e => 150 + Math.sign(e) * distPix(Math.abs(e));
    const x1 = ex(-lo - lw2), x2 = ex(-lo + lw2);
    $('vlane-l').setAttribute('x1', fmt(x1, 1));
    $('vlane-l').setAttribute('x2', fmt(x1, 1));
    $('vlane-r').setAttribute('x1', fmt(x2, 1));
    $('vlane-r').setAttribute('x2', fmt(x2, 1));
    const f = $('vlane-fill');
    f.setAttribute('x', fmt(Math.min(x1, x2), 1));
    f.setAttribute('width', fmt(Math.abs(x2 - x1), 1));
    lg.style.display = 'block';
  } else lg.style.display = 'none';

  const mini = $('vh-mini');
  mini.textContent = t('tile_crash') + ': ' + (s.collision_count || 0);
  mini.className = 'pill' + ((s.collision_count || 0) > 0 ? ' red' : '');

  // impact map + log + last-impact zone
  updVehHits(hits);
  const list = $('vh-hits');
  if (!hits.length) {
    list.innerHTML = '<div class="ev"><span class="m">'
                   + t('veh_none') + '</span></div>';
    $('vh-zone').textContent = '–';
    $('vh-zage').textContent = t('veh_none');
  } else {
    list.innerHTML = hits.slice().reverse().map(h =>
      '<div class="ev ev-collision"><span class="t">' + fmtAgo(h.age)
      + '</span><span class="ty">' + vzone(h.f, h.r)
      + '</span><span class="m">' + escV(h.o) + ' · '
      + t('veh_impulse') + ' ' + h.i + '</span></div>').join('');
    const h = hits[hits.length - 1];
    $('vh-zone').textContent = vzone(h.f, h.r);
    $('vh-zage').textContent = fmtAgo(h.age) + ' · ' + t('veh_impulse')
                             + ' ' + h.i;
  }
}

/* --------------------------------------------------------------- history */
const HIST_LEN = 300; // ~60s at 5Hz
const H = {
  speed: [], limit: [], avg: [],
  throttle: [], brake: [], steer: [],
  lon: [], lat: [], jerk: [],
  yaw: [], steerY: [],
  ttc: [], thw: [], gg: [],
  laneoff: [], laneEdgeP: [], laneEdgeN: [],
};
function push(arr, v){ arr.push(v); if(arr.length>HIST_LEN) arr.shift(); }

const PADL = 34;

function drawLine(ctx, data, color, minY, maxY, w, h) {
  if (!data.length) return;
  const span = Math.max(1e-6, maxY - minY);
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.lineJoin = 'round'; ctx.beginPath();
  data.forEach((v, i) => {
    const x = PADL + (i / (HIST_LEN-1)) * (w - PADL - 4);
    const y = h - 6 - ((v - minY) / span) * (h - 12);
    if (i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}
function drawAxis(ctx, w, h, minY, maxY, label) {
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  [0, 0.5, 1].forEach(f => {
    const y = Math.round(h - 6 - f*(h-12)) + 0.5;
    ctx.beginPath(); ctx.moveTo(PADL, y); ctx.lineTo(w-4, y); ctx.stroke();
  });
  ctx.fillStyle = C.mute; ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'right';
  ctx.fillText(fmt(maxY,1), PADL-4, 10);
  ctx.fillText(fmt((maxY+minY)/2,1), PADL-4, h/2+3);
  ctx.fillText(fmt(minY,1), PADL-4, h-4);
  if (label) { ctx.textAlign = 'left'; ctx.fillText(label, PADL+6, 10); }
  if (minY < 0 && maxY > 0) {
    const y = Math.round(h - 6 - ((0-minY)/(maxY-minY))*(h-12)) + 0.5;
    ctx.strokeStyle = C.axis;
    ctx.beginPath(); ctx.moveTo(PADL, y); ctx.lineTo(w-4, y); ctx.stroke();
  }
}
function setupCanvas(c) {
  const ctx = c.getContext('2d');
  c.width = c.clientWidth * devicePixelRatio;
  c.height = c.clientHeight * devicePixelRatio;
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  return [ctx, c.clientWidth, c.clientHeight];
}
function renderChart(id, series, range, label) {
  const c = $(id); if (!c) return;
  const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  drawAxis(ctx, w, h, range[0], range[1], label);
  series.forEach(([data, col]) => drawLine(ctx, data, col, range[0], range[1], w, h));
}

/* ------------------------------------------------------- hover tooltips */
const CHART_CFG = {
  speedhist:{unit:' km/h', d:1, series:[['speed','lg_speed',C.blue],
             ['limit','lg_limit',C.yellow],['avg','lg_avg',C.green]]},
  ctrlhist:{unit:'', d:0, series:[['throttle','lbl_throttle',C.green],
             ['brake','lbl_brake',C.red],['steer','lbl_steer',C.yellow]]},
  acchist:{unit:' m/s²', d:2, series:[['lon','lg_lon',C.blue],
             ['lat','lg_lat',C.red],['jerk','lg_jerk',C.yellow]]},
  yawhist:{unit:'', d:1, series:[['yaw','lg_yaw',C.blue],
             ['steerY','lg_steer100',C.yellow]]},
  ttchist:{unit:' s', d:1, series:[['thw','THW',C.blue],['ttc','TTC',C.red]]},
  lanehist:{unit:' m', d:2, series:[['laneoff','lg_laneoff',C.blue]]},
};
const tooltipEl = document.getElementById('tooltip');
function placeTooltip(e) {
  const pad = 14;
  let x = e.clientX + pad, y = e.clientY + 12;
  const r = tooltipEl.getBoundingClientRect();
  if (x + r.width > innerWidth - 8) x = e.clientX - r.width - pad;
  if (y + r.height > innerHeight - 8) y = e.clientY - r.height - 12;
  tooltipEl.style.left = x + 'px'; tooltipEl.style.top = y + 'px';
}
function initHover() {
  Object.keys(CHART_CFG).forEach(id => {
    const c = $(id); if (!c) return;
    const xl = c.parentElement.querySelector('.xline');
    c.addEventListener('mousemove', e => {
      const cfg = CHART_CFG[id];
      const r = c.getBoundingClientRect();
      const frac = clamp((e.clientX - r.left - PADL) / (r.width - PADL - 4), 0, 1);
      const idx = Math.round(frac * (HIST_LEN - 1));
      const rows = cfg.series.map(([k, nk, col]) => {
        const d = H[k]; if (!d.length) return '';
        const v = d[Math.min(idx, d.length - 1)];
        return '<div class="tr"><i style="background:'+col+'"></i>'+t(nk)+
               '<span class="tv">'+fmt(v, cfg.d)+cfg.unit+'</span></div>';
      }).join('');
      if (!rows) return;
      tooltipEl.innerHTML = rows;
      tooltipEl.style.display = 'block';
      placeTooltip(e);
      if (xl) { xl.style.display = 'block';
        xl.style.left = (PADL + frac * (r.width - PADL - 4)) + 'px'; }
    });
    c.addEventListener('mouseleave', () => {
      tooltipEl.style.display = 'none';
      if (xl) xl.style.display = 'none';
    });
  });
}

/* ------------------------------------------------------- special charts */
function renderGG() {
  const c = $('gg'); const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  const cx = w/2, cy = h/2, G = 9.81;
  const scale = Math.min(w,h)/2 / (1.5*G);
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  for (let g=0.5; g<=1.5; g+=0.5) {
    ctx.beginPath(); ctx.arc(cx, cy, g*G*scale, 0, Math.PI*2); ctx.stroke();
  }
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,h);
  ctx.moveTo(0,cy); ctx.lineTo(w,cy); ctx.stroke();
  ctx.fillStyle = C.mute; ctx.font='11px system-ui, sans-serif';
  ctx.fillText('lon+', cx+4, 10); ctx.fillText('lat+', w-26, cy-4);
  H.gg.forEach((p, i) => {
    const age = (H.gg.length - i) / H.gg.length;
    ctx.fillStyle = 'rgba(57,135,229,'+clamp(1-age,0.06,1)+')';
    ctx.beginPath();
    ctx.arc(cx + p[1]*scale, cy - p[0]*scale, 2.2, 0, Math.PI*2);
    ctx.fill();
  });
  if (H.gg.length) {
    const p = H.gg[H.gg.length-1];
    ctx.fillStyle = C.warn;
    ctx.beginPath();
    ctx.arc(cx + p[1]*scale, cy - p[0]*scale, 4, 0, Math.PI*2);
    ctx.fill();
  }
}

function renderMinimap(s) {
  const c = $('minimap'); const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  const cx = w/2, cy = h/2;
  const R = 80; const sc = Math.min(w,h)/2 / R;
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  [20,40,60,80].forEach(r => {
    ctx.beginPath(); ctx.arc(cx, cy, r*sc, 0, Math.PI*2); ctx.stroke();
    ctx.fillStyle = C.mute; ctx.font='11px system-ui, sans-serif';
    ctx.fillText(r+'m', cx+4, cy - r*sc + 12);
  });
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,h);
  ctx.moveTo(0,cy); ctx.lineTo(w,cy); ctx.stroke();
  ctx.fillStyle = 'rgba(57,135,229,0.10)';
  ctx.beginPath(); ctx.moveTo(cx, cy);
  const ang = 20*Math.PI/180;
  ctx.lineTo(cx + Math.sin(-ang)*60*sc, cy - Math.cos(-ang)*60*sc);
  ctx.lineTo(cx + Math.sin(ang)*60*sc, cy - Math.cos(ang)*60*sc);
  ctx.closePath(); ctx.fill();
  (s.nearby_list||[]).forEach(v => {
    const x = cx + v.rx * sc;
    const y = cy - v.fx * sc;
    ctx.fillStyle = KIND_C[v.kind] || C.red;
    ctx.beginPath();
    ctx.arc(x, y, v.kind === 'walker' ? 3 : 4, 0, Math.PI*2); ctx.fill();
    ctx.strokeStyle = 'rgba(13,17,23,.9)'; ctx.lineWidth = 2; ctx.stroke();
    if (v.kmh !== undefined) {
      ctx.fillStyle = C.mute; ctx.font = '11px system-ui, sans-serif';
      ctx.fillText(Math.round(v.kmh), x+6, y-4);
    }
  });
  ctx.fillStyle = C.warn;
  ctx.beginPath();
  ctx.moveTo(cx, cy-8); ctx.lineTo(cx-5, cy+6);
  ctx.lineTo(cx+5, cy+6); ctx.closePath(); ctx.fill();
}

function renderLane(s) {
  const c = $('lanevis'); const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  const lw = s.lane_width || 3.5;
  const cx = w/2;
  const scale = (w*0.8) / Math.max(lw, 3.5);
  ctx.fillStyle = '#1c2330';
  ctx.fillRect(cx - lw*scale/2, h*0.2, lw*scale, h*0.6);
  ctx.strokeStyle = C.axis; ctx.setLineDash([6,6]);
  ctx.beginPath(); ctx.moveTo(cx, h*0.15); ctx.lineTo(cx, h*0.85); ctx.stroke();
  ctx.setLineDash([]);
  const off = clamp(s.lane_offset || 0, -lw/2, lw/2);
  const vx = cx + off*scale;
  ctx.fillStyle = s.on_road ? C.green : C.crit;
  ctx.beginPath();
  if (ctx.roundRect) { ctx.roundRect(vx-8, h*0.45, 16, 22, 4); ctx.fill(); }
  else ctx.fillRect(vx-8, h*0.45, 16, 22);
}

function renderTrail(pts) {
  const c = $('trail'); const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  if (!pts.length) return;
  const xs = pts.map(p=>p[0]), ys = pts.map(p=>p[1]);
  const minX=Math.min(...xs), maxX=Math.max(...xs);
  const minY=Math.min(...ys), maxY=Math.max(...ys);
  const pad=16;
  const dx=(maxX-minX)||1, dy=(maxY-minY)||1;
  const sc = Math.min((w-pad*2)/dx, (h-pad*2)/dy);
  ctx.strokeStyle = C.blue; ctx.lineWidth = 2; ctx.lineJoin='round';
  ctx.beginPath();
  pts.forEach((p,i) => {
    const x = pad+(p[0]-minX)*sc, y = pad+(p[1]-minY)*sc;
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.stroke();
  const last = pts[pts.length-1];
  ctx.fillStyle = C.warn; ctx.beginPath();
  ctx.arc(pad+(last[0]-minX)*sc, pad+(last[1]-minY)*sc, 4, 0, Math.PI*2);
  ctx.fill();
}

function renderColl(vals) {
  const c = $('collchart'); const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle = C.grid;
  ctx.beginPath(); ctx.moveTo(0, h-6.5); ctx.lineTo(w, h-6.5); ctx.stroke();
  if (!vals.length) return;
  const maxV = Math.max(1, ...vals);
  ctx.strokeStyle = C.red; ctx.lineWidth = 2; ctx.beginPath();
  vals.forEach((v,i) => {
    const x = (i/(vals.length-1||1))*w;
    const y = h - (v/maxV)*h*0.9 - 6;
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.stroke();
}

/* ------------------------------------------------------- sessions page */
let SESS = [];
async function loadSessions() {
  try {
    const r = await fetch('/sessions');
    SESS = await r.json();
  } catch (e) { SESS = []; }
  renderSessTable(); renderHist();
}
setInterval(() => { if (!$('view-stats').hidden) loadSessions(); }, 10000);

function fmtDur(sec) {
  sec = Math.max(0, Math.floor(sec));
  const m = Math.floor(sec/60), s = sec%60;
  return (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}

function renderSessTable() {
  const tb = $('sess-body'); if (!tb) return;
  const empty = $('sess-empty');
  if (!SESS.length) {
    tb.innerHTML = '';
    empty.style.display = 'block';
    return;
  }
  empty.style.display = 'none';
  let html = '', lastDate = '';
  SESS.forEach(s => {
    if (s.date !== lastDate) {
      html += '<tr class="day"><td colspan="9">'+s.date+'</td></tr>';
      lastDate = s.date;
    }
    const live = s.current
      ? ' <span class="badge">'+t('badge_live')+'</span>' : '';
    html += '<tr'+(s.current?' class="live"':'')+'>'
      + '<td>'+(s.time||'').replace(/-/g,':')+live+'</td>'
      + '<td>'+(s.map||'-')+'</td>'
      + '<td>'+fmtDur(s.trip_sec||0)+'</td>'
      + '<td class="num">'+fmt(s.trip_km||0,2)+'</td>'
      + '<td class="num'+((s.crashes||0)>0?' warnnum':'')+'">'+(s.crashes||0)+'</td>'
      + '<td class="num'+((s.out_of_lane||0)>0?' warnnum':'')+'">'+(s.out_of_lane||0)+'</td>'
      + '<td class="num">'+(s.red_light_violations||0)+'</td>'
      + '<td class="num">'+(s.hard_brakes||0)+'</td>'
      + '<td class="num">'+fmt(s.speed_compliance==null?100:s.speed_compliance,0)+'%</td>'
      + '</tr>';
  });
  tb.innerHTML = html;
}

let HIST_LAYOUT = null;
function renderHist() {
  const c = $('histchart'); if (!c) return;
  const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  const list = SESS.slice(0, 20).slice().reverse(); // oldest -> newest
  const padL = 26, padB = 30, padT = 16;
  ctx.strokeStyle = C.grid;
  ctx.beginPath(); ctx.moveTo(padL, h-padB+0.5); ctx.lineTo(w-4, h-padB+0.5);
  ctx.stroke();
  if (!list.length) { HIST_LAYOUT = null; return; }
  const maxV = Math.max(1,
    ...list.map(s => Math.max(s.crashes||0, s.out_of_lane||0)));
  const plotH = h - padB - padT;
  const gw = (w - padL - 8) / list.length;
  const bw = Math.max(4, Math.min(20, (gw - 12) / 2));
  ctx.font = '11px system-ui, sans-serif';
  let lastDate = '';
  list.forEach((s, i) => {
    const gx = padL + i*gw + gw/2;
    const pairs = [[s.crashes||0, C.red, gx - bw - 1],
                   [s.out_of_lane||0, C.blue, gx + 1]];
    pairs.forEach(([v, col, x]) => {
      const bh = (v / maxV) * plotH;
      ctx.fillStyle = col;
      ctx.beginPath();
      if (ctx.roundRect)
        ctx.roundRect(x, h - padB - bh, bw, Math.max(bh, v>0?3:0), [4,4,0,0]);
      else ctx.rect(x, h - padB - bh, bw, bh);
      ctx.fill();
      if (v > 0) {
        ctx.fillStyle = C.mute; ctx.textAlign = 'center';
        ctx.fillText(v, x + bw/2, h - padB - bh - 4);
      }
    });
    ctx.fillStyle = C.mute; ctx.textAlign = 'center';
    ctx.fillText((s.time||'').slice(0,5).replace('-',':'), gx, h - padB + 13);
    if (s.date !== lastDate) {
      ctx.fillText((s.date||'').slice(5), gx, h - padB + 25);
      lastDate = s.date;
    }
  });
  HIST_LAYOUT = {padL, gw, list};
}
(function(){
  const c = $('histchart'); if (!c) return;
  c.addEventListener('mousemove', e => {
    if (!HIST_LAYOUT) return;
    const r = c.getBoundingClientRect();
    const i = Math.floor((e.clientX - r.left - HIST_LAYOUT.padL) / HIST_LAYOUT.gw);
    const s = HIST_LAYOUT.list[i];
    if (!s) { tooltipEl.style.display = 'none'; return; }
    tooltipEl.innerHTML =
      '<div class="th">'+s.date+' '+(s.time||'').replace(/-/g,':')+'</div>'
      + '<div class="tr"><i style="background:'+C.red+'"></i>'+t('tile_crash')
      + '<span class="tv">'+(s.crashes||0)+'</span></div>'
      + '<div class="tr"><i style="background:'+C.blue+'"></i>'+t('tile_lane')
      + '<span class="tv">'+(s.out_of_lane||0)+'</span></div>';
    tooltipEl.style.display = 'block';
    placeTooltip(e);
  });
  c.addEventListener('mouseleave', () => { tooltipEl.style.display = 'none'; });
})();

/* --------------------------------------------------------- pedal tuning */
const TUNE_DEF = {throttle_max:0.8, throttle_curve:1.8,
                  brake_max:1.0, brake_curve:2.2};
let TUNE = Object.assign({}, TUNE_DEF);
let tuneTimer = null, tuneDragging = false;

function tuneToUI() {
  $('r-tmax').value = TUNE.throttle_max;
  $('r-tcurve').value = TUNE.throttle_curve;
  $('r-bmax').value = TUNE.brake_max;
  $('r-bcurve').value = TUNE.brake_curve;
  $('v-tmax').textContent = Math.round(TUNE.throttle_max*100)+'%';
  $('v-tcurve').textContent = fmt(TUNE.throttle_curve,1);
  $('v-bmax').textContent = Math.round(TUNE.brake_max*100)+'%';
  $('v-bcurve').textContent = fmt(TUNE.brake_curve,1);
  renderTuneCurve();
}
function renderTuneCurve() {
  const c = $('tunecurve'); if (!c) return;
  const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  ctx.strokeRect(8.5, 4.5, w-13, h-13);
  const drawCurve = (mx, cv, col) => {
    ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i <= 60; i++) {
      const p = i/60, v = mx * Math.pow(p, cv);
      const x = 8 + p*(w-13), y = h-8 - v*(h-13);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  };
  drawCurve(TUNE.throttle_max, TUNE.throttle_curve, C.green);
  drawCurve(TUNE.brake_max, TUNE.brake_curve, C.red);
}
function postTune() {
  clearTimeout(tuneTimer);
  tuneTimer = setTimeout(() => {
    fetch('/tuning', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(TUNE)}).catch(()=>{});
  }, 250);
}
async function loadTune() {
  if (tuneDragging) return;
  try {
    const r = await fetch('/tuning');
    TUNE = Object.assign({}, TUNE_DEF, await r.json());
    tuneToUI();
  } catch (e) {}
}
[['r-tmax','throttle_max'],['r-tcurve','throttle_curve'],
 ['r-bmax','brake_max'],['r-bcurve','brake_curve']].forEach(([id,key]) => {
  const el = $(id);
  el.addEventListener('input', () => {
    TUNE[key] = parseFloat(el.value); tuneToUI(); postTune();
  });
  el.addEventListener('pointerdown', () => { tuneDragging = true; });
  el.addEventListener('pointerup', () => { tuneDragging = false; });
});
$('tune-reset').onclick = () => {
  TUNE = Object.assign({}, TUNE_DEF); tuneToUI(); postTune();
};
loadTune();
setInterval(loadTune, 5000);   // reflect in-sim [ / ] key changes


/* ---------------------------------------------------- vehicle performance */
// Brakes / power / top speed multipliers (vehicle_tuning.py); the dashboard's
// own worker thread applies them to the cars. / 車両性能の倍率。
const VT = {params:null, values:{}, defaults:{}, status:null, dragging:false, timer:null};
function vtBuild() {
  const box = $('vt-rows');
  box.innerHTML = '';
  for (const p of VT.params) {
    const row = document.createElement('div');
    row.className = 'frow';
    row.innerHTML =
      `<label for="vt-r-${p.key}" title="${escV(p.help)}">${escV(p[LANG] || p.en)}</label>` +
      `<b id="vt-v-${p.key}"></b>` +
      `<button class="fr" id="vt-x-${p.key}" title="${escV(t('ff_reset'))}" aria-label="${escV(t('ff_reset') + ': ' + (p[LANG] || p.en))}">↺</button>` +
      `<input type="range" id="vt-r-${p.key}" min="${p.min}" max="${p.max}" step="${p.step}">` +
      (p.help ? `<div class="fhelp">${escV(p.help)}</div>` : '');
    box.appendChild(row);
    const r = row.querySelector('input');
    r.addEventListener('input', () => vtSet(p.key, parseFloat(r.value)));
    r.addEventListener('pointerdown', () => { VT.dragging = true; });
    r.addEventListener('pointerup', () => { VT.dragging = false; });
    row.querySelector('.fr').onclick = () => vtSet(p.key, VT.defaults[p.key]);
  }
  vtToUI();
}
function vtToUI() {
  for (const p of VT.params) {
    const v = VT.values[p.key], r = $('vt-r-' + p.key);
    if (!r) continue;
    if (!VT.dragging) r.value = v;
    $('vt-v-' + p.key).textContent = p.step >= 1 ? String(v) : '×' + Number(v).toFixed(2);
    $('vt-x-' + p.key).classList.toggle('on', Math.abs(v - VT.defaults[p.key]) > p.step / 2);
  }
  const s = VT.status, cars = (s && s.cars) || [];
  const head = `<tr><th>${t('vt_car')}</th><th>${t('vt_brake')}</th><th>${t('vt_torque')}</th><th>${t('vt_top')}</th></tr>`;
  const cell = (now, base, unit) => now === base
    ? `<td class="num">${now} ${unit}</td>`
    : `<td class="num">${now} ${unit} <span style="color:var(--mute)">(${base})</span></td>`;
  $('vt-cars').innerHTML = head + (cars.length ? cars.map(c =>
    `<tr><td>${escV(c.type.replace('vehicle.', '').replace('delivery.', ''))}${c.hero ? ' · <b>' + t('vt_me') + '</b>' : ''}` +
    `${c.waiting ? ' <span class="pill yellow" title="' + escV(t('vt_wait_tip')) + '">' + t('vt_wait') + '</span>' : ''}</td>` +
    cell(c.brake_nm, c.brake_base, 'Nm') + cell(c.torque_nm, c.torque_base, 'Nm') +
    cell(c.top_kmh, c.top_base, 'km/h') + '</tr>').join('')
    : `<tr><td class="note" colspan="4">${t('vt_none')}</td></tr>`);
}
function vtSet(key, v) {
  VT.values[key] = v; vtToUI();
  clearTimeout(VT.timer);
  VT.timer = setTimeout(() => { VT.timer = null; vtPost({values: VT.values}); }, 200);
}
async function vtPost(body) {
  try {
    const r = await fetch('/vehicle_tuning', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
    vtApply(await r.json());
  } catch (e) {}
}
function vtApply(d) {
  if (!d || !d.params) return;
  const first = !VT.params;
  VT.params = d.params; VT.defaults = d.defaults; VT.status = d.status;
  if (!VT.dragging && !VT.timer) VT.values = d.values;
  if (first) vtBuild(); else vtToUI();
}
async function vtTick() {
  if (!secVisible('driving') && VT.params) return;
  try { vtApply(await (await fetch('/vehicle_tuning')).json()); } catch (e) {}
}
$('vt-reset').onclick = () => vtPost({reset: true});
$('vt-stock').onclick = () => vtPost({neutral: true});
setInterval(vtTick, 1000);
vtTick();

/* ------------------------------------------------------------- pod view */
// The LastMile pod: shell view for every pod, and the showcase -- one pod
// taken over and scrubbed through the HTML's whole cycle with the slider.
// LastMile ポッド：外殻表示と、1 台を乗っ取って HTML の全手順をスライダーで再生。
const PD = {st: null, dragging: false, sendT: 0, lastBuilt: ''};
async function podCmd(body) {
  try {
    const r = await fetch('/delivery/pod', {method:'POST', headers:{'Content-Type':'application/json'},
                                            body: JSON.stringify(body)});
    const d = await r.json();
    if (!d.ok) $('pd-msg').textContent = d.error || 'error';
  } catch (e) { $('pd-msg').textContent = String(e); }
}
function podFmt(t) { const s = Math.max(0, t); return Math.floor(s / 60) + ':' + String(Math.floor(s % 60)).padStart(2, '0'); }
function podBuild(p) {
  // step ticks and the clickable step list (built once per step table)
  const key = JSON.stringify(p.steps.map(s => s.id));
  if (key === PD.lastBuilt) return;
  PD.lastBuilt = key;
  $('pd-scrub').max = p.total;
  $('pd-ticks').innerHTML = p.steps.map(s =>
    `<span style="left:${(s.t0 / p.total * 100).toFixed(2)}%;background:${s.scenario === 'drone' ? '#3987e5' : '#ff8c00'}"></span>`).join('');
  const list = sc => p.steps.filter(s => s.scenario === sc).map((s, i) =>
    `<button class="pd-step" data-t="${s.t0}" id="pd-s-${s.id}"><b>${i + 1}</b>${escV(s.title)}<i>${Math.round(s.t1 - s.t0)} s</i></button>`).join('');
  $('pd-steps-drone').innerHTML = list('drone');
  $('pd-steps-direct').innerHTML = list('direct');
  document.querySelectorAll('.pd-step').forEach(b => b.onclick = () => {
    if (!PD.st || !PD.st.showcase) { $('pd-msg').textContent = t('pd_take_first'); return; }
    podCmd({cmd: 'pod_time', t: parseFloat(b.dataset.t)});
  });
}
function podRender() {
  const s = PD.st, p = s && s.pod;
  document.querySelectorAll('[data-tooltips]').forEach(b =>
    b.setAttribute('aria-pressed', (s && s.running && (b.dataset.tooltips === '1') === !!s.tooltips) ? 'true' : 'false'));
  $('pd-run').textContent = !s || !s.available ? t('dr_none') : s.running ? t('dr_live') : t('dr_stopped');
  if (!p) { $('pd-none').hidden = false; $('pd-body').hidden = true; return; }
  $('pd-none').hidden = !!p.vans.length; $('pd-body').hidden = !p.vans.length;
  if (!p.vans.length) return;
  podBuild(p);
  document.querySelectorAll('[data-shell]').forEach(b =>
    b.setAttribute('aria-pressed', b.dataset.shell === p.shell ? 'true' : 'false'));
  $('pd-shell').textContent = t('pd_shell_' + p.shell);
  const sel = $('pd-van'), cur = sel.value;
  sel.innerHTML = p.vans.map(v => `<option value="${v.id}">${t('pd_pod')} ${v.id} · ${escV(v.state)}${v.free ? '' : ' · ' + t('pd_busy')}</option>`).join('');
  if (cur && p.vans.some(v => String(v.id) === cur)) sel.value = cur;
  else if (p.showcase) sel.value = p.showcase.van;
  const sc = p.showcase;
  $('pd-take').hidden = !!sc; $('pd-release').hidden = !sc;
  $('pd-play').disabled = !sc; $('pd-scrub').disabled = !sc; $('pd-speed').disabled = !sc;
  $('pd-on').textContent = sc ? t('pd_pod') + ' ' + sc.van : '—';
  $('pd-step').textContent = sc ? sc.title : '—';
  $('pd-msg').textContent = p.msg || (sc ? '' : t('pd_hint'));
  if (sc) {
    if (!PD.dragging) $('pd-scrub').value = sc.t;
    $('pd-time').textContent = podFmt(sc.t) + ' / ' + podFmt(p.total);
    $('pd-play').innerHTML = (sc.playing ? '<svg class="ico" viewBox="0 0 24 24" aria-hidden="true"><path d="M6 4h4v16H6zM14 4h4v16h-4z" fill="currentColor" stroke="none"/></svg>' : '<svg class="ico" viewBox="0 0 24 24" aria-hidden="true"><path d="M7 4v16l13-8z" fill="currentColor" stroke="none"/></svg>') + escV(sc.playing ? t('pd_pause') : t('pd_play'));
    $('pd-play').dataset.playing = sc.playing ? '1' : '0';
    $('pd-speed').value = String(sc.speed);
    document.querySelectorAll('.pd-step').forEach(b => b.classList.toggle('on', b.id === 'pd-s-' + sc.step));
    $('pd-scen').textContent = sc.scenario === 'drone' ? t('pd_scen_drone') : t('pd_scen_direct');
  } else {
    $('pd-time').textContent = '—';
    $('pd-scen').textContent = '—';
    document.querySelectorAll('.pd-step').forEach(b => b.classList.remove('on'));
  }
  // deck: drones on their slots, magazine packs / 甲板と電池マガジン
  const slotName = sl => sl === 1 ? t('pd_lift') : 'B' + (sl === 0 ? 1 : sl);
  $('pd-deck').innerHTML = `<tr><th>${t('pd_pod')}</th><th>${t('pd_deck')}</th><th>${t('pd_mag')}</th></tr>` +
    p.vans.map(v => `<tr><td class="num">${v.id}</td><td>${v.drones.map(d => d ? `<span class="pd-chip"><b>D${d.id}</b> ${slotName(d.slot)} <span class="batt"><i style="width:${d.battery}%;background:${d.battery < 30 ? '#d03b3b' : d.battery < 60 ? '#fab219' : '#199e70'}"></i></span>${d.battery}%</span>` : '').join(' ')}</td>` +
      `<td>${v.packs.map(k => k === null ? '<span class="pd-pack empty">—</span>' : `<span class="pd-pack">${k}%</span>`).join('')}</td></tr>`).join('');
}
async function podTick() {
  if ($('view-drone').hidden && !secVisible('delivery')) return;
  try { PD.st = await (await fetch('/delivery/state')).json(); } catch (e) { PD.st = null; }
  podRender();
}
document.querySelectorAll('[data-shell]').forEach(b => b.onclick = () => podCmd({cmd: 'shell', mode: b.dataset.shell}));
document.querySelectorAll('[data-tooltips]').forEach(b => b.onclick = () => podCmd({cmd: 'tooltips', on: b.dataset.tooltips === '1'}));
$('pd-take').onclick = () => podCmd({cmd: 'pod_takeover', van: parseInt($('pd-van').value), t: 0,
                                     speed: parseFloat($('pd-speed').value)});
$('pd-release').onclick = () => podCmd({cmd: 'pod_release'});
$('pd-play').onclick = () => podCmd({cmd: 'pod_play', playing: $('pd-play').dataset.playing !== '1',
                                     speed: parseFloat($('pd-speed').value)});
$('pd-speed').onchange = () => podCmd({cmd: 'pod_play', playing: $('pd-play').dataset.playing === '1',
                                       speed: parseFloat($('pd-speed').value)});
$('pd-scrub').addEventListener('pointerdown', () => { PD.dragging = true; });
$('pd-scrub').addEventListener('pointerup', () => { PD.dragging = false; });
$('pd-scrub').addEventListener('input', () => {
  const v = parseFloat($('pd-scrub').value);
  $('pd-time').textContent = podFmt(v) + ' / ' + podFmt(parseFloat($('pd-scrub').max));
  const now = Date.now();
  if (now - PD.sendT > 120) { PD.sendT = now; podCmd({cmd: 'pod_time', t: v}); }
});
$('pd-scrub').addEventListener('change', () => podCmd({cmd: 'pod_time', t: parseFloat($('pd-scrub').value)}));
setInterval(podTick, 400);

/* -------------------------------------------------------- driver alerts */
// Same rules as the in-sim alerts (ALERT_RULES in the G923 script):
// lane crossing, collision, red light, hard brake, TTC < 1.5 s, limit+10.
let AC = null, alertTimer = null;
const AL_PREV = {lane:null, crash:null, rlv:null, hb:null};
const AL_COOL = {};

function beep(freq, dur, times) {
  try {
    AC = AC || new (window.AudioContext || window.webkitAudioContext)();
    if (AC.state === 'suspended') AC.resume();
    for (let i = 0; i < times; i++) {
      const o = AC.createOscillator(), g = AC.createGain();
      o.type = 'sine'; o.frequency.value = freq;
      const t0 = AC.currentTime + i * (dur + 0.07);
      g.gain.setValueAtTime(0.0001, t0);
      g.gain.exponentialRampToValueAtTime(0.25, t0 + 0.01);
      g.gain.exponentialRampToValueAtTime(0.0001, t0 + dur);
      o.connect(g); g.connect(AC.destination);
      o.start(t0); o.stop(t0 + dur + 0.02);
    }
  } catch (e) {}
}
function showAlert(cat, text, severity, ms, cool) {
  ms = ms || 2500; cool = cool || 3000;
  const now = Date.now();
  if (now < (AL_COOL[cat] || 0)) return;
  AL_COOL[cat] = now + cool;
  const el = $('alertbar');
  el.textContent = text;
  el.className = severity + ' show';
  beep(severity === 'danger' ? 880 : 660, 0.14,
       severity === 'danger' ? 2 : 1);
  clearTimeout(alertTimer);
  alertTimer = setTimeout(() => { el.className = severity; }, ms);
}
function checkAlerts(s) {
  if (AL_PREV.lane !== null && s.hero_found) {
    if (s.lane_invasions > AL_PREV.lane)
      showAlert('lane', t('al_lane'), 'warn');
    if (s.collision_count > AL_PREV.crash)
      showAlert('crash', t('al_crash'), 'danger', 3500, 2000);
    if (s.red_light_violations > AL_PREV.rlv)
      showAlert('rlv', t('al_redlight'), 'danger');
    if (s.hard_brake_count > AL_PREV.hb)
      showAlert('hb', t('al_hardbrake'), 'warn');
  }
  AL_PREV.lane = s.lane_invasions; AL_PREV.crash = s.collision_count;
  AL_PREV.rlv = s.red_light_violations; AL_PREV.hb = s.hard_brake_count;
  if (s.ttc > 0 && s.ttc < 1.5 && s.speed_kmh > 20)
    showAlert('fcw', t('al_fcw') + ' (TTC ' + fmt(s.ttc,1) + 's)',
              'danger', 2000, 3000);
  if (s.speed_limit_kmh > 0 && s.speed_kmh > s.speed_limit_kmh + 10)
    showAlert('speed', t('al_speeding') + ' ' + fmt(s.speed_kmh,0)
              + ' / ' + fmt(s.speed_limit_kmh,0) + ' km/h', 'warn',
              2500, 6000);
}
if (new URLSearchParams(location.search).get('alerttest'))
  setTimeout(() => showAlert('test', t('al_test'), 'danger',
                             3000, 0), 800);

/* --------------------------------------------------------------- update */
function pillFor(delta) {
  if (delta > 10) return ['red', t('pill_over')];
  if (delta > 0) return ['yellow','+'];
  return ['green', t('pill_ok')];
}

async function tick() {
  try {
    const r = await fetch('/telemetry');
    const s = await r.json();
    $('status-dot').className = 'dot'+(s.connected?'':' bad');
    $('status').textContent = s.connected
      ? (s.hero_found ? t('st_conn_hero') : t('st_conn_wait'))
      : t('st_disconn');
    $('map').textContent = s.map || '-';
    $('vtype').textContent = (s.vehicle_type || '-').replace('vehicle.','');
    $('mass').textContent = fmt(s.mass_kg, 0);
    $('nearby').textContent = s.nearby_vehicles;
    $('fps').textContent = fmt(s.sim_fps, 1);
    $('simt').textContent = fmt(s.sim_time_sec, 0);

    // Speed card
    $('speed').textContent = fmt(s.speed_kmh, 0);
    $('splimit').textContent = s.speed_limit_kmh>0 ? fmt(s.speed_limit_kmh,0) : '-';
    $('avgkmh').textContent = fmt(s.avg_kmh,0);
    const [pc,pt] = pillFor(s.speed_delta_kmh||0);
    const pill = $('spd-pill'); pill.className='pill '+pc; pill.textContent=pt;
    const gmap = {'-1':'R','0':'N'};
    $('gear').textContent = gmap[s.gear] !== undefined ? gmap[s.gear] : s.gear;
    $('heading').textContent = fmt(s.heading_deg,1)+'°';
    $('i-rev').className = 'ind'+(s.reverse?' on':'');
    $('i-hb').className = 'ind'+(s.hand_brake?' on':'');
    $('i-abs').className = 'ind'+(s.abs_active?' on':'');
    $('i-tcs').className = 'ind'+(s.tcs_active?' on':'');

    // Controls
    $('t-val').textContent = Math.round(s.throttle*100)+'%';
    $('t-bar').style.width = (s.throttle*100)+'%';
    $('b-val').textContent = Math.round(s.brake*100)+'%';
    $('b-bar').style.width = (s.brake*100)+'%';
    $('s-val').textContent = fmt(s.steer,2);
    const sp = clamp(s.steer, -1, 1);
    if (sp>=0) { $('s-bar').style.left='50%'; $('s-bar').style.width=(sp*50)+'%'; }
    else { $('s-bar').style.left=(50+sp*50)+'%'; $('s-bar').style.width=(-sp*50)+'%'; }

    // Safety
    $('fdist').textContent = s.front_distance>0 ? fmt(s.front_distance,1) : '—';
    $('thw').textContent = s.thw>0 ? fmt(s.thw,1) : '—';
    $('thw').style.color = (s.thw>0 && s.thw<1) ? C.crit
                         : (s.thw>0 && s.thw<2) ? C.warn : C.ink;
    $('ttc').textContent = s.ttc>0 ? fmt(s.ttc,1) : '—';
    $('ttc').style.color = (s.ttc>0 && s.ttc<2) ? C.crit
                         : (s.ttc>0 && s.ttc<4) ? C.warn : C.ink;
    $('collcnt').textContent = s.collision_count||0;
    $('collcnt2').textContent = s.collision_count||0;
    $('collcnt').style.color = (s.collision_count||0)>0 ? C.crit : C.ink;
    $('hbcnt').textContent = s.hard_brake_count;
    $('hbcnt2').textContent = s.hard_brake_count;
    const tls = s.at_traffic_light ? s.traffic_light : 'None';
    $('tl').textContent = (I18N[LANG] && I18N[LANG]['tl_'+tls]) || tls;
    $('tl').style.color = tls==='Red'?C.crit : tls==='Yellow'?C.warn
                        : tls==='Green'?C.good : C.mute;
    const stab = Math.round((s.stability||0)*100);
    $('stab').textContent = stab+'%';
    $('stab-bar').style.width = stab+'%';
    $('stab-bar').style.background = stab>70?C.green:stab>40?C.warn:C.crit;

    // Dynamics
    $('lonA').textContent = fmt(s.lon_accel);
    $('latA').textContent = fmt(s.lat_accel);
    $('verA').textContent = fmt(s.vert_accel);
    $('jerk').textContent = fmt(s.jerk);
    $('gfr').textContent  = fmt(s.g_force);
    $('yaw').textContent  = fmt(s.yaw_rate_deg,1)+'°/s';
    $('slip').textContent = fmt(s.slip_angle_deg,1)+'°';
    $('curv').textContent = fmt(s.curvature,4);

    // Driver performance
    $('sdlp').textContent = fmt(s.sdlp,3)+' m';
    $('sdlp').style.color = s.sdlp>0.4?C.crit:s.sdlp>0.25?C.warn:C.ink;
    $('srr').textContent = fmt(s.steer_reversal_rate,0)+' /min';
    $('jerkrms').textContent = fmt(s.jerk_rms,2)+' m/s³';
    $('laneinv').textContent = s.lane_invasions;
    $('rlv').textContent = s.red_light_violations;
    $('rlv').style.color = s.red_light_violations>0?C.crit:C.ink;
    const comp = clamp(s.speed_compliance||100, 0, 100);
    $('comp').textContent = fmt(comp,1)+'%';
    $('comp-bar').style.width = comp+'%';
    $('comp-bar').style.background = comp>90?C.green:comp>70?C.warn:C.crit;

    // Driver alerts (banner + beep) / 運転警告（バナー＋ビープ）
    checkAlerts(s);

    // Stats tiles (current session)
    $('st-crash').textContent = s.collision_count||0;
    $('st-lane').textContent = s.lane_invasions||0;
    $('st-rlv').textContent = s.red_light_violations||0;
    $('st-hb').textContent = s.hard_brake_count||0;
    $('st-dist').textContent = fmt(s.trip_km,2);
    $('st-dur').textContent = fmtDur(s.trip_sec);
    $('st-avg').textContent = fmt(s.avg_kmh,0);
    $('st-comp').textContent = fmt(comp,0);

    // Events
    const evs = s.events||[];
    $('events').innerHTML = evs.length
      ? evs.map(e => '<div class="ev ev-'+e.type+'"><span class="t">'+e.t+
          '</span><span class="ty">'+t('ev_'+e.type)+
          '</span><span class="m">'+e.msg+'</span></div>').join('')
      : '<div class="ev"><span class="t">—</span><span class="ty">'+
        t('ev_none')+'</span></div>';

    // Lane
    $('onroad').textContent = s.on_road ? t('yes') : t('offroad');
    $('onroad').style.color = s.on_road ? C.good : C.crit;
    $('road').textContent = s.road_id;
    $('lane').textContent = s.lane_id;
    $('lw').textContent = fmt(s.lane_width,2)+' m';
    $('lo').textContent = fmt(s.lane_offset,2)+' m';
    $('junc').textContent = s.at_junction ? t('yes') : t('no');

    // Weather
    const W = s.weather||{};
    $('wcl').textContent = Math.round(W.cloudiness||0);
    $('wcl-bar').style.width = (W.cloudiness||0)+'%';
    $('wrn').textContent = Math.round(W.precipitation||0);
    $('wrn-bar').style.width = (W.precipitation||0)+'%';
    $('wwt').textContent = Math.round(W.wetness||0);
    $('wwt-bar').style.width = (W.wetness||0)+'%';
    $('wfg').textContent = Math.round(W.fog_density||0);
    $('wfg-bar').style.width = (W.fog_density||0)+'%';
    $('wwd').textContent = Math.round(W.wind_intensity||0);
    $('wsa').textContent = fmt(W.sun_altitude,0)+'°';

    // Trip
    $('tripkm').textContent = fmt(s.trip_km,2);
    $('tripsec').textContent = fmtDur(s.trip_sec);
    $('avgkmh2').textContent = fmt(s.avg_kmh,0)+' km/h';
    $('cpeak').textContent = fmt(s.collision_peak,1);

    // GNSS / IMU
    $('x').textContent = fmt(s.location.x,1);
    $('y').textContent = fmt(s.location.y,1);
    $('z').textContent = fmt(s.location.z,1);
    $('lat').textContent = fmt(s.gnss.lat,6);
    $('lon').textContent = fmt(s.gnss.lon,6);
    $('ax').textContent = fmt(s.imu.ax,1);
    $('ay').textContent = fmt(s.imu.ay,1);
    $('az').textContent = fmt(s.imu.az,1);
    $('gx').textContent = fmt(s.imu.gx,2);
    $('gy').textContent = fmt(s.imu.gy,2);
    $('gz').textContent = fmt(s.imu.gz,2);
    $('compass').textContent = fmt(s.compass_deg,1)+'°';

    // Live chart legend values ("graph parameters")
    $('lv-speed').textContent = fmt(s.speed_kmh,0);
    $('lv-limit').textContent = s.speed_limit_kmh>0 ? fmt(s.speed_limit_kmh,0) : '–';
    $('lv-avg').textContent = fmt(s.avg_kmh,0);
    $('lv-lon').textContent = fmt(s.lon_accel,1);
    $('lv-lat').textContent = fmt(s.lat_accel,1);
    $('lv-jerk').textContent = fmt(s.jerk,1);
    $('lv-yaw').textContent = fmt(s.yaw_rate_deg,1);
    $('lv-steer100').textContent = fmt((s.steer||0)*100,0);
    $('lv-laneoff').textContent = fmt(s.lane_offset,2)+' m';

    // History
    push(H.speed, s.speed_kmh||0);
    push(H.limit, s.speed_limit_kmh||0);
    push(H.avg,   s.avg_kmh||0);
    push(H.throttle, (s.throttle||0)*100);
    push(H.brake,    (s.brake||0)*100);
    push(H.steer,    (s.steer||0)*100);
    push(H.lon, s.lon_accel||0);
    push(H.lat, s.lat_accel||0);
    push(H.jerk, clamp(s.jerk||0, -20, 20));
    push(H.yaw, s.yaw_rate_deg||0);
    push(H.steerY, (s.steer||0)*100);
    push(H.ttc, s.ttc>0 ? Math.min(s.ttc, 15) : 15);
    push(H.thw, s.thw>0 ? Math.min(s.thw, 15) : 15);
    const lw2 = (s.lane_width||3.5)/2;
    push(H.laneoff, clamp(s.lane_offset||0, -3, 3));
    push(H.laneEdgeP, lw2);
    push(H.laneEdgeN, -lw2);
    H.gg.push([s.lon_accel||0, s.lat_accel||0]); if (H.gg.length>150) H.gg.shift();

    if (!$('view-dash').hidden) {
      const spdMax = Math.max(60, ...H.speed, ...H.limit) * 1.1;
      renderChart('speedhist',
        [[H.limit,C.yellow],[H.avg,C.green],[H.speed,C.blue]],
        [0, spdMax], 'km/h');
      renderChart('ctrlhist',
        [[H.throttle,C.green],[H.brake,C.red],[H.steer,C.yellow]],
        [-100, 100], '%');
      renderChart('acchist',
        [[H.lon,C.blue],[H.lat,C.red],[H.jerk,C.yellow]],
        [-10, 10], 'm/s²');
      renderChart('yawhist',
        [[H.yaw,C.blue],[H.steerY,C.yellow]],
        [-90, 90], '°/s');
      renderChart('ttchist',
        [[H.thw,C.blue],[H.ttc,C.red]], [0, 15], 's');
      renderChart('lanehist',
        [[H.laneEdgeP,C.red],[H.laneEdgeN,C.red],[H.laneoff,C.blue]],
        [-2.5, 2.5], 'm');
      renderGG();
      renderMinimap(s);
      renderLane(s);
      renderTrail(s.trail||[]);
      renderColl(s.collision_recent||[]);
    }
    if (!$('view-veh').hidden) renderVeh(s);
  } catch (e) {
    $('status-dot').className = 'dot bad';
    $('status').textContent = t('st_disconn');
  }
}
/* ----------------------------------------------------------- drones view */
// delivery_sim.py publishes /delivery/state twice a second and /delivery/map
// once; the map is world metres, x to the right and y down (CARLA's top view).
// delivery_sim.py が状態を毎秒 2 回、地図を一度公開する（x 右・y 下）。
const DR = {map:null, state:null, view:null, drag:null, tipAt:null, loadingMap:false,
            dirTouched:false};
const DR_ST_COL = {docked:'#3987e5', reloading:'#6aa8f0', delivering:'#199e70',
                   to_station:'#c98500', to_van:'#9085e9', charging:'#fab219',
                   standby:'#5fd45f', falling:'#ec835a', crashed:'#d03b3b'};
const drBattCol = b => b > 50 ? '#199e70' : b > 25 ? '#fab219' : '#e66767';
const drEsc = s => String(s == null ? '' : s).replace(/[<>&]/g,
  c => ({'<':'&lt;', '>':'&gt;', '&':'&amp;'})[c]);
const drSt = st => t('st_' + st) === 'st_' + st ? st : t('st_' + st);

async function drLoadMap() {
  if (DR.loadingMap) return;
  DR.loadingMap = true;
  try {
    const r = await fetch('/delivery/map');
    if (r.ok) { const m = await r.json(); if (m && m.roads) { DR.map = m; DR.view = null; } }
  } catch (e) {}
  DR.loadingMap = false;
}

async function drTick() {
  if ($('view-drone').hidden && !secVisible('delivery')) return;
  try { DR.state = await (await fetch('/delivery/state')).json(); }
  catch (e) { DR.state = null; }
  const s = DR.state;
  if (s && s.available && (!DR.map || DR.map.map !== s.map)) await drLoadMap();
  drRender();
}

function drFit() {
  const m = DR.map, cv = $('dmap');
  if (!m) return;
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const pl of m.roads) for (const [x, y] of pl) {
    x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
  }
  const W = cv.clientWidth || 800, H = cv.clientHeight || 600;
  DR.view = {cx:(x0 + x1) / 2, cy:(y0 + y1) / 2,
             s:0.94 * Math.min(W / Math.max(x1 - x0, 1), H / Math.max(y1 - y0, 1))};
}

function drP(x, y) {
  const v = DR.view, cv = $('dmap');
  return [(x - v.cx) * v.s + cv.clientWidth / 2, (y - v.cy) * v.s + cv.clientHeight / 2];
}

function drBox(ctx, x, y, ex, ey, yaw) {
  const r = yaw * Math.PI / 180, c = Math.cos(r), s = Math.sin(r);
  ctx.beginPath();
  [[ex, ey], [-ex, ey], [-ex, -ey], [ex, -ey]].forEach(([a, b], i) => {
    const p = drP(x + a * c - b * s, y + a * s + b * c);
    if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]);
  });
  ctx.closePath();
}

function drArrow(ctx, x, y, dx, dy, col, w) {
  const a = Math.atan2(dy, dx), L = Math.hypot(dx, dy);
  if (L < 1) return;
  ctx.strokeStyle = ctx.fillStyle = col; ctx.lineWidth = w;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke();
  const h = Math.min(10, L * 0.4);
  ctx.beginPath();
  ctx.moveTo(x + dx, y + dy);
  ctx.lineTo(x + dx - h * Math.cos(a - 0.45), y + dy - h * Math.sin(a - 0.45));
  ctx.lineTo(x + dx - h * Math.cos(a + 0.45), y + dy - h * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
}

function drDrawMap() {
  const cv = $('dmap'), dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (W < 30 || H < 30) return;      // hidden (other tab) / 非表示
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
  }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0c1119'; ctx.fillRect(0, 0, W, H);
  const m = DR.map, s = DR.state;
  DR.hits = [];
  if (!m) {
    ctx.fillStyle = C.mute; ctx.font = '13px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'center';
    ctx.fillText(t('dr_nomap'), W / 2, H / 2);
    return;
  }
  if (!DR.view) drFit();
  const v = DR.view;
  // buildings, then roads / 建物、道路
  ctx.fillStyle = '#18202d'; ctx.strokeStyle = '#243044'; ctx.lineWidth = 1;
  for (const [x, y, ex, ey, yaw] of m.buildings) {
    drBox(ctx, x, y, ex, ey, yaw); ctx.fill(); ctx.stroke();
  }
  ctx.strokeStyle = '#33415a'; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  ctx.lineWidth = Math.max(1.2, 3.4 * v.s);
  for (const pl of m.roads) {
    ctx.beginPath();
    pl.forEach(([x, y], i) => { const p = drP(x, y); if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]); });
    ctx.stroke();
  }
  const R = k => Math.max(k, k * v.s / 2.2);     // marker size grows when zoomed in
  // charge station / 充電所
  ctx.fillStyle = '#78ff3c33'; ctx.strokeStyle = '#78ff3c';
  for (const [x, y] of m.station) {
    const p = drP(x, y), r = R(3.5);
    ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r);
  }
  if (m.station.length) {
    const p = drP(m.station[0][0], m.station[0][1]);
    ctx.fillStyle = '#78ff3c'; ctx.font = '600 11px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'left'; ctx.fillText(t('dr_lg_station'), p[0] + 10, p[1] - 8);
    DR.hits.push({x:p[0], y:p[1], r:12, tip:t('dr_lg_station') + ' · ' + m.station.length + ' pads'});
  }
  // EV chargers for the pods / ポッド用 EV 充電器
  for (const [x, y] of m.chargers || []) {
    const p = drP(x, y), r = R(4.5);
    ctx.fillStyle = '#ffd22833'; ctx.strokeStyle = '#ffd228'; ctx.lineWidth = 1.5;
    ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r);
    ctx.fillStyle = '#ffd228'; ctx.font = '700 11px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'center'; ctx.fillText('EV', p[0], p[1] + 3);
    DR.hits.push({x:p[0], y:p[1], r:9, tip:t('dr_lg_ev')});
  }
  if (!s || !s.available) return;
  // drones on the pads, listed beside the station / 充電パッド上の機体
  const parked = (s.drones || []).filter(d => d.status === 'charging' || d.status === 'standby');
  if (parked.length && m.station.length) {
    const p = drP(m.station[0][0], m.station[0][1]);
    ctx.font = '600 11.5px ' + getComputedStyle(document.body).fontFamily; ctx.textAlign = 'left';
    parked.forEach((d, i) => {
      ctx.fillStyle = drBattCol(d.battery);
      ctx.fillText('D' + d.id + ' ' + Math.round(d.battery) + '% ' + drSt(d.status),
                   p[0] + 10, p[1] + 7 + 13 * i);
    });
  }
  // hubs / ドローン地点
  for (const h of s.hubs || []) {
    const p = drP(h.x, h.y), r = R(5);
    ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(Math.PI / 4);
    ctx.strokeStyle = '#dc00dc'; ctx.lineWidth = 2; ctx.strokeRect(-r, -r, 2 * r, 2 * r);
    ctx.restore();
    DR.hits.push({x:p[0], y:p[1], r:8, tip:t('dr_lg_hub') + ' · van ' + h.van});
  }
  // orders / 注文
  for (const o of s.orders || []) {
    const p = drP(o.x, o.y), r = R(3.2);
    let col = o.kind === 'drone' ? '#00a0ff' : '#ff8c00';
    if (o.status === 'delivered') col = '#0ca30c';
    if (o.status === 'failed') col = '#d03b3b';
    ctx.globalAlpha = o.status === 'delivered' ? 0.55 : 1;
    ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill();
    if (o.status === 'queued' || o.status === 'in_flight') {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = o.status === 'in_flight' ? 2 : 1;
      ctx.beginPath(); ctx.arc(p[0], p[1], r + 2.5, 0, 7); ctx.stroke();
    }
    if (o.status === 'failed') {
      ctx.strokeStyle = '#ff9c9c'; ctx.lineWidth = 1.6; ctx.beginPath();
      ctx.moveTo(p[0] - r - 2, p[1] - r - 2); ctx.lineTo(p[0] + r + 2, p[1] + r + 2);
      ctx.moveTo(p[0] + r + 2, p[1] - r - 2); ctx.lineTo(p[0] - r - 2, p[1] + r + 2); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    DR.hits.push({x:p[0], y:p[1], r:7, tip:'#' + o.id + ' ' + (o.kind === 'drone' ? 'DRONE' : 'F2F') +
      ' · ' + t('ost_' + o.status) + (o.van ? ' · van ' + o.van : '') + (o.reason ? ' · ' + o.reason : '')});
  }
  // other cars / 他の車
  ctx.fillStyle = '#8a96aa';     // at least 7 x 3.5 px, however far out / 縮小時も最小サイズ
  for (const [x, y, yaw] of s.cars || []) {
    drBox(ctx, x, y, Math.max(2.3, 3.5 / v.s), Math.max(1.0, 1.75 / v.s), yaw); ctx.fill();
  }
  // pods / 配送ポッド
  const vanPos = {};
  for (const van of s.vans || []) {
    vanPos[van.id] = [van.x, van.y];
    ctx.fillStyle = '#3987e5'; ctx.strokeStyle = drBattCol(van.battery); ctx.lineWidth = 2.2;
    drBox(ctx, van.x, van.y, Math.max(2.3, 7 / v.s), Math.max(1.1, 3.5 / v.s), van.yaw);
    ctx.fill(); ctx.stroke();
    const p = drP(van.x, van.y);
    ctx.fillStyle = '#e8eef6'; ctx.font = '700 11px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'center';
    ctx.fillText('V' + van.id + ' ' + Math.round(van.battery) + '%', p[0], p[1] - 9);
    DR.hits.push({x:p[0], y:p[1], r:10, tip:'VAN ' + van.id + ' · battery ' + Math.round(van.battery) + '% · ' + van.state +
      ' · ' + (van.drone ? 'D' + van.drone : t('dr_nodrone')) + ' · ' + van.delivered + ' ✓ ' + van.failed + ' ✗'});
  }
  // drones / ドローン
  for (const d of s.drones || []) {
    const p = drP(d.x, d.y);
    const flying = ['delivering', 'to_station', 'to_van', 'falling'].includes(d.status);
    if (flying && d.van && vanPos[d.van] && d.status !== 'to_station') {
      const q = drP(vanPos[d.van][0], vanPos[d.van][1]);
      ctx.strokeStyle = '#ffffff30'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(p[0], p[1]); ctx.stroke(); ctx.setLineDash([]);
    }
    if (d.status === 'docked' || d.status === 'reloading') continue;  // inside its pod
    const r = flying ? R(5) : R(3.6);
    ctx.fillStyle = drBattCol(d.battery);
    ctx.strokeStyle = DR_ST_COL[d.status] || '#fff'; ctx.lineWidth = 2.2;
    ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill(); ctx.stroke();
    if (d.status === 'crashed' || d.status === 'falling') {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(p[0] - r, p[1] - r); ctx.lineTo(p[0] + r, p[1] + r);
      ctx.moveTo(p[0] + r, p[1] - r); ctx.lineTo(p[0] - r, p[1] + r); ctx.stroke();
    }
    if (flying || d.status === 'crashed') {     // pad drones are listed by the station
      ctx.fillStyle = '#e8eef6'; ctx.font = '700 11.5px ' + getComputedStyle(document.body).fontFamily;
      ctx.textAlign = 'left';
      ctx.fillText('D' + d.id + ' ' + Math.round(d.battery) + '%', p[0] + r + 3, p[1] + 4);
    }
    DR.hits.push({x:p[0], y:p[1], r:r + 4, tip:'D' + d.id + ' · ' + drSt(d.status) + ' · ' +
      Math.round(d.battery) + '% · z ' + fmt(d.z, 0) + ' m' + (d.note ? ' · ' + d.note : '')});
  }
  // wind compass in the corner (map orientation) / 隅に風向（地図と同じ向き）
  const w = s.wind || {};
  const cx = W - 54, cy = 54;
  ctx.fillStyle = '#090c12cc'; ctx.strokeStyle = '#2c3a4e'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, 42, 0, 7); ctx.fill(); ctx.stroke();
  const L = Math.min(36, 6 + (w.speed || 0) * 2.2), a = (w.dir || 0) * Math.PI / 180;
  drArrow(ctx, cx - L / 2 * Math.cos(a), cy - L / 2 * Math.sin(a), L * Math.cos(a), L * Math.sin(a),
          w.gust ? '#ff7a59' : '#7fb8ff', 3);
  ctx.fillStyle = w.gust ? '#ff7a59' : '#cfe3ff'; ctx.textAlign = 'center';
  ctx.font = '700 11px ' + getComputedStyle(document.body).fontFamily;
  ctx.fillText(fmt(w.speed || 0, 1) + ' m/s' + (w.gust ? ' GUST' : ''), cx, cy + 58);
}

function drDrawCompass() {
  const cv = $('dwind'), dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (W < 30 || H < 30) return;      // hidden (other tab): nothing to draw / 非表示
  if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0c1119'; ctx.fillRect(0, 0, W, H);
  const cx = W / 2, cy = H / 2, R0 = Math.min(W, H) / 2 - 12;
  ctx.strokeStyle = '#2c3a4e'; ctx.lineWidth = 1;
  for (const k of [0.33, 0.66, 1]) { ctx.beginPath(); ctx.arc(cx, cy, R0 * k, 0, 7); ctx.stroke(); }
  ctx.fillStyle = C.mute; ctx.font = '11px ' + getComputedStyle(document.body).fontFamily;
  ctx.textAlign = 'center';
  [['0°', R0 + 8, 0], ['90°', 0, R0 + 8], ['180°', -R0 - 9, 0], ['270°', 0, -R0 - 4]]
    .forEach(([s, dx, dy]) => ctx.fillText(s, cx + dx, cy + dy + 3));
  const w = (DR.state && DR.state.wind) || {};
  // the chosen gust direction / 選択中の突風方向
  const ga = (+$('dc-dir').value) * Math.PI / 180;
  ctx.setLineDash([4, 4]); ctx.strokeStyle = '#3987e588'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R0 * Math.cos(ga), cy + R0 * Math.sin(ga)); ctx.stroke();
  ctx.setLineDash([]);
  // the wind now: 15 m/s reaches the rim / 現在の風（15 m/s で外周）
  const L = Math.min(R0, R0 * (w.speed || 0) / 15), a = (w.dir || 0) * Math.PI / 180;
  drArrow(ctx, cx - L / 2 * Math.cos(a), cy - L / 2 * Math.sin(a), L * Math.cos(a), L * Math.sin(a),
          w.gust ? '#ff7a59' : '#7fb8ff', 4);
  // the drones' limit / ドローンが抗える限界
  ctx.strokeStyle = '#d03b3b55'; ctx.setLineDash([2, 3]);
  ctx.beginPath(); ctx.arc(cx, cy, R0 * 11 / 15, 0, 7); ctx.stroke(); ctx.setLineDash([]);
}

function drRender() {
  const s = DR.state;
  const live = s && s.available && s.running;
  $('dr-run').textContent = !s || !s.available ? t('dr_none') : live ? t('dr_live') : t('dr_stopped');
  $('dr-run').style.color = live ? '#5fd45f' : '#ff9c9c';
  ['dc-gust', 'dc-setbase', 'dc-setrate', 'dc-calm'].forEach(id => { $(id).disabled = !live; });
  drDrawMap();
  drDrawCompass();
  if (!s || !s.available) {
    $('dr-table').innerHTML = $('dr-vans').innerHTML = '';
    $('dr-stats').innerHTML = $('dr-events').innerHTML =
      '<div class="dempty">' + t('dr_waiting') + '</div>';
    return;
  }
  const ds = s.drones || [], st = s.stats || {}, w = s.wind || {};
  const cnt = f => ds.filter(f).length;
  $('dr-n').textContent = $('dr-n2').textContent = ds.length;
  $('dr-fly').textContent = cnt(d => ['delivering', 'to_station', 'to_van'].includes(d.status));
  $('dr-van').textContent = cnt(d => ['docked', 'reloading'].includes(d.status));
  $('dr-st').textContent = cnt(d => ['charging', 'standby'].includes(d.status));
  $('dr-down').textContent = cnt(d => ['falling', 'crashed'].includes(d.status));
  $('dr-deliv').textContent = st.delivered || 0;
  $('dr-busy').textContent = (st.busy_vans || 0) + ' / ' + (s.vans || []).length;
  $('dr-fail').textContent = st.failed || 0;
  $('dr-wind').textContent = fmt(w.speed, 1) + ' m/s ' + Math.round(w.dir || 0) + '°';
  $('dr-t').textContent = fmt(s.t, 0);
  $('dr-map-name').textContent = (s.map || '').split('/').pop();
  $('dr-gust').textContent = (w.grounded ? t('dr_grounded') + ' · ' : '') +
    (w.gust ? 'GUST ' + fmt(w.gust_peak, 0) + ' m/s' : t('dr_steady'));
  $('dr-gust').className = 'pill ' + (w.grounded ? 'red' : w.gust ? 'yellow' : 'green');
  $('dw-hold').textContent = w.hold ? fmt(w.hold, 0) + ' m/s' + (w.grounded ? ' — ' + t('dr_grounded') : '') : '–';
  $('dw-now').textContent = fmt(w.speed, 1) + ' m/s → ' + Math.round(w.dir || 0) + '°';
  $('dw-base').textContent = fmt(w.base_speed, 1) + ' m/s → ' + Math.round(w.base_dir || 0) + '°';
  $('dw-rate').textContent = fmt(w.gust_rate, 2) + ' / min';
  if (!DR.dirTouched) { $('dc-dir').value = Math.round(w.base_dir || 0); drSyncCtl(); }
  // drone table / 機体一覧
  const rows = ds.map(d => {
    const col = DR_ST_COL[d.status] || '#888';
    return '<tr><td class="num">D' + d.id + '</td>' +
      '<td><span class="stp" style="background:' + col + '26;color:' + col + '">' + drEsc(drSt(d.status)) + '</span></td>' +
      '<td class="num"><span class="batt"><i style="width:' + clamp(d.battery, 0, 100) + '%;background:' +
        drBattCol(d.battery) + '"></i></span>' + fmt(d.battery, 0) + '%</td>' +
      '<td class="num">' + (d.van ? 'V' + d.van : '–') + '</td>' +
      '<td class="num">' + (d.order ? '#' + d.order : '–') + '</td>' +
      '<td class="num">' + fmt(d.z, 1) + ' m</td>' +
      '<td class="num">' + fmt(d.drift, 2) + ' m</td>' +
      '<td class="num">' + d.trips + '</td><td class="num">' + d.crashes + '</td>' +
      '<td class="note">' + drEsc(d.fail && ['falling', 'crashed'].includes(d.status) ? d.fail : d.note) + '</td></tr>';
  }).join('');
  $('dr-table').innerHTML = '<tr><th>' + [t('dr_th_drone'), t('dr_th_status'), t('dr_th_batt'),
    t('dr_th_van'), t('dr_th_order'), t('dr_th_alt'), t('dr_th_drift'), t('dr_th_trips'),
    t('dr_th_crash'), t('dr_th_note')].join('</th><th>') + '</th></tr>' + rows;
  // vans / 配送ポッド
  $('dr-vans').innerHTML = '<tr><th>' + [t('dr_th_van'), t('dr_th_podbatt'), t('dr_th_state'),
    t('dr_th_drone'), t('dr_th_pend'), '✓', '✗'].join('</th><th>') + '</th></tr>' +
    (s.vans || []).map(v => '<tr><td class="num">V' + v.id + '</td>' +
      '<td class="num"><span class="batt"><i style="width:' + clamp(v.battery, 0, 100) + '%;background:' +
        drBattCol(v.battery) + '"></i></span>' + fmt(v.battery, 0) + '%</td><td>' + drEsc(v.state) + '</td>' +
      '<td class="num">' + (v.drone ? 'D' + v.drone : v.incoming ? '→ D' + v.incoming : '–') + '</td>' +
      '<td class="num">' + v.pending + '</td><td class="num">' + v.delivered + '</td>' +
      '<td class="num">' + v.failed + '</td></tr>').join('');
  // stats / 集計
  const fr = st.failed_by_reason || {};
  const kv = (k, v) => '<div class="kv"><span>' + k + '</span><span class="v">' + v + '</span></div>';
  $('dr-stats').innerHTML =
    kv(t('dr_s_deliv'), (st.delivered || 0) + ' <span class="unit">(' + (st.delivered_drone || 0) +
       ' ' + t('dr_s_bydrone') + ')</span>') +
    kv(t('dr_s_fail'), st.failed || 0) +
    Object.keys(fr).sort().map(k => kv('&nbsp;&nbsp;· ' + drEsc(t('fr_' + k) === 'fr_' + k ? k : t('fr_' + k)), fr[k])).join('') +
    kv(t('dr_s_trips'), st.trips || 0) + kv(t('dr_s_crash'), st.crashes || 0) +
    kv(t('dr_s_swaps'), st.swaps || 0) + kv(t('dr_s_charge'), st.charge_trips || 0) +
    kv(t('dr_s_repair'), st.repairs || 0) + kv(t('dr_s_gusts'), st.gusts || 0) +
    kv(t('dr_s_km'), fmt(st.drone_km, 2) + ' km') + kv(t('dr_s_queued'), st.queued || 0) +
    kv(t('dr_s_evch'), st.ev_charges || 0) + kv(t('dr_s_podkm'), fmt(st.pod_km, 2) + ' km') +
    kv(t('dr_s_strand'), st.pods_stranded || 0);
  // events / イベント
  const evs = (s.events || []).slice().reverse();
  $('dr-events').innerHTML = evs.length ? evs.map(e =>
    '<div class="ev ev-' + drEsc(e.event) + '"><span class="t">' + fmt(e.t, 0) + 's</span>' +
    '<span class="ty">' + drEsc(e.event) + (e.van !== '' && e.van != null ? ' V' + e.van : '') + '</span>' +
    '<span class="m">' + (e.order ? '#' + e.order + ' ' : '') + drEsc(e.detail) + '</span></div>').join('')
    : '<div class="dempty">' + t('dr_noevents') + '</div>';
}

function drSyncCtl() {
  $('dc-speed-v').textContent = $('dc-speed').value;
  $('dc-dir-v').textContent = $('dc-dir').value + '°';
  $('dc-dur-v').textContent = $('dc-dur').value + ' s';
  $('dc-base-v').textContent = $('dc-base').value;
  $('dc-rate-v').textContent = $('dc-rate').value;
}

async function drSend(cmd) {
  try {
    const r = await fetch('/delivery/wind', {method:'POST', headers:{'Content-Type':'application/json'},
                                             body:JSON.stringify(cmd)});
    const j = await r.json();
    $('dc-msg').textContent = j.ok ? t('dr_sent') + ': ' + JSON.stringify(j.sent) : (j.error || 'error');
  } catch (e) { $('dc-msg').textContent = String(e); }
}

['dc-speed', 'dc-dur', 'dc-base', 'dc-rate'].forEach(id => $(id).addEventListener('input', drSyncCtl));
$('dc-dir').addEventListener('input', () => { DR.dirTouched = true; drSyncCtl(); drDrawCompass(); });
$('dc-follow').onclick = () => { DR.dirTouched = false; drRender(); };
$('dc-gust').onclick = () => drSend({cmd:'gust', speed:+$('dc-speed').value,
                                     dir:+$('dc-dir').value, duration:+$('dc-dur').value});
$('dc-setbase').onclick = () => drSend({cmd:'wind', speed:+$('dc-base').value, dir:+$('dc-dir').value});
$('dc-setrate').onclick = () => drSend({cmd:'gust_rate', value:+$('dc-rate').value});
$('dc-calm').onclick = () => drSend({cmd:'calm'});
$('dwind').addEventListener('click', ev => {
  const r = $('dwind').getBoundingClientRect();
  const a = Math.atan2(ev.clientY - r.top - r.height / 2, ev.clientX - r.left - r.width / 2);
  $('dc-dir').value = Math.round((a * 180 / Math.PI + 360) % 360);
  DR.dirTouched = true; drSyncCtl(); drDrawCompass();
});
// map: wheel zoom at the cursor, drag to pan, double-click to fit
// 地図：ホイールでカーソル位置ズーム、ドラッグで移動、ダブルクリックで全体
$('dmap').addEventListener('wheel', ev => {
  if (!DR.view) return;
  ev.preventDefault();
  const r = $('dmap').getBoundingClientRect(), v = DR.view;
  const mx = ev.clientX - r.left - r.width / 2, my = ev.clientY - r.top - r.height / 2;
  const wx = v.cx + mx / v.s, wy = v.cy + my / v.s;
  v.s = clamp(v.s * (ev.deltaY < 0 ? 1.18 : 1 / 1.18), 0.05, 40);
  v.cx = wx - mx / v.s; v.cy = wy - my / v.s;
  drDrawMap();
}, {passive:false});
$('dmap').addEventListener('mousedown', ev => {
  if (!DR.view) return;
  DR.drag = {x:ev.clientX, y:ev.clientY, cx:DR.view.cx, cy:DR.view.cy};
  $('dmap').classList.add('drag');
});
window.addEventListener('mouseup', () => { DR.drag = null; $('dmap').classList.remove('drag'); });
window.addEventListener('mousemove', ev => {
  const cv = $('dmap'), tip = $('dmap-tip');
  if (DR.drag && DR.view) {
    DR.view.cx = DR.drag.cx - (ev.clientX - DR.drag.x) / DR.view.s;
    DR.view.cy = DR.drag.cy - (ev.clientY - DR.drag.y) / DR.view.s;
    drDrawMap(); tip.style.display = 'none'; return;
  }
  if ($('view-drone').hidden || !DR.hits) return;
  const r = cv.getBoundingClientRect();
  const x = ev.clientX - r.left, y = ev.clientY - r.top;
  if (x < 0 || y < 0 || x > r.width || y > r.height) { tip.style.display = 'none'; return; }
  let best = null, bd = 1e9;
  for (const h of DR.hits) { const d = Math.hypot(h.x - x, h.y - y); if (d < h.r && d < bd) { best = h; bd = d; } }
  if (!best) { tip.style.display = 'none'; return; }
  tip.textContent = best.tip; tip.style.display = 'block';
  tip.style.left = Math.min(x + 14, r.width - tip.offsetWidth - 4) + 'px'; tip.style.top = (y + 14) + 'px';
});
$('dmap').addEventListener('dblclick', () => { drFit(); drDrawMap(); });
/* ------------------------------------------------------- force feedback */
// Every setting of carla_ffb.py as a slider, built from the schema /ffb
// sends (ffb_settings.py). Changes are POSTed and carla_ffb.py picks them
// up within half a second. / carla_ffb.py の全設定をスライダーで表示。
const FF = {params:null, groups:null, values:{}, defaults:{}, presets:[],
            status:null, dragging:false, timer:null, pending:{}};
const ffNum = (p, v) => {
  const d = p.step >= 1 ? 0 : Math.min(4, Math.max(0, -Math.floor(Math.log10(p.step))));
  return Number(v).toFixed(d);
};
function ffBuild() {
  const box = $('ff-groups');
  box.innerHTML = '';
  for (const g of FF.groups) {
    const card = document.createElement('div');
    card.className = 'card';
    card.innerHTML = `<h3>${escV(g[LANG] || g.en)}</h3>`;
    for (const p of FF.params.filter(p => p.group === g.key)) {
      const row = document.createElement('div');
      row.className = 'frow';
      row.innerHTML =
        `<label for="ff-r-${p.key}" title="${escV(p.help)}">${escV(p[LANG] || p.en)}</label>` +
        `<b id="ff-v-${p.key}"></b>` +
        `<button class="fr" id="ff-x-${p.key}" title="${escV(t('ff_reset'))}" aria-label="${escV(t('ff_reset') + ': ' + (p[LANG] || p.en))}">↺</button>` +
        `<input type="range" id="ff-r-${p.key}" min="${p.min}" max="${p.max}" step="${p.step}">` +
        (p.help ? `<div class="fhelp">${escV(p.help)}</div>` : '');
      card.appendChild(row);
      const r = row.querySelector('input');
      r.addEventListener('input', () => ffSet(p.key, parseFloat(r.value)));
      r.addEventListener('pointerdown', () => { FF.dragging = true; });
      r.addEventListener('pointerup', () => { FF.dragging = false; });
      row.querySelector('.fr').onclick = () => ffSet(p.key, FF.defaults[p.key]);
    }
    box.appendChild(card);
  }
  ffPresets();
  ffToUI();
}
// Built-in presets, then the user's own (× deletes); the one matching the
// current settings is outlined. / 組み込み→ユーザープリセット。一致中を強調。
function ffPresets() {
  const pb = $('ff-presets');
  pb.innerHTML = '';
  const add = (name, label, user) => {
    const b = document.createElement('button');
    b.className = 'dbtn' + (name === 'forza_horizon' ? ' gust' : '') + (user ? ' user' : '') +
                  (FF.active === name ? ' on' : '');
    b.textContent = label;
    b.onclick = () => ffPost({preset: name});
    if (user) {
      const x = document.createElement('span');
      x.className = 'px'; x.textContent = '×'; x.title = t('ff_pdel_q');
      x.onclick = ev => {
        ev.stopPropagation();
        if (confirm(t('ff_pdel_q') + ' "' + name + '"?'))
          ffPost({delete_preset: name}).then(() => { $('ff-pmsg').textContent = t('ff_pdeleted') + ': ' + name; });
      };
      b.appendChild(x);
    }
    pb.appendChild(b);
  };
  for (const name of FF.presets) add(name, t('ff_p_' + name), false);
  for (const name of FF.userPresets) add(name, name, true);
  FF.presetKey = JSON.stringify([FF.userPresets, FF.active, LANG]);
}
$('ff-psave').onclick = () => {
  const name = $('ff-pname').value.trim();
  if (!name) { $('ff-pmsg').textContent = t('ff_pname_need'); return; }
  if (FF.userPresets.includes(name) && !confirm(t('ff_preplace_q') + ' "' + name + '"?')) return;
  // send any slider move still waiting, so the preset has it
  // 送信待ちのスライダー値も含めて保存
  clearTimeout(FF.timer);
  const vals = FF.pending; FF.pending = {};
  ffPost({values: vals, save_preset: name}).then(d => {
    $('ff-pmsg').textContent = d && d.error ? d.error : t('ff_psaved') + ' ' + name;
    if (d && !d.error) $('ff-pname').value = '';
  });
};
$('ff-pname').addEventListener('keydown', e => { if (e.key === 'Enter') $('ff-psave').click(); });
function ffToUI() {
  for (const p of FF.params) {
    const v = FF.values[p.key], r = $('ff-r-' + p.key);
    if (!r) continue;
    if (document.activeElement !== r || !FF.dragging) r.value = v;
    $('ff-v-' + p.key).textContent = ffNum(p, v);
    const changed = Math.abs(v - FF.defaults[p.key]) > p.step / 2;
    $('ff-x-' + p.key).classList.toggle('on', changed);
    r.previousElementSibling.previousElementSibling.previousElementSibling
      .classList.toggle('chg', changed);
  }
  ffDrawCurve();
}
function ffSet(key, v) {
  FF.values[key] = v; FF.pending[key] = v;
  ffToUI();
  clearTimeout(FF.timer);
  FF.timer = setTimeout(() => {
    const vals = FF.pending; FF.pending = {};
    ffPost({values: vals});
  }, 150);
}
async function ffPost(body) {
  try {
    const r = await fetch('/ffb', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
    const d = await r.json();
    ffApply(d);
    $('ff-msg').textContent = t('ff_saved') + ' · ' + t('ff_hint');
    return d;
  } catch (e) { return null; }
}
function ffApply(d) {
  if (!d || !d.params) return;
  const first = !FF.params;
  FF.params = d.params; FF.groups = d.groups; FF.defaults = d.defaults;
  FF.presets = d.presets; FF.status = d.status;
  FF.userPresets = d.user_presets || []; FF.active = d.active;
  if (!FF.dragging && !Object.keys(FF.pending).length) FF.values = d.values;
  if (first) ffBuild(); else ffToUI();
  if (!first && FF.presetKey !== JSON.stringify([FF.userPresets, FF.active, LANG])) ffPresets();
  ffStatus();
}
function ffStatus() {
  const s = FF.status;
  const live = s && s.age < 3;
  $('ff-state').textContent = !s ? t('ff_offline') : live ? t('ff_running') : t('ff_stale');
  $('ff-state').style.color = live ? 'var(--good)' : 'var(--mute)';
  $('ff-hero').textContent = s ? s.hero : '-';
  $('ff-kmh').textContent = live ? fmt(s.kmh, 0) : '-';
  $('ff-surf').textContent = live ? s.surface : '-';
  $('ff-sat').textContent = live ? Math.round(s.front_sat * 100) + '%' : '-';
  const bar = (id, v, signed) => {
    const el = $('ff-m-' + id);
    v = Math.max(-1, Math.min(1, live ? v : 0));
    if (signed) {
      el.style.left = (50 + Math.min(0, v) * 50) + '%';
      el.style.width = Math.abs(v) * 50 + '%';
    } else el.style.width = Math.max(0, v) * 100 + '%';
    $('ff-v-' + id).textContent = live ? fmt(v * 100, 0) + '%' : '-';
  };
  bar('wheel', s ? s.wheel : 0, true);
  bar('out', s ? s.output : 0, true);
  bar('damper', s ? s.damper : 0, false);
  bar('friction', s ? s.friction : 0, false);
  ffDrawCurve();
}
// Steady-state centring torque against steering position (kingpin + centre
// spring, through the output stage); the tyre's self-aligning torque comes
// on top while cornering. / 舵角に対する定常センタリングトルク。
function ffCentring(s, kmh) {
  const V = FF.values, ss = x => { x = Math.max(0, Math.min(1, x)); return x*x*(3-2*x); };
  const k = V.spring_standstill + (1 - V.spring_standstill) * ss(kmh / V.spring_full_kmh);
  const spring = -V.center_spring * k * Math.sign(s) * Math.pow(Math.abs(s), V.spring_linearity);
  let f = (-V.kpi_nm * s / V.torque_full_nm + spring) * V.strength;
  let a = Math.abs(f);
  if (a > V.soft_knee && V.soft_knee < 1)
    a = V.soft_knee + (1 - V.soft_knee) * Math.tanh((a - V.soft_knee) / (1 - V.soft_knee));
  f = Math.sign(f) * a;
  f = f * (1 - V.min_force) + V.min_force * Math.tanh(f / V.min_force_blend);
  return Math.max(-1, Math.min(1, f));
}
function ffDrawCurve() {
  const c = $('fcurve');
  if (!c || !FF.params || !secVisible('ffb')) return;
  const [ctx, w, h] = setupCanvas(c);
  ctx.clearRect(0, 0, w, h);
  const X = s => 10 + (s + 1) / 2 * (w - 20), Y = f => h / 2 - f * (h / 2 - 10);
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(X(-1), Y(0)); ctx.lineTo(X(1), Y(0));
  ctx.moveTo(X(0), Y(1)); ctx.lineTo(X(0), Y(-1)); ctx.stroke();
  const s = FF.status, live = s && s.age < 3, kmh = live ? s.kmh : 60;
  const draw = (k, col) => {
    ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i <= 120; i++) {
      const x = -1 + i / 60, y = ffCentring(x, k);
      if (i === 0) ctx.moveTo(X(x), Y(y)); else ctx.lineTo(X(x), Y(y));
    }
    ctx.stroke();
  };
  draw(0, '#69758a');
  draw(kmh, '#3987e5');
  ctx.fillStyle = C.ink || '#e8eef6'; ctx.font = '11px ' + getComputedStyle(document.body).fontFamily;
  ctx.fillText(Math.round(kmh) + ' km/h', 14, 16);
  ctx.fillStyle = '#69758a';
  ctx.fillText('L', 12, h / 2 - 4); ctx.fillText('R', w - 20, h / 2 - 4);
  if (live) {
    const sx = Math.max(-1, Math.min(1, s.steer));
    ctx.strokeStyle = '#c98500'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(X(sx), Y(1)); ctx.lineTo(X(sx), Y(-1)); ctx.stroke();
  }
}
async function ffTick() {
  if (!secVisible('ffb') && FF.params) return;
  try { ffApply(await (await fetch('/ffb')).json()); } catch (e) {}
}
setInterval(ffTick, 500);
window.addEventListener('resize', ffDrawCurve);

$('dmap-fit').onclick = () => { drFit(); drDrawMap(); };
window.addEventListener('resize', () => { if (!$('view-drone').hidden || secVisible('delivery')) drRender(); });
drSyncCtl();
setInterval(drTick, 500);

{
  // old links (#ffb, #pod, #drone...) land where those things live now
  // 旧リンクは新しい場所へ
  const h = location.hash.slice(1);
  const old = {ffb: ['settings', 'ffb'], pod: ['drone'], drone: ['drone'], veh: ['veh'],
               stats: ['stats'], settings: ['settings']};
  if (old[h]) showView(...old[h]);
  else if (h.startsWith('settings-')) showView('settings', h.slice(9));
}
applyLang();
labelRanges();
initHover();
loadSessions();
setInterval(tick, 200);
tick();
</script></body></html>
"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--dashboard-port", type=int, default=5000)
    p.add_argument("--log-dir",
                   default=os.path.join(
                       os.path.dirname(os.path.abspath(__file__)), "data"),
                   help="Directory for CSV session logs "
                        "(default: <script dir>/data)")
    p.add_argument("--no-log", action="store_true",
                   help="Disable CSV session logging")
    p.add_argument("--delivery-live-dir", default=delivery_live.DEFAULT_DIR,
                   help="delivery_sim.py's live state folder (Drones tab)")
    args = p.parse_args()
    global DELIVERY_DIR
    DELIVERY_DIR = args.delivery_live_dir

    log_dir = None if args.no_log else args.log_dir
    t = threading.Thread(
        target=telemetry_worker, args=(args.host, args.port, log_dir),
        daemon=True)
    t.start()
    threading.Thread(target=vehicle_tuning_worker, args=(args.host, args.port),
                     daemon=True).start()

    print("[dashboard] serving http://0.0.0.0:%d/" % args.dashboard_port)
    app.run(host="0.0.0.0", port=args.dashboard_port,
            debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
