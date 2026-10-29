"""Force-feedback settings shared by carla_ffb.py and dashboard.py.

Every number carla_ffb.py uses is a setting here, with its range and a label,
so the dashboard can show a slider for each. Values live in ffb_tuning.json;
carla_ffb.py re-reads that file whenever it changes (twice a second), so a
change from the dashboard is felt on the wheel at once. ffb_status.json is the
other direction: what the wheel is doing right now, for the dashboard.

The default preset is modelled on Forza Horizon's FFB: a speed-dependent
centre spring, a self-aligning torque that goes light at the limit, a
dynamic damper that settles the wheel on centre without overshoot, a smooth
minimum force, and a torque slew limit so nothing steps.

carla_ffb.py と dashboard.py が共有するフォースフィードバック設定。値は
ffb_tuning.json に保存され、carla_ffb.py は変更を毎秒 2 回読み直すため、
ダッシュボードの変更はすぐハンドルに反映される。既定値は Forza Horizon 風。
"""

import json
import os
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TUNING_PATH = os.path.join(BASE_DIR, "ffb_tuning.json")
STATUS_PATH = os.path.join(BASE_DIR, "data", "ffb_status.json")
# presets the user saved from the dashboard: {name: every value}
# ダッシュボードから保存したユーザープリセット
USER_PRESETS_PATH = os.path.join(BASE_DIR, "ffb_presets.json")
PRESET_NAME_MAX = 40

# (key, default, min, max, step, group, English label, Japanese label, help)
# 既定値は Forza Horizon 風プリセット / defaults = the Forza Horizon preset
PARAMS = [
    # overall / 全体
    ("strength", 1.0, 0.0, 2.0, 0.05, "overall", "Force feedback scale", "全体の強さ",
     "Multiplies every force. Forza's 'Force Feedback Scale'."),
    ("invert", 1, 0, 1, 1, "overall", "Invert direction", "向きを反転",
     "1 if the wheel pushes the wrong way."),
    ("torque_full_nm", 240.0, 80.0, 600.0, 5.0, "overall", "Torque at 100 % (Nm)", "100% 出力のトルク [Nm]",
     "Kingpin torque that maps to full motor force. Lower = heavier steering."),
    # self-aligning (tyres) / セルフアライニング（タイヤ）
    ("pneumatic_trail", 1.0, 0.0, 3.0, 0.05, "tyres", "Pneumatic trail scale", "ニューマチックトレール",
     "Tyre self-aligning torque: builds with cornering, goes light just before "
     "the fronts let go. Forza's 'Pneumatic Trail Scale'."),
    ("mechanical_trail", 1.0, 0.0, 3.0, 0.05, "tyres", "Mechanical trail scale", "メカニカルトレール",
     "Caster torque from the cornering force; stays heavy at the limit. "
     "Forza's 'Mechanical Trail Scale'."),
    ("contact_half_len", 0.10, 0.03, 0.25, 0.005, "tyres", "Contact patch half length (m)", "接地長の半分 [m]",
     "Longer patch = more self-aligning torque."),
    ("caster_trail", 0.010, 0.0, 0.05, 0.001, "tyres", "Caster trail (m)", "キャスタートレール [m]",
     "Mechanical trail length."),
    ("slip_peak_deg", 11.0, 4.0, 20.0, 0.5, "tyres", "Slip angle of full slide (deg)", "完全スライドのスリップ角 [度]",
     "Where the front tyres are fully sliding; the light patch comes at ~0.6 of it."),
    ("mu", 1.0, 0.4, 1.6, 0.05, "tyres", "Tyre friction", "タイヤ摩擦係数", "Grip used by the tyre model."),
    ("cg_height", 0.55, 0.2, 1.2, 0.05, "tyres", "Centre of gravity height (m)", "重心高 [m]",
     "Weight transfer: heavier under braking, lighter under power."),
    ("kpi_nm", 55.0, 0.0, 200.0, 5.0, "tyres", "Kingpin centring at full lock (Nm)", "キングピン復元 [Nm]",
     "Steering-geometry centring, felt most at low speed."),
    ("slip_blend_lo", 2.0, 0.0, 20.0, 0.5, "tyres", "Tyre model fades in from (km/h)", "タイヤモデル開始速度 [km/h]", ""),
    ("slip_blend_hi", 9.0, 1.0, 40.0, 0.5, "tyres", "...fully on at (km/h)", "完全に有効な速度 [km/h]", ""),
    # centre spring / センタースプリング
    ("center_spring", 0.35, 0.0, 1.5, 0.05, "spring", "Centre spring scale", "センタースプリング",
     "Pulls the wheel back to centre, stronger with speed. Forza's 'Center Spring Scale'."),
    ("spring_standstill", 0.25, 0.0, 1.0, 0.05, "spring", "Spring at standstill (fraction)", "停車時のスプリング割合", ""),
    ("spring_full_kmh", 80.0, 5.0, 200.0, 5.0, "spring", "Spring full strength at (km/h)", "最大になる速度 [km/h]", ""),
    ("spring_linearity", 1.0, 0.5, 2.0, 0.05, "spring", "Spring linearity", "スプリングの線形性",
     "1 = linear; above 1 softer around centre, firmer near lock; below 1 the reverse."),
    # damping and friction / ダンピングと摩擦
    ("damper_base", 0.15, 0.0, 1.0, 0.01, "damping", "Wheel damper", "ダンパー",
     "Resists fast wheel movement. Forza's 'Wheel Damper Scale'."),
    ("damper_per_kmh", 0.0025, 0.0, 0.01, 0.0001, "damping", "Dynamic damper per km/h", "速度比例ダンパー",
     "Extra damping with speed. Forza's 'Dynamic Damper Behavior'."),
    ("damper_max", 0.55, 0.0, 1.0, 0.01, "damping", "Damper limit", "ダンパー上限", ""),
    ("return_damper", 0.30, 0.0, 1.0, 0.01, "damping", "Return damper", "リターンダンパー",
     "Brakes the wheel as it swings back to centre (turning in stays light), "
     "so it snaps back and stops on centre instead of overshooting."),
    ("center_damper", 0.30, 0.0, 1.0, 0.01, "damping", "Centre damper (settling)", "センターダンパー",
     "Extra damping close to centre, so the wheel settles there instead of overshooting."),
    ("center_zone", 0.06, 0.01, 0.3, 0.01, "damping", "Centre zone (of full lock)", "センター域",
     "How close to centre the centre damper and minimum-force fade act."),
    ("friction_column", 0.04, 0.0, 0.5, 0.01, "damping", "Column friction", "コラム摩擦", "Always on."),
    ("friction_park", 0.25, 0.0, 1.0, 0.01, "damping", "Parking friction", "据え切り摩擦", "Tyre scrub when stopped."),
    ("friction_park_kmh", 6.0, 1.0, 30.0, 0.5, "damping", "Parking friction fades by (km/h)", "据え切り摩擦の減衰速度 [km/h]", ""),
    # road feel / 路面の感触
    ("road_feel", 0.8, 0.0, 3.0, 0.05, "road", "Road feel scale", "路面の感触",
     "Asphalt texture and bumps. Forza's 'Road Feel Scale'."),
    ("offroad_feel", 1.0, 0.0, 3.0, 0.05, "road", "Off-road feel scale", "オフロードの感触",
     "Texture on shoulders, kerbs and off the road. Forza's 'Off-Road Feel Scale'."),
    ("bump_thresh", 3.0, 0.5, 10.0, 0.1, "road", "Bump threshold (m/s²)", "突き上げ閾値 [m/s²]", ""),
    ("bump_gain", 0.025, 0.0, 0.1, 0.001, "road", "Bump strength", "突き上げの強さ", ""),
    ("scrub_buzz", 0.06, 0.0, 0.3, 0.005, "road", "Front-slide buzz", "前輪スライド振動", ""),
    # effects / 効果
    ("kerb_jolt", 0.40, 0.0, 1.0, 0.01, "effects", "Kerb strike", "縁石の衝撃", ""),
    ("crash_gain", 0.9, 0.0, 2.0, 0.05, "effects", "Crash jolt", "衝突の衝撃", ""),
    ("crash_div", 4000.0, 500.0, 20000.0, 100.0, "effects", "Crash impulse divisor", "衝撃量の割り数",
     "Lower = stronger jolt for the same hit."),
    # smoothing and output / 平滑化と出力
    ("smoothing_ms", 80.0, 10.0, 250.0, 5.0, "output", "Simulator smoothing (ms)", "平滑化 [ms]",
     "Smooths the 20-30 Hz simulator state up to the 250 Hz FFB rate."),
    ("torque_slew", 8.0, 1.0, 40.0, 0.5, "output", "Torque slew limit (/s)", "トルク変化率上限 [/s]",
     "Fastest the steering torque may change (full scale per second); kerbs and crashes bypass it."),
    ("min_force", 0.03, 0.0, 0.2, 0.005, "output", "Minimum force", "最小フォース",
     "Overcomes the motor's dead band. Forza's 'Minimum Force'."),
    ("min_force_blend", 0.10, 0.02, 0.4, 0.01, "output", "Minimum force blend", "最小フォースのなじみ幅",
     "How gently the minimum force fades in; wider = smoother through zero."),
    ("soft_knee", 0.80, 0.3, 1.0, 0.01, "output", "Soft clip knee", "ソフトクリップ開始点",
     "Forces above this are compressed instead of clipped."),
    ("end_stop_gain", 25.0, 0.0, 60.0, 1.0, "output", "Steering lock end stop", "ロック位置のエンドストップ", ""),
    # car / 車
    ("tire_friction", 2.6, 1.0, 5.0, 0.1, "car", "Car tyre grip (CARLA)", "車のタイヤグリップ (CARLA)",
     "Applied to the hero car's physics. CARLA's default is about 3.5."),
]

GROUPS = [("overall", "Overall", "全体"), ("tyres", "Self-aligning torque (tyres)", "セルフアライニングトルク"),
          ("spring", "Centre spring", "センタースプリング"), ("damping", "Damping & friction", "ダンピングと摩擦"),
          ("road", "Road feel", "路面の感触"), ("effects", "Effects", "効果"),
          ("output", "Smoothing & output", "平滑化と出力"), ("car", "Car", "車")]

DEFAULTS = {p[0]: p[1] for p in PARAMS}
LIMITS = {p[0]: (p[2], p[3]) for p in PARAMS}

# presets: only the keys that differ from the Forza Horizon defaults
# プリセット（既定値＝Forza Horizon 風との差分のみ）
PRESETS = {
    "forza_horizon": {},
    # the tuning carla_ffb.py had before the settings were added
    # 設定化以前の carla_ffb.py の値
    "previous": {"torque_full_nm": 260.0, "kpi_nm": 70.0, "center_spring": 0.0,
                 "damper_base": 0.12, "damper_per_kmh": 0.0020, "damper_max": 0.45,
                 "center_damper": 0.0, "return_damper": 0.0, "friction_column": 0.05, "friction_park": 0.55,
                 "road_feel": 1.0, "scrub_buzz": 0.07, "kerb_jolt": 0.45,
                 "smoothing_ms": 30.0, "torque_slew": 40.0, "min_force": 0.04,
                 "min_force_blend": 0.02},
    "light": {"strength": 0.7, "torque_full_nm": 320.0, "center_spring": 0.25,
              "damper_base": 0.15, "friction_park": 0.3, "road_feel": 0.6},
    "heavy": {"strength": 1.2, "torque_full_nm": 190.0, "center_spring": 0.5,
              "damper_base": 0.3, "damper_max": 0.7, "friction_park": 0.6},
}


def clamp_value(key, value):
    lo, hi = LIMITS[key]
    v = min(max(float(value), lo), hi)
    return int(round(v)) if key == "invert" else v


def load(path=TUNING_PATH):
    """Defaults overlaid with whatever valid values the tuning file has."""
    out = dict(DEFAULTS)
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return out
    for k, v in data.items():
        if k in out:
            try:
                out[k] = clamp_value(k, v)
            except (TypeError, ValueError):
                pass
    return out


def save(values, path=TUNING_PATH):
    clean = {k: clamp_value(k, v) for k, v in values.items() if k in DEFAULTS}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(clean, f, indent=1, sort_keys=True)
    os.replace(tmp, path)
    return clean


def preset(name, path=USER_PRESETS_PATH):
    """A built-in or user preset's full values (None if there is no such one).
    組み込みまたはユーザープリセットの全値。"""
    out = dict(DEFAULTS)
    if name in PRESETS:
        out.update(PRESETS[name])
        return out
    saved = user_presets(path).get(name)
    if saved is None:
        return None
    for k, v in saved.items():          # older presets miss newer keys / 新項目は既定値
        if k in out:
            out[k] = clamp_value(k, v)
    return out


def user_presets(path=USER_PRESETS_PATH):
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.items() if isinstance(v, dict)}


def _write_user_presets(presets, path):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(presets, f, indent=1, sort_keys=True)
    os.replace(tmp, path)


def save_preset(name, values, path=USER_PRESETS_PATH):
    """Save ``values`` as user preset ``name`` (replacing one of that name).
    Built-in names can't be overwritten. Returns the cleaned name.
    値をユーザープリセットとして保存（同名は上書き、組み込み名は不可）。"""
    name = " ".join(str(name).split())[:PRESET_NAME_MAX]
    if not name:
        raise ValueError("empty preset name")
    if name in PRESETS:
        raise ValueError("'%s' is a built-in preset" % name)
    presets = user_presets(path)
    presets[name] = {k: clamp_value(k, v) for k, v in values.items() if k in DEFAULTS}
    _write_user_presets(presets, path)
    return name


def delete_preset(name, path=USER_PRESETS_PATH):
    presets = user_presets(path)
    if presets.pop(name, None) is not None:
        _write_user_presets(presets, path)


def write_status(status, path=STATUS_PATH):
    status["wall_time"] = time.time()
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(status, f)
        os.replace(tmp, path)
    except OSError:
        pass


def read_status(path=STATUS_PATH):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
