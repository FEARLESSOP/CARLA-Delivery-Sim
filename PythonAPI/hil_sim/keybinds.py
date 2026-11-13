"""Key and wheel-button bindings shared by the driving window and the dashboard.

Every action the driving script (``manual_control_steeringwheel G923.py``)
supports has a *keyboard* binding and a *wheel* binding. The bindings live in
``keybinds.json`` next to this file (created on first use); the dashboard's
Settings > Keybinds page edits it, and the driving window re-reads it about
once a second.

Spellings
---------
keyboard  pygame key names with optional modifiers, lower case:
          ``"w"``, ``"up"``, ``"space"``, ``"ctrl+i"``, ``"shift+c"``, ``"f5"``.
          A "hold" action (throttle, steer...) ignores modifiers.
wheel     ``"button:<n>"`` (pygame button index) or ``"hat:<n>:<dir>"`` with
          dir = up / down / left / right (the D-pad). Throttle, brake and
          steering are wheel/pedal *axes* (wheel_config.ini) and not bindable.

Names, not numbers
------------------
``g923_buttons.json`` (created on first use) maps each wheel input to the name
printed on the Logitech G923 (PlayStation version): ``"button:3": "Triangle"``.
The dashboard identify mode lets the driver press a button and correct its
name, so the UI never shows a raw "button 7". An input without a name is shown
as "unknown button (press to name)".

The driving window writes the last wheel input to ``wheel_last.json`` (for the
dashboard's press-to-bind) and skips wheel actions while ``keybind_learn.json``
says the dashboard is listening.

This module has no pygame / carla import so the dashboard and the tests can
use it on their own.

運転ウインドウとダッシュボードで共有するキー・ホイールボタン割り当て。

運転スクリプトの各操作にキーボードとホイールの割り当てがあり、本ファイルと
同じ場所の keybinds.json（初回に作成）に保存される。ダッシュボードの
設定 > キー割り当て で編集し、運転ウインドウは約 1 秒ごとに読み直す。
ホイールの各入力の名前（×・△・右パドルなど）は g923_buttons.json にあり、
ダッシュボードの識別モードでボタンを押して名前を確認・修正できる。
画面には「ボタン 7」のような番号は表示しない。
"""

import json
import os
import re
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
KEYBINDS_PATH = os.path.join(BASE_DIR, "keybinds.json")
LABELS_PATH = os.path.join(BASE_DIR, "g923_buttons.json")
WHEEL_LAST_PATH = os.path.join(BASE_DIR, "wheel_last.json")
LEARN_PATH = os.path.join(BASE_DIR, "keybind_learn.json")
WHEEL_INI_PATH = os.path.join(BASE_DIR, "wheel_config.ini")

UNKNOWN_EN = "unknown button (press to name)"
UNKNOWN_JA = "未登録のボタン（押して名前を付ける）"

# --------------------------------------------------------------------------
# Actions / 操作一覧
# --------------------------------------------------------------------------
# (group key, EN, JA)
GROUPS = [
    ("drive", "Driving", "運転"),
    ("gears", "Gears", "ギア"),
    ("signals", "Turn signals", "ウインカー"),
    ("camera", "Camera & view", "カメラ・視点"),
    ("eye", "Driver eye (own cars)", "ドライバー視点（自作車両）"),
    ("sensors", "Camera sensors", "カメラセンサー"),
    ("sim", "Car, weather, map", "車両・天候・マップ"),
    ("tuning", "Pedal strength", "ペダル強度"),
    ("window", "Window", "ウインドウ"),
]

# id, group, kind ("hold" = polled while held, "press" = fires once),
# EN, JA, default keys, default wheel inputs, axis (wheel side is an axis).
# Defaults reproduce the script's behaviour before keybinds.json existed.
# 既定値は keybinds.json 導入前のスクリプトの動作と同じ。
_A = []


def _act(aid, group, kind, en, ja, keys=(), wheel=(), axis=False, window="driving"):
    _A.append({"id": aid, "group": group, "kind": kind, "en": en, "ja": ja,
               "keys": list(keys), "wheel": list(wheel), "axis": axis,
               "window": window})


_act("throttle", "drive", "hold", "Throttle", "アクセル", ["w", "up"], axis=True)
_act("brake", "drive", "hold", "Brake / reverse when stopped",
     "ブレーキ（停止後はリバース）", ["s", "down"], axis=True)
_act("steer_left", "drive", "hold", "Steer left", "左へ操舵", ["a", "left"], axis=True)
_act("steer_right", "drive", "hold", "Steer right", "右へ操舵", ["d", "right"], axis=True)
_act("handbrake", "drive", "hold", "Handbrake", "ハンドブレーキ", ["space"], ["button:7"])

_act("reverse", "gears", "press", "Toggle reverse", "リバース切替", ["q"], ["button:6"])
_act("gear_up", "gears", "press", "Gear up", "シフトアップ", ["."], ["button:4"])
_act("gear_down", "gears", "press", "Gear down", "シフトダウン", [","], ["button:5"])
_act("transmission", "gears", "press", "Manual / automatic", "MT / AT 切替",
     ["m"], ["button:9"])

_act("signal_left", "signals", "press", "Left turn signal", "左ウインカー", [], ["button:11"])
_act("signal_right", "signals", "press", "Right turn signal", "右ウインカー", [], ["button:10"])
_act("hazard", "signals", "press", "Hazard lights", "ハザードランプ", ["e"], ["button:8"])

_act("camera_next", "camera", "press", "Next camera (eye / hood / chase)",
     "次のカメラ（視点 / ボンネット / 追跡）", ["tab"], ["button:2"])
_act("mirrors", "camera", "press", "Toggle mirrors", "ミラー表示切替", ["n"])
_act("glance_left", "camera", "hold", "Glance left (hold)", "左をチラ見（押している間）", ["z"])
_act("glance_right", "camera", "hold", "Glance right (hold)", "右をチラ見（押している間）", ["x"])
_act("recording", "camera", "press", "Camera recording", "カメラ録画切替", ["r"])
# the road logic (OpenDRIVE) drawn around the car, road_overlay.py; no wheel default
# 道路ロジック表示（road_overlay.py）。ホイールの既定は無し
_act("road_overlay", "camera", "press", "Road logic overlay (what the AI drives on)",
     "道路ロジック表示（AI が使う道路情報）", ["o"])
# CARLA spectator window pod / drone tracking camera (cycled with Q / E, delivery_sim.py)
# CARLA 観戦ウィンドウのポッド・ドローン追従カメラ（Q / E で巡回、delivery_sim.py）
_act("pod_cam_prev", "camera", "press", "Previous pod / drone camera (CARLA window)",
     "前のポッド・ドローンカメラ（CARLA 観戦）", ["q"], window="carla")
_act("pod_cam_next", "camera", "press", "Next pod / drone camera (CARLA window)",
     "次のポッド・ドローンカメラ（CARLA 観戦）", ["e"], window="carla")

_act("eye_fwd", "eye", "press", "Eye forward", "視点を前へ", ["ctrl+i"])
_act("eye_back", "eye", "press", "Eye back", "視点を後ろへ", ["ctrl+k"])
_act("eye_left", "eye", "press", "Eye left", "視点を左へ", ["ctrl+j"])
_act("eye_right", "eye", "press", "Eye right", "視点を右へ", ["ctrl+l"])
_act("eye_down", "eye", "press", "Eye down", "視点を下へ", ["ctrl+u"])
_act("eye_up", "eye", "press", "Eye up", "視点を上へ", ["ctrl+o"])

_act("sensor_next", "sensors", "press", "Next sensor (RGB, depth, ...)",
     "次のセンサー（RGB・深度など）", ["`"], ["button:23"])
for _n in range(1, 10):
    _act("sensor_%d" % _n, "sensors", "press", "Sensor %d" % _n,
         "センサー %d" % _n, [str(_n)])

_act("autopilot", "sim", "press", "Autopilot", "オートパイロット切替", ["p"])
_act("car_next", "sim", "press", "Respawn as next car", "次の車種で再出現",
     ["backspace"], ["button:0"])
_act("car_prev", "sim", "press", "Respawn as previous car", "前の車種で再出現",
     ["shift+backspace"])
_act("weather_next", "sim", "press", "Next weather", "次の天候", ["c"], ["button:3"])
_act("weather_prev", "sim", "press", "Previous weather", "前の天候", ["shift+c"])
_act("map_next", "sim", "press", "Next map", "次のマップ", ["f5"])
_act("map_prev", "sim", "press", "Previous map", "前のマップ", ["shift+f5"])

_act("throttle_up", "tuning", "press", "Throttle strength up", "アクセル強度 上げ", ["]"])
_act("throttle_down", "tuning", "press", "Throttle strength down", "アクセル強度 下げ", ["["])
_act("brake_up", "tuning", "press", "Brake strength up", "ブレーキ強度 上げ", ["shift+]"])
_act("brake_down", "tuning", "press", "Brake strength down", "ブレーキ強度 下げ", ["shift+["])

_act("hud", "window", "press", "HUD info", "HUD 情報表示切替", ["f1"], ["button:1"])
_act("help", "window", "press", "Help (key list)", "ヘルプ（キー一覧）", ["h", "shift+/"])
_act("fullscreen", "window", "press", "Fullscreen", "フルスクリーン", ["f11"])
_act("quit", "window", "press", "Quit", "終了", ["escape", "ctrl+q"])

ACTIONS = _A
ACTION_IDS = [a["id"] for a in ACTIONS]
BY_ID = {a["id"]: a for a in ACTIONS}

# Actions that only make sense while driving a car (not a walker).
# 車両運転中のみ有効な操作（歩行者では無効）。
VEHICLE_ONLY = {"reverse", "gear_up", "gear_down", "transmission", "autopilot"}

# wheel_config.ini keys that used to hold these button numbers; read once as
# the defaults so a customised ini keeps working (migrated into keybinds.json).
# 以前 wheel_config.ini にあったボタン番号。既定値として読み込み、
# keybinds.json へ移行する。
INI_BUTTONS = {"handbrake": "handbrake", "reverse": "reverse",
               "gear_up": "gear_up", "gear_down": "gear_down",
               "transmission": "transmission"}

# --------------------------------------------------------------------------
# Logitech G923 (PlayStation version, PC mode c266, new-lg4ff) names
# G923（PS 版、PC モード c266、new-lg4ff）のボタン名
# --------------------------------------------------------------------------
# pygame numbers the buttons in HID order (kernel BTN_TRIGGER.. then
# BTN_TRIGGER_HAPPY1..9 = 25 buttons); the D-pad is hat 0. Uses in the
# script before this file agreed: 10/11 = R3/L3 signals, 4/5 = paddles,
# 6/7 = R2/L2, 9 = Options, 23 = Enter. Confirm on the wheel in the
# dashboard's identify mode.
DEFAULT_LABELS = {
    "button:0": "Cross (X)", "button:1": "Square", "button:2": "Circle",
    "button:3": "Triangle", "button:4": "R paddle", "button:5": "L paddle",
    "button:6": "R2", "button:7": "L2", "button:8": "Share",
    "button:9": "Options", "button:10": "R3", "button:11": "L3",
    "button:12": "Shifter 1", "button:13": "Shifter 2", "button:14": "Shifter 3",
    "button:15": "Shifter 4", "button:16": "Shifter 5", "button:17": "Shifter 6",
    "button:18": "Shifter R", "button:19": "Plus (+)", "button:20": "Minus (-)",
    "button:21": "Dial right ⟳", "button:22": "Dial left ⟲",
    "button:23": "Enter", "button:24": "PS button",
    "hat:0:up": "D-pad up", "hat:0:down": "D-pad down",
    "hat:0:left": "D-pad left", "hat:0:right": "D-pad right",
}
# Japanese display of the stock names (a renamed button shows as typed).
# 既定名の日本語表示（利用者が付けた名前はそのまま表示）。
LABEL_JA = {
    "Cross (X)": "×（バツ）", "Square": "□（四角）", "Circle": "○（丸）",
    "Triangle": "△（三角）", "R paddle": "右パドル", "L paddle": "左パドル",
    "Share": "SHARE", "Options": "OPTIONS", "Shifter 1": "シフター 1 速",
    "Shifter 2": "シフター 2 速", "Shifter 3": "シフター 3 速",
    "Shifter 4": "シフター 4 速", "Shifter 5": "シフター 5 速",
    "Shifter 6": "シフター 6 速", "Shifter R": "シフター R",
    "Plus (+)": "＋ボタン", "Minus (-)": "－ボタン",
    "Dial right ⟳": "ダイヤル右回し ⟳", "Dial left ⟲": "ダイヤル左回し ⟲",
    "Enter": "Enter（決定）", "PS button": "PS ボタン",
    "D-pad up": "十字キー ↑", "D-pad down": "十字キー ↓",
    "D-pad left": "十字キー ←", "D-pad right": "十字キー →",
}

HAT_DIRS = {"up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0)}

_WHEEL_RE = re.compile(r"^(button:(\d{1,2})|hat:(\d):(up|down|left|right))$")
MODS = ("ctrl", "shift", "alt")
_NAMED_KEYS = {
    "up", "down", "left", "right", "space", "tab", "return", "enter", "escape",
    "backspace", "delete", "insert", "home", "end", "page up", "page down",
    "[", "]", ",", ".", "/", "\\", ";", "'", "`", "-", "=",
    "left shift", "right shift", "left ctrl", "right ctrl", "left alt",
    "right alt", "caps lock",
    "[+]", "[-]", "[*]", "[/]", "[.]",
}
KNOWN_KEYS = (_NAMED_KEYS | set("abcdefghijklmnopqrstuvwxyz0123456789")
              | {"f%d" % i for i in range(1, 16)} | {"[%d]" % i for i in range(10)})


# --------------------------------------------------------------------------
# spellings / 表記
# --------------------------------------------------------------------------
def parse_key(spec):
    """``"Ctrl+Shift+C"`` -> (frozenset({"ctrl", "shift"}), "c"), or None.
    The last ``+`` part is the key, so ``"shift++"`` would not parse, but
    "+" is not a key name here anyway (``"[+]"`` is the keypad plus)."""
    if not isinstance(spec, str):
        return None
    s = spec.strip().lower()
    if not s:
        return None
    mods = set()
    while True:
        head, sep, tail = s.partition("+")
        if sep and head in MODS and tail:
            mods.add(head)
            s = tail
        else:
            break
    if s not in KNOWN_KEYS:
        return None
    return frozenset(mods), s


def norm_key(spec):
    p = parse_key(spec)
    if p is None:
        return None
    mods, key = p
    return "+".join([m for m in MODS if m in mods] + [key])


def parse_wheel(spec):
    """``"button:3"`` -> ("button", 3, None); ``"hat:0:up"`` -> ("hat", 0, "up")."""
    if not isinstance(spec, str):
        return None
    m = _WHEEL_RE.match(spec.strip().lower())
    if not m:
        return None
    if m.group(2) is not None:
        return ("button", int(m.group(2)), None)
    return ("hat", int(m.group(3)), m.group(4))


def norm_wheel(spec):
    p = parse_wheel(spec)
    if p is None:
        return None
    return ("button:%d" % p[1]) if p[0] == "button" else ("hat:%d:%s" % (p[1], p[2]))


_PRETTY = {"up": "Up", "down": "Down", "left": "Left", "right": "Right",
           "space": "Space", "tab": "Tab", "return": "Enter", "enter": "Num Enter",
           "escape": "Esc", "backspace": "Backspace", "delete": "Delete",
           "insert": "Insert", "home": "Home", "end": "End", "page up": "PgUp",
           "page down": "PgDn", "left shift": "L-Shift", "right shift": "R-Shift",
           "left ctrl": "L-Ctrl", "right ctrl": "R-Ctrl", "left alt": "L-Alt",
           "right alt": "R-Alt", "caps lock": "Caps Lock"}


def pretty_key(spec):
    """``"ctrl+i"`` -> ``"Ctrl+I"``; ``"shift+/"`` -> ``"Shift+/"``."""
    p = parse_key(spec)
    if p is None:
        return str(spec)
    mods, key = p
    if key in _PRETTY:
        k = _PRETTY[key]
    elif len(key) == 1 or (key[0] == "f" and key[1:].isdigit()):
        k = key.upper()
    elif key.startswith("[") and key.endswith("]"):
        k = "Num " + key[1:-1]
    else:
        k = key
    return "+".join([m.capitalize() for m in MODS if m in mods] + [k])


# --------------------------------------------------------------------------
# JSON helpers / JSON の読み書き
# --------------------------------------------------------------------------
def _read_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return None


def _write_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(data, fh, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def _ini_buttons(ini_path):
    """Button numbers stated in wheel_config.ini ([G29 Racing Wheel])."""
    out = {}
    try:
        from configparser import ConfigParser
        cp = ConfigParser()
        cp.read(ini_path)
        for action, key in INI_BUTTONS.items():
            if cp.has_option("G29 Racing Wheel", key):
                out[action] = int(cp.get("G29 Racing Wheel", key))
    except Exception:
        pass
    return out


# --------------------------------------------------------------------------
# bindings / 割り当て
# --------------------------------------------------------------------------
def defaults(ini_path=None):
    """{action: {"keys": [...], "wheel": [...]}} = today's behaviour; the five
    button numbers wheel_config.ini can set are taken from it.
    / 既定の割り当て。wheel_config.ini のボタン番号はそこから読む。"""
    ini_path = ini_path or WHEEL_INI_PATH
    out = {a["id"]: {"keys": list(a["keys"]), "wheel": list(a["wheel"])}
           for a in ACTIONS}
    for action, idx in _ini_buttons(ini_path).items():
        out[action]["wheel"] = ["button:%d" % idx]
    return out


def clean(bindings, base=None):
    """Validate a bindings dict: unknown actions dropped, bad spellings
    dropped, duplicates in one cell removed, axis actions keep no wheel
    input, missing actions filled from ``base`` (default: defaults())."""
    base = base if base is not None else defaults()
    out = {}
    src = bindings if isinstance(bindings, dict) else {}
    for a in ACTIONS:
        aid = a["id"]
        cell = src.get(aid)
        if not isinstance(cell, dict):
            out[aid] = {"keys": list(base[aid]["keys"]),
                        "wheel": list(base[aid]["wheel"])}
            continue
        keys, wheel = [], []
        for k in cell.get("keys", base[aid]["keys"]) or []:
            n = norm_key(k)
            if n and n not in keys:
                keys.append(n)
        if not a["axis"]:
            for w in cell.get("wheel", base[aid]["wheel"]) or []:
                n = norm_wheel(w)
                if n and n not in wheel:
                    wheel.append(n)
        out[aid] = {"keys": keys[:4], "wheel": wheel[:4]}
    return out


def load(path=None, ini_path=None, create=True):
    """Bindings from keybinds.json over the defaults. Creates the file with
    the defaults when missing (``create``). / keybinds.json を読む（無ければ作成）。"""
    path = path or KEYBINDS_PATH
    base = defaults(ini_path)
    data = _read_json(path)
    if not isinstance(data, dict):
        if create and not os.path.exists(path):
            try:
                save(base, path)
            except Exception:
                pass
        return base
    return clean(data.get("bindings", data), base)


def save(bindings, path=None):
    path = path or KEYBINDS_PATH
    data = {
        "_about": "Key / wheel bindings for manual_control_steeringwheel G923.py. "
                  "Edit on the dashboard (Settings > Keybinds). "
                  "/ 運転スクリプトのキー・ホイール割り当て。ダッシュボードで編集。",
        "bindings": clean(bindings),
    }
    _write_json(path, data)
    return data["bindings"]


def conflicts(bindings):
    """{"keys": {spec: [action, ...]}, "wheel": {spec: [action, ...]}} for every
    input used by more than one action. A hold action reacts to its key with
    any modifier, so it clashes with every binding on the same base key.
    Actions targeting different windows (driving vs carla) do not clash for keys.
    / 複数の操作に割り当てられた入力。別ウィンドウのキー割り当ては衝突しない。"""
    key_users, wheel_users = {}, {}
    for a in ACTIONS:
        win = a.get("window", "driving")
        cell = bindings.get(a["id"], {})
        for k in cell.get("keys", []):
            p = parse_key(k)
            if p is None:
                continue
            key_users.setdefault(p[1], []).append((a["id"], p[0], a["kind"] == "hold", k, win))
        for w in cell.get("wheel", []):
            n = norm_wheel(w)
            if n:
                wheel_users.setdefault(n, []).append(a["id"])
    out_keys = {}
    for base_key, users in key_users.items():
        for i, (aid, mods, hold, spec, win) in enumerate(users):
            for aid2, mods2, hold2, spec2, win2 in users[i + 1:]:
                if aid2 == aid or win != win2:
                    continue
                if hold or hold2 or mods == mods2:
                    for s in (spec, spec2):
                        lst = out_keys.setdefault(s, [])
                        for x in (aid, aid2):
                            if x not in lst:
                                lst.append(x)
    out_wheel = {w: u for w, u in wheel_users.items() if len(set(u)) > 1}
    return {"keys": out_keys, "wheel": out_wheel}


# --------------------------------------------------------------------------
# wheel names / ホイールの名前
# --------------------------------------------------------------------------
def load_labels(path=None, create=True):
    """{"labels": {spec: name}, "confirmed": [spec, ...]}. The file holds the
    whole map once written; without it, the stock G923 names.
    / ボタン名の対応表（ファイルが無ければ G923 の既定名）。"""
    path = path or LABELS_PATH
    data = _read_json(path)
    labels = dict(DEFAULT_LABELS)
    confirmed = []
    if isinstance(data, dict):
        # the file holds the whole map (a name moved to another button must
        # not come back from the stock table) / ファイルが全体の対応表を持つ
        if isinstance(data.get("labels"), dict):
            labels = {}
        for k, v in (data.get("labels") or {}).items():
            n = norm_wheel(k)
            if n is None:
                continue
            if v is None or str(v).strip() == "":
                labels.pop(n, None)
            else:
                labels[n] = str(v).strip()[:40]
        confirmed = [n for n in (norm_wheel(c) for c in data.get("confirmed") or []) if n]
    elif create and not os.path.exists(path):
        try:
            save_labels({"labels": labels, "confirmed": []}, path)
        except Exception:
            pass
    return {"labels": labels, "confirmed": confirmed}


def save_labels(data, path=None):
    path = path or LABELS_PATH
    labels = {}
    for k, v in (data.get("labels") or {}).items():
        n = norm_wheel(k)
        if n and v is not None and str(v).strip():
            labels[n] = str(v).strip()[:40]
    confirmed = sorted({n for n in (norm_wheel(c) for c in data.get("confirmed") or []) if n})
    out = {
        "_about": "Names of the Logitech G923 (PlayStation) buttons as pygame "
                  "numbers them. Correct them on the dashboard (Settings > Keybinds "
                  "> identify). / G923 のボタン名。ダッシュボードの識別モードで修正。",
        "labels": dict(sorted(labels.items(), key=lambda kv: _spec_order(kv[0]))),
        "confirmed": sorted(confirmed, key=_spec_order),
    }
    _write_json(path, out)
    return {"labels": out["labels"], "confirmed": out["confirmed"]}


def _spec_order(spec):
    p = parse_wheel(spec)
    if p is None:
        return (9, 0, "")
    return (0, p[1], "") if p[0] == "button" else (1, p[1], p[2])


def set_label(spec, name, path=None):
    """Name one wheel input (and mark it confirmed). Another input that had
    the same name loses it, so names stay unique. / 名前を付けて確認済みにする。"""
    path = path or LABELS_PATH
    n = norm_wheel(spec)
    if n is None:
        raise ValueError("bad wheel input")
    data = load_labels(path)
    name = str(name or "").strip()[:40]
    if name:
        for k in [k for k, v in data["labels"].items() if v == name and k != n]:
            del data["labels"][k]
            if k in data["confirmed"]:
                data["confirmed"].remove(k)
        data["labels"][n] = name
    else:
        data["labels"].pop(n, None)
    if n not in data["confirmed"]:
        data["confirmed"].append(n)
    return save_labels(data, path)


def confirm_label(spec, path=None):
    path = path or LABELS_PATH
    n = norm_wheel(spec)
    if n is None:
        raise ValueError("bad wheel input")
    data = load_labels(path)
    if n not in data["confirmed"]:
        data["confirmed"].append(n)
    return save_labels(data, path)


def label_for(spec, labels=None, lang="en"):
    """Display name of a wheel input, or the "unknown button" text; never a
    raw number. / ホイール入力の表示名（番号は出さない）。"""
    labels = labels if labels is not None else load_labels(create=False)["labels"]
    n = norm_wheel(spec)
    name = labels.get(n) if n else None
    if not name:
        return UNKNOWN_JA if lang == "ja" else UNKNOWN_EN
    return LABEL_JA.get(name, name) if lang == "ja" else name


# --------------------------------------------------------------------------
# live wheel input (driving window -> dashboard) / ホイール入力の受け渡し
# --------------------------------------------------------------------------
def write_wheel_last(kind, index, direction=None, path=None):
    """Driving window: remember the last wheel button / hat press."""
    path = path or WHEEL_LAST_PATH
    spec = ("button:%d" % index) if kind == "button" else ("hat:%d:%s" % (index, direction))
    try:
        _write_json(path, {"spec": spec, "t": time.time()})
    except Exception:
        pass
    return spec


def read_wheel_last(path=None):
    path = path or WHEEL_LAST_PATH
    data = _read_json(path)
    if not isinstance(data, dict) or norm_wheel(data.get("spec")) is None:
        return None
    return {"spec": norm_wheel(data["spec"]), "t": float(data.get("t", 0))}


def set_learning(seconds, path=None):
    """Dashboard: ask the driving window to only report wheel presses (not
    act on them) for ``seconds``; 0 ends it. / 識別中はボタン操作を無効化。"""
    path = path or LEARN_PATH
    until = time.time() + seconds if seconds > 0 else 0.0
    _write_json(path, {"until": until})
    return until


def learning_until(path=None):
    path = path or LEARN_PATH
    data = _read_json(path)
    try:
        return float(data.get("until", 0)) if isinstance(data, dict) else 0.0
    except (TypeError, ValueError):
        return 0.0


# --------------------------------------------------------------------------
# help text / ヘルプ表示
# --------------------------------------------------------------------------
def help_rows(bindings, labels=None, lang="en"):
    """Rows for the in-game help: ("group", title) and
    ("row", action, keys, wheel). / ゲーム内ヘルプの行。"""
    labels = labels if labels is not None else load_labels(create=False)["labels"]
    rows = []
    for gkey, gen, gja in GROUPS:
        acts = [a for a in ACTIONS if a["group"] == gkey]
        if gkey == "sensors":                        # 1..9 on one line
            first = [a for a in acts if not a["id"].startswith("sensor_")
                     or a["id"] == "sensor_next"]
            digits = [a for a in acts if a not in first]
        else:
            first, digits = acts, []
        rows.append(("group", "%s / %s" % (gen, gja)))
        for a in first:
            cell = bindings.get(a["id"], {})
            keys = ", ".join(pretty_key(k) for k in cell.get("keys", [])) or "-"
            if a["axis"]:
                wheel = "wheel / pedal" if lang == "en" else "ハンドル / ペダル"
            else:
                wheel = ", ".join(label_for(w, labels, lang)
                                  for w in cell.get("wheel", [])) or "-"
            rows.append(("row", "%s / %s" % (a["en"], a["ja"]), keys, wheel))
        if digits:
            keys = " ".join(pretty_key(bindings[a["id"]]["keys"][0])
                            if bindings.get(a["id"], {}).get("keys") else "-"
                            for a in digits)
            wheel = ", ".join(label_for(w, labels, lang) for a in digits
                              for w in bindings.get(a["id"], {}).get("wheel", [])) or "-"
            rows.append(("row", "Sensor 1-9 / センサー 1-9", keys, wheel))
    return rows
