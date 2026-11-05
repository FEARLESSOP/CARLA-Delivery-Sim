#!/usr/bin/env python3
"""Map switching (F5 / Shift+F5) under run_pipeline.py: the map cycle and the
request-file protocol between the driving window and the pipeline.

Why a protocol and not ``client.load_world()`` in the driving window: under
run_pipeline.py about eight clients share one world -- generate_traffic.py
(the synchronous tick master and TrafficManager server), delivery_sim.py,
dashboard.py, carla_ffb.py, traction_fx.py, drone_viewer.py ... A map load
from one of them destroys every actor the others hold, while the world is
synchronous and the TrafficManager is still driving the old cars. libcarla
then abort()s in those clients ("terminate called ... TimeoutException"), the
TM dies with them, and the whole rig went down with the driving window. So
the driving window only *asks*: it writes ``map_request.json`` and exits with
``MAP_SWITCH_RC``. run_pipeline.py stops the world clients (traffic first),
loads the map, and starts them all again on it with the same options and the
same hero car. Standalone (no pipeline) the window still loads the map itself.

This module imports no ``carla`` so tests/test_map_switch.py runs without a
server.

run_pipeline.py 配下のマップ切替（F5 / Shift+F5）：マップ巡回順と、運転
ウィンドウとパイプライン間のリクエストファイル方式。

運転ウィンドウで ``client.load_world()`` を呼ばない理由：パイプライン配下では
約 8 つのクライアントが同じワールドを共有している（同期 tick マスタ兼
TrafficManager サーバの generate_traffic.py、delivery_sim.py、dashboard.py、
carla_ffb.py、traction_fx.py、drone_viewer.py など）。その一つがマップを
読み込むと、ワールドが同期モードで TM が旧車両を動かしている最中に、他の全
クライアントが持つアクターが消える。libcarla はそれらのクライアントで
abort() し（"terminate called ... TimeoutException"）、TM も道連れになり、
運転ウィンドウと共にリグ全体が落ちていた。そこで運転ウィンドウは*依頼*だけ
を行う：``map_request.json`` を書き ``MAP_SWITCH_RC`` で終了する。
run_pipeline.py がワールドのクライアントを止め（交通を最初に）、マップを
読み込み、同じオプション・同じ自車で全てを再起動する。単体起動（パイプライン
なし）の場合は従来どおり運転ウィンドウ自身がマップを読み込む。

本モジュールは ``carla`` を import しないため、tests/test_map_switch.py は
サーバ無しで動く。
"""

import json
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REQUEST_PATH = os.path.join(HERE, "map_request.json")
# Exit code of the driving window meaning "switch the map, then start me
# again" (75 = EX_TEMPFAIL). / 「マップを切り替えて再起動して」を表す終了コード
MAP_SWITCH_RC = 75
# A request older than this is left over from an earlier run and ignored.
# これより古いリクエストは以前の実行の残骸として無視する
REQUEST_MAX_AGE_S = 120.0

# Levels that are not drivable towns: test/blank levels and streaming
# sublevels. / 走行用ではないレベル（テスト用・空・サブレベル）
_SKIP_NAMES = {"OpenDriveMap", "EmptyMap"}
_SKIP_PARTS = ("/Sublevels/", "/TestMaps/")


def short_name(path):
    """'/Game/Carla/Maps/Town10HD_Opt' -> 'Town10HD';
    'Japan/Maps/OitaKosen/OitaKosen' -> 'OitaKosen'. / マップの短い名前"""
    return str(path).rstrip("/").split("/")[-1].replace("_Opt", "")


def _town_key(name):
    """Natural order for CARLA's towns: Town2 < Town10HD. / 町の自然順"""
    m = re.match(r"Town(\d+)(.*)$", name)
    return (int(m.group(1)), m.group(2)) if m else (10 ** 6, name)


def build_cycle(available):
    """The F5 map cycle from ``client.get_available_maps()`` (or
    ``installed_maps()``): every drivable installed map once, in a stable
    order -- CARLA's own towns first (Town01 .. Town10HD), then your own maps
    by package and name (Circuits/MarinaBay, Circuits/SpaFrancorchamps,
    Japan/OitaKosen, Japan/OitaStation, Singapore/PasirRis ...). ``_Opt``
    variants collapse into their town; sublevels, test and blank levels are
    left out, so a map that is not installed can never be picked.

    F5 のマップ巡回順。インストール済みの走行可能なマップを一度ずつ、安定
    した順で並べる：CARLA の町（Town01〜Town10HD）、次に自作マップを
    パッケージ名・マップ名順に。``_Opt`` 版は同じ町にまとめ、サブレベル・
    テスト用・空のレベルは除く。未インストールのマップは選ばれない。
    """
    towns, own = {}, {}
    for path in available or ():
        p = "/" + str(path).replace("\\", "/").strip("/")
        if any(part in p for part in _SKIP_PARTS):
            continue
        name = short_name(p)
        if not name or name in _SKIP_NAMES:
            continue
        parts = p.split("/")
        # '/Game/<Package>/Maps/...' ; CARLA's own towns live in package Carla
        package = parts[2] if len(parts) > 2 and parts[1] == "Game" else (
            parts[1] if len(parts) > 1 else "")
        if package == "Carla":
            towns[name] = True
        else:
            own.setdefault(name, package)
    cycle = sorted(towns, key=_town_key)
    cycle += sorted(own, key=lambda n: (own[n].lower(), n.lower()))
    return cycle


def installed_maps(content_dir):
    """Fallback for ``get_available_maps()``: the ``.umap`` levels under
    ``<CARLA_ROOT>/CarlaUE4/Content``, as '/Game/...' paths.
    ``get_available_maps()`` の代替：Content 以下の .umap を '/Game/...' で返す。"""
    found = []
    for root, _dirs, files in os.walk(content_dir):
        for f in files:
            if f.endswith(".umap"):
                rel = os.path.relpath(os.path.join(root, f[:-5]), content_dir)
                found.append("/Game/" + rel.replace(os.sep, "/"))
    return sorted(found)


def step(cycle, current, offset):
    """The map ``offset`` places after ``current`` in ``cycle`` (wraps;
    -1 = previous). A current map outside the cycle counts as just before
    its start, so F5 gives the first map and Shift+F5 the last.
    巡回順で現在のマップから ``offset`` 番目（-1 で前）。巡回外のマップに
    いる場合、F5 は先頭、Shift+F5 は末尾になる。"""
    if not cycle:
        return None
    cur = short_name(current) if current else None
    if cur in cycle:
        return cycle[(cycle.index(cur) + offset) % len(cycle)]
    return cycle[0] if offset >= 0 else cycle[offset % len(cycle)]


def write_request(target, hero=None, current=None, path=REQUEST_PATH):
    """Ask run_pipeline.py to switch to ``target``, keeping the hero model.
    Written atomically (temp file + rename) so the pipeline never reads half
    a file. / パイプラインへマップ切替を依頼する（自車の車種は維持）。"""
    req = {"map": target, "hero": hero or "", "from": current or "",
           "time": time.time(), "pid": os.getpid()}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(req, f, indent=1)
    os.replace(tmp, path)
    return req


def read_request(path=REQUEST_PATH, max_age=REQUEST_MAX_AGE_S, now=None):
    """The pending request (dict with a non-empty 'map'), or None if there is
    none, it is unreadable, or it is stale. / 有効な依頼（無い・壊れている・
    古い場合は None）"""
    try:
        with open(path) as f:
            req = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(req, dict) or not str(req.get("map") or "").strip():
        return None
    now = time.time() if now is None else now
    try:
        if max_age is not None and now - float(req.get("time", 0)) > max_age:
            return None
    except (TypeError, ValueError):
        return None
    req["map"] = str(req["map"]).strip()
    req["hero"] = str(req.get("hero") or "").strip()
    return req


def clear_request(path=REQUEST_PATH):
    """Remove the request once taken (or at pipeline start). / 依頼を消す"""
    for p in (path, path + ".tmp"):
        try:
            os.remove(p)
        except OSError:
            pass
