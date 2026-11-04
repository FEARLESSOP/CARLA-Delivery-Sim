"""JARTIC open data, type D (交通規制情報), for one map (maps/.venv or carla-venv).

The police regulations of a prefecture as a monthly CSV (JARTIC オープンデータ
https://www.jartic.or.jp/service/opendata/, 拡張版標準フォーマット k_2.1, cp932):
one row per regulation with its place (規制場所の経度緯度 "lon lat;lon lat;..."),
point / line / area (点・線・面コード 1/2/3), the common type code
(共通規制種別コード) and the prefecture's own name of it (県別規制種別名称: the
names below come from the file itself, never from a guessed code table),
entry / banned / allowed directions as coordinates, times and vehicle classes.

maps.json "jartic_d": {"csv": <path>, "stops": "add" (keep stops JARTIC lacks unless
"no_stop": [{x, y, why}] imagery shows them gone) | "replace"} turns it on
for a map (osm_city/markings_jp.py, signs.py, jp_signs.py). Maps without the
key are untouched. The rows inside the map are cached in
build/<Name>/jartic_d.json (map frame, x east / y north) with the CSV's date.

The CSV is downloaded data: it is only parsed (csv module, cp932), never run.

JARTIC 交通規制情報（タイプ D）を地図座標に切り出す。maps.json "jartic_d" で有効化。
"""

import csv
import json
import math
import os

import numpy as np

import geo

HERE = os.path.dirname(os.path.abspath(__file__))
# common codes used here, as named in the Oita file (大分県警 k_2.1, 2026-09)
# 使用する共通規制種別コード（大分県警ファイルの名称）
ONEWAY, NO_TURN, NO_PASSING, LANES, BICYCLE_PAVEMENT = 11, 12, 17, 20, 81
STOP, CROSSING, SIGNAL, SPEED, NO_PARKING = 63, 85, 98, 112, 115


def entry(name):
    """maps.json entry of the map called ``name`` ({} if none). / maps.json の地図設定。"""
    try:
        cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    except (OSError, ValueError):
        return {}
    return next((v for v in cfg.values() if isinstance(v, dict) and v.get("name") == name), {})


def config(meta):
    """The map's "jartic_d" settings, or None (the map does not use JARTIC). / 設定（無ければ None）。"""
    # a build dir's meta.json "jartic_d" (scratch copies) or maps.json / ビルド側 meta.json か maps.json
    c = meta.get("jartic_d") or entry(meta.get("name")).get("jartic_d")
    if not c:
        return None
    if isinstance(c, str):
        c = {"csv": c}
    c = dict(c)
    c["csv"] = os.path.expanduser(c["csv"])
    c.setdefault("stops", "add")
    return c


def _pts(s, fr):
    out = []
    for p in (s or "").split(";"):
        q = p.split()
        if len(q) >= 2:
            try:
                out.append(list(fr.xy(float(q[0]), float(q[1]))))
            except ValueError:
                pass
    return out


def _int(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def parse(csv_path, meta, margin=30.0):
    """Rows of the CSV with any point inside the map box (+margin), as dicts in the map frame.
    地図範囲内の規制を地図座標で返す。"""
    fr = geo.Frame(meta)
    hx, hy = meta["half_x"] + margin, meta["half_y"] + margin
    out = []
    with open(csv_path, encoding="cp932", newline="") as fh:
        r = csv.reader(fh)
        h = next(r)
        col = {k: i for i, k in enumerate(h)}

        def g(row, k):
            i = col.get(k)
            return row[i].strip() if i is not None and i < len(row) else ""
        for row in r:
            xy = _pts(g(row, "規制場所の経度緯度"), fr)
            if not xy or not any(abs(x) < hx and abs(y) < hy for x, y in xy):
                continue
            t0, t1 = g(row, "規制時間1_開始"), g(row, "規制時間1_終了")
            out.append({
                "code": _int(g(row, "共通規制種別コード")), "name": g(row, "県別規制種別名称"),
                "kind": _int(g(row, "点・線・面コード")), "key": g(row, "ユニークキー"),
                "updated": g(row, "データ更新日"), "xy": [[round(a, 2) for a in p] for p in xy],
                "entry": _pts(g(row, "進入方向(座標)"), fr), "toward": _pts(g(row, "指定する方向(座標)"), fr),
                "forbid": _pts(g(row, "禁止する方向(座標)"), fr),
                "time": ("%s-%s" % (t0, t1)) if t0 and (t0, t1) != ("0000", "2400") else None,
                "speed": _int(g(row, "速度")), "stop_lines": _int(g(row, "停止線本数")),
                "n_crossings": _int(g(row, "横断歩道設置本数")),
                "signal": {"1": True, "2": False}.get(g(row, "信号の有無コード")),
                "vehicles": g(row, "対象車両コード1_A"), "except": g(row, "除外車両コード1_D"),
                "sides": g(row, "片側・両側コード"),
            })
    return out


def load(meta, out_dir=None):
    """The map's regulations (cached in <out_dir>/jartic_d.json), or None. / 地図の規制一覧。"""
    c = config(meta)
    if c is None:
        return None
    cache = os.path.join(out_dir, "jartic_d.json") if out_dir else None
    src = os.path.abspath(c["csv"])
    st = os.stat(src)
    if cache and os.path.exists(cache):
        d = json.load(open(cache))
        if d.get("csv") == src and d.get("size") == st.st_size and d.get("mtime") == int(st.st_mtime):
            return d["rows"]
    rows = parse(src, meta)
    if cache:
        json.dump({"csv": src, "size": st.st_size, "mtime": int(st.st_mtime), "rows": rows},
                  open(cache, "w"), ensure_ascii=False, indent=0)
    return rows


# -- matching to the road logic / 道路情報との対応 ---------------------------------

def _dist_poly(p, P):
    """Distance from p to polyline P (n x 2) and the index of the segment. / 点と折れ線の距離。"""
    a, b = P[:-1], P[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-9)
    t = np.clip(((p - a) * ab).sum(1) / L2, 0.0, 1.0)
    q = a + ab * t[:, None]
    d = np.hypot(*(q - p).T)
    i = int(np.argmin(d))
    return float(d[i]), i


def match_approach(P, E, approaches, lanes_by, near=30.0):
    """The approach (road, junction) a point regulation with entry point E means:
    its lane passes E (or E lies up the road behind its start) travelling toward P,
    and it ends within ``near`` m of P. Returns ((road, j), score) or (None, None).
    点規制（一時停止など）の進入路: 進入方向の点を通り P へ向かい、P の近くで終わる車線。"""
    P, E = np.asarray(P, float), np.asarray(E, float)
    want = P - E
    want = want / (np.linalg.norm(want) or 1.0)
    best = (None, None)
    for (road, j), al in approaches.items():
        ends = [np.asarray(a["end"]) for a in al]
        dend = min(np.linalg.norm(e - P) for e in ends)
        if dend > near:
            continue
        for a in al:
            l = lanes_by.get((road, a["lane"]))
            if l is None or len(l["pts"]) < 2:
                continue
            C = np.array([q["c"] for q in l["pts"]])
            d, i = _dist_poly(E, C)
            f = C[min(i + 1, len(C) - 1)] - C[i]
            f = f / (np.linalg.norm(f) or 1.0)
            # the end direction too: the entry point may be far up a winding road
            # 終端の向きも見る（進入点が遠い場合）
            fe = C[-1] - C[max(0, len(C) - 4)]
            fe = fe / (np.linalg.norm(fe) or 1.0)
            if float(f @ want) < 0.5 and float(fe @ want) < 0.5:
                continue
            if d > 8.0 and np.linalg.norm(E - P) < 40.0:
                continue
            score = d + 0.5 * dend
            if best[1] is None or score < best[1]:
                best = ((road, j), score)
    return best


def lanes_along(line, lanes, tol=6.0, frac=0.6, min_len=15.0):
    """Non-junction lanes lying along a line regulation (its centre polyline):
    at least ``frac`` of their points within ``tol`` m. [(road, lane)].
    線規制に沿う車線（点の frac 以上が tol 以内）。"""
    L = np.asarray(line, float)
    if len(L) < 2:
        return []
    lo, hi = L.min(0) - tol, L.max(0) + tol
    out = []
    for l in lanes:
        if l["junction"] >= 0 or len(l["pts"]) < 3:
            continue
        C = np.array([q["c"] for q in l["pts"]])
        if (C.max(0) < lo).any() or (C.min(0) > hi).any():
            continue
        if np.hypot(*(C[-1] - C[0])) < min_len and len(C) < min_len:
            continue
        ok = sum(1 for p in C[::2] if _dist_poly(p, L)[0] <= tol)
        if ok >= frac * len(C[::2]):
            out.append((l["road"], l["lane"]))
    return out


def short_key(r):
    """The decision number part of a row's unique key (for logs). / ユニークキーの短縮形。"""
    k = r.get("key", "")
    return k[20:24].lstrip("0") + "/" + k[24:].lstrip("0") if len(k) >= 32 else k


def turn_of(P, E, T):
    """'straight' / 'left' / 'right' for travel from E through P toward T (map frame,
    counter-clockwise positive). / 進行方向の区別。"""
    a = math.atan2(P[1] - E[1], P[0] - E[0])
    b = math.atan2(T[1] - P[1], T[0] - P[0])
    d = math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)
    return "straight" if abs(d) < 35.0 else ("left" if d > 0 else "right")
