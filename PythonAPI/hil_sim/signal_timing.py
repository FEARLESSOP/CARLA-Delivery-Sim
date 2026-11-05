"""Japanese signal timing for CARLA's traffic lights, from JARTIC type C.
JARTIC 交差点制御情報（タイプ C）から CARLA の信号機に日本の信号時間を設定する。

JARTIC open data type C (交差点制御情報, monthly per prefecture police) gives,
for each controlled intersection, its VICS links (定義) and every 5 minutes
its cycle length (s) and splits #1-#6 (% of the cycle) (制御). It has no
coordinates (VICS link geometry is licensed), so an intersection cannot be
matched to a map junction without other evidence. This tool therefore uses
the data as an **Oita-typical profile**: per junction size (3 or 4 arms),
weekday or weekend/holiday, and 5-minute slot of the day, the median cycle
and the median share of the main phase across the intersections of one VICS
second mesh (493165 holds the Oita Kosen map). Splits in a row with the same
right of way form one phase.
タイプ C には座標が無い（VICS リンク形状は有償）ため、地図の交差点と対応付けず、
同じ 2 次メッシュの交差点から「大分の典型」（規模・平日/休日・5 分毎）を作る。

CARLA runs a junction's controllers one after another, each Green -> Yellow
-> Red (the Red is the all-red before the next phase). The map's controllers
are phases (osm_city/signal_phases.py: "<cluster>_p1" main axis, "_p2" ...).
Japanese practice: yellow 3 s, all-red 2 s; green = cycle x share - 5 s.
CARLA は交差点のコントローラを順に回す（青→黄→赤、赤は全赤）。黄 3 秒・全赤 2 秒、
青 = サイクル × 比率 − 5 秒。

  python signal_timing.py profile <定義.csv> <制御.csv> [--mesh 493165]   build data/signal_timing_oita.json
  python signal_timing.py show [--time 08:00] [--day weekday|weekend]       print the plans
  python signal_timing.py apply [--time HH:MM] [--day ...] [--dry-run] [--watch]
                                                                           set them on the running world
  python signal_timing.py test                                             offline self-test (no server)

run_pipeline.py starts "apply --watch --only-map <map>" on maps whose maps.json
entry has "signal_timing" (the profile path). With --watch it re-applies when
the map changes or the 5-minute slot moves on.
maps.json に "signal_timing" があるマップでは run_pipeline.py が --watch で起動する。
"""

import argparse
import collections
import csv
import datetime
import json
import math
import os
import re
import statistics
import sys
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILE = os.path.join(HERE, "data", "signal_timing_oita.json")
YELLOW, ALL_RED, MIN_GREEN = 3.0, 2.0, 6.0
FW = "０１２３４５６７８９"
# Japanese national holidays 2026-2027, substitute holidays (振替休日) and the
# citizens' holiday (国民の休日) included (内閣府「国民の祝日」) / 国民の祝日・振替休日・国民の休日
HOLIDAYS = {
    "2026/01/01", "2026/01/12", "2026/02/11", "2026/02/23", "2026/03/20", "2026/04/29", "2026/05/03",
    "2026/05/04", "2026/05/05", "2026/05/06", "2026/07/20", "2026/08/11", "2026/09/21", "2026/09/22",
    "2026/09/23", "2026/10/12", "2026/11/03", "2026/11/23",
    "2027/01/01", "2027/01/11", "2027/02/11", "2027/02/23", "2027/03/21", "2027/03/22", "2027/04/29",
    "2027/05/03", "2027/05/04", "2027/05/05", "2027/07/19", "2027/08/11", "2027/09/20", "2027/09/23",
    "2027/10/11", "2027/11/03", "2027/11/23",
}


# -- profile from the CSVs / CSV からプロファイル -------------------------------------
def phases_of(row_split_rights, shares):
    """Phase shares from splits: consecutive splits with the same right of way
    (inflow-link pattern) are one phase; unused splits (all 0 / empty) dropped.
    同じ通行権のスプリットの連続を 1 現示にまとめる。"""
    out, last = [], None
    for pat, sh in zip(row_split_rights, shares):
        if not pat or set(pat) <= {"0", "-"} or sh <= 0:
            continue
        if pat == last:
            out[-1] += sh
        else:
            out.append(sh)
        last = pat
    return out


def build_profile(defs, ctl, mesh="493165"):
    """Read the two CSVs (cp932) into the profile dict. / 2 つの CSV からプロファイル。"""
    fw = lambda i: FW[i]
    inter = {}
    with open(defs, encoding="cp932", newline="") as fh:
        r = csv.reader(fh)
        h = next(r)
        for row in r:
            d = dict(zip(h, row))
            nin = int(d["流入リンク数"])
            meshes = {d["流入リンク＃%s定義_２次メッシュコード" % fw(i)] for i in range(1, nin + 1)}
            if mesh and mesh not in meshes:
                continue
            pats = []
            for s in range(1, 7):
                pats.append("".join(d.get("スプリット＃%s通行権_流入リンク＃%s" % (fw(s), fw(i)), "") or "-"
                                    for i in range(1, nin + 1)))
            inter[int(d["交差点番号"])] = {"arms": nin, "pats": pats}
    acc = collections.defaultdict(lambda: collections.defaultdict(list))
    dates = set()
    with open(ctl, encoding="cp932", newline="") as fh:
        r = csv.reader(fh)
        next(r)
        for row in r:
            n = int(row[2])
            if n not in inter or not row[3]:
                continue
            day = row[0][:10]
            dates.add(day)
            dt = datetime.datetime.strptime(day, "%Y/%m/%d")
            kind = "weekend" if dt.weekday() >= 5 or day in HOLIDAYS else "weekday"
            slot = (int(row[0][11:13]) * 60 + int(row[0][14:16])) // 5
            cyc = int(row[3])
            sh = [int(x) if x else 0 for x in row[4:10]]
            ph = phases_of(inter[n]["pats"], sh)
            if len(ph) < 2 or sum(ph) <= 0:
                continue
            main = max(ph) / float(sum(ph))
            size = "3" if inter[n]["arms"] <= 3 else "4"
            acc[(kind, size)][slot].append((cyc, main, len(ph)))
    prof = {"source": {"defs": os.path.basename(defs), "control": os.path.basename(ctl), "mesh": mesh,
                       "intersections": sorted(inter), "arms": {str(k): v["arms"] for k, v in inter.items()},
                       "dates": [min(dates), max(dates)] if dates else []},
            "yellow_s": YELLOW, "all_red_s": ALL_RED, "plans": {}}
    for (kind, size), slots in acc.items():
        seq = []
        for s in range(288):
            v = slots.get(s)
            if not v:
                seq.append(None)
                continue
            seq.append([round(statistics.median(c for c, m, p in v)), round(statistics.median(m for c, m, p in v), 3),
                        len(v)])
        prof["plans"]["%s_%s" % (kind, size)] = seq
    return prof


# -- plans / 信号計画 ----------------------------------------------------------------
def load_profile(path=PROFILE):
    return json.load(open(path))


def day_kind(when):
    return "weekend" if when.weekday() >= 5 or when.strftime("%Y/%m/%d") in HOLIDAYS else "weekday"


def plan_for(prof, size, kind, minute):
    """(cycle s, main-phase share) for a junction size at a minute of the day,
    from the nearest slot with data. / 規模・曜日・時刻のサイクルと主現示比率。"""
    key = "%s_%s" % (kind, "3" if str(size) == "3" else "4")
    seq = prof["plans"].get(key) or prof["plans"].get("weekday_4")
    s0 = int(minute // 5) % 288
    for k in range(288):
        for s in ((s0 - k) % 288, (s0 + k) % 288):
            if seq[s]:
                return float(seq[s][0]), float(seq[s][1])
    return 100.0, 0.6


def phase_times(cycle, main_share, n_phases):
    """[(green, yellow, red)] per phase: phase 1 (the main axis) gets the main
    share, the others split the rest evenly; each phase = green + 3 + 2 s and
    the phases ALWAYS add up to the cycle: greens below MIN_GREEN are raised and
    the time taken from the longer greens; if the cycle cannot give every phase
    MIN_GREEN, the green time is split evenly (a warning in the plan).
    現示ごとの (青, 黄, 赤)。合計は常にサイクルに一致（最小青は長い青から補う）。"""
    n = max(1, int(n_phases))
    avail = cycle - n * (YELLOW + ALL_RED)
    shares = [1.0] if n == 1 else [main_share] + [(1.0 - main_share) / (n - 1)] * (n - 1)
    if avail <= n * MIN_GREEN:
        greens = [max(avail, 0.0) / n] * n
    else:
        base = [cycle * s - YELLOW - ALL_RED for s in shares]      # green = cycle x split - clearance
        greens = list(base)
        fixed = [False] * n
        while True:
            low = [i for i in range(n) if not fixed[i] and greens[i] < MIN_GREEN]
            if not low:
                break
            for i in low:
                greens[i], fixed[i] = MIN_GREEN, True
            rest = avail - sum(g for g, f in zip(greens, fixed) if f)
            free = [i for i in range(n) if not fixed[i]]
            tot = sum(max(base[i], 1e-6) for i in free) or 1.0
            for i in free:
                greens[i] = rest * max(base[i], 1e-6) / tot
    greens = [round(g, 2) for g in greens]
    greens[0] = round(avail - sum(greens[1:]), 2)        # exact sum after rounding / 丸め誤差を第 1 現示で吸収
    return [(g, YELLOW, ALL_RED) for g in greens]


def phases_from_xodr(xodr_text):
    """{signal id: (cluster, phase index)} and {cluster: n phases} from the
    xodr's "<cluster>_p<k>" controllers. / xodr の現示コントローラから信号→現示。"""
    root = ET.fromstring(xodr_text)
    sig, n = {}, collections.Counter()
    for c in root.findall("controller"):
        m = re.match(r"(.+)_p(\d+)$", c.get("id", ""))
        if not m:
            continue
        n[m.group(1)] = max(n[m.group(1)], int(m.group(2)))
        for x in c.findall("control"):
            sig[x.get("signalId")] = (m.group(1), int(m.group(2)) - 1)
    return sig, dict(n)


def plan_world(lights, xodr_text, prof, when):
    """What to set on each light: [(light, green, yellow, red, why)].
    ``lights`` are carla.TrafficLight (or anything with get_opendrive_id and
    get_group_traffic_lights). 各信号機に設定する時間。"""
    sig, nph = phases_from_xodr(xodr_text)
    kind = day_kind(when)
    minute = when.hour * 60 + when.minute
    out, seen = [], set()
    for tl in lights:
        if tl.id in seen:
            continue
        group = list(tl.get_group_traffic_lights()) or [tl]
        for g in group:
            seen.add(g.id)
        size = 3 if len(group) <= 3 else 4
        cycle, main = plan_for(prof, size, kind, minute)
        for g in group:
            cl = sig.get(g.get_opendrive_id())
            if cl is None:
                # no phase controllers (the map has one controller per junction): even split
                # 現示コントローラ無し: 均等
                times = phase_times(cycle, 1.0 / max(1, len(group)), 1)[0]
                why = "no phases in the xodr"
            else:
                times = phase_times(cycle, main, nph[cl[0]])[cl[1]]
                why = "%s phase %d/%d, %s %s-arm, cycle %d s, main %.0f%%" % (
                    cl[0][-16:], cl[1] + 1, nph[cl[0]], kind, size, cycle, main * 100)
            out.append((g,) + tuple(times) + (why,))
    return out


def apply(world, prof, when, dry=False, reset=True):
    """Set the plan on every traffic light of the world. / 全信号機に設定する。"""
    lights = list(world.get_actors().filter("traffic.traffic_light"))
    rows = plan_world(lights, world.get_map().to_opendrive(), prof, when)
    groups = set()
    for tl, g, y, r, why in rows:
        print("  %-6s green %5.1f yellow %.0f red %.0f  %s" % (tl.get_opendrive_id(), g, y, r, why))
        if not dry:
            tl.set_green_time(g)
            tl.set_yellow_time(y)
            tl.set_red_time(r)
            groups.add(min(x.id for x in tl.get_group_traffic_lights()) if tl.get_group_traffic_lights() else tl.id)
    if reset and not dry:
        done = set()
        for tl, *_ in rows:
            key = min(x.id for x in tl.get_group_traffic_lights()) if tl.get_group_traffic_lights() else tl.id
            if key not in done:
                tl.reset_group()            # restart the cycle with the new times / 新しい時間で周期を再開
                done.add(key)
    return rows


# -- self-test / 自己テスト ----------------------------------------------------------
def self_test():
    """Offline: phases from splits, plan arithmetic, xodr phase reading, the
    world plan on stand-in lights. / オフライン確認。"""
    assert phases_of(["1100", "1100", "0011", "0011", "0000", "----"], [46, 6, 22, 6, 0, 0]) == [52, 28]
    assert phases_of(["101", "010", "101"], [40, 30, 30]) == [40, 30, 30]
    for cyc, main, n in ((100, 0.6, 2), (150, 0.7, 2), (70, 0.5, 3), (60, 0.9, 2)):
        pt = phase_times(cyc, main, n)
        assert len(pt) == n
        assert all(g >= MIN_GREEN and y == 3 and r == 2 for g, y, r in pt), pt
        assert abs(sum(g + y + r for g, y, r in pt) - cyc) < 0.01, (cyc, pt)
    for cyc, n in ((90, 11), (90, 6), (150, 6), (60, 4)):               # many phases / 現示が多い場合
        pt = phase_times(cyc, 0.6, n)
        assert abs(sum(g + y + r for g, y, r in pt) - cyc) < 0.01, (cyc, n, pt)
    xodr = ('<OpenDRIVE><controller id="c_p1"><control signalId="a"/><control signalId="b"/></controller>'
            '<controller id="c_p2"><control signalId="c"/></controller></OpenDRIVE>')
    sig, n = phases_from_xodr(xodr)
    assert sig == {"a": ("c", 0), "b": ("c", 0), "c": ("c", 1)} and n == {"c": 2}

    class TL(object):
        def __init__(self, i, od):
            self.id, self.od, self.grp = i, od, []

        def get_opendrive_id(self):
            return self.od

        def get_group_traffic_lights(self):
            return self.grp
    ls = [TL(1, "a"), TL(2, "b"), TL(3, "c")]
    for t in ls:
        t.grp = ls
    prof = {"plans": {"weekday_3": [[120, 0.7, 5]] * 288, "weekend_3": [[90, 0.6, 5]] * 288,
                      "weekday_4": [[100, 0.6, 5]] * 288}}
    rows = plan_world(ls, xodr, prof, datetime.datetime(2026, 9, 1, 8, 0))     # a Tuesday / 火曜
    t = {r[0].od: r[1:4] for r in rows}
    assert t["a"] == t["b"] and abs(t["a"][0] - (120 * 0.7 - 5)) < 0.2 and abs(t["c"][0] - (120 * 0.3 - 5)) < 0.2
    assert abs(t["a"][0] + t["c"][0] + 10 - 120) < 0.6
    rows = plan_world(ls, xodr, prof, datetime.datetime(2026, 9, 21, 8, 0))    # 敬老の日 / holiday
    assert abs({r[0].od: r[1] for r in rows}["a"] - (90 * 0.6 - 5)) < 0.2
    assert day_kind(datetime.datetime(2026, 11, 3)) == "weekend"          # 文化の日 (Tue) / 火曜の祝日
    assert day_kind(datetime.datetime(2026, 11, 23)) == "weekend"         # 勤労感謝の日 (Mon)
    assert day_kind(datetime.datetime(2027, 3, 22)) == "weekend"          # 振替休日
    assert day_kind(datetime.datetime(2026, 11, 4)) == "weekday"
    if os.path.exists(PROFILE):
        p = load_profile()
        for k, seq in p["plans"].items():
            assert len(seq) == 288 and any(seq), k
        c8, m8 = plan_for(p, 4, "weekday", 8 * 60)
        c3, m3 = plan_for(p, 4, "weekday", 3 * 60)
        assert 60 <= c3 <= c8 <= 180 and 0.5 <= m8 < 1.0, (c3, c8, m8)
    print("signal_timing self-test OK / 自己テスト OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("profile", help="build the profile from the type C CSVs / CSV からプロファイル作成")
    p.add_argument("defs")
    p.add_argument("control")
    p.add_argument("--mesh", default="493165")
    p.add_argument("-o", "--out", default=PROFILE)
    for name in ("show", "apply"):
        q = sub.add_parser(name)
        q.add_argument("--time", help="HH:MM (default: now) / 時刻（既定: 現在）")
        q.add_argument("--day", choices=("weekday", "weekend"))
        q.add_argument("--profile", default=PROFILE)
        if name == "apply":
            q.add_argument("--host", default="127.0.0.1")
            q.add_argument("--port", type=int, default=2000)
            q.add_argument("--dry-run", action="store_true", help="print only / 表示のみ")
            q.add_argument("--watch", action="store_true",
                           help="re-apply on map change or new 5-min slot / マップ変更・時間帯変化で再設定")
            q.add_argument("--only-map", help="apply only while this map is loaded / このマップの時だけ設定")
    sub.add_parser("test")
    a = ap.parse_args()
    if a.cmd == "test":
        self_test()
        return
    if a.cmd == "profile":
        prof = build_profile(a.defs, a.control, a.mesh)
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        json.dump(prof, open(a.out, "w"), ensure_ascii=False)
        print("profile: %d intersections in mesh %s, plans %s -> %s" % (
            len(prof["source"]["intersections"]), a.mesh, sorted(prof["plans"]), a.out))
        return
    prof = load_profile(a.profile)

    def when():
        w = datetime.datetime.now()
        if a.time:
            hh, mm = map(int, a.time.split(":"))
            w = w.replace(hour=hh, minute=mm)
        if a.day == "weekday":
            while day_kind(w) != "weekday":
                w -= datetime.timedelta(days=1)
        elif a.day == "weekend":
            while day_kind(w) != "weekend":
                w += datetime.timedelta(days=1)
        return w
    if a.cmd == "show":
        w = when()
        print("%s %s / 時刻 %s" % (w.strftime("%Y-%m-%d %H:%M"), day_kind(w), w.strftime("%H:%M")))
        for size in (3, 4):
            c, m = plan_for(prof, size, day_kind(w), w.hour * 60 + w.minute)
            print("  %d-arm: cycle %d s, main %.0f%%, 2 phases %s" % (size, c, m * 100, phase_times(c, m, 2)))
        return
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(20.0)
    last = None
    while True:
        try:
            world = client.get_world()
            map_name = world.get_map().name.split("/")[-1]
            w = when()
            key = (map_name, day_kind(w), (w.hour * 60 + w.minute) // 5)
            if a.only_map and map_name != a.only_map:
                key = last           # another map (F5): leave its lights alone / 他のマップは触らない
            elif key != last:
                world.wait_for_tick(20.0)    # synchronous world: actors after a tick / 同期モード
        except RuntimeError as exc:          # server busy or loading / サーバーが応答しない
            print("  no answer from the server (%s), retrying / 再試行" % exc)
            if not a.watch:
                raise
            time.sleep(10.0)
            continue
        if key != last:
            print("apply %s %s / 設定" % (key[0], w.strftime("%H:%M %a")))
            # restart the cycles only on a new map (first apply); a new 5-minute slot just
            # changes the times, which each light takes up at its next state change
            # 周期の再開は新しいマップの時だけ。時間帯の変化では時間のみ変更（次の切替から反映）
            rows = apply(world, prof, w, dry=a.dry_run, reset=last is None or last[0] != key[0])
            print("  %d lights / 信号機 %d 基" % (len(rows), len(rows)))
            last = key
        if not a.watch:
            break
        time.sleep(30.0)


if __name__ == "__main__":
    main()
