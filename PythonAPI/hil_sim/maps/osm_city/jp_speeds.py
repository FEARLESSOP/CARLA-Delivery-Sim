"""Japanese speed limits for a built map's roads (最高速度), and the outer ring.

  maps/.venv/bin/python osm_city/jp_speeds.py <build dir> <in.xodr> <out.xodr> [--ring 100] [--osm <map.osm>]
  maps/.venv/bin/python osm_city/jp_speeds.py --test

Per road (guide ch. 29):
1. the outer ring road ("Ring Road"): ``--ring`` km/h (maps.json "ring_speed_kmh");
2. a real posted limit: the JARTIC 最高速度 zone (crossings.pkl "speed_set")
   or a real OSM maxspeed tag (the map's original .osm, not the class
   defaults roadnet.py adds);
3. otherwise the legal limit (道路交通法施行令 11 条, revised 2026-09-01):
   60 km/h on a road with a centre line, a median, or two or more lanes one way;
   30 km/h on a road without (residential and narrow streets). A centre line is
   where markings_jp.py paints one: two-way, tertiary and up (or the ring),
   carriageway >= 5.5 m, or two or more lanes each way.
Junction connectors take the lower of the two roads they join, and keep a
slower curve limit netconvert gave them.

Only the lanes' <speed max> values change (the xodr text is patched in place,
never regenerated: signs.py must not be re-run on a live map, guide ch. 25).
With ``--ring`` it also adds a 274 speed post at the start of every ring road
that follows a junction where a street joins (osm_city/speed_spot.py picks a
free kerb). Writes <build dir>/speeds.json (every road: old, new, why).

日本の最高速度: 外周道路は指定値、規制（JARTIC・OSM の実タグ）はその値、
それ以外は法定速度（中央線あり 60、なし 30）。
"""

import collections
import json
import os
import pickle
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
TERT_UP = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
           "secondary_link", "tertiary", "tertiary_link"}
CENTRE_MIN = 5.5           # carriageway for a centre line (= markings_jp.CENTRE_MIN) / 中央線の最小幅員
LEGAL_CENTRE, LEGAL_NONE = 60, 30
RING_POST_MIN = 40.0       # a ring road this long gets a 100 post after a joining street / 標識を置く最短長


def legal_limit(two_way, tert_up, carriageway_m, lanes_one_way):
    """Legal limit on a road without a sign (km/h). / 標識の無い道路の法定速度。"""
    if lanes_one_way >= 2:
        return LEGAL_CENTRE                    # a centre line, or lanes (車両通行帯) one way
    if two_way:
        return LEGAL_CENTRE if tert_up and carriageway_m >= CENTRE_MIN else LEGAL_NONE
    return LEGAL_CENTRE if tert_up else LEGAL_NONE     # one-way: a dual carriageway half (median) / 分離帯


def way_of(road):
    u = road.find("userData[@code='sumoId']")
    if u is None:
        return None, False
    v = u.get("value")
    return v.lstrip("-").split("#")[0], v.startswith("-")


def driving_lanes(road):
    sec = road.find("lanes/laneSection")
    out = []
    for side in ("left", "right"):
        g = sec.find(side) if sec is not None else None
        for ln in (g.findall("lane") if g is not None else []):
            if ln.get("type") == "driving":
                w = ln.find("width")
                out.append(float(w.get("a")) if w is not None else 3.0)
    return out


def kmh_of(road):
    v = [float(s.get("max")) * 3.6 for s in road.iter("speed") if s.get("max")]
    return round(min(v)) if v else None


def osm_tags(path):
    ways = {}
    for w in ET.parse(path).getroot().findall("way"):
        ways[w.get("id")] = {t.get("k"): t.get("v") for t in w.findall("tag")}
    return ways


def real_maxspeed(orig, way):
    """A maxspeed tag in the user's own .osm (split ways keep their id + 2 digits).
    元の OSM の maxspeed（分割したウェイは ID + 2 桁）。"""
    for w in (way, way[:-2] if way and len(way) > 2 else None):
        t = orig.get(w) if w else None
        if t and t.get("maxspeed"):
            m = re.match(r"\s*(\d+)", t["maxspeed"])
            if m:
                return int(m.group(1))
    return None


def plan(root, tags, orig, jartic, ring_kmh=None):
    """{road id: (km/h, why)} for every road. / 全道路の制限速度と理由。"""
    roads = {int(r.get("id")): r for r in root.findall("road")}
    by_way = collections.defaultdict(list)
    for rid, r in roads.items():
        if r.get("junction") == "-1":
            w, rev = way_of(r)
            if w:
                by_way[w].append((rid, rev))
    out = {}
    for rid, r in roads.items():
        if r.get("junction") != "-1":
            continue
        w, rev = way_of(r)
        t = tags.get(w, {})
        if ring_kmh and r.get("name") == "Ring Road":
            out[rid] = (ring_kmh, "outer ring")
            continue
        if rid in jartic:
            out[rid] = (int(jartic[rid]), "JARTIC 最高速度 zone")
            continue
        real = real_maxspeed(orig, w) if w else None
        if real:
            out[rid] = (real, "OSM maxspeed tag")
            continue
        lanes = driving_lanes(r)
        partner = [p for p, prev in by_way.get(w, ()) if prev != rev]
        two_way = bool(partner)
        width = sum(lanes) + (sum(driving_lanes(roads[partner[0]])) if two_way else 0.0)
        tert = t.get("highway") in TERT_UP
        try:
            tagged = int(float((t.get("lanes") or "0").split(";")[0]))
        except ValueError:
            tagged = 0
        per_dir = max(len(lanes), tagged // 2 if two_way else tagged)
        v = legal_limit(two_way, tert, width, per_dir)
        out[rid] = (v, "legal: %s, %s, %.1f m, %d lane(s) one way" % (
            "two-way" if two_way else "one-way", t.get("highway", "?"), width, per_dir))
    # junction connectors: the lower of the two roads, a slower curve limit kept
    # 交差点内: 接続する道路の低い方（netconvert の曲線の制限が低ければそれ）
    for rid, r in roads.items():
        if r.get("junction") == "-1":
            continue
        ends = [int(e.get("elementId")) for e in r.find("link") if e.get("elementType") == "road"] \
            if r.find("link") is not None else []
        new = [out[e][0] for e in ends if e in out]
        old_ends = [kmh_of(roads[e]) for e in ends if e in roads and kmh_of(roads[e])]
        old = kmh_of(r)
        if not new:
            continue
        v = min(new)
        why = "connector: lower of the roads it joins"
        if old and old_ends and old < min(old_ends) - 0.5 and old < v:
            v, why = old, "connector: netconvert's curve limit"
        out[rid] = (v, why)
    return out


def patch_text(text, speeds):
    """Set every lane <speed max> of each road to its new limit (m/s), text only.
    xodr を文字列のまま書き換える（各道路の車線 speed max）。"""
    out, pos, n = [], 0, 0
    for m in re.finditer(r'<road\b[^>]*\bid="(\d+)"[^>]*>.*?</road>', text, re.S):
        rid = int(m.group(1))
        if rid not in speeds:
            continue
        ms = "%.2f" % (speeds[rid][0] / 3.6)
        body = m.group(0)
        new, k = re.subn(r'(<speed\b[^>]*\bmax=")[0-9.]+(")', lambda q: q.group(1) + ms + q.group(2), body)
        if new != body:
            out.append(text[pos:m.start()])
            out.append(new)
            pos = m.end()
            n += k
    out.append(text[pos:])
    return "".join(out), n


def drop_unreal_posts(text, speeds):
    """Remove 274 posts that only showed a class default: on roads with no real
    limit (legal now) whose value disagrees. They were placed from roadnet.py's
    invented 40 for wide estate streets, not from JARTIC or a real OSM tag.
    実在しない速度標識（道路種別の既定値由来）を外す。"""
    gone = []

    def road_sub(m):
        rid = int(m.group(1))
        kmh, why = speeds.get(rid, (None, ""))
        if not why.startswith("legal"):
            return m.group(0)

        def sig_sub(q):
            v = re.search(r'\bvalue="([0-9.]+)"', q.group(0))
            if 'type="274"' in q.group(0) and v and int(float(v.group(1))) != kmh:
                gone.append((rid, int(float(v.group(1))), kmh))
                return ""
            return q.group(0)
        return re.sub(r'<signal\b[^>]*/>', sig_sub, m.group(0))
    text = re.sub(r'<road\b[^>]*\bid="(\d+)"[^>]*>.*?</road>', road_sub, text, flags=re.S)
    return text, gone


def ring_posts(root, speeds, ring_kmh):
    """Ring roads that get a speed post: after a junction where a street that is
    not the ring joins, long enough. / 外周道路で速度標識を置く道路。"""
    roads = {int(r.get("id")): r for r in root.findall("road")}
    entering = collections.defaultdict(set)
    for r in roads.values():
        if r.get("junction") != "-1":
            continue
        su = r.find("link/successor")
        if su is not None and su.get("elementType") == "junction":
            entering[su.get("elementId")].add(int(r.get("id")))
    posts = []
    for rid, r in roads.items():
        if r.get("name") != "Ring Road" or r.get("junction") != "-1" or float(r.get("length")) < RING_POST_MIN:
            continue
        pr = r.find("link/predecessor")
        if pr is None or pr.get("elementType") != "junction":
            continue
        if any(roads[e].get("name") != "Ring Road" for e in entering.get(pr.get("elementId"), ())):
            posts.append(rid)
    return posts


def add_posts(text, build, posts, kmh, s=8.0):
    """Add one 274 signal per road in ``posts`` (t from speed_spot.Spots).
    速度標識（274）を追加する。"""
    sys.path.insert(0, HERE)
    import speed_spot
    meta = json.load(open(os.path.join(build, "meta.json")))
    spots = speed_spot.Spots(build, meta)
    ids = [int(x) for x in re.findall(r'<signal\b[^>]*\bid="(\d+)"', text)]
    sid = max(ids + [900000]) + 1
    added, skipped = [], []
    for rid in posts:
        t, side, why = spots.choose(rid, s, 2.0)
        if t is None:
            skipped.append((rid, why))
            continue
        # subtype = the value: CARLA picks the post model by subtype
        # (TrafficLightManager.SpeedLimitModels["100"]); "none" spawned nothing
        # subtype は速度値（CARLA は subtype で標識モデルを選ぶ）
        sig = ('<signal s="%.1f" t="%.2f" id="%d" name="speed_%d" dynamic="no" orientation="+" zOffset="0" '
               'country="OpenDRIVE" type="274" subtype="%d" value="%d" unit="km/h" height="2.2" width="0.6" />'
               % (s, t, sid, kmh, kmh, kmh))
        m = re.search(r'(<road\b[^>]*\bid="%d"[^>]*>)(.*?)(</road>)' % rid, text, re.S)
        body = m.group(2)
        if "<signals>" in body:
            body = body.replace("<signals>", "<signals>" + sig, 1)
        elif "<signals/>" in body:
            body = body.replace("<signals/>", "<signals>" + sig + "</signals>", 1)
        else:
            body = body + "<signals>" + sig + "</signals>"
        text = text[:m.start(2)] + body + text[m.end(2):]
        added.append((rid, sid, round(t, 2), side))
        sid += 1
    return text, added, skipped


def self_test():
    assert legal_limit(True, True, 6.0, 1) == 60          # tertiary with a centre line / 中央線あり
    assert legal_limit(True, False, 6.0, 1) == 30         # residential: no centre line painted / 生活道路
    assert legal_limit(True, True, 5.0, 1) == 30          # too narrow for a centre line / 狭い
    assert legal_limit(False, True, 3.2, 1) == 60         # dual carriageway half / 分離帯あり
    assert legal_limit(False, False, 3.0, 1) == 30        # one-way estate street / 一方通行の生活道路
    assert legal_limit(False, False, 6.5, 2) == 60        # one-way with two lanes / 2 車線
    x = ET.fromstring(
        '<OpenDRIVE>'
        '<road name="Ring Road" id="1" junction="-1" length="200"><link><predecessor elementType="junction" elementId="9"/></link>'
        '<lanes><laneSection><left><lane id="-1" type="driving"><width a="3.5"/><speed sOffset="0" max="11.11"/></lane></left></laneSection></lanes>'
        '<userData code="sumoId" value="-114#0"/></road>'
        '<road name="" id="2" junction="-1" length="80"><link><successor elementType="junction" elementId="9"/></link>'
        '<lanes><laneSection><left><lane id="-1" type="driving"><width a="2.9"/><speed sOffset="0" max="11.11"/></lane></left></laneSection></lanes>'
        '<userData code="sumoId" value="500#0"/></road>'
        '<road name="" id="3" junction="-1" length="80">'
        '<lanes><laneSection><left><lane id="-1" type="driving"><width a="2.9"/><speed sOffset="0" max="11.11"/></lane></left></laneSection></lanes>'
        '<userData code="sumoId" value="-500#0"/></road>'
        '<road name="" id="4" junction="9" length="10"><link><predecessor elementType="road" elementId="2"/><successor elementType="road" elementId="1"/></link>'
        '<lanes><laneSection><left><lane id="-1" type="driving"><width a="3"/><speed sOffset="0" max="8.00"/></lane></left></laneSection></lanes></road>'
        '</OpenDRIVE>')
    sp = plan(x, {"500": {"highway": "residential"}}, {}, {}, ring_kmh=100)
    assert sp[1][0] == 100 and sp[2][0] == 30 and sp[3][0] == 30, sp
    assert sp[4] == (29, "connector: netconvert's curve limit"), sp[4]   # its 28.8 km/h curve is kept / 曲線の制限を維持
    sp2 = plan(x, {"500": {"highway": "tertiary"}}, {}, {}, ring_kmh=100)
    assert sp2[2][0] == 60 and sp2[4][1] == "connector: netconvert's curve limit" and sp2[4][0] == 29, sp2
    assert plan(x, {}, {"500": {"maxspeed": "40"}}, {2: 50})[2] == (50, "JARTIC 最高速度 zone")
    assert plan(x, {}, {"500": {"maxspeed": "40"}}, {})[3] == (40, "OSM maxspeed tag")
    text = ET.tostring(x, encoding="unicode")
    new, n = patch_text(text, {1: (100, ""), 2: (30, "")})
    assert n == 2 and 'max="27.78"' in new and 'max="8.33"' in new and 'max="8.00"' in new, new
    assert ring_posts(x, sp, 100) == [1]
    t2 = re.sub(r'<userData code="sumoId" value="500#0" ?/>',
                '<signals><signal id="7" type="274" value="40"/></signals><userData code="sumoId" value="500#0"/>', text)
    t3, gone = drop_unreal_posts(t2, sp)
    assert gone == [(2, 40, 30)] and 'type="274"' not in t3, gone
    _, kept = drop_unreal_posts(t2, {2: (40, "JARTIC 最高速度 zone")})
    assert kept == []
    print("jp_speeds self-test OK / 自己テスト OK")


def main():
    if sys.argv[1:] == ["--test"]:
        self_test()
        return
    args = sys.argv[1:]
    build, src, dst = args[:3]
    ring = int(args[args.index("--ring") + 1]) if "--ring" in args else None
    osm = args[args.index("--osm") + 1] if "--osm" in args else None
    name = os.path.basename(os.path.normpath(build))
    text = open(src).read()
    root = ET.fromstring(text)
    tags = osm_tags(os.path.join(build, name + ".ring.osm"))
    orig = osm_tags(osm) if osm else {}
    cx = pickle.load(open(os.path.join(build, "crossings.pkl"), "rb"))
    jartic = {int(k): v for k, v in (cx.get("speed_set") or {}).items()}
    speeds = plan(root, tags, orig, jartic, ring)
    old = {int(r.get("id")): kmh_of(r) for r in root.findall("road")}
    new_text, n = patch_text(text, speeds)
    new_text, gone = drop_unreal_posts(new_text, speeds)
    added, skipped = [], []
    if ring:
        have = {int(m.group(1)) for m in re.finditer(r'<road\b[^>]*\bid="(\d+)"[^>]*>(?:(?!</road>).)*type="274"', new_text, re.S)}
        posts = [r for r in ring_posts(root, speeds, ring) if r not in have]
        new_text, added, skipped = add_posts(new_text, build, posts, ring)
    ET.fromstring(new_text)                    # still well-formed / 整形式の確認
    open(dst, "w").write(new_text)
    report = {"roads": {str(k): {"old": old.get(k), "new": v, "why": w} for k, (v, w) in sorted(speeds.items())},
              "ring_kmh": ring, "posts_added": added, "posts_skipped": skipped,
              "posts_removed": gone}
    json.dump(report, open(os.path.join(build, "speeds.json"), "w"), ensure_ascii=False, indent=1)
    plain = [v for k, (v, w) in speeds.items() if not w.startswith("connector")]
    print("[jp_speeds] %d lane speeds set on %d roads; non-junction roads by km/h %s; why %s; ring posts %d (%d skipped); "
          "%d unreal posts removed" % (n, len(speeds), sorted(collections.Counter(plain).items()),
                                       dict(collections.Counter(w.split(":")[0] for v, w in speeds.values())),
                                       len(added), len(skipped), len(gone)))


if __name__ == "__main__":
    main()
