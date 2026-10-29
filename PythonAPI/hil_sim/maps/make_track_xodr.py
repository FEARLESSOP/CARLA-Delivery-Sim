"""OpenDRIVE road logic for a closed race track, from its centreline.

  python make_track_xodr.py <track.json> <out.xodr> [--name MarinaBay] [--lanes 3] [--speed 70]

track.json: {"offset": [dx, dy, dz], "points": [[x, y, z, width], ...]} -- the
centreline in the model's frame (Blender axes, metres), one point per ~1 m,
in race direction, starting at the start/finish line. offset is added to
every point (the same shift applied to the 3-D scene).

The lap becomes two roads joined into a ring by two short connecting roads
inside junctions (CARLA only follows road-to-road links through junctions). Each is a chain of paramPoly3 curves, every
~4 m, through the centreline points with continuous heading (Hermite), so
it closes exactly. The lanes are all on the right of the reference line,
so the whole track is one-way in race direction: ``--lanes`` driving lanes
that share the measured width, kept centred on the centreline by a lane
offset. The elevation follows the measured height.

Blender and OpenDRIVE flip y the same way on their way into CARLA, so the
centreline is written as is.

閉じたサーキットのセンターラインから OpenDRIVE（道路情報）を作る。周回を 2 本の
道路に分けて輪にし、全車線をレース方向の一方通行にする。
"""

import argparse
import json
import math

SEG_M = 4.0           # geometry piece length / 幾何要素の長さ
REC_M = 20.0          # width / offset / elevation record spacing / 幅・高さの記録間隔


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pieces(P):
    """Hermite paramPoly3 pieces through P[i] (closed). / 区間ごとの paramPoly3。"""
    n = len(P)
    head = [math.atan2(P[(i + 1) % n][1] - P[i - 1][1], P[(i + 1) % n][0] - P[i - 1][0])
            for i in range(n)]
    out = []
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        h0, h1 = head[i], head[(i + 1) % n]
        c, s = math.cos(-h0), math.sin(-h0)
        ex, ey = dx * c - dy * s, dx * s + dy * c          # end in the start frame / 始点座標系の終点
        t0 = (L, 0.0)
        dh = wrap(h1 - h0)
        t1 = (L * math.cos(dh), L * math.sin(dh))
        cu, cv = 3 * ex - 2 * t0[0] - t1[0], 3 * ey - 2 * t0[1] - t1[1]
        du, dv = -2 * ex + t0[0] + t1[0], -2 * ey + t0[1] + t1[1]
        # arc length of the curve (Simpson) / 曲線長
        def speed(p):
            return math.hypot(t0[0] + 2 * cu * p + 3 * du * p * p, 2 * cv * p + 3 * dv * p * p)
        arc = sum((speed(k / 8) + 4 * speed((k + 0.5) / 8) + speed((k + 1) / 8)) / 6 / 8
                  for k in range(8))
        out.append({"x": a[0], "y": a[1], "hdg": h0, "len": arc,
                    "bU": t0[0], "cU": cu, "dU": du, "bV": 0.0, "cV": cv, "dV": dv})
    return out


def records(values, s_list, step):
    """Linear (a, b) records every ``step`` m. / step ごとの線形記録。"""
    recs, j = [], 0
    total = s_list[-1]
    s = 0.0
    while s < total - 1e-6:
        e = min(s + step, total)
        def at(q):
            k = max(i for i in range(len(s_list)) if s_list[i] <= q + 1e-9)
            if k >= len(s_list) - 1:
                return values[-1]
            u = (q - s_list[k]) / max(s_list[k + 1] - s_list[k], 1e-9)
            return values[k] + (values[k + 1] - values[k]) * u
        va, vb = at(s), at(e)
        recs.append((s, va, (vb - va) / max(e - s, 1e-9)))
        s = e
    return recs


def road_xml(rid, pcs, z, w, lanes, speed, pred, succ, junction=-1):
    s_list = [0.0]
    for p in pcs:
        s_list.append(s_list[-1] + p["len"])
    length = s_list[-1]
    g = []
    for p, s in zip(pcs, s_list):
        g.append('      <geometry s="%.4f" x="%.4f" y="%.4f" hdg="%.6f" length="%.4f">'
                 '<paramPoly3 aU="0" bU="%.6f" cU="%.6f" dU="%.6f" aV="0" bV="0" cV="%.6f" dV="%.6f" '
                 'pRange="normalized"/></geometry>'
                 % (s, p["x"], p["y"], p["hdg"], p["len"], p["bU"], p["cU"], p["dU"], p["cV"], p["dV"]))
    elev = "\n".join('      <elevation s="%.4f" a="%.4f" b="%.6f" c="0" d="0"/>' % r
                     for r in records(z, s_list, REC_M))
    offs = "\n".join('      <laneOffset s="%.4f" a="%.4f" b="%.6f" c="0" d="0"/>' % (s, a / 2, b / 2)
                     for s, a, b in records(w, s_list, REC_M))
    right = []
    for k in range(1, lanes + 1):
        widths = "\n".join('            <width sOffset="%.4f" a="%.4f" b="%.6f" c="0" d="0"/>'
                           % (s, a / lanes, b / lanes) for s, a, b in records(w, s_list, REC_M))
        mark = "solid" if k == lanes else "broken"
        right.append('''          <lane id="-%d" type="driving" level="false">
            <link><predecessor id="-%d"/><successor id="-%d"/></link>
%s
            <roadMark sOffset="0" type="%s" weight="standard" color="white" width="0.15"/>
          </lane>''' % (k, k, k, widths, mark))
    def ref(r):
        kind, eid, cp = r
        return ('elementType="junction" elementId="%d"' % eid if kind == "junction"
                else 'elementType="road" elementId="%d" contactPoint="%s"' % (eid, cp))
    return '''  <road name="Track%d" length="%.4f" id="%d" junction="%d">
    <link>
      <predecessor %s/>
      <successor %s/>
    </link>
    <type s="0" type="rural"><speed max="%d" unit="km/h"/></type>
    <planView>
%s
    </planView>
    <elevationProfile>
%s
    </elevationProfile>
    <lateralProfile/>
    <lanes>
%s
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false">
            <roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.15"/>
          </lane>
        </center>
        <right>
%s
        </right>
      </laneSection>
    </lanes>
  </road>''' % (rid, length, rid, junction, ref(pred), ref(succ), speed, "\n".join(g), elev, offs,
                 "\n".join(right))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("track")
    ap.add_argument("out")
    ap.add_argument("--name", default="MarinaBay")
    ap.add_argument("--lanes", type=int, default=3)
    ap.add_argument("--speed", type=int, default=70, help="speed limit km/h (traffic manager)")
    a = ap.parse_args()
    t = json.load(open(a.track))
    ox, oy, oz = t["offset"]
    pts = [(p[0] + ox, p[1] + oy, p[2] + oz, p[3]) for p in t["points"]]
    step = max(1, int(round(SEG_M / (sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                                         for i in range(len(pts) - 1)) / (len(pts) - 1)))))
    P = pts[::step]
    pcs = pieces(P)
    n, half = len(P), len(P) // 2
    # road 1: pieces [0, half-1), connector 3 (junction 100): piece half-1,
    # road 2: [half, n-1), connector 4 (junction 101): piece n-1
    # 道路 1・2 と、接続道路 3・4（交差点 100・101 内）
    spans = [(1, 0, half - 1, ("junction", 101, None), ("junction", 100, None), -1),
             (3, half - 1, half, ("road", 1, "end"), ("road", 2, "start"), 100),
             (2, half, n - 1, ("junction", 100, None), ("junction", 101, None), -1),
             (4, n - 1, n, ("road", 2, "end"), ("road", 1, "start"), 101)]
    roads = []
    for rid, lo, hi, pred, succ, junc in spans:
        idx = list(range(lo, hi))
        z = [P[i][2] for i in idx] + [P[hi % n][2]]
        w = [P[i][3] for i in idx] + [P[hi % n][3]]
        roads.append(road_xml(rid, [pcs[i] for i in idx], z, w, a.lanes, a.speed, pred, succ, junc))
    links = "".join('<laneLink from="-%d" to="-%d"/>' % (k, k) for k in range(1, a.lanes + 1))
    for jid, incoming, conn in ((100, 1, 3), (101, 2, 4)):
        roads.append('''  <junction id="%d" name="lap%d">
    <connection id="0" incomingRoad="%d" connectingRoad="%d" contactPoint="start">%s</connection>
  </junction>''' % (jid, jid, incoming, conn, links))
    xs = [p[0] for p in P]; ys = [p[1] for p in P]
    xml = '''<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="%s" version="1.00" north="%.2f" south="%.2f" east="%.2f" west="%.2f"/>
%s
</OpenDRIVE>
''' % (a.name, max(ys), min(ys), max(xs), min(xs), "\n".join(roads))
    open(a.out, "w").write(xml)
    print("%s: %d geometry pieces, lap %.0f m, %d lanes, %d km/h"
          % (a.out, len(pcs), sum(p["len"] for p in pcs), a.lanes, a.speed))


if __name__ == "__main__":
    main()
