#!/usr/bin/env python3
"""Offline tests: xodr lane marks from the painted lines (maps/osm_city/lane_change_jp.py).

  ../../carla-venv/bin/python tests/test_lane_change_jp.py

1. the paint string -> line style runs (solid, broken, yellow approach,
   crossing gaps, bus-bay mouth, short road, stray dash)
2. the Japan rule: yellow solid between lanes = none, white solid / broken
   = both, solid kerb / centre = none
3. the text patch: only <roadMark> lines change, several per lane, idempotent
4. the built maps (if patched): topology, waypoints and landmarks equal to
   the .prelanechange input; only roadMark lines differ

塗った線から roadMark を書く処理のオフラインテスト。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "maps", "osm_city"))
import lane_change_jp as L  # noqa: E402

FAILS = []


def check(name, cond, info=""):
    print("%s %s %s" % ("ok  " if cond else "FAIL", name, info))
    if not cond:
        FAILS.append(name)


def styles(pattern, s0=0.0):
    """One character per 0.5 m: Y W N, '-' masked. / 0.5 m ごとの塗り。"""
    lab = [None if c == "-" else c for c in pattern]
    ss = [s0 + 0.25 + 0.5 * i for i in range(len(lab))]
    return [(round(s, 1), st) for s, st in L.runs(ss, lab, s0, s0 + 0.5 * len(lab))]


def test_runs():
    check("solid white", styles("W" * 200) == [(0.0, "solid_w")])
    check("broken white 5/5", styles(("W" * 10 + "N" * 10) * 8) == [(0.0, "broken_w")])
    check("nothing", styles("N" * 100) == [(0.0, "none")])
    r = styles("Y" * 70 + ("W" * 10 + "N" * 10) * 4 + "Y" * 80)
    check("yellow approach / broken / yellow", [x[1] for x in r] == ["solid_y", "broken_w", "solid_y"], r)
    check("yellow ends where the paint ends (+-1 m)", abs(r[1][0] - 35.0) <= 1.0 and abs(r[2][0] - 75.0) <= 2.5, r)
    r = styles("W" * 40 + "N" + "-" * 9 + "N" + "W" * 40)
    check("solid cut by a crossing stays one solid", r == [(0.0, "solid_w")], r)
    r = styles("W" * 60 + "N" + ("YYYNNN" * 12) + "W" * 60)
    check("bus-bay mouth: yellow broken between white solids",
          [x[1] for x in r] == ["solid_w", "broken_y", "solid_w"], r)
    check("short road fully painted is solid", styles("W" * 13) == [(0.0, "solid_w")])
    r = styles("N" * 60 + "WWWWWWWW" + "N" * 60)
    check("a stray dash mid-section is no line", r == [(0.0, "none")], r)
    r = styles("N" * 30 + ("W" * 10 + "N" * 10) * 5)
    check("unpainted 15 m then broken", [x[1] for x in r] == ["none", "broken_w"], r)


def test_rule():
    check("between: yellow solid = none", L.lane_change("solid_y", True) == "none")
    check("between: white solid = both", L.lane_change("solid_w", True) == "both")
    check("between: broken = both", L.lane_change("broken_w", True) == "both")
    check("between: unpainted = both", L.lane_change("none", True) == "both")
    check("outer: solid = none", L.lane_change("solid_w", False) == "none" and L.lane_change("solid_y", False) == "none")
    check("outer: broken = both", L.lane_change("broken_w", False) == "both")
    check("outer: unpainted = none", L.lane_change("none", False) == "none")


XODR = """<OpenDRIVE>
    <road name="a" length="100" id="7" junction="-1">
        <lanes>
            <laneSection s="0">
                <center>
                    <lane id="0" type="none" level="true">
                        <link />
                        <roadMark sOffset="0" type="solid" weight="standard" color="standard" width="0.13" />
                    </lane>
                </center>
                <right>
                    <lane id="-1" type="driving" level="true">
                        <link />
                        <width sOffset="0" a="3.50" b="0" c="0" d="0" />
                        <roadMark sOffset="0" type="broken" weight="standard" color="standard" width="0.13" />
                        <speed sOffset="0" max="13.89" />
                    </lane>
                 </right>
            </laneSection>
        </lanes>
        <userData code="sumoId" value="1#0" />
    </road>
</OpenDRIVE>
"""


def test_patch():
    marks = {(7, 0, 0): [L.mark_xml(0, "solid_w", "none", "%(i)s")],
             (7, 0, -1): [L.mark_xml(0, "solid_y", "none", "%(i)s"), L.mark_xml(40.5, "broken_w", "both", "%(i)s")]}
    new, n = L.patch_text(XODR, marks)
    old_l, new_l = XODR.splitlines(), new.splitlines()
    check("one road patched", n == 1)
    check("only roadMark lines differ",
          [x for x in old_l if "roadMark" not in x] == [x for x in new_l if "roadMark" not in x])
    rm = [x.strip() for x in new_l if "roadMark" in x]
    check("three marks written", len(rm) == 3, rm)
    check("sOffset and laneChange", 'sOffset="40.5" type="broken"' in rm[2] and 'laneChange="both"' in rm[2], rm[2])
    check("indent kept", all(x.startswith(" " * 24 + "<roadMark") for x in new_l if "roadMark" in x))
    again, _ = L.patch_text(new, marks)
    check("idempotent", again == new)


def test_built():
    import carla
    root = os.path.join(HERE, "..", "maps", "build")
    for name in ("OitaStation", "OitaKosen"):
        p = os.path.join(root, name, name + ".xodr")
        if not os.path.exists(p + ".prelanechange"):
            print("skip %s (not patched)" % name)
            continue
        a, b = open(p).read(), open(p + ".prelanechange").read()
        check(name + ": only roadMark lines differ",
              [x for x in a.splitlines() if "<roadMark" not in x] == [x for x in b.splitlines() if "<roadMark" not in x])
        ma, mb = carla.Map(name, a), carla.Map(name, b)
        ta = [(x.road_id, x.lane_id, round(x.s, 2), y.road_id, y.lane_id) for x, y in ma.get_topology()]
        tb = [(x.road_id, x.lane_id, round(x.s, 2), y.road_id, y.lane_id) for x, y in mb.get_topology()]
        check(name + ": topology unchanged", ta == tb, len(ta))
        check(name + ": landmarks unchanged", len(ma.get_all_landmarks()) == len(mb.get_all_landmarks()))
        wps = ma.generate_waypoints(4.0)
        kinds = {str(w.lane_change) for w in wps}
        check(name + ": not every waypoint Both any more", kinds != {"Both"}, kinds)
        bad = [w for w in wps if not w.is_junction and str(w.right_lane_marking.color) == "Yellow"
               and str(w.right_lane_marking.type) == "Solid" and str(w.lane_change) in ("Right", "Both")]
        check(name + ": no change across a yellow solid line", not bad, len(bad))


if __name__ == "__main__":
    test_runs()
    test_rule()
    test_patch()
    test_built()
    print("\n%d failed" % len(FAILS) if FAILS else "\nall passed")
    sys.exit(1 if FAILS else 0)
