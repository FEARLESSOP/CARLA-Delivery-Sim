"""Signal poles as jp_signs.py computes them vs carla.Map (carla-venv; no shapely).

  carla-venv/bin/python check_signal_poles.py <build dir>

Reads <build>/<Name>.xodr and <build>/jp_plates.json ("signal_poles", written by
jp_signs.py), loads the xodr into carla.Map offline and compares every dynamic
signal's landmark position (CARLA y south -> map y north). PASS when all lie
within 0.05 m; also lists plates mounted on signal poles and their distance.
jp_signs.py の信号柱位置を carla.Map と照合する（0.05 m 以内で合格）。
"""

import json
import math
import os
import sys

import carla


def main():
    out = sys.argv[1]
    meta = json.load(open(os.path.join(out, "meta.json")))
    xodr = open(os.path.join(out, meta["name"] + ".xodr")).read()
    rec = json.load(open(os.path.join(out, "jp_plates.json")))
    ours = {i: (x, y) for i, x, y in rec.get("signal_poles", [])}
    m = carla.Map("check", xodr)
    worst, bad, n = 0.0, [], 0
    for lm in m.get_all_landmarks():
        if not lm.is_dynamic:
            continue
        n += 1
        loc = lm.transform.location
        if lm.id not in ours:
            bad.append("%s: not in jp_plates.json" % lm.id)
            continue
        d = math.hypot(ours[lm.id][0] - loc.x, ours[lm.id][1] + loc.y)
        worst = max(worst, d)
        if d > 0.05:
            bad.append("%s: %.3f m off carla.Map (%.3f, %.3f)" % (lm.id, d, loc.x, -loc.y))
    for p in rec["plates"]:
        if p.get("signal"):
            x, y = ours[p["signal"]]
            print("plate %s on %s: post %.3f m from the pole" % (p["kind"], p["signal"], math.hypot(p["x"] - x, p["y"] - y)))
    for b in bad:
        print("  " + b)
    print("signal poles: %d dynamic signals, worst %.3f m: %s" % (n, worst, "PASS" if not bad and n else "FAIL"))
    sys.exit(0 if not bad and n else 1)


if __name__ == "__main__":
    main()
