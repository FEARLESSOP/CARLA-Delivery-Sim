#!/usr/bin/env python3
"""dashboard.py's POST /delivery/charge (the Delivery page's "Send to charge"
button): a valid van id appends {"cmd": "charge", "van": id} to the live
folder's commands.jsonl; anything else is a 400 and writes nothing. Flask test
client against a temporary delivery_live folder, no sim, no CARLA.

  ../../carla-venv/bin/python tests/test_delivery_charge_route.py

「充電へ」ルートの試験（一時フォルダ、シミュ・CARLA 不要）。
"""
import json
import os
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import dashboard  # noqa: E402
from delivery import live as delivery_live  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


def lines(folder):
    p = os.path.join(folder, delivery_live.COMMANDS)
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return [json.loads(x) for x in f if x.strip()]


print("=== POST /delivery/charge / 充電へ ===")
tmp = tempfile.mkdtemp(prefix="charge_route_")
dashboard.DELIVERY_DIR = tmp
c = dashboard.app.test_client()

r = c.post("/delivery/charge", json={"van": 3})
check("van 3: 200 and ok", r.status_code == 200 and r.get_json().get("ok") is True, r.get_json())
check("reply echoes the command", r.get_json().get("sent") == {"cmd": "charge", "van": 3}, r.get_json())
check("commands.jsonl gets the charge line", lines(tmp) == [{"cmd": "charge", "van": 3}], lines(tmp))

r = c.post("/delivery/charge", json={"van": 99})
check("van 99 (upper bound) accepted", r.status_code == 200 and lines(tmp)[-1] == {"cmd": "charge", "van": 99})
r = c.post("/delivery/charge", json={"van": 1})
check("van 1 (lower bound) accepted", r.status_code == 200 and lines(tmp)[-1] == {"cmd": "charge", "van": 1})

n = len(lines(tmp))
for bad in ({"van": 0}, {"van": 100}, {"van": -2}, {"van": "3"}, {"van": 2.5}, {"van": True},
            {"van": None}, {}, {"vans": 3}, [3]):
    r = c.post("/delivery/charge", json=bad)
    check("rejects %s with 400" % json.dumps(bad), r.status_code == 400 and r.get_json().get("ok") is False,
          r.status_code)
r = c.post("/delivery/charge", data="not json", content_type="application/json")
check("rejects a non-JSON body with 400", r.status_code == 400)
check("rejected requests write nothing", len(lines(tmp)) == n, lines(tmp))
check("GET is not allowed", c.get("/delivery/charge").status_code == 405)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
