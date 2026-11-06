"""Live control of a running pipeline, and the launcher's requests.

The dashboard writes requests, run_pipeline.py / launcher.py read them
(atomic JSON files with a sequence number, like map_switch.py and
windows_control.py):

- ``data/pipeline_control.json``: requests to the running pipeline
  (traffic, delivery, map, car, helpers on/off, stop)
- ``data/pipeline_state.json``: what the pipeline runs now
- ``data/launch_request.json``: requests to the launcher (launch, stop, relaunch)
- ``data/launcher_state.json``: idle / starting / running / stopping

動作中のパイプラインとランチャーへの依頼・状態ファイル。
"""

import json
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
CONTROL_PATH = os.path.join(DATA, "pipeline_control.json")
STATE_PATH = os.path.join(DATA, "pipeline_state.json")
LAUNCH_PATH = os.path.join(DATA, "launch_request.json")
LAUNCHER_STATE_PATH = os.path.join(DATA, "launcher_state.json")
LOG_PATH = os.path.join(DATA, "pipeline.log")

ACTIONS = ("traffic", "delivery", "map", "vehicle", "helper", "stop")
LAUNCH_ACTIONS = ("launch", "stop", "relaunch")


def write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(obj, fh)
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def request(path, action, **kw):
    """Write a request with the next sequence number. / 次の番号で依頼を書く。"""
    old = read_json(path) or {}
    req = dict(kw, action=action, seq=int(old.get("seq", 0)) + 1, t=time.time())
    write_json(path, req)
    return req


class Reader(object):
    """Hands out each new request once; requests already there at start are
    old (an earlier run) and skipped. / 新しい依頼を 1 回だけ渡す（起動前の依頼は無視）。"""

    def __init__(self, path, actions):
        self.path, self.actions = path, actions
        old = read_json(path) or {}
        self.seq = int(old.get("seq", 0))

    def take(self):
        req = read_json(self.path)
        if not req or int(req.get("seq", 0)) <= self.seq:
            return None
        self.seq = int(req.get("seq", 0))
        return req if req.get("action") in self.actions else None


def self_test():
    import tempfile
    p = os.path.join(tempfile.mkdtemp(), "c.json")
    request(p, "traffic", vehicles=5)
    r = Reader(p, ACTIONS)
    assert r.take() is None                      # was there before / 起動前の依頼
    request(p, "traffic", vehicles=7)
    got = r.take()
    assert got["action"] == "traffic" and got["vehicles"] == 7 and r.take() is None
    request(p, "bogus")
    assert r.take() is None
    print("pipeline_control self-test OK")


if __name__ == "__main__":
    self_test()
