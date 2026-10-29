"""Live link between delivery_sim.py and dashboard.py, through files (no carla).

delivery_sim writes ``map.json`` once (roads, buildings, charge station) and
``state.json`` a couple of times a second (vans, drones, orders, wind...);
both are replaced atomically so the dashboard never reads half a file. The
dashboard appends commands (a gust, a new base wind...) to
``commands.jsonl`` as one JSON object per line; delivery_sim reads the new
lines. Files keep working if either side restarts.

delivery_sim.py と dashboard.py をファイルで結ぶ（carla 非依存）。
delivery_sim は ``map.json``（道路・建物・充電所）を一度、``state.json`` を
毎秒数回アトミックに書き換える。ダッシュボードは ``commands.jsonl`` に
コマンド（突風など）を 1 行 1 JSON で追記し、delivery_sim が新しい行を読む。
"""

import json
import os
import time

DEFAULT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "data", "delivery_live")
STATE, MAP, COMMANDS = "state.json", "map.json", "commands.jsonl"


def write_json(path, obj):
    tmp = "%s.%d.tmp" % (path, os.getpid())
    with open(tmp, "w") as f:
        json.dump(obj, f, separators=(",", ":"))
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class LiveWriter(object):
    """The simulation's side. / シミュレーション側。"""

    def __init__(self, folder=DEFAULT_DIR):
        self.dir = folder
        os.makedirs(folder, exist_ok=True)
        self._cmd = os.path.join(folder, COMMANDS)
        open(self._cmd, "w").close()        # forget commands from an earlier run
        self._offset = 0

    def write_map(self, obj):
        write_json(os.path.join(self.dir, MAP), obj)

    def write_state(self, obj):
        obj["wall_time"] = time.time()
        write_json(os.path.join(self.dir, STATE), obj)

    def commands(self):
        """New commands since the last call. / 前回以降の新しいコマンド。"""
        try:
            with open(self._cmd, "rb") as f:    # bytes, so the offset is exact
                f.seek(self._offset)
                lines = f.readlines()
                if lines and not lines[-1].endswith(b"\n"):
                    lines = lines[:-1]      # still being written / 書き込み途中
                self._offset += sum(len(l) for l in lines)
        except OSError:
            return []
        out = []
        for line in lines:
            try:
                out.append(json.loads(line.decode("utf-8")))
            except ValueError:
                pass
        return out

    def close(self):
        """Mark the state as finished so the dashboard stops showing it live."""
        s = read_json(os.path.join(self.dir, STATE)) or {}
        s["running"] = False
        write_json(os.path.join(self.dir, STATE), s)


def send_command(folder, cmd):
    """The dashboard's side: queue one command. / ダッシュボード側。"""
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, COMMANDS), "a") as f:
        f.write(json.dumps(cmd) + "\n")
