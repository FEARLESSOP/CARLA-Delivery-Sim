#!/usr/bin/env python3
"""Open / close the optional windows of a run_pipeline.py session from the
dashboard: the request and status files, and the pipeline-side supervisor.

The optional windows are the ones the driver can do without: the sensor viewer
(sensor_viewer.py) and the drone camera wall (drone_viewer.py). Both only
*read* the world -- they never tick it, change its settings or own actors
other than their own cameras -- so starting or stopping one never disturbs
the simulation. The driving window, the dashboard (it serves these controls),
traffic (the tick master), the delivery vans and force feedback are not
offered: stopping any of them changes or ends the session.

Protocol (JSON side channels next to the scripts, like map_request.json):

- ``pipeline_control.json`` -- written by the dashboard:
  ``{"drone_cams": {"action": "open", "t": 1730790000.1}, ...}``, one entry
  per window, the newest click wins. The pipeline never writes it (only clears
  it at startup), so the dashboard's read-modify-write cannot race it; the
  pipeline remembers the last ``t`` it carried out per window.
- ``pipeline_status.json`` -- written by the pipeline about once a second:
  which windows are open, their pids, whether they can be opened at all.

Closing a window -- from the dashboard (SIGTERM to that one process) or with
the window's own close button / Esc / Q (exit code 0) -- is a normal close:
no "died -> restarting", no shutdown. A crash (any other exit code) is still
restarted a bounded number of times. Which windows are open survives an F5
map switch: the pipeline starts the clients again from this state, not from
the launch flags.

This module imports no ``carla`` so tests/test_windows_control.py runs
without a server.

ダッシュボードから run_pipeline.py の任意ウィンドウを開閉する：依頼ファイル・
状態ファイルと、パイプライン側の管理クラス。

任意ウィンドウとは運転に必須でないもの：センサービューア（sensor_viewer.py）
とドローンカメラ一覧（drone_viewer.py）。どちらもワールドを*読むだけ*で、
tick もせず設定も変えず、自分のカメラ以外のアクターも持たないため、開閉しても
シミュレーションに影響しない。運転ウィンドウ、ダッシュボード（この操作を提供
する側）、交通（tick マスタ）、配送車、フォースフィードバックは対象外：止める
とセッションが変わるか終わるため。

方式（map_request.json と同じくスクリプト横の JSON ファイル）：

- ``pipeline_control.json`` — ダッシュボードが書く。ウィンドウごとに 1 項目、
  最新のクリックが有効。パイプラインは書かない（起動時に消すだけ）ので、
  ダッシュボードの読み書きと競合しない。実行済みの ``t`` をウィンドウごとに覚える。
- ``pipeline_status.json`` — パイプラインが約 1 秒ごとに書く：開いている
  ウィンドウ、pid、開けるかどうか。

ダッシュボードからの閉じる（そのプロセスだけに SIGTERM）も、ウィンドウ自身の
閉じるボタン / Esc / Q（終了コード 0）も通常の終了として扱う：「died ->
restarting」もシャットダウンも起こさない。クラッシュ（それ以外の終了コード）
は従来どおり回数制限付きで再起動する。開閉状態は F5 のマップ切替後も維持され、
起動オプションではなくこの状態からクライアントを再起動する。

本モジュールは ``carla`` を import しないため、tests/test_windows_control.py は
サーバ無しで動く。
"""

import json
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_PATH = os.path.join(HERE, "pipeline_control.json")
STATUS_PATH = os.path.join(HERE, "pipeline_status.json")
# A status file not rewritten for this long means no pipeline is supervising
# (it writes about once a second; a map switch can pause it for a while).
# これより古い状態ファイルはパイプラインが動いていないことを示す
STATUS_MAX_AGE_S = 15.0
# Crash restarts a window gets each time it is opened / 開くたびの再起動回数
WINDOW_RESTARTS = 3

# The optional windows, in dashboard order: name -> labels and description.
# 任意ウィンドウ（ダッシュボードの表示順）：名前 -> 表示名と説明
WINDOWS = {
    "sensor_viewer": {
        "en": "Sensor viewer", "ja": "センサービューア",
        "desc_en": "RGB front/rear, segmentation, LiDAR and depth views of the hero car",
        "desc_ja": "自車の前後 RGB・セグメンテーション・LiDAR・深度の映像",
    },
    "drone_cams": {
        "en": "Drone cameras", "ja": "ドローンカメラ",
        "desc_en": "CCTV wall with one camera per flying delivery drone",
        "desc_ja": "飛行中の配送ドローンごとのカメラを並べた CCTV 画面",
    },
}
ACTIONS = ("open", "close")


def _write_json(path, data):
    """Atomic write (temp file + rename): readers never see half a file.
    一時ファイル + rename で書き、読み手が途中の内容を見ないようにする。"""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def _read_json(path):
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def write_request(window, action, path=CONTROL_PATH, now=None):
    """Dashboard side: ask the pipeline to open or close ``window``. Merges
    into the file so a click on another window is not lost.
    ダッシュボード側：ウィンドウの開閉を依頼する（他ウィンドウの依頼は残す）。"""
    if window not in WINDOWS:
        raise ValueError("unknown window: %r" % (window,))
    if action not in ACTIONS:
        raise ValueError("unknown action: %r" % (action,))
    reqs = _read_json(path) or {}
    reqs = {k: v for k, v in reqs.items() if k in WINDOWS and isinstance(v, dict)}
    t = time.time() if now is None else now
    # strictly increasing per window, even for two clicks in one clock tick
    # 同じ時刻の 2 回のクリックでも単調増加にする
    try:
        t = max(t, float(reqs.get(window, {}).get("t", 0.0)) + 1e-3)
    except (TypeError, ValueError):
        pass
    reqs[window] = {"action": action, "t": t}
    _write_json(path, reqs)
    return reqs[window]


def read_requests(path=CONTROL_PATH):
    """All pending entries {window: (action, t)}; bad entries are skipped.
    有効な依頼の一覧（壊れた項目は無視）。"""
    out = {}
    for name, req in (_read_json(path) or {}).items():
        if name not in WINDOWS or not isinstance(req, dict):
            continue
        if req.get("action") not in ACTIONS:
            continue
        try:
            out[name] = (req["action"], float(req["t"]))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def clear(path):
    """Remove a side-channel file (and its temp file). / ファイルを消す。"""
    for p in (path, path + ".tmp"):
        try:
            os.remove(p)
        except OSError:
            pass


def _pid_alive(pid):
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except (PermissionError, TypeError, ValueError, OSError):
        return True
    return True


def read_status(path=STATUS_PATH, now=None, max_age=STATUS_MAX_AGE_S):
    """Dashboard side: the pipeline's status with ``live`` set (False if no
    pipeline wrote it recently or its process is gone). Every window of
    WINDOWS is present, with its labels.
    ダッシュボード側：パイプラインの状態（最近書かれていない・プロセスが無い
    場合は live=False）。全ウィンドウを表示名付きで含む。"""
    st = _read_json(path) or {}
    now = time.time() if now is None else now
    try:
        age = now - float(st["t"])
    except (KeyError, TypeError, ValueError):
        age = None
    live = (age is not None and age <= max_age and _pid_alive(st.get("pid"))
            and st.get("phase") != "stopped")
    wins = st.get("windows") if isinstance(st.get("windows"), dict) else {}
    out = []
    for name, lab in WINDOWS.items():
        w = wins.get(name) if isinstance(wins.get(name), dict) else {}
        row = dict(lab, name=name,
                   running=bool(live and w.get("running")),
                   pid=w.get("pid") if live and w.get("running") else None,
                   available=bool(live and w.get("available")),
                   reason=w.get("reason") or "",
                   wanted=bool(w.get("wanted")))
        out.append(row)
    return {"live": live, "phase": st.get("phase", "") if live else "",
            "map": st.get("map", ""), "age": None if age is None else round(age, 1),
            "windows": out}


class WindowSupervisor(object):
    """Pipeline side: which optional windows should be open, starting and
    stopping them, and the status file. Processes are (proc, name) entries of
    run_pipeline's ``children`` list; launching and stopping are passed in so
    the tests can fake them.

    パイプライン側：どの任意ウィンドウを開くべきか、その起動・停止、状態
    ファイル。プロセスは run_pipeline の ``children``（(proc, name) のリスト）
    の要素。起動・停止は引数で受け取る（試験で偽物に差し替えるため）。

    ``wanted``    {name: bool} -- open at launch (the command-line flags)
    ``cmds``      {name: cmd list} -- the same command line as at launch
    ``unavailable`` {name: reason} -- windows that cannot be opened in this run
    ``launch(cmd, name)`` -> proc ; ``stop([(proc, name)])`` stops cleanly
    """

    def __init__(self, wanted, cmds, launch, stop, unavailable=None,
                 control_path=CONTROL_PATH, status_path=STATUS_PATH, now=None):
        self.cmds = dict(cmds)
        self.unavailable = dict(unavailable or {})
        self.want = {n: bool(wanted.get(n)) and n not in self.unavailable
                     for n in WINDOWS}
        self.launch, self.stop = launch, stop
        self.control_path, self.status_path = control_path, status_path
        # requests older than this pipeline are left over from an earlier run
        # この実行より古い依頼は以前の実行の残骸
        start = time.time() if now is None else now
        self.seen = {n: start for n in WINDOWS}
        self.restart_left = {n: WINDOW_RESTARTS for n in WINDOWS}
        clear(control_path)

    # -- helpers / 補助 ----------------------------------------------------
    @staticmethod
    def find(children, name):
        """The live process of ``name`` in children, or None. / 生存中のもの"""
        for proc, n in children:
            if n == name and proc.poll() is None:
                return proc
        return None

    def can_open(self, name):
        return name in WINDOWS and name in self.cmds and name not in self.unavailable

    # -- start / open / close ------------------------------------------------
    def start_wanted(self, children):
        """Start every window that should be open and is not running: at
        startup and after each map switch (state, not launch flags).
        開くべきで動いていないウィンドウを起動する（起動時とマップ切替後）。"""
        # a new world gets fresh crash restarts, like the other helpers
        # 新しいワールドでは他の補助プロセス同様に再起動回数を戻す
        self.restart_left = {n: WINDOW_RESTARTS for n in WINDOWS}
        for name in WINDOWS:
            if self.want[name]:
                self.open(children, name, reason="restore")

    def open(self, children, name, reason="dashboard"):
        """Open ``name``; nothing happens if it is already running. Returns
        the action taken: "opened", "already-open" or "unavailable".
        開く（既に動いていれば何もしない）。"""
        if not self.can_open(name):
            return "unavailable"
        self.want[name] = True
        if self.find(children, name) is not None:
            return "already-open"
        # drop a dead entry left by a crash / クラッシュで残った死んだ項目を除く
        children[:] = [c for c in children if c[1] != name]
        if reason == "dashboard":
            print("[pipeline] opening %s (requested from the dashboard) / ダッシュ"
                  "ボードの依頼で %s を開く" % (name, name))
            self.restart_left[name] = WINDOW_RESTARTS
        children.append((self.launch(self.cmds[name], name), name))
        return "opened"

    def close(self, children, name):
        """Close ``name`` with a clean SIGTERM of that one process; it is not
        restarted and stays closed across map switches. Returns "closed" or
        "already-closed".
        そのプロセスだけを SIGTERM で閉じる（再起動せず、マップ切替後も閉じたまま）。"""
        if name not in WINDOWS:
            return "unavailable"
        self.want[name] = False
        live = [c for c in children if c[1] == name and c[0].poll() is None]
        children[:] = [c for c in children if c[1] != name]
        if not live:
            return "already-closed"
        print("[pipeline] closing %s (requested from the dashboard) / ダッシュ"
              "ボードの依頼で %s を閉じる" % (name, name))
        self.stop(live)
        return "closed"

    def poll_requests(self, children):
        """Carry out new dashboard requests. Returns [(name, result)].
        新しいダッシュボードの依頼を実行する。"""
        done = []
        for name, (action, t) in sorted(read_requests(self.control_path).items()):
            if t <= self.seen.get(name, 0.0):
                continue
            self.seen[name] = t
            res = (self.open(children, name) if action == "open"
                   else self.close(children, name))
            if res == "unavailable":
                print("[pipeline] cannot open %s: %s / %s は開けない"
                      % (name, self.unavailable.get(name, "not configured"), name))
            done.append((name, res))
        return done

    def handle_exit(self, children, name, rc):
        """A window process exited by itself. rc 0 means the user closed it
        (close button / Esc / Q): forget it, no restart, stays closed. Returns
        True if handled that way; False means a crash for the normal restart.
        ウィンドウが自分で終了した。rc 0 はユーザーが閉じた（閉じるボタン /
        Esc / Q）：再起動せず閉じたままにする。そう扱えば True、False は
        クラッシュ（通常の再起動へ）。"""
        if name not in WINDOWS or rc != 0:
            return False
        print("[pipeline] %s closed by the user -> not restarting (open it again "
              "from the dashboard) / %s はユーザーが閉じたため再起動しない"
              "（ダッシュボードから再度開ける）" % (name, name))
        self.want[name] = False
        children[:] = [c for c in children if c[1] != name]
        return True

    def take_restart(self, name):
        """One crash restart of ``name``, if any are left. / 再起動回数を 1 つ使う"""
        if self.restart_left.get(name, 0) <= 0:
            return False
        self.restart_left[name] -= 1
        return True

    # -- status / 状態 -------------------------------------------------------
    def status(self, children, phase="running", map_name=""):
        wins = {}
        for name in WINDOWS:
            proc = self.find(children, name)
            wins[name] = {"running": proc is not None,
                          "pid": proc.pid if proc is not None else None,
                          "wanted": self.want[name],
                          "available": self.can_open(name),
                          "reason": self.unavailable.get(name, "")}
        return {"t": time.time(), "pid": os.getpid(), "phase": phase,
                "map": map_name or "", "windows": wins}

    def write_status(self, children, phase="running", map_name=""):
        st = self.status(children, phase, map_name)
        try:
            _write_json(self.status_path, st)
        except OSError as exc:
            print("[pipeline] cannot write %s: %s" % (self.status_path, exc))
        return st
