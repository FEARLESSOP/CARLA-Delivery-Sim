"""The launcher: `python3 run_pipeline.py` with no arguments.

Starts only the dashboard (http://localhost:5000). The Launch page there picks
the map, traffic, pods, your car and every other option; Launch starts the
pipeline (run_pipeline.py with those flags) as a child, and the launcher stays
up when it ends, so the dashboard returns to the Launch page. Stop, Relaunch
(new options that need a restart) come from the dashboard too
(pipeline_control.py: data/launch_request.json). Ctrl+C here stops everything.

The pipeline's output goes to data/pipeline.log (the dashboard shows its tail);
the last options are kept in launch_options.json.

ランチャー（引数なしの run_pipeline.py）: ダッシュボードだけを起動し、そこで
設定して起動・停止・再起動する。
"""

import os
import signal
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

import launch_options
import pipeline_control as pc

BASE_DIR = Path(__file__).resolve().parent
VENV_PY = BASE_DIR.parent.parent / "carla-venv" / "bin" / "python"
DASH_PORT = 5000


def log(msg):
    print("[launcher] %s" % msg, flush=True)


class Launcher(object):
    def __init__(self, port=DASH_PORT):
        self.port = port
        self.dash = None
        self.child = None
        self.options = launch_options.load_saved()
        self.phase = "idle"
        self.started = None
        self.last_exit = None
        self.relaunch = None             # options to start once the child is gone / 終了後に起動する設定
        self.stop_deadline = None
        self.reader = pc.Reader(pc.LAUNCH_PATH, pc.LAUNCH_ACTIONS)
        self.script = BASE_DIR / "run_pipeline.py"     # a stand-in in tests / テストでは代用

    # -- dashboard / ダッシュボード
    def start_dashboard(self):
        cmd = [str(VENV_PY), "dashboard.py", "--dashboard-port", str(self.port),
               "--log-dir", str(BASE_DIR / "data")]
        self.dash = subprocess.Popen(cmd, cwd=str(BASE_DIR), start_new_session=True)
        log("dashboard at http://localhost:%d/ (pid %d)" % (self.port, self.dash.pid))

    # -- the pipeline / パイプライン
    def launch(self, opts):
        self.options = launch_options.clean(opts)
        launch_options.save(self.options)
        argv = launch_options.to_argv(self.options)
        cmd = [sys.executable, "-u", str(self.script)] + argv + [
            "--external-dashboard", "--dashboard-port", str(self.port)]
        os.makedirs(os.path.dirname(pc.LOG_PATH), exist_ok=True)
        out = open(pc.LOG_PATH, "w")
        out.write("$ run_pipeline.py %s\n" % " ".join(argv))
        out.flush()
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self.child = subprocess.Popen(cmd, cwd=str(BASE_DIR), stdout=out, stderr=subprocess.STDOUT,
                                      stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        self.phase, self.started = "starting", time.time()
        log("launched the pipeline (pid %d): %s" % (self.child.pid, " ".join(argv)))

    def stop(self):
        if self.child is not None and self.child.poll() is None and self.phase != "stopping":
            log("stopping the pipeline (pid %d)" % self.child.pid)
            os.killpg(self.child.pid, signal.SIGINT)
            self.phase, self.stop_deadline = "stopping", time.time() + 60.0

    def write_state(self):
        st = pc.read_json(pc.STATE_PATH) or {}
        if self.phase == "starting" and st.get("phase") == "running" and st.get("pid") == \
                (self.child.pid if self.child else None):
            self.phase = "running"
        pc.write_json(pc.LAUNCHER_STATE_PATH, {
            "phase": self.phase, "options": self.options, "pid": self.child.pid if self.child else None,
            "started": self.started, "last_exit": self.last_exit, "t": time.time(),
            "dashboard_port": self.port})

    def step(self):
        req = self.reader.take()
        if req is not None:
            a = req["action"]
            busy = self.child is not None and self.child.poll() is None
            if a == "launch" and not busy:
                self.launch(req.get("options") or self.options)
            elif a == "stop":
                self.stop()
            elif a == "relaunch":
                self.relaunch = req.get("options") or self.options
                if busy:
                    self.stop()
            elif a == "launch" and busy:
                log("already running: ignored a launch request")
        if self.child is not None:
            rc = self.child.poll()
            if rc is None and self.phase == "stopping" and time.time() > self.stop_deadline:
                log("the pipeline did not stop in 60 s -> SIGKILL")
                os.killpg(self.child.pid, signal.SIGKILL)
            elif rc is not None:
                log("the pipeline exited (rc=%s)" % rc)
                self.last_exit = {"rc": rc, "t": time.time()}
                self.child, self.phase = None, "idle"
        if self.child is None and self.relaunch is not None:
            opts, self.relaunch = self.relaunch, None
            self.launch(opts)
        if self.dash is not None and self.dash.poll() is not None:
            log("the dashboard exited (rc=%s) -> restarting it" % self.dash.returncode)
            time.sleep(1.0)
            self.start_dashboard()
        self.write_state()

    def shutdown(self):
        self.stop()
        deadline = time.time() + 60.0
        while self.child is not None and self.child.poll() is None and time.time() < deadline:
            time.sleep(0.5)
        if self.child is not None and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGKILL)
        if self.dash is not None and self.dash.poll() is None:
            os.killpg(self.dash.pid, signal.SIGTERM)
        self.phase = "idle"
        self.child = None
        pc.write_json(pc.LAUNCHER_STATE_PATH, {"phase": "off", "t": time.time()})


def main():
    if not VENV_PY.exists():
        log("ERROR: venv python not found at %s" % VENV_PY)
        return 1
    L = Launcher()
    stop = {"now": False}

    def on_signal(signum, _frame):
        stop["now"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    L.start_dashboard()
    url = "http://localhost:%d/" % L.port
    log("pick the map, traffic and pods on the Launch page: %s  (Ctrl+C quits)" % url)
    try:
        time.sleep(2.0)
        if os.environ.get("DISPLAY"):
            webbrowser.open(url)
    except Exception:
        pass
    while not stop["now"]:
        L.step()
        time.sleep(0.5)
    log("shutting down")
    L.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
