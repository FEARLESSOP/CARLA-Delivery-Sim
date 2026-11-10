"""Experiment queue: works through research/queue.csv by itself.

Each row is one training run (brain, algorithm, seed, hours, tag). The runner
keeps at most --parallel runs going, starts the next pending row when one ends,
marks rows running / done / failed in the file, and logs each run to
research/queue_logs/<id>.log. Edit queue.csv while it runs (add, remove,
reorder pending rows); it re-reads the file every minute. Runs never touch the
main brains (each --tag run has its own folder under research/exp/).

  python3 -m rl_delivery.queue_runner --parallel 3            # until the queue is empty
  python3 -m rl_delivery.queue_runner --parallel 3 --watch    # keep waiting for new rows
  systemctl --user start rl-queue                              # as a service (survives the agent)

queue.csv columns: id, tag, brain, algo, seed, hours, envs, args (extra
trainer flags), status, started, ended, note. status: pending (or empty) / running / done / failed / skip.

実験キュー：queue.csv を上から順に、同時 N 本まで自動で学習する。
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
RESEARCH = os.path.join(HERE, "research")
QUEUE = os.path.join(RESEARCH, "queue.csv")
STATE = os.path.join(RESEARCH, "queue_state.json")
LOGS = os.path.join(RESEARCH, "queue_logs")
PY = os.path.join(os.path.dirname(os.path.dirname(HIL)), "carla-venv", "bin", "python")
COLS = ["id", "tag", "brain", "algo", "seed", "hours", "envs", "args", "status", "started", "ended", "note"]


def read():
    if not os.path.exists(QUEUE):
        return []
    with open(QUEUE, newline="") as f:
        return [dict({c: "" for c in COLS}, **{k: (v or "") for k, v in r.items() if k in COLS})
                for r in csv.DictReader(f)]


def write(rows):
    tmp = QUEUE + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, COLS)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, QUEUE)


def alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:                                   # a finished child is a zombie until reaped / ゾンビ判定
        with open("/proc/%d/stat" % pid) as f:
            return f.read().split()[2] != "Z"
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parallel", type=int, default=3)
    ap.add_argument("--watch", action="store_true", help="keep running when the queue is empty")
    a = ap.parse_args()
    os.makedirs(LOGS, exist_ok=True)
    running = {}                          # id -> Popen / 実行中
    if os.path.exists(STATE):
        try:
            for rid, pid in json.load(open(STATE)).get("running", {}).items():
                if alive(pid):
                    running[rid] = pid            # left by an earlier runner: keep watching / 前回の続き
        except (OSError, ValueError):
            pass
    print("[queue] runner started, parallel %d" % a.parallel, flush=True)
    while True:
        rows = read()
        by_id = {r["id"]: r for r in rows}
        # finished runs / 終わった実行
        for rid in list(running):
            proc = running[rid]
            done = proc.poll() is not None if hasattr(proc, "poll") else not alive(proc)
            if done:
                rc = proc.returncode if hasattr(proc, "poll") else 0
                if rid in by_id:
                    by_id[rid]["status"] = "done" if rc == 0 else "failed"
                    by_id[rid]["ended"] = time.strftime("%Y-%m-%d %H:%M")
                print("[queue] %s %s" % (rid, "done" if rc == 0 else "failed (rc %s)" % rc), flush=True)
                del running[rid]
        # start pending runs / 次の実行を開始
        for r in rows:
            if len(running) >= a.parallel:
                break
            if r["status"] not in ("", "pending") or not r["id"] or not r["brain"]:
                continue
            cmd = [PY, "-W", "ignore", "-m", "rl_delivery.train", "--brain", r["brain"],
                   "--tag", r["tag"] or "untagged", "--seed", r["seed"] or "0",
                   "--hours", r["hours"] or "3", "--envs", r["envs"] or "6"]
            if r["algo"]:
                cmd += ["--algo", r["algo"]]
            cmd += (r["args"] or "").split()           # extra trainer flags, e.g. --eval-every 50000
            log = open(os.path.join(LOGS, "%s.log" % r["id"]), "a")
            env = dict(os.environ, OMP_NUM_THREADS="1")
            running[r["id"]] = subprocess.Popen(cmd, cwd=HIL, stdout=log, stderr=subprocess.STDOUT, env=env)
            r["status"], r["started"] = "running", time.strftime("%Y-%m-%d %H:%M")
            print("[queue] started %s: %s" % (r["id"], " ".join(cmd[4:])), flush=True)
        write(rows)
        with open(STATE, "w") as f:
            json.dump({"running": {k: (v.pid if hasattr(v, "pid") else v) for k, v in running.items()},
                       "t": time.time()}, f)
        pending = [r for r in rows if r["status"] in ("", "pending")]
        if not running and not pending and not a.watch:
            print("[queue] queue empty, runner stops", flush=True)
            return
        if os.path.exists(os.path.join(RESEARCH, "STOP_queue")):
            print("[queue] STOP_queue found: no new runs; waiting for the running ones", flush=True)
            if not running:
                os.remove(os.path.join(RESEARCH, "STOP_queue"))
                return
            for r in rows:
                if r["status"] in ("", "pending"):
                    r["status"] = "pending"
            a.parallel = 0
        time.sleep(float(os.environ.get("RL_QUEUE_POLL", "60")))


if __name__ == "__main__":
    sys.exit(main())
