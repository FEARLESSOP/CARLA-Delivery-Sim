"""Machine-health snapshot and preflight for RL runs on this rig.

Why this exists / 目的: on 2026-09-14 a training run (`rl.run_arm`)
took the whole workstation down five minutes in -- a *hard lockup* on
CPU 0 inside the NVIDIA modeset kernel thread (`nvkms_kthread_q_callback`
spinning in `native_queued_spin_lock_slowpath`), after which every other
CPU soft-locked waiting on its TLB-flush IPIs. Nothing in user space can
prevent a kernel-driver lockup, but a run CAN

  1. refuse to start on a machine that is already showing GPU faults
     (`NRVM: Xid`) or lockups in this boot,
  2. record what the machine looked like when the run began, in a file
     that is flushed immediately, so the post-mortem after a power cycle
     is not guesswork, and
  3. keep its own load low (torch threads, nice, no rendering) so it is
     not the straw that breaks the driver.

This module does 1 and 2; rl/run_arm.py does 3. Everything here is
best-effort: a missing `journalctl`/`nvidia-smi` degrades to "unknown",
never to an exception.

学習 run が PC 全体を固まらせた（NVIDIA modeset カーネルスレッド内の
ハードロックアップ）ことへの対策。ユーザ空間からカーネルドライバの
ロックアップは防げないが、(1) すでに GPU 障害やロックアップが出ている
ブートでは開始を拒否し、(2) 開始時点の機器状態を即時 flush されるファイル
に残して電源断後の原因調査を可能にし、(3) 自身の負荷を下げる
（run_arm.py 側）ことはできる。どのコマンドが無くても例外にはならない。
"""

import json
import os
import re
import subprocess
import time

# Kernel-log patterns that mean "this boot is already hurt".
# 「このブートは既に不健全」を示すカーネルログのパターン。
_BAD_PATTERNS = (
    ('xid', r'NVRM: Xid'),
    ('soft_lockup', r'soft lockup'),
    ('hard_lockup', r'hard LOCKUP'),
    ('oops', r'\bOops\b|BUG: unable to handle|general protection fault'),
    ('mce', r'\bmce: |Machine check'),
)


def _run(cmd, timeout=10):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout)
        return out.stdout if out.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def kernel_faults(boot=0):
    """Count fault lines in the kernel log of `boot` (0 = this boot,
    -1 = previous). / 指定ブートのカーネルログ中の障害行数。"""
    log = _run(['journalctl', '-k', '-b', str(boot), '--no-pager',
                '-o', 'short-iso'], timeout=20)
    counts = {}
    last = {}
    for key, pat in _BAD_PATTERNS:
        rx = re.compile(pat)
        hits = [ln for ln in log.splitlines() if rx.search(ln)]
        counts[key] = len(hits)
        if hits:
            last[key] = hits[-1][:200]
    return {'counts': counts, 'last_lines': last,
            'available': bool(log)}


def gpu_state():
    out = _run(['nvidia-smi', '--query-gpu=name,driver_version,'
                'memory.used,memory.total,temperature.gpu,utilization.gpu',
                '--format=csv,noheader,nounits'])
    if not out.strip():
        return {'available': False}
    f = [x.strip() for x in out.strip().splitlines()[0].split(',')]
    keys = ('name', 'driver', 'mem_used_mib', 'mem_total_mib', 'temp_c',
            'util_pct')
    return dict(zip(keys, f), available=True)


def uptime_s():
    try:
        with open('/proc/uptime') as fh:
            return float(fh.read().split()[0])
    except (OSError, ValueError):
        return -1.0


def snapshot():
    """Everything worth knowing about the box right now, JSON-able.
    / 今この瞬間の機器状態（JSON 化可能）。"""
    try:
        with open('/proc/loadavg') as fh:
            load = fh.read().split()[:3]
    except OSError:
        load = []
    mem = {}
    try:
        with open('/proc/meminfo') as fh:
            for ln in fh:
                k, v = ln.split(':', 1)
                if k in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
                    mem[k] = int(v.split()[0]) // 1024      # MiB
    except (OSError, ValueError):
        pass
    boot_id = ''
    try:
        with open('/proc/sys/kernel/random/boot_id') as fh:
            boot_id = fh.read().strip()
    except OSError:
        pass
    return {
        'wall_time': time.time(),
        'iso_time': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'boot_id': boot_id,
        'uptime_s': round(uptime_s(), 1),
        'loadavg': load,
        'mem_mib': mem,
        'cpu_count': os.cpu_count(),
        'gpu': gpu_state(),
        'kernel_this_boot': kernel_faults(0),
        'kernel_prev_boot': kernel_faults(-1),
    }


def preflight(min_uptime_s=120.0, allow_faults=False):
    """Return (ok, warnings, snapshot). `ok` is False when this boot has
    already logged GPU faults or lockups, unless `allow_faults`.

    (ok, 警告一覧, スナップショット) を返す。このブートに既に GPU 障害や
    ロックアップが記録されていれば、allow_faults でない限り ok=False。
    """
    snap = snapshot()
    warns = []
    ok = True
    this = snap['kernel_this_boot']
    prev = snap['kernel_prev_boot']
    bad_now = {k: v for k, v in this['counts'].items() if v}
    if bad_now:
        msg = ('kernel already logged %s in THIS boot -- the GPU/driver '
               'is in a degraded state; reboot before training '
               '/ このブートで既に %s が記録されている。学習前に再起動'
               % (bad_now, bad_now))
        warns.append(msg)
        ok = allow_faults
    bad_prev = {k: v for k, v in prev['counts'].items()
                if v and k in ('hard_lockup', 'soft_lockup', 'oops')}
    if bad_prev:
        warns.append('the PREVIOUS boot ended with %s -- if that was an RL '
                     'run, keep this one short and watch `journalctl -kf` '
                     '/ 前回のブートは %s で終わった。この run は短くし '
                     'journalctl -kf を監視すること' % (bad_prev, bad_prev))
    if 0 <= snap['uptime_s'] < min_uptime_s:
        warns.append('machine booted %.0f s ago; letting the desktop settle '
                     'first is cheap / 起動から %.0f 秒。少し待つのが安全'
                     % (snap['uptime_s'], snap['uptime_s']))
    if not this['available']:
        warns.append('journalctl unavailable: cannot check kernel log '
                     '/ journalctl が使えずカーネルログを確認できない')
    g = snap['gpu']
    if g.get('available') and g.get('mem_used_mib', '0').isdigit() \
            and int(g['mem_used_mib']) > 12000:
        warns.append('GPU memory already %s MiB used -- another CARLA or '
                     'a leaked one? / GPU メモリが既に %s MiB 使用中'
                     % (g['mem_used_mib'], g['mem_used_mib']))
    return ok, warns, snap


def write_snapshot(path, extra=None):
    """Write snapshot() (+extra) to `path`, fsync'd so it survives a
    freeze. / snapshot() を fsync 付きで書く（フリーズしても残る）。"""
    snap = snapshot()
    if extra:
        snap.update(extra)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as fh:
        json.dump(snap, fh, indent=1)
        fh.flush()
        os.fsync(fh.fileno())
    return snap


if __name__ == '__main__':
    ok, warns, snap = preflight()
    print(json.dumps(snap, indent=1))
    for w in warns:
        print('WARNING:', w)
    print('preflight:', 'OK' if ok else 'BLOCKED')
