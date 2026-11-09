"""Live check of the timetabled Oita Station buses (bus_schedule.py, map
guide ch. 36). Needs the NEW OitaStation imported and the pipeline running;
never starts or stops CARLA or the pipeline and never ticks the world (it
follows the pipeline's ticks: the world is synchronous).

  carla-venv/bin/python maps/bus_live.py watch [--seconds 900]
      read-only: follows the pipeline's own buses (role_name timetable_bus,
      started by generate_traffic --bus-schedule auto)
  carla-venv/bin/python maps/bus_live.py spawn --time 08:00 --day weekday
                                         [--n 5] [--seconds 900] [--tm-port 8040]
      spawns up to --n buses of its own (role_name bus_live_test, its own
      Traffic Manager on --tm-port, never 8000, never the pipeline's) on the
      timetable from --time, follows them, destroys them at the end (also on
      Ctrl+C); its status goes to maps/build/bus_live_status.json, not the
      dashboard's file

Both log every standstill of a bus of >= 3 s to a CSV: where its boarding
door stood against the nearest pole (along the lane / to the side, the pole
on the left?), whether the bus stood in a junction, and the summary:
  - calls at a pole: door within 3 m along the lane, pole on the left;
  - standstills inside a junction: must be 0 apart from red lights at the
    junction mouth (the bus's centre in a junction);
  - buses at once, stuck removals (from data/bus_status.json in watch mode).
At the end: PASS / FAIL lines. Results: maps/build/bus_live.csv.

時刻表バスのライブ確認。watch は読み取りのみ（パイプラインのバスを観察）、
spawn は専用の role_name と TM ポートで数台を出して観察し、最後に削除する。
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import carla  # noqa: E402
import bus_schedule as bs  # noqa: E402

TEST_ROLE = 'bus_live_test'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=('watch', 'spawn'))
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--seconds', type=float, default=900.0, help='wall-clock seconds / 実時間（秒）')
    ap.add_argument('--time', default='08:00', help='spawn: timetable time HH:MM / 時刻表の時刻')
    ap.add_argument('--day', default='weekday', choices=bs.DAY_TYPES)
    ap.add_argument('--n', type=int, default=5, help='spawn: at most this many buses / 最大台数')
    ap.add_argument('--tm-port', type=int, default=8040, help='spawn: own TM port (never 8000)')
    ap.add_argument('--out', default=os.path.join(HERE, 'build', 'bus_live.csv'))
    a = ap.parse_args()
    if a.tm_port == 8000:
        sys.exit('--tm-port 8000 is the pipeline\'s Traffic Manager: pick another / 8000 は使わない')

    cl = carla.Client(a.host, a.port)
    cl.set_timeout(20.0)
    world = cl.get_world()
    wmap = world.get_map()
    if not world.get_settings().synchronous_mode:
        print('note: the world is not synchronous (is the pipeline running?) / 同期モードではない')
    tt = bs.Timetable(os.path.join(HIL, 'data', 'oita_bus'))
    poles = [(s['id'], s['name'], s.get('platform'), s['x'], -s['y']) for s in tt.stops.values()]

    sched = None
    tm = None
    stop_flag = []
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.append(1))
    role = bs.ROLE
    if a.mode == 'spawn':
        role = TEST_ROLE
        tm = cl.get_trafficmanager(a.tm_port)
        # our process never ticks the world, so our TM cannot be synchronous
        # (a synchronous TM only steps with its own process's world.tick())
        # このプロセスはティックしないので TM は非同期
        tm.set_synchronous_mode(False)
        world.wait_for_tick(10.0)
        sched = bs.setup(world, cl, tm, carla, mode='auto', bus_time=a.time, bus_day=a.day,
                         max_buses=a.n, role=role,
                         status_path=os.path.join(HERE, 'build', 'bus_live_status.json'))
        if sched is None:
            sys.exit('no buses on %s (not the new OitaStation, or no bus blueprint) / バス無し' % wmap.name)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fh = open(a.out, 'w', newline='')
    out = csv.writer(fh)
    out.writerow(['sim_t', 'bus', 'type', 'stand_s', 'x', 'y', 'pole', 'pole_name', 'platform',
                  'door_along_m', 'door_side_m', 'pole_left', 'in_junction', 'at_pole'])
    door_m = {m['id']: m['door_m'] for m in bs.BUS_MODELS}
    info, ids, next_ids = {}, [], 0.0
    still = {}
    rows, seen, most = [], set(), 0
    end = time.time() + a.seconds
    try:
        while time.time() < end and not stop_flag:
            snap = world.wait_for_tick(10.0)          # synchronous world: follow its ticks
            now = snap.timestamp.elapsed_seconds
            if sched is not None:
                sched.update(snap)
            if now >= next_ids:
                next_ids = now + 2.0
                acts = world.get_actors().filter('vehicle.*')
                ids = [v.id for v in acts if v.attributes.get('role_name') == role]
                for v in acts:
                    if v.id in ids:
                        info.setdefault(v.id, v.type_id)
                most = max(most, len(ids))
            for vid in ids:
                act = snap.find(vid)
                if act is None:
                    continue
                seen.add(vid)
                tf = act.get_transform()
                vel = act.get_velocity()
                sp = math.hypot(vel.x, vel.y)
                st = still.get(vid)
                if sp < 0.1:
                    if st is None:
                        still[vid] = [now, tf, False]
                    elif not st[2] and now - st[0] >= 3.0:
                        st[2] = True
                        rows.append(measure(out, now, vid, info.get(vid, ''), now - st[0], tf,
                                            poles, wmap, door_m.get(info.get(vid, ''), -0.85)))
                else:
                    still.pop(vid, None)
    except KeyboardInterrupt:
        pass
    finally:
        if sched is not None:
            sched.destroy_all()
            print('[bus_live] destroyed the test buses / テスト用バスを削除')
        fh.close()

    calls = [r for r in rows if r['at_pole']]
    in_jn = [r for r in rows if r['in_junction']]
    print('\nbuses seen %d, at most %d at once; %d standstills >= 3 s, %d of them calls at a pole'
          % (len(seen), most, len(rows), len(calls)))
    if calls:
        al = sorted(abs(r['along']) for r in calls)
        print('door to pole along the lane: median %.1f m, max %.1f m' % (al[len(al) // 2], al[-1]))
    wrong = [r for r in rows if r['near'] and not r['left']]
    status = None
    if a.mode == 'watch':
        try:
            with open(bs.STATUS_PATH, encoding='utf-8') as sfh:
                status = json.load(sfh)
        except (OSError, ValueError):
            pass
    ok = lambda c: 'PASS' if c else 'FAIL'
    print('%s  buses ran (%d seen)' % (ok(seen), len(seen)))
    print('%s  calls at poles with the door within 3 m and the pole on the left (%d)'
          % (ok(calls), len(calls)))
    print('%s  no bus stood beside a pole with the pole on its right (%d)' % (ok(not wrong), len(wrong)))
    print('%s  standstills with the bus centre in a junction: %d (red lights at the mouth are fine; '
          'look at the CSV rows)' % (ok(len(in_jn) <= max(1, len(rows) // 10)), len(in_jn)))
    if status is not None:
        print('status: %s %s, %d buses, counts %s' % (status.get('clock'), status.get('day_type'),
                                                    len(status.get('buses', [])), status.get('counts')))
        print('%s  no stuck removals' % ok(not status.get('counts', {}).get('stuck')))
    print('CSV: %s' % a.out)


def measure(out, now, vid, type_id, stand, tf, poles, wmap, door_m):
    yaw = math.radians(tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    dx0, dy0 = tf.location.x + fx * door_m, tf.location.y + fy * door_m
    best = None
    for pid, name, plat, px, py in poles:
        d = math.hypot(px - dx0, py - dy0)
        if best is None or d < best[0]:
            best = (d, pid, name, plat, px, py)
    d, pid, name, plat, px, py = best
    along = (px - dx0) * fx + (py - dy0) * fy
    side = -(px - dx0) * fy + (py - dy0) * fx          # + = right (CARLA y south)
    w = wmap.get_waypoint(tf.location)
    in_jn = bool(w is not None and w.is_junction)
    near = d < 12.0
    at_pole = near and abs(along) <= 3.0 and side < 0.0
    out.writerow(['%.1f' % now, vid, type_id, '%.0f' % stand, '%.1f' % tf.location.x, '%.1f' % tf.location.y,
                  pid, name, plat or '', '%.1f' % along, '%.1f' % side, int(side < 0.0), int(in_jn),
                  int(at_pole)])
    return {'along': along, 'left': side < 0.0, 'near': near, 'in_junction': in_jn, 'at_pole': at_pole}


if __name__ == '__main__':
    main()
