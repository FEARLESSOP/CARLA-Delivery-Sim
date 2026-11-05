"""Japanese driving rules for the delivery vans (Japan profile maps only).

The vans are not Traffic Manager cars: each is driven by its own BasicAgent
(delivery_sim.Van, ``agent.run_step()`` -> ``apply_control``), so the
RuleKeeper in generate_traffic.py never sees them (live: 29 km/h through
止まれ). Here the same `traffic_profile.RuleKeeper` -- the same 止まれ
StopLines, walker yield and lane speeds, same JSON profile -- runs for the
vans, with `SpeedGate` standing in for the Traffic Manager. The RuleKeeper
"tells the TM" a desired speed or a stop; the gate only records it, and the
van applies it to its own control while it is DRIVING:

* stop (``vehicle_percentage_speed_difference 100``): throttle 0, full brake
  (no hand brake, so the van's stuck guard stays quiet), steering kept;
* desired speed (``set_desired_speed``): the BasicAgent's target becomes
  min(the van's own --speed, that speed) -- never faster than the van would
  go anyway, so the delivery timing only gets slower where the law says so.

Only DRIVING vans are handed to the RuleKeeper. A van that stops for a
delivery, a charger or the yard is dropped from it (its 止まれ / walker state
and the gate are cleared), so the stop rule never releases a delivery stop
and a delivery stop never counts as -- or keeps -- a 止まれ hold. When the van
drives on it starts fresh: a 止まれ ahead is stopped at again.
Lane changes: BasicAgent does not change lanes on its own; ignored.

Other maps: select_profile() gives None and delivery_sim never builds this
(zero extra calls, the vans drive exactly as before).

配送車用の日本の交通ルール。配送車は TM ではなく BasicAgent で走るため、
同じ RuleKeeper を TM の代わりの SpeedGate と組み合わせて配送車にも適用する。
走行中の車だけを対象にし、配送停車中は状態を消す（配送停車と止まれが干渉しない）。
"""

import traffic_profile


class SpeedGate(object):
    """Duck-typed Traffic Manager for RuleKeeper: records, per vehicle id,
    a stop or a desired speed (km/h); never talks to the server.
    / RuleKeeper 用の TM の代役。停止・目標速度を記録するだけ。"""

    def __init__(self):
        self.stop = {}           # id -> True while held / 停止中
        self.kmh = {}            # id -> desired km/h / 目標速度

    @staticmethod
    def _id(actor):
        return getattr(actor, 'id', actor)

    def set_desired_speed(self, actor, kmh):
        vid = self._id(actor)
        self.kmh[vid] = float(kmh)
        self.stop.pop(vid, None)

    def vehicle_percentage_speed_difference(self, actor, pct):
        if float(pct) >= 100.0:
            self.stop[self._id(actor)] = True

    def auto_lane_change(self, actor, allow):
        pass                     # BasicAgent keeps its lane / 車線変更しない

    def clear(self, vid):
        self.stop.pop(vid, None)
        self.kmh.pop(vid, None)


class VanRules(object):
    """RuleKeeper + SpeedGate for the delivery vans. `update()` once per tick
    (before the vans step), `apply()` on each DRIVING van's control.
    / 配送車用の RuleKeeper。毎 tick update、走行中の制御に apply。"""

    def __init__(self, world, wmap, profile, lane_type=None, xodr=None):
        self.gate = SpeedGate()
        self.rk = traffic_profile.RuleKeeper(world, wmap, self.gate, [], profile,
                                             driving_lane_type=lane_type, xodr=xodr)
        self.driving = set()
        self.sent = {}           # id -> km/h last given to the BasicAgent

    def _forget(self, vid):
        rk = self.rk
        for d in (rk.stop_state, rk.held, rk.target, rk.cooldown):
            d.pop(vid, None)
        self.gate.clear(vid)

    def update(self, snapshot, driving_ids):
        """``driving_ids``: car ids of the vans in the DRIVING state."""
        now = set(driving_ids)
        for vid in self.driving - now:
            self._forget(vid)                  # parked for work: not ours / 停車作業中
        self.driving = now
        self.rk.ids = list(now)
        self.rk.update(snapshot)

    def stopped(self, vid):
        return bool(self.gate.stop.get(vid))

    def apply(self, vid, agent, base_kmh, control):
        """Bring the gate's speed or stop into the van's own control.
        Returns the control to apply. / ゲートの速度・停止を制御に反映。"""
        kmh = self.gate.kmh.get(vid)
        want = base_kmh if kmh is None else min(base_kmh, kmh)
        if self.sent.get(vid) != want:
            self.sent[vid] = want
            agent.set_target_speed(want)       # takes effect on the next step / 次の step から
        if self.stopped(vid):
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
        return control

    def summary(self):
        c = self.rk.stop_count
        return {'stop_line_stops': c['stops'], 'stop_line_timeouts': c['timeouts'],
                'stop_line_missed': c['missed'], 'walker_holds': self.rk.holds}


def make(world, wmap, force='auto', lane_type=None):
    """VanRules for this map's country profile, or None (every other map:
    no call at all). / このマップのプロファイル用 VanRules、無ければ None。"""
    name, profile = traffic_profile.select_profile(wmap.name, force=force)
    if profile is None:
        return None
    rules = VanRules(world, wmap, profile, lane_type=lane_type)
    print('  delivery vans: %s rules -- 止まれ full stop %s, yield to walkers %s, '
          'lane speed cap / 配送車にも %s の交通ルール'
          % (name, 'on' if rules.rk.stop_lines is not None else 'off',
             'on' if rules.rk.yield_cfg else 'off', name), flush=True)
    return rules
