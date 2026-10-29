"""What a LastMile pod does on the road and at its stops (mixed into delivery_sim.Van).

A LastMile pod carries 4 drones on its deck (the launch lift and bays B1..B4,
one bay free for the crane) and a battery magazine. Its subsystems run on
their own tracks (lmp_rig.PodAnim) with the HTML's keyframes, so they work
side by side:

  - Getting a drone ready happens whenever the cargo system is free, also
    while driving: the crane puts the fullest drone that can make the trip on
    the lift, the shuttle brings the parcel and the drone winches it up (the
    parcel is in the drone the moment the hook animation ends), and the
    swapper gives it the fullest pack.
  - Drone stop: every loaded drone launches in turn (roof open, lift up,
    launch, lift down, the next one craned up and loaded...). The pod does
    not wait for them: each drone delivers and then flies to a meeting point
    -- the pod's next stop if that is a face-to-face stop it can reach, else
    this stop (the pod then waits here).
  - Face-to-face stop: the box station and side port (as the HTML).
  - Meeting point: after the stop's own work the pod opens its roof, raises
    the lift and lands the drones circling overhead one at a time; each is
    lowered and craned onto a bay. Then the pod drives on and gets the next
    drone ready on the way.

Drones charge on their bays and packs in the magazine, both from the pod's
battery. The city station is the backup: standbys, and drones that run low
while waiting at a meeting point.

LastMile ポッドの走行中・停車地での作業（delivery_sim.Van に組み込む）。機構ごとに
独立したトラックで動かし、走行中も次の機体を準備する。複数機を順に発進させて待たずに
次へ向かい、合流地点で回収する。
"""

from delivery import lmp_rig as R
from delivery.backends import IN_VAN, FLYING
from delivery.planning import DRONE, FACE_TO_FACE

LMP_DRONE, LMP_DIRECT = "lmp_drone", "lmp_direct"
FREE_ORDER = (3, 4, 0, 2)       # where an arriving drone parks / 到着機の置き場


class LmpVanMixin(object):
    """Methods for a Van whose ``pod`` is a lmp_pod.LastMilePod."""

    def lmp_init(self):
        self.lmp = True
        self.lmp_units = [None] * 4          # drone index -> DroneUnit / 機体
        self.slots = [0, R.LAUNCH, 3, 4]     # drone index -> deck slot (None: out) / 区画
        self.packs = [100.0, None, 100.0, 100.0]   # magazine / 電池マガジン
        self.anim = R.PodAnim()
        self._loading = None                 # (unit, order) being loaded / 積み込み中
        self.loaded = None                   # (unit, order) ready on the lift / 発進準備完了
        self._crane = None                   # the crane's moves while it runs
        self._tray_out = False               # the last tray waits on the presenter
        self.launched = set()                # ids of orders launched / 発進済みの注文 id
        self.meets = {}                      # id(stop) -> {"stop", "units"} / 合流地点
        self._job = None
        self._landing = None                 # the drone cleared to land / 着陸許可機
        self._stow = False                   # a landed drone is on the lift / 着陸機をしまう
        self.swaps_in_pod = 0
        self.in_air = {}                     # unit -> its order / 飛行中の機体と注文

    def lmp_reset(self):
        """Everything stowed (towing): tracks at rest, nothing loading."""
        self.anim = R.PodAnim()
        self._loading = self.loaded = self._crane = self._job = self._landing = None
        self._tray_out = self._stow = False

    # -- where drones sit / 機体の位置
    def dock_for(self, unit):
        if getattr(self, "lmp", False) and unit in self.lmp_units:
            slot = self.slots[self.lmp_units.index(unit)]
            if slot is not None:
                return self.pod.slot_location(slot, self.pod.mech.lift)
        return self.dock

    def unit_pos(self, unit):
        return self._world(self.dock_for(unit))

    def accept_drone(self, unit):
        """A drone joins this van (start, or a standby arriving). / 機体を受け入れる。"""
        if not getattr(self, "lmp", False):
            unit.dock_into(self)
            self.drone = unit
            return True
        if None not in self.lmp_units:
            return False
        i = self.lmp_units.index(None)
        taken = {s for u, s in zip(self.lmp_units, self.slots) if u is not None}
        slot = R.LAUNCH if R.LAUNCH not in taken and self.anim.lift_down() else \
            next((s for s in FREE_ORDER if s not in taken), None)
        if slot is None:
            return False
        self.lmp_units[i], self.slots[i] = unit, slot
        unit.dock_into(self)
        if slot == R.LAUNCH:
            self.drone = unit
        return True

    def release_drone(self, unit):
        """``unit`` leaves for good (to charge, crashed). / 機体が離れる。"""
        if self.drone is unit:
            self.drone = None
        if getattr(self, "lmp", False) and unit in self.lmp_units:
            i = self.lmp_units.index(unit)
            self.lmp_units[i], self.slots[i] = None, None
            if self._landing is unit:
                self._landing = None
            if self.loaded and self.loaded[0] is unit:
                self.loaded = None
        if getattr(self, "lmp", False):
            self.in_air.pop(unit, None)

    def rig_slots(self):
        return [s if u is not None and u.status in IN_VAN else None
                for u, s in zip(self.lmp_units, self.slots)]

    def docked_units(self):
        return [(i, u) for i, u in enumerate(self.lmp_units)
                if u is not None and u.status in IN_VAN and self.slots[i] is not None]

    def free_bay(self, also_free=None):
        taken = {s for u, s in zip(self.lmp_units, self.slots) if u is not None and s is not None}
        return next((s for s in (R.BUFFER, 0, 3, 4) if s not in taken or s == also_free), R.BUFFER)

    def flying_units(self):
        return [u for u in self.lmp_units if u is not None and u.status in FLYING and u.van is self]

    # -- battery magazine / 電池マガジン
    def best_pack(self):
        ks = [k for k, p in enumerate(self.packs) if p is not None]
        return max(ks, key=lambda k: self.packs[k], default=None)

    def swap_pack(self, t, unit):
        """The fullest pack into ``unit``; its own goes to the empty slot."""
        k = self.best_pack()
        if k is None or self.packs[k] <= unit.battery.level + 1.0:
            return
        empty = self.packs.index(None)
        old, unit.battery.level = unit.battery.level, self.packs[k]
        self.packs[k], self.packs[empty] = None, old
        self.swaps_in_pod += 1
        self.log(t, "battery_swap", pos=self.roof(),
                 detail="D%d: pack at %.0f%% in, %.0f%% pack to the magazine"
                 % (unit.id, unit.battery.level, old))

    def pack_step(self, dt):
        """Magazine packs charge from the pod's battery. / マガジンの電池を充電。"""
        bm, ratio = self.sky.bm, self.sky.drones.pod_draw
        for k, p in enumerate(self.packs):
            if p is not None and p < 100.0 and self.energy.level > 1.0:
                self.packs[k] = min(100.0, p + bm.slow * dt)
                self.energy.drain(dt, bm.slow * ratio)

    # -- energy for a mission / 配達に必要な電池
    def meet_point(self, stop):
        """Where this stop's drones meet the pod: its next stop if that is a
        face-to-face stop, else this stop. / 合流地点（次が対面配達ならそこ）。"""
        i = self.stops.index(stop) if stop in self.stops else -1
        nxt = self.stops[i + 1] if 0 <= i < len(self.stops) - 1 else None
        return nxt if nxt is not None and nxt.kind == FACE_TO_FACE else stop

    def mission_need(self, order, meet_stop=None):
        hover, z, _ = self.trip_plan(order)
        stop = meet_stop or self._order_stop(order)
        meet = stop.location if stop is not None else self.car_pos()
        mz = self.sky.check.flight_z(hover, meet)
        bm = self.sky.bm
        return bm.mission_cost(self.roof(), hover, meet, z, mz, self.args.drone_speed,
                               self.args.climb_speed, self.args.hover_s, 3.0,
                               self.args.meet_hold_s, self.sky.drones.plan_wind()) + bm.reserve

    def _order_stop(self, order):
        stop = next((s for s in self.stops if order in (s.drone_order or ())), None)
        return self.meet_point(stop) if stop is not None else None

    def next_drone_order(self):
        for s in self.stops:
            if s.kind == DRONE:
                for o in s.drone_order or ():
                    if o in self.pending and o.id not in self.launched:
                        return o
        return None

    def hub_orders(self, stop):
        return [o for o in (stop.drone_order or ()) if o in self.pending and o.id not in self.launched]

    # -- every tick / 毎 tick
    def lmp_tick(self, t, dt):
        """Advance the tracks, react to their key moments, pose the pod, and
        get the next drone ready when nothing else needs the cargo system."""
        a = self.anim
        for track, name, t0, t1, done in a.step(dt * self.args.lmp_speed):
            if track == "m" and name == "load" and t0 < R.HOOK_DONE <= t1 and self._loading:
                self._loading[0]._hang()          # the parcel is in the cradle now / 荷物が機体へ
            if track == "batt" and t0 < R.SWAP_DONE <= t1 and self._loading:
                self.swap_pack(t, self._loading[0])
            if track == "x" and name == "direct_order" and t0 < R.HANDED <= t1:
                self._hand_over(t)
            if track == "m" and name == "tray_back" and done:
                self._tray_out = False
            if track == "crane" and done:
                self._crane_done(t)
        if self._loading and not a.busy("m") and not a.busy("batt"):
            self.loaded, self._loading = self._loading, None
            self.log(t, "drone_ready", self.loaded[1], pos=self.roof(),
                     detail="D%d at %.0f%% with order #%d on the lift"
                     % (self.loaded[0].id, self.loaded[0].battery.level, self.loaded[1].id))
        if self._prep_allowed():
            self.lmp_prep(t)
        self.pod.apply(a.mech(self.rig_slots(), self.pod.mech.jacks), self.lmp_units)

    def _prep_allowed(self):
        if self.state in ("stranded", "ev_charging", "yard_in", "yard_out", "waiting_charger"):
            return False
        job = self._job
        if job is not None and job.get("phase") in ("direct", "receive"):
            return False
        # drones due at the stop the pod is at or heading to come first: the
        # lift must be free for them / 合流予定の機体を優先（リフトを空ける）
        # ...unless this stop still has drones to launch / 発進が残っていれば準備を続ける
        here = self.stops[0] if self.stops else None
        if here is not None and here.kind == DRONE and self.hub_orders(here):
            return True
        for m in self.meets.values():
            if here is not None and m["stop"] is here and any(
                    u.van is self and u.status in FLYING for u in m["units"]):
                return False
        return True

    def lmp_prep(self, t):
        """One step towards a loaded drone on the lift. / 発進準備を 1 段進める。"""
        a = self.anim
        if self.loaded or self._loading or a.busy("m") or a.busy("batt") or a.busy("crane") \
                or a.busy("x") or a.busy("roof"):
            return
        if self._tray_out:
            a.play("tray_back")               # last tray back to its slot first / トレイを戻す
            return
        order = self.next_drone_order()
        if order is None or not a.lift_down():
            return
        need = self.mission_need(order)
        k = self.best_pack()
        pack = self.packs[k] if k is not None else 0.0
        able = lambda u: max(u.battery.level, pack) >= need
        u = self.drone
        if u is not None and u.status in IN_VAN and able(u):
            a.play("load")
            a.play("swap")
            self._loading = (u, order)
            self.log(t, "loading", order, pos=self.roof(),
                     detail="D%d gets order #%d (%s)" % (u.id, order.id,
                                                        "on the way" if self.state == "driving"
                                                        else "at the stop"))
            return
        bays = [(i, v) for i, v in self.docked_units() if self.slots[i] != R.LAUNCH and able(v)]
        if not bays:
            return
        bi, bv = max(bays, key=lambda iv: iv[1].battery.level)
        bs = self.slots[bi]
        if u is not None and u in self.lmp_units:
            li = self.lmp_units.index(u)
            buf = self.free_bay()
            moves = [(li, R.LAUNCH, buf), (bi, bs, R.LAUNCH), (li, buf, bs)]
        else:
            moves = [(bi, bs, R.LAUNCH)]
        self._crane = a.play_crane(moves)
        self.log(t, "drone_switch", pos=self.roof(),
                 detail="crane: D%d (%.0f%%) onto the lift%s" % (
                     bv.id, bv.battery.level, ", D%d to charge" % u.id if u else ""))

    def _crane_done(self, t):
        for di, a, b, ta, tb in self._crane or []:
            self.slots[di] = b
        self._crane = None
        self.drone = next((u for u, s in zip(self.lmp_units, self.slots)
                           if u is not None and s == R.LAUNCH and u.status in IN_VAN), None)

    # -- drone stop / ドローン地点
    def lmp_begin_drone(self, t):
        s = self.stops[0]
        self._job = {"kind": "hub", "phase": "launch", "wait": 0.0}
        levels = ", ".join("D%d %.0f%%" % (u.id, u.battery.level) for _, u in self.docked_units())
        self.log(t, "drone_stop", batch=s.batch, pos=self.roof(),
                 detail="%d drops; on deck: %s" % (len(self.hub_orders(s)), levels or "none"))

    def lmp_drone_work(self, t, dt):
        self.hold()
        a, job, stop = self.anim, self._job, self.stops[0]
        if job["phase"] == "receive":
            if self._receive(t, stop, dt):
                job["phase"] = "launch"
            return
        if a.busy("roof"):
            return
        left = self.hub_orders(stop)
        if self.loaded and self.loaded[1] in left:
            if self.sky.drones.grounded:
                self._say(t, "hold", "wind_hold", pos=self.roof(),
                          detail="wind %.1f m/s: launches wait" % self.sky.drones._wind)
                return
            if a.roof_closed():
                a.play("open")
            elif not a.lift_up():
                a.play("raise")
            else:
                self._launch(t, stop)
                a.play("lower")               # doors stay open for the next / 扉は開けたまま
            return
        here = self.meets.get(id(stop))
        if not left:
            if here and any(u.van is self and u.status in FLYING for u in here["units"]):
                job["phase"] = "receive"      # they meet the pod here / ここで合流
                return
            if not a.roof_closed():
                a.play("lower" if a.lift_up() else "close")
                return
            self._end_drone_work(t)
            self._job = None
            self._next_stage(t)
            return
        # orders left, nothing loaded: wait for the prep, the drones due here,
        # or a drone that can make it / 準備中・帰還待ち・機体待ち
        if self._loading or self._crane or a.busy("m") or a.busy("batt"):
            return
        if here and any(u.van is self and u.status in FLYING for u in here["units"]):
            job["phase"] = "receive"
            return
        self._no_drone(t, dt, left[0])

    def _no_drone(self, t, dt, order):
        """Nobody on deck can make the next trip: charge, or call a standby."""
        job = self._job
        job["wait"] += dt
        docked = self.docked_units()
        need = self.mission_need(order)
        if docked:
            best = max(u.battery.level for _, u in docked)
            wait = self.sky.bm.wait_to_reach(best, need, self.sky.bm.slow)
            if wait <= self.args.max_charge_wait:
                self._say(t, "charge", "drone_charging_in_van", batch=order.batch, pos=self.roof(),
                          detail="best drone at %.0f%%, needs %.0f%% (~%.0f s on the pod's battery)"
                          % (best, need, wait))
                return
            if None not in self.lmp_units:      # deck full: send the emptiest to charge
                low = min(docked, key=lambda iu: iu[1].battery.level)[1]
                if low is not self.drone or len(docked) == 1:
                    self.sky.drones.send_to_station(low, t)
                return
        if self.incoming is None:
            self.sky.drones.request(self, need)
            self._say(t, "req", "drone_requested", batch=order.batch, pos=self.roof(),
                      detail="no drone able to fly #%d; a standby is asked for" % order.id)
        if job["wait"] > self.args.drone_wait_max:
            for o in self.hub_orders(self.stops[0]):
                self.fail_order(t, o, "no_drone")

    def _launch(self, t, stop):
        u, order = self.loaded
        self.loaded = None
        hover, z, moved = self.trip_plan(order)
        meet_stop = self.meet_point(stop)
        need = self.mission_need(order, meet_stop)
        if meet_stop is not stop and u.battery.level < need:
            meet_stop = stop                  # can't reach the next stop: meet here / ここで合流
            need = self.mission_need(order, stop)
        meet = meet_stop.location
        u.start_mission(self, order, hover, z, meet, self.sky.check.flight_z(hover, meet))
        i = self.lmp_units.index(u)
        self.slots[i] = None
        self.drone = None
        self.launched.add(order.id)
        self.in_air[u] = order
        self._tray_out = True
        m = self.meets.setdefault(id(meet_stop), {"stop": meet_stop, "units": []})
        m["units"].append(u)
        where = "here" if meet_stop is stop else "at the next stop (%s)" % meet_stop.kind
        self.log(t, "drone_launch", order, pos=self.roof(),
                 detail="D%d at %.0f%% (needs ~%.0f%%), cruise %.0f m; meets the pod %s"
                 % (u.id, u.battery.level, need - self.sky.bm.reserve, z - self.roof()[2], where))

    # -- receiving drones at a meeting point / 合流地点での回収
    def _receive(self, t, stop, dt):
        """Land the drones due here one by one; True when all are home."""
        a = self.anim
        if self._landing is not None:
            return False                      # it is coming down / 降下中
        if a.busy("roof") or a.busy("crane"):
            return False
        if self._stow:
            if a.lift_up():
                a.play("lower")
                return False
            u = self.drone
            if u is not None and u in self.lmp_units:
                i = self.lmp_units.index(u)
                self._crane = a.play_crane([(i, R.LAUNCH, self.free_bay())])
            self._stow = False
            return False
        m = self.meets.get(id(stop))
        due = [u for u in (m["units"] if m else []) if u.van is self and u.status in FLYING]
        if not due:
            if not a.roof_closed():           # lift down, doors shut first / 先に格納
                a.play("lower" if a.lift_up() else "close")
                return False
            self.meets.pop(id(stop), None)
            return True
        if self.drone is not None and self.drone.status in IN_VAN:
            if a.lift_up():
                a.play("lower")
            elif a.lift_down():               # make the lift free / リフトを空ける
                i = self.lmp_units.index(self.drone)
                if self.loaded and self.loaded[0] is self.drone:
                    self.loaded = None        # it keeps the parcel for later / 荷物は保持
                self._crane = a.play_crane([(i, R.LAUNCH, self.free_bay())])
            return False
        if a.roof_closed():
            a.play("open")
            return False
        if not a.lift_up():
            a.play("raise")
            return False
        ready = next((u for u in due if getattr(u.flight, "holding", False)), None)
        if ready is not None:
            pad = self._world(self.pod.pad)
            ready.flight.clear(lambda: pad)
            self._landing = ready
            self.log(t, "landing", pos=self.roof(), detail="D%d cleared to land" % ready.id)
        return False

    def lmp_landed(self, t, unit):
        """A drone is down on the raised pad: it is ours again. / 着陸して収容。"""
        if unit not in self.lmp_units:
            return
        i = self.lmp_units.index(unit)
        self.slots[i] = R.LAUNCH
        unit.dock_into(self)
        self.drone = unit
        self._landing, self._stow = None, True
        for m in self.meets.values():
            if unit in m["units"]:
                m["units"].remove(unit)
        self.log(t, "drone_home", pos=self.roof(),
                 detail="D%d landed at %.0f%%, to a bay to charge" % (unit.id, unit.battery.level))

    def lmp_on_drone_event(self, t, unit, kind):
        if kind == "trip_done":
            self.lmp_landed(t, unit)
        elif kind == "crashed":
            if self._landing is unit:
                self._landing = None
            for m in self.meets.values():
                if unit in m["units"]:
                    m["units"].remove(unit)
        elif kind == "at_meet":
            self.log(t, "drone_waiting", pos=unit.pos,
                     detail="D%d at the meeting point, waiting for the pod" % unit.id)

    # -- face-to-face stop / 対面配達
    def lmp_begin_direct(self, t):
        self.handover_queue = list(self.stops[0].orders)
        self._job = {"kind": "f2f", "phase": "settle"}
        self.log(t, "box_station", batch=self.stops[0].batch, pos=self.car_pos(),
                 detail="%d parcel(s) through the side port" % len(self.handover_queue))

    def _hand_over(self, t):
        if self.handover_queue:
            c = self.pod._world(R.H(R.CELL_CENTER))
            self.finish_order(t, self.handover_queue.pop(0), (c.x, c.y, c.z))

    def lmp_direct_work(self, t, dt):
        self.hold()
        a, job = self.anim, self._job
        ph = job["phase"]
        if ph == "settle":                    # let the drone prep finish first / 準備を完了
            if self._loading or self._crane or a.busy("m") or a.busy("batt") or a.busy("roof"):
                return
            job["phase"] = "direct"
            a.play("direct_order")
        elif ph == "direct":
            if a.busy("x"):
                return
            if self.handover_queue:
                a.play("direct_order")
            elif a.name.get("x") is None and job.get("collected"):
                a.x_on = False                # station and customer go / 撤去
                job["phase"] = "receive"
            else:
                a.play("direct_collect")
                job["collected"] = True
        elif ph == "receive":
            if self._receive(t, self.stops[0], dt):
                self._job = None
                self._next_stage(t)

    # -- the dashboard's showcase / ダッシュボードのショーケース
    def showcase_ok(self):
        """Why the dashboard can't take this pod over right now, or None."""
        if not getattr(self, "lmp", False):
            return "not a LastMile pod"
        if self.flying_units():
            return "drones of this pod are out"
        if self.kinematic:
            return "the pod is in the charging yard"
        if self._loading or self._crane or self.anim.busy("roof") or self.anim.busy("x"):
            return "the pod is in the middle of a sequence"
        return None

    def showcase_pose(self, t):
        """Pose the whole combined HTML cycle at ``t`` (drones in their HTML places)."""
        m = R.combined(t)
        m.drone_slot = [s if u is not None else None for u, s in zip(self.lmp_units, m.drone_slot)]
        self.pod.apply(m, self.lmp_units)

    def showcase_release(self):
        """Back to the van's own state (the next tick poses it from its tracks)."""
        if self._loading:
            self._loading = None              # its parcel load restarts / 積み込みはやり直し
