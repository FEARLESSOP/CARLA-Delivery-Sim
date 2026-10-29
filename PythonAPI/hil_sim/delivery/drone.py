"""Server-free drone sortie state machine (the ``kinematic`` backend's brain).

A sortie is one round trip per drop, starting and ending in the van: climb
out of the van (through the pod's roof hatch) to that trip's cruise altitude,
fly over the drop point, descend, hover while the parcel is handed off, climb
back, fly back over the van and descend into it. If more drops are left the
drone reloads a parcel inside the van and goes again; after the last one it
stays in. Each trip has its own altitude (``leg_z``), set by what is in the
way (planning.leg_altitude). Positions are (x, y, z) in CARLA world metres.

ドローン出撃の状態機械（サーバー不要、``kinematic`` バックエンドの中核）。
投下地点ごとに車から出て戻る往復：車（ポッドの屋根ハッチ）から巡航高度へ
上昇 → 投下地点の上空へ移動・降下・ホバリングして荷物を渡す → 再上昇して
車の上空に戻り車内へ降下。残りがあれば車内で次の荷物を積んで再出発し、
最後の後は車内に留まる。往復ごとに障害物に応じた巡航高度（``leg_z``）を持つ。
座標は CARLA ワールド座標 [m] の (x, y, z)。
"""

import math

IDLE = "idle"
TAKEOFF = "takeoff"
CRUISE = "cruise"
DESCEND = "descend"
DROP = "drop"
ASCEND = "ascend"
RETURN = "return"
LAND = "land"
RELOAD = "reload"
DONE = "done"
OUTBOUND = (TAKEOFF, CRUISE, DESCEND, DROP)   # carrying a parcel / 荷物を運搬中


def _move_towards(p, target, max_step):
    d = [target[i] - p[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    if n <= max_step or n == 0.0:
        return tuple(target), True
    return tuple(p[i] + d[i] / n * max_step for i in range(3)), False


class Sortie(object):
    """Round trips from ``home_fn()`` to each of ``drops``, reloading in between.

    ``home_fn`` returns where the drone sits in the van (inside the pod, under
    the hatch), so the drone homes in on the van even if it moved.
    ``update(dt)`` advances the flight and returns a list of ``(event,
    payload)`` tuples: ("delivered", i), ("reloading", i) when it is back in
    the van to load parcel ``i``, and ("landed", None) after the last trip.

    ``home_fn()``（車内、ハッチの真下）から各 ``drops`` への往復を、間に荷物の
    積み込みを挟んで行う。``update(dt)`` は ("delivered", i)・("reloading", i)
    （車内で i 番目の荷物を積む）・("landed", None)（最後の往復の後）を返す。
    """

    def __init__(self, home_fn, drops, cruise_alt=40.0, drop_height=3.0,
                 h_speed=12.0, v_speed=4.0, hover_s=4.0, reload_s=5.0, leg_z=None):
        self.home_fn = home_fn
        self.drops = list(drops)
        self.cruise_alt = cruise_alt      # metres above the launch point / 発進点からの高さ
        self.drop_height = drop_height    # hover height above the drop point / 投下点上のホバ高さ
        self.h_speed = h_speed
        self.v_speed = v_speed
        self.hover_s = hover_s
        self.reload_s = reload_s          # loading the next parcel / 次の荷物の積み込み [s]
        self.pos = tuple(home_fn())
        # world z of each round trip / 往復ごとのワールド z
        if leg_z is None:
            leg_z = [self.pos[2] + cruise_alt] * len(self.drops)
        if len(leg_z) != len(self.drops):
            raise ValueError("need one altitude per drop (one per round trip)")
        self.leg_z = list(leg_z)
        self.state = TAKEOFF if self.drops else DONE
        self.target_i = 0
        self._timer = 0.0
        self.distance = 0.0

    @property
    def done(self):
        return self.state == DONE

    @property
    def loaded(self):
        """Carrying a parcel right now. / 荷物を運搬中か。"""
        return self.state in OUTBOUND

    def _step(self, target, speed, dt):
        new, arrived = _move_towards(self.pos, target, speed * dt)
        self.distance += math.sqrt(sum((new[i] - self.pos[i]) ** 2 for i in range(3)))
        self.pos = new
        return arrived

    def update(self, dt):
        events = []
        s = self.state
        i = self.target_i
        if s == TAKEOFF:
            if self._step((self.pos[0], self.pos[1], self.leg_z[i]), self.v_speed, dt):
                self.state = CRUISE
        elif s == CRUISE:
            d = self.drops[i]
            if self._step((d[0], d[1], self.leg_z[i]), self.h_speed, dt):
                self.state = DESCEND
        elif s == DESCEND:
            d = self.drops[i]
            if self._step((d[0], d[1], d[2] + self.drop_height), self.v_speed, dt):
                self.state, self._timer = DROP, self.hover_s
        elif s == DROP:
            self._timer -= dt
            if self._timer <= 0.0:
                events.append(("delivered", i))
                self.state = ASCEND
        elif s == ASCEND:
            if self._step((self.pos[0], self.pos[1], self.leg_z[i]), self.v_speed, dt):
                self.state = RETURN
        elif s == RETURN:
            h = self.home_fn()
            if self._step((h[0], h[1], self.leg_z[i]), self.h_speed, dt):
                self.state = LAND
        elif s == LAND:
            h = self.home_fn()
            # keep tracking the van horizontally while descending
            self.pos = (h[0], h[1], self.pos[2])
            if self._step(tuple(h), self.v_speed, dt):
                if i + 1 < len(self.drops):
                    events.append(("reloading", i + 1))
                    self.state, self._timer = RELOAD, self.reload_s
                else:
                    events.append(("landed", None))
                    self.state = DONE
        elif s == RELOAD:
            self._timer -= dt
            if self._timer <= 0.0:
                self.target_i += 1
                self.state = TAKEOFF
        return events

    def heading(self):
        """Yaw (deg) towards the current horizontal target, for the 3D model."""
        if self.state in OUTBOUND and self.drops:
            t = self.drops[self.target_i]
        else:
            t = self.home_fn()
        dx, dy = t[0] - self.pos[0], t[1] - self.pos[1]
        if dx * dx + dy * dy < 1.0:
            return None             # straight up or down: keep the last heading
        return math.degrees(math.atan2(dy, dx))


class Transfer(object):
    """Empty point-to-point flight: straight up to ``cruise_z``, across, down
    onto ``target_fn()`` (a van's dock or a charge-station pad).

    ``update(dt)`` returns [("arrived", None)] once it has landed.
    荷物なしの移動飛行：``cruise_z`` まで上昇 → 水平移動 → ``target_fn()``（車内
    または充電パッド）へ降下。着地で ("arrived", None) を返す。
    """

    def __init__(self, start, target_fn, cruise_z, h_speed=12.0, v_speed=4.0):
        self.pos = tuple(start)
        self.target_fn = target_fn
        self.cruise_z = max(cruise_z, start[2])
        self.h_speed, self.v_speed = h_speed, v_speed
        self.state = TAKEOFF
        self.distance = 0.0
        self.loaded = False

    @property
    def done(self):
        return self.state == DONE

    def _step(self, target, speed, dt):
        new, arrived = _move_towards(self.pos, target, speed * dt)
        self.distance += math.sqrt(sum((new[i] - self.pos[i]) ** 2 for i in range(3)))
        self.pos = new
        return arrived

    def update(self, dt):
        if self.state == TAKEOFF:
            if self._step((self.pos[0], self.pos[1], self.cruise_z), self.v_speed, dt):
                self.state = CRUISE
        elif self.state == CRUISE:
            h = self.target_fn()
            if self._step((h[0], h[1], self.cruise_z), self.h_speed, dt):
                self.state = LAND
        elif self.state == LAND:
            h = self.target_fn()
            self.pos = (h[0], h[1], self.pos[2])
            if self._step(tuple(h), self.v_speed, dt):
                self.state = DONE
                return [("arrived", None)]
        return []

    def heading(self):
        t = self.target_fn()
        dx, dy = t[0] - self.pos[0], t[1] - self.pos[1]
        if dx * dx + dy * dy < 1.0:
            return None
        return math.degrees(math.atan2(dy, dx))


TO_MEET, HOLD = "to_meet", "hold"


class Mission(object):
    """One delivery from a pod, ending at a meeting point instead of back home.

    Up from ``start`` (the pod's raised launch pad) to ``cruise_z``, over the
    drop, down, hover while the tether lowers the parcel, up to ``meet_z``,
    over ``meet`` (where the pod will be), and hold there until the pod clears
    the drone to land (``clear(land_fn)``, ``land_fn()`` = its pad); then
    across to the pad and down. Events: ("delivered", 0), ("at_meet", None),
    ("landed", None). World metres, like Sortie.

    ポッドから 1 回配達し、帰路は合流地点へ。合流地点上空で待機し、ポッドが着陸を
    許可したら（``clear``）パッドへ降りる。
    """

    def __init__(self, start, drop, cruise_z, meet, meet_z, h_speed=12.0, v_speed=4.0,
                 hover_s=4.0, drop_height=3.0):
        self.pos = tuple(start)
        self.drops = [tuple(drop)]
        self.meet = tuple(meet)
        self.cruise_z = max(cruise_z, start[2] + 2.0)
        self.meet_z = max(meet_z, meet[2] + 15.0)
        self.h_speed, self.v_speed = h_speed, v_speed
        self.hover_s, self.drop_height = hover_s, drop_height
        self.land_fn = None
        self.state = TAKEOFF
        self.target_i = 0
        self._timer = 0.0
        self.distance = 0.0
        self.hold_s = 0.0               # time spent holding / 待機時間

    @property
    def done(self):
        return self.state == DONE

    @property
    def loaded(self):
        return self.state in OUTBOUND

    @property
    def holding(self):
        return self.state == HOLD

    def clear(self, land_fn):
        """The pod is ready: land on ``land_fn()``. / 着陸許可。"""
        self.land_fn = land_fn

    def _step(self, target, speed, dt):
        new, arrived = _move_towards(self.pos, target, speed * dt)
        self.distance += math.sqrt(sum((new[i] - self.pos[i]) ** 2 for i in range(3)))
        self.pos = new
        return arrived

    def update(self, dt):
        events, s, d = [], self.state, self.drops[0]
        if s == TAKEOFF:
            if self._step((self.pos[0], self.pos[1], self.cruise_z), self.v_speed, dt):
                self.state = CRUISE
        elif s == CRUISE:
            if self._step((d[0], d[1], self.cruise_z), self.h_speed, dt):
                self.state = DESCEND
        elif s == DESCEND:
            if self._step((d[0], d[1], d[2] + self.drop_height), self.v_speed, dt):
                self.state, self._timer = DROP, self.hover_s
        elif s == DROP:
            self._timer -= dt
            if self._timer <= 0.0:
                events.append(("delivered", 0))
                self.state = ASCEND
        elif s == ASCEND:
            if self._step((self.pos[0], self.pos[1], self.meet_z), self.v_speed, dt):
                self.state = TO_MEET
        elif s == TO_MEET:
            if self._step((self.meet[0], self.meet[1], self.meet_z), self.h_speed, dt):
                self.state = HOLD
                events.append(("at_meet", None))
        elif s == HOLD:
            self.hold_s += dt
            if self.land_fn is not None:
                self.state = RETURN
        elif s == RETURN:
            h = self.land_fn()
            if self._step((h[0], h[1], self.pos[2]), self.h_speed * 0.5, dt):
                self.state = LAND
        elif s == LAND:
            h = self.land_fn()
            self.pos = (h[0], h[1], self.pos[2])
            if self._step(tuple(h), self.v_speed, dt):
                events.append(("landed", None))
                self.state = DONE
        return events

    def heading(self):
        if self.state in OUTBOUND:
            t = self.drops[0]
        elif self.state in (RETURN, LAND) and self.land_fn is not None:
            t = self.land_fn()
        else:
            t = self.meet
        dx, dy = t[0] - self.pos[0], t[1] - self.pos[1]
        if dx * dx + dy * dy < 1.0:
            return None
        return math.degrees(math.atan2(dy, dx))
