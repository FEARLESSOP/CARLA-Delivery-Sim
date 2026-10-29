"""The LastMile pod (LMP-4) in CARLA: its moving parts, shell modes, station and customer.

The pod is the vehicle vehicle.delivery.lastmile (build_lastmile.sh). Every
moving part is a prop attached to it and is posed by re-setting its offset
from the car (ApplyTransform on an attached actor sets the relative
transform), from a lmp_rig.Mech. Only the parts that moved are sent.

Shell modes, as the HTML's buttons: solid (shell + street-side wall),
cutaway (street-side wall hidden), xray (see-through copies of both), frame
(no shell). Hidden props stay attached, parked below the road.

The box station and the customer exist only while a direct delivery (or the
dashboard's showcase) needs them. The customer is a CARLA pedestrian walked
to the station and back, carrying the parcel.

LastMile ポッドの可動部品・外殻表示モード・箱型ステーション・客。可動部品は
車両に取り付けたプロップで、lmp_rig.Mech から相対位置を設定して動かす。
"""

import math
import random

import carla

from delivery import lmp_rig as R

VEHICLE = "vehicle.delivery.lastmile"
SHELL_MODES = ("solid", "cutaway", "xray", "frame")
HIDDEN = carla.Transform(carla.Location(z=-60.0))      # parked under the road / 路面下
PARCEL_M_BP, PARCEL_XL_BP = "static.prop.lmpparcelm", "static.prop.lmpparcelxl"
DRONE_BP, DRONE_PROP_BP, DRONE_HOOK_BP = ("static.prop.lmpdrone", "static.prop.lmpdroneprop",
                                          "static.prop.lmpdronehook")
ALL_PROPS = tuple("static.prop." + n.lower() for n in (
    "LMPShell", "LMPShellStreet", "LMPShellX", "LMPShellStreetX", "LMPHatchFront", "LMPHatchRear",
    "LMPJack", "LMPLiftPad", "LMPScissorA", "LMPScissorB", "LMPLiftPin", "LMPShuttleMast",
    "LMPShuttleCarriage", "LMPShuttleFork1", "LMPShuttleFork2", "LMPPresenterTable",
    "LMPSwapperCarrier", "LMPSwapperBoom", "LMPSwapperJaw", "LMPBatteryMagazine", "LMPBattery",
    "LMPHoistBridge", "LMPHoistGripper", "LMPTray", "LMPParcelM", "LMPParcelXL", "LMPPortDoor",
    "LMPPortStage1", "LMPPortStage2", "LMPPortStage3", "LMPPortCarrier", "LMPDrone",
    "LMPDroneProp", "LMPDroneHook", "LMPStation", "LMPStationFlap", "LMPStationDoor"))
ROTOR_STEP_DEG = 53.0       # per tick at full spin / 全回転時の 1 tick あたり


def available(world):
    lib = world.get_blueprint_library()
    return bool(lib.filter(VEHICLE)) and bool(lib.filter("static.prop.lmpshell"))


def _tf(pose):
    x, y, z, pitch, yaw, roll = pose
    return carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))


def _key(pose):
    return tuple(round(v, 4) for v in pose)


class LastMilePod(object):
    """The LMP-4's moving parts on ``car``; see the module docstring."""

    standalone, side_port = True, True

    def __init__(self, world, client, car):
        self.world, self.client, self.car = world, client, car
        self.lib = world.get_blueprint_library()
        self.mech = R.Mech()
        self.parts = {}              # (prop, node) -> actor
        self._last = {}              # actor id -> last pose key sent / 最後に送った姿勢
        for (prop, node), pose in R.part_poses(R.REST).items():
            self.parts[(prop, node)] = self._spawn(prop, pose)
        # parcels and batteries exist even when not shown (they hide below)
        for key, prop in ((("LMPParcelM", None), "LMPParcelM"), (("LMPParcelXL", None), "LMPParcelXL"),
                          (("LMPBattery", "Battery_A"), "LMPBattery")):
            if key not in self.parts:
                self.parts[key] = self._spawn(prop, None)
        self.shell = {n: self._spawn(n, (0.0,) * 6) for n in
                      ("LMPShell", "LMPShellStreet", "LMPShellX", "LMPShellStreetX")}
        self.mode = None
        self.set_shell("solid")
        self.station = {}            # prop -> actor, while a station is out
        self.walker = self.walker_parcel = None
        self._spin = 0.0
        # the raised launch pad: where drones take off and land (flight home)
        # 上げたリフトパッド：離着陸の位置
        self.pad = self.dock = carla.Location(*R.H((R.LX, R.pad_top(R.LIFT_MAX), R.DRONE_Z)))

    # -- spawning / 生成
    def _spawn(self, prop, pose):
        bp = self.lib.find("static.prop." + prop.lower())
        a = self.world.spawn_actor(bp, _tf(pose) if pose else HIDDEN, attach_to=self.car)
        self._last[a.id] = _key(pose) if pose else "hidden"
        return a

    @staticmethod
    def slot_location(slot, lift=R.LIFT_MIN):
        """Pad top of deck position ``slot`` (CARLA pod frame). / 区画のパッド上面。"""
        return carla.Location(*R.H((R.SLOTS[slot], R.slot_top(slot, lift), R.DRONE_Z)))

    # -- shell / 外殻
    def set_shell(self, mode):
        if mode not in SHELL_MODES or mode == self.mode:
            return
        show = {"solid": ("LMPShell", "LMPShellStreet"), "cutaway": ("LMPShell",),
                "xray": ("LMPShellX", "LMPShellStreetX"), "frame": ()}[mode]
        self.client.apply_batch([carla.command.ApplyTransform(
            a.id, carla.Transform() if n in show else HIDDEN) for n, a in self.shell.items()])
        self.mode = mode

    # -- posing / 姿勢
    def _cmd(self, actor, pose, cmds):
        k = _key(pose) if pose is not None else "hidden"
        if self._last.get(actor.id) != k:
            self._last[actor.id] = k
            cmds.append(carla.command.ApplyTransform(actor.id, _tf(pose) if pose else HIDDEN))

    def apply(self, mech=None, drones=None):
        """Pose every part from ``mech`` (default: the last one).

        ``drones``: the four drone units D1..D4 (DroneUnit or None), placed
        where ``mech`` has them unless they are flying on their own.
        ``drones`` は D1..D4 の機体（飛行中の機体は動かさない）。
        """
        m = self.mech = mech or self.mech
        cmds = []
        poses = R.part_poses(m)
        for key, actor in self.parts.items():
            self._cmd(actor, poses.get(key), cmds)
        if drones:
            self._drones(m, drones, cmds)
        self._station(m, cmds)
        if cmds:
            self.client.apply_batch(cmds)
        self._customer(m)

    def _drones(self, m, drones, cmds):
        poses = R.drone_poses(m)
        for i, u in enumerate(drones):
            if u is None or u.body is None or poses[i] is None or not u.docked_in(self):
                continue
            self._cmd(u.body, poses[i], cmds)
            launch = i == m.launch
            if u.hook is not None:
                self._cmd(u.hook, R.hook_pose(m.hook if launch else 0.0), cmds)
            spin = m.spin if launch else 0.0
            if spin > 0.0:
                self._spin += ROTOR_STEP_DEG * spin
                for r, p in zip(u.rotors, R.prop_poses(self._spin + 17.0 * i)):
                    self._cmd(r, p, cmds)
        if m.hook > 0.01 and m.launch is not None and drones[m.launch] is not None:
            self._tether(m)

    def _tether(self, m):
        """The winch line from the launch drone down to its hook. / 吊りひも。"""
        top = (R.LX, R.pad_top(m.lift) + R.WINCH_Y, R.DRONE_Z)
        bottom = (R.LX, R.pad_top(m.lift) + R.HOOK_Y - m.hook, R.DRONE_Z)
        tf = self.car.get_transform()
        a = tf.transform(carla.Location(*R.H(top)))
        b = tf.transform(carla.Location(*R.H(bottom)))
        self.world.debug.draw_line(a, b, thickness=0.006, color=carla.Color(20, 20, 20),
                                   life_time=0.12)

    # -- box station / 箱型ステーション
    def _station(self, m, cmds):
        if m.station and not self.station:
            for prop, pose in R.station_poses(m).items():
                self.station[prop] = self.world.spawn_actor(
                    self.lib.find("static.prop." + prop.lower()), _tf(pose), attach_to=self.car)
                self._last[self.station[prop].id] = _key(pose)
        elif not m.station and self.station:
            for a in self.station.values():
                a.destroy()
            self.station = {}
        if self.station:
            for prop, pose in R.station_poses(m).items():
                self._cmd(self.station[prop], pose, cmds)

    # -- customer / 客
    def _world(self, local):
        tf = self.car.get_transform()
        return tf.transform(carla.Location(local[0], local[1], local[2]))

    def _customer(self, m):
        person = R.person_pose(m)
        if person is None:
            for a in (self.walker_parcel, self.walker):
                if a is not None:
                    a.destroy()
            self.walker = self.walker_parcel = None
            return
        x, y, z, yaw, walking, carry = person
        target = self._world((x, y, z))
        if self.walker is None:
            bps = self.lib.filter("walker.pedestrian.*")
            if not bps:
                return
            bp = random.choice(bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "true")
            here = carla.Transform(target + carla.Location(z=1.0),
                                   carla.Rotation(yaw=self.car.get_transform().rotation.yaw + yaw))
            self.walker = self.world.try_spawn_actor(bp, here)
            if self.walker is None:
                return
        loc = self.walker.get_location()
        d = carla.Vector3D(target.x - loc.x, target.y - loc.y, 0.0)
        dist = math.hypot(d.x, d.y)
        ctl = carla.WalkerControl()
        if dist > 0.15:
            ctl.direction = carla.Vector3D(d.x / dist, d.y / dist, 0.0)
            ctl.speed = min(1.6, dist * 2.0) if walking or dist > 0.6 else 0.6
        self.walker.apply_control(ctl)
        if carry > 0.5 and self.walker_parcel is None:
            self.walker_parcel = self.world.try_spawn_actor(
                self.lib.find(PARCEL_XL_BP),
                carla.Transform(carla.Location(x=0.36, z=0.05)), attach_to=self.walker)

    # -- the PodV2 interface (towing, charging) / 旧ポッドと同じ呼び出し
    def pose(self, jacks=None, hatch=None, side=None, arm=None):
        if jacks is not None:
            self.mech.jacks = jacks
        if hatch is not None:
            self.mech.doors = hatch
        if side is not None or arm is not None:
            self.mech.port = [arm if arm is not None else self.mech.port[0],
                              side if side is not None else self.mech.port[1]]
        self.apply()

    def load_parcel(self):
        pass

    def release_parcel(self):
        p = self._world(R.H(self.mech.parcel_xl or R.REST.parcel_xl))
        return (p.x, p.y, p.z)

    @property
    def actors(self):
        out = list(self.parts.values()) + list(self.shell.values()) + list(self.station.values())
        return out + [a for a in (self.walker_parcel, self.walker) if a is not None]

    def destroy(self):
        for a in self.actors:
            a.destroy()
        self.parts, self.shell, self.station = {}, {}, {}
        self.walker = self.walker_parcel = None
