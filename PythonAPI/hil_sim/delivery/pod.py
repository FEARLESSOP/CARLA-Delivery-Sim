"""The delivery pod: the artist's Pod.fbx riding on a hidden car, with moving parts.

The pod mesh can't drive (a CARLA vehicle needs a rigged skeleton), so it is
attached to a car of the same length and wheelbase. Its jacks and roof-hatch
doors are props of their own, attached to the same car and animated by
re-setting their offset from the car (ApplyTransform on an attached actor
sets the relative transform; the attachment holds while the car drives).
All parts of one pose go to the server in one batch.

配送ポッド：アーティスト製 Pod.fbx を隠れた車に載せ、可動部品を持たせる。
ジャッキと屋根ハッチの扉は別プロップとして同じ車に取り付け、車からの相対
位置を再設定して動かす（取り付け済みアクターへの ApplyTransform は相対変換を
設定し、走行中も取り付けは保たれる）。1 姿勢の全部品を 1 回のバッチで送る。
"""

import carla

from delivery import rig

POD_PROP = "static.prop.deliverypod"
JACK_PROP = "static.prop.podjack"
HATCH_PROPS = {"front": "static.prop.podhatchfront", "rear": "static.prop.podhatchrear"}
SIDE_DOOR_PROP = "static.prop.podsidedoor"
SLIDE_PROPS = {"stage1": "static.prop.podslidestage1", "stage2": "static.prop.podslidestage2",
               "stage3": "static.prop.podslidestage3", "carrier": "static.prop.podslidecarrier"}
PARCEL_PROP = "static.prop.parcel"
# every prop the delivery code spawns (for cleaning up after a crash)
# 配送コードが生成する全プロップ（異常終了後の片付け用）
ALL_PROPS = (POD_PROP, JACK_PROP, HATCH_PROPS["front"], HATCH_PROPS["rear"], SIDE_DOOR_PROP,
             "static.prop.deliverydrone", "static.prop.dronerotor", PARCEL_PROP) + \
    tuple(SLIDE_PROPS.values())
# the pod as its own vehicle (build_models.sh registers it); without it the
# pod prop rides on a hidden car / 車両として登録済みならそれを使い、無ければ隠れた車に載せる
POD_VEHICLE = "vehicle.delivery.pod"
CARRIER, CARRIER_COLOR = "vehicle.toyota.prius", "0,0,0"


def available(world):
    lib = world.get_blueprint_library()
    return all(lib.filter(b) for b in (POD_PROP, JACK_PROP) + tuple(HATCH_PROPS.values()))


def is_vehicle(world):
    """The pod exists as a drivable vehicle. / ポッドが車両として存在するか。"""
    return bool(world.get_blueprint_library().filter(POD_VEHICLE))


def has_side_port(world):
    lib = world.get_blueprint_library()
    return all(lib.filter(b) for b in (SIDE_DOOR_PROP,) + tuple(SLIDE_PROPS.values()))


def _tf(pose, mount):
    x, y, z, pitch, yaw, roll = pose
    return carla.Transform(carla.Location(mount.x + x, mount.y + y, mount.z + z),
                           carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))


class Pod(object):
    """The pod's moving parts on ``car``: 4 jacks, 2 hatch doors and (PodV2) the
    side door and arm. ``car`` is the pod vehicle itself, or a hidden carrier
    car, in which case the pod body is a prop on it too.
    ポッドの可動部品（ジャッキ・ハッチ扉・サイドドアとアーム）。``car`` はポッド
    車両そのもの、または隠れた車（その場合は本体もプロップ）。
    """

    def __init__(self, world, client, car):
        self.world, self.client, self.car = world, client, car
        self.mount = self._mount_point(car)
        lib = world.get_blueprint_library()
        spawn = lambda bp, pose: world.spawn_actor(lib.find(bp), _tf(pose, self.mount),
                                                   attach_to=car)
        self.standalone = car.type_id == POD_VEHICLE
        self.body = None if self.standalone else spawn(POD_PROP, (0.0,) * 6)
        self.jacks = {k: spawn(JACK_PROP, p) for k, p in rig.jack_poses(0.0).items()}
        self.doors = {k: spawn(HATCH_PROPS[k], p) for k, p in rig.hatch_poses(0.0).items()}
        self.side_door, self.slide = None, {}
        if has_side_port(world):
            self.side_door = spawn(SIDE_DOOR_PROP, rig.side_door_pose(0.0))
            self.slide = {k: spawn(SLIDE_PROPS[k], p) for k, p in rig.slide_poses(0.0).items()}
        self.parcel = None          # on the arm's tray / トレイ上の荷物
        self.jacks_u = self.hatch_u = self.side_u = self.arm_u = 0.0
        m = self.mount
        # drone spots in the car frame: on the hatch (roof) and parked inside
        # 車座標でのドローン位置：ハッチ上（屋根）と車内の格納位置
        hx, hy = rig.HATCH_XY
        self.pad = carla.Location(m.x + hx, m.y + hy, m.z + rig.ROOF_Z)
        self.dock = carla.Location(m.x + hx, m.y + hy, m.z + rig.DOCK_Z)

    @staticmethod
    def _mount_point(car):
        """The pod's origin (ground, between the axles) in the car frame.

        The pod vehicle is built with its origin exactly there. A carrier car
        is measured from its own wheels -- only once it has settled: cars
        spawn in the air and drop, and the wheel positions (from the server,
        now) and the car's transform (last snapshot) disagree while it falls,
        which once put every pod part ~10 cm too low.
        ポッド車両は原点がそこにある。隠れた車は着地後に車輪から求める（落下中は
        車輪位置と車体位置の時刻がずれ、部品が約 10 cm 下がった）。
        """
        if car.type_id == POD_VEHICLE:
            return carla.Location(0.0, 0.0, 0.0)
        inv = car.get_transform().get_inverse_matrix()
        wheels = car.get_physics_control().wheels

        def local(p):       # wheel positions are world centimetres / 世界座標 cm
            v = (p.x / 100.0, p.y / 100.0, p.z / 100.0, 1.0)
            return [sum(inv[r][k] * v[k] for k in range(4)) for r in range(3)]

        pts = [local(w.position) for w in wheels]
        return carla.Location(x=sum(p[0] for p in pts) / len(pts),
                              z=sum(p[2] for p in pts) / len(pts) - wheels[0].radius / 100.0)

    @property
    def side_port(self):
        return self.side_door is not None

    def load_parcel(self):
        """A parcel onto the arm's tray (still inside the pod). / トレイに荷物。"""
        if self.parcel is None and self.slide:
            self.parcel = self.world.try_spawn_actor(
                self.world.get_blueprint_library().find(PARCEL_PROP),
                carla.Transform(carla.Location(z=rig.CARRIER_TOP)),
                attach_to=self.slide["carrier"])

    def release_parcel(self):
        """Take the parcel off the tray: returns where it was (world x, y, z).

        CARLA can't detach it, so the caller spawns a free one there.
        トレイから外し、その位置（ワールド座標）を返す。
        """
        x, y, z, _, _, _ = rig.slide_poses(self.arm_u)["carrier"]
        p = self.car.get_transform().transform(carla.Location(
            self.mount.x + x, self.mount.y + y, self.mount.z + z + rig.CARRIER_TOP))
        if self.parcel is not None:
            self.parcel.destroy()
            self.parcel = None
        return (p.x, p.y, p.z)

    def pose(self, jacks=None, hatch=None, side=None, arm=None):
        """Move jacks (0 up .. 1 down), hatch, side door and/or arm (0 shut / in
        .. 1 open / out), all in one batch."""
        cmds = []
        if jacks is not None:
            self.jacks_u = jacks
            cmds += [carla.command.ApplyTransform(self.jacks[k].id, _tf(p, self.mount))
                     for k, p in rig.jack_poses(jacks).items()]
        if hatch is not None:
            self.hatch_u = hatch
            cmds += [carla.command.ApplyTransform(self.doors[k].id, _tf(p, self.mount))
                     for k, p in rig.hatch_poses(hatch).items()]
        if side is not None and self.side_door is not None:
            self.side_u = side
            cmds.append(carla.command.ApplyTransform(self.side_door.id,
                                                     _tf(rig.side_door_pose(side), self.mount)))
        if arm is not None and self.slide:
            self.arm_u = arm
            cmds += [carla.command.ApplyTransform(self.slide[k].id, _tf(p, self.mount))
                     for k, p in rig.slide_poses(arm).items()]
        if cmds:
            self.client.apply_batch(cmds)

    @property
    def actors(self):
        parts = list(self.jacks.values()) + list(self.doors.values()) + list(self.slide.values())
        return ([self.parcel] if self.parcel else []) + parts + \
            ([self.side_door] if self.side_door else []) + ([self.body] if self.body else [])

    def destroy(self):
        for a in self.actors:
            a.destroy()
