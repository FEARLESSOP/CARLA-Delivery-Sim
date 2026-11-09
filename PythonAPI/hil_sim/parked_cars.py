"""Parked cars: real CARLA vehicles (CARLA's own and the user's imported cars)
standing on the parking spots of a map, physics off.

The spots come from the map build (maps/build/<Map>/parked_cars.json, made by
maps/osm_city/core_cars.py from the parked cars of an embedded model such as
the Oita Station v8 model, whose own car meshes the map no longer has). The
user asked for CARLA / imported cars there instead of the model's cars
(2026-11-08). generate_traffic.py calls spawn() on maps that have the file
(--parked-cars auto) and destroys the cars with the rest at exit.

  - pool: CARLA cars (base_type car; no police, emergency or delivery pods)
    and every car of cars/cars.json except the F1 car and npc_only vehicles
    (the Oita bus, own_cars.py);
  - a car longer (+0.6 m) or wider (+0.5 m) than its spot is swapped for a
    smaller one; the sizes seen are kept in data/vehicle_extents.json;
  - the car stands on the spot's ground: placed 0.3 m up, physics off, then
    lowered so its bounding box sits on the ground; nose either way;
  - only where the loaded level's ground is where the spots say (within
    0.3 m): spots from a newer build than the installed level park nothing;
  - cars left by a killed run (role_name parked) are cleared first.

駐車車両: マップの駐車位置に CARLA の車とユーザーの輸入車を置く（物理なし）。
"""
import json
import os

import own_cars

HERE = os.path.dirname(os.path.abspath(__file__))
EXTENTS = os.path.join(HERE, 'data', 'vehicle_extents.json')
SKIP = ('police', 'ambulance', 'firetruck', 'fusorosa', 'carlacola', 'european_hgv', 'cybertruck',
        'vehicle.delivery.', 'microlino', 'sprinter', 'redbull')
SLACK = 0.6          # a car may be this much longer than its spot / 駐車枠より長くてよい量
WSLACK = 0.5         # ... and this much wider / 幅の余裕
GROUND_TOL = 0.3     # spot z vs the level's ground / 地面との許容差
MIN_MATCH = 0.6      # share of spots on the level's ground, else none / 一致率がこれ未満なら置かない
LIFT = 0.3           # spawn height above the ground, lowered after / 生成時の浮かせ量


def data_path(map_name):
    short = (map_name or '').rsplit('/', 1)[-1]
    return os.path.join(HERE, 'maps', 'build', short, 'parked_cars.json')


def load(map_name):
    """parked_cars.json of the loaded map, or None. / マップの駐車位置"""
    try:
        with open(data_path(map_name)) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def user_car_ids():
    """vehicle.<make>.<model> of the user's imported cars (cars/cars.json),
    without the npc_only vehicles (the bus). / 自作車両（npc_only を除く）"""
    return own_cars.user_car_ids()


def pool_ids(all_ids, base_types, user_ids, npc=()):
    """Blueprint ids that may park: CARLA cars and the user's cars, no
    special vehicles. all_ids: ids; base_types: {id: base_type}; npc: ids
    never parked (npc_only vehicles)."""
    out = []
    for i in sorted(all_ids):
        if any(s in i for s in SKIP) or i in npc:
            continue
        if i in user_ids or base_types.get(i) == 'car':
            out.append(i)
    return out


def spot_transform(spot, flip, carla):
    """carla.Transform of a spot (map frame: y north; CARLA: y south), LIFT up."""
    yaw = -float(spot['yaw']) + (180.0 if flip else 0.0)
    return carla.Transform(carla.Location(float(spot['x']), -float(spot['y']), float(spot['z']) + LIFT),
                           carla.Rotation(yaw=yaw))


def size(v):
    """(length, width) from an extents entry: [L, W], or L alone (older file)."""
    if v is None:
        return None, None
    if isinstance(v, (list, tuple)):
        return float(v[0]), (float(v[1]) if len(v) > 1 and v[1] is not None else None)
    return float(v), None


def fits(length, spot, width=None):
    if length is not None and length > float(spot['length']) + SLACK:
        return False
    return width is None or 'width' not in spot or width <= float(spot['width']) + WSLACK


def choose(spot, ids, extents, rng, avoid=()):
    """A blueprint id for a spot: one known to fit, else one not known to be
    too long; None if every candidate is known to be too long."""
    ok = [i for i in ids if i not in avoid and _fits_ext(extents.get(i), spot)]
    known = [i for i in ok if i in extents]
    if known and rng.random() < 0.7:
        return rng.choice(known)
    return rng.choice(ok) if ok else None


def _fits_ext(v, spot):
    L, W = size(v)
    return fits(L, spot, W)


def on_ground(world, carla, spots, log=print):
    """Spots whose z is the loaded level's ground (ground_projection from 2 m
    above). [] when too few match: the level is not the build the spots came
    from (e.g. the model-only Oita Station still installed). / 地面と一致する駐車位置"""
    ok = []
    for s in spots:
        loc = carla.Location(float(s['x']), -float(s['y']), float(s['z']) + 2.0)
        try:
            p = world.ground_projection(loc, 6.0)
        except RuntimeError:
            p = None
        if p is not None and abs(p.location.z - float(s['z'])) <= GROUND_TOL:
            ok.append(s)
    if spots and len(ok) < MIN_MATCH * len(spots):
        log('[parked] only %d of %d spots lie on this level\'s ground: the installed level is not the build '
            'parked_cars.json came from; no parked cars / 地面と一致しないため駐車車両なし' % (len(ok), len(spots)))
        return []
    return ok


def _load_extents():
    try:
        return {k: list(size(v)) for k, v in json.load(open(EXTENTS)).items()}
    except (OSError, ValueError, AttributeError, TypeError):
        return {}


def _save_extents(ext):
    try:
        json.dump(ext, open(EXTENTS, 'w'), indent=0, sort_keys=True)
    except OSError:
        pass


def spawn(client, world, data, rng, carla, apply_batch, tick, log=print, limit=0):
    """Park a car on every spot (or a random `limit` of them). apply_batch(batch,
    label) -> responses (the caller's chunked batch); tick(): world.tick for the
    tick master, else world.wait_for_tick. Returns the actor ids. / 全駐車位置に車を置く"""
    lib = world.get_blueprint_library()
    bps = {bp.id: bp for bp in lib.filter('vehicle.*')}
    base = {i: (bp.get_attribute('base_type').as_str() if bp.has_attribute('base_type') else '')
            for i, bp in bps.items()}
    ids = pool_ids(bps, base, user_car_ids(), own_cars.npc_only_ids())
    if not ids:
        log('[parked] no vehicle blueprints to park')
        return []
    # cars a killed run left behind / 前回の残り
    old = [a.id for a in world.get_actors().filter('vehicle.*') if a.attributes.get('role_name') == 'parked']
    if old:
        client.apply_batch_sync([carla.command.DestroyActor(i) for i in old])
        log('[parked] %d cars of an earlier run cleared' % len(old))
    spots = on_ground(world, carla, list(data.get('cars', [])), log)
    if limit and len(spots) > limit:
        spots = rng.sample(spots, limit)
    if not spots:
        return []
    ext = _load_extents()
    SpawnActor, FutureActor = carla.command.SpawnActor, carla.command.FutureActor
    SetSimulatePhysics = carla.command.SetSimulatePhysics
    placed = {}                       # actor id -> (spot, spawn transform) / 車 -> 駐車位置
    todo = [(s, set()) for s in spots]
    try:
        for round_ in range(3):
            batch, meta = [], []
            for s, avoid in todo:
                bid = choose(s, ids, ext, rng, avoid)
                if bid is None:
                    continue
                bp = bps[bid]
                if bp.has_attribute('color'):
                    bp.set_attribute('color', rng.choice(bp.get_attribute('color').recommended_values))
                bp.set_attribute('role_name', 'parked')
                tf = spot_transform(s, rng.random() < 0.5, carla)
                batch.append(SpawnActor(bp, tf).then(SetSimulatePhysics(FutureActor, False)))
                meta.append((s, avoid, bid, tf))
            if not batch:
                break
            resp = apply_batch(batch, 'parked cars')
            tick()
            todo, swap = [], []
            for r, (s, avoid, bid, tf) in zip(resp, meta):
                if r.error:
                    continue
                a = world.get_actor(r.actor_id)
                if a is None:
                    continue
                L, W = 2.0 * a.bounding_box.extent.x, 2.0 * a.bounding_box.extent.y
                ext[bid] = [round(L, 2), round(W, 2)]
                if not fits(L, s, W):
                    swap.append(r.actor_id)
                    todo.append((s, avoid | {bid}))
                    continue
                placed[r.actor_id] = (s, tf, a)
            if swap:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in swap])
            if not todo:
                break
        _save_extents(ext)
        # down onto the ground from the known spawn transform (an actor's
        # get_transform() read (0, 0, 0) in the tick master) / 生成時の変換から下ろす
        for aid, (s, tf, a) in placed.items():
            bb = a.bounding_box
            bottom = tf.location.z + bb.location.z - bb.extent.z
            down = carla.Transform(carla.Location(tf.location.x, tf.location.y,
                                                  tf.location.z + float(s['z']) - bottom + 0.02), tf.rotation)
            a.set_transform(down)
    except Exception as e:                          # never leak half a car park / 途中で失敗したら片付ける
        log('[parked] failed (%s): removing the %d cars parked so far' % (e, len(placed)))
        if placed:
            client.apply_batch_sync([carla.command.DestroyActor(i) for i in placed])
        return []
    users = user_car_ids()
    n_user = sum(1 for (_, _, a) in placed.values() if a.type_id in users)
    log('[parked] %d cars parked on %d spots (%d of them the user\'s cars) / 駐車車両 %d 台'
        % (len(placed), len(spots), n_user, len(placed)))
    return list(placed)


def heading_ok(spot_yaw, actor_yaw):
    """True if a car's yaw (CARLA) lies along its spot (either way)."""
    d = (float(actor_yaw) + float(spot_yaw)) % 180.0
    return min(d, 180.0 - d) < 1.0
