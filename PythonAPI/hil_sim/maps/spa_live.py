#!/usr/bin/env python3
"""Verification test script for Spa-Francorchamps map in CARLA.
Loads the map, checks spawn point, verifies OpenDRIVE road logic,
spawns the Red Bull RB16 F1 car, drives it forward briefly,
and saves diagnostic images.

CARLA で Spa-Francorchamps マップを検証するスクリプト。
マップの読み込み、スタート位置、OpenDRIVE の道路論理、
F1 マシン (Red Bull RB16) のスポーンと走行をテストします。
"""

import sys
import os
import time
import math
import json

# Ensure carla is in python path
for egg in os.listdir('/home/inouelab/Downloads/CARLA_0.9.16/PythonAPI/carla/dist'):
    if egg.endswith('.egg'):
        sys.path.append(os.path.join('/home/inouelab/Downloads/CARLA_0.9.16/PythonAPI/carla/dist', egg))

import carla
import numpy as np

MAPS_JSON = '/home/inouelab/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/maps/maps.json'


def main():
    with open(MAPS_JSON) as f:
        maps_cfg = json.load(f)
    spa_cfg = maps_cfg.get('spa_francorchamps', {})
    start_cfg = spa_cfg.get('start', {})
    print(f"[TEST] maps.json start config: {start_cfg}")

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    available = client.get_available_maps()
    print(f"[TEST] Available maps: {available}")
    spa_map_name = [m for m in available if 'SpaFrancorchamps' in m]
    if not spa_map_name:
        print("[ERROR] SpaFrancorchamps not found in available maps!")
        return 1

    target_map = spa_map_name[0]
    print(f"[TEST] Loading world: {target_map}...")
    world = client.load_world(target_map)
    time.sleep(2.0)

    carla_map = world.get_map()
    print(f"[TEST] Loaded map name: {carla_map.name}")

    # Check OpenDRIVE waypoints
    spawn_points = carla_map.get_spawn_points()
    print(f"[TEST] Found {len(spawn_points)} spawn points from OpenDRIVE.")

    # Target spawn from maps.json
    start_transform = carla.Transform(
        carla.Location(x=start_cfg['x'], y=start_cfg['y'], z=start_cfg['z']),
        carla.Rotation(yaw=start_cfg['yaw'])
    )
    print(f"[TEST] Desired start transform: {start_transform}")

    # Query nearest waypoint
    wp = carla_map.get_waypoint(start_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    print(f"[TEST] Nearest OpenDRIVE waypoint: {wp.transform} (road_id={wp.road_id}, lane_id={wp.lane_id}, s={wp.s:.2f})")

    # Verify waypoint succession along track
    next_wps = wp.next(50.0)
    print(f"[TEST] Next 50m waypoint: {next_wps[0].transform if next_wps else 'None'}")

    # Spawn hero vehicle (Red Bull RB16)
    bp_lib = world.get_blueprint_library()
    f1_bp = bp_lib.find('vehicle.redbull.rb16')
    f1_bp.set_attribute('role_name', 'hero')

    vehicle = world.try_spawn_actor(f1_bp, start_transform)
    if vehicle is None:
        print("[WARNING] Direct spawn at start_transform failed, trying raised Z (+1.0m)...")
        raised_tf = carla.Transform(
            carla.Location(x=start_cfg['x'], y=start_cfg['y'], z=start_cfg['z'] + 1.0),
            carla.Rotation(yaw=start_cfg['yaw'])
        )
        vehicle = world.try_spawn_actor(f1_bp, raised_tf)

    if vehicle is None:
        print("[ERROR] Failed to spawn vehicle.redbull.rb16!")
        return 1

    print(f"[TEST] Successfully spawned {vehicle.type_id} at id={vehicle.id}")
    time.sleep(1.0)

    # Attach RGB camera for Chase view
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1280')
    cam_bp.set_attribute('image_size_y', '720')
    cam_bp.set_attribute('fov', '90')

    # Chase camera transform (from car_views.json)
    chase_tf = carla.Transform(carla.Location(x=-5.0, y=0.0, z=1.7), carla.Rotation(pitch=-9.0))
    chase_cam = world.spawn_actor(cam_bp, chase_tf, attach_to=vehicle)

    # Cockpit camera transform (from car_views.json tuned driver eye)
    cockpit_tf = carla.Transform(carla.Location(x=0.14, y=0.0, z=0.74), carla.Rotation(pitch=-6.0))
    cockpit_cam = world.spawn_actor(cam_bp, cockpit_tf, attach_to=vehicle)

    images = {}
    chase_cam.listen(lambda img: images.update({'chase': img}))
    cockpit_cam.listen(lambda img: images.update({'cockpit': img}))

    # Let physics settle
    for _ in range(30):
        world.tick()
        time.sleep(0.05)

    pos_before = vehicle.get_location()
    print(f"[TEST] Vehicle position settled: {pos_before}")

    # Drive forward
    vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0, brake=0.0))
    for _ in range(60):
        world.tick()
        time.sleep(0.05)

    pos_after = vehicle.get_location()
    speed = vehicle.get_velocity()
    speed_kmh = 3.6 * math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
    dist = pos_after.distance(pos_before)
    print(f"[TEST] Vehicle after throttle: pos={pos_after}, dist_moved={dist:.2f}m, speed={speed_kmh:.1f} km/h")

    # Save images
    out_dir = '/tmp/spa_verify'
    os.makedirs(out_dir, exist_ok=True)
    if 'chase' in images:
        images['chase'].save_to_disk(os.path.join(out_dir, 'spa_chase.png'))
        print(f"[TEST] Saved {out_dir}/spa_chase.png")
    if 'cockpit' in images:
        images['cockpit'].save_to_disk(os.path.join(out_dir, 'spa_cockpit.png'))
        print(f"[TEST] Saved {out_dir}/spa_cockpit.png")

    # Cleanup actors
    chase_cam.destroy()
    cockpit_cam.destroy()
    vehicle.destroy()
    print("[TEST] All tests completed successfully!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
