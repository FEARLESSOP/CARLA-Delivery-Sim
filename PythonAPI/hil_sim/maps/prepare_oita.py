"""Prepare Ōita Station 3D map and OpenDRIVE road logic for CARLA import.
大分駅周辺の 3D マップと OpenDRIVE 道路論理を CARLA インポート用に準備する。

Usage / 使用方法:
    python prepare_oita.py --stage /tmp/oita_stage
"""

import argparse
import os
import subprocess
import sys


DEFAULT_FBX = "/home/inouelab/Downloads/oita_station_map_v8/oita_station_map_v8/FBX_Maya_Unreal/oita_station_map_v2.fbx"
DEFAULT_OSM = "/home/inouelab/Downloads/oita_station_map_v8/oita_station_map_v8/traffic_sumo/oita_station.osm"
DEFAULT_STAGE = "/tmp/oita_stage"
MAP_NAME = "OitaStation"


def generate_opendrive(osm_path, out_xodr):
    """Generate OpenDRIVE (.xodr) with Japanese left-hand traffic rule.
    日本の左側通行規則に合わせた OpenDRIVE (.xodr) を生成する。
    """
    netconvert_bin = "/home/inouelab/Downloads/CARLA_0.9.16/carla-venv/bin/netconvert"
    if not os.path.exists(netconvert_bin):
        netconvert_bin = "netconvert"

    proj = "+proj=eqc +lat_ts=33.0855531587 +lat_0=33.2326262 +lon_0=131.6060258 +x_0=0 +y_0=0 +R=6367449 +units=m +no_defs"
    cmd = [
        netconvert_bin,
        "--osm-files", osm_path,
        "-o", "/tmp/oita_temp.net.xml",
        "--lefthand", "true",
        "--proj", proj,
        "--offset.disable-normalization", "true",
        "--geometry.remove", "true",
        "--roundabouts.guess", "true",
        "--ramps.guess", "true",
        "--junctions.join", "true",
        "--junctions.corner-detail", "5",
        "--tls.guess-signals", "true",
        "--tls.discard-simple", "true",
        "--tls.join", "true",
        "--tls.default-type", "actuated",
        "--output.street-names", "true",
        "--output.original-names", "true",
        "--remove-edges.isolated", "true",
        "--keep-edges.components", "1",
        "--no-turnarounds.tls", "true",
        "--opendrive-output", out_xodr,
        "--verbose", "false"
    ]
    print("Generating OpenDRIVE road logic via netconvert...")
    subprocess.check_call(cmd)
    print(f"Generated: {out_xodr} ({os.path.getsize(out_xodr) / (1024*1024):.2f} MB)")


def prepare_mesh_in_blender(in_fbx, out_fbx):
    """Consolidate meshes and apply CARLA semantic prefixes in Blender.
    Blender でメッシュを結合・整理し CARLA セマンティック接頭辞を付与する。
    """
    script = f"""
import bpy, os

bpy.ops.wm.read_factory_settings(use_empty=True)
print("Importing source FBX...")
bpy.ops.import_scene.fbx(filepath="{in_fbx}")

# 1. Join 742 individual buildings into a single static mesh
# 742 棟の建物を 1 つのスタティックメッシュに結合
bldg_objs = [o for o in bpy.data.objects if o.type == 'MESH' and ("Bldg" in o.name or "Building" in o.name or "Structure" in o.name)]
# Fallback: include any mesh objects not part of known non-building categories
if not bldg_objs:
    exclude = {"Ground", "Roads", "Road_Markings", "JR_Oita_City", "Railway", "Street_Furniture", "Traffic_Signals", "Trees", "Cars"}
    bldg_objs = [o for o in bpy.data.objects if o.type == 'MESH' and o.name not in exclude]
if bldg_objs:
    bpy.ops.object.select_all(action="DESELECT")
    for o in bldg_objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = bldg_objs[0]
    bpy.ops.object.join()
    bldg = bpy.context.view_layer.objects.active
    bldg.name = bldg.data.name = "SM_Buildings"

# 2. Separate Ground object by material to classify surfaces
# Ground オブジェクトをマテリアルごとに分離して路面・歩道・縁石を分類
ground = bpy.data.objects.get("Ground")
if ground:
    bpy.ops.object.select_all(action="DESELECT")
    ground.select_set(True)
    bpy.context.view_layer.objects.active = ground
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="MATERIAL")
    bpy.ops.object.mode_set(mode="OBJECT")
    
    parts = [o for o in bpy.context.scene.objects if o.name.startswith("Ground")]
    lot_parts, sidewalk_parts, curb_parts, grass_parts, plaza_parts = [], [], [], [], []
    for p in parts:
        if not p.data.polygons:
            bpy.data.objects.remove(p)
            continue
        mat_name = p.data.materials[p.data.polygons[0].material_index].name
        if "lot" in mat_name:
            lot_parts.append(p)
        elif "kerb" in mat_name or "curb" in mat_name:
            curb_parts.append(p)
        elif "sidewalk" in mat_name or "tactile" in mat_name:
            sidewalk_parts.append(p)
        elif "grass" in mat_name:
            grass_parts.append(p)
        else:
            plaza_parts.append(p)

    # Road_Road_Roads: Asphalt carriageways + parking lots
    # 車道アスファルトと駐車場
    road_obj = bpy.data.objects.get("Roads")
    if road_obj:
        bpy.ops.object.select_all(action="DESELECT")
        road_obj.select_set(True)
        for p in lot_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = road_obj
        if lot_parts:
            bpy.ops.object.join()
        road_obj.name = road_obj.data.name = "Road_Road_Roads"

    # Road_Curb_Kerb: Street curbs / 縁石
    if curb_parts:
        bpy.ops.object.select_all(action="DESELECT")
        for p in curb_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = curb_parts[0]
        if len(curb_parts) > 1:
            bpy.ops.object.join()
        c = bpy.context.view_layer.objects.active
        c.name = c.data.name = "Road_Curb_Kerb"

    # Road_Sidewalk_Sidewalk: Sidewalks and tactile paving / 歩道・点字ブロック
    if sidewalk_parts:
        bpy.ops.object.select_all(action="DESELECT")
        for p in sidewalk_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = sidewalk_parts[0]
        if len(sidewalk_parts) > 1:
            bpy.ops.object.join()
        sw = bpy.context.view_layer.objects.active
        sw.name = sw.data.name = "Road_Sidewalk_Sidewalk"

    # Road_Grass_Lawn: Grass and lawns / 芝生
    if grass_parts:
        bpy.ops.object.select_all(action="DESELECT")
        for p in grass_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = grass_parts[0]
        if len(grass_parts) > 1:
            bpy.ops.object.join()
        gr = bpy.context.view_layer.objects.active
        gr.name = gr.data.name = "Road_Grass_Lawn"

    # SM_Ground_Plaza: Pedestrian plazas and paved station forecourt / 駅前広場・タイル舗装
    if plaza_parts:
        bpy.ops.object.select_all(action="DESELECT")
        for p in plaza_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = plaza_parts[0]
        if len(plaza_parts) > 1:
            bpy.ops.object.join()
        pl = bpy.context.view_layer.objects.active
        pl.name = pl.data.name = "SM_Ground_Plaza"

# 3. Rename remaining layers to CARLA static mesh / semantic standards
# 残りのレイヤーを CARLA の命名規約に従ってリネーム
renames = {{
    "Road_Markings": "Road_Marking_Markings",
    "JR_Oita_City": "SM_JR_Oita_City",
    "Railway": "SM_Railway",
    "Trains": "SM_Trains",
    "Street_Furniture": "SM_Street_Furniture",
    "Road_Signs": "SM_Road_Signs",
    "Traffic_Signals": "SM_Traffic_Signals",
    "Trees": "SM_Trees",
    "Cars": "SM_Cars"
}}
for old_name, new_name in renames.items():
    o = bpy.data.objects.get(old_name)
    if o:
        o.name = o.data.name = new_name

print("Exporting consolidated FBX...")
bpy.ops.object.select_all(action="SELECT")
bpy.ops.export_scene.fbx(
    filepath="{out_fbx}",
    use_selection=True,
    object_types={{"MESH"}},
    apply_unit_scale=True,
    bake_space_transform=False,
    mesh_smooth_type="FACE",
    path_mode="COPY",
    embed_textures=True
)
print("Finished exporting {out_fbx}")
"""
    tmp_py = "/tmp/run_blender_prepare_oita.py"
    with open(tmp_py, "w") as f:
        f.write(script)

    print("Running Blender to prepare map FBX...")
    subprocess.check_call(["blender", "-b", "-P", tmp_py])
    print(f"Generated FBX: {out_fbx} ({os.path.getsize(out_fbx) / (1024*1024):.2f} MB)")


def main():
    parser = argparse.ArgumentParser(description="Prepare Ōita Station map for CARLA")
    parser.add_argument("--fbx", default=DEFAULT_FBX, help="Path to source FBX")
    parser.add_argument("--osm", default=DEFAULT_OSM, help="Path to source OSM")
    parser.add_argument("--stage", default=DEFAULT_STAGE, help="Output staging directory")
    args = parser.parse_args()

    os.makedirs(args.stage, exist_ok=True)
    out_fbx = os.path.join(args.stage, f"{MAP_NAME}.fbx")
    out_xodr = os.path.join(args.stage, f"{MAP_NAME}.xodr")

    if not os.path.exists(out_xodr):
        generate_opendrive(args.osm, out_xodr)
    else:
        print(f"OpenDRIVE already exists: {out_xodr}")

    if not os.path.exists(out_fbx):
        prepare_mesh_in_blender(args.fbx, out_fbx)
    else:
        print(f"FBX already exists: {out_fbx}")

    print("\nStaging complete. Run:")
    print(f"  bash build_map.sh oita_station {args.stage}")


if __name__ == "__main__":
    main()
