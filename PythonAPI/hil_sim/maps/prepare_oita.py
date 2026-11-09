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
    """Generate OpenDRIVE (.xodr) for Japanese keep-left traffic (netconvert --lefthand).
    日本の左側通行に合わせた OpenDRIVE (.xodr) を生成する（netconvert --lefthand）。
    """
    import xml.etree.ElementTree as ET

    netconvert_bin = "/home/inouelab/Downloads/CARLA_0.9.16/carla-venv/bin/netconvert"
    if not os.path.exists(netconvert_bin):
        netconvert_bin = "netconvert"

    proj = "+proj=eqc +lat_ts=33.0855531587 +lat_0=33.2326262 +lon_0=131.6060258 +x_0=0 +y_0=0 +R=6367449 +units=m +no_defs"
    raw_xodr = out_xodr + ".raw"
    venv_py = "/home/inouelab/Downloads/CARLA_0.9.16/carla-venv/bin/python"
    if not os.path.exists(venv_py):
        venv_py = sys.executable
    here = os.path.dirname(os.path.abspath(__file__))
    # the model's ring road 外周道路 is not in OSM: add it, and cut the streets
    # where they meet it (add_ring_road.py) / OSM に無い外周道路を追加
    ring_osm = out_xodr + ".ring.osm"
    subprocess.check_call([venv_py, os.path.join(here, "add_ring_road.py"), osm_path, ring_osm])
    cmd = [
        netconvert_bin,
        "--osm-files", ring_osm,
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
        "--opendrive-output", raw_xodr,
        "--verbose", "false"
    ]
    print("Generating OpenDRIVE road logic via netconvert...")
    subprocess.check_call(cmd)

    # Do NOT mark the roads rule="LHT". netconvert --lefthand already builds
    # keep-left traffic: every direction is its own road running the way the
    # cars go, on the left half of the street, with ordinary (negative)
    # lanes. rule="LHT" tells CARLA those lanes run against the road, so
    # every car drove backwards on the right half of the street, and the
    # traffic manager's hybrid mode jittered far-away cars on the spot.
    # rule="LHT" は付けない。netconvert --lefthand で既に左側通行になっており、
    # 付けると全車線の向きが逆になって右側を走る。
    tree = ET.parse(raw_xodr)
    root = tree.getroot()
    for el in [root.find("header")] + root.findall("road"):
        if el is not None and "rule" in el.attrib:
            del el.attrib["rule"]
    count = len(root.findall("road"))
    tree.write(out_xodr, encoding="utf-8", xml_declaration=True)
    if os.path.exists(raw_xodr):
        os.remove(raw_xodr)
    # traffic lights: on the ground, one per approach, grouped by junction
    # (fix_xodr_signals.py; run with the carla venv so the poles get checked)
    # 信号機を地面・進入路ごと・交差点単位に修正
    subprocess.check_call([venv_py, os.path.join(here, "fix_xodr_signals.py"), out_xodr, out_xodr])
    os.remove(ring_osm)
    print(f"Generated keep-left OpenDRIVE with {count} roads: {out_xodr} ({os.path.getsize(out_xodr) / (1024*1024):.2f} MB)")


def prepare_mesh_in_blender(in_fbx, out_fbx):
    """Consolidate meshes, sanitize material names, and apply CARLA semantic prefixes in Blender.
    Blender でメッシュを結合・整理し、マテリアル名をサニタイズして CARLA セマンティック接頭辞を付与する。
    """
    script = f"""
import bpy, os

bpy.ops.wm.read_factory_settings(use_empty=True)
print("Importing source FBX...")
bpy.ops.import_scene.fbx(filepath="{in_fbx}")

# 1. Rename all problem materials (avoiding 'sign' and 'light' substrings which trigger CARLA commandlet drop)
RENAME_MAP = {{
    'M_across_sign': 'M_across_board',
    'M_car_headlight': 'M_car_headlamp',
    'M_car_taillight': 'M_car_taillamp',
    'M_decal_jroc_sign': 'M_decal_jroc_logo',
    'M_road_signs': 'M_road_boards',
    'M_roof_slab_light': 'M_roof_slab_lit',
    'M_sign_best': 'M_board_best',
    'M_sign_collegio': 'M_board_collegio',
    'M_sign_ekimae_parking': 'M_board_ekimae_parking',
    'M_sign_hanamaru': 'M_board_hanamaru',
    'M_sign_jankara': 'M_board_jankara',
    'M_sign_npc': 'M_board_npc',
    'M_sign_red_mark': 'M_board_red_mark',
    'M_sign_regal': 'M_board_regal',
    'M_sign_routeinn': 'M_board_routeinn',
    'M_signal_housing': 'M_pole_housing',
    'M_vsign_atlas': 'M_vboard_atlas',
}}
for m in bpy.data.materials:
    if m.name in RENAME_MAP:
        m.name = RENAME_MAP[m.name]
    else:
        new_name = m.name
        if "sign" in new_name.lower():
            new_name = new_name.replace("sign", "board").replace("Sign", "Board").replace("SIGN", "BOARD")
        if "light" in new_name.lower():
            new_name = new_name.replace("light", "lit").replace("Light", "Lit").replace("LIGHT", "LIT")
        if new_name != m.name:
            m.name = new_name

# 2. Join 742 individual buildings by 16 tile groups (Buildings_A1 ... Buildings_D4)
tiles = ["A1","A2","A3","A4","B1","B2","B3","B4","C1","C2","C3","C4","D1","D2","D3","D4"]
for t in tiles:
    parent = bpy.data.objects.get("Buildings_" + t)
    children = [o for o in bpy.data.objects if o.parent == parent and o.type == "MESH"]
    if children:
        bpy.ops.object.select_all(action="DESELECT")
        for c in children:
            c.select_set(True)
        bpy.context.view_layer.objects.active = children[0]
        bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        joined.name = joined.data.name = f"SM_Buildings_{{t}}"
        joined.parent = None
    if parent:
        bpy.data.objects.remove(parent)

# 3. Separate Ground object by material to classify surfaces
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
    lot_parts, plot_parts, sidewalk_parts, curb_parts, grass_parts, plaza_parts = [], [], [], [], [], []
    for p in parts:
        if not p.data.polygons:
            bpy.data.objects.remove(p)
            continue
        mat_name = p.data.materials[p.data.polygons[0].material_index].name
        # M_plot (building plots) contains "lot" but is not a car park: Terrain, not Road
        # 敷地 (M_plot) は駐車場ではない: Road ではなく Terrain
        if mat_name.startswith("M_plot"):
            plot_parts.append(p)
        elif "lot" in mat_name:
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

    # Terrain_Lots_Plots: building plots / 建物の敷地
    if plot_parts:
        bpy.ops.object.select_all(action="DESELECT")
        for p in plot_parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = plot_parts[0]
        if len(plot_parts) > 1:
            bpy.ops.object.join()
        pt = bpy.context.view_layer.objects.active
        pt.name = pt.data.name = "Terrain_Lots_Plots"

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

# 4. Rename remaining layers to CARLA static mesh standards (avoiding sign/light)
renames = {{
    "Road_Markings": "Road_Marking_Markings",
    "JR_Oita_City": "SM_JR_Oita_City",
    "Railway": "SM_Railway",
    "Trains": "SM_Trains",
    "Street_Furniture": "SM_Street_Furniture",
    "Road_Signs": "SM_Road_Boards",
    "Traffic_Signals": "SM_Traffic_Poles",
    "Trees": "SM_Trees",
    "Cars": "SM_Cars"
}}
for old_name, new_name in renames.items():
    o = bpy.data.objects.get(old_name)
    if o:
        o.name = o.data.name = new_name

# 5. Sanity check all meshes and materials
mesh_names = [o.name for o in bpy.data.objects if o.type == 'MESH']
for name in mesh_names:
    assert "sign" not in name.lower() and "light" not in name.lower(), f"Disallowed mesh name: {{name}}"
    o = bpy.data.objects[name]
    for m in o.data.materials:
        if m:
            assert "sign" not in m.name.lower() and "light" not in m.name.lower(), f"Disallowed material: {{m.name}} in {{name}}"

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
