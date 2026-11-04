"""Unreal Editor step after the map import (build_map.sh runs it with the
PythonScript commandlet): the maples' leaf meshes get CARLA's foliage master
material, so the cards are cut out by the texture's alpha (masked), render
from both sides, sway with the weather's wind, and take the season swap
(seasons.py sets the "BaseColor" texture parameter at runtime).

Env: FOLIAGE_PKG (e.g. Japan), FOLIAGE_MAP (e.g. OitaKosen), FOLIAGE_TEXDIR
(the build's textures folder: one <species>_leaves_summer.png with alpha per
tree species; each species' leaf meshes are named *<Species>Leaves*).

取り込み後の UE エディタ処理: モミジの葉メッシュに CARLA の植生マスター
マテリアル（マスク・両面・風で揺れる）を設定する。季節は実行時に差し替え。
"""

import os

import unreal

PKG = os.environ["FOLIAGE_PKG"]
NAME = os.environ["FOLIAGE_MAP"]
TEXDIR = os.environ["FOLIAGE_TEXDIR"]
MASTER = "/Game/Carla/Static/GenericMaterials/Masters/M_Foliage"
DEST = "/Game/%s/Static/Foliage/%s" % (PKG, NAME)
SPECIES = {"maple": "MapleLeaves", "sakura": "SakuraLeaves", "ginkgo": "GinkgoLeaves",
           "zelkova": "ZelkovaLeaves", "camphor": "CamphorLeaves"}      # = jtrees.KIND

tools = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary


def log(s):
    unreal.log_warning("[ue_foliage] " + s)


def import_texture(path, name):
    task = unreal.AssetImportTask()
    task.filename = path
    task.destination_path = DEST
    task.destination_name = name
    task.automated = True
    task.replace_existing = True
    task.save = True
    tools.import_asset_tasks([task])
    tex = lib.load_asset("%s/%s" % (DEST, name))
    if tex is None:
        raise RuntimeError("texture import failed: %s" % path)
    return tex


def main():
    master = lib.load_asset(MASTER)
    if master is None:
        raise RuntimeError("no foliage master at " + MASTER)
    root = "/Game/%s/Static" % PKG
    assets = lib.list_assets(root, recursive=True, include_folder=False)
    total = 0
    for sp, key in SPECIES.items():
        png = os.path.join(TEXDIR, "%s_leaves_summer.png" % sp)
        meshes = [p for p in assets if key in p.split("/")[-1] and NAME in p.split("/")[-1]]
        if not meshes or not os.path.exists(png):
            continue
        cap = key[:-len("Leaves")]
        tex = import_texture(png, "T_%sLeaves" % cap)
        mi_path = "%s/MI_%sLeaves" % (DEST, cap)
        mi = lib.load_asset(mi_path) if lib.does_asset_exist(mi_path) else tools.create_asset(
            "MI_%sLeaves" % cap, DEST, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        mel.set_material_instance_parent(mi, master)
        mel.set_material_instance_texture_parameter_value(mi, "BaseColor", tex)
        lib.save_loaded_asset(mi)
        n = 0
        for p in meshes:
            sm = lib.load_asset(p)
            if not isinstance(sm, unreal.StaticMesh):
                continue
            for i in range(len(sm.static_materials)):
                sm.set_material(i, mi)
            lib.save_loaded_asset(sm)
            n += 1
        log("%s: %d leaf meshes on %s" % (sp, n, mi_path))
        total += n
    log("%d leaf meshes on the foliage material (%s)" % (total, MASTER))
    if total == 0:
        raise RuntimeError("no *Leaves meshes for %s under %s" % (NAME, root))


main()
