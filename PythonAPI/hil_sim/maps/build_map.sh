#!/usr/bin/env bash
# Import a prepared circuit (FBX + OpenDRIVE) into the packaged CARLA as a map.
#
#   maps/build_map.sh <map key> <stage dir>
#
# <stage dir> holds <Name>.fbx and <Name>.xodr (maps/extract_track.py,
# make_track_xodr.py and prepare_map.py make them). The map is then loadable
# with client.load_world("<Name>"); restart CARLA first (it scans maps at start).
# Needs the CARLA 0.9.16 source tree with a built editor, UE 4.26.
#
# 用意したサーキット（FBX + OpenDRIVE）をパッケージ版 CARLA にマップとして入れる。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CARLA_ROOT="$(cd "$HERE/../../.." && pwd)"
KEY="$1"; SRC="$2"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
export UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
PY="$CARLA_ROOT/carla-venv/bin/python"
read -r NAME PKG < <("$PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(c['name'], c['package'])" "$HERE/maps.json" "$KEY")
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
if [ -f "$CARLA_SRC/Unreal/CarlaUnreal/CarlaUnreal.uproject" ]; then
    UPROJECT="$CARLA_SRC/Unreal/CarlaUnreal/CarlaUnreal.uproject"
    ENGINE_INI="${CARLA_SRC}/Unreal/CarlaUnreal/Config/DefaultEngine.ini"
else
    UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
    ENGINE_INI="${CARLA_SRC}/Unreal/CarlaUE4/Config/DefaultEngine.ini"
fi
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
STAGE="$CARLA_SRC/Import/$PKG"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
PARK="$CARLA_SRC/Import_parked"
step() { printf '\n== %s\n' "$*"; }

step "1/4 stage $NAME ($PKG)"
mkdir -p "$STAGE" "$CONTENT/$PKG/Maps"
rm -rf "$STAGE/$NAME".* "$STAGE/textures" "$STAGE/$NAME.fbm" "$CONTENT/$PKG/Maps/$NAME"
# the importer moves each mesh into Static/<tag>/<map>; a mesh left there by an
# earlier import blocks the move ("already exists"), and the cooked map keeps
# the OLD mesh beside the new one / 前回の取り込みの残りを消す（残ると古いメッシュが残る）
rm -rf "$CONTENT/$PKG/Static/"*/"$NAME"
cp "$SRC/$NAME.fbx" "$SRC/$NAME.xodr" "$STAGE/"
[ -d "$SRC/textures" ] && cp -r "$SRC/textures" "$STAGE/"
[ -d "$SRC/$NAME.fbm" ] && cp -r "$SRC/$NAME.fbm" "$STAGE/"
cat > "$STAGE/$PKG.json" <<JSON
{"maps": [{"name": "$NAME", "source": "./$NAME.fbx", "use_carla_materials": false, "xodr": "./$NAME.xodr"}], "props": []}
JSON

step "2/4 import (only this package in Import/; Import.py takes every .json there)"
mkdir -p "$PARK"
sed -i 's/r.GenerateMeshDistanceFields=True/r.GenerateMeshDistanceFields=False/' "$ENGINE_INI"
restore() {
    sed -i 's/r.GenerateMeshDistanceFields=False/r.GenerateMeshDistanceFields=True/' "$ENGINE_INI" 2>/dev/null || true
    [ -d "$PARK" ] && find "$PARK" -mindepth 1 -maxdepth 1 -exec mv -t "$CARLA_SRC/Import/" {} + 2>/dev/null; rmdir "$PARK" 2>/dev/null || true
}
trap restore EXIT
unpark() {
    [ -d "$PARK" ] && find "$PARK" -mindepth 1 -maxdepth 1 -exec mv -t "$CARLA_SRC/Import/" {} + 2>/dev/null; rmdir "$PARK" 2>/dev/null || true
}
find "$CARLA_SRC/Import" -mindepth 1 -maxdepth 1 ! -name "$PKG" ! -name README.md -exec mv -t "$PARK/" {} +
if [ -f "$CARLA_SRC/Util/Tools/Import.py" ]; then
    IMPORT_PY="Util/Tools/Import.py"
else
    IMPORT_PY="Util/BuildTools/Import.py"
fi
(cd "$CARLA_SRC" && "$PY" "$IMPORT_PY" --package "$PKG")
unpark
ls "$CONTENT/$PKG/Maps/$NAME/" || { echo "import failed: no map"; exit 1; }

if ls "$SRC"/textures/*_leaves_summer.png >/dev/null 2>&1; then
    # trees with leaves of their own (maple.py): CARLA's foliage material on
    # them (masked cut-out, two-sided, sways with the wind, season swap)
    # 葉を独立させた樹木: CARLA の植生マテリアルを設定（マスク・両面・風・季節）
    step "2b/4 foliage material"
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && FOLIAGE_PKG="$PKG" FOLIAGE_MAP="$NAME" \
        FOLIAGE_TEXDIR="$SRC/textures" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_foliage.py" -unattended -nosplash) \
        > "$SRC/ue_foliage.log" 2>&1 || true
    grep "\[ue_foliage\]" "$SRC/ue_foliage.log" || true
    grep -q "\[ue_foliage\] [1-9][0-9]* leaf meshes on the foliage" "$SRC/ue_foliage.log" || { echo "foliage step failed (see $SRC/ue_foliage.log)"; exit 1; }
fi

RECAST="$CARLA_SRC/Build/recast-c10-install/bin/RecastBuilder"
if [ -d "$SRC/meshes" ] && [ -x "$RECAST" ] && [ ! -f "$CONTENT/$PKG/Maps/$NAME/Nav/$NAME.bin" ]; then
    # the walkers' navmesh (FBX2OBJ is missing, so the import makes none and
    # step 1 deleted the old one): guide ch. 7 / 歩行者ナビメッシュ（ガイド 7 章）
    step "2c/4 walker navmesh"
    MV="$HERE/.venv/bin/python"
    # the navmesh only depends on the road/pavement/grass/street meshes; a full
    # map takes ~2 h, so it is cached by a hash of exactly those inputs
    # 入力メッシュのハッシュでキャッシュ（全体で約 2 時間かかる）
    NAVKEY="$("$MV" -c "
import glob, hashlib, os, sys, numpy as np
keys = ('Road_Road', 'Road_Curb', 'Road_Crosswalk', 'Road_Sidewalk', 'Road_Grass', 'Terrain_Street')
h = hashlib.sha1()
for f in sorted(glob.glob(os.path.join(sys.argv[1], '*.npz'))):
    if os.path.basename(f).rsplit('_', 2)[0] in keys:
        d = np.load(f)
        h.update(os.path.basename(f).encode())
        for k in sorted(d.files):
            if k.endswith(('|V', '|I')):
                h.update(k.encode()); h.update(np.ascontiguousarray(d[k]).tobytes())
print(h.hexdigest())" "$SRC/meshes")"
    NAVCACHE="$SRC/nav_cache/$NAVKEY.bin"
    if [ -s "$NAVCACHE" ]; then
        echo "navmesh inputs unchanged ($NAVKEY): cached navmesh reused"
    else
        NAVTMP="$(mktemp -d)"
        "$MV" "$HERE/osm_city/nav_crosswalks.py" "$SRC/meshes" "$NAVTMP/cw.npz"
        "$MV" "$HERE/osm_city/nav_obj.py" "$SRC/meshes" "$NAVTMP/$NAME.obj" "$NAVTMP/cw.npz"
        (cd "$NAVTMP" && "$RECAST" "$NAME.obj" > recast.log 2>&1) || true
        tail -3 "$NAVTMP/recast.log"
        [ -s "$NAVTMP/$NAME.bin" ] || { echo "navmesh failed (see $NAVTMP/recast.log)"; exit 1; }
        mkdir -p "$SRC/nav_cache"
        cp "$NAVTMP/$NAME.bin" "$NAVCACHE"
    fi
    mkdir -p "$CONTENT/$PKG/Maps/$NAME/Nav"
    cp "$NAVCACHE" "$CONTENT/$PKG/Maps/$NAME/Nav/$NAME.bin"
fi

JPSIG="$("$PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(int(c.get('jp_shingo', c.get('road_style') == 'japan' or c.get('package') == 'Japan')))" "$HERE/maps.json" "$KEY")"
if [ "$JPSIG" = "1" ]; then
    # Japanese traffic signals (guide ch. 24): our own horizontal LED head on a
    # cantilever (osm_city/jp_shingo.py), built into BP_JpShingo, a copy of
    # CARLA's light blueprint, and a TrafficLightManager saved in this level
    # that spawns it; states/controllers/timing stay CARLA's. Other maps keep
    # CARLA's lights. The editor often segfaults on exit after saving, so the
    # log markers decide / 日本の信号機（ガイド 24 章）
    step "2d/4 Japanese traffic signals"
    SHG="$HERE/build/JpShingo"
    # no stale parts: a failed Blender run must not import the old FBX
    # 古い FBX を消してから作る（失敗時に前回分を取り込まない）
    rm -f "$SHG"/SM_JpShingo{Pole,Arm,Head,Lamp}.fbx "$SHG/layout.json"
    blender -b --python "$HERE/osm_city/jp_shingo.py" -- "$SHG" > "$SRC/jp_shingo_blender.log" 2>&1 || true
    SHG_OK=1
    grep -q "^\[jp_shingo\] wrote" "$SRC/jp_shingo_blender.log" || SHG_OK=0
    for f in SM_JpShingoPole.fbx SM_JpShingoArm.fbx SM_JpShingoHead.fbx SM_JpShingoLamp.fbx layout.json; do
        [ -s "$SHG/$f" ] || SHG_OK=0
    done
    [ "$SHG_OK" = 1 ] || { echo "jp_shingo.py failed (see $SRC/jp_shingo_blender.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && SHINGO_SRC="$SHG" SHINGO_PKG="$PKG" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_shingo.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_shingo.log" 2>&1 || true
    grep -q "\[ue_jp_shingo\] done" "$SRC/ue_jp_shingo.log" || { echo "BP_JpShingo step failed (see $SRC/ue_jp_shingo.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && SHINGO_PKG="$PKG" SHINGO_MAP="$NAME" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_shingo_level.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_shingo_level.log" 2>&1 || true
    grep "\[ue_jp_shingo_level\]" "$SRC/ue_jp_shingo_level.log" | tail -1 || true
    grep -q "spawns BP_JpShingo_C" "$SRC/ue_jp_shingo_level.log" || { echo "signal manager step failed (see $SRC/ue_jp_shingo_level.log)"; exit 1; }
fi

JPSOK="$("$PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(int(c.get('jp_sokudo', c.get('road_style') == 'japan' or c.get('package') == 'Japan')))" "$HERE/maps.json" "$KEY")"
if [ "$JPSOK" = "1" ]; then
    # Japanese maximum-speed posts (guide ch. 25): our own 323 plate (white,
    # red ring, blue numerals) on a white pole (osm_city/jp_sokudo.py), built
    # into BP_JpSokudo<V>, copies of CARLA's speed-limit BP, and set in the
    # level's TrafficLightManager.SpeedLimitModels; the speed value and its
    # triggers stay CARLA's (SpawnSignals). Log markers decide, not rc.
    # 日本の最高速度標識（ガイド 25 章）
    step "2e/4 Japanese speed posts"
    SOK="$HERE/build/JpSokudo"
    rm -f "$SOK"/SM_JpSokudo*.fbx "$SOK"/T_JpSokudo*.png "$SOK/layout.json"
    blender -b --python "$HERE/osm_city/jp_sokudo.py" -- "$SOK" > "$SRC/jp_sokudo_blender.log" 2>&1 || true
    grep -q "^\[jp_sokudo\] wrote" "$SRC/jp_sokudo_blender.log" && [ -s "$SOK/layout.json" ] \
        || { echo "jp_sokudo.py failed (see $SRC/jp_sokudo_blender.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && SOKUDO_SRC="$SOK" SOKUDO_PKG="$PKG" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_sokudo.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_sokudo.log" 2>&1 || true
    grep -q "\[ue_jp_sokudo\] done" "$SRC/ue_jp_sokudo.log" || { echo "BP_JpSokudo step failed (see $SRC/ue_jp_sokudo.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && SOKUDO_PKG="$PKG" SOKUDO_MAP="$NAME" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_sokudo_level.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_sokudo_level.log" 2>&1 || true
    grep "\[ue_jp_sokudo_level\]" "$SRC/ue_jp_sokudo_level.log" | tail -1 || true
    grep -q "\[ue_jp_sokudo_level\] speed posts set:" "$SRC/ue_jp_sokudo_level.log" || { echo "speed post manager step failed (see $SRC/ue_jp_sokudo_level.log)"; exit 1; }
fi

# Japanese pedestrian heads (guide ch. 26): the map meshes Terrain_JpHokousha_*
# (osm_city/jp_hokousha.py, one per signalised crossing) get our lamp materials;
# the rig lights them at run time (hil_sim/hokousha.py). Only when the build has
# them / 歩行者用信号機のランプ材質（ガイド 26 章）。メッシュがある場合のみ
JPHOK="$("$PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(int(c.get('jp_hokousha', c.get('road_style') == 'japan' or c.get('package') == 'Japan')))" "$HERE/maps.json" "$KEY")"
if [ "$JPHOK" = "1" ] && ls "$SRC"/meshes/Terrain_JpHokousha_*.npz > /dev/null 2>&1; then
    step "2f/4 Japanese pedestrian heads"
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && HOKOUSHA_PKG="$PKG" HOKOUSHA_MAP="$NAME" TMPDIR="$SRC" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_hokousha.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_hokousha.log" 2>&1 || true
    grep "\[ue_jp_hokousha\]" "$SRC/ue_jp_hokousha.log" | tail -1 || true
    grep -q "\[ue_jp_hokousha\] [1-9][0-9]* pedestrian head meshes" "$SRC/ue_jp_hokousha.log" \
        || { echo "pedestrian head step failed (see $SRC/ue_jp_hokousha.log)"; exit 1; }
fi

GAITO="$("$PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(int(c.get('gaito', True)))" "$HERE/maps.json" "$KEY")"
if [ "$GAITO" = "1" ] && ls "$SRC"/meshes/Terrain_Street_*.npz > /dev/null 2>&1; then
    # Street lamps that light up (guide ch. 28): every props.py lamp head gets a
    # BP_JpGaito (CARLA's street-light BP, our glowing plate, no shadows, drawn
    # to 80 m); the day/night cycle switches them (LightManager, Street group).
    # Generated maps only ("gaito": false opts out). Log markers decide, not rc.
    # 街灯（ガイド 28 章）: 各灯具に BP_JpGaito を置く
    step "2g/4 street lamps"
    "$HERE/.venv/bin/python" "$HERE/osm_city/gaito.py" "$SRC" > "$SRC/gaito.log" 2>&1 || true
    grep -q "^\[gaito\] [1-9][0-9]* lamp heads" "$SRC/gaito.log" || { echo "gaito.py failed (see $SRC/gaito.log)"; exit 1; }
    GTO="$HERE/build/JpGaito"
    blender -b --disable-autoexec --python "$HERE/osm_city/jp_gaito.py" -- "$GTO" > "$SRC/jp_gaito_blender.log" 2>&1 || true
    grep -q "^\[jp_gaito\] wrote" "$SRC/jp_gaito_blender.log" || { echo "jp_gaito.py failed (see $SRC/jp_gaito_blender.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && GAITO_STEP=bp GAITO_SRC="$GTO" GAITO_PKG="$PKG" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_gaito.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_gaito.log" 2>&1 || true
    grep -q "BP_JpGaito saved" "$SRC/ue_jp_gaito.log" || { echo "BP_JpGaito step failed (see $SRC/ue_jp_gaito.log)"; exit 1; }
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && GAITO_STEP=level GAITO_PKG="$PKG" GAITO_MAP="$NAME" GAITO_HEADS="$SRC/gaito.json" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_jp_gaito.py" -unattended -nosplash -nullrhi) \
        > "$SRC/ue_jp_gaito_level.log" 2>&1 || true
    grep "street lamps placed" "$SRC/ue_jp_gaito_level.log" | tail -1 || true
    grep -q "[1-9][0-9]* of [0-9]* street lamps placed" "$SRC/ue_jp_gaito_level.log" \
        || { echo "street lamp level step failed (see $SRC/ue_jp_gaito_level.log)"; exit 1; }
fi

step "3/4 cook"
rm -rf "$COOKED"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/$PKG/Maps/$NAME/$NAME" -cooksinglepackage \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate \
    -ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False)

step "4/4 install into $CARLA_ROOT"
DST="$CARLA_ROOT/CarlaUE4/Content/$PKG"
rm -rf "$DST/Maps/$NAME" "$DST/Static/"*/"$NAME"        # no stale cooked files / 古い cook 結果を残さない
mkdir -p "$DST/Config" "$DST/Maps"
cp -r "$COOKED/CarlaUE4/Content/$PKG/." "$DST/"
# the road logic and navmesh are plain files the cook skips. The TM cache the
# cook makes is NOT installed: on OitaKosen it segfaulted every TM client
# (traffic and the driving window, rc -11) twice; without it TM builds its own
# (map guide ch. 21) / 道路情報とナビメッシュをコピー（TM キャッシュは入れない）
for d in OpenDrive Nav; do
    [ -d "$CONTENT/$PKG/Maps/$NAME/$d" ] && mkdir -p "$DST/Maps/$NAME/$d" && cp -r "$CONTENT/$PKG/Maps/$NAME/$d/." "$DST/Maps/$NAME/$d/"
done
# update Package.json to list all maps in the package / パッケージ内の全マップを登録
"$PY" -c "
import json, os, glob
for cfg_dir in ['$CONTENT/$PKG/Config', '$DST/Config']:
    os.makedirs(cfg_dir, exist_ok=True)
    maps = sorted([os.path.basename(p) for p in glob.glob('$DST/Maps/*') if os.path.isdir(p)])
    data = {'props': [], 'maps': [{'name': m, 'path': f'/Game/$PKG/Maps/{m}', 'use_carla_materials': False} for m in maps]}
    with open(os.path.join(cfg_dir, '$PKG.Package.json'), 'w') as f:
        json.dump(data, f, indent=4)
"
find "$DST/Maps/$NAME" -maxdepth 2 | head -20 || true
echo "done: restart CARLA, then client.load_world('$NAME')"
