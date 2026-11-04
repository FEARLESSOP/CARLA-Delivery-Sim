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
# the road logic (and nav / TM caches if made) are plain files the cook skips
# 道路情報などの通常ファイルは cook されないのでコピーする
for d in OpenDrive Nav TM; do
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
