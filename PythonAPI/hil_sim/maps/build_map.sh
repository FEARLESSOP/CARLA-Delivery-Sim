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
UPROJECT="$CARLA_SRC/Unreal/CarlaUnreal/CarlaUnreal.uproject"
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
STAGE="$CARLA_SRC/Import/$PKG"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
PARK="$CARLA_SRC/Import_parked"
step() { printf '\n== %s\n' "$*"; }

step "1/4 stage $NAME ($PKG)"
mkdir -p "$STAGE" "$CONTENT/$PKG/Maps"
rm -rf "$STAGE/$NAME".* "$CONTENT/$PKG/Maps/$NAME"
cp "$SRC/$NAME.fbx" "$SRC/$NAME.xodr" "$STAGE/"
cat > "$STAGE/$PKG.json" <<JSON
{"maps": [{"name": "$NAME", "source": "./$NAME.fbx", "use_carla_materials": false, "xodr": "./$NAME.xodr"}], "props": []}
JSON

step "2/4 import (only this package in Import/; Import.py takes every .json there)"
mkdir -p "$PARK"
ENGINE_INI="${CARLA_SRC}/Unreal/CarlaUnreal/Config/DefaultEngine.ini"
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
(cd "$CARLA_SRC" && "$PY" Util/Tools/Import.py --package "$PKG")
unpark
ls "$CONTENT/$PKG/Maps/$NAME/" || { echo "import failed: no map"; exit 1; }

step "3/4 cook"
rm -rf "$COOKED"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/$PKG/Maps/$NAME/$NAME" -cooksinglepackage \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate \
    -ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False)

step "4/4 install into $CARLA_ROOT"
DST="$CARLA_ROOT/CarlaUE4/Content/$PKG"
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
