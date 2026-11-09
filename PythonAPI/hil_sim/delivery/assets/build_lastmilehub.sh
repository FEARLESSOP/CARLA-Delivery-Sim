#!/usr/bin/env bash
# Build the LastMile Hub LMH-3 (a drivable vehicle that carries three Mini
# Delivery Pods) into its own package, LastMileHubModels, next to the LMP-4
# (LastMileModels) and the PodV2 (DeliveryModels), which stay as they are.
#
#   delivery/assets/build_lastmilehub.sh [model_dir]   build up to the cooked output
#   delivery/assets/build_lastmilehub.sh --install     install the cooked output
#                                                       (CARLA must be STOPPED)
#   model_dir default: ~/Downloads/LastMileHub_FBX_Maya_Unreal (the .glb files are used)
#
# Makes vehicle.delivery.lastmilehub, the props static.prop.lmh* (moving parts
# of the hub, the Mini Pod body and its parts, parcels, charging-dock pieces)
# and delivery/assets/lmh_parts.json (where every part sits and how it moves;
# compose_lmh_parts.py). Needs Blender, the CARLA 0.9.16 source tree with a
# built editor, UE 4.26. Restart CARLA after --install.
#
# VehicleFactory is shared: the source tree's factory lists every vehicle we
# ever added (PodV2, LMP-4, the user's cars); build_vehicle.py appends ours,
# and the cooked factory that --install copies lists them all. Whichever of
# build_cars.sh / build_lastmile.sh / this script installs LAST provides the
# factory, so it is complete as long as each built from the same source tree.
# A factory entry whose package is not installed only fails to spawn.
#
# LastMile Hub LMH-3 を独自パッケージ LastMileHubModels としてビルドする。
# 既定ではクック済み出力まで（インストールは --install、CARLA 停止中のみ）。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CARLA_ROOT="$(cd "$HERE/../../../.." && pwd)"          # packaged CARLA
FACTORY_REL="CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory"
BACKUP="$HERE/backup/packaged"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
export UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
BLENDER="${BLENDER:-blender}"
export LMP_PKG=LastMileHubModels                         # strip_collision.py reads it
PKG=$LMP_PKG
VEH=LastMileHub
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
COOKED_FACTORY="$CARLA_SRC/Dist/${PKG}_factory"
# outside Import/: Import.py would import every JSON there / Import/ の外に置く
STAGE="$CARLA_SRC/Dist/${PKG}_stage"
PY="$CARLA_ROOT/carla-venv/bin/python"   # Import.py imports carla
MINI_JS="${MINI_JS:-$HOME/Downloads/lastmile _site 2/lastmile _site/js_3d_lastmile/mini.js}"
step() { printf '\n== %s\n' "$*"; }

install_cooked() {
    if pgrep -f CarlaUE4-Linux-Shipping >/dev/null; then
        echo "CARLA is running: stop it first (cooked assets of a loaded package can crash it)"; exit 1
    fi
    SRC_PKG="$COOKED/CarlaUE4/Content/$PKG"
    [ -d "$SRC_PKG/Vehicles/$VEH" ] || { echo "no cooked package in $SRC_PKG: build first"; exit 1; }
    [ -f "$COOKED_FACTORY/$FACTORY_REL.uasset" ] || { echo "no cooked VehicleFactory"; exit 1; }
    rm -rf "$CARLA_ROOT/CarlaUE4/Content/$PKG"
    cp -r "$SRC_PKG" "$CARLA_ROOT/CarlaUE4/Content/$PKG"
    mkdir -p "$BACKUP"
    [ -f "$BACKUP/VehicleFactory.uasset" ] || \
        cp -p "$CARLA_ROOT/$FACTORY_REL.uasset" "$CARLA_ROOT/$FACTORY_REL.uexp" "$BACKUP/"
    for ext in uasset uexp; do          # copy then rename / コピー後に置換
        cp "$COOKED_FACTORY/$FACTORY_REL.$ext" "$CARLA_ROOT/$FACTORY_REL.$ext.new"
        mv -f "$CARLA_ROOT/$FACTORY_REL.$ext.new" "$CARLA_ROOT/$FACTORY_REL.$ext"
    done
    grep -c '"name"' "$CARLA_ROOT/CarlaUE4/Content/$PKG/Config/$PKG.Package.json" | xargs echo "props installed:"
    echo "done: start CARLA again (props and vehicles are read at server start)"
}

if [ "${1:-}" = "--install" ]; then
    install_cooked
    exit 0
fi
MODEL_DIR="${1:-$HOME/Downloads/LastMileHub_FBX_Maya_Unreal}"

step "1/9 Blender: vehicle, moving parts, Mini Pod, parcels, dock from $MODEL_DIR"
rm -rf "$STAGE" "$CONTENT/$PKG"          # a clean import every time / 毎回作り直す
mkdir -p "$STAGE"
"$BLENDER" -b --disable-autoexec -P "$HERE/prepare_lastmilehub.py" -- "$MODEL_DIR/LastMileHub.glb" \
    "$MODEL_DIR/LastMileHub_MiniPod.glb" "$MODEL_DIR/LastMileHub_ChargeDock.glb" "$STAGE" \
    "$MODEL_DIR/LastMileHub_Neighbourhood.glb" \
    2>&1 | grep -E "^PROP|^VEHICLE|wrote|WARNING|Error|Traceback" || true
[ -f "$STAGE/lmh_measured.json" ] || { echo "Blender step failed"; exit 1; }
if [ -f "$MINI_JS" ]; then
    python3 -I "$HERE/compose_lmh_parts.py" "$STAGE/lmh_measured.json" "$HERE/lmh_parts.json" "$MINI_JS"
else
    python3 -I "$HERE/compose_lmh_parts.py" "$STAGE/lmh_measured.json" "$HERE/lmh_parts.json"
fi
python3 -I - "$STAGE" "$PKG" <<'EOF'
import json, os, sys
stage, pkg = sys.argv[1:3]
big = {"LMHDockBay"}
props = [{"name": n, "size": "big" if n in big else "small", "source": "./%s.fbx" % n, "tag": "Car"}
         for n in sorted(f[:-4] for f in os.listdir(stage)
                         if f.endswith(".fbx") and f != "LMHVehicle.fbx")]
json.dump({"maps": [], "props": props}, open(os.path.join(stage, pkg + ".json"), "w"), indent=2)
print("%d props" % len(props))
EOF

step "2/9 CARLA import of the props (FBX -> uasset), Import/ untouched"
(cd "$STAGE" && "$PY" "$HERE/import_props_package.py" "$CARLA_SRC" "$STAGE/$PKG.json") 2>&1 | \
    grep -E "Commandlet:|imported|Error" | cut -c1-160 || true
# with bCombineMeshes=0 Unreal names the mesh <file>_<object>: X.X -> X_X.X_X
sed -i -E 's#/([A-Za-z0-9]+)\.\1"#/\1_\1.\1_\1"#' "$CONTENT/$PKG/Config/$PKG.Package.json"

step "3/9 PrepareAssetsForCooking (builds the props map)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" \
    -run=PrepareAssetsForCooking -PackageName="$PKG" -OnlyPrepareMaps=false) >"$STAGE/prepare.log" 2>&1 || true
grep -E "Success|error" "$STAGE/prepare.log" | tail -2 || true
sed -i -E 's#/([A-Za-z0-9]+)\.\1"#/\1_\1.\1_\1"#' "$CONTENT/$PKG/Config/$PKG.Package.json"

step "4/9 strip collision (undoes the step above's complex-as-simple)"
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/strip_collision.py" 2>&1 \
    | grep -E "strip_collision" | tail -2 || true

step "5/9 vehicle: import on the Prius skeleton, blueprints, VehicleFactory entry"
rm -f "$HERE/build_lastmilehub_vehicle.log"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && POD_VEHICLE_FBX="$STAGE/LMHVehicle.fbx" \
    VEH_NAME=$VEH VEH_DEST="/Game/$PKG/Vehicles/$VEH" VEH_MAKE=delivery VEH_MODEL=lastmilehub \
    VEH_WHEEL_R_CM=28 VEH_WHEEL_W_CM=19 VEH_MASS=1500 VEH_BOUNDS=163.5,75,104.5,104.5 \
    VEH_COLOR=230,234,238 VEH_TEXTURES=1 VEH_LOG=build_lastmilehub_vehicle.log \
    "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/build_vehicle.py" >/dev/null 2>&1) || true
cat "$HERE/build_lastmilehub_vehicle.log"
tail -1 "$HERE/build_lastmilehub_vehicle.log" | grep -q '^OK$' || { echo "vehicle build failed"; exit 1; }

step "6/9 materials: PBR from the glTF (glass translucent, LEDs emissive)"
rm -f "$HERE/lmh_materials.log"
LMH_PKG=$PKG LMH_SPEC="$STAGE/lmh_materials.json" \
    "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/lmh_materials.py" >/dev/null 2>&1 || true
cat "$HERE/lmh_materials.log"
tail -1 "$HERE/lmh_materials.log" | grep -q '^OK$' || { echo "materials step failed"; exit 1; }

step "7/9 cook the props, every prop mesh and the vehicle (with what they use)"
rm -rf "$COOKED" "$COOKED_FACTORY"
MAPS="$(cat "$CONTENT/MapPathsLinux.txt")+/Game/$PKG/Vehicles/$VEH/BP_$VEH"
# each prop explicitly: the props map alone once left one out (car guide ch. 6)
for f in "$STAGE"/LMH*.fbx; do
    n="$(basename "$f" .fbx)"; [ "$n" = LMHVehicle ] && continue
    MAPS="$MAPS+/Game/$PKG/Static/Car/$n/${n}_$n"
done
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook -map="$MAPS" \
    -cooksinglepackage -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate) \
    >"$STAGE/cook.log" 2>&1 || true
grep -E "Success - |Failure - " "$STAGE/cook.log" | tail -1
rm -rf "$CONTENT/$PKG/Maps/PropsMap"

step "8/9 cook VehicleFactory alone (every vehicle entry, not the cars it lists)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/Carla/Blueprints/Vehicles/VehicleFactory" -cooksinglepackagenorefs \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED_FACTORY") >"$STAGE/cook_factory.log" 2>&1 || true
grep -E "Success - |Failure - " "$STAGE/cook_factory.log" | tail -1

step "9/9 check the cooked output against lmh_parts.json (no install)"
SRC_PKG="$COOKED/CarlaUE4/Content/$PKG"
mkdir -p "$SRC_PKG/Config"
cp "$CONTENT/$PKG/Config/$PKG.Package.json" "$SRC_PKG/Config/"
rm -rf "$SRC_PKG/Maps"
python3 -I - "$SRC_PKG" "$PKG" "$HERE/lmh_parts.json" "$VEH" "$COOKED_FACTORY/$FACTORY_REL.uasset" <<'EOF'
import json, os, sys
src, pkg, parts, veh, factory = sys.argv[1:6]
cfg = json.load(open(os.path.join(src, "Config", pkg + ".Package.json")))
names = {p["name"].lower(): p["path"] for p in cfg["props"]}
bad = []
for p in cfg["props"]:
    rel = p["path"].split(".")[0].replace("/Game/%s/" % pkg, "")
    for ext in (".uasset", ".uexp"):
        if not os.path.isfile(os.path.join(src, rel + ext)):
            bad.append(rel + ext)
want = set()
def walk(o):
    if isinstance(o, dict):
        for k, v in o.items():
            if isinstance(v, str) and v.startswith("static.prop."):
                want.add(v[len("static.prop."):])
            walk(v)
    elif isinstance(o, list):
        for v in o:
            walk(v)
walk(json.load(open(parts)))
missing = sorted(want - set(names))
vdir = os.path.join(src, "Vehicles", veh)
if not os.path.isfile(os.path.join(vdir, "BP_%s.uasset" % veh)):
    bad.append("BP_%s" % veh)
fac = open(factory, "rb").read()
print("props in package: %d, referenced by lmh_parts.json: %d, missing: %s" % (len(names), len(want), missing))
print("cooked files missing: %s" % (bad or "none"))
print("vehicle assets cooked: %d" % len([f for f in os.listdir(vdir) if f.endswith(".uasset")]))
print("factory lists BP_%s: %s" % (veh, (b"BP_" + veh.encode()) in fac))
sys.exit(1 if missing or bad or (b"BP_" + veh.encode()) not in fac else 0)
EOF
cat <<EOF

Cooked, NOT installed. With CARLA stopped, install with:
  bash "$HERE/build_lastmilehub.sh" --install
It replaces $CARLA_ROOT/CarlaUE4/Content/$PKG (new) and the packaged
VehicleFactory (backup in $BACKUP); then start CARLA again.
EOF
