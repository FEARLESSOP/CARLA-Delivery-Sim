#!/usr/bin/env bash
# Build the LastMile pod (LMP-4, from LastMile_Pod_3D.html's Unreal export) and
# install it into the packaged CARLA next to the PodV2 pod, which stays
# installed: delivery_sim.py --pod-model pod goes back to it.
#
#   delivery/assets/build_lastmile.sh [model_dir]   (default ~/Downloads/unreal/unreal)
#
# Makes the package LastMileModels (props static.prop.lmp*, X-ray shell copies)
# and the vehicle vehicle.delivery.lastmile; CARLA's VehicleFactory then lists
# both pods. build_models.sh --restore-factory puts the original factory back.
# Needs Blender, the CARLA 0.9.16 source tree with a built editor, UE 4.26.
# Restart CARLA afterwards (props and vehicles are read at server start).
#
# LastMile ポッドをビルドし、PodV2 ポッドと並べてパッケージ版 CARLA に入れる
# （--pod-model pod で元に戻せる）。終了後に CARLA を再起動すること。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CARLA_ROOT="$(cd "$HERE/../../../.." && pwd)"          # packaged CARLA
FACTORY_REL="CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory"
BACKUP="$HERE/backup/packaged"
MODEL_DIR="${1:-$HOME/Downloads/unreal/unreal}"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
export UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
BLENDER="${BLENDER:-blender}"
export LMP_PKG=LastMileModels
PKG=$LMP_PKG
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
COOKED_FACTORY="$CARLA_SRC/Dist/${PKG}_factory"
PY="$CARLA_ROOT/carla-venv/bin/python"   # Import.py imports carla
STAGE="$CARLA_SRC/Import/$PKG"

step() { printf '\n== %s\n' "$*"; }

step "1/9 Blender: vehicle, shell and moving parts from $MODEL_DIR"
rm -rf "$STAGE" "$CONTENT/$PKG"          # a clean import every time / 毎回作り直す
mkdir -p "$STAGE"
"$BLENDER" -b -P "$HERE/prepare_lastmile.py" -- "$MODEL_DIR/LastMilePod.fbx" \
    "$MODEL_DIR/LastMilePod_Drone.fbx" "$MODEL_DIR/LastMilePod_BoxStation.fbx" "$STAGE" \
    2>&1 | grep -E "size |LMPVehicle|wrote|Error|Traceback" || true
[ -f "$STAGE/lmp_parts.json" ] || { echo "Blender step failed"; exit 1; }
cp "$STAGE/lmp_parts.json" "$HERE/../lmp_parts.json"      # read by delivery/lmp_rig.py
"$PY" - "$STAGE" "$PKG" <<'EOF'
import json, os, sys
stage, pkg = sys.argv[1:3]
big = {"LMPShell", "LMPShellStreet", "LMPStation", "LMPHoistBridge", "LMPBatteryMagazine",
       "LMPLiftPad", "LMPShuttleMast"}
props = [{"name": n, "size": "big" if n in big else "small", "source": "./%s.fbx" % n, "tag": "Car"}
         for n in sorted(f[:-4] for f in os.listdir(stage)
                         if f.endswith(".fbx") and f != "LMPVehicle.fbx")]
json.dump({"maps": [], "props": props}, open(os.path.join(stage, pkg + ".json"), "w"), indent=2)
print("%d props" % len(props))
EOF

step "2/9 CARLA import of the props (FBX -> uasset)"
(cd "$CARLA_SRC" && "$PY" Util/BuildTools/Import.py --package "$PKG")
# with bCombineMeshes=0 Unreal names the mesh <file>_<object>: X.X -> X_X.X_X
sed -i -E 's#/([A-Za-z0-9]+)\.\1"#/\1_\1.\1_\1"#' "$CONTENT/$PKG/Config/$PKG.Package.json"

step "3/9 X-ray shell copies (translucent material, extra props)"
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/make_xray.py" >/dev/null 2>&1 || true
cat "$HERE/make_xray.log"
tail -1 "$HERE/make_xray.log" | grep -q '^OK$' || { echo "x-ray step failed"; exit 1; }

step "4/9 PrepareAssetsForCooking (builds the props map)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" \
    -run=PrepareAssetsForCooking -PackageName="$PKG" -OnlyPrepareMaps=false)

step "5/9 strip collision (undoes the step above's complex-as-simple)"
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/strip_collision.py" 2>&1 \
    | grep -E "strip_collision|Error" | tail -3 || true

step "6/9 vehicle: import on the Prius skeleton, blueprints, VehicleFactory entry"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && POD_VEHICLE_FBX="$STAGE/LMPVehicle.fbx" \
    VEH_NAME=LastMilePod VEH_DEST="/Game/$PKG/Vehicles/LastMilePod" VEH_MODEL=lastmile \
    VEH_TEXTURES=1 VEH_LOG=build_lastmile_vehicle.log \
    "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/build_vehicle.py" >/dev/null 2>&1) || true
cat "$HERE/build_lastmile_vehicle.log"
tail -1 "$HERE/build_lastmile_vehicle.log" | grep -q '^OK$' || { echo "vehicle build failed"; exit 1; }

step "7/9 cook the props and the vehicle (with what they use)"
rm -rf "$COOKED" "$COOKED_FACTORY"
MAPS="$(cat "$CONTENT/MapPathsLinux.txt")+/Game/$PKG/Vehicles/LastMilePod/BP_LastMilePod"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook -map="$MAPS" \
    -cooksinglepackage -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate)
rm -rf "$CONTENT/$PKG/Maps/PropsMap"

step "8/9 cook VehicleFactory alone (both pods, not the cars it lists)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/Carla/Blueprints/Vehicles/VehicleFactory" -cooksinglepackagenorefs \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED_FACTORY")

step "9/9 install into $CARLA_ROOT"
SRC_PKG="$COOKED/CarlaUE4/Content/$PKG"
mkdir -p "$SRC_PKG/Config"
cp "$CONTENT/$PKG/Config/$PKG.Package.json" "$SRC_PKG/Config/"
rm -rf "$SRC_PKG/Maps"
rm -rf "$CARLA_ROOT/CarlaUE4/Content/$PKG"
cp -r "$SRC_PKG" "$CARLA_ROOT/CarlaUE4/Content/$PKG"
mkdir -p "$BACKUP"
if [ ! -f "$BACKUP/VehicleFactory.uasset" ]; then
    cp -p "$CARLA_ROOT/$FACTORY_REL.uasset" "$CARLA_ROOT/$FACTORY_REL.uexp" "$BACKUP/"
fi
# copy then rename: a running CARLA keeps reading the old file instead of a
# half-written one / 実行中の CARLA が書きかけを読まないようコピー後に置換
for ext in uasset uexp; do
    cp "$COOKED_FACTORY/$FACTORY_REL.$ext" "$CARLA_ROOT/$FACTORY_REL.$ext.new"
    mv -f "$CARLA_ROOT/$FACTORY_REL.$ext.new" "$CARLA_ROOT/$FACTORY_REL.$ext"
done
grep -c '"name"' "$CARLA_ROOT/CarlaUE4/Content/$PKG/Config/$PKG.Package.json" | xargs echo "props installed:"
ls "$CARLA_ROOT/CarlaUE4/Content/$PKG/Vehicles/LastMilePod" | grep -c uasset | \
    xargs echo "vehicle assets installed:"
echo "done: restart CARLA (props and vehicles are read at server start)"
