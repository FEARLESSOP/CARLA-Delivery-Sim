#!/usr/bin/env bash
# Build the delivery pod / drone props and the drivable pod vehicle from FBX
# and install them into the packaged CARLA. Needs Blender, a CARLA 0.9.16
# *source* tree with a built editor, and UE 4.26. Run again whenever the FBX
# files change, then restart CARLA (props and vehicles are read at start).
#
#   delivery/assets/build_models.sh [fbx_dir]        (default ~/Downloads)
#   delivery/assets/build_models.sh --restore-factory
#
# Uses PodV2.fbx when it is there (side-port door and arm), else Pod.fbx.
# The pod vehicle (vehicle.delivery.pod) needs an entry in CARLA's own
# VehicleFactory, so the packaged VehicleFactory.uasset/.uexp is replaced; the
# original is kept in backup/ and --restore-factory puts it back. Otherwise
# only Content/DeliveryModels is installed: the cook also produces ~2300
# dependency files from the source tree, which must not overwrite the
# packaged build's own content.
#
# Env overrides: CARLA_SRC (source tree), UE4_ROOT, BLENDER.
#
# 配送ポッド／ドローンのプロップと走行可能なポッド車両を FBX からビルドし、
# パッケージ版 CARLA に入れる。車両登録のため VehicleFactory を差し替える
# （元は backup/ に保存、--restore-factory で戻す）。FBX を変えたら再実行し、
# CARLA を再起動すること。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CARLA_ROOT="$(cd "$HERE/../../../.." && pwd)"          # packaged CARLA
FACTORY_REL="CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory"
BACKUP="$HERE/backup/packaged"

if [ "${1:-}" = "--restore-factory" ]; then
    cp -p "$BACKUP/VehicleFactory.uasset" "$BACKUP/VehicleFactory.uexp" \
        "$(dirname "$CARLA_ROOT/$FACTORY_REL")/"
    echo "restored the original VehicleFactory; restart CARLA"
    exit 0
fi

FBX_DIR="${1:-$HOME/Downloads}"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
export UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
BLENDER="${BLENDER:-blender}"
PKG=DeliveryModels
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
COOKED_FACTORY="$CARLA_SRC/Dist/${PKG}_factory"
PY="$CARLA_ROOT/carla-venv/bin/python"   # Import.py imports carla
POD_FBX="$FBX_DIR/PodV2.fbx"
[ -f "$POD_FBX" ] || POD_FBX="$FBX_DIR/Pod.fbx"

step() { printf '\n== %s\n' "$*"; }

step "1/8 Blender: props and the rigged pod vehicle from $(basename "$POD_FBX")"
STAGE="$CARLA_SRC/Import/$PKG"
rm -rf "$STAGE"
mkdir -p "$STAGE"
"$BLENDER" -b -P "$HERE/prepare_models.py" -- \
    "$POD_FBX" "$FBX_DIR/Drone.fbx" "$STAGE" 2>&1 | grep -E "size|bone at|PodVehicle|Error" || true
SIDE=""
if [ -f "$STAGE/PodSideDoor.fbx" ]; then
    SIDE=',
    {"name": "PodSideDoor",     "size": "tiny", "source": "./PodSideDoor.fbx",     "tag": "Car"},
    {"name": "PodSlideStage1",  "size": "tiny", "source": "./PodSlideStage1.fbx",  "tag": "Car"},
    {"name": "PodSlideStage2",  "size": "tiny", "source": "./PodSlideStage2.fbx",  "tag": "Car"},
    {"name": "PodSlideStage3",  "size": "tiny", "source": "./PodSlideStage3.fbx",  "tag": "Car"},
    {"name": "PodSlideCarrier", "size": "tiny", "source": "./PodSlideCarrier.fbx", "tag": "Car"}'
fi
cat > "$STAGE/$PKG.json" <<EOF
{
  "maps": [],
  "props": [
    {"name": "DeliveryPod",   "size": "big",    "source": "./Pod.fbx",           "tag": "Car"},
    {"name": "PodJack",       "size": "tiny",   "source": "./PodJack.fbx",       "tag": "Car"},
    {"name": "PodHatchFront", "size": "small",  "source": "./PodHatchFront.fbx", "tag": "Car"},
    {"name": "PodHatchRear",  "size": "small",  "source": "./PodHatchRear.fbx",  "tag": "Car"},
    {"name": "DeliveryDrone", "size": "small",  "source": "./Drone.fbx",         "tag": "Dynamic"},
    {"name": "DroneRotor",    "size": "tiny",   "source": "./DroneRotor.fbx",    "tag": "Dynamic"},
    {"name": "Parcel",        "size": "tiny",   "source": "./Parcel.fbx",        "tag": "Dynamic"}$SIDE
  ]
}
EOF

step "2/8 CARLA import of the props (FBX -> uasset)"
(cd "$CARLA_SRC" && "$PY" Util/BuildTools/Import.py --package "$PKG")
# with bCombineMeshes=0 Unreal names the mesh <file>_<object>: X.X -> X_X.X_X
sed -i -E 's#/([A-Za-z0-9]+)\.\1"#/\1_\1.\1_\1"#' "$CONTENT/$PKG/Config/$PKG.Package.json"

step "3/8 PrepareAssetsForCooking (builds the props map)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" \
    -run=PrepareAssetsForCooking -PackageName="$PKG" -OnlyPrepareMaps=false)

step "4/8 strip collision (undoes the step above's complex-as-simple)"
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/strip_collision.py" 2>&1 \
    | grep -E "strip_collision|Error" || true

step "5/8 pod vehicle: import on the Prius skeleton, blueprints, VehicleFactory entry"
# the editor tends to crash on exit after this script; its log says whether it worked
# このスクリプトの後エディタが終了時に落ちることがあるため、ログで成否を判断する
(cd "$CARLA_SRC/Unreal/CarlaUE4" && POD_VEHICLE_FBX="$STAGE/PodVehicle.fbx" "$EDITOR" "$UPROJECT" \
    -run=pythonscript -script="$HERE/build_vehicle.py" >/dev/null 2>&1) || true
cat "$HERE/build_vehicle.log"
tail -1 "$HERE/build_vehicle.log" | grep -q '^OK$' || { echo "vehicle build failed"; exit 1; }

step "6/8 cook the props and the vehicle (with what they use)"
rm -rf "$COOKED" "$COOKED_FACTORY"
MAPS="$(cat "$CONTENT/MapPathsLinux.txt")+/Game/$PKG/Vehicles/DeliveryPod/BP_DeliveryPod"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook -map="$MAPS" \
    -cooksinglepackage -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate)
rm -rf "$CONTENT/$PKG/Maps/PropsMap"

step "7/8 cook VehicleFactory alone (not the 41 cars it lists)"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/Carla/Blueprints/Vehicles/VehicleFactory" -cooksinglepackagenorefs \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED_FACTORY")

step "8/8 install into $CARLA_ROOT"
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
cp "$COOKED_FACTORY/$FACTORY_REL.uasset" "$COOKED_FACTORY/$FACTORY_REL.uexp" \
    "$(dirname "$CARLA_ROOT/$FACTORY_REL")/"
grep '"name"' "$CARLA_ROOT/CarlaUE4/Content/$PKG/Config/$PKG.Package.json"
ls "$CARLA_ROOT/CarlaUE4/Content/$PKG/Vehicles/DeliveryPod" | grep -c uasset | \
    xargs echo "vehicle assets installed:"
echo "done: restart CARLA (props and vehicles are read at server start)"
