#!/usr/bin/env bash
# Build the see-through zone-box panels (/Game/ZoneTint) in the CARLA source
# copy, cook them and install only Content/ZoneTint into the packaged CARLA.
#   maps/build_zone_faces.sh
# ゾーン箱の半透明パネルを作り、クックして ZoneTint だけをインストールする。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CARLA_ROOT="$(cd "$HERE/../../.." && pwd)"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
COOKED="$CARLA_SRC/Dist/ZoneTint_cooked"
COLOURS="Red Cyan Yellow Magenta Lime Orange Sky Mint Violet"

echo "== 1/3 make the panels"
"${BLENDER:-blender}" -b -P "$HERE/zone_faces_blender.py" -- "$HERE/build/zone_faces" >/dev/null
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/ue_zone_faces.py" >/dev/null 2>&1 || true
cat "$HERE/ue_zone_faces.log"
tail -1 "$HERE/ue_zone_faces.log" | grep -q '^OK$' || { echo "panel build failed"; exit 1; }

echo "== 2/3 cook"
rm -rf "$COOKED"
PKGS=""
for c in $COLOURS; do for k in 1 2 4 8 16 32; do PKGS="$PKGS+/Game/ZoneTint/ZoneTint_${c}_$k"; done; done
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook -map="${PKGS#+}" \
    -cooksinglepackage -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate) \
    | grep -E "Success|Error" | tail -5

echo "== 3/3 install"
rm -rf "$CARLA_ROOT/CarlaUE4/Content/ZoneTint"
cp -r "$COOKED/CarlaUE4/Content/ZoneTint" "$CARLA_ROOT/CarlaUE4/Content/ZoneTint"
ls "$CARLA_ROOT/CarlaUE4/Content/ZoneTint"
