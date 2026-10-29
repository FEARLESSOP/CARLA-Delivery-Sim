#!/usr/bin/env bash
# Build your own cars (cars.json) into the packaged CARLA: each becomes a
# vehicle (vehicle.<make>.<model>) plus a steering-wheel prop the driving
# script turns with the real wheel. Restart CARLA afterwards.
#
#   cars/build_cars.sh [car key ...]      (default: every car in cars.json)
#
# Needs Blender, the CARLA 0.9.16 source tree with a built editor, UE 4.26.
# Same steps as delivery/assets/build_lastmile.sh; the pods stay installed.
#
# cars.json の車両をパッケージ版 CARLA に入れる（車両＋ハンドル用プロップ）。
# 終了後に CARLA を再起動すること。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ASSETS="$HERE/../delivery/assets"
CARLA_ROOT="$(cd "$HERE/../../.." && pwd)"
FACTORY_REL="CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
export UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"
export LMP_PKG=UserCars
PKG=$LMP_PKG
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
UPROJECT="$CARLA_SRC/Unreal/CarlaUE4/CarlaUE4.uproject"
CONTENT="$CARLA_SRC/Unreal/CarlaUE4/Content"
COOKED="$CARLA_SRC/Dist/${PKG}_cooked"
COOKED_FACTORY="$CARLA_SRC/Dist/${PKG}_factory"
STAGE="$CARLA_SRC/Import/$PKG"
# vehicle FBXs and per-car data, outside Import/: Import.py imports EVERY
# .json under Import/ as a package, whatever --package says
# 車両 FBX と車両データは Import/ の外へ（Import.py は Import/ 内の全 json を取り込む）
VEHSTAGE="$CARLA_SRC/Dist/${PKG}_vehicles"
PARK="$CARLA_SRC/Import_parked"
PY="$CARLA_ROOT/carla-venv/bin/python"
step() { printf '\n== %s\n' "$*"; }

KEYS=("$@")
[ ${#KEYS[@]} -gt 0 ] || mapfile -t KEYS < <("$PY" -c "import json,sys; print('\n'.join(k for k in json.load(open(sys.argv[1])) if not k.startswith('_')))" "$HERE/cars.json")
echo "cars: ${KEYS[*]}"

step "1/8 Blender: vehicle + steering wheel per car"
rm -rf "$STAGE" "$VEHSTAGE" "$CONTENT/$PKG"   # textures go to $STAGE/<Name>_textures
mkdir -p "$STAGE" "$VEHSTAGE"
for k in "${KEYS[@]}"; do
    "${BLENDER:-blender}" -b -P "$HERE/prepare_car.py" -- "$HERE/cars.json" "$k" "$STAGE" \
        2>&1 | grep -E "^CAR|Error|SystemExit" | cut -c1-200 || true
    [ -f "$STAGE/$k.json" ] || { echo "Blender step failed for $k"; exit 1; }
    mv "$STAGE/$k.json" "$VEHSTAGE/"
done
mv "$STAGE"/*Vehicle.fbx "$VEHSTAGE/"
"$PY" - "$STAGE" "$PKG" <<'PYEOF'
import json, os, sys
stage, pkg = sys.argv[1:3]
props = [{"name": f[:-4], "size": "small", "source": "./" + f, "tag": "Car"}
         for f in sorted(os.listdir(stage)) if f.endswith("Steer.fbx")]
json.dump({"maps": [], "props": props}, open(os.path.join(stage, pkg + ".json"), "w"), indent=2)
print("%d steering-wheel props" % len(props))
PYEOF

step "2/8 import the steering-wheel props"
# only our package in Import/ while it runs; the others come back afterwards
# 取り込み中は Import/ に自分のパッケージだけを置き、終了後に戻す
mkdir -p "$PARK"
restore() { [ -d "$PARK" ] && find "$PARK" -mindepth 1 -maxdepth 1 -exec mv -t "$CARLA_SRC/Import/" {} + 2>/dev/null; rmdir "$PARK" 2>/dev/null || true; }
trap restore EXIT
find "$CARLA_SRC/Import" -mindepth 1 -maxdepth 1 ! -name "$PKG" ! -name README.md -exec mv -t "$PARK/" {} +
(cd "$CARLA_SRC" && "$PY" Util/BuildTools/Import.py --package "$PKG")
restore
sed -i -E 's#/([A-Za-z0-9]+)\.\1"#/\1_\1.\1_\1"#' "$CONTENT/$PKG/Config/$PKG.Package.json"

step "3/8 PrepareAssetsForCooking + strip collision"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=PrepareAssetsForCooking \
    -PackageName="$PKG" -OnlyPrepareMaps=false)
"$EDITOR" "$UPROJECT" -run=pythonscript -script="$ASSETS/strip_collision.py" 2>&1 \
    | grep -E "strip_collision" | tail -2 || true

step "4/8 vehicles: import on the Prius skeleton, blueprints, VehicleFactory entries"
BPS=""
for k in "${KEYS[@]}"; do
    eval "$("$PY" - "$VEHSTAGE/$k.json" <<'PYEOF'
import json, sys
c = json.load(open(sys.argv[1]))
_, make, model = c["blueprint"].split(".")
e = c["bounds_extent_m"]
print('NAME=%s MAKE=%s MODEL=%s R=%.2f W=%.2f MASS=%.0f BOUNDS=%.1f,%.1f,%.1f,%.1f' % (
    c["name"], make, model, c["wheel_radius_m"] * 100, c["tyre_width_m"] * 100, c["mass_kg"],
    e[0] * 100, e[1] * 100, e[2] * 100, c["bounds_centre_m"][2] * 100))
PYEOF
)"
    (cd "$CARLA_SRC/Unreal/CarlaUE4" && POD_VEHICLE_FBX="$VEHSTAGE/${NAME}Vehicle.fbx" \
        VEH_NAME="$NAME" VEH_DEST="/Game/$PKG/Vehicles/$NAME" VEH_MAKE="$MAKE" VEH_MODEL="$MODEL" \
        VEH_WHEEL_R_CM="$R" VEH_WHEEL_W_CM="$W" VEH_MASS="$MASS" VEH_BOUNDS="$BOUNDS" \
        VEH_TEXTURES=1 VEH_LOG="build_car_$k.log" \
        "$EDITOR" "$UPROJECT" -run=pythonscript -script="$ASSETS/build_vehicle.py" >/dev/null 2>&1) || true
    cat "$ASSETS/build_car_$k.log"
    tail -1 "$ASSETS/build_car_$k.log" | grep -q '^OK$' || { echo "vehicle build failed: $k"; exit 1; }
    GLASS="$("$PY" -c "import json,sys; print(','.join(json.load(open(sys.argv[1])).get('glass', [])))" "$VEHSTAGE/$k.json")"
    if [ -n "$GLASS" ]; then          # see-through windows / 窓を半透明に
        CAR_DIR="/Game/$PKG/Vehicles/$NAME" CAR_GLASS="$GLASS" CAR_GLASS_LOG="make_glass_$k.log" \
            "$EDITOR" "$UPROJECT" -run=pythonscript -script="$HERE/make_glass.py" >/dev/null 2>&1 || true
        cat "$HERE/make_glass_$k.log"
        tail -1 "$HERE/make_glass_$k.log" | grep -q '^OK$' || { echo "glass failed: $k"; exit 1; }
    fi
    # the steering-wheel prop too: the props map alone left one out (Soarer)
    # ハンドルも明示的に cook する（props map だけでは漏れることがある）
    BPS="$BPS+/Game/$PKG/Vehicles/$NAME/BP_$NAME+/Game/$PKG/Static/Car/${NAME}Steer/${NAME}Steer_${NAME}Steer"
done

step "5/8 cook the props and the cars"
rm -rf "$COOKED" "$COOKED_FACTORY"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="$(cat "$CONTENT/MapPathsLinux.txt")$BPS" -cooksinglepackage \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED" -iterate)
rm -rf "$CONTENT/$PKG/Maps/PropsMap"

step "6/8 cook VehicleFactory alone"
(cd "$CARLA_SRC/Unreal/CarlaUE4" && "$EDITOR" "$UPROJECT" -run=cook \
    -map="/Game/Carla/Blueprints/Vehicles/VehicleFactory" -cooksinglepackagenorefs \
    -targetplatform="LinuxNoEditor" -OutputDir="$COOKED_FACTORY")

step "7/8 install into $CARLA_ROOT"
SRC_PKG="$COOKED/CarlaUE4/Content/$PKG"
mkdir -p "$SRC_PKG/Config"
cp "$CONTENT/$PKG/Config/$PKG.Package.json" "$SRC_PKG/Config/"
rm -rf "$SRC_PKG/Maps" "$CARLA_ROOT/CarlaUE4/Content/$PKG"
cp -r "$SRC_PKG" "$CARLA_ROOT/CarlaUE4/Content/$PKG"
BACKUP="$ASSETS/backup/packaged"
mkdir -p "$BACKUP"
[ -f "$BACKUP/VehicleFactory.uasset" ] || cp -p "$CARLA_ROOT/$FACTORY_REL.uasset" "$CARLA_ROOT/$FACTORY_REL.uexp" "$BACKUP/"
for ext in uasset uexp; do        # copy then rename / コピー後に置換
    cp "$COOKED_FACTORY/$FACTORY_REL.$ext" "$CARLA_ROOT/$FACTORY_REL.$ext.new"
    mv -f "$CARLA_ROOT/$FACTORY_REL.$ext.new" "$CARLA_ROOT/$FACTORY_REL.$ext"
done

step "8/8 camera / steering-wheel data -> cars/car_views.json"
"$PY" - "$VEHSTAGE" "$HERE/car_views.json" "${KEYS[@]}" <<'PYEOF'
import json, os, sys
stage, out, keys = sys.argv[1], sys.argv[2], sys.argv[3:]
views = json.load(open(out)) if os.path.exists(out) else {}
for k in keys:
    c = json.load(open(os.path.join(stage, k + ".json")))
    old = views.get(c["blueprint"], {})
    # keep a driver eye you already tuned / 調整済みの視点は残す
    if "driver_eye" in old and old.get("eye_tuned"):
        c["driver_eye"], c["eye_tuned"] = old["driver_eye"], True
    if "cameras" in old:
        c["cameras"] = old["cameras"]
    if "driver_eye_pitch" in old:
        c["driver_eye_pitch"] = old["driver_eye_pitch"]
    if "driver_eye_yaw" in old:
        c["driver_eye_yaw"] = old["driver_eye_yaw"]
    views[c["blueprint"]] = c
json.dump(views, open(out, "w"), indent=2)
print("car_views.json:", ", ".join(sorted(views)))
PYEOF
echo "done: restart CARLA (vehicles and props are read at server start)"
