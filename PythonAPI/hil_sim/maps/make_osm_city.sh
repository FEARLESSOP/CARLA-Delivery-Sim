#!/usr/bin/env bash
# Build a city map from an OpenStreetMap file into CARLA, end to end:
# OSM -> road logic (with a ring road) -> lanes -> 3-D model (roads, kerbs,
# markings, buildings, trees) -> FBX -> import / cook / install.
#
#   maps/make_osm_city.sh <map key in maps.json> [--lefthand] [--no-import]
#
# The maps.json entry needs "name", "package" and "osm" (the .osm file; its
# <bounds> are the map's extent); optional "terrain_glb" (topoexport GLB, or a
# list: the main export then add-ons covering more ground)
# makes a hilly map (osm_city/terrain.py, elevate.py). Preview renders go to
# maps/build/<Name>/previews. Restart CARLA afterwards.
# OSM ファイルから都市マップを作り CARLA に入れる。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KEY="$1"; shift
LEFT=""; IMPORT=1
for a in "$@"; do
    case "$a" in
        --lefthand) LEFT="--lefthand" ;;
        --no-import) IMPORT=0 ;;
    esac
done
CARLA_PY="$HERE/../../../carla-venv/bin/python"
VENV="$HERE/.venv"
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
"$VENV/bin/python" -c "import shapely, mapbox_earcut, PIL, numpy, scipy" 2>/dev/null || \
    "$VENV/bin/pip" install -q numpy pillow shapely mapbox_earcut scipy
NAME="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]]['name'])" "$HERE/maps.json" "$KEY")"
OSM="$(python3 -c "import json,sys,os; print(os.path.expanduser(json.load(open(sys.argv[1]))[sys.argv[2]]['osm']))" "$HERE/maps.json" "$KEY")"
# terrain_glb: one path or a list (main export, then add-ons) / 1 つまたはリスト
mapfile -d '' GLBS < <(python3 -c "
import json,sys,os
g=json.load(open(sys.argv[1]))[sys.argv[2]].get('terrain_glb') or []
for p in ([g] if isinstance(g,str) else g): sys.stdout.write(os.path.expanduser(p)+'\0')" "$HERE/maps.json" "$KEY")
OUT="$HERE/build/$NAME"
mkdir -p "$OUT"
echo "== road logic"
# maps.json "road_fixes": dual-carriageway lanes, near-miss snapping, medians, bus bays
# 分離道路の車線数・行き止まり接続・中央分離帯・バスベイ（地図ごとに指定）
FIXES="$(python3 -c "import json,sys; print('--road-fixes' if json.load(open(sys.argv[1]))[sys.argv[2]].get('road_fixes') else '')" "$HERE/maps.json" "$KEY")"
STYLE="$(python3 -c "import json,sys; s=json.load(open(sys.argv[1]))[sys.argv[2]].get('road_style'); print('--road-style=%s' % s if s else '')" "$HERE/maps.json" "$KEY")"
"$CARLA_PY" "$HERE/osm_city/roadnet.py" "$OSM" "$OUT" "$NAME" $LEFT $FIXES $STYLE
# keep-left: light poles to the right of their lanes / 左側通行: 信号柱を車線の右側へ
[ -n "$LEFT" ] && "$CARLA_PY" "$HERE/osm_city/lht_poles.py" "$OUT/$NAME.xodr" "$OUT/$NAME.xodr"
"$CARLA_PY" "$HERE/osm_city/lanes.py" "$OUT/$NAME.xodr" "$OUT/lanes.json"
if [ ${#GLBS[@]} -gt 0 ]; then
    # hilly map (maps.json "terrain_glb"): terrain field, road elevation, lanes again
    # 起伏のある地図: 地形の高さ場、道路の標高、車線の再抽出
    echo "== terrain"
    # each export's drawn SW corner (maps.json "terrain_sw": [[lat, lon], ...]) seeds its alignment
    # 各エクスポートの南西角（位置合わせの初期値）
    mapfile -t SWS < <(python3 -c "
import json,sys
for q in json.load(open(sys.argv[1]))[sys.argv[2]].get('terrain_sw') or []: print('%s,%s' % tuple(q))" "$HERE/maps.json" "$KEY")
    "$VENV/bin/python" "$HERE/osm_city/terrain.py" "$OUT" --glb "${GLBS[@]}" --osm "$OSM" --sw "${SWS[@]}"
    "$VENV/bin/python" "$HERE/osm_city/elevate.py" "$OUT"
    # road_fixes: dead-end loops whose head would end at a road at another height
    # move clear (the rest get a guardrail in the model) / 高さの違う道路に近い転回ループを離す
    [ -n "$FIXES" ] && "$CARLA_PY" "$HERE/osm_city/turnloops.py" "$OUT/$NAME.xodr" --refine
    # light poles again on the elevated roads: a pinned light (positionInertial)
    # carries the road's z (guide ch. 24) / 標高後に信号柱を再計算（固定位置の z）
    [ -n "$LEFT" ] && "$CARLA_PY" "$HERE/osm_city/lht_poles.py" "$OUT/$NAME.xodr" "$OUT/$NAME.xodr"
    "$CARLA_PY" "$HERE/osm_city/lanes.py" "$OUT/$NAME.xodr" "$OUT/lanes.json"
fi
echo "== 3-D model"
"$VENV/bin/python" "$HERE/osm_city/build.py" "$OSM" "$OUT"
"$VENV/bin/python" "$HERE/osm_city/signs.py" "$OUT"
# maps.json "jartic_d" only: Japanese sign plates (no-op elsewhere) / 日本の標識板（JARTIC 地図のみ）
"$VENV/bin/python" "$HERE/osm_city/jp_signs.py" "$OSM" "$OUT"
# Japan maps: crossings for the walkers (hokousha.json, road logic read only) and
# the pedestrian heads at signalised crossings (guide ch. 26)
# 日本の地図: 横断歩道データと歩行者用信号機（ガイド 26 章）
if [ "$("$CARLA_PY" -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(int(c.get('jp_hokousha', c.get('road_style') == 'japan' or c.get('package') == 'Japan')))" "$HERE/maps.json" "$KEY")" = 1 ]; then
    "$CARLA_PY" "$HERE/osm_city/hokousha_crossings.py" "$OUT"
    "$VENV/bin/python" "$HERE/osm_city/jp_hokousha.py" "$OUT"
fi
# nothing on or over a road (exit 1 stops the chain before the FBX) / 道路上に物が無いこと
"$VENV/bin/python" "$HERE/osm_city/check_roads.py" "$OSM" "$OUT"
"${BLENDER:-blender}" -b -P "$HERE/osm_city/blender_build.py" -- "$OUT" --fbx 2>&1 | grep -E "wrote|Error" || true
[ "$IMPORT" = 1 ] && "$HERE/build_map.sh" "$KEY" "$OUT"
echo "done: $OUT"
