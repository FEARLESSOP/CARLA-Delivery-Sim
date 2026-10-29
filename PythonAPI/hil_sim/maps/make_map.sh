#!/usr/bin/env bash
# Build one of your own circuits (maps.json) into CARLA, end to end:
# centreline + start pose -> OpenDRIVE road logic -> map mesh -> import/cook/install.
#
#   maps/make_map.sh marina_bay
#
# The first run makes maps/.venv (numpy, scipy, scikit-image) for the track
# analysis. Restart CARLA afterwards. / 自作サーキットを一括で CARLA に入れる。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KEY="$1"
CARLA_SRC="${CARLA_SRC:-$HOME/Documents/Dinesh-carla-copy/carla}"
VENV="$HERE/.venv"
[ -x "$VENV/bin/python" ] || { python3 -m venv "$VENV" && "$VENV/bin/pip" install -q numpy scipy scikit-image pillow; }
read -r NAME PKG LANES SPEED < <(python3 -c "import json,sys; c=json.load(open(sys.argv[1]))[sys.argv[2]]; print(c['name'], c['package'], c.get('lanes', 3), c.get('speed_kmh', 70))" "$HERE/maps.json" "$KEY")
OUT="$CARLA_SRC/Dist/${PKG}_stage/$NAME"
mkdir -p "$OUT"
echo "== track: centreline, width, start pose"
"$VENV/bin/python" "$HERE/extract_track.py" "$HERE/maps.json" "$KEY" "$OUT"
echo "== road logic"
python3 "$HERE/make_track_xodr.py" "$OUT/track.json" "$OUT/$NAME.xodr" --name "$NAME" --lanes "$LANES" --speed "$SPEED"
echo "== map mesh"
"${BLENDER:-blender}" -b -P "$HERE/prepare_map.py" -- "$HERE/maps.json" "$KEY" "$OUT" 2>&1 | grep -E "^dropped|^pieces|^wrote|Error" || true
"$HERE/build_map.sh" "$KEY" "$OUT"
