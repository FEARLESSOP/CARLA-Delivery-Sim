#!/usr/bin/env bash
# setup_new_pc.sh — set up the CARLA 0.9.16 HIL rig on a new Ubuntu 24.04 PC.
#
#   curl -fsSL https://raw.githubusercontent.com/FEARLESSOP/CARLA-Delivery-Sim/master/setup_new_pc.sh -o setup_new_pc.sh
#   bash setup_new_pc.sh                          # everything from the internet
#   OLD_PC=user@oldpc bash setup_new_pc.sh        # + cooked maps/cars, data, memory from the old PC
#   DRY_RUN=1 bash setup_new_pc.sh                # only print what would be done
#
# Safe to re-run: every step checks whether it is done already. See
# NEW_PC_SETUP.md for what each step does and what stays manual (the NVIDIA
# driver reboot, the G923 kernel driver, Unreal Engine for building assets).
#
# Environment:
#   CARLA_DIR      where CARLA goes            (default ~/Downloads/CARLA_0.9.16)
#   OLD_PC         user@host of the old PC to rsync from (optional)
#   OLD_HOME       the old PC's home folder    (default: same path as here, i.e. $HOME)
#   OLD_CARLA_DIR  the CARLA folder on the old PC (default: same place relative to home)
#   BUILD_SYNC     runtime | full | none        (maps/build from OLD_PC; default runtime)
#   SKIP_SYSTEM=1  skip apt packages
#   INSTALL_NVIDIA=1  also install the NVIDIA 595-open driver (then reboot)
#   SKIP_SKILLS=1  skip the Claude Code skills (needs GitHub access to the private repo)
#   SKIP_TESTS=1   skip the offline self-tests
#
# 新しい PC に CARLA HIL リグを構築するスクリプト（再実行可能）。
set -uo pipefail

CARLA_DIR="${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}"
OLD_PC="${OLD_PC:-}"
OLD_HOME="${OLD_HOME:-$HOME}"
BUILD_SYNC="${BUILD_SYNC:-runtime}"
DRY_RUN="${DRY_RUN:-0}"
REPO="https://github.com/FEARLESSOP/CARLA-Delivery-Sim.git"
SKILLS_REPO="FEARLESSOP/carla-skills"
CARLA_TAR_URL="https://carla-releases.b-cdn.net/Linux/CARLA_0.9.16.tar.gz"
NODE_MAJOR=22
HIL="$CARLA_DIR/PythonAPI/hil_sim"
VPY="$CARLA_DIR/carla-venv/bin/python"
# the CARLA folder on the old PC: the same place relative to home (or set OLD_CARLA_DIR)
case "$CARLA_DIR" in
  "$HOME"/*) OLD_CARLA_DIR="${OLD_CARLA_DIR:-$OLD_HOME/${CARLA_DIR#$HOME/}}";;
  *)         OLD_CARLA_DIR="${OLD_CARLA_DIR:-$CARLA_DIR}";;
esac
WARN=()

c_b=$'\e[1m'; c_g=$'\e[32m'; c_y=$'\e[33m'; c_r=$'\e[31m'; c_0=$'\e[0m'
step() { printf '\n%s== %s%s\n' "$c_b" "$*" "$c_0"; }
ok()   { printf '%s  ok%s %s\n' "$c_g" "$c_0" "$*"; }
note() { printf '%s  !%s  %s\n' "$c_y" "$c_0" "$*"; WARN+=("$*"); }
die()  { printf '%s  ERROR%s %s\n' "$c_r" "$c_0" "$*"; exit 1; }
run()  { if [ "$DRY_RUN" = 1 ]; then printf '  [dry-run] %s\n' "$*"; else eval "$@"; fi; }
have() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------- 0. checks
step "0. checks"
[ "$(id -u)" = 0 ] && die "run as your normal user, not root (sudo is asked for when needed)"
. /etc/os-release 2>/dev/null || true
[ "${ID:-}" = ubuntu ] || note "not Ubuntu (${PRETTY_NAME:-unknown}); written for Ubuntu 24.04"
[ "${VERSION_ID:-}" = 24.04 ] || note "Ubuntu ${VERSION_ID:-?}: tested on 24.04"
mkdir -p "$(dirname "$CARLA_DIR")"
FREE_GB=$(df -BG --output=avail "$(dirname "$CARLA_DIR")" | tail -1 | tr -dc 0-9)
NEED=45; [ -n "$OLD_PC" ] && NEED=55
[ "${FREE_GB:-0}" -ge "$NEED" ] && ok "${FREE_GB} GB free" || note "only ${FREE_GB} GB free; about ${NEED} GB needed"
ok "CARLA folder: $CARLA_DIR${OLD_PC:+, old PC: $OLD_PC}"

# --------------------------------------------------------------- 1. system
step "1. system packages"
if [ "${SKIP_SYSTEM:-0}" = 1 ]; then
  ok "skipped (SKIP_SYSTEM=1)"
else
  PKGS="git curl rsync xz-utils build-essential python3-venv python3-dev blender jstest-gtk joystick evtest"
  MISSING=$(for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 || echo "$p"; done | tr '\n' ' ')
  if [ -n "${MISSING// }" ]; then
    run "sudo apt-get update && sudo apt-get install -y $MISSING" && ok "installed: $MISSING"
  else
    ok "all present"
  fi
  if have nvidia-smi && nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA driver: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)"
  elif [ "${INSTALL_NVIDIA:-0}" = 1 ]; then
    run "sudo apt-get install -y linux-modules-nvidia-595-open-generic-hwe-24.04 nvidia-driver-595-open"
    note "NVIDIA driver installed: REBOOT, then re-run this script"
  else
    note "no working NVIDIA driver: install it (INSTALL_NVIDIA=1, or setup guide 2.1) and reboot"
  fi
  if id -nG "$USER" | grep -qw input; then ok "$USER is in the input group"
  else run "sudo usermod -aG input $USER"; note "added $USER to the input group (G923): log out and in"; fi
  lsmod 2>/dev/null | grep -q hid_logitech_new && ok "G923 driver (new-lg4ff) loaded" \
    || note "G923 driver new-lg4ff not loaded: install it from https://github.com/berarma/new-lg4ff (DKMS, its README) if you use the wheel"
fi

# ---------------------------------------------------------------- 2. CARLA
step "2. CARLA 0.9.16 (packaged release, ~15 GB download)"
if [ -x "$CARLA_DIR/CarlaUE4.sh" ]; then
  ok "already there"
else
  TAR="$(dirname "$CARLA_DIR")/CARLA_0.9.16.tar.gz"
  run "curl -L -C - --retry 5 -o '$TAR' '$CARLA_TAR_URL'"
  run "mkdir -p '$CARLA_DIR' && tar -xzf '$TAR' -C '$CARLA_DIR'"
  [ "$DRY_RUN" = 1 ] || [ -x "$CARLA_DIR/CarlaUE4.sh" ] || die "CARLA did not unpack into $CARLA_DIR"
  ok "unpacked (you can delete $TAR)"
fi

# ------------------------------------------------------------ 3. the rig
step "3. the rig code (git: $REPO)"
cd "$CARLA_DIR" 2>/dev/null || { [ "$DRY_RUN" = 1 ] || die "no $CARLA_DIR"; }
if [ -d "$CARLA_DIR/.git" ]; then
  if [ -z "$(git -C "$CARLA_DIR" status --porcelain -- PythonAPI/hil_sim 2>/dev/null | head -1)" ]; then
    run "git -C '$CARLA_DIR' pull --ff-only origin master" && ok "updated"
  else
    note "the rig has local changes: not pulling (git -C $CARLA_DIR status)"
  fi
else
  run "git -C '$CARLA_DIR' init -q && git -C '$CARLA_DIR' remote add origin '$REPO'"
  run "git -C '$CARLA_DIR' fetch -q origin && git -C '$CARLA_DIR' checkout -q -f -B master origin/master"
  ok "checked out over the release (adds hil_sim, docs, VehicleFactory)"
fi

# --------------------------------------------------------- 4. Python venv
step "4. Python venv (carla-venv)"
if [ -x "$VPY" ] && PYGAME_HIDE_SUPPORT_PROMPT=1 "$VPY" -c "import carla, pygame, flask" 2>/dev/null; then
  ok "present: $("$VPY" -c 'import carla; print("carla", carla.__version__ if hasattr(carla,"__version__") else "0.9.16")' 2>/dev/null)"
else
  run "python3 -m venv --without-pip '$CARLA_DIR/carla-venv'"
  run "curl -sSLo /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py && '$VPY' /tmp/get-pip.py -q"
  run "'$VPY' -m pip install -q carla==0.9.16 pygame numpy flask evdev shapely pillow matplotlib pandas \
       networkx gymnasium stable-baselines3 tensorboard eclipse-sumo trimesh pygltflib"
  run "'$VPY' -m pip install -q torch --index-url https://download.pytorch.org/whl/cpu"
  ok "created"
fi

# ---------------------------------------------------------- 5. Node.js
step "5. Node.js $NODE_MAJOR LTS (~/.local/node, only to rebuild the dashboard UI)"
if [ -x "$HOME/.local/node/bin/node" ]; then
  ok "present: $("$HOME/.local/node/bin/node" --version)"
else
  V=$(curl -s https://nodejs.org/dist/index.json | python3 -c "import json,sys;print([r['version'] for r in json.load(sys.stdin) if r['lts'] and r['version'].startswith('v$NODE_MAJOR')][0])")
  T=$(mktemp -d)
  run "curl -sSLo '$T/node.tar.xz' https://nodejs.org/dist/$V/node-$V-linux-x64.tar.xz && curl -sSLo '$T/SHASUMS256.txt' https://nodejs.org/dist/$V/SHASUMS256.txt"
  run "(cd '$T' && grep ' node-$V-linux-x64.tar.xz\$' SHASUMS256.txt | sed 's/node-$V-linux-x64.tar.xz/node.tar.xz/' | sha256sum -c -)" || die "Node checksum mismatch"
  run "mkdir -p '$HOME/.local/node' && tar -xJf '$T/node.tar.xz' -C '$HOME/.local/node' --strip-components=1"
  ok "Node $V (checksum verified)"
fi
if [ -f "$HIL/web/package.json" ] && [ ! -d "$HIL/web/node_modules" ]; then
  run "(export PATH=\$HOME/.local/node/bin:\$PATH; cd '$HIL/web' && npm ci --no-audit --no-fund >/dev/null)" && ok "web/ dependencies installed"
fi
[ -f "$HIL/web/dist/index.html" ] && ok "dashboard UI build present (web/dist)" || {
  run "(export PATH=\$HOME/.local/node/bin:\$PATH; cd '$HIL/web' && npm run build >/dev/null)" && ok "dashboard UI built"; }

# ------------------------------------------- 6. content from the old PC
step "6. cooked maps/cars, run-time data and memory from the old PC"
if [ -z "$OLD_PC" ]; then
  note "OLD_PC not set: own maps (OitaKosen, OitaStation, PasirRis, circuits), own cars and pods are NOT installed (not in git: too big). Re-run with OLD_PC=user@oldpc, or rebuild them (setup guide 5.10, 5.12, 5.13)"
else
  have rsync || die "rsync missing"
  O="$OLD_PC:$OLD_CARLA_DIR"
  for d in CarlaUE4/Content/DeliveryModels CarlaUE4/Content/LastMileModels CarlaUE4/Content/UserCars \
           CarlaUE4/Content/Japan CarlaUE4/Content/Singapore CarlaUE4/Content/Circuits Import/Japan; do
    run "mkdir -p '$CARLA_DIR/$d' && rsync -a --info=stats0 '$O/$d/' '$CARLA_DIR/$d/'" && ok "$d"
  done
  run "rsync -a '$O/CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory.u'* '$CARLA_DIR/CarlaUE4/Content/Carla/Blueprints/Vehicles/'" && ok "VehicleFactory (pods + own cars)"
  run "rsync -a '$O/PythonAPI/hil_sim/data/' '$HIL/data/'" && ok "hil_sim/data (sessions, signal timing, scenarios)"
  for f in ffb_tuning.json ffb_presets.json vehicle_tuning.json pedal_tuning.json keybinds.json g923_buttons.json \
           wheel_config.ini season.json weather_cycle.json launch_options.json; do
    run "rsync -a '$O/PythonAPI/hil_sim/$f' '$HIL/$f' 2>/dev/null" || true
  done
  ok "your tuning, keybinds, wheel calibration and launch settings"
  case "$BUILD_SYNC" in
    full)    run "rsync -a '$O/PythonAPI/hil_sim/maps/build/' '$HIL/maps/build/'" && ok "maps/build (full, ~13 GB)";;
    runtime) run "rsync -a --prune-empty-dirs --include='*/' --include='textures/***' --include='nav_cache/***' \
                  --include='*.json' --include='*.csv' --include='*.xodr' --exclude='*' \
                  '$O/PythonAPI/hil_sim/maps/build/' '$HIL/maps/build/'" && ok "maps/build (run-time files: textures, json, csv, xodr, navmesh cache)";;
    *)       ok "maps/build skipped (BUILD_SYNC=none)";;
  esac
  run "rsync -a '$OLD_PC:$OLD_HOME/Downloads/oita_station_map_v8' '$HOME/Downloads/' 2>/dev/null" && ok "oita_station_map_v8 (delivery addresses)" || true
  OLD_MEM="-$(echo "$OLD_HOME/Downloads/carla" | tr '/' '-' | sed 's/^-//')"
  NEW_MEM="-$(echo "$HOME/Downloads/carla" | tr '/' '-' | sed 's/^-//')"
  run "mkdir -p '$HOME/.claude/projects' && rsync -a '$OLD_PC:$OLD_HOME/.claude/projects/$OLD_MEM/memory/' '$HOME/.claude/projects/$NEW_MEM/memory/' 2>/dev/null" \
    && ok "Claude Code memory" || note "Claude Code memory not copied"
fi

# ---------------------------------------- 7. Claude Code skills + agents
step "7. Claude Code skills and agents ($SKILLS_REPO, private)"
if [ "${SKIP_SKILLS:-0}" = 1 ]; then
  ok "skipped (SKIP_SKILLS=1)"
elif [ -d "$HOME/.claude/skills/.git" ]; then
  run "git -C '$HOME/.claude/skills' pull -q --ff-only" && ok "skills updated"
else
  if have gh && gh auth status >/dev/null 2>&1; then
    run "mkdir -p '$HOME/.claude' && gh repo clone '$SKILLS_REPO' '$HOME/.claude/skills.new' -- -q"
    if [ -d "$HOME/.claude/skills" ] && [ "$DRY_RUN" != 1 ]; then
      run "cp -rn '$HOME/.claude/skills/.' '$HOME/.claude/skills.new/' && mv '$HOME/.claude/skills' '$HOME/.claude/skills.bak.$(date +%s)'"
    fi
    run "mv '$HOME/.claude/skills.new' '$HOME/.claude/skills'" && ok "skills cloned"
  else
    note "skills not installed: the repo is private. Run 'gh auth login' (sudo apt install gh), then re-run"
  fi
fi
if [ -d "$HOME/.claude/skills/carla-map-generator" ]; then
  run "mkdir -p '$HOME/.claude/agents' && cp '$HOME/.claude/skills/'carla-*/agents/*.md '$HOME/.claude/agents/'" && ok "agents installed"
  [ -e "$CARLA_DIR/map_guide" ] || run "ln -s '$HOME/.claude/skills/carla-map-generator/map_guide' '$CARLA_DIR/map_guide'"
fi

# ------------------------------------------------- 8. machine-specific paths
step "8. paths written for the old PC (/home/inouelab)"
if [ "$HOME" = /home/inouelab ]; then
  ok "same home folder: nothing to change"
else
  for f in "$HIL/maps/maps.json" "$HIL/maps/prepare_oita.py" "$HIL/cars/cars.json"; do
    [ -f "$f" ] && grep -q /home/inouelab "$f" && run "sed -i.bak 's#/home/inouelab#$HOME#g' '$f'" && ok "$(basename "$f") (backup .bak)"
  done
  note "your home is not /home/inouelab: paths were rewritten; Claude Code memory folder names contain the path (setup guide 7.3)"
fi

# ------------------------------------------------------------ 9. tests
step "9. offline self-tests (no CARLA needed)"
if [ "${SKIP_TESTS:-0}" = 1 ] || [ "$DRY_RUN" = 1 ]; then
  ok "skipped"
else
  cd "$HIL" || die "no $HIL"
  FAILS=0
  for t in tests/test_launcher.py tests/test_pipeline_control.py tests/test_delivery.py tests/test_traction_fx.py \
           tests/test_weather_cycle.py tests/test_traffic_profile.py; do
    if timeout 300 "$VPY" "$t" >/tmp/setup_test.log 2>&1; then ok "$t"; else FAILS=$((FAILS+1)); note "$t failed (see /tmp/setup_test.log)"; fi
  done
fi

# ------------------------------------------------------------- summary
step "done"
if [ ${#WARN[@]} -gt 0 ]; then
  printf '%sStill to do / check:%s\n' "$c_y" "$c_0"
  for w in "${WARN[@]}"; do printf '  - %s\n' "$w"; done
fi
cat <<EOF

Next:
  cd $HIL
  ../../carla-venv/bin/python calibrate_wheel.py      # once, with the G923 plugged in (PC mode)
  python3 run_pipeline.py                             # the launcher: pick map/traffic on http://localhost:5000
Full reference: $CARLA_DIR/CARLA_RIG_SETUP_FROM_SCRATCH.md (section 8: checklist)
EOF
