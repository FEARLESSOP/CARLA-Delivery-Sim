# CARLA HIL Rig: Every Change, and How to Rebuild It From Scratch

This file lists everything that was changed or added on top of a stock CARLA
0.9.16 to get this machine's driving-simulator rig to where it is now
(2026-11-09). It is meant to be enough to set the rig up again on a new PC.

Each section says what was changed, why, which files are involved, how it
works, how to set it up and run it, and what is still wrong with it. It was
written from the actual files on this machine: the rig code, its docs, the
Claude Code memory notes, the git diffs, the skill guides and the system
itself. Anything that could not be checked is marked **(unverified)**.

> **All rig documents by topic:** `PythonAPI/hil_sim/docs/README.md`.

> **Machine-specific values.** This machine's user is `inouelab`, and most
> paths are under `/home/inouelab/...` (`~`). Absolute paths are hard-coded in
> a few places (listed in [section 6.4](#64-hard-coded-paths-to-change-on-a-new-pc)).
> If your new user name or folders are different, change those places. The
> easiest fix is to keep the same layout: `~/Downloads/CARLA_0.9.16`,
> `~/Documents/Dinesh-carla-copy/carla` and `~/UnrealEngine_4.26`.

---

## Contents

1. [Overview](#1-overview)
2. [Hardware and OS setup](#2-hardware-and-os-setup)
3. [Stock CARLA 0.9.16, the Python venv, and where the rig must sit](#3-stock-carla-0916-the-python-venv-and-where-the-rig-must-sit)
4. [The CARLA source tree + Unreal Engine 4.26 (needed for every import)](#4-the-carla-source-tree--unreal-engine-426-needed-for-every-import)
5. [Every rig feature](#5-every-rig-feature)
6. [Changes to the packaged CARLA folder, and how to reproduce each one](#6-changes-to-the-packaged-carla-folder-and-how-to-reproduce-each-one)
7. [AI assistant setup: Claude Code and Google Antigravity](#7-ai-assistant-setup-claude-code-and-google-antigravity)
8. [New-PC checklist, in order](#8-new-pc-checklist-in-order)
9. [Troubleshooting, known issues and open items](#9-troubleshooting-known-issues-and-open-items)
10. [Timeline of changes](#10-timeline-of-changes)
11. [Things that could not be verified, and contradictions found](#11-things-that-could-not-be-verified-and-contradictions-found)

---

## 1. Overview

### 1.1 What the rig is

A **human-in-the-loop (HIL) driving simulator** built on CARLA 0.9.16
(Unreal Engine 4.26). You drive it with a **Logitech G923** wheel and pedals,
or with W/A/S/D. Over time, the following has been built on top of the
original study code:

- **Driving:** force feedback (FFB) computed from the car's physics, tunable
  live; pedal and steering tuning; a first-person driver eye for every
  vehicle; a steering wheel that turns 1:1 with the G923 on your own cars;
  skid marks.
- **A web dashboard** on `http://localhost:5000`: telemetry, statistics,
  delivery view, and settings (pedals, vehicle performance, FFB, delivery
  sim, season and wind).
- **A delivery simulation:** delivery pods (an artist's pod and the
  "LastMile" LMP-4 pod) carrying drones, box stations, charge stations,
  batteries, wind and failures.
- **Own cars:** 12 imported vehicles (RX-7, Supra A70, i-MiEV, NSX-R,
  Soarer, Sileighty, BMW 3.0 CSL, Red Bull RB16, Ferrari SF90 XX, Rolls-Royce
  Dawn, Honda S800, and the NPC-only Oita bus), with invisible window glass,
  blinkers and mirrors.
- **Own maps:** Oita Station (Japan; since 2026-11-09 the user's v8 model kept
  as the core of a generated city to ±640 m, with parked cars and timetabled
  Oita buses), Pasir Ris (Singapore), Oita Kosen (Japan, hilly terrain,
  landmarks, real road signs from JARTIC open data, seasonal trees that sway in
  the wind), and two race circuits (Marina Bay, Spa-Francorchamps).
- **Road logic overlay** (O key): the OpenDRIVE map the AI drives on, drawn in
  the game; lane-change marks on Japan maps follow the painted lines.
- **An RL research layer:** a SAC agent, study arms, and plots.
- **Claude Code skills** that build maps and import cars: `carla-map-generator`
  and `carla-car-importer`, in the repo `FEARLESSOP/carla-skills`; the same
  knowledge is reachable for **Google Antigravity** through `AGENTS.md` files
  (section 7).

### 1.2 Where it started

The original rig was `~/Downloads/v2x-HIL-Carla-simulation-oita-master.zip`
(files dated 2026-07-16). It held only these files: `README.md`,
`agent_drive.py`, `dashboard.py`, `docs/RESEARCH_PROTOCOL.md`,
`docs/RL_DESIGN.md`, `generate_traffic.py`,
`manual_control_steeringwheel G923.py`, `rl/` (`carla_env`, `evaluate`,
`extract_scenarios`, `plot_progress`, `rules`, `smoke_test`, `train_sac`),
`run_pipeline.py`, `sensor_viewer.py`, `wheel_config.ini` and
`.claude/skills/grill-me`, `grilling`. It was written for CARLA **0.9.15**.
The original dashboard already had the Dashboard and Statistics tabs, pedal
tuning (`pedal_tuning.json`), driver alerts and the turn-signal file.

**Everything else in this file was added or changed after that.**

### 1.3 The folders

```
~/Downloads/CARLA_0.9.16/              <- the packaged CARLA 0.9.16 that RUNS (git repo -> FEARLESSOP/CARLA-Delivery-Sim)
├── CarlaUE4.sh                          stock launcher
├── carla-venv/                          Python 3.12 venv for every rig script (added)
├── map_guide -> ~/.claude/skills/carla-map-generator/map_guide   (symlink, added)
├── AGENTS.md                            instructions for Google Antigravity (rules, method, gates, checks, traps; added 2026-11-09)
├── .agents/skills/                      Antigravity skill stubs -> the two Claude skills (added 2026-11-09)
├── Import/Japan/                        OitaStation staging copy (added)
├── CarlaUE4/Content/
│   ├── Carla/Blueprints/Vehicles/VehicleFactory.uasset/.uexp   REPLACED (lists the pods + own cars)
│   ├── DeliveryModels/                  pod / drone props + vehicle.delivery.pod (added)
│   ├── LastMileModels/                  LMP-4 pod, 37 props + vehicle.delivery.lastmile (added)
│   ├── UserCars/                        12 own vehicles (incl. the Oita bus) + steering-wheel props (added)
│   ├── Japan/                           maps OitaStation, OitaKosen (added)
│   ├── Singapore/                       map PasirRis (added)
│   └── Circuits/                        maps MarinaBay, SpaFrancorchamps (added)
└── PythonAPI/hil_sim/                   THE RIG (all code, must sit exactly here)
    ├── AGENTS.md                        the rig's Antigravity instructions (mirror of CLAUDE.md)
    └── FILES.md                         where is what: every important file by topic (start here)

~/Documents/Dinesh-carla-copy/carla    <- CARLA 0.9.16 SOURCE tree, branch ue4/0.9.16, with a built editor
                                          (used only to import + cook new assets; small local patches)
~/UnrealEngine_4.26                    <- Unreal Engine 4.26, CarlaUnreal fork, branch carla (built from source)
~/Downloads/carla                      <- a UE5 CARLA git checkout (branch ue5-dev). NOT used by the rig.
~/.claude/skills                       <- git repo FEARLESSOP/carla-skills (map + car skills)
~/.claude/agents                       <- the 15 agent definitions copied out of the skills (12 map + 3 car)
```

**Which "carla" folder is the real one.** The memory note
`carla-hil-rig-setup-state` covers this. `~/Downloads/CARLA_0.9.16` is the
simulator that runs. `~/Downloads/carla` is the UE5 `ue5-dev` *source*
checkout (3.4 GB, 2.1 GB of it `.git`). It has no compiled binary and no
content, and it cannot run without a UE 5.5 build, which this machine
doesn't have. The user twice pointed at it as "the CARLA I downloaded", but
it is not the simulator. It is kept on disk by the user's choice. Since
2026-10-26 the user also edits the UE4 *source* copy
`~/Documents/Dinesh-carla-copy/carla` in the editor (`make launch`). So
"is CarlaUE4 launched?" can mean that editor, whose port 2000 only opens
after you press Play.

### 1.4 The processes (`run_pipeline.py`)

```
run_pipeline.py  (parent; each child in its own process group; up to 3 restarts per helper)
 ├─ CarlaUE4.sh                    simulator (PRIME offload env vars on hybrid GPUs)   [skipped with --no-sim / if port 2000 answers]
 ├─ load_world_map()               loads --map in a throwaway client (120 s + check; was util/config.py, 10 s)
 ├─ generate_traffic.py            THE TICK MASTER (sync mode, 30 Hz), NPC cars/bikes/walkers, heartbeat file;
 │                                  also parked cars (parked_cars.py) and timetabled buses (bus_schedule.py)
 ├─ dashboard.py                   Flask on :5000; also applies vehicle tuning and seasons
 ├─ agent_drive.py                 (--agent) autonomous 'ego' car
 ├─ delivery_sim.py --passive      (--delivery N) pods + drones
 ├─ drone_viewer.py                (--drone-cams)
 ├─ carla_ffb.py                   (--ffb) skipped if one already runs
 ├─ traction_fx.py                 skid marks (unless --no-traction-fx)
 ├─ road_overlay.py                road logic overlay, idle until O / dashboard (unless --no-road-overlay)
 ├─ sensor_viewer.py               multi-camera window (unless --no-sensor-viewer)
 └─ manual_control_steeringwheel G923.py   the driving window (pipeline waits on it; ESC ends everything)
```

The components never talk to each other directly. They find vehicles in the
CARLA world by `role_name` (`hero` = the human, `ego` = the agent), and they
share small JSON files next to the scripts: `pedal_tuning.json`,
`turn_signal.json`, `traffic_heartbeat.json`, `ffb_tuning.json`,
`vehicle_tuning.json`, `season.json`, and `data/ffb_status.json` and
`data/delivery_live/`.

Started on every map: `weather_cycle.py` (day/night + weather, 5.17;
`--no-weather-cycle` turns it off).
Started per map: `signal_timing.py apply --watch --only-map <Map>` on maps
whose maps.json entry has `"signal_timing"` (OitaKosen; 5.14).
Not started by the pipeline: `seasons.py` (the dashboard applies seasons itself; the CLI is optional).

---

## 2. Hardware and OS setup

The machine: Ubuntu **24.04.5 LTS**, kernel `7.0.0-38-generic` (HWE).
Intel **i5-14600K** (20 threads) on an ASUS TUF GAMING B760-PLUS WIFI D4,
62 GB DDR4-3200, an **NVIDIA RTX 5080** (Blackwell, PCI id `10de:2c02`,
16 GB) plus the Intel UHD 770 iGPU, and three LG FHD monitors. The system
Python is 3.12.3. Blender is 4.0.2 (`/usr/bin/blender`, from apt).

### 2.1 NVIDIA driver: the open kernel module is required

**What:** the RTX 5080 works **only with the `-open` NVIDIA kernel modules**.

**Why:** with the closed flavour (for example `nvidia-driver-610` instead of
`nvidia-driver-610-open`), the modules load, but the GPU never starts. You
get `RmInitAdapter failed`, no `/sys/class/drm/card*` for the NVIDIA card,
`nvidia-smi` says "No devices were found", and the monitors on the RTX
outputs go dark. CARLA then quietly renders on the iGPU. `dmesg` is blocked
here (`kernel.dmesg_restrict=1`), so use `journalctl -k -b | grep -i nvrm`
to see the message `requires use of the NVIDIA open kernel modules`.

**Currently installed (checked 2026-11-04):** `nvidia-driver-595-open`
**595.99.02**, with Canonical's **prebuilt signed** modules
(`linux-modules-nvidia-595-open-generic-hwe-24.04`). `prime-select query`
reports `on-demand`.

**How it was installed:** with the script `~/Downloads/install_nvidia_driver.sh`
(`sudo bash ~/Downloads/install_nvidia_driver.sh [--reboot]`). Secure Boot is
**enabled** on this machine. The script installs the signed prebuilt module
package *before* the driver metapackage, so apt never falls back to a DKMS
build and no MOK enrolment is needed. The equivalent by hand:

```bash
sudo apt install linux-modules-nvidia-595-open-generic-hwe-24.04 nvidia-driver-595-open
sudo reboot
nvidia-smi                       # must list "NVIDIA GeForce RTX 5080"
ls /sys/class/drm/               # an NVIDIA card* must appear
```

**After every driver upgrade, check the flavour (`-open`), not just the
version.**

**History (2026-09-14):** while chasing GPU crashes the driver went
595-open → 610-open (still crashed) → 610 proprietary (the GPU vanished)
→ 610-open 610.43.02. The machine is now back on 595-open 595.99.02. The
date of that last change is not in the notes **(unverified)**. The
`nvidia-dkms-610` package is in state `rc` (removed).

You can't swap the module live: the terminal (`warp-terminal`) holds
`/dev/nvidiactl` open. Install, then reboot.

### 2.2 DKMS and gcc-14 (only if you build the driver with DKMS)

**Only relevant if apt falls back to a DKMS build.** Prefer the prebuilt
signed modules above. With the default **gcc-13 (13.3.0)** the NVIDIA DKMS
build dies with a deterministic compiler error:

```
nvidia-uvm/uvm_gpu_semaphore.c:866:1: internal compiler error: in build_abbrev_table, at dwarf2out.cc:9244
```

That leaves `nvidia-dkms-*` half-configured (`iF`), with no `nvidia.ko` on
disk. The cause is that the kernel needs full DWARF5 debug info
(`CONFIG_GENDWARFKSYMS`), and gcc-13 crashes generating it. The fix is to
build with gcc-14 (installed here: gcc-14 14.2.0):

```bash
sudo apt-get install -y gcc-14
sudo env CC=gcc-14 CXX=g++-14 dkms build   -m nvidia -v <ver> -k $(uname -r) --force
sudo dkms install -m nvidia -v <ver> -k $(uname -r) --force
sudo dpkg --configure -a
# verify: dkms status = installed; /lib/modules/$(uname -r)/updates/dkms/nvidia*.ko.zst exist; dpkg -l shows ii
```

Don't confuse this with the *random* `internal compiler error: Segmentation
fault` that appears at a different place on each run under `-j16`. That one
is the CPU problem in 2.4: build at `-j2`. On 2026-09-14,
`/usr/src/nvidia-*/dkms.conf` was patched to `PROCS_NUM=2`, but a package
upgrade reverts that. (The current `/usr/src/nvidia-595.99.02/dkms.conf` has
no `PROCS_NUM` line.)

### 2.3 Hybrid GPU: PRIME render offload

**What:** with `prime-select on-demand`, the desktop (X) runs on the Intel
iGPU. So does any program started plainly, **CarlaUE4 included**.

**Why it matters:** a bare `./CarlaUE4.sh` renders on the UHD 770. It
crawls, `nvidia-smi` shows no process, and eventually it dies with
`VK_ERROR_DEVICE_LOST` (in the kernel log: `i915 ... GPU HANG`). One whole
session on 2026-10-26 ran on the iGPU without anyone noticing.

**How it is handled:** `run_pipeline.py` has `nvidia_offload_env()` and
`--gpu {auto,nvidia,off}`. `auto` adds the three variables when
`prime-select query` says `on-demand`. When you run CARLA **outside** the
pipeline, set them yourself:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only \
  ./CarlaUE4.sh
nvidia-smi    # CarlaUE4-Linux-Shipping must be listed (~5.5-5.8 GB, 57-182 W)
```

### 2.4 CPU stability (i5-14600K)

**What was seen (2026-09-14):** under parallel load (`make -j16`) the
compiler crashed at a different spot on every run. At `-j2` it passed, and
compiling one file 12 times in a row passed 12 of 12. These are the
symptoms of Intel's **Raptor Lake Vmin Shift** problem. CPU corruption can
also show up as GPU Xid errors (corrupt command buffers).

Already ruled out: RAM at JEDEC spec (no XMP), temperatures (48 °C), MCE
and EDAC (this board has no ECC, so memory errors would be silent).
Microcode is 0x133.

**Next steps the notes recommend (not done):** BIOS → load Intel Default
Settings and turn off ASUS Multicore Enhancement; run memtest86+ overnight;
if it is still unstable, RMA the CPU (Intel's extended 5-year warranty).
**Rule of thumb on this box:** build at low parallelism.

**A healthy CARLA session uses 185–400 % CPU** (measured with 60 vehicles
and 15 walkers; that is about 20 % of 20 cores). A number over 100 % in
`top` is not a problem.

### 2.5 GPU crashes (NVRM Xid) and the whole-PC freeze

- **Xid crash:** the server dies with `VK_ERROR_DEVICE_LOST`, signal 11, or
  "GameThread timed out waiting for RenderThread". What you see: the
  driving window freezes. Check `journalctl -k | grep "NVRM: Xid"` before
  debugging any Python. The trigger that was measured is spawning or
  destroying many actors in one batch, so `generate_traffic.py` now does
  both in chunks (`--spawn-chunk 5 --spawn-pause 0.5`). This is a
  workaround, not a fix. After repeated faults, reboot.
- **Hard lockup (2026-09-14 13:34):** an RL run froze the whole PC inside
  `nvidia_modeset` (the display thread). The safeguards are now in
  `rl/run_arm.py`. Things only you can change: `kernel.hardlockup_panic=1` +
  `kernel.panic=30` (sysctl), and moving the third monitor off the RTX 5080.
- Full write-up: `PythonAPI/hil_sim/docs/TROUBLESHOOTING.md`.

### 2.6 Monitors

Three identical LG FHD panels, told apart by their EDID serials:
`...L425` (GNOME primary), `...L426` and `...B261`. The motherboard has only
HDMI ×2 + DP ×1. L426 and B261 use the two Intel HDMI ports, so the third
panel is always on the RTX 5080. If the NVIDIA module is broken, that panel
is the one that goes dark. The layout is saved in `~/.config/monitors.xml`.
When a display is missing, first count the Intel connectors
(`/sys/class/drm/card1-*/status`).

### 2.7 The Logitech G923 wheel

| Piece | State on this machine |
|---|---|
| Kernel driver | **new-lg4ff** 0.5.0 via DKMS (`/usr/src/new-lg4ff-0.5.0`, cloned from `https://github.com/berarma/new-lg4ff`); module `hid_logitech_new` loaded |
| udev rules | `/etc/udev/rules.d/99-logitech-wheel-perms.rules` (+ fanatec, thrustmaster), installed 2026-08-28; they make the G923 (`046d:c266`) sysfs files (`range`, `gain`, `autocenter`, `damper_level`, ...) writable |
| Tools | `/usr/local/bin/oversteer` (installed the same day as the udev rules; that the rules came from Oversteer is an assumption **(unverified)**), `jstest-gtk`, `jstest`, `evtest` |
| Permissions | user is in the `input` group (`sudo usermod -aG input $USER`, then log out and in) so `carla_ffb.py` can open the evdev device |
| Mode | the G923 must be in **PC mode** (`046d:c266`); PS4 mode (`c267`) puts the pedals on other axes, which `calibrate_wheel.py` detects |

To install new-lg4ff, follow its upstream README (DKMS install). On this
machine it is registered as `new-lg4ff/0.5.0` for each kernel. Its exact
install commands were not recorded **(unverified)**.

---

## 3. Stock CARLA 0.9.16, the Python venv, and where the rig must sit

### 3.1 Download and extract the packaged release

What was done (scripts in `~/Downloads`: `resume_carla_download.sh`,
`finish_carla_setup.sh`):

```bash
mkdir -p ~/Downloads/CARLA_0.9.16
curl -L -C - --retry 5 --retry-delay 5 -o ~/Downloads/CARLA_0.9.16.tar.gz \
     https://carla-releases.b-cdn.net/Linux/CARLA_0.9.16.tar.gz      # 8,346,095,504 bytes
tar -xzf ~/Downloads/CARLA_0.9.16.tar.gz -C ~/Downloads/CARLA_0.9.16
chmod +x ~/Downloads/CARLA_0.9.16/CarlaUE4.sh
```

A plain backgrounded `curl` dies silently when its shell closes. That once
stalled the download at 1.25 GB. Use `-C -` to resume. The installed folder
is now 38 GB, with all the additions.

### 3.2 The venv `carla-venv`

The system Python has **no pip or ensurepip**. The venv was built without
pip and bootstrapped with `get-pip.py`. (Note: `get-pip.py --no-index
--find-links` fails because no pip wheel is cached.) The first wheels were
kept in `~/Downloads/carla_wheels/` (carla 0.9.16 cp312, pygame, numpy,
flask, plus `get-pip.py`).

```bash
cd ~/Downloads/CARLA_0.9.16
python3 -m venv --without-pip carla-venv
carla-venv/bin/python ~/Downloads/carla_wheels/get-pip.py      # or download get-pip.py
carla-venv/bin/pip install carla==0.9.16 pygame numpy flask evdev shapely pillow \
    matplotlib pandas networkx gymnasium stable-baselines3 tensorboard eclipse-sumo \
    trimesh pygltflib
carla-venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu   # CPU torch for RL
```

What is in it now (Python 3.12.3, `pip freeze`): carla 0.9.16, pygame 2.6.1,
numpy 2.5.2, Flask 3.1.3, evdev 2.0.0 (FFB), shapely 2.2.0 (BasicAgent and
the delivery sim need it), pillow 12.3.0 (seasons), eclipse-sumo 1.28.0
(gives `carla-venv/bin/netconvert`, used for the OSM road logic),
gymnasium 1.3.0, stable_baselines3 2.9.0, torch 2.14.0+cpu, tensorboard
2.21.0, matplotlib, pandas, networkx, trimesh, pygltflib, PyOpenGL. Exact
versions may not exist on a new PC; any compatible ones should do. **The
client `carla` version must match the server (0.9.16).**

### 3.3 Where the rig must sit (this matters)

The rig **must** be at `<CARLA_ROOT>/PythonAPI/hil_sim`, with its files
directly inside, not nested in the zip's `v2x-...-master/` folder.

**Why:** `run_pipeline.py` works everything out from its own location:

```python
BASE_DIR   = Path(__file__).resolve().parent     # .../PythonAPI/hil_sim
CARLA_ROOT = BASE_DIR.parent.parent
VENV_PY    = CARLA_ROOT / "carla-venv" / "bin" / "python"
UTIL_DIR   = CARLA_ROOT / "PythonAPI" / "util"   # config.py loads maps
```

It launches every child with `VENV_PY`, the simulator with
`CARLA_ROOT/CarlaUE4.sh`, and maps with `PythonAPI/util/config.py`. One
level too deep, `CARLA_ROOT` becomes `PythonAPI/` and the run dies with
`ERROR: venv python not found`. The build scripts work the same way
(`cars/build_cars.sh`, `maps/build_map.sh` and
`delivery/assets/build_*.sh` compute the CARLA root from their own path).

Ubuntu has no `python` command, only `python3`. Start the pipeline with
`python3 run_pipeline.py`. It then uses the venv for the children.

---

## 4. The CARLA source tree + Unreal Engine 4.26 (needed for every import)

The packaged CARLA can't import new assets. Every new map, car, prop or pod
is imported and **cooked** in a CARLA **source** tree with a built UE4
editor, and only the cooked package is copied into the packaged CARLA. You
need this only to **rebuild** assets. If you copy the cooked content from
the old PC (section 6), the rig runs without it.

### 4.1 Unreal Engine 4.26 (CarlaUnreal fork)

- Location: `~/UnrealEngine_4.26`, remote `https://github.com/CarlaUnreal/UnrealEngine.git`,
  branch `carla` (grafted shallow clone, HEAD `e9d9e60c85f6`, 2026-06-12).
- It is a private repo: link your Epic Games account to GitHub first
  (unrealengine.com → Connected Accounts → GitHub, then accept the
  invitation). You may need a personal access token (PAT) as the password.
- Build, from `~/Documents/Dinesh-carla-copy/carla/carla-digital-twin-setup.md`:

```bash
sudo apt-get install -y build-essential mono-complete clang cmake ninja-build python3-dev \
    libvulkan-dev libsdl2-dev libssl-dev libxml2-dev libfreetype6-dev xdg-user-dirs dos2unix
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
cd ~/UnrealEngine_4.26 && ./Setup.sh && ./GenerateProjectFiles.sh && make -j$(nproc)
echo 'export UE4_ROOT=~/UnrealEngine_4.26' >> ~/.bashrc      # present in this machine's ~/.bashrc
```

  (That guide also lists `python3.10-dev`, which may not exist on 24.04.)
  Given the CPU instability in 2.4, consider a lower `-j` if the build
  crashes at random places.
- **ShaderCompileWorker** was skipped by the first build and had to be built
  on its own (memory note `carla-ue4-setup-state`; the exact command was not
  recorded, but the usual UE4 one is `make ShaderCompileWorker` **(unverified)**).
- The engine is registered in `~/.config/Epic/UnrealEngine/Install.ini` as
  `{0004E9C0-08DF-2EC6-06E2-95908E6BAF55}=/home/inouelab/UnrealEngine_4.26`.
  That is why `CarlaUE4.uproject`'s `EngineAssociation` changed (see 4.3).
  On a new PC the editor writes a new GUID.

### 4.2 The CARLA source tree

- Location: `~/Documents/Dinesh-carla-copy/carla`, remote
  `https://github.com/carla-simulator/carla.git`, branch **`ue4/0.9.16`**
  (HEAD `1cd0f377a`, "Quality of Life Improvements to IAI Integration (#9533)").
- The content (~24 GB) is downloaded into `Unreal/CarlaUE4/Content/Carla`
  (`./Update.sh`, the standard CARLA step). The StreetMap plugin is cloned
  into `Unreal/CarlaUE4/Plugins/Streetmap` (from
  `https://github.com/carla-simulator/StreetMap.git`).
- `make launch` opens the editor on the RTX 5080. Known harmless log noise:
  a Vulkan "VendorId != Unknown" ensure (three GPUs: Intel, NVIDIA,
  llvmpipe) and missing `BP_TLOpenDrive_RHT/LHT` (now created as copies, see
  4.4).
- Don't run `make import` / `make package` for the rig's assets. They
  rebuild the editor, and this tree has uncommitted C++ while editors are
  open. The rig's build scripts call the editor commandlets directly.

### 4.3 Every change made to the source tree (`git status` / `git diff`)

| File | Change | Why |
|---|---|---|
| `Util/BuildTools/Setup.sh` (+67/−29) | adds `--sysroot=$UE4_ROOT/.../v17_clang-10.0.1-centos7/...` (`UNREAL_HOSTED_CFLAGS`) to CFLAGS/CXXFLAGS and to every dependency build (toolchain files, rpclib, gtest, recast, xerces, sqlite, proj, libpng); boost is bootstrapped with `--with-toolset=gcc` but built with clang-10 + libc++, using a `custom-include` dir that symlinks the system `pyconfig.h`; recast is linked with libc++ and `-lpthread`; the libpng URL becomes `download.sourceforge.net` and only the library is installed; patchelf is compiled with a single `g++` line instead of autotools | building CARLA's dependencies on Ubuntu 24.04 (system gcc 13 / clang 18, newer glibc) against UE4.26's bundled clang-10 toolchain. The errors met before: `pyconfig.h not found`, `clang++-10.0 not found` (logs left in `error.txt`, `tesy.txt`) |
| `Util/BuildTools/BuildOSM2ODR.sh` (+11/−22) | same sysroot flags, `-DJSON_HAS_CPP_11`, `-Wl,-L` for the libc++ path; drops a duplicated server build block | osm2odr wouldn't build with the hosted toolchain |
| `Unreal/CarlaUE4/CarlaUE4.uproject` | `EngineAssociation` "4.26" → the source-build GUID; adds plugins **StreetMap** and **GeometryCacheAbcFile** (enabled) | the editor rewrites the association when it opens with a source-built engine; StreetMap is for the OSM / Digital Twin tool. (`PythonScriptPlugin` and `EditorScriptingUtilities`, which the rig's editor scripts need, are already enabled in stock 0.9.16) |
| `Plugins/CarlaTools/Source/CarlaTools/Private/OpenDriveToMap.cpp`, `Public/OpenDriveToMap.h` | new properties `GroundMaterial` (defaults to `MI_DarkConcrete`), `bFlatTerrain` (default true: height 0, landscape at −5 cm, so there are no dips by the roads), `bGenerateVegetation` (default false), `bRemoveSidewalksOverRoads` (default true: drops sidewalk triangles whose centroid lies inside a driving lane, more than 0.25 m from its edge) | for the Digital Twin OSM generator experiments (Oita Station tests, 2026-10-21/22): flat ground, no stray trees, no sidewalks drawn over roads |
| `Plugins/CarlaTools/Content/OnroadMapGenerator/BP_OpenDriveToMap.uasset` | ground defaults to `MI_Concrete_01` (set by `map_tools/setup_generator.py`) | same |
| `Util/DockerUtils/dist/build.sh` | mode 644 → 755 | make it executable for the navmesh step |
| untracked: `Unreal/CarlaUE4/.ignore`, `carla-digital-twin-setup.md`, `error.txt`, `test.txt`, `tesy.txt` | an editor search-ignore list, the setup guide, and old build logs | — |
| `Util/DockerUtils/dist/RecastBuilder` (git-ignored) | a RecastBuilder binary copied in (2026-10-21) | CARLA's navmesh step expects tools here; **FBX2OBJ is missing**, which is why the walker navmesh is made by hand (5.12.6) |

Other state in the source tree:

- `Build/` holds the built dependencies, including
  `Build/recast-c10-install/bin/RecastBuilder`, which `maps/build_map.sh`
  step 2c uses.
- `Import/` holds the staging folders (`Circuits`, `DeliveryModels`, `Japan`,
  `LastMileModels`, `Singapore`, `UserCars`, `roadpainter_decals.json`).
  `Dist/` holds the cooked outputs (`<Pkg>_cooked`, `<Pkg>_factory`,
  `UserCars_vehicles`). They are rebuildable intermediates.
- The engine plugins `carla-digitaltwins` and `carla-mesh-generation` are
  untracked and were not compiled (as of 2026-10-21).
  `~/Documents/Dinesh-carla-copy/disabled-plugins/HoudiniEngine` was moved
  out of the way **(why: not recorded)**.

To reproduce on a new PC: clone the branch, then copy the five modified
files from the old tree, or apply a patch made there:
`git -C ~/Documents/Dinesh-carla-copy/carla diff > carla-src.patch`, then
`git apply carla-src.patch` on the new PC. The binary `.uasset` doesn't
survive a text patch, so copy that file, or rerun `map_tools/setup_generator.py`.

### 4.4 The Digital Twin side work (`~/Documents/Dinesh-carla-copy/map_tools`)

This is separate from the rig's own map builder. It covers CARLA's built-in
OSM generator (`UW_OnRoadMainWidget`).

- `setup_generator.py` (already run): ground `MI_Concrete_01`, 12 extra facade
  material instances in `FacadeMaterials/Custom`.
- `finish_map.py MAP` (random facades; run headless:
  `UE4Editor ... -run=pythonscript -script="finish_map.py MAP" -nullrhi`).
- `clean_tiles.py MAP X_Y ...` empties broken tiles. Also there:
  `add_sidewalks.py`, `set_ground.py`, `osm_server/`.
- `BP_TLOpenDrive_LHT` / `_RHT` were created as copies of `BP_TLOpenDrive`.
- Pitfalls: use an `overpass-api.de/api/map?bbox=W,S,E,N` URL (an
  openstreetmap.org export link crashes the editor); tiles are 500 m;
  building names with `! ' &` crash a tile (sanitise them); don't generate
  while in Play mode.
- `OitaStationTest1` was still pending (tiles 2_2, 1_3). Backups are in
  `~/Documents/Dinesh-carla-copy/backups/`. **None of these test maps are in
  the packaged CARLA.**

### 4.5 The UE5 checkout `~/Downloads/carla` (not used)

Branch `ue5-dev`. Local changes:
- `Unreal/CarlaUnreal/CarlaUnreal.uproject`: rewritten to a short form with
  `RenderDocPlugin`, `PythonScriptPlugin`, `CarlaTools` and `StreetMap`
  **disabled**, and several plugins removed (EditorScriptingUtilities,
  AlembicHairImporter, HairStrands, SunPosition, ModelingToolsEditorMode,
  RigLogic, Volumetrics, PerformanceMonitor). Why is not recorded
  **(unverified)**.
- `Util/DockerUtils/dist/build.sh` made executable.
- Untracked: `DELIVERY_SIM_GUIDE.md` (an 808-line guide to the delivery sim,
  kept here only for reference), `Import/Japan/` (`OitaStation.fbx`,
  `.xodr`, `.xodr.before_ring`: a copy of the Oita Station staging files)
  and `Import/roadpainter_decals.json` (road-painter decal amounts for
  OitaStation).

`maps/build_map.sh` checks for a `CarlaUnreal.uproject` and a
`Util/Tools/Import.py`, so it could point at a UE5 tree (`CARLA_SRC=...`).
This has never been used. **You don't need this folder for the rig.**

---

## 5. Every rig feature

All paths below are relative to `~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/`
unless they start with `~` or `/`. Code, comments and docs in the rig are
bilingual EN + JA (a project rule in `CLAUDE.md`). Result figures are
English only.

### 5.1 The pipeline (`run_pipeline.py`)

**What it does:** one command starts everything (1.4) and tears everything
down on ESC or Ctrl+C.

**Changes since the original:**
- PRIME offload (`--gpu`).
- Reuses a CARLA that already answers on port 2000 (`--no-sim`), and on
  reuse forces async mode to clear a frozen synchronous world left behind
  by a dead tick master.
- A freeze watchdog on `traffic_heartbeat.json` (`--stall-timeout 20`,
  `--stall-grace 90`) restarts the tick master.
- Prints the kernel's `NVRM: Xid` lines when CARLA dies.
- Bounded restarts: 3 per helper per run.
- `--delivery N` / `--delivery-args`, `--drone-cams`, `--ffb` /
  `--ffb-strength`, `--keyboard`, `--start-vehicle`, `--res` / `--fps` /
  `--mirror-fps` / `--viewer-fps` / `--sensor-fps`, `--cyclists`,
  `--wheelchair-share`, `--runners`, `--no-traction-fx`, `--no-hybrid`,
  `--agent`, `--autopilot`.
- Hybrid physics is **off on keep-left maps**. It reads maps.json
  `keep_left` *or* `left_hand_traffic`; before 2026-11-03 it read only the
  wrong key.
- With `--autopilot` the hero uses its own TrafficManager port (tm-port+1).
  Otherwise `set_autopilot()` took TM 8000 from generate_traffic, and the
  traffic never moved.
- **Map load (2026-11-09):** `load_world_map()` loads the map in a throwaway
  client with a 120 s timeout and then checks the loaded map's name (also for
  F5 and the dashboard's map switch). `PythonAPI/util/config.py` waited only
  10 s: on the extended OitaStation it reported a failure while the server
  was still loading, and every client started next timed out. Tests:
  `tests/test_map_switch.py`, `tests/test_pipeline_control.py`.
- `--no-road-overlay` (5.21), `--no-weather-cycle` (5.17).

**All options (from the argument parser):**
`--host --port(2000) --tm-port(8000) --map --vehicles(30) --walkers(10)
--cyclists(3) --wheelchair-share(0.15) --runners(0.1) --no-traffic
--no-dashboard --no-traction-fx --dashboard-port(5000) --no-sensor-viewer
--sensor-viewer-res(1440x720) --vehicle-filter(vehicle.*) --start-vehicle
--control-res/--res(1080p) --fov(100) --autopilot --agent POLICY
--delivery N --delivery-args --drone-cams --drone-cams-res(1280x720) --ffb
--ffb-strength --keyboard --no-log --no-sim --fps(60) --mirror-fps(30)
--viewer-fps(30) --sensor-fps(20) --no-hybrid --carla-args
--gpu{auto,nvidia,off} --boot-timeout(45) --stall-timeout(20) --stall-grace(90)`

**Run:**
```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
python3 run_pipeline.py --map Town02 --vehicles 60 --walkers 15
python3 run_pipeline.py --map Town05 --vehicles 0 --walkers 0 --delivery 3 --ffb
# the unattended Oita Kosen demo left running on 2026-11-03:
python3 run_pipeline.py --map OitaKosen --vehicles 9 --autopilot --walkers 20 --cyclists 0 --delivery 3
```

**Gotchas:**
- On shutdown the pipeline runs `pkill -9 -f CarlaUE4-Linux-Shipping`,
  which kills **every** CARLA on the machine, including test servers on
  other ports (unless `--no-sim`).
- To stop it, signal the Python `run_pipeline.py` process, not a wrapping
  bash. Signalling the wrapper once left two CARLAs running.
- **Never start a second CARLA server while the pipeline runs.** VRAM ran
  out on 2026-10-27 and the pipeline collapsed.
- If a helper has used up its 3 restarts, restart the pipeline or start that
  helper by hand.

### 5.2 Driving window, wheel and keyboard (`manual_control_steeringwheel G923.py`)

Mind the space in the file name; always quote it.

**Changes since the original:**
1. **Keyboard driving (2026-08-23).** Before this, `DualControl.__init__`
   raised `ValueError("No joystick detected")` without a wheel, and W/A/S/D
   never worked even with a wheel, because the wheel code overwrote the keys
   on every frame. Now exactly one input path runs per frame. With no wheel
   it falls back to the keyboard, and `--keyboard` forces it. The keyboard
   pedals ramp up over about 0.4 s and obey the same `pedal_tuning.json`
   caps. S engages reverse only once the car has stopped. That way keyboard
   sessions give telemetry comparable to wheel sessions. Test:
   `tests/test_keyboard_control.py` (22 checks, no server needed).
   Section 1 fails while a real G923 is plugged in; that comes from the
   environment, not from a bug.
2. **Steering range fix** (`wheel_config.ini`): `steer_gain = 1.83`,
   `steer_curve = 0.5` (it was 1.0 / 1.1). Keep `steer_gain = 1/tan(steer_curve)`
   so the full 900° lock is used. Before, steering maxed out at ±321°. The
   brake axis changed `1` → `3` (throttle 2). Paddles `gear_up = 4`,
   `gear_down = 5` and `transmission = 9` were added. `wheel_range_deg`
   (default 900) drives the on-screen steering wheel.
3. **Paddle shifting:** in automatic, the first paddle press switches to
   manual. Turn signals: wheel buttons 10 = right, 11 = left (pressing again
   cancels). The state goes to `turn_signal.json`, and the car's
   `LeftBlinker`/`RightBlinker` light state is set at once.
4. **Per-car driver eye** (`cars/car_views.py`, `cars/car_views.json` for
   your own cars, `cars/carla_views.json` for all 41 CARLA vehicles, each
   checked on camera). Nudge the eye live with **Ctrl+I/K** (forward/back),
   **Ctrl+J/L** (left/right) and **Ctrl+U/O** (down/up), 2 cm per press,
   saved per car. Hood and Chase views scale with the vehicle size.
5. **Steering-wheel prop** on your own cars: `static.prop.<name>steer` is
   spawned on the hub and rolled 1:1 with the G923 (positive roll =
   clockwise = right, checked on camera).
6. `--start-vehicle`, Backspace cycles through vehicles alphabetically
   (Shift goes back), `--res`, `--fps`, `--mirror-fps`, and `--tm-port` for
   the hero's autopilot.
7. **HUD crash fix:** `sorted(vehicles)` crashed when two cars were at equal
   distance, which killed the whole pipeline. Fixed with `key=lambda dv: dv[0]`.

**Keys:** W/↑ throttle · S/↓ brake (reverse once stopped) · A/D steer ·
SPACE handbrake · TAB camera (Driver Eye / Hood / Chase) · N mirrors ·
Z/X glance · Q reverse · M manual/auto · `,` `.` gears · P autopilot ·
C / Shift+C weather · `[` `]` throttle strength (Shift: brake) · F5 map ·
F11 fullscreen · R record · F1 HUD · H help · ESC quit.

**Axis calibration:** `calibrate_wheel.py` asks you to press each control,
sees which axis moves (PC mode `c266` vs PS4 mode `c267`), and writes
`wheel_config.ini`:
```bash
~/Downloads/CARLA_0.9.16/carla-venv/bin/python calibrate_wheel.py
```

### 5.3 Pedal tuning (`pedal_tuning.json`)

This was already in the original. The formula is
`output = strength × press^curve`. Change it from the dashboard
(Settings › Driving › Pedal Tuning) or with `[`/`]`. The file is re-read
about once a second. The current values on this machine are
`throttle_max 1.0, throttle_curve 0.7, brake_max 1.0, brake_curve 2.2`
(README defaults: throttle 80 % / 1.8, brake 100 % / 2.2). The file is
git-ignored and created on first run.

### 5.4 Force feedback (`carla_ffb.py`, `ffb_settings.py`, `ffb_tuning.json`, `ffb_presets.json`)

**What:** physics-based FFB for the G923 through evdev. It only reads the
world and never ticks it. What you feel:

- **Self-aligning torque** from a Fiala brush tyre model, with the slip
  angle computed from CARLA's velocity, yaw rate and the wheel angle. It
  goes light just before the front tyres let go, and the wheel
  countersteers by itself when the rear steps out.
- **Weight transfer.**
- **Kingpin centring and scrub friction** at low speed.
- **Kerb hits**, detected by checking the OpenDRIVE lane type under each
  front wheel.
- **Road texture**, bumps, **collision jolts**, and a soft **end stop**.

Damping and friction run as hardware effects in the wheel driver (about
1 kHz). With new-lg4ff they are scaled by `/sys/class/hidraw/<dev>/device/damper_level`.

**Why it is live-tunable (2026-10-27):** the user wanted smooth recentring
like Forza Horizon, with every variable adjustable from the dashboard. Every
tunable is a row in `ffb_settings.py` `PARAMS`: key, default, min, max,
step, group, EN/JA label and help. The defaults are the "forza_horizon"
preset. The other built-in presets are `previous`, `light` and `heavy`.
User presets are saved to `ffb_presets.json` (this machine has one named
"2"). Values live in `ffb_tuning.json`, which `carla_ffb.py` re-reads twice
a second. The live wheel status goes to `data/ffb_status.json`. Smooth
recentring comes from four things: a return damper, a centre damper, a wider
min-force blend and a torque slew limit. Test: `tests/test_ffb.py` (uses a
toy rim model).

**Run:**
```bash
../../carla-venv/bin/python carla_ffb.py [--strength 1.0] [--verbose]    # or run_pipeline.py --ffb
```
`carla_ffb.py` is the user's own script. It runs permanently as a
supervisor plus a `--child` worker. **To load new code, send SIGTERM to the
`--child` worker only**, and the supervisor respawns it. SIGINT makes the
child exit with rc 0, and then the supervisor quits too. Tuning changes
need no restart. `run_pipeline.py --ffb` sees a running copy and doesn't
start a second.

Needs: `pip install evdev` in the venv, and the `input` group (2.7).

### 5.5 Vehicle performance tuning (`vehicle_tuning.py`, `vehicle_tuning.json`)

**What (2026-10-27):** multipliers for brakes, engine power, top speed,
drag and handbrake, applied to the cars by the dashboard's worker thread
(card "Vehicle Performance" in Settings › Driving). Current values:
`brake_scale 1.5, power_scale 1.3, top_speed_scale 1.25, drag 1.0,
handbrake 1.0, brakes_all 1, engine_all 0`.

**How:** CARLA's `get_physics_control` returns the **last applied** values,
so scaling those compounds (×2 then ×0.5 = ×1). That was a real bug:
lowering a slider did nothing. Factory values now come from a probe car
spawned at z = −500 with role `vt_probe`, cached in
`data/vehicle_factory.json`. `apply_physics_control` stops the car dead,
so the same batch also sends `ApplyTargetVelocity`. carla_ffb's tyre-grip
write goes through `vehicle_tuning.apply` as well. Pods are skipped while
`kinematic` is set in the delivery state. Test: `tests/test_vehicle_tuning.py`.

### 5.6 The dashboard (`dashboard.py`, Flask on :5000)

**Tabs** (the original had only Dashboard and Statistics):

| Tab | Content |
|---|---|
| **Drive** (`#dash`) | Speed, Driver Inputs, Safety/ADAS (TTC/THW), Driver Performance (SDLP, steering reversals, jerk RMS, speed compliance), Lane Position, Driving Events, Speed History vs Limit, G-G diagram, Acceleration, Vehicle Dynamics, Yaw Rate & Steering, Lane/Road, Weather, **Season & wind / 季節と風** (added 2026-11-04), Trip, Collision Intensity, Situational Awareness (80 m), World Trail, GNSS/IMU |
| **Vehicle** (`#veh`) | vehicle status, sensors, impacts |
| **Delivery** (`#drone`) | city map, deliveries, drones, pods, events, LastMile pod "Take over" animation player and deck |
| **Statistics** (`#stats`) | session tiles, a per-session bar chart, every saved session by day |
| **Settings** (`#settings-driving`, `-ffb`, `-delivery`) | Driving (Pedal Tuning, Vehicle Performance), Force feedback (sliders for every FFB parameter, presets), Delivery sim (wind, pod shell view, "CARLA tooltips") |

**HTTP routes:** `/telemetry`, `/delivery/state`, `/delivery/map`,
`/delivery/pod` (POST), `/delivery/wind` (POST), `/tuning` (pedals),
`/ffb`, `/vehicle_tuning`, `/season`, `/signal` (turn signal: POST
`{"side":"left|right|off"}`), `/sessions`, `/download` (CSV), `/shutdown`, `/`.

More: EN / 日本語 switch (`?lang=ja`); `?alerttest=1` shows a test alert
banner. A turn signal makes a broken-line crossing count as a lane change.
Data goes to `data/<date>/<time>/telemetry.csv` (30 columns, about 10 Hz;
the header is now flushed at open, a fix for 0-byte files) and
`summary.json`. Options: `--host --port --dashboard-port --log-dir
--no-log --delivery-live-dir`.

**Delivery City Map speed (2026-11-06):** the map's lanes, markings and
buildings are built once per map as a few `Path2D` objects in world metres,
drawn with one transform, onto an image that covers the view plus half a view
on each side. Panning slides that image, zooming stretches it and redraws it
sharp 140 ms after the wheel stops, and mouse events redraw at most once per
frame. OitaKosen (2942 lanes, 3190 markings, 1208 buildings): panning went
from 7.7 to 100 frames/s, a full redraw from 7.5 ms to 1.3 ms.

### 5.7 Skid marks (`traction_fx.py`)

**What (2026-10-28):** dark debug-line streaks behind each sliding tyre,
wider the harder it slides, fading after `--mark-life` (25 s). Since
2026-11-06 a slide is the rear-axle slip angle over 8° (from velocity and yaw
rate), or a tyre slip ratio over 0.3 held 0.3 s with a hard pedal (>= 0.9) or
the handbrake. The old per-tyre rule (slip angle 9°, ratio 0.3) drew marks
behind a car on autopilot: CARLA's tyre slip reads 0.3-0.5 on a gripping car
whose pedals change every tick (TM-driven Tesla and RX-7, median 0.31-0.33 at
30 km/h). The pipeline starts it; turn it off with
`--no-traction-fx`. Options: `--host --port --mark-life --role`. Test:
`tests/test_traction_fx.py`.

**The look (2026-11-06):** CARLA's debug lines are round tubes and only
pure black doesn't glow; one thick line per tyre lay on the road like a
black hose, stepped at every joint. Each mark is now 7 thin lines (0.018 m)
across a 0.22 m band, perpendicular to the tyre's travel, drawn every 0.25 m:
a light slide draws the middle lines, a hard one all; random gaps make it
patchy; each line lives a random 35-100 % of `--mark-life`, so marks fade.
No frame-rate cost (30 frames/s idle and during a slide).

**Tyre smoke was built and then removed at the user's request** (it looked
like white puffballs). The code, `fx/` and the FXModels package were
deleted. Don't add it back unless asked.

**Two CARLA traps:** debug lines are self-lit, so any colour except pure
black (0, 0, 0) glows white on camera; and `carla.Transform.transform(loc)`
**changes `loc` in place**, so pass a fresh copy.

### 5.8 Traffic (`generate_traffic.py`) and lane spawns (`lane_spawns.py`)

**What changed in `generate_traffic.py`:**
- It is **the tick master** (sync, `--sim-fps` default 30). It probes for a
  real tick before trusting `synchronous_mode`, restores async on exit
  (also on SIGTERM), and writes `traffic_heartbeat.json` every 0.5 s.
- Spawns and destroys in chunks (`--spawn-chunk 5 --spawn-pause 0.5`;
  `0` means all at once) because of the GPU crash (2.5).
- `--hybrid` mode.
- `--driver-profile` / `--profile-map` / `--error-scale` / `--passive` for
  the RL arm B.
- `--cyclists`, `--wheelchair-share`, `--runners`, `--respawn`,
  `--car-lights-on`, `--hero`, `--no-rendering`.
- Walkers retry their spawn and are lifted 1 m (2026-11-03).
- Uses `lane_spawns.lane_spawn_points()`.
- `--parked-cars auto|none`, `--parked-max N`: parked cars on maps with
  `maps/build/<Map>/parked_cars.json` (5.23).
- `--bus-schedule auto|none|<folder>`, `--bus-time`, `--bus-day`,
  `--bus-max`: timetabled buses on maps with maps.json `bus_schedule` (5.20).
- `npc_only` vehicles (the Oita bus) are never picked for random traffic
  (`own_cars.py`).

Several "Spawn failed because of collision" lines at startup are normal.

**`lane_spawns.py`:** a map's spawn points are baked in at import. When the
road logic is replaced later (Oita Station's ring road), some points sit off
the road or face the wrong way. This keeps only points on a driving lane,
outside junctions, moved to the lane centre and turned with the lane. If
too few are left, it returns all of them.

### 5.9 The unstick watchdog (not in the repo)

The TM deletes a car that stays idle for 90 s, and on OitaKosen TM cars
wedge in tight junctions (9.2). For the 2026-11-03 demo a **scratch
script** was written:
`/tmp/claude-1000/-home-inouelab-Downloads-carla/2fc4bb2b-.../scratchpad/unstick_watchdog.py`.

Every 5 s it moves a car (or the hero) that hasn't moved 1 m in 45 s, and
isn't at a red or yellow light, 10 m ahead on its lane. It keeps 9 TM cars
(`TARGET`) and 20 walkers. It skips odd blueprints as replacements (race
cars, pods, trucks, ...) and logs to `unstick.log`. It lives in `/tmp`, so
**it dies with a reboot and is not in the repo.** If you want it on the new
PC, copy it into `hil_sim/` and fix its hard-coded paths. The real fix is a
junction margin in the map layout (open item).

### 5.10 The delivery simulation (`delivery_sim.py`, `delivery/`, `docs/DELIVERY.md`)

**What:** last-mile delivery on any map. Batches of 3–5 orders arrive at
random times. Each order is face-to-face (the pod drives to the building)
or by drone (the pod stops at a hub point and launches drones). Several
vans: each batch goes to exactly one van (`planning.Dispatcher`). Requested
2026-10-26 on Town03; checked on Town01–05, Town10HD, OitaStation and
OitaKosen.

**Modules:** `planning.py` and `drone.py` must stay free of `carla` imports
(that is what lets `tests/test_delivery.py` run without a server).

| Module | Job |
|---|---|
| `backends.py` | how the drone exists in the world; `DroneUnit` pool |
| `battery.py`, `wind.py`, `yard.py` | pure models: battery, wind with a Drift controller, off-road EV charging yards |
| `station.py` | charge station, standby drones, swaps, recovery, wind hold |
| `pod.py`, `rig.py` | the artist's pod with jacks, roof hatch and side port; pivots and poses |
| `lmp_pod.py`, `lmp_rig.py`, `lmp_van.py` | the LastMile LMP-4 pod: its parts, an animation ported from `LastMile_Pod_3D.html` (timeline.js / direct.js), on-road behaviour |
| `boxes.py` | box stations placed in the world for face-to-face orders on keep-left maps |
| `route.py` | `SafeRoutePlanner`: a GlobalRoutePlanner that can't run away (on Oita Station it once ate 59.6 GB) |
| `traffic.py` | in-process background traffic |
| `live.py` | file link to the dashboard (`data/delivery_live/`) |
| `hotkeys.py` | reads Q/E/X pressed in the CARLA window through X11 `XQueryKeymap` (with an `XSetErrorHandler`, because Xlib's default handler exits the process) |

**Behaviour and bug history (from the memory notes):**
- **Sync by default** (`--async` to opt out). In async, `BasicAgent` weaved
  off the road.
- Tick once after spawning: a fresh vehicle reports (0, 0, 0) until the next
  tick. Each leg is routed from `car.get_location()`.
- The box prop docked on the roof shoved the van 11 m, so it is solid only
  more than 3 m above the roof.
- `max_throttle 1.0` (Town03 has 17° hills); a stuck van is nudged 6 m along
  its route, and after 5 failures it is placed at the stop.
- Each leg's drone height = 8 m above the tallest obstacle within 5 m of its
  path (at least 15 m).
- Pre-flight raycasts (`SkyCheck`, 17 rays) took calm-air collisions from
  5/34 trips to 0/70.
- An off-road guard re-plans the route, then resets the van onto the lane.
- Random seed by default (printed; `--seed` replays it); `--speedup` for
  real-time pacing.
- Inside the pipeline it runs `--passive` (generate_traffic owns the 30 Hz
  tick). With `--no-traffic` it becomes the tick master itself.
- Drone failures: collision, blown off course, battery died, motor failure,
  parcel torn off, no safe drop, no drone. Since 2026-10-30, drift alone no
  longer crashes a drone (`--lost-drift` default 0).
- Pod EV battery (`PodEnergyModel`, `EvChargers`). A charger visit is a
  `Stop(CHARGE, ...)`; use `Stop.batch` (None for a charge stop).
- **Town05 charging yard:** two bays behind the drone station, a scripted
  kinematic path, one pod at a time.
- **Camera:** free by default. In the CARLA window, Q/E cycle locks over
  pods → flying drones → wrecks, and X toggles the X-ray shell.
- **Keep-left maps (2026-10-30):** face-to-face orders go to box stations by
  the kerb, because the pod's side port is on its left. Box height is
  measured on the 3-D model, because OSM-map lane edges don't match the mesh.
- `Order` is an unhashable dataclass, so track orders by `.id`.

**Pods and drones (3-D models):**
1. **Artist pod and drone (2026-10-27):** `~/Downloads/Pod.fbx`, `PodV2.fbx`
   (side-port door and arm) and `Drone.fbx`, built by
   `delivery/assets/build_models.sh` into package **DeliveryModels**: props
   `static.prop.deliverypod`, `deliverydrone`, `podjack`, `podhatchfront`,
   `podhatchrear`, `dronerotor`, `parcel`, the side-port parts, and the
   drivable **`vehicle.delivery.pod`** (PodV2). Moving parts are attached
   props animated with `carla.command.ApplyTransform`. On an attached actor
   that sets the offset **relative to its parent**, and it survives driving.
2. **LastMile LMP-4 (2026-10-28, now the default pod):**
   `~/Downloads/unreal/unreal/LastMilePod.fbx` (+ `_Drone`, `_BoxStation`),
   built by `delivery/assets/build_lastmile.sh` into **LastMileModels**
   (37 props `static.prop.lmp*` + X-ray shell twins + **`vehicle.delivery.lastmile`**).
   It has 4 drones per pod, battery swap and a crane. Subsystems run on
   their own tracks (`lmp_rig.PodAnim`), so it can load while driving.
   Drones fly a `drone.Mission` to a meeting point. The animation is checked
   against positions recorded from the HTML (`tests/test_lmp_rig.py`,
   `tests/data/lmp_html_truth.json`). `--pod-model pod` goes back to PodV2.
   The dashboard "Take over" slider has **not been tested live**.

**How the builds work** (the same pattern is used for cars):
1. Blender (`prepare_models.py` / `prepare_lastmile.py`) writes FBX props
   and a pod vehicle on the **Prius skeleton**.
   - Build the rig in **cm**: scene unit scale 0.01. Otherwise the root bone
     gets 100× scale, and CARLA reports the failed spawn as "collision".
2. `Util/BuildTools/Import.py --package <Pkg>`, run in the source tree with
   `carla-venv` python.
3. `-run=PrepareAssetsForCooking` builds the props map. It forces
   `CTF_UseComplexAsSimple`, which turns the whole mesh into collision, so:
4. `strip_collision.py` (an editor Python commandlet) removes that again.
   Otherwise the attached pod tipped the car onto its side.
5. `build_vehicle.py` (editor Python): imports the vehicle onto the Prius
   skeleton, reuses `AnimBP_Toyota_Prius` and `Phys_Toyota_Prius`, makes
   wheel BPs, duplicates `BP_ToyotaPrius`, and **appends a VehicleParameters
   entry to CARLA's VehicleFactory**. Save the imported materials, because
   the editor segfaults on exit. UE Python struct-array elements are copies,
   so rebuild the list.
6. `-run=cook -map=A+B -cooksinglepackage -targetplatform=LinuxNoEditor`.
7. Cook `VehicleFactory` on its own with `-cooksinglepackagenorefs` (not the
   41 cars it lists).
8. Install: copy **only** `Content/<Pkg>` into the packaged CARLA, plus the
   cooked `VehicleFactory.uasset/.uexp`. The cooked tarball also has about
   2300 Carla/Engine dependency files, which **must not** overwrite the
   packaged build. The original packaged factory is backed up in
   `delivery/assets/backup/packaged/` (`--restore-factory` puts it back).
   `make_xray.py` makes the LMP see-through shell twins.

```bash
delivery/assets/build_models.sh [fbx_dir]          # default ~/Downloads (PodV2.fbx or Pod.fbx, Drone.fbx)
delivery/assets/build_models.sh --restore-factory
delivery/assets/build_lastmile.sh [model_dir]      # default ~/Downloads/unreal/unreal
# env overrides: CARLA_SRC, UE4_ROOT, BLENDER. Restart CARLA afterwards.
```

**Run the sim:**
```bash
../../carla-venv/bin/python delivery_sim.py --first-now                 # CARLA running
../../carla-venv/bin/python delivery_sim.py --map Town05 --vans 3
../../carla-venv/bin/python delivery_sim.py --follow drone
python3 run_pipeline.py --map Town05 --vehicles 0 --walkers 0 --delivery 3 --ffb
python3 run_pipeline.py --map Town05 --delivery 3 --drone-cams --delivery-args "--tooltips"
```
`delivery_sim.py` has about 90 options (see `--help`), for example
`--vans --seed --passive --async --speedup --pod-model --shell
solid|cutaway|xray|frame --standby-drones --wind --no-box-stations
--tooltips --stock-models --drone kinematic|airsim`. Logs go to
`data/delivery/<timestamp>/events.csv` + `summary.json`. To redeploy into a
running pipeline, send TERM to the `delivery_sim` child (it has 3 restarts).

**AirSim backend: unverified.** `--drone airsim` targets **CarlaAir**
(github `louiszengCN/CarlaAir`: CARLA 0.9.16 + AirSim 1.8.1 in one UE4.26
process, port 41451). CarlaAir is **not installed** here, so this path has
never run. Its CARLA↔NED offsets are map-specific, and only Town10HD's are
published. The fleet code also disables it ("no fleet support").

**Public repo:** the packaged-CARLA git repo's remote is
`https://github.com/FEARLESSOP/CARLA-Delivery-Sim` (see 6.2).
`~/Downloads/carla/DELIVERY_SIM_GUIDE.md` is a long walkthrough (how to run
it, how each part works, the CarlaAir plan).

### 5.11 Sensor viewer, drone camera wall, RL agent

- `sensor_viewer.py` (original): RGB front/rear, segmentation, LiDAR and
  depth views of the `hero`. Options: `--res --fps --sensor-fps
  --lidar-channels --lidar-points`.
- `drone_viewer.py` (added): a CCTV-style tile per drone, with a downward
  camera only while the drone flies (OFFLINE otherwise), 10 fps per camera.
  F11 fullscreen, Esc/Q to close. Started by `--drone-cams`.
- `agent_drive.py`: an autonomous `ego` car (`--policy autopilot|model:PATH`),
  scored with `rl/rules.py`. Uses `delivery/route.py` on imported maps.
- **RL layer** (`rl/`, `docs/RL_DESIGN.md`, `docs/RESEARCH_PROTOCOL.md`).
  Added 2026-09-14: `run_arm.py` (map → passive traffic → SAC →
  checkpoint eval; arm A = default traffic, arm B = human-error profile
  traffic), `health.py` (preflight that refuses to start if this boot logged
  `NVRM: Xid` or lockups), `paths.py` (all output under `rl/results/`),
  `config.py` (`STUDY_MAP = 'Town02'`, `EVAL_TRAFFIC = 'human_x1'`),
  `plot_progress.py`, `plot_improvement.py`, `plot_town_events.py`,
  `plot_style.py`. Machine safeguards: `no_rendering_mode`,
  `--torch-threads 4`, `nice`, line-flushed logs, the replay buffer saved
  with every checkpoint, and `--resume`.

```bash
python -m rl.extract_scenarios
python -m rl.run_arm --arm A --seed 0 --timesteps 60000
python -m rl.run_arm --arm A --seed 0 --resume
python -m rl.plot_progress
```
(Run these with the venv python. Never run them while a human session uses
port 2000.) Open issue: an untrained policy that stands still scores +888.
The reward is fixed by the protocol, so change it only with the user's OK.

### 5.12 Own cars (`cars/`, package `UserCars`)

**What (2026-10-28 → 11-09):** downloaded Sketchfab models became drivable
CARLA vehicles, each with a separate steering-wheel prop (the bus excepted).
All 12 were rebuilt in one `build_cars.sh` run on 2026-11-09:

| Key | Blueprint | Drive side |
|---|---|---|
| rx7 | `vehicle.mazda.rx7` | right |
| supra | `vehicle.toyota.supra_a70` | right |
| imiev | `vehicle.mitsubishi.i_miev` | right |
| nsx | `vehicle.honda.nsx_r` | right |
| soarer | `vehicle.toyota.soarer` | right |
| sileighty | `vehicle.nissan.sileighty` | right |
| csl | `vehicle.bmw.csl_e9` | left (LHD in its geometry, even though its folder says rhd) |
| rb16 | `vehicle.redbull.rb16` | center (F1 car, fixed eye + Cockpit/T-Cam/Chase cameras) |
| sf90 | `vehicle.ferrari.sf90_xx` | right (LHD model converted with `rhd_convert`, car guide ch. 13) |
| dawn | `vehicle.rollsroyce.dawn` | right (LHD model converted) |
| s800 | `vehicle.honda.s800` | right (as modelled) |
| oitabus | `vehicle.oitabus.erga` | **NPC only** (Isuzu Erga, 10.43 m, 9,660 kg; no interior; own physics box, 7000 Nm brakes, COM 1.1 m; car guide ch. 16). Driven only by the bus timetable (5.20) |

The four new vehicles (SF90, Dawn, S800, bus) are installed but **not yet
checked live**. `npc_only` (cars.json) keeps a vehicle out of parking, random
traffic, the "Your car" list and the hero cycle (`own_cars.py`,
`tests/test_heavy_npc.py`).

**Why:** the user wants right-hand-drive cars with a fixed first-person view
and a visible wheel that turns, for left-hand-traffic maps.

**Files:**
- `cars.json`: per-car rules: source path, `forward` axis, scale, `wheels`,
  `steering_wheel` (or `{"auto_in": [...]}` when it is merged into an
  interior mesh), mass, `tyre_width_m`, `drive_side`, `glass`,
  `glass_objects`, `blinkers`, `mirrors`, `lenses`.
- `prepare_car.py`: Blender, `blender -b -P prepare_car.py -- <cars.json> <key> <out_dir>`.
  Writes `<Name>Vehicle.fbx` (on a Prius-style armature, cm, x forward,
  origin on the ground between the axles), `<Name>Steer.fbx` (hub at the
  origin, column along +x) and `<key>.json`. Blender (x, y, z) maps to
  CARLA (x, −y, z).
- `make_glass.py` (editor Python): the window slots get
  `M_CarGlassClear` (masked, opacity 0), so the windows are **fully
  invisible**, which is what the user wants.
- `make_lights.py` (editor Python): blinker faces get `M_CarBlinker`
  (`MI_CarBlinkerL/R`; emissive × Glow **250000** × a 1.5 Hz square wave).
  The vehicle mesh is tagged `emissive`, because CARLA's BaseVehiclePawn
  sets "Left/Right Blinker" only on components with that tag. Mirrors get
  `M_CarMirror`.
- `build_cars.sh`: the whole build. `car_views.py` / `car_views.json`
  hold the eye and steering hub per car. `carla_views.json` holds the eyes
  of CARLA's own 41 vehicles. `survey/` has the survey helpers
  (`export_carla_vehicles.py`, `find_seat.py`). Sources live in
  `cars/sources/<key>/` (git-ignored; zips unzipped to `source/unzipped/`).

**Build:**
```bash
cars/build_cars.sh [car key ...]          # default: every car in cars.json; then restart CARLA
```
Steps:
1. Blender per car.
2. Import the steering-wheel props.
   - `Import.py` imports **every** `.json` under `Import/`, whatever
     `--package` says, so the script parks the other packages in
     `Import_parked` and keeps the per-car json in `Dist/UserCars_vehicles`.
3. PrepareAssetsForCooking + strip collision.
4. Vehicles: `build_vehicle.py` + `make_glass.py` + `make_lights.py` per car.
5. Cook.
   - Every `<Name>Steer` mesh is in the cook list explicitly, because the
     props map silently dropped the Soarer's.
6. Cook VehicleFactory alone.
7. Install.
8. `car_views.json`.

Logs: `cars/make_glass_<key>.log`, `cars/make_lights_<key>.log` and
`delivery/assets/build_car_<key>.log`.

**Gotchas:**
- glb textures are packed with names that have no extension. Write them to
  PNG first, or the car comes out white.
- Glass rules match name prefixes only (`glass_` also matched mirror glass).
- Opaque lamp lenses → the `lenses` rule (i-MiEV, Soarer).
- Glow 40 was invisible; 250000 works but **blooms more than CARLA's own
  MKZ** (known issue).
- The RB16 has no blinkers.
- Restart CARLA after a build. Ask before restarting the user's pipeline.

**Skill:** `carla-car-importer` (section 7) carries the whole workflow and
`car_guide/` chapters 01–16.

### 5.13 Maps

All map work lives in `maps/`. `maps/maps.json` has one entry per map.
Imports go through `maps/build_map.sh`, so read that first.

#### 5.13.1 `maps/build_map.sh <map key> <stage dir>` (the importer)

The stage dir holds `<Name>.fbx` + `<Name>.xodr` (+ `textures/`). Steps:

1. **Stage** into `<CARLA_SRC>/Import/<Pkg>`, deleting stale meshes in
   `Content/<Pkg>/Static/*/<Name>` first. A mesh left by an earlier import
   blocks the move, and the cooked map keeps the old mesh beside the new
   one. After an import, `grep -c "already exists"` in the log should be 0.
2. **Import** with only this package in `Import/` (the others are parked)
   and mesh distance fields turned off in `DefaultEngine.ini` for the
   duration (put back after).
   - **2b (foliage):** when `textures/*_leaves_summer.png` exist,
     `maps/ue_foliage.py` (PythonScript commandlet) puts the leaf meshes on
     CARLA's `M_Foliage` (masked, two-sided, sways with the wind). The
     script fails unless it logs "N leaf meshes on the foliage".
   - **2c (navmesh):** when `meshes/` exist, RecastBuilder is available and
     there is no `Nav/<Name>.bin`: build the walker navmesh with
     `osm_city/nav_crosswalks.py` + `osm_city/nav_obj.py` +
     `Build/recast-c10-install/bin/RecastBuilder`, cached in
     `build/<Name>/nav_cache/<sha1 of the road meshes>.bin`. A full Oita
     Kosen navmesh takes about 2 h.
3. **Cook** the map (`-cooksinglepackage`, distance fields off).
4. **Install** `Content/<Pkg>` into the packaged CARLA. Also copy the plain
   files the cook skips (`OpenDrive/`, `Nav/`, `TM/`), and rewrite
   `<Pkg>.Package.json` to list every map in the package.

Restart CARLA, then `client.load_world('<Name>')`. Installing over a map
that a running CARLA has loaded is not safe.

**Why there is a hand-made navmesh:** CARLA's own step uses **FBX2OBJ, which
is missing** (`Util/DockerUtils/dist`). The import reports 0 errors, but no
`Nav/<Map>.bin` is written and **0 walkers spawn**. In the OBJ, write all
vertices first and then `usemtl` + faces: CARLA's OBJ loader resets the
material on each `v` line. Without `crosswalk` areas, walkers never cross.

#### 5.13.2 Oita Station (`oita_station`, package `Japan`)

**Since 2026-11-09 the installed OitaStation is the extended map** (built
2026-11-08/09, agent-checker APPROVED, imported 2026-11-09). The user asked to
"touch the current one ... not build from scratch" and "extend it with new
info". Map guide ch. 35 has every step and trap.

- **What:** the user's v8 model is kept as an embedded **core** (maps.json
  `"core": {"fbx", "bounds": [-384, -406, 410, 412], "x_scale": 0.9983243}`,
  lifted to the terrain level core_z 2.17 m; start z 2.67) inside a city
  generated by `make_osm_city.sh` out to ±640 m: Overpass OSM
  (`~/Downloads/oita_station_ext/oita_station_ext_topo.osm`, with the
  buildings OSM lacks added from the user's topoexport by
  `osm_city/topo_buildings.py`), terrain from the same topoexport
  (`~/Downloads/oita_station_topo/...glb`), one keep-left road logic for the
  whole map with a new ring road at the edge (100 km/h).
- **Buildings:** every v8 building except the JR Oita City station and the
  railway (`core_keep`) is replaced by generated Japanese buildings; the
  layout keeps off `core_hard` (the model's streets, pavements and plazas, a
  triangle union: a raster mask stair-stepped the lot edges) and lifts lots in
  the core by CORE_LIFT 0.12 m. Facades from the v8 author's survey +
  Street View (`core_facades.py`); Japanese shop fronts with doors, vending
  machines, kanban, pots and のぼり on every Japan map (`shopfront_jp.py`,
  doors drawn by `buildings_jp`); the v8 bicycles and shop-front props of the
  replaced shops removed (`core_props.py`).
- **Core ground:** re-materialled in the generator's materials
  (`core_materials.py`; M_plot → `Terrain_Lots_Plots`; 止まれ glyphs as paint,
  cut 止まれ words dropped by `drop_cut_texts`).
- **Road logic in the core:** lane widths fitted to the model's asphalt
  (`fit_core_widths.py`, seam targets, kerb-bulge rule; reference line moved,
  t = 0 stays the kerb), seam wedges (`layout.add_seam_fill`), a drive patch
  where lanes leave the model's asphalt (`core_patch.py`: check_drive 0 in
  every class), the kept station / viaduct walls a hard limit with the gate
  `check_keep_walls.py` (no lane within 0.3 m; osm_fixes dropped the roads
  under the station), the bus pool ways kept (`access: null`) and
  `bus_bays.json` shared by every Layout. Poles, plates, pedestrian heads and
  speed posts keep off the v8 asphalt (`core_merge.core_asphalt`).
- **Parked cars:** the v8 car meshes became 440 spots (`core_cars.py` →
  `parked_cars.json`), where `parked_cars.py` parks CARLA and the user's cars
  (5.23). Buses (5.20) and lane-change marks (5.22) are on this map too.
- **Rebuild:** `maps/make_osm_city.sh oita_station --lefthand` (the core_fbx
  CUT step, about 90 min, is cached in `build/OitaStation/core/stamp.json`
  until lanes.json or the code changes).
- **Rollback:** the model-only entry `oita_station_v8` (below). It has the
  same level name, and lookups by name take the FIRST entry, so it must stay
  after `oita_station` in maps.json.
- **Open items:** 9.2.

**The original model-only build (now `oita_station_v8`, the rollback):**

- Source: an artist's 3-D model `~/Downloads/oita_station_map_v8/.../FBX_Maya_Unreal/oita_station_map_v2.fbx`
  plus its `traffic_sumo/oita_station.osm`, and `buildings.csv`.
- **Road logic:** `maps/prepare_oita.py` runs:
  - `add_ring_road.py`: the model's ring road 外周道路 is not in OSM. It is
    a rounded square, centre line 423.25 m out, 2 × 3.75 m lanes, and the
    streets are cut where they meet it.
  - **`netconvert --lefthand`** (from `carla-venv/bin/netconvert`, with a
    fixed `eqc` projection).
  - It **strips any `rule="LHT"`**. netconvert `--lefthand` already builds
    keep-left roads; stamping LHT reversed every lane, so cars drove
    backwards on the right, and hybrid mode jittered them.
  - `fix_xodr_signals.py`: netconvert's lights float 5 m up, stand one per
    lane in the road, and have controllers no junction uses. The script
    sets zOffset 0, puts one pole per approach on the kerb set back 4 m,
    and adds junction-controller links. Without those links,
    `get_group_traffic_lights()` segfaulted the server.
  - The mesh goes through Blender.

```bash
~/Downloads/CARLA_0.9.16/carla-venv/bin/python maps/prepare_oita.py --stage /tmp/oita_stage
maps/build_map.sh oita_station /tmp/oita_stage
```

- maps.json: `"keep_left": true` (pods park in the left-most lane).
  `left_hand_traffic` is only for maps that really use CARLA's LHT lanes.
- The buildings are merged tiles tagged Terrain, so delivery uses
  `buildings_csv` (CARLA x = east, y = −north).
- The level's baked spawn points are stale, so they are filtered by
  `lane_spawns.py`.
- **TM cache:** `Maps/OitaStation/TM/OitaStation.bin` came from the old
  xodr and made the TM segfault in `InMemoryMap::Load`. It was renamed
  `.bin.before_ring`. Move or rebuild the TM cache whenever the xodr
  geometry changes. (The 2026-11-09 import installs no TM cache at all.)
- `delivery/route.py` SafeRoutePlanner exists because of this map
  (runaway segments went from 118 to 0). Box sites and hubs are kept to
  the largest strongly connected part of the road graph.
- Staging copies are in `~/Downloads/CARLA_0.9.16/Import/Japan/`
  (`Japan.json`, `OitaStation.fbx`, `.xodr`, `.xodr.before_ring`).

#### 5.13.3 The OSM city builder: `maps/make_osm_city.sh <key> [--lefthand] [--no-import]`

This builds a whole city from an `.osm` file. Its first use was 2026-10-30.
The chain (from the script):

1. `osm_city/roadnet.py <osm> build/<Name> <Name> [--lefthand] [--road-fixes] [--road-style=...]`
   (carla-venv): clips to the bounds, adds a ring road, runs netconvert,
   then fix_xodr_signals.
2. `--lefthand` only: `osm_city/lht_poles.py` puts light poles to the right
   of their lanes.
3. `osm_city/lanes.py` → `lanes.json`.
4. If there is `terrain_glb`: `terrain.py` (height field), `elevate.py`
   (road elevation by network least squares), then `lanes.py` again.
5. `osm_city/build.py` (maps/.venv): layout, markings, ground, buildings,
   props, textures, landmarks, trees.
6. `signs.py` (CARLA's 274 speed / 205 give-way signals); `jp_signs.py`
   (Japanese plates, only for maps with `jartic_d`).
   Japan maps also: `jp_speeds.py` (5.18), `hokousha_crossings.py` +
   `jp_hokousha.py` (pedestrian heads).
   **Embedded core** (maps.json `core`, OitaStation): after the terrain block
   `fit_core_widths.py`, lanes / lht_poles again and the gate
   `check_keep_walls.py` (exit 1 stops the chain); after the model
   `core_merge.py`.
   Japan maps: `lane_change_jp.py` (5.22), then `shopfront_jp.py`.
7. `check_roads.py` (exit 1 if anything stands on or over a road, which
   stops the chain), then `surface_grid.py` (lawn grid for charging stations).
8. `blender -b -P osm_city/blender_build.py -- build/<Name> --fbx`
   (previews: `--render name:x,y,z:tx,ty,tz`); embedded core: `core_fbx.py`
   cuts the model (cached) and joins it with the generated FBX.
9. `build_map.sh`.

`maps/.venv` is created on the first run with `numpy pillow shapely
mapbox_earcut scipy` (now also scikit-image, used by the circuit tools).
The skill says shapely ≥ 2.1. Outputs go to `maps/build/<Name>/`. Blender
4.0 here has no OpenImageDenoise, so keep denoising off.

**The user's rules for every map:** no building over a road; no floating
wheels (the kerb collider is the visible mesh); markings in the right
places; CARLA's own signs and lights; the correct driving side
(`--lefthand`, never `rule="LHT"`); a local look with facade depth, colours
from Street View; Google Maps / Street View only as a visual reference,
never as textures. Material and mesh names must not contain "light" or
"sign": CARLA's importer filters them, and a campus tile was lost that way.

#### 5.13.4 Pasir Ris (`pasir_ris`, package `Singapore`)

From `~/Downloads/map (1).osm`, keep-left, with a ring road inside the OSM
box and along the near side of the Tampines Expressway (the expressway is
cut out). HDB service roads were retagged as residential, because
netconvert excludes cars from service roads. Hand-modelled landmarks:
`landmarks/prcs.py` (Pasir Ris Crest Secondary, traced at zoom 20 from
`build/PasirRis/prcs_trace.json`) and `landmarks/lvss.py` (Loyang View
Secondary). Size: 326 objects, 3,374,737 triangles.

```bash
maps/make_osm_city.sh pasir_ris --lefthand
```

**Deferred:** a larger Pasir Ris map (2.00 × 2.23 km, landmarks Downtown
East, White Sands, Pasir Ris Street 11 blocks 130–136, and the demolished
Coral Primary). The user took it off the queue on 2026-11-04. **Start it
only if asked.** The brief is in `maps/queue/01_pasir_ris_large.md` and
`maps/queue/pasir_ris_large_BRIEF.md`; the OSM is downloaded in
`~/Downloads/pasir_ris_large_osm`. Build it as a new map; never overwrite
`build/PasirRis`.

#### 5.13.5 Oita Kosen (`oita_kosen`, package `Japan`): terrain, landmarks, signs, trees

The first **hilly** map. Imported 2026-11-02 and updated through 2026-11-04.

- **Inputs:**
  - OSM: `~/Downloads/oita kosen/map_kosen_extended.osm`. This is the
    user's `map (5).osm` merged with an OSM API strip down to 33.2288 N
    for the 7-Eleven (`osm_south/`).
  - Terrain: two **topoexport** GLBs (`topoexport-064FAE/...glb` and the
    add-on `map-addon-topoexport-238C9F(seven))/...glb`), aligned from
    `terrain_sw`.
- maps.json flags: `speed_kmh 40`, `keep_left`, `road_fixes: true`
  (dual-carriageway lanes, near-miss snapping, medians, bus bays; **on for
  this map only**, because these fixes broke Pasir Ris),
  `road_style: "japan"` (`markings_jp.py`), `building_levels`, `osm_fixes`
  (move nodes, split ways, retag; `osmfix.py`), `landmarks`, `jartic_d`,
  `area_features`.
- **Terrain pipeline:** `terrain.py` → `elevate.py` → the ground is draped
  on a dense 2.5 m grid; rigid props are offset in z; buildings stand on
  their highest ground with a plinth. The checks run relative to the draped
  surface. Pasir Ris was checked byte-identical after this change. The
  guide covers it in chapters 11, 12 and 15.
- **Landmarks** (built biggest first, then the rest in parallel scratch
  copies, at the user's choice):
  - `landmarks/kosen.py`: 大分工業高等専門学校 campus, every building in OSM
    area 113350012.
  - `dorm.py`: 明野寮 dormitory.
  - `seven.py`: the 7-Eleven 大分明野北店, which is not in OSM as a
    building.
  - All share `onterrain.py`. Each has its own `test_*.py`.
- **Area features (2026-11-04, `landmarks/akeno.py`):** the OSM is out of
  date along the main road between the 7-Eleven and the campus, so these
  were added: greenhouses, a camphor tree row, a windbreak, 大分高専第2グラウンド
  as sand with a 12 m ball-stop net, a car park where 社宅S-1 was
  demolished, and an 8-storey off-white slab with roof solar plus a garage
  block. `AKENO_WIP=1` switches scratch builds to the `_area_features_wip` key.
- **Japanese buildings:** `buildings_jp.py` (apaato, houses with deep
  eaves, mansions, shops), `textures_jp*.py` and `surfaces_jp.py`.
- **Road signs and signals (JARTIC):** see 5.14.
- **Trees, seasons, wind:** see 5.15.
- **Walker navmesh:** made by hand (5.13.1). **The navmesh installed in
  CARLA is the pre-akeno one, reused.** `nav_cache/a5084608...bin` is a
  copy of `cfa0829...bin` (see `build/OitaKosen/nav_cache/README_stale.txt`).
  The real recast was made in scratch `recast_akeno` and should replace it.
- Size: about 7.6 M triangles with all the trees.

```bash
maps/make_osm_city.sh oita_kosen --lefthand        # the whole chain (~2 h extra if the navmesh must be rebuilt)
```

#### 5.13.6 Race circuits (`marina_bay`, `spa_francorchamps`, package `Circuits`)

Not in the memory notes, but present in the files (2026-10-28/29). Sources
are `maps/sources/marina-bay/source/singapore.glb` and
`maps/sources/spa-francorchamps/source/spa1998.glb` (git-ignored).
`maps/make_map.sh <key>` runs:

1. `extract_track.py`: the centreline from the `track_material`, the width
   from `road_materials`, the start pose from `start_line` / `pole`
   (needs scipy and scikit-image).
2. `make_track_xodr.py`: a closed ring of paramPoly3 roads, `--lanes 3`,
   `--speed` (MarinaBay 70, Spa 250).
3. `prepare_map.py` (Blender: drops the sky materials, splits the mesh by
   material into `Road_Road_*` and the like).
4. `build_map.sh`.

Staging goes to `<CARLA_SRC>/Dist/Circuits_stage/<Name>`. `test_spa.py`
checks Spa with the RB16.

```bash
maps/make_map.sh marina_bay
maps/make_map.sh spa_francorchamps
```

#### 5.13.7 Map guide

All map knowledge is in `~/.claude/skills/carla-map-generator/map_guide/`
(README + 39 chapters, each under 100 lines, at the user's request). A
symlink is at `~/Downloads/CARLA_0.9.16/map_guide`. Every new lesson goes
into a chapter and the change log (ch. 9 / 20 / 27, now 38), and open items
are kept current (ch. 8). The older `MAP_BUILDING_GUIDE.md` no longer exists.
Checks every build must pass: `check_roads` PASS, `check_drive` 0 spots in
every class, `check_paths` (every crossing end on the network),
`check_markings` 0, `check_keep_walls` on core maps (root `AGENTS.md`
section 7 has the commands).

### 5.14 Road signs and signal timing (JARTIC open data, `signal_timing.py`)

**Data** (`https://www.jartic.or.jp/service/opendata/`, monthly, cp932 CSVs
in `~/Downloads/import/`):
- **Type C** `typeC_oita_2026_09/大分県警_定義_202609.csv` + `大分県警_制御_202609.csv`:
  交差点制御情報, i.e. cycle length and splits #1–6 every 5 minutes for 40
  intersections. Intersections are given only by VICS links, with **no
  coordinates**. Mesh 493165 covers Oita Kosen.
- **Type D** `typeD_oita/大分県警_202609_k_2.1.csv`: 交通規制情報 with lon/lat
  (63 一時停止, 85 横断歩道, 115 駐車禁止, 12 指定方向外進行禁止, 98 信号機,
  112 最高速度, 11 一方通行). The user first thought type C held the signs.

**Signs on the map (maps.json `jartic_d`, imported live 2026-11-04):**
74 stops (policy "add": keep stops JARTIC lacks unless imagery shows them
gone), 178 Japanese plates modelled beside the kerb (`jp_signs.py`), CARLA's
own 206/205/274 where they exist, stub-road lights removed, and signal
controllers split into phases (`signal_phases.py`).

**Signal timing (`signal_timing.py`):** type C has no coordinates, so it is
used as an "Oita-typical" profile: the median cycle and main-phase share by
junction size, day type and 5-minute slot. Yellow 3 s, all-red 2 s,
green = cycle × share − 5 s.

```bash
../../carla-venv/bin/python signal_timing.py profile "~/Downloads/import/typeC_oita_2026_09/大分県警_定義_202609.csv" \
    "~/Downloads/import/typeC_oita_2026_09/大分県警_制御_202609.csv" --mesh 493165   # -> data/signal_timing_oita.json
../../carla-venv/bin/python signal_timing.py show [--time 08:00] [--day weekday|weekend]
../../carla-venv/bin/python signal_timing.py apply [--watch] [--dry-run]           # on the running world
../../carla-venv/bin/python signal_timing.py test
```
(Expand `~` yourself if the shell doesn't do it inside quotes.)
Since 2026-11-06 `run_pipeline.py` starts `apply --watch --only-map <Map>` on
maps with maps.json `"signal_timing"` (the profile path; OitaKosen), at every
start and again after F5 back onto that map; other maps keep CARLA's times.
`data/signal_timing_oita.json` is git-ignored: without it the helper is
simply not started. The holiday table covers 2026–2027.
**Known issue:** the cluster 54/55/252 runs at 6 phases; the real fix is to
merge it in the road logic.

### 5.15 Trees, seasons and wind (`seasons.py`, `ue_foliage.py`, `jtrees.py`)

**What (2026-11-04, checked live):** every tree on Japan-style maps is one
of five generated Japanese species (`osm_city/jtrees.py`: maple, sakura,
ginkgo, zelkova, camphor; `maple.py` came first). Each species' leaves are
separate `*<Species>Leaves*` objects on CARLA's `M_Foliage` (build_map step
2b), so they sway with the weather's `wind_intensity`.

`seasons.py` swaps each species' leaf texture live
(`World.apply_color_texture_to_objects`). The seasons are hanami, spring,
summer, autumn_early, autumn (紅葉), winter, or "auto" by date for Kyushu.
The choice is saved in `season.json` (now `autumn`, wind 30), and the
dashboard's season worker applies it again when the map reloads.

```bash
../../carla-venv/bin/python seasons.py --season autumn --wind 40
../../carla-venv/bin/python seasons.py --season auto
../../carla-venv/bin/python seasons.py --list
```

**Runtime dependency:** the textures are read from
`maps/build/<Map>/textures/<species>_leaves_<season>.png` at run time, and
`maps/build/` is **git-ignored**. Copy that folder to the new PC, or the
season swap reports "no texture".

`osm_city/jp_cone.py` makes a 700 mm カラーコーン. It is not yet a spawnable
prop (that needs the props list plus a prop cook). The user's SketchUp
reference packs (maple, cone, `~/Downloads/oita kosen/3Dmodel-sketchup`,
`maple-tree`) are **study material only**: never import or copy them.

### 5.16 Japanese rules, signals, speed posts and pedestrian signals (Japan maps only)

**What (2026-11-05/06, live on OitaKosen):** four parts, each only on maps
with `road_style: japan` / package Japan; every other map is unchanged.

| Part | Files | Guide |
|---|---|---|
| NPC rules: 止まれ full stop, yield to walkers, lane speed × 0.9, no lane change 30 m before junctions; the same for delivery vans | `traffic_profile.py`, `traffic_profiles.json`, `delivery/rules.py` | ch. 23 |
| Signal model BP_JpShingo: horizontal LED head (green-yellow-red from the left), cantilever arm; still a CARLA TrafficLight | `maps/osm_city/jp_shingo.py`, `maps/ue_jp_shingo*.py`, build_map step 2d | ch. 24 |
| Speed posts BP_JpSokudo20–80 (blue numerals) | `maps/osm_city/jp_sokudo.py`, `maps/ue_jp_sokudo*.py`, step 2e | ch. 25 |
| Pedestrian signals: two-lamp heads with countdown, phase from the car lights, walkers wait on red/flash | `hokousha.py`, `maps/osm_city/hokousha_crossings.py`, `jp_hokousha*.py`, `maps/ue_jp_hokousha.py`, step 2f | ch. 26 |

Live checks: `maps/hokousha_live.py`, `maps/jp_sokudo_live.py`,
`maps/stop_live.py [--role delivery]`. Lessons: the stock light BP scales its
pole ×1.2, so `ue_jp_shingo.py` sets scale 1 (else the arm floats 1.2 m above
the head); a walk under 5 s is not started (long police cycles); a BP-only
change needs just cook + install (build_map steps 3–4).

### 5.17 Day/night cycle and dynamic weather (`weather_cycle.py`)

**What (2026-11-06, checked live on OitaKosen):** 24 h in `day_minutes` (5)
real minutes; the sun from Oita's latitude and the date. Weather spells of
1.5-4 game hours (clear / partly / cloudy / rain / storm, `rain_chance` 25 %),
rain only after clouds, puddles that dry. Wind untouched. Settings in
`weather_cycle.json` (git-ignored, written by the dashboard card "Day &
weather" on the Drive tab, endpoint `/daycycle`), state in
`data/weather_status.json`. At precipitation >= `--rain-ground` (20)
`delivery/station.DroneFleet` crashes every flying drone (`rain`) and holds
take-offs; checked live (D1 came down at 27 %, nothing flew for the rest of the
rain). The hero's headlights follow the dark (`auto_headlights` in the driving
window), the TM cars get `update_vehicle_lights` while the cycle runs.
Street lamps: BP_JpGaito at all 1,613 OitaKosen lamp heads (build_map step 2g,
map guide ch. 28), on below 3° of sun, 150 000 lm (`lamp_lumens`), 30 frames/s
with all on. Dashboard: time slider, day length − / + (1 min to 24 h), lamps
Auto / On / Off and brightness.
Test: `tests/test_weather_cycle.py`; UE: `maps/ue_jp_gaito_test.py` (14/14).

### 5.18 Speed limits (`maps/osm_city/jp_speeds.py`, `lane_limits.py`)

**What (2026-11-06, live on OitaKosen):** the outer ring at 100 km/h
(maps.json `ring_speed_kmh`, 44 posts BP_JpSokudo100), every other road at
its real limit: JARTIC 最高速度 zones and real OSM `maxspeed` (the user's own
.osm), else the legal 60 with a centre line / 30 without. Ten 40 posts that
came from roadnet's invented class defaults were removed. NPCs drive lane
speed x 0.9 and slow for bends (`traffic_profiles.json` `speed.curve`); the
dashboard and HUD show the lane limit (`lane_limits.py`). Note: the
`speed_limit_kmh` column of the logged data is now that lane limit on your own
maps. CARLA's own `get_speed_limit()` ignores the posts on these keep-left
roads. Map guide ch. 29.

### 5.19 The launcher and the React dashboard (`launcher.py`, `web/`)

**What (2026-11-06):** `python3 run_pipeline.py` with **no arguments** starts
only the dashboard (http://localhost:5000, Simulation page). The user picks
the map (from the installed levels, as F5 does), cars, pedestrians, cyclists,
delivery pods, their car (own cars + CARLA's), force feedback, display and
advanced options, then Launch. `launcher.py` runs `run_pipeline.py <flags>
--external-dashboard` as a child, logs to `data/pipeline.log`, keeps the
options in `launch_options.json`, and returns to the Launch page when the run
ends. With flags, `run_pipeline.py` works as before.

**Live control** (`pipeline_control.py`, `data/pipeline_control.json` /
`pipeline_state.json`): while running, the dashboard switches the map (the
F5 path), restarts the traffic with new counts (the tick master is restarted,
never stopped), restarts / stops the delivery sim (pods), restarts the driving
window with another car, turns FFB / skid marks / weather cycle / sensor viewer
/ drone wall on or off, or stops. Restart-only options relaunch through the
launcher. Every option carries `live` / `map` / `restart` in
`launch_options.py`.

**Frontend:** React 19 + TypeScript (strict) + Vite + Tailwind v4 in
`hil_sim/web/` (developer guide `web/DEV.md`), React Bits components
(`src/components/bits`: Aurora, StarBorder, ShinyText, SpotlightCard, CountUp,
AnimatedList, ...), design system from the ui-ux-pro-max skill (dark slate,
green accent, Fira Sans / Fira Code bundled). `dashboard.py` is the backend only
(its 5000-line embedded page was removed) and serves `web/dist`, which is
committed, so the rig runs without Node. Node 22 LTS lives in
`~/.local/node` (official tarball, checksum verified, no sudo) and is needed
only to rebuild:

```bash
export PATH=~/.local/node/bin:$PATH
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/web && npm install && npm run build
```

Tests: `tests/test_launcher.py` (15), `tests/test_pipeline_control.py` (14),
plus the map-switch and Windows-card tests.

### 5.20 Timetabled buses (`bus_schedule.py`, `data/oita_bus/`)

**What (2026-11-08, vehicle built 2026-11-09):** real Oita buses (大分バス,
大分交通, highway buses) drive into OitaStation, call at their stops (Oita
Station platforms 1-8, 降車専用, 上野の森口, 要町 and the city stops) with the
door at the pole, and leave again, on the published timetable.
- **Data:** Oita Prefecture's GTFS-JP on BODIK
  (https://data.bodik.jp/organization/440001, **CC BY 4.0**; the credit line is
  in `data/oita_bus/sources.md` and must stay wherever the buses are shown).
  gtfs-data.jp has no Oita-city feeds; the 大分バス website forbids reuse.
  `data/oita_bus/` (git-ignored: copy it) holds `stops.json`, `routes.json`,
  `timetable.json`, `shapes.json` and `tools/` to regenerate them. Every
  national holiday runs the Sunday timetable (the feed misses 11/3 and 11/23).
- **Planning:** each stop → the kerb lane with the pole on the bus's left;
  GTFS shape vias; SafeRoutePlanner; TM `set_path` (no own lane changes, never
  a point within 4 m of a junction), Japan rules through `SpeedArbiter`.
- **Clock:** starts at the sim's shown time, then runs in real time (the day
  cycle would flood the station); the dashboard clock re-anchors it.
- **Vehicle:** `vehicle.oitabus.erga` (5.12), role `timetable_bus`; ≤ 25 at
  once; a bus with no progress for 3 min is removed.
- **Run:** on by itself on maps with maps.json `"bus_schedule": "data/oita_bus"`
  (only `oita_station`); `generate_traffic.py --bus-schedule none` turns it off.
  Dashboard card "Timetabled buses" (Drive page, `/buses`, `data/bus_status.json`).
- **Checks:** `../../carla-venv/bin/python bus_schedule.py report --xodr maps/build/OitaStation/OitaStation.xodr`
  (offline mapping), `maps/bus_live.py watch | spawn` (live), `tests/test_bus_schedule.py`.
  Map guide ch. 36. **Live check on the installed map: still to do.**

### 5.21 Road logic overlay (`road_overlay.py`)

**What (2026-11-09):** the OpenDRIVE map the TM, the autopilot and the NPCs
drive on, drawn in the game with debug lines, like CARLA's OpenDRIVE view in
the UE4 editor: road edges (red), direction arrows (keep-left shows here),
junction paths and boxes, other lane types, rules (止まれ, give way, speed
posts, lights with their stop bars, lane limits), lane-change permission,
ids, and the raw OSM ways. Drawn round the hero and round what the CarlaUE4
window's camera looks at. **O** in the driving window, or the Drive tab card
"Road logic overlay" (settings `data/road_overlay.json`). Started on every
map by run_pipeline (`--no-road-overlay`), idle until turned on. Colours are
very dim on purpose (debug lines are self-lit and bloom: a measured ladder
showed channel values 1-3 clean, 6+ glowing); FPS stays within about 2 %
(800 shapes per 2 s refresh). Live check: `maps/road_overlay_live.py`
(`--calibrate 1,3,6,12` for the colour ladder). Test
`tests/test_road_overlay.py`. Map guide ch. 37.

### 5.22 Lane-change marks from the paint (`maps/osm_city/lane_change_jp.py`)

**What (2026-11-09):** netconvert writes `<roadMark>` without `laneChange`,
and CARLA then allows lane changes everywhere (29 247 / 29 247 OitaStation
waypoints read Both). On Japan maps this step rewrites every roadMark from
the BUILT marking meshes: yellow solid between lanes (進路変更禁止) = none,
white solid / broken = both, solid kerb / centre lines = none. Run by
`make_osm_city.sh` after `core_merge`; only `<roadMark>` lines change
(`.xodr.prelanechange` keeps the old file; report `lane_change.json`). Test
`tests/test_lane_change_jp.py`. **Installed for OitaStation** (xodr copy +
cache clear, 2026-11-09); **OitaKosen is patched in its build dir, install
pending.** Neither map paints yellow lines between lanes of one direction, so
TM's lane-change links are unchanged there; painting 進路変更禁止 from JARTIC
type D is a possible next step. Map guide ch. 39.

### 5.23 Parked cars (`parked_cars.py`)

**What (2026-11-08):** real vehicles on a map's parking spots
(`maps/build/<Map>/parked_cars.json`; OitaStation: 440 spots made from the v8
model's car meshes by `core_cars.py`): CARLA's cars and the user's cars (not
the F1 car, not `npc_only`), physics off, lowered onto the ground, a smaller
car where one does not fit (sizes cached in `data/vehicle_extents.json`). A
ground check skips every spot whose ground does not match the loaded level
(a build newer than the install parks nothing). Role `parked`.
`generate_traffic.py --parked-cars auto|none --parked-max N`. Test
`tests/test_parked_cars.py`. **Live check on the installed map: still to do.**

---

## 6. Changes to the packaged CARLA folder, and how to reproduce each one

### 6.1 Everything added or replaced under `~/Downloads/CARLA_0.9.16`

| Path | Kind | Size | Rebuild with | Or copy |
|---|---|---|---|---|
| `carla-venv/` | added | — | 3.2 | don't copy (venvs aren't portable across paths) |
| `PythonAPI/hil_sim/` | added (the rig) | — | git + copy | yes |
| `CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory.uasset/.uexp` | **replaced** (lists `vehicle.delivery.pod`, `vehicle.delivery.lastmile` and the 12 own vehicles, incl. `vehicle.oitabus.erga`, rebuilt 2026-11-09) | 70 KB | the last of `build_models.sh` / `build_lastmile.sh` / `build_cars.sh` | yes (the stock one is backed up in `hil_sim/delivery/assets/backup/packaged/`, md5 `7c7ce2a7...` / `8d1ab7ef...`) |
| `CarlaUE4/Content/DeliveryModels/` | added | 25 MB | `delivery/assets/build_models.sh` | yes |
| `CarlaUE4/Content/LastMileModels/` | added | 34 MB | `delivery/assets/build_lastmile.sh` | yes |
| `CarlaUE4/Content/UserCars/` | added (12 vehicles, installed 2026-11-09) | 520 MB | `cars/build_cars.sh` (no keys) | yes |
| `CarlaUE4/Content/Japan/` (Maps/OitaStation 441 MB = the **extended** map imported 2026-11-09, Maps/OitaKosen 348 MB = import of 2026-11-07, shared Static 3.9 GB, plus Shingo / Sokudo / Hokousha / Gaito BPs) | added | 4.7 GB | `make_osm_city.sh oita_station --lefthand`; `make_osm_city.sh oita_kosen --lefthand` (rollback: `prepare_oita.py` + `build_map.sh oita_station_v8`) | yes |
| `CarlaUE4/Content/Singapore/` | added | 887 MB | `make_osm_city.sh pasir_ris --lefthand` | yes |
| `CarlaUE4/Content/Circuits/` | added | 439 MB | `make_map.sh marina_bay` / `spa_francorchamps` | yes |
| `CarlaUE4/Content/Japan/Maps/OitaStation/TM/` | none since the 2026-11-09 import (the old `.bin` was a stale TM cache; the TM builds its map at start) | — | — | — |
| `CarlaUE4/Content/Japan/Maps/OitaStation/OpenDrive/OitaStation.xodr` | replaced 2026-11-09 10:00 by the lane-change version (5.22; xodr-only install) | 11 MB | `lane_change_jp.py` | yes |
| `AGENTS.md`, `.agents/skills/` | added 2026-11-09 (Google Antigravity, section 7.4) | — | this guide | yes |
| `Import/Japan/` | added (OitaStation staging copy) | — | `prepare_oita.py` | optional |
| `map_guide` | symlink → `~/.claude/skills/carla-map-generator/map_guide` | — | `ln -s` | — |
| `.git`, `.gitignore` | added (repo `FEARLESSOP/CARLA-Delivery-Sim`) | 361 MB pack | — | — |
| `history`, `history.txt` | added | — | (shell history leftovers, not needed) | — |

**Important:** the cook outputs in the source tree's `Dist/` must not be
copied wholesale into the packaged CARLA. Only `Content/<Pkg>` + the
VehicleFactory go across (5.10).

**Rebuild order** if you rebuild everything: each vehicle build appends to
the *current* source-tree VehicleFactory and installs the cooked factory.
So run `build_models.sh` → `build_lastmile.sh` → `build_cars.sh`; the last
run's factory lists all of them. That this order reproduces today's factory
exactly is an assumption **(unverified)**.

### 6.2 The git repo of the packaged CARLA (`FEARLESSOP/CARLA-Delivery-Sim`)

`~/Downloads/CARLA_0.9.16` is a git repo, branch `master`, remote
`https://github.com/FEARLESSOP/CARLA-Delivery-Sim.git` (the delivery-sim
repo; its visibility is public according to the user's own brief). It has
**one commit**, `e181643` "Initial commit: custom maps, vehicles, and
delivery simulator" (2026-10-29), with 5991 files. The `.gitignore` tracks
only what was added: `hil_sim`, `Import/Japan`, the VehicleFactory, and
`Content/{Circuits,UserCars,LastMileModels,Japan(part),DeliveryModels}`.
It ignores the stock engine and content, the venvs, `data/`, `_out/`,
`maps/sources/`, `cars/sources/`, `build/`, and the runtime JSON files.

**Update 2026-11-04, second commit `afa41d3`:** all rig code, docs and the
small configs are now pushed. That covers `maps/osm_city/`, `seasons.py`,
`signal_timing.py`, `lane_spawns.py`, `cars/make_lights.py`, `Import/Japan`,
the current VehicleFactory, every `Config/*.Package.json` and this file.

**What the repo still does NOT have**, by the user's choice (2026-11-04):

- The cooked map and car content changed since 10-29: about 4.2 GB in 8000
  files under `CarlaUE4/Content/`. That is all of Oita Kosen, the
  re-imported Oita Station assets, `Content/Singapore/` and the rebuilt
  `UserCars`. `OitaKosen.uexp` is 114 MB, over GitHub's 100 MB limit, and
  Git LFS on a free account is only 1 GB.
  - Rebuild it with the scripts (`maps/build_map.sh`, `cars/build_cars.sh`;
    they need the sources in 6.3), or copy the folder from the old PC (8.2).
- Everything git-ignored that is still needed at run time:
  - the seasonal textures and `buildings.csv` in `maps/build/<Map>/`
  - `data/signal_timing_oita.json`
  - `data/vehicle_factory.json` (regenerated)
  - the model sources
  - your tuned values: `ffb_tuning.json`, `ffb_presets.json`, `vehicle_tuning.json`
- The traffic unstick watchdog (`unstick_watchdog.py`, `watchdog_loop.sh`).
  It still lives only in a Claude scratch folder.
- The cooked content of the 2026-11-07/09 imports (the extended OitaStation,
  the re-imported OitaKosen, the 12-vehicle UserCars). The **code** is in the
  repo through `5c1c7d2` (2026-11-09, "Rig code since 11-05: extended Oita
  Station, timetabled buses, Oita Bus, road overlay, shop fronts"; earlier:
  `10f9e2f`, `e911cb0` 11-05, `e9357e0` ... `bee4db0` 11-06). The doc updates
  made after it (this file, the AGENTS.md files, `.agents/`) are not
  committed yet.

So **cloning the repo gives you the current code but not the current cooked
content.** For a full copy of today's rig, copy the folders directly (8.2).

### 6.3 What else lives outside CARLA and must come along

| Path | What | Needed for |
|---|---|---|
| `~/Downloads/Pod.fbx`, `PodV2.fbx`, `Drone.fbx` | artist pod and drone | rebuilding DeliveryModels |
| `~/Downloads/unreal/unreal/LastMilePod*.fbx/.glb`, `README_Unreal.md` | LMP-4 export | rebuilding LastMileModels |
| `hil_sim/cars/sources/<key>/` | car models | rebuilding UserCars |
| `hil_sim/maps/sources/` | circuit glbs | rebuilding Circuits |
| `~/Downloads/oita_station_map_v8/` | Oita Station model, OSM, buildings.csv | rebuilding OitaStation (the core FBX) and the v8 rollback; `buildings_csv` at run time (delivery) |
| `~/Downloads/oita_station_ext/`, `~/Downloads/oita_station_topo/` | the extended map's Overpass OSM (+ topoexport buildings) and topoexport GLB | rebuilding the extended OitaStation |
| `hil_sim/data/oita_bus/` | bus timetable (GTFS-JP, BODIK, CC BY 4.0) + `tools/` | timetabled buses at run time (git-ignored with `data/`) |
| `~/Downloads/map (1).osm` | Pasir Ris OSM | rebuilding PasirRis |
| `~/Downloads/oita kosen/` | Oita Kosen OSM, topoexport GLBs, reference packs | rebuilding OitaKosen |
| `~/Downloads/import/typeC_oita_2026_09`, `typeD_oita` | JARTIC CSVs (previous months are removed online, so keep them) | signs and signal profile |
| `hil_sim/maps/build/` | per-map build output (textures, buildings.csv, nav_cache, photos, traces) | **run time** (seasons, delivery addresses) + fast rebuilds (navmesh cache) |
| `hil_sim/data/` | sessions, scenarios, signal profile, delivery logs | study data, RL arm B |
| `~/Downloads/pasir_ris_large_osm` | deferred map | later |
| `~/.claude/` (skills, agents, memory) | section 7 | Claude Code |

### 6.4 Hard-coded paths to change on a new PC

- `maps/maps.json`: `source` / `osm` / `terrain_glb` / `buildings_csv` /
  `jartic_d.csv` / `core.fbx` (absolute `/home/inouelab/...` or `~/...`).
- `AGENTS.md` files and `.agents/skills/*/SKILL.md`: absolute paths under
  `/home/inouelab`.
- `maps/prepare_oita.py`: `DEFAULT_FBX`, `DEFAULT_OSM`, and
  `netconvert_bin` / `venv_py` (`/home/inouelab/Downloads/CARLA_0.9.16/...`).
- `cars/cars.json`: `source` paths (`~/Downloads/CARLA_0.9.16/...`).
- Build scripts: defaults `CARLA_SRC=$HOME/Documents/Dinesh-carla-copy/carla`,
  `UE4_ROOT=$HOME/UnrealEngine_4.26`, `BLENDER=blender`. Override them
  with environment variables.
- `~/.bashrc`: `export UE4_ROOT=~/UnrealEngine_4.26`.
- Claude Code memory folder names contain the path (section 7.3).

---

## 7. AI assistant setup: Claude Code and Google Antigravity

Two agentic tools work on this rig. **Claude Code** reads `CLAUDE.md`, its
skills (`~/.claude/skills`), agent definitions (`~/.claude/agents`) and memory
(`~/.claude/projects/.../memory`). **Google Antigravity** reads none of those
by itself; it reads `AGENTS.md` files and `.agents/` folders (7.4). The
knowledge is kept in ONE place (the skills, guides and memory, all plain
Markdown) and the Antigravity files point into it with absolute paths, so
both tools follow the same rules, method, gates and checks. Whoever learns a
lesson records it in the guide chapter (and, for a user preference, in the
memory folder), and keeps `AGENTS.md` in step when a rule or trap changes.

### 7.1 The skills repo `FEARLESSOP/carla-skills` (private)

Its git root is `~/.claude/skills` itself. The `.gitignore` tracks only
`carla-map-generator/`, `carla-car-importer/`, `README.md` and `.gitignore`;
the other skills there (`ui-ux-pro-max`, `synced`) are not tracked. The repo
was renamed from `carla-map-generator` on 2026-11-04. Commits:
`c14207b` (map skill + guide + agents), `d9ffcab` (restructure as
carla-skills), `0127536` (car importer).

- **`carla-map-generator`**: `SKILL.md`, `map_guide/` (01–39), and 12 agents:
  `photo-getter`, `building-builder`, `road-checker`, `agent-checker`,
  `general-builder`, `general-painter`, `road-builder`, `road-painter`,
  `road-sign-creator`, `traffic-rules-builder`, `signal-builder`,
  `charge-station-builder`. It works in stages, with `agent-checker` as the
  gate (APPROVED/REJECTED, checking the files themselves) between them.
  Landmark build order: biggest first, then the others in parallel scratch
  copies. The user wants live updates as each stage finishes.
- **`carla-car-importer`**: `SKILL.md`, `car_guide/` (01–16), and 3 agents:
  `car-surveyor`, `car-builder`, `car-checker`.
- Later commits (as of 2026-11-09) include `6c48e5f` "Guides: extended Oita
  Station, timetabled buses, heavy NPC cars, road overlay, lane-change marks".

**Install on the new PC** (from the repo README):
```bash
git clone https://github.com/FEARLESSOP/carla-skills ~/carla-skills
mkdir -p ~/.claude/skills ~/.claude/agents
cp -r ~/carla-skills/carla-map-generator ~/carla-skills/carla-car-importer ~/.claude/skills/
cp ~/.claude/skills/*/agents/*.md ~/.claude/agents/
# or clone straight into an empty ~/.claude/skills (its .gitignore leaves other skills untracked)
```
New agent types load only in a new Claude Code session. Claude in Chrome is
used by `photo-getter` for Street View and satellite imagery.

### 7.2 Project instructions

- `hil_sim/CLAUDE.md`: architecture, placement, commands, EN+JA rule, the
  tick master, the delivery notes (mirrored for Antigravity in
  `hil_sim/AGENTS.md`).
- `hil_sim/.claude/skills/grill-me`, `grilling`: project skills from the
  original zip.

### 7.3 Memory

Claude Code keeps notes per working directory in
`~/.claude/projects/<path with / replaced by ->/memory/`. These were the
sources for this file:

- `-home-inouelab-Downloads-carla/memory/` (main rig history, 29 notes; index `MEMORY.md`)
- `-home-inouelab-Downloads-CARLA-0-9-16-PythonAPI-hil-sim/memory/` (GPU,
  CPU, monitors, RL; 11 notes)
- `-home-inouelab-Documents-Dinesh-carla-copy/memory/` (UE4 build, digital
  twin; 3 notes)

To carry them over, copy those folders. If the new user name differs,
rename the folder to match the new path (for example
`-home-<newuser>-Downloads-carla`). The notes also mention paths with
`inouelab`; they are history, so leave them as they are.

### 7.4 Google Antigravity (added 2026-11-09)

**What Antigravity loads** (from its docs, https://antigravity.google/docs/rules,
`/docs/skills`, `/docs/subagents`, read 2026-11-09):
- rules: `AGENTS.md` or `GEMINI.md`, `.agents/AGENTS.md`, `.agents/rules/*.md`
  (YAML frontmatter with `trigger` required there; legacy `.agent/rules` works),
  in the **workspace folder and every parent up to the git root**; subfolders
  below the workspace are NOT loaded. Global: `~/.gemini/AGENTS.md` /
  `GEMINI.md`, `~/.gemini/config/rules/*.md`. AGENTS.md needs no frontmatter and
  is always on. Limit 24,000 bytes per file; always-on rules share 20,000 tokens.
- skills: `<workspace or parent>/.agents/skills/<name>/SKILL.md` (frontmatter
  `name`, `description`), global `~/.gemini/config/skills/`.
- sub-agents: `.agents/agents/<name>.md` (frontmatter `name`, `description`,
  `tools`, ...), started with `invoke_subagent`, or ad hoc with `define_subagent`;
  they do not inherit the conversation.

**Files made for it** (the git root is `~/Downloads/CARLA_0.9.16`, so opening
either that folder or `hil_sim` as the workspace loads the root file):

| File | Content |
|---|---|
| `~/Downloads/CARLA_0.9.16/AGENTS.md` (~18 KB) | what the rig is, where everything lives, the user's rules, the working method, the map and car workflows with every role and its file, the gate, the checks with pass criteria, tests and venvs, the expensive traps, the current state |
| `PythonAPI/hil_sim/AGENTS.md` | the rig specifics, mirroring `CLAUDE.md` (placement, commands, restarts, architecture, maps, cars) |
| `.agents/skills/carla-map-generator/SKILL.md`, `.agents/skills/carla-car-importer/SKILL.md` | stubs that send Antigravity to the Claude skills, guides and role files |

No `.agents/agents/` files were made: the Claude agent files list Claude tool
names in `tools:`, and an unknown tool name can hang an Antigravity sub-agent.
Antigravity plays each role itself, or starts a sub-agent with the role file's
text as its prompt. The memory notes are Claude's but plain Markdown:
Antigravity is told to read them and may add notes in the same format.

**On a new PC:** these files travel with the CARLA folder (copy or git); the
skills and memory with section 7.1 / 7.3. If the user name differs, fix the
absolute paths in them (6.4).

**Working efficiently (both tools):** the method lives in the root
`AGENTS.md` sections 4-8: read the guide chapter first; develop on a scratch
copy of the build dir, then one full `make_osm_city.sh` rerun; know the CUT
(about 90 min, keyed on lanes.json) and navmesh caches; control builds to
attribute side effects; a preview image after every step; long runs in the
background watched by PID; import only after asking, then relaunch through the
launcher and live-check; record the lesson.

---

## 8. New-PC checklist, in order

**One script does all of this:** `setup_new_pc.sh` (see `NEW_PC_SETUP.md`):
`curl -fsSL https://raw.githubusercontent.com/FEARLESSOP/CARLA-Delivery-Sim/master/setup_new_pc.sh -o setup_new_pc.sh && OLD_PC=user@oldpc bash setup_new_pc.sh`.
The steps below are the manual version and the reference.

Commands assume the same layout and user folders. Change paths as in 6.4.

### 8.1 System

```bash
# 1. Ubuntu 24.04 LTS (HWE kernel). Secure Boot may stay on if you use the signed prebuilt modules.
sudo apt update && sudo apt full-upgrade
# 2. NVIDIA (Blackwell -> -open only), see 2.1
sudo apt install linux-modules-nvidia-595-open-generic-hwe-24.04 nvidia-driver-595-open
sudo reboot
nvidia-smi && ls /sys/class/drm/
prime-select query                 # on-demand on a hybrid desktop (then PRIME offload is needed, 2.3)
# 3. Tools
sudo apt install -y blender git curl build-essential python3-venv jstest-gtk joystick evtest gcc-14
blender --version                  # 4.0.2 here
# 4. G923: install new-lg4ff (https://github.com/berarma/new-lg4ff, DKMS; follow its README),
#    the wheel udev rules (99-logitech-wheel-perms.rules; Oversteer optional), and:
sudo usermod -aG input $USER       # log out / in
lsmod | grep hid_logitech_new
```
Put the G923 in PC mode. Plug in the monitors (2.6). Optional sysctl
against hard locks: `kernel.hardlockup_panic=1`, `kernel.panic=30`.

### 8.2 CARLA + the rig

```bash
# 5. Packaged CARLA 0.9.16
mkdir -p ~/Downloads/CARLA_0.9.16 && cd ~/Downloads
curl -L -C - --retry 5 -o CARLA_0.9.16.tar.gz https://carla-releases.b-cdn.net/Linux/CARLA_0.9.16.tar.gz
tar -xzf CARLA_0.9.16.tar.gz -C CARLA_0.9.16

# 6a. The rig + content, FROM THE OLD PC (gives the current state). Run on the new PC, OLD = old host:
OLD=user@oldpc
for d in PythonAPI/hil_sim CarlaUE4/Content/DeliveryModels CarlaUE4/Content/LastMileModels \
         CarlaUE4/Content/UserCars CarlaUE4/Content/Japan CarlaUE4/Content/Singapore \
         CarlaUE4/Content/Circuits Import/Japan; do
  mkdir -p ~/Downloads/CARLA_0.9.16/$d
  rsync -a --exclude carla-venv --exclude '.venv' $OLD:Downloads/CARLA_0.9.16/$d/ ~/Downloads/CARLA_0.9.16/$d/
done
rsync -a $OLD:Downloads/CARLA_0.9.16/CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory.u* \
      ~/Downloads/CARLA_0.9.16/CarlaUE4/Content/Carla/Blueprints/Vehicles/
rsync -a $OLD:Downloads/CARLA_0.9.16/.git $OLD:Downloads/CARLA_0.9.16/.gitignore ~/Downloads/CARLA_0.9.16/

# 6b. OR from GitHub (current code; cooked maps/cars since 10-29 missing, rebuild them, see 6.2):
cd ~/Downloads/CARLA_0.9.16
git init && git remote add origin https://github.com/FEARLESSOP/CARLA-Delivery-Sim.git
git fetch origin && git checkout -f -B master origin/master

# 7. venv (3.2)
cd ~/Downloads/CARLA_0.9.16
python3 -m venv --without-pip carla-venv
curl -sSLo /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py && carla-venv/bin/python /tmp/get-pip.py
carla-venv/bin/pip install carla==0.9.16 pygame numpy flask evdev shapely pillow matplotlib pandas \
    networkx gymnasium stable-baselines3 tensorboard eclipse-sumo trimesh pygltflib
carla-venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
ln -s ~/.claude/skills/carla-map-generator/map_guide ~/Downloads/CARLA_0.9.16/map_guide   # after step 12

# 8. Copy the runtime data that git ignores (6.3), at least:
rsync -a $OLD:Downloads/CARLA_0.9.16/PythonAPI/hil_sim/maps/build/  ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/maps/build/
rsync -a $OLD:Downloads/CARLA_0.9.16/PythonAPI/hil_sim/data/        ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/data/
rsync -a $OLD:Downloads/oita_station_map_v8 ~/Downloads/          # buildings_csv for delivery on OitaStation
# then fix the absolute paths (6.4)
```

### 8.3 First run and checks

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
../../carla-venv/bin/python tests/test_keyboard_control.py   # section 1 fails if a G923 is plugged in (expected)
../../carla-venv/bin/python tests/test_delivery.py
../../carla-venv/bin/python tests/test_ffb.py
../../carla-venv/bin/python calibrate_wheel.py               # writes wheel_config.ini
python3 run_pipeline.py --map Town02 --vehicles 20 --walkers 5 --keyboard
nvidia-smi                                                   # CarlaUE4-Linux-Shipping listed?
# dashboard: http://localhost:5000
python3 run_pipeline.py --map OitaKosen --vehicles 9 --autopilot --walkers 20 --cyclists 0 --delivery 3 --ffb
../../carla-venv/bin/python -c "import carla; c=carla.Client('127.0.0.1',2000); c.set_timeout(10); print(c.get_available_maps())"
```
Check that the map list has `/Game/Japan/Maps/OitaKosen`, `OitaStation`,
`/Game/Singapore/Maps/PasirRis` and `/Game/Circuits/Maps/MarinaBay`,
`SpaFrancorchamps`. Pick an own car with
`--start-vehicle vehicle.mazda.rx7`. Set the season on the dashboard
(Drive tab › Season & wind).

### 8.4 Only if you will build new assets

```bash
# 9. UE 4.26 (4.1): Epic<->GitHub link, then
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
cd ~/UnrealEngine_4.26 && ./Setup.sh && ./GenerateProjectFiles.sh && make -j$(nproc)
echo 'export UE4_ROOT=~/UnrealEngine_4.26' >> ~/.bashrc
# 10. CARLA source (4.2/4.3)
mkdir -p ~/Documents/Dinesh-carla-copy && cd ~/Documents/Dinesh-carla-copy
git clone -b ue4/0.9.16 https://github.com/carla-simulator/carla.git
cd carla && ./Update.sh                      # content (~24 GB)
git apply ~/carla-src.patch                  # made on the old PC: git diff > carla-src.patch
# copy BP_OpenDriveToMap.uasset from the old tree if you use the Digital Twin tool
make launch                                  # builds deps + editor (first run 30-60 min), opens the editor
ls Build/recast-c10-install/bin/RecastBuilder    # needed by build_map.sh step 2c
# 11. maps/.venv is created by make_osm_city.sh / make_map.sh on first use
```
Rebuild commands: 5.10 (pods), 5.12 (cars), 5.13 (maps). Restart CARLA
after any install.

### 8.5 Claude Code

```bash
# 12. skills + agents (7.1), memory (7.3)
git clone https://github.com/FEARLESSOP/carla-skills ~/carla-skills
mkdir -p ~/.claude/skills ~/.claude/agents
cp -r ~/carla-skills/carla-map-generator ~/carla-skills/carla-car-importer ~/.claude/skills/
cp ~/.claude/skills/*/agents/*.md ~/.claude/agents/
rsync -a $OLD:.claude/projects/-home-inouelab-Downloads-carla ~/.claude/projects/   # rename if the user differs
# Google Antigravity: nothing to install for the rig; AGENTS.md and .agents/ come with
# ~/Downloads/CARLA_0.9.16 (7.4). Open ~/Downloads/CARLA_0.9.16 (or hil_sim) as the workspace.
```

---

## 9. Troubleshooting, known issues and open items

### 9.1 Troubleshooting (from `docs/TROUBLESHOOTING.md`, README, memory)

| Symptom | Cause / fix |
|---|---|
| `ERROR: venv python not found` | rig nested too deep; it must be `<CARLA_ROOT>/PythonAPI/hil_sim` (3.3) |
| sim crawls, `nvidia-smi` shows no CarlaUE4, `i915 GPU HANG` | running on the iGPU; use PRIME offload (2.3) |
| driving window freezes, server signal 11, `VK_ERROR_DEVICE_LOST` | NVRM Xid; `journalctl -k \| grep "NVRM: Xid"`; keep spawn chunking; reboot after repeated faults (2.5) |
| whole PC frozen | nvidia_modeset hard lockup; `journalctl -k -b -1 \| grep -E "hard LOCKUP\|soft lockup\|NVRM: Xid"` (2.5) |
| frozen world, stale heartbeat | tick master died; the pipeline restarts it; a stale sync world is cleared on reuse |
| frozen world, fresh heartbeat | GPU crash, not the tick master |
| "CPU over 100 %" | normal: 185–400 % is healthy (2.4) |
| port 2000 in use / signal 11 at start | an old CARLA is still running; `pkill -f "[C]arlaUE4-Linux-Shipping"` (the bracket stops the pattern matching your own shell; plain `pkill -f CarlaUE4...` from a shell whose command line contains it kills that shell, exit 144) |
| RTX 5080 gone, one monitor dark | the closed NVIDIA module was installed; install `-open` (2.1) |
| NVIDIA DKMS ICE in `dwarf2out.cc` | gcc-13 bug; build with gcc-14 (2.2) |
| wheel not detected | keyboard fallback; check new-lg4ff, PC mode, `jstest-gtk`, `calibrate_wheel.py` |
| 0 walkers on a new map | no `Nav/<Map>.bin` (FBX2OBJ missing); hand navmesh (5.13.1) |
| map traffic drives backwards / jitters | `rule="LHT"` on netconvert `--lefthand` roads; strip it (5.13.2) |
| TM segfault in `InMemoryMap::Load` | stale `TM/<Map>.bin` after xodr change; move it away |
| route planner eats RAM | use `delivery/route.py` SafeRoutePlanner; test new map code under `ulimit -v` |
| imported car is white | glb textures not written to PNG |
| windows opaque | glass rule didn't match (prefixes only) |
| new prop/vehicle not spawnable | restart CARLA (blueprints are read at server start); check it is in the cook list |
| a map object vanished after import | its material/mesh name contains "light" or "sign" |
| stale old mesh next to new | leftover `Static/*/<Map>`; build_map.sh deletes it now; grep "already exists" |
| `telemetry.csv` 0 bytes | fixed (header flushed at open); otherwise the dashboard never started |
| killed your own shell | never `pkill -f <pattern>` / `grep dashboard.py` from a command line that contains the pattern; find the PID by port (`ss -ltnp \| grep :5000`) |
| measured sim speed is nonsense | use `wait_for_tick()` timestamps, not `get_snapshot()` from a second client |
| headless test server | `./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2010 -nosound`, then `load_world`; `-quality-level=Low` segfaulted on `load_world` there; never run it beside the pipeline |
| AI walker velocity 0 / far TM cars velocity 0 | API quirk / hybrid teleporting; judge motion by displacement |
| every client times out right after the map loads (big map) | old `util/config.py` 10 s map load; fixed by `load_world_map` (5.1) |
| AI changes lanes across solid lines on an own map | netconvert writes no `laneChange`; `lane_change_jp.py` on Japan maps (5.22); see it with the overlay (5.21) |
| a new client sees no actors / all lights "yellow" | sync world: read after `world.wait_for_tick()` |
| no parked cars on OitaStation | `parked_cars.json` newer than the installed level (ground check), or `--parked-cars none` |
| a UE commandlet "crashed" (segfault) | it often segfaults on exit after saving: judge by its own saved / done / OK log lines |
| `get_xodr_crosswalks.py ... No module named 'carla'` in an import log | harmless (CARLA's Import.py helper) |

### 9.2 Known issues and open items

- **OitaStation (extended map, imported 2026-11-09):**
  - live checks not done yet: traffic, parked cars, buses, signals, walkers;
  - `generate_traffic` hit a TM port 8000 bind error once on the first
    OitaStation run (restarted, "No InMemoryMap cache"); then the user went
    back to OitaKosen from the dashboard;
  - 13 core signal poles stand in gaps of the v8 asphalt;
  - the viaduct ends mid-air at the core edges;
  - tactile strips end at the drive-patch edges;
  - teardrop pads at 3 dead-end loops;
  - `check_signal_poles`: 5 mismatches;
  - the station canopy over the j228 area is accepted (the user keeps the
    station as it is);
  - core street lamps are static (gaito reads Terrain_Street only); some
    v8 lines lie up to 1.2 m off the fitted lane edges (their marks read none).
- **OitaKosen lane-change marks:** patched xodr in `maps/build/OitaKosen`,
  install pending (xodr copy + cache clear, 5.22). Painting 進路変更禁止 from
  JARTIC type D is a possible next step (neither map paints yellow lines
  between same-direction lanes).
- **New vehicles not checked live:** SF90, Dawn, S800, and the Oita bus
  (overhang ray, braking from 40 km/h, off-tracking, never parked).
- **TM cars wedge at Oita Kosen junctions.** They stray about 1.6 m (p90)
  and stick on island noses, lamps, palms, planters and a building corner.
  The TM deletes cars idle for 90 s. The workaround is the scratch watchdog
  (5.9). The real fix is a ≥ 1.5 m junction margin, with no lamps or trees
  on narrow islands (guide ch. 8 open item).
- **Floating spawn points on Oita Kosen.** Seen live on 2026-11-04 by the
  car-builder agent during the blinker tests: about 17 of the 973 spawn
  points have an invisible Terrain collision above the visible road (about
  4.7 m higher at one measured point), so a car spawned there rests in the
  air. Workaround: cast a ray down and skip spawn points where the first hit
  is well above the road. The cause in the map build is not yet found (open).
- **Stale walker navmesh on Oita Kosen:** the pre-akeno navmesh is installed.
  Replace it with the scratch `recast_akeno` result (5.13.5).
- **Blinker glow** on own cars (Glow 250000) blooms more than CARLA's MKZ.
- **6-phase signal cluster** 54/55/252: merge it in the road logic.
- Checker notes for akeno: a darker camphor row, and roof plant along the
  north edge.
- `jp_cone.py` cone is not a spawnable prop yet.
- **AirSim / CarlaAir backend** never run. The LMP "Take over" slider has not
  been tested live.
- RL: the reward lets an agent that stands still score +888 (needs the user's
  OK to change). Timing randomisation, action latency, the confrontation
  runner and the mixed-effects analysis are still open (`docs/RESEARCH_PROTOCOL.md`).
- Pasir Ris: confirm the wheel-contact test after the clean import. The
  importer gives no Buildings/Vegetation semantic tags. No LODs are made.
- **Deferred:** Pasir Ris large map. Only start it on request (5.13.4).
- Hardware: CPU stability not settled (2.4). Xid faults were reduced, not
  eliminated (2.5).
- The repo `FEARLESSOP/CARLA-Delivery-Sim` has the code through
  `5c1c7d2` (2026-11-09) but not the cooked content since 10-29 (6.2).

---

## 10. Timeline of changes

Built from the memory notes, guide change logs, file dates and the zip.
Dates are JST.

| Date | Change |
|---|---|
| 2026-07-16 | original rig zip `v2x-HIL-Carla-simulation-oita-master` (CARLA 0.9.15) |
| 2026-08-23 | packaged CARLA 0.9.16 downloaded and extracted; `carla-venv`; rig moved to `PythonAPI/hil_sim` (it was nested in `~/Downloads/carla/...`); W/A/S/D keyboard driving + 2 bug fixes + test; first live run at 13:25 (Town02, 61 vehicles, 10 walkers, dashboard, telemetry) |
| 2026-08-28 | PRIME offload re-verified on 595.84-open; new-lg4ff DKMS, wheel udev rules and Oversteer installed (file dates) |
| 2026-09-14 | GPU Xid investigation, spawn/teardown chunking, driver swaps (595-open → 610-open → 610 → 610-open), DKMS gcc-14 fix, CPU instability found, healthy CPU baseline, monitor layout; RL study-arm pipeline (`run_arm`, `health`, `paths`, `config`, plots), whole-PC lockup at 13:34 and RL safeguards; result figures English only |
| 2026-10-20 → 22 | UE 4.26 (CarlaUnreal) built; CARLA source `ue4/0.9.16` patched (Setup.sh, BuildOSM2ODR.sh) and `make launch` working; Digital Twin generator changes (OpenDriveToMap), map_tools, Oita Station test maps |
| 2026-10-26 | delivery sim built (kinematic backend, 3 bugs found and fixed, multi-van, pipeline `--delivery`, `--ffb`); discovered the iGPU problem for a whole session |
| 2026-10-27 | FFB settings live from the dashboard (`ffb_settings.py`, Forza preset); vehicle tuning; artist pod and drone props (`build_models.sh`), animated pod, battery/wind/failures, Drones tab, EV battery, Town05 yard, PodV2 vehicle + VehicleFactory replacement |
| 2026-10-28 | LastMile LMP-4 pod; skid marks (smoke built and removed); own cars RX-7, Supra, i-MiEV; driver eyes for all 41 CARLA vehicles; Marina Bay circuit (`make_map.sh`) |
| 2026-10-29 | NSX-R, Soarer, Sileighty, BMW CSL, RB16; Spa-Francorchamps; **git commit `e181643`** pushed to FEARLESSOP/CARLA-Delivery-Sim |
| 2026-10-30 | Oita Station fixes (no LHT stamp, signals, SafeRoutePlanner, buildings_csv, ring road, stale TM cache); box stations on keep-left maps; OSM city builder + Pasir Ris (PRCS, Loyang View landmarks); map guide started, then moved into the skill; Oita Kosen terrain pipeline started |
| 2026-11-02 | Oita Kosen imported (campus, dorm, 7-Eleven; Japanese buildings, road style, road fixes); landmark build order chosen; Pasir Ris large queued |
| 2026-11-03 | Oita Kosen walker navmesh by hand; pipeline fixes (keep_left key, hero TM port, walker retries); unattended demo with the scratch watchdog |
| 2026-11-04 | seasonal Japanese trees + wind + dashboard card; JARTIC signs and signal timing (road-sign-creator agent); akeno area features; blinkers, mirrors, lenses on own cars; `carla-car-importer` skill; skills repo renamed `carla-skills`; Pasir Ris large deferred; this document |
| 2026-11-05 | real drivetrains for all cars (car guide ch. 15); SF90 / Dawn blinker and LHD → RHD fixes (surveyed 11-04); commit `e911cb0` (drivetrains, the three new RHD cars, smoother road edges, outer guardrail); build work given priority over the live demo (CARLA reniced +5) |
| 2026-11-05/06 | Japanese plates per NPA, one-way/no-entry fixes; Japan NPC and delivery rules; BP_JpShingo signals, BP_JpSokudo speed posts, pedestrian signals; police signal timing auto-started; keybinds page, car_prev, F5 crash fix, dashboard Windows card; `FILES.md` index; agents traffic-rules-builder and signal-builder; day/night cycle + dynamic weather, rain grounds the drones; street lamps; skid marks fixed for autopilot; speed limits (ring 100, real/legal elsewhere); launcher + live control + React/TypeScript dashboard; three-screen window and the smooth hero autopilot (P); OitaKosen road-plateau cause found, `check_drive.py`, `recovery.py` (reverse-out); commits `e9357e0` … `bee4db0`, one-script new-PC setup |
| 2026-11-07 | OitaKosen road rework: corners rounded, elevate exact, check_drive 0/0/0/0, lights 0.5 m behind the stop line; Japanese markings audit (`check_markings.py` 81 → 0); pavement network (`pavements.py`, walkable pieces 420 → 3); OitaKosen re-imported (09:40); build_map clears `~/carlaCache` and makes its build path absolute; `surface_grid.py`; LastMile charging stations + agent `charge-station-builder`; LastMile Hub + Mini Pods and the shared parcel locker, live on OitaKosen |
| 2026-11-08 | OitaStation extended (v8 core in a generated city to ±640 m): `topo_buildings.py`, `fit_core_widths.py`, `core_facades.py` (office / car-park styles), `core_materials.py`, `shopfront_jp.py` + `core_props.py`, `core_cars.py` + `parked_cars.py`; timetabled Oita buses (`bus_schedule.py`, BODIK GTFS-JP, CC BY 4.0, dashboard card); heavy NPC vehicle keys and `own_cars.py` (car guide ch. 16) |
| 2026-11-09 | kept-wall gate `check_keep_walls.py` + osm_fixes, drive patch `core_patch.py`, cut 止まれ removed, `core_hard` triangle union → agent-checker APPROVED; **OitaStation extended imported**; all 12 cars rebuilt incl. the NPC-only Oita bus; `load_world_map` (120 s) replaces config.py's 10 s map load; road logic overlay (`road_overlay.py`, O key); lane-change marks from the paint (`lane_change_jp.py`, installed on OitaStation); commits `5c1c7d2` (rig) and `6c48e5f` (skills); `AGENTS.md` + `.agents/skills` for Google Antigravity; this document updated |
| 2026-11-09..13 | Google Antigravity ran the RL work (pod/drone brains, FA-RAMPPO recovery planner, CARLA bridge, hero autopilot cornering, Q/E camera); its write-ups are archived in `PythonAPI/hil_sim/docs/archive/antigravity_2026-11/` with the claims that do not match the logs |
| 2026-11-13 | drone judder fixed: delivery loop 4 Hz → 27-30 Hz (data-returning CARLA calls wait ~30 ms when first in a tick; cached boxes, `RayWorker`, torch 1 thread; `docs/TROUBLESHOOTING.md`), `[loop]` timing lines, smoother follow camera; docs organised: index `PythonAPI/hil_sim/docs/README.md` |

---

## 11. Things that could not be verified, and contradictions found

- **README says CARLA 0.9.15** (`hil_sim/README.md`, from the original
  zip), but the rig runs on 0.9.16.
- **NVIDIA driver:** the notes end with `nvidia-driver-610-open` 610.43.02
  (2026-09-14), but the machine now runs `nvidia-driver-595-open`
  **595.99.02** with prebuilt signed modules. When and why it went back is
  not recorded. The gcc-14 / DKMS step applies only to DKMS builds.
- **uproject plugin:** the brief said the `CarlaUE4.uproject` change adds
  the PythonScript plugin. The diff actually adds **StreetMap** and
  **GeometryCacheAbcFile** and changes `EngineAssociation`.
  `PythonScriptPlugin` is already enabled in stock 0.9.16.
- **Floating spawn points on Oita Kosen:** measured live on 2026-11-04 (section 9); cause still open.
- **Circuits (Marina Bay, Spa)** and the **RB16** are not in any memory note.
  They are described from the files only.
- **README/memory say 7 own cars**; `cars.json` and `UserCars` have **8**
  (with the RB16).
- Exact install commands for new-lg4ff, Oversteer and the udev rules, and
  the ShaderCompileWorker build command, were not recorded.
- Why the UE5 checkout's uproject was trimmed, and why HoudiniEngine was
  disabled, is not recorded.
- That the VehicleFactory rebuild order in 6.1 reproduces today's file
  exactly is an assumption.
- The visibility of `FEARLESSOP/CARLA-Delivery-Sim` (public) comes from the
  user's brief, not from a check.
- The memory note on folder placement still describes the old nested
  location as current. The later setup-state note confirms it was moved.
- **OitaKosen import of 2026-11-07:** installed at 09:40 (log
  `maps/build/OitaKosen/import_20261107b.log`, `Success - 0 error(s)`); which of
  that day's road fixes it already contains is not recorded.
- **Antigravity behaviour** was taken from its docs, not tested here: that the
  `.agents/skills` stubs are discovered, that the root AGENTS.md is loaded when
  `hil_sim` is the workspace (parent folders up to the git root), and how
  `invoke_subagent` / `define_subagent` behave. Third-party guides mention a
  "Load nested AGENTS.md files" setting; the official page does not.
- The README lists `--res` default `1080p`, while the G923 script's own
  default is written as `1920x1080`. These are the same resolution.
