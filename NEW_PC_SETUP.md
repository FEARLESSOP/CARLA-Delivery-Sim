# Set up the CARLA HIL rig on a new PC (one script)

Everything below runs from one script, `setup_new_pc.sh`, in this repo
(FEARLESSOP/CARLA-Delivery-Sim). It is safe to run again: each step checks
first and skips what is already done.

## Quick start

On the new PC (Ubuntu 24.04, your normal user, internet):

```bash
cd ~
curl -fsSL https://raw.githubusercontent.com/FEARLESSOP/CARLA-Delivery-Sim/master/setup_new_pc.sh -o setup_new_pc.sh

# A) the old PC is reachable over SSH (gives the complete, current rig):
OLD_PC=inouelab@old-pc-hostname bash setup_new_pc.sh

# B) internet only (code, CARLA, venv; your own maps/cars/pods must be copied later or rebuilt):
bash setup_new_pc.sh
```

Preview without changing anything: `DRY_RUN=1 bash setup_new_pc.sh`.

When it finishes:

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
../../carla-venv/bin/python calibrate_wheel.py   # once, G923 plugged in, PC mode
python3 run_pipeline.py                          # the launcher -> http://localhost:5000
```

## What the script does

| Step | What | Time / size |
|---|---|---|
| 0 | checks: not root, Ubuntu 24.04, free disk (~45 GB, ~55 GB with the old PC's content) | — |
| 1 | apt: git, curl, rsync, build tools, python3-venv, Blender, joystick tools; adds you to the `input` group (G923); checks the NVIDIA driver and the G923 kernel driver | a few min |
| 2 | downloads and unpacks the packaged **CARLA 0.9.16** into `~/Downloads/CARLA_0.9.16` | ~15 GB, 10-40 min |
| 3 | puts this repo over it with git (the rig `PythonAPI/hil_sim`, docs, the modified VehicleFactory) | 1 min |
| 4 | creates `carla-venv` with every Python package the rig uses (carla 0.9.16, pygame, flask, shapely, gymnasium, stable-baselines3, CPU torch, ...) | 5-10 min |
| 5 | Node.js 22 LTS into `~/.local/node` (official build, checksum verified, no sudo) and the dashboard's npm packages; the built dashboard (`web/dist`) is already in git | 1-2 min |
| 6 | **only with `OLD_PC`**: rsync of the cooked content git cannot hold (Japan, Singapore, Circuits maps, UserCars, DeliveryModels, LastMileModels, VehicleFactory), `hil_sim/data`, your tuning/keybinds/wheel calibration/launch settings, the run-time files of `maps/build`, `oita_station_map_v8`, and the Claude Code memory | ~6 GB, depends on the network |
| 7 | Claude Code skills + agents (`FEARLESSOP/carla-skills`, private: needs `gh auth login` first), and the `map_guide` link | 1 min |
| 8 | if your home folder is not `/home/inouelab`: rewrites the absolute paths in `maps/maps.json`, `maps/prepare_oita.py`, `cars/cars.json` (backups `.bak`) | — |
| 9 | offline self-tests (launcher, live control, delivery, skid marks, weather, traffic rules) | 1 min |

At the end it lists what is still to do.

## Options

| Variable | Default | Meaning |
|---|---|---|
| `OLD_PC` | — | `user@host` of the old PC to copy content and settings from (SSH) |
| `OLD_HOME` | your `$HOME` | the old PC's home folder, if the user name differs |
| `OLD_CARLA_DIR` | same place as here | the CARLA folder on the old PC |
| `CARLA_DIR` | `~/Downloads/CARLA_0.9.16` | where CARLA goes on this PC (keep the default: docs and memory use it) |
| `BUILD_SYNC` | `runtime` | `maps/build` from the old PC: `runtime` (textures, json, csv, xodr, navmesh cache), `full` (~13 GB, for rebuilding maps), `none` |
| `INSTALL_NVIDIA=1` | off | also install `nvidia-driver-595-open` (RTX 50 needs the open module), then reboot and re-run |
| `SKIP_SYSTEM=1` | off | no apt / sudo |
| `SKIP_SKILLS=1` | off | no Claude Code skills |
| `SKIP_TESTS=1` | off | no self-tests |
| `DRY_RUN=1` | off | print the actions only |

Example, a different user on the old PC:

```bash
OLD_PC=inouelab@192.168.1.20 OLD_HOME=/home/inouelab BUILD_SYNC=full bash setup_new_pc.sh
```

## What stays manual

- **NVIDIA driver**: with `INSTALL_NVIDIA=1` it installs, but you must
  **reboot** and run the script again. Hybrid laptops/desktops: PRIME offload
  (setup guide 2.3; `run_pipeline.py --gpu auto` handles it).
- **G923 wheel**: the kernel driver `new-lg4ff` (DKMS, follow
  https://github.com/berarma/new-lg4ff), log out/in after the `input` group,
  put the wheel in PC mode, then `calibrate_wheel.py`.
- **Monitors, BIOS/CPU settings, sysctl against hard locks**: setup guide 2.4-2.6.
- **Building new maps or cars** needs Unreal Engine 4.26 and the CARLA source
  tree (tens of GB, an hour of compiling): setup guide 8.4. Not needed to run
  the rig.
- **Skills (private repo)**: `sudo apt install gh && gh auth login`, then run
  the script again.
- **Without `OLD_PC`**: your own maps, cars and pods are missing. Copy them later
  (`OLD_PC=... bash setup_new_pc.sh` again) or rebuild them (setup guide 5.10,
  5.12, 5.13).

## Check it works

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
python3 run_pipeline.py            # Launch page at http://localhost:5000
```

On the Launch page the map list should show OitaKosen, OitaStation, PasirRis,
MarinaBay and SpaFrancorchamps (with `OLD_PC`), and your own cars under "Your
cars". Launch, drive, and try a live change (cars, pods) on the same page.

## Troubleshooting

- **The script stops at a step**: fix what it says and run it again; finished
  steps are skipped.
- **`rsync: connection refused`**: the old PC needs `sudo apt install openssh-server`;
  test with `ssh $OLD_PC true`.
- **CARLA download stalls**: run again; `curl -C -` resumes.
- **The dashboard says "UI not built yet"**: `export PATH=~/.local/node/bin:$PATH && cd
  ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/web && npm install && npm run build`.
- More: `PythonAPI/hil_sim/docs/TROUBLESHOOTING.md`, and the full reference
  `CARLA_RIG_SETUP_FROM_SCRATCH.md` (section 8 is the manual version of this
  script).
