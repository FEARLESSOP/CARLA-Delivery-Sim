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

# B) internet only (code, CARLA, venv; only the 10-29 cooked maps/cars/pods, see below):
bash setup_new_pc.sh
```

Preview without changing anything: `DRY_RUN=1 bash setup_new_pc.sh`.

When it finishes:

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
../../carla-venv/bin/python calibrate_wheel.py   # once, G923 plugged in, PC mode
python3 run_pipeline.py                          # the launcher -> http://localhost:5000
```

## Or let Claude Code do it

Before you start, on the new PC:

1. Install Claude Code and log in (https://docs.claude.com/en/docs/claude-code/setup).
2. If you copy from the old PC: `openssh-server` on the old PC, then on the new
   one `ssh-keygen` (if you have no key) and `ssh-copy-id inouelab@<old-pc-ip>`,
   so `ssh inouelab@<old-pc-ip> true` works without a password.
3. Start `claude` in your home folder and paste the prompt below (fill in the
   `<...>` parts; delete the old-PC lines if there is none).

```text
Set up my CARLA 0.9.16 HIL driving-simulator rig on this PC from my repo
https://github.com/FEARLESSOP/CARLA-Delivery-Sim.

1. Read NEW_PC_SETUP.md in the repo first and follow it.
2. Download setup_new_pc.sh to ~/setup_new_pc.sh and read it before running anything.
3. My old PC is inouelab@<old-pc-ip> (SSH key login works; check with
   `ssh inouelab@<old-pc-ip> true`). Use OLD_PC=inouelab@<old-pc-ip>.
   No old PC: run without OLD_PC and tell me what will be missing.
4. Run it once with DRY_RUN=1 and show me the plan, then wait for my OK.
5. Never type passwords or tokens yourself. For anything that needs sudo,
   give me the exact commands to run in my own terminal (the apt packages of
   step 1, `gh`, `usermod -aG input`, the NVIDIA driver if nvidia-smi fails),
   and run the script itself with SKIP_SYSTEM=1. For the private skills repo,
   ask me to run `gh auth login`.
6. If a step fails, read the message, fix the cause and run the script again
   (finished steps are skipped). Do not change the repo's code to get past a
   failure without asking me. Tell me when a reboot or log-out is needed.
7. At the end: make sure the self-tests passed, start
   `python3 run_pipeline.py` in ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim and
   check that the launcher at http://localhost:5000 lists the maps (OitaKosen
   with the old PC) and my own cars, then stop it with Ctrl+C. Give me the list
   of what is still manual (G923 driver, wheel calibration, monitors).
8. Do not install Unreal Engine or the CARLA source unless I ask: they are
   only for building maps and cars. Do not commit or push anything.
```

When the script has copied your Claude memory (step 6) and the skills
(step 7), restart `claude` so it picks them up.

## What the script does

| Step | What | Time / size |
|---|---|---|
| 0 | checks: not root, Ubuntu 24.04, free disk (asks for 45 GB, 55 GB with `OLD_PC`; uses ~30 / ~40 GB, the 8.3 GB tarball can be deleted after) | — |
| 1 | apt: git, curl, rsync, build tools, python3-venv, Blender, joystick tools; adds you to the `input` group (G923); checks the NVIDIA driver and the G923 kernel driver | a few min |
| 2 | downloads and unpacks the packaged **CARLA 0.9.16** into `~/Downloads/CARLA_0.9.16` | 8.3 GB download, 17 GB unpacked, 10-40 min |
| 3 | puts this repo over it with git: the rig `PythonAPI/hil_sim`, docs, the modified VehicleFactory, and the cooked content as it was on 2026-10-29 (OitaStation, the circuits, the pods, the first own cars) | a few min |
| 4 | creates `carla-venv` with every Python package the rig uses (carla 0.9.16, pygame, flask, shapely, gymnasium, stable-baselines3, CPU torch, ...) | 5-10 min |
| 5 | Node.js 22 LTS into `~/.local/node` (official build, checksum verified, no sudo) and the dashboard's npm packages; the built dashboard (`web/dist`) is already in git | 1-2 min |
| 6 | **only with `OLD_PC`**: rsync (`--delete`, the old PC wins) of the current cooked content (Japan, Singapore, Circuits maps, UserCars, DeliveryModels, LastMileModels, VehicleFactory), `hil_sim/data`, your tuning/keybinds/wheel calibration/launch settings, the run-time files of `maps/build`, `oita_station_map_v8`, and the Claude Code memory | ~10 GB (5 content + 4 `maps/build` + 1 data), depends on the network |
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
- **Claude Code itself** is not installed by the script (install it, log in,
  then step 7 adds the skills and agents).
- **Skills (private repo)**: `sudo apt install gh && gh auth login`, then run
  the script again.
- **Without `OLD_PC`**: git only has the cooked content of 2026-10-29
  (OitaStation, SpaFrancorchamps/circuits, pods, the first own cars). Missing:
  OitaKosen, PasirRis, SF90/Dawn/S800 and every change after that date. Copy them
  later (`OLD_PC=... bash setup_new_pc.sh` again) or rebuild them (needs the
  build tools below; setup guide 5.10, 5.12, 5.13).

## Do I need the CARLA UE4 fork and the CARLA source?

**Not to run the rig.** The packaged CARLA 0.9.16 (step 2) plus the cooked
content (git + `OLD_PC`) is everything `run_pipeline.py` uses.

**Only to build or rebuild assets**: `maps/build_map.sh`, `cars/build_cars.sh`,
the pod build and the `ue_*.py` steps (signals, lamps, speed posts) run inside
the Unreal editor. For that you need, on top (setup guide 8.4):

| What | Where | Size |
|---|---|---|
| Unreal Engine 4.26, CARLA's fork (`CarlaUnreal/UnrealEngine`, branch `carla`; link your Epic account to GitHub first) | `~/UnrealEngine_4.26` (`UE4_ROOT`) | ~210 GB built, 1-3 h compile |
| CARLA source `ue4/0.9.16` + `./Update.sh` content + `make launch` | `~/Documents/Dinesh-carla-copy/carla` (`CARLA_SRC`) | ~50 GB, 30-60 min first build |
| the old tree's local changes (OpenDriveToMap, BuildOSM2ODR.sh, Setup.sh, uproject): `git diff > carla-src.patch` on the old PC, `git apply` on the new one | | small |
| Python build deps (numpy, pillow, shapely, mapbox_earcut, scipy, scikit-image) | `maps/.venv`, made by `make_osm_city.sh` / `make_map.sh` on first use | <1 GB |
| full `maps/build` (`BUILD_SYNC=full`) | | ~13 GB |

Keep those paths (the build scripts default to them, or set `UE4_ROOT` /
`CARLA_SRC`). Plan ~300 GB free disk for a build PC.

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
