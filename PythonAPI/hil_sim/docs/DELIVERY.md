# Car + drone delivery mission / 車 + ドローン配送ミッション

`delivery_sim.py` runs a last-mile delivery scenario on **any map** (Town03
by default) with **one or more vans** (`--vans N`), standalone or inside
`run_pipeline.py --delivery N`.
`delivery_sim.py` は**任意のマップ**（既定 Town03）で **1 台以上の配送車**
（`--vans N`）によるラストワンマイル配送を実行します。単体でも
`run_pipeline.py --delivery N` の中でも動きます。

**Several vans / 複数台:** every batch goes to exactly one van: the oldest
waiting batch to the nearest free van (`planning.Dispatcher`). A building
never has two open orders, so no two vans ever serve the same order.
各バッチは必ず 1 台だけが担当します（最も古い待機バッチを最寄りの空車へ）。
同じ建物に未配達の注文が 2 つ入ることもないため、2 台が同じ注文を扱うことは
ありません。

**Any map / 任意のマップ:** checked on Town01, 02, 03, 04, 05 and 10HD. Maps
with fewer than 20 usable buildings get roadside addresses added. Maps whose
pavements are only in the 3-D model (no sidewalk lanes in the road logic: the
osm_city maps such as OitaKosen) find the drop point in front of each building
by raycasting from the lane edge towards it until the ground steps up onto the
kerb (`pavement_drop`, +0.4 m onto the pavement); OitaKosen: 814 of 1206
buildings, 6 s at startup.
Town01〜05・10HD で確認済み。使える建物が 20 未満のマップでは道路沿いの
地点を追加します。歩道が 3D モデルにしか無いマップ（OitaKosen など）では、
車線端から建物側へ縁石の段差を探して建物前の歩道に配達点を置きます。

## Behaviour / 動作

1. **Orders / 注文** — at random times (`--min-gap`..`--max-gap` sim seconds,
   default 20–90 s) a batch of **3–5** orders arrives. Each order is
   `face_to_face` or `drone` (`--drone-prob`, default 0.5) and targets a real
   Town03 building. The drone orders of one batch are drawn within
   `--drone-radius` (150 m) of each other.
   ランダムな時刻（既定 20〜90 秒間隔）に **3〜5 件** の注文バッチが届きます。
   各注文は `face_to_face` か `drone`（`--drone-prob`）で、Town03 の実在の建物が
   配送先です。同一バッチのドローン注文は互いに `--drone-radius` 以内です。
2. **Planning / 計画** — face-to-face orders become one stop each, at the
   driving lane next to the building. All drone orders share **one hub
   stop**: starting from the centroid of the drone orders, the car picks the
   road point (rightmost lane, outside junctions, within `--hub-radius`)
   whose round-trip drone tour is shortest. The stops are then ordered by the
   shortest road distance (CARLA `GlobalRoutePlanner`).
   対面注文は建物横の車線に 1 件ずつ停車。ドローン注文はまとめて **1 か所の
   ハブ**：ドローン注文の重心から `--hub-radius` 以内の道路点（最も右の車線・
   交差点外）のうち、ドローンの往復ツアーが最短になる点を選びます。停車順は
   道路距離（`GlobalRoutePlanner`）が最短になる順です。
3. **Driving / 走行** — the van (`role_name=delivery`) drives with CARLA's
   `BasicAgent` (obeys traffic lights and other vehicles), and stops
   `--handover-s` seconds at each face-to-face stop.
   配送車は `BasicAgent` で走行し（信号・他車に従う）、対面停車では
   `--handover-s` 秒停車します。
4. **Drone / ドローン** — at the hub the drone takes off from the van's roof,
   visits every drop in the shortest order, descends and hovers at each, then
   flies back and lands on the van. The van waits for it. **Each leg has its
   own cruise height**: `--clearance` (8 m) above the tallest building, tree,
   pole or light within `--corridor` (5 m) of that leg's straight path, but
   at least `--min-alt` (15 m). On Town03 that gives 15–35 m (median 20 m).
   `--cruise-alt N` forces one fixed height instead.
   ハブでドローンが屋根から離陸し、最短順に各投下地点で降下・ホバリングし、
   車に戻って着陸します。車はその間待機します。**区間ごとに巡航高度が異なり**、
   各区間の直線経路から `--corridor`（5 m）以内にある最も高い建物・樹木・
   ポール・信号の `--clearance`（8 m）上を飛びます（最低 `--min-alt` 15 m）。
   Town03 では 15〜35 m（中央値 20 m）。`--cruise-alt N` で固定高度。

Batches that arrive while the van is busy are queued and served in order.
With the defaults orders arrive faster than one van can serve them — raise
`--min-gap/--max-gap` for a steady state.
配送中に届いたバッチは待ち行列に入り順に処理されます。既定値では 1 台で
処理しきれない頻度なので、定常状態にするには間隔を広げてください。

## Run / 実行

From the launcher (`python3 run_pipeline.py`, no arguments) set **Delivery
pods** on the Launch page. While it runs, change the number on the Simulation
page and Apply: the delivery sim restarts with the new number of pods (0 turns
it off and closes the drone camera wall; drones in the air are lost).
ランチャーの起動ページで配送ポッドの台数を選び、実行中も変更できます。

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
# CARLA must be running (it loads Town03 itself) / CARLA 起動済みであること
../../carla-venv/bin/python delivery_sim.py --first-now            # random orders each run
../../carla-venv/bin/python delivery_sim.py --seed 482913         # replay a printed seed
../../carla-venv/bin/python delivery_sim.py --vehicles 60 --walkers 80
../../carla-venv/bin/python delivery_sim.py --map Town05 --vans 3   # three vans
# inside the full rig (traffic script owns the tick, vans follow it)
python3 run_pipeline.py --map Town05 --vehicles 0 --walkers 0 --delivery 3 --ffb
../../carla-venv/bin/python delivery_sim.py --max-batches 3
../../carla-venv/bin/python delivery_sim.py --follow drone           # camera on the drone
```

The script runs CARLA **synchronously** at 20 Hz by default (`--async` to
opt out): `BasicAgent` over-steers off the road when frames are uneven.
既定で 20 Hz の同期モードで実行します。非同期ではフレームが不均一になり
`BasicAgent` の操舵が過大になって道路を外れます。

Each run uses a random seed (printed; `--seed` replays it). 30 autopilot cars
and 40 pedestrians are spawned by default (`--vehicles`, `--walkers`,
`--walker-cross`). An off-road guard re-plans the van's route if it strays
past its lane or wedges itself, and resets it onto the lane on a second
strike; see `reroute` / `lane_reset` in the log.
シードは毎回ランダム（表示され、`--seed` で再現）。既定で自動運転車 30 台と
歩行者 40 人を生成。路外逸脱ガードが車線逸脱・立ち往生時に経路を再計画し、
再発時は車線に戻す（ログの `reroute` / `lane_reset`）。

Markers in the world: orange = face-to-face, blue = drone order,
magenta = drone hub, blue trail = drone path.
ワールド内の表示：橙＝対面、青＝ドローン注文、マゼンタ＝ハブ、青線＝飛行経路。

Logs go to `data/delivery/<timestamp>/events.csv` (every order, plan,
arrival, delivery, launch, landing) and `summary.json`.
ログは `data/delivery/<日時>/events.csv` と `summary.json` に保存されます。

## Drone backends / ドローンのバックエンド

| `--drone` | Runs on / 動作環境 | Drone / 機体 |
|---|---|---|
| `kinematic` (default) | stock CARLA 0.9.16 | the `Drone.fbx` model (`static.prop.deliverydrone`) carrying a `static.prop.box01` parcel; a drawn quadcopter if the model is missing or with `--stock-models` / ドローンモデル（未導入時は描画） |
| `airsim` | [CarlaAir](https://github.com/louiszengCN/CarlaAir) v0.1.7 (CARLA 0.9.16 + AirSim 1.8.1, UE4.26) | real AirSim multirotor via `airsim.MultirotorClient` (port 41451) / 本物の AirSim 機体 |

**`airsim` is untested** — CarlaAir is not installed on this machine.
To use it: install CarlaAir, `pip install airsim` in the venv, start
`./CarlaAir.sh Town03`, then run with `--drone airsim`.
**`airsim` は未検証**（本機に CarlaAir 未インストール）。CarlaAir を導入し、
venv に `airsim` を入れ、`./CarlaAir.sh Town03` 起動後に `--drone airsim` で実行。

CARLA ↔ AirSim coordinates (CarlaAir `guide/COORDINATE_SYSTEMS.md`):
`ned = (x + ox, y + oy, -z + oz)`. The offsets are map-specific; CarlaAir
only publishes Town10HD's `(172.20, -183.86, 27.45)`. For Town03 measure
them with `--airsim-calibrate` (needs the drone to be visible as a CARLA
actor) or pass `--airsim-offset OX OY OZ`. `--sync` is refused with
`airsim` because it would freeze AirSim's clock.
座標変換のオフセットはマップ固有で、公開値は Town10HD のみ。Town03 では
`--airsim-calibrate` で計測するか `--airsim-offset` で指定してください。

## Models / 3D モデル

The van is `Pod.fbx` attached to a hidden black `vehicle.toyota.prius` (same
length and wheelbase; the pod mesh can't drive by itself). Its moving parts
are separate props attached to the same car: 4 jacks and 2 roof-hatch doors,
animated by re-setting their offset from the car (`delivery/pod.py`,
`delivery/rig.py`). At every delivery stop: full stop → jacks down; at a drone
stop the hatch opens and the drone (rotors spinning) does one round trip per
drop, reloading inside the pod in between; then hatch closes → jacks up →
drive. Props come from `delivery/assets/build_models.sh` (Blender split →
CARLA import → strip collision → cook → copy `Content/DeliveryModels` into
the packaged CARLA). Rerun it after changing the FBX files, then restart
CARLA. `--stock-models` brings back the Sprinter van and the drawn drone.

バンは `Pod.fbx` を隠れた黒いプリウスに取り付けたもの。停車地点ではジャッキを
下ろし、ドローン地点ではハッチを開けて投下地点ごとに往復（車内で再積載）、
最後にハッチを閉じジャッキを上げて出発する。

## The LastMile pod (LMP-4) / LastMile ポッド

The default pod is the LMP-4 of `LastMile_Pod_3D.html`, from its Unreal
export (`~/Downloads/unreal/unreal/LastMilePod.fbx` + drone + box station).
`delivery/assets/build_lastmile.sh` splits it (Blender), imports and cooks
it and installs `vehicle.delivery.lastmile` plus ~37 props
(`static.prop.lmp*`) next to the earlier pod, which stays installed:
`--delivery-args "--pod-model pod"` goes back to it. Restart CARLA after a
build.

- **Animations**: `delivery/lmp_rig.py` is the HTML's timeline code ported
  to Python (same keyframes; `tests/test_lmp_rig.py` checks it against
  positions recorded from the HTML page itself). Speed: `--lmp-speed`.
- **Tracks**: the pod's subsystems (cargo, battery swapper, roof and lift,
  crane, side port) each replay their part of the HTML timeline on their own
  clock, so they can work side by side (`lmp_rig.PodAnim`).
- **Getting a drone ready** happens whenever the cargo system is free, also
  while driving: the crane puts the fullest drone that can make the trip on
  the lift, the shuttle brings the parcel and the drone winches it up (the
  parcel is in the drone only when the hook animation ends), and the swapper
  gives it the fullest pack.
- **Drone stop**: every loaded drone launches in turn (roof open, lift up,
  launch, lift down, the next drone craned up and loaded, ...). The pod does
  not wait: each drone delivers (parcel lowered on its tether) and flies to a
  **meeting point** -- the pod's next stop if that is a face-to-face stop the
  drone can reach, else this stop (the pod then waits here).
- **Face-to-face stop**: a box station appears beside the side port (the door
  stays on the side the model has, for left-hand-traffic maps); per order
  retrieve → load the port → the slide carries the tray 1.45 m into the
  station; then a pedestrian walks out, opens compartment C08 and carries the
  parcel away; the tray goes back.
- **Meeting point**: after its own work there, the pod opens the roof, raises
  the lift and lands the drones circling overhead one at a time; each is
  lowered and craned onto a bay, and the pod drives on, loading the next drone
  on the way. A drone that runs low while waiting flies to the charge station.
  `--meet-hold-s` is the waiting time each launch budgets battery for.
- **Drones**: 4 per pod (lift + bays, one bay kept free for the crane). They
  charge on deck and spare packs in the magazine, both from the pod's battery
  (`--pod-kwh`, `--drone-wh`; 4 drones + 3 packs cost ~2 % of the pod per
  minute). The city station is the backup (standbys, fast charge).
- **X-ray**: `--shell solid|cutaway|xray|frame`, X in the CARLA window, or
  the dashboard (Settings > Delivery sim). CARLA can't fade a material at runtime, so
  the shell has see-through twins that are swapped in.
- **Dashboard**: Delivery tab > LastMile pod: "Take over" a pod and scrub / play the
  whole combined cycle (drone, then direct delivery, 121 s) on it with the
  slider; its deliveries pause until "Release". Deck and magazine levels.

既定のポッドは HTML の LMP-4。HTML のアニメーションを移植し、4 機搭載・電池交換・
クレーンによる機体入替・箱型ステーションへの受け渡しを再現。X 線表示と
ダッシュボードのスライダーにも対応。`--pod-model pod` で以前のポッドに戻せる。

## Batteries, wind, failures / バッテリー・風・故障

One drone per van plus `--standby-drones` at a charge station (sidewalk pads
near the city centre). Batteries drain in flight (`--battery-drain`, more
with a parcel, climbing, or wind), charge slowly in a van and fast on a pad.
Before each trip the van checks the cost: fly / charge in the van for up to
`--max-charge-wait` / send the drone to the station and get a standby
(`delivery/station.py`). Wind is a base breeze plus random and dashboard
gusts (`delivery/wind.py`); drones counter up to `--drone-max-wind` after a
lag, so gusts push them, and nothing takes off above `--wind-hold`. Rain from `weather_cycle.py` (CARLA precipitation >= `--rain-ground`, 20) brings every flying drone down (a `rain` crash) and holds all take-offs until it stops (`rain_hold` / `rain_over` events). Failures
(order fails, drone crashes and is recovered after `--recover-s`):
collision (per-tick raycast), blown off course, battery died, motor failure,
parcel torn off, no safe drop, no drone. Flights are checked against real
geometry by raycast before take-off (`SkyCheck`). Live state for the
dashboard's **Delivery** tab goes to `data/delivery_live/` (`delivery/live.py`).

一台につき 1 機＋充電ステーションの待機機。飛行で電池を消費し、車内で低速、
パッドで急速充電。電池不足ならステーションへ飛び待機機と交換。風は基本風＋
突風（ダッシュボードからも）。衝突・ロスト・電池切れ・モーター故障・荷物落下
で配達失敗。ダッシュボードの「ドローン」タブで地図・電池・風を確認・操作。

## Pod EV battery and order rate / ポッドの EV 電池と注文頻度

Each pod has an EV battery (`--pod-drain-km` per km, `--pod-idle-drain` for
its systems, a little for charging its drone). It declines a batch it can't
finish and still reach a charger, goes to a road-side EV charger
(`--ev-chargers`, default one per two pods) below `--pod-low`, gives its
orders back below `--pod-critical`, and is towed if it runs flat. Orders
arrive `--min-gap..--max-gap` apart divided by the number of pods; an idle
pod with nothing queued gets a batch within seconds; at most
`--order-backlog` (default: one per pod) batches wait.

各ポッドは EV 電池を持ち、残量が少なければ路側の EV 充電器へ。注文頻度は
ポッド数に比例し、空いたポッドには数秒で次のバッチが届く。

## Code / コード

| File | Role / 役割 |
|---|---|
| `delivery_sim.py` | CARLA side: building discovery, van, mission loop, drawing, logging / CARLA 側 |
| `delivery/planning.py` | orders, drone tour, hub choice, stop order (no CARLA) / 計画（CARLA 不要） |
| `delivery/drone.py` | kinematic sortie state machine (no CARLA) / 出撃の状態機械 |
| `delivery/zones.py` | pod delivery zones, no-fly zones, detour paths (no CARLA) / 配送ゾーン・飛行禁止区域 |
| `delivery/backends.py` | `KinematicDrone`, `AirSimDrone` |
| `tests/test_delivery.py` | planner + sortie tests, no server / サーバー不要のテスト |
| `delivery/lmp_rig.py` | LastMile pod animation: the HTML's timelines ported (no CARLA) / アニメーション |
| `delivery/lmp_pod.py` | LastMile pod parts, shell modes, box station, customer / 部品と表示 |
| `delivery/lmp_van.py` | LastMile pod stop logic: drone and direct stops, deck, magazine / 停車地の作業 |
| `delivery/assets/build_lastmile.sh` | build + install the LastMile pod (Blender, UE4 editor, cook) / ビルド |
| `tests/test_lmp_rig.py`, `tests/test_lmp_van.py` | port vs the HTML; stop logic / 試験 |
| `delivery/drone_sensors.py` | sensor readings per drone for the Drones page (no CARLA) / センサー値 |
| `drone_window.py`, `drone_windows.py` | a drone's CARLA window (chase cam + onboard cams) and its process control / CARLA ビュー |
| `tests/test_drone_sensors.py`, `tests/test_drone_window.py` | sensors, damage, ray budget; CARLA view / 試験 |
| `delivery/lmh_rig.py`, `delivery/lmh_hub.py` | LastMile Hub + Mini Pods: timeline and state machine (no CARLA); the CARLA side / ハブとミニポッド |
| `delivery/minipod_sensors.py`, `tests/test_lmh.py`, `tests/lmh_live.py` | Mini Pod sensors; tests; live check / センサー・試験 |

## Drone sensors, damage and the CARLA view (Drones page) / センサー・損傷・CARLA ビュー

Every publish (2 Hz) `Fleet.sensor_readings` gives each drone a `sensors`
block in the live state: what its real sensors would report, computed from
what the sim knows (position, heading, flight phase, battery, wind and the
controller's wind estimate, parcel, pod). The suite fits the LMP drone (a
~3 kg quad-X with a winch, 0.97 m rotor tip to tip) and follows what
delivery drones carry (DJI FlyCart 30, Zipline P2, Matternet M2, Wing,
Amazon MK30; sources in `delivery/drone_sensors.py`):

| Sensor | Live values | From |
|---|---|---|
| GNSS / RTK (+ compass in the puck) | lat / lon (map geo reference), fix, satellites, HDOP, accuracy | position; float near obstacles / low, no fix in the pod |
| IMU, barometer | specific force (−T/m), rates, roll / pitch, vibration; pressure altitude | velocity and acceleration between publishes, thrust needed |
| Downward lidar, optical flow | height above ground (0.2–40 m), flow below 30 m | budgeted downward raycast |
| Obstacle (front / rear stereo + lidar) | distances ahead / behind / below | budgeted raycasts along the heading |
| Airspeed & wind estimate | airspeed, true wind, controller estimate, drift, limit | `delivery/wind.py` (`Drift.w_est`) |
| Smart battery, motors / ESCs | SoC, voltage, current, temperature, time left; RPM, throttle, current, temperature per motor | momentum-theory power for the thrust needed, battery model |
| Winch & load cell | state, tether length, speed, load | the drop hover's winch lowering |
| Datalink, Remote ID / ADS-B, parachute | RSSI / SNR / latency / loss to the pod, broadcast, armed / deployed | distance to the pod or station, flight state, crash height |

Each has a health flag (ok / warn / fail / off): a crash, a motor failure,
a flat battery, a torn-off parcel or wind beyond the drone show as faults.
The raycasts (down first, then ahead / behind, round-robin, at most 12 rays
every 0.2 s) run on their own thread with their own client: in synchronous
mode a ray can wait up to ~50 ms for the server's frame, and the sim loop
must never wait. The publish only reads their cache: 12 drones cost < 1.5 ms
(`sensor_ms` in the live state). The sim does not slow a parachute fall.

**Damage.** `drone_sensors.damage_model` turns a failure into broken /
damaged parts (ids of the page's 3D model), deterministically:

| Cause | Parts |
|---|---|
| collision (`collision:<label>`) | the corner it flew into (from its last airborne velocity in the drone frame; unknown: a fixed corner per drone and crash): rotor + arm broken, motor damaged, the neighbouring rotor and the shell damaged, FPV + front stereo (front blow), rear stereo (rear) or antennas (side) damaged |
| motor failure | that motor and its ESC broken, its rotor damaged |
| battery ran flat | battery broken |
| landing after a fall (crashed) | > 2 m: right-hand rotors (it lies on its right side) broken, > 5 m skids, > 8 m shell + GNSS; a deployed parachute makes them only damaged |
| parcel torn off | hook broken, winch damaged |

It runs only while a drone is down or lost its parcel (~4 µs), clears when
the drone is repaired, and appears as `sensors.damage` (`parts`, `cause`,
`impact`, `fall_h`, `detail`, `broken`). The page draws it on the 3D model.

**CARLA view.** "Open CARLA view" posts `/drones/window` and the dashboard
starts `drone_window.py --drone N` detached (`drone_windows.py`, one window
per drone, at most 4): a chase camera rigidly on the drone body (behind and
above, so the drone itself is in the picture; level and free when it lies
crashed on its side), an orbit view, and its four onboard cameras (FPV
gimbal 640×360 90°, delivery camera straight down 480×360 100°, front and
rear stereo left eyes 480×270 110°) in a strip at 10 fps, with a HUD. While
docked the camera rides on the pod car; a respawned body is re-attached.
Every sensor has `role_name` `hil_dronewin_<id>`; SIGTERM / Esc / closing
destroys them, a new window sweeps up leftovers of its drone. The browser
gets no video.

毎回の公開（2 Hz）で各機に `sensors`（実機相当のセンサー値と健全性）を付け、
ドローンページで表示する。レイは専用スレッド（別クライアント）で実行し、公開は
キャッシュを読むだけ（12 機で 1.5 ms 未満）。墜落・故障では `damage_model` が
壊れた部品（衝突方向・落下高さ・原因から決定的に）を `sensors.damage` に出し、
ページの 3D モデルで破損表示する。「CARLA ビューを開く」で `drone_window.py`
（機体を追うチェイスカメラと機載カメラ）が別ウィンドウで開き、閉じると
センサーをすべて破棄する。

## Delivery zones and no-fly zones / 配送ゾーンと飛行禁止区域

Drawn on the dashboard's City Map (Delivery page, edit mode), saved per map in
`data/delivery_zones.json` (CARLA world metres, `GET/POST /delivery/zones`);
the running sim picks a change up within a second (`zones` event).

- **Pod zone** (one rectangle per pod, the pod's own colour): that pod only
  takes orders whose drop point is inside it. Pods without a zone take the
  orders outside every pod zone. New batches are made inside the zone of a
  pod that is free and has nothing queued first. When every pod has a zone,
  queued orders outside all of them are cancelled (`orders_cancelled`,
  stat `cancelled_outside_zones`); overlapping zones: the smaller one wins.
- **No-fly zone** (red, any number): nothing flies over it. Drone orders
  inside one are delivered face to face (`nofly_f2f`), drone stops and
  meeting points are never inside one, and every drone leg (sortie, pod
  mission, transfer to the station or a van) turns at the zone's corners
  with a 6 m margin (`zones.Airspace`, shortest path over the corners);
  flights in the air re-plan when a zone is drawn. Cruise height and battery
  estimates use the detour.
- **Meeting points** (per pod, up to 24, clicked on the map and snapped to
  the nearest kerb point a pod may stop at, `stops` in the map export): a
  pod that has any launches its drones and takes them back only there. Its
  drone stop is the best of its points for each batch; its drones meet it at
  its next stop only if that stop is within 15 m of one of its points, else
  back at the drone stop. Points inside a no-fly zone are ignored. Pods
  without points choose their stops themselves.
- **Distribution points** (per LastMile hub, `dist`, up to 24, snapped like
  meeting points): the hub parks only at the best of them for each batch and
  sends its Mini Pods from there. No-fly zones do not apply to the pods.
- Everything above is saved per map, like the key bindings: switching maps
  loads that map's zones and points.
- Tests: `tests/test_zones.py`.

ダッシュボードの地図でポッドごとの配送ゾーン（各ポッドの色）と飛行禁止区域（赤）、
合流地点（道路上の停車可能点に吸着）を設定し、地図ごとに保存する。ゾーンを持つポッドはその中の注文だけ、ゾーン無しのポッドは残りを担当。飛行禁止
区域の中のドローン注文は対面配達になり、飛行経路は区域の角を回って迂回する。

## LastMile Hub with Mini Pods / LastMile ハブとミニポッド

A second kind of delivery vehicle: the **LastMile Hub LMH-3** (the "Mini pods"
tab of the user's `LastMile_Pod_3D.html`, model in
`~/Downloads/LastMileHub_FBX_Maya_Unreal/`). It carries three **Mini Delivery
Pods** (0.70 x 0.54 x 0.67 m, six wheels, skid steer) and delivers with them
instead of drones. Launch page: **Mini-pod hubs** (`--minipod-hubs N` in
`run_pipeline.py`, `--lmh N` in `delivery_sim.py`). Hub ids follow the drone
pods' (pods 1-3 + hubs 4-5), Mini Pod ids are 101, 102, ... (3 per hub).
2 つ目の配送車：ミニ配送ポッド 3 台を積んだ LastMile ハブ。

**Who it serves / 担当.** By default a hub without a pod zone of its own takes
the orders **inside the no-fly zones** (pods drive; nothing flies), and only
those while any no-fly zone is drawn; with none drawn it takes any order like
an unzoned pod. Draw a pod zone for the hub on the City Map and it uses that
zone instead (your choice wins). Mixed batches are split: the no-fly orders go
to a hub, the rest stays with the drone pods (`nofly_minipod` event). Without a
hub that covers them, no-fly orders are delivered face to face by a drone pod
as before (`nofly_f2f`). The live state shows each hub's `area`
(`nofly` / `zone` / `any`).
既定ではゾーンを持たないハブが飛行禁止区域内の注文を担当。ゾーンを描けばそちらが優先。

**At a batch / バッチごと.**
1. The hub plans **distribution points**: your `dist` points for it (City Map,
   saved in `data/delivery_zones.json` next to `meet`, up to 24 per hub, each
   snapped to the nearest kerb point) -- the best one for the batch -- or, without
   any, the kerb point nearest the orders' centroid. Orders farther than half a
   pod trip (`--minipod-range`, 1500 m out and back) get another point. It may be
   inside a no-fly zone: the hub is a road vehicle.
2. It drives there like any van (BasicAgent, the Japan rules, the off-road
   guard, reverse-out), capped at `--lmh-speed` 25 km/h (it runs on the Prius
   drivetrain), and parks at the kerb: keep left, kerb and bay doors on the left.
3. The HTML sequence (`delivery/lmh_rig.py`, from the site's `mini.js`): scan
   (storage loop past the scan tunnel, 6 s), divert (gates 3, 2, 1), lift
   (carriages down 0.72 m), load (shutter up, rollers push the parcel 0.64 m into
   the pod), release (plugs out, pins out, ramp doors fold down). The pods roll
   down the ramps at 27.4 / 27.7 / 30.0 s, raise their flags once clear, and the
   doors close behind them.
4. Each pod drives the **kerb side** of the road: the lane-graph route (A* over
   waypoints, both directions, kerb-side lanes only outside junctions, so it
   crosses only at junctions) offset to `--minipod-inset` 0.4 m inside the
   kerb-side lane edge (the 路側帯 strip), at 1.6 m/s (Japan's limit for
   遠隔操作型小型車 is 6 km/h), 0.8 m/s near walkers. Heights come from the lane
   and a rate-limited ray thread (`MiniPodProbe`).
5. **No pod enters a building.** Two kinds of handover:
   - **front drop**: the pod stops at the drop point in front of the place with
     its rear to it, rolls up the rear shutter, the floor belt pushes the parcel
     out onto the ground (`deliver.ground_drop`, 0.85 m behind it), shutter down;
     the parcel lies there for 10 s;
   - **delivery box** (the order goes to a box station, `delivery/boxes.py`):
     every box site is the web viewer's Apartment B **parcel locker**
     (`static.prop.lmhlocker` + two `lmhlockerflap`, from
     `LastMileHub_Neighbourhood.glb`) once the LastMile Hub props are
     installed; otherwise the old LMP station. It has two robot ports: the
     model's own low one at Mini Pod bay height (0.29-0.65 m) and an upper one
     added straight above at the LMP-4 side-port tray height (1.0-1.37 m over
     the pavement), so **both pod types deliver into the same locker**: the
     LMP-4 parks in the kerb lane with its side port under the upper port and
     slides its arm in (the upper flap follows its station-flap timing). The
     locker stands 0.8 m behind the kerb and a Mini Pod docks on the pavement. As in mini.js m_aptB: the pod turns 180°, reverses
     0.39 m up to the port, the port flap swings inward (77°, 0.5 s) as the
     rear shutter rolls up, the floor belt pushes the parcel 0.77 m into the
     locker (1.4 s), shutter and flap close. A text over the locker shows its
     screen (ROBOT PORT POD DOCKING / RECEIVING / PARCEL IN - CODE SENT). The
     customer later collects from the locker.
6. The pods come back to the ramp foot, turn to face the kerb, wait for the door,
   reverse 1.69 m up into the bay at 0.6 m/s; pins in, plug in, door up. With
   **meeting points** (`meet`) the hub drives to the best one once all pods are
   out and its doors are shut; the pods go there and wait until it parks.
   Without, the hub waits at the distribution point. More than 3 orders: the
   next pods load as soon as one is back.

**Pods / ポッド.** Battery (`--minipod-kwh` 1 kWh, `--minipod-drain-km` 5 %/km,
0.8 % per handover, 15 % reserve), charged in the bay from the hub's battery. A
trip that needs more than the pod has waits for charging; an order out of range
gets another stop (`pod_range`). Obstacles: walkers and cars on its path within
2.5 m stop it; blocked for 6 s it backs up 1 m and re-plans, after 3 tries it
skips 4 m ahead (`pod_stuck`). Rare random faults (`--minipod-fail-rate` per
hour: motor, lidar, collision; off with `--no-random-failures`): the pod stops
(`crashed`, damaged parts in the state), another pod drives to it, takes the
parcel (`pod_rescue`) and delivers it; with none free within
`--minipod-fail-timeout` the order fails. The crashed pod is back in its bay
after `--minipod-recover-s`.

**Events / イベント:** `hub_dist`, `hub_load`, `hub_meet`, `pod_out`, `pod_door`,
`pod_handover`, `pod_back`, `pod_docked`, `pod_fault`, `pod_rescue`,
`pod_recovered`, `pod_stuck`, `pod_range`, `nofly_minipod`.

**Live state / 公開状態** (`/delivery/state`): `vans[]` entries have `kind`
(`lmp` / `lmh`); hubs also `pods_docked`, `dist`, `meet`, `phase`, `area`.
Top-level `minipods[]`: `id, hub, bay, status` (docked, loading, rolling_out,
to_door, handover, returning, docking, waiting, crashed, stuck), `x, y, z, yaw,
speed, battery, order, dest, kind` (front / box), `lid, shutter, flag, actor,
trips, km, fail, path` (next <= 30 points), `sensors` (per key: label, value,
unit, health, group, part -- lidar, 6 cameras, ultrasonic ring, bumper,
GNSS/RTK, IMU, 6 wheel encoders and motor currents, battery V/I/T/SoC/range,
lid, shutter, flag, load cell, LTE and latency, e-stop;
`delivery/minipod_sensors.py`, sources there) and `damage`. 18 pods cost
~1.3 ms per publish (`minipod_ms`).

**Models / モデル.** The parts table is `delivery/assets/lmh_parts.json` (hub
parts with their motions, bays, pod parts, parcels). Until the LMH props are
imported into CARLA (needs CARLA stopped), the hub is the LMP-4 vehicle with
its shell and every Mini Pod a `static.prop.box01` box (0.66 m); the motions
run all the same and show as soon as the props exist. The box is solid, so it
is hidden while it is inside the hub's footprint (in the bay, on the ramp) and
the pod paths keep clear of the real vehicle's size (the LMP-4 is 4.3 x 2.0 m):
before that fix the boxes shoved the parked stand-in up to 3 m.
代用中のハブは LMP-4、ミニポッドは段ボール箱（車体内では隠す）。

**Tests / 試験:** `tests/test_lmh.py` (no CARLA); `tests/lmh_live.py` runs one
hub + pods for one cycle in a running world without ticking it and cleans up.

## Mini Pods page, CARLA view and the map tools / ミニポッドページ・CARLA ビュー・地図の道具

**Mini Pods page** (dashboard `#minipods`, nav "Mini Pods" / ミニポッド;
`web/src/views/minipods/`, its own lazy chunk, three.js shared with the Drones
page). Same layout family as the Drones page: one tab per pod (status dot,
`H<hub>·B<bay>`, battery, ✕ when a part is broken); left the Pod overview
(trips, km, battery, hub / bay, order, **Open CARLA view**, **Diagnostics**),
the Component view (close-up renders: lidar, front camera, wheels & motors,
lid & payload bay) and the **Hub card** (state, phase, battery, pods docked,
the effective **delivery area** -- no-fly zones by default, its own zone once
drawn -- distribution / meeting point, the three bays and which pod sits in
each); centre the real pod in 3D (`web/public/models/lmh_minipod.glb`, served
by `dashboard.py` at `/models/`; physical materials, six wheels rolling at the
live speed, lid / shutter / flag from `lid` / `shutter` / `flag`, headlights on
the street, a spinning lidar beam, X-ray with the battery, compute unit and
drive motors inside, every part clickable), the status tiles (Lidar, Cameras,
GNSS, Obstacle stop, Lid lock, Link) and the Damage report (broken red /
damaged amber on the model: a broken wheel is gone with shards on the ground,
a torn lid hangs askew, a cracked lidar shows cracks); right the battery gauge,
speed and distance-to-destination sparklines, ETA, and a mini map (track,
planned `path`, destination, hub, the hub's distribution and meeting points).
Below, the sensor cards, rendered from the self-describing `sensors` dict and
grouped by `group` -- the page hard-codes no sensor key; `part` (a GLB node
name like `MiniPod_P1_Wheel3`, or a mount key like `lidar`, `cam_front`) picks
the 3D part a card points at. Until the delivery sim is restarted it publishes
no `minipods`: the page shows an empty state and a "restart the delivery sim"
banner.

**CARLA view** (`minipod_window.py`, started detached by `dashboard.py`
`POST /minipods/window {id, action}` through `minipod_windows.py`; pid files in
`data/minipod_windows/`, at most 3 at once). A chase camera behind and above
the pod body (you see the robot itself), an orbit view, and a strip of six
onboard cameras: front, rear, left, right, front-down kerb camera, payload
camera under the lid, mounted from `lmh_parts.json` `minipod.sensors_mounts`
(plausible offsets on the 0.70 x 0.54 x 0.67 m body when missing). Keys 1
chase, 2 orbit, 3-8 onboard, Tab, H HUD, F11, Esc. HUD: id, status, hub / bay,
battery, speed, order, lid, damage and sensor faults. Docked pods are followed
on the hub vehicle; a respawned body is re-attached; every sensor is
`role_name` `hil_podwin_<id>` (leftovers swept at start, all destroyed on
close / SIGTERM). Test without pods: `minipod_window.py --id 99 --actor <any
actor id> --snapshot /tmp/p.png --duration 10`.

**Map tools** (Delivery page City Map, **Edit zones**): a third mode
**Distribution points** next to Zones and Meeting points, listing only LMH hubs
(chips in the hub's colour). Same gestures as meeting points: the "D+" preview
snaps to the nearest kerb stop, click adds (max 24 per hub), drag moves
(re-snapped), Delete / Backspace removes, Esc drops the tool. Drawn as a square
"D<n>" badge on a short post (vs the round meeting-point pin); tooltip "Hub 4
distribution point 2"; own Layers entry ("Distribution points (LMH)") and
legend line. For an LMH hub the meeting points are where its pods return to
the hub (and are never flagged for no-fly zones). In Zones mode an LMH hub chip
shows **"default: no-fly zones"** until a zone is drawn for it, and every
no-fly zone gets a thin inner outline in the colour of each hub that uses the
default; drawing a zone switches the hub to it, deleting the zone returns it to
the default. Every save posts `{map, pods, nofly, meet, dist}`; a reply
without `dist` reads as none. The map also draws the mini pods (dot in the hub
colour, ring = status, heading tick, ✕ when crashed / stuck) and their planned
paths (dashed), each with its own Layers entry.

ミニポッドページ（#minipods）：ドローンページと同じ構成で、ポッドごとのタブ・概要・
部品ビュー・ハブカード（配達範囲・ベイ）・実モデルの 3D（車輪・蓋・シャッター・旗が
状態どおりに動く、X 線、損傷表示）・走行データ・小地図・センサーカード（自己記述的な
辞書から描画）。「CARLA ビューを開く」で minipod_window.py（チェイス・周回・機載 6
台）。地図の「ゾーン編集」に配布地点（LMH ハブのみ、四角い「D」）を追加。LMH ハブは
ゾーンを描くまで飛行禁止区域が配達範囲（ハブの色の内枠で表示）。

## Box stations in the world / 世界に置かれた箱型ステーション

On keep-left maps (`"keep_left": true` in `maps/maps.json`, e.g. Oita Station)
the LastMile pods no longer set their own box down beside them. Instead
`delivery/boxes.py` places box stations around the city at startup, like post
boxes:

- on the raised pavement just behind the kerb of a kerb-side lane, away from
  junctions, at least `--box-spacing` (110 m) apart; the kerb is found on the
  3-D model (where the ground steps up), since the road logic's lane edge
  need not match the modelled road;
- each site is checked: every point of the box's footprint and the
  customer's spot on the same pavement level, nothing in the way up to 2 m,
  and an arm reach of at most 3 m;
- intake towards the road (the pod's side port is on its left), compartment
  door towards the pavement.

A face-to-face order goes to a free box. The pod drives there, docks (creeps
along the kerb lane, holding the lane centre, until its port is level with
the box), extends the side port as far as that box needs, puts the parcel in
and drives on -- the rest of the flow (drones meeting at the stop) is
unchanged. The parcel waits in the box; 20-60 s later a pedestrian walks up
along the pavement, opens the compartment, takes it and leaves. The
dashboard's Delivery map shows the boxes. `--no-box-stations` goes back to
the pod's own box; keep-right maps always use it (the port faces the road
centre there).

左側通行のマップでは、箱型ステーションを歩道に点在させ、対面注文はその箱へ
配達する。ポッドは横付けしてサイドポートを必要な長さまで伸ばし、荷物を入れて
出発する。その後、歩行者が受け取りに来る。

## Charging stations / 充電ステーション

The web viewer's charging station (charge dock, `LastMileHub_ChargeDock.glb`
-> `static.prop.lmhchargestation`, 14 m x 9.8 m) stands beside the roads:
**bay 03** for an LMP-4 pod and **bay 04** for a LastMile Hub, each with an
automatic plug unit (`lmhplugz/y/boom/head`), the Mini Pod pads and the drone
deck behind them. With stations in the sim their bays replace the road-side
EV chargers (`delivery/charge_sites.py`).

- **Launch page, Charging stations card**: *Random* + a count (0 = off, the
  old road-side chargers) or *Manual*: the spots placed on the Delivery map
  (editor mode "Charging stations": click, drag, R to turn, Delete; up to 12
  per map, saved in `data/delivery_zones.json` `charge`). Flags
  `--charge-stations N --charge-mode random|manual`.
- **Free ground only**: plain lawn, never over a building, road, pavement or
  footpath, markings, a court, a car park or a house lot. Our generated maps
  carry a surface grid (`maps/build/<Map>/surface_grid.npz`, made by
  `osm_city/surface_grid.py`, part of `make_osm_city.sh`), because road and
  pavement raycast alike there; other maps use the raycast labels. Also
  checked: nothing standing over the footprint (raycast), lamp/signal poles
  (environment objects), the ground flat and level with the road, the road
  straight there, the drive-in strips clear (no pole, guardrail, bus stop or
  crossing in the way), the bays reachable by every LastMile vehicle in the
  sim (road graph), each vehicle fits (drive-in width, roof clearance: LMP-4
  3.96 m under bay 03's 4.12 m, hub 2.09 m under bay 04's 3.04 m) and has
  room to turn on the spot in the lane. Close to the road only: the open
  edge stands where the lawn starts behind the kerb and pavement, at most
  4 m beyond the lane edge; an access road of paving slabs
  (`static.prop.lmhapron`, the station floor's material) runs from the kerb
  to the open edge over both bays' drive-in strips. A manual spot is moved onto the nearest kerb lane on its side;
  spots that fail are shown on the map with the reason (`station_errors`).
- **Waiting bay and footpath**: each station also has a waiting bay (6 m x
  2.8 m, paved, beside it, upstream first) and a paved footpath round its
  sides and back, both on free lawn (a station without room for them is not
  placed). With every bay of its kind taken, a vehicle drives to the nearest
  free waiting bay, slides sideways onto it (off the lane), waits, and takes
  the first bay of its kind that frees up at any station. Walkers that would
  walk into a station or its waiting bay are sent round it on the footpath
  (`charge_sites.WalkerGuard`, twice a second).
- **Charge time**: from the charger's power and the pack: `--station-kw` 50
  (DC bays), `--roadside-kw` 22 (road-side and yard chargers), `--lmp-kwh`
  (0 = `--pod-kwh` 10), `--hub-kwh` 10; full power to 80 %, then tapering
  to a quarter at 100 % (75 -> 98 % at a station: ~4.6 min). Energy flows only
  once the plug unit is in. `--pod-charge-rate` > 0 brings back a fixed
  rate. Mini Pods charge on the station's pads (M1-M3) at `--pad-kw` 3 while
  their hub charges; drones charge on the stations' decks (3 pads each,
  `station.DeckStation`, the old sidewalk pads only as overflow).
- **Traffic**: before turning in, reversing out or sliding into / out of a
  waiting bay the vehicle waits for a gap: nothing within its turning circle
  + 1.5 m, nobody on its path to the bay, no vehicle closing in along the
  lane within 30 m (40 s at most, then it goes).
- **Charging**: the vehicle drives to its bay's entry in the kerb lane, turns
  on the spot, drives in nose first; the plug unit's boom pushes the head
  0.375 m into the front inlet (last 50 mm slowly), charges, pulls back; the
  vehicle reverses out and turns back onto the lane. One vehicle moves at a
  station at a time. State: `stations[]` (corners, bays with van and phase
  free / in / plugging / charging / unplugging / out).

## Wind / 風

A gust stronger than a drone (11 m/s) pushes it off its path; it flies back
once the gust eases and only crashes if the push takes it into something
(the collision check runs on the drifted position). In a test a 16 m/s gust
pushed a drone 17.7 m off course and it carried on. `--lost-drift M` brings
back the old hard limit (lost when pushed further than M metres).

突風で流されても、何かに衝突しない限りドローンは経路に戻る。

## Oita Station ring road / 外周道路

The 3D model has a ring road (外周道路) round its outside that is not in
OpenStreetMap. `maps/add_ring_road.py` (run by `maps/prepare_oita.py` before
netconvert) cuts the OSM streets where they meet it and adds it as a
two-lane, 7.5 m road with a T-junction at each street, the way the model's
own build does. Autopilot cars and pods drive on it; nothing runs past it.

- The traffic manager's cached map (`.../OitaStation/TM/OitaStation.bin`)
  was made from the original road logic and crashed the traffic manager with
  the new one. It is moved aside (`.bin.before_ring`); the traffic manager
  builds its map itself in under a second.
- The level's baked spawn points come from the original road logic (43
  outside the ring, most facing backwards). `lane_spawns.py` keeps the ones
  on a lane, snapped to its centre and heading; traffic, the hero car and
  the delivery pods use it.
- Pods only stop where they can drive both in and out again (one-way
  streets can lead nowhere), re-route when a route ends short of a box
  station, and drive round the block to a box they have just passed.

外周道路を道路情報に追加し、その外側の道路を削除した。
