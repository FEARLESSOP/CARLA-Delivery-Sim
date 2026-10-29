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
with fewer than 20 usable buildings get roadside addresses added.
Town01〜05・10HD で確認済み。使える建物が 20 未満のマップでは道路沿いの
地点を追加します。

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
lag, so gusts push them, and nothing takes off above `--wind-hold`. Failures
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
| `delivery/backends.py` | `KinematicDrone`, `AirSimDrone` |
| `tests/test_delivery.py` | planner + sortie tests, no server / サーバー不要のテスト |
| `delivery/lmp_rig.py` | LastMile pod animation: the HTML's timelines ported (no CARLA) / アニメーション |
| `delivery/lmp_pod.py` | LastMile pod parts, shell modes, box station, customer / 部品と表示 |
| `delivery/lmp_van.py` | LastMile pod stop logic: drone and direct stops, deck, magazine / 停車地の作業 |
| `delivery/assets/build_lastmile.sh` | build + install the LastMile pod (Blender, UE4 editor, cook) / ビルド |
| `tests/test_lmp_rig.py`, `tests/test_lmp_van.py` | port vs the HTML; stop logic / 試験 |
