# CARLA HIL Driving Simulator / CARLA HILドライビングシミュレータ/ ver.1

Human-in-the-loop driving study on CARLA 0.9.15, driven with a Logitech G923
steering wheel. One command starts the simulator, traffic, a live telemetry
dashboard, and the driver's view.

CARLA 0.9.15上のヒューマン・イン・ザ・ループ（HIL）運転実験環境。
Logitech G923ステアリングホイールで運転します。コマンド1つでシミュレータ・
交通・テレメトリダッシュボード・ドライバー視点をまとめて起動できます。

---

## File structure / ファイル構成

**Looking for a file? See [FILES.md](FILES.md)**: every important file by topic
(running, traffic and signals, delivery, maps, live checks, cars, tests).
/ 重要なファイルの一覧は FILES.md（分野別）。

```
hil_sim/
├── run_pipeline.py                      # start everything / 全プロセス一括起動
├── manual_control_steeringwheel G923.py # driving window (G923) / 運転ウィンドウ（G923）
├── dashboard.py                         # web dashboard, port 5000 / Webダッシュボード（ポート5000）
├── sensor_viewer.py                     # multi-camera viewer / マルチカメラビューア
├── drone_viewer.py                      # CCTV wall of drone cameras (--drone-cams) / ドローンカメラ一覧
├── drone_window.py                      # one drone's CARLA view (dashboard Drones tab) / 1 機の CARLA ビュー
├── drone_windows.py                     # starts / closes those windows for the dashboard / その起動・終了
├── minipod_window.py                    # one mini pod's CARLA view (dashboard Mini Pods tab) / 1 台の CARLA ビュー
├── minipod_windows.py                   # starts / closes those windows for the dashboard / その起動・終了
├── generate_traffic.py                  # NPC traffic / NPC交通生成
├── traffic_profile.py                   # Japan driving rules for NPCs / 日本の交通ルール（NPC）
├── traffic_profiles.json                # which maps get which profile / プロファイルと適用マップ
├── hokousha.py                          # Japanese pedestrian signals + walkers / 歩行者用信号
├── signal_timing.py                     # real police signal timing, auto on OitaKosen / 実際の信号時間
├── traction_fx.py                       # skid marks when the tyres slide / スキッドマーク
├── road_overlay.py                      # road logic overlay, O key / 道路ロジック表示（O キー）
├── bus_schedule.py                      # timetabled buses (OitaStation, GTFS-JP) / 時刻表どおりの路線バス
├── parked_cars.py                       # parked cars on a map's spots / 駐車車両
├── own_cars.py                          # cars.json ids, npc_only filter / 自作車両 ID・NPC 専用の除外
├── cars/                                # your own cars: build + per-car driver view / 自作車両
├── wheel_config.ini                     # wheel axes & steering feel / ホイール軸・ステア感度
├── pedal_tuning.json                    # pedal sensitivity (live) / ペダル感度（走行中変更可）
├── keybinds.py                          # key + wheel-button bindings / キー・ホイールボタン割り当て
├── map_switch.py                        # F5 map cycle + switch request / F5 のマップ巡回と切替依頼
├── windows_control.py                   # open/close viewer windows from the dashboard / ビューアの開閉
├── keybinds.json                        # bindings in use (dashboard) / 現在の割り当て（ダッシュボードで編集）
├── g923_buttons.json                    # G923 button names / G923 のボタン名
├── agent_drive.py                       # autonomous ego client (RL/autopilot) / 自動運転egoクライアント
├── delivery_sim.py                      # car + drone delivery on Town03 (docs/DELIVERY.md) / 車+ドローン配送
├── delivery/                            # delivery planner + drone backends / 配送計画・ドローン
├── rl/                                  # RL layer (docs/RL_DESIGN.md) / RL層
│   ├── rules.py                         # shared alert + reward rules / 警告・報酬の共有ルール
│   ├── carla_env.py                     # Gym environment / Gym環境
│   ├── extract_scenarios.py             # mine human-error clips / 人間エラー抽出
│   ├── train_sac.py                     # SAC training / SAC学習
│   └── smoke_test.py                    # env sanity check / 環境の動作確認
├── README.md
├── FILES.md                             # where is what / ファイルの場所
├── CLAUDE.md, AGENTS.md                 # instructions for Claude Code / Google Antigravity / AI 向け説明
└── data/                                # simulation database / 計測データ保存先
    ├── scenarios/                       # mined clips + driver profiles / 抽出クリップ・プロファイル
    └── 2026-07-07/                      # one folder per day / 日付ごとのフォルダ
        └── 17-17-53/                    # one folder per session (start time) / セッション（開始時刻）ごと
            ├── telemetry.csv            # full time series, ~10 Hz / 全時系列データ（約10Hz）
            └── summary.json             # session totals / セッション集計
```

> This folder must stay directly under `PythonAPI/` — the scripts resolve
> the CARLA egg via `../carla/dist` and read `wheel_config.ini` from their
> own directory.
>
> このフォルダは必ず `PythonAPI/` 直下に置いてください。各スクリプトは
> `../carla/dist` からCARLA eggを解決し、`wheel_config.ini` を自身の
> ディレクトリから読み込みます。

---

## New PC / 新しい PC

One script sets up everything (CARLA, the rig, venv, Node, content from the old
PC, skills): see [NEW_PC_SETUP.md](../../NEW_PC_SETUP.md).
新しい PC は `setup_new_pc.sh` 一つで構築できます。

## How to run / 実行方法

**The launcher (no arguments):**

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
python3 run_pipeline.py
```

Only the dashboard starts and opens http://localhost:5000 on its Simulation
page. Pick the map, cars, pedestrians, cyclists, delivery pods, your car,
force feedback, display and the rest, then **Launch** (`launcher.py`; the
options are kept in `launch_options.json` for next time). While it runs the
same page becomes **live controls**:

- map: switched like F5 (CARLA keeps running);
- cars / pedestrians / cyclists: the traffic restarts with the new counts;
- delivery pods: the delivery sim restarts with the new number (0 = off);
- your car: the driving window restarts with it;
- force feedback, skid marks, day/night + weather, sensor viewer, drone wall: on / off at once;
- display and other restart-only options: **Restart with these settings**.

Stop returns to the Launch page; closing the driving window does too; Ctrl+C in
the terminal stops everything. The pipeline's output is on the page
(`data/pipeline.log`). Every option has a `live` / `map switch` / `restart`
tag; the list is `launch_options.py`.

ランチャー: 引数なしで起動するとダッシュボードだけが立ち上がり、そこでマップ・
交通量・配送車・自車などを選んで起動します。実行中は同じページでマップ切替・
交通量・配送車・自車・各機能の入切を変更でき、再起動が必要な設定はボタン一つで
再起動します。

**With flags (as before):**

```bash
python3 run_pipeline.py --map Town02 --vehicles 60 --walkers 15
```

The dashboard's live controls work here too; a restart from the dashboard
needs the launcher.

The pipeline boots CarlaUE4, loads the map, spawns traffic, starts the
dashboard (http://localhost:5000) and the sensor viewer, then opens the
driving window. Quit with ESC in the driving window or Ctrl+C in the
terminal — everything (including the simulator) shuts down together.
If a CARLA server is already running on port 2000, it is reused instead of
starting a second one.

CarlaUE4起動 → マップ読込 → 交通生成 → ダッシュボード
（http://localhost:5000）→ センサービューア → 運転ウィンドウの順に
立ち上がります。終了は運転ウィンドウでESC、または端末でCtrl+C。
シミュレータを含む全プロセスが一括終了します。ポート2000で既にCARLAが
動いている場合は、新たに起動せずそれを再利用します。

Useful options / 主なオプション:

| Option | Meaning / 意味 |
|---|---|
| `--map Town02` | map to load / 読み込むマップ |
| `--vehicles N --walkers N` | traffic amount / 交通量（車・歩行者） |
| `--cyclists N` | bicycles with riders, on top of `--vehicles` (default 3) / 自転車 |
| `--wheelchair-share F` / `--runners F` | share of walkers in a wheelchair (0.15) / jogging (0.1) / 車椅子・ジョギングの割合 |
| `--vehicle-filter vehicle.tesla.model3` | hero vehicle model / 自車の車種 |
| `--start-vehicle vehicle.mazda.rx7` | first hero car; Backspace still cycles through all / 最初の車種（Backspace で切替可） |
| `--no-sim` | reuse an already-running CARLA / 起動済みCARLAを再利用 |
| `--no-traffic` / `--no-dashboard` / `--no-sensor-viewer` | skip a component (the sensor viewer can still be opened later from the dashboard) / 各コンポーネントを起動しない（センサービューアは後からダッシュボードで開ける） |
| `--no-traction-fx` | no skid marks / スキッドマークなし |
| `--no-road-overlay` | do not start the road logic overlay helper (O key) / 道路ロジック表示を起動しない |
| `--res 1080p` (or `--control-res`) | driving window resolution: `720p`, `1080p` (default), `1440p`, `4k` or `WxH` / 運転ウィンドウの解像度 |
| `--fov 100` | driver-view field of view / ドライバー視点の視野角 |
| `--no-log` | disable CSV logging / CSV記録を無効化 |
| `--keyboard` | drive with W/A/S/D instead of the wheel / ホイールの代わりにW/A/S/Dで運転 |
| `--delivery N` | also run N car + drone delivery vans; each order goes to exactly one van (docs/DELIVERY.md) / 車+ドローンの配送車を N 台追加（各注文は 1 台のみが担当） |
| `--minipod-hubs N` | also run N LastMile hubs that deliver with 3 Mini Pods each (`delivery_sim.py --lmh N`); by default they take the orders inside no-fly zones (docs/DELIVERY.md) / ミニポッドで配達する LastMile ハブを N 台追加（既定で飛行禁止区域の注文を担当） |
| `--drone-cams` / `--drone-cams-res 1280x720` | open the drone camera wall at start (needs `--delivery`; without the flag it can be opened later from the dashboard) / ドローンカメラ一覧を起動時に開く（後からダッシュボードでも開ける） |
| `--delivery-args "..."` | extra options for `delivery_sim.py`, e.g. `"--first-now"` / `delivery_sim.py` への追加オプション |
| `--ffb` / `--ffb-strength X` | also run `carla_ffb.py` (G923 force feedback); skipped if one is already running / `carla_ffb.py`（G923 のフォースフィードバック）も起動（既に起動中なら起動しない） |

```bash
python3 run_pipeline.py --map Town05 --vehicles 0 --walkers 0 --delivery 3 --ffb
```

---

## Dashboard / ダッシュボード

Open **http://localhost:5000** in a browser.

Displays and controls are kept apart: four tabs show what is happening, and
every control is on the **Settings** tab.

- **Drive tab**: live speed, pedals, safety metrics (TTC/THW), driver
  performance (SDLP, steering reversals, jerk RMS, speed compliance),
  charts with hover tooltips, mini-map, events log.
- **Vehicle tab** (`#veh`): vehicle status, sensors, impacts.
- **Delivery tab** (`#drone`): city map, deliveries, drones, pods, events,
  and the LastMile pod: its animation player ("Take over") and deck.
- **Settings tab** (`#settings-driving`, `-ffb`, `-delivery`): Driving
  (pedal tuning, vehicle performance), Force feedback, Delivery sim (wind,
  pod shell view). Everything applies live and is saved.
- **Drones tab** (`#drones/<id>`): a fleet console for the delivery
  drones. Tabs for every drone (status, battery); the drone itself in
  real-time 3D in the centre (the real LMP model with physical materials,
  rotors at the live motor RPM, an **X-ray** cutaway that shows the battery
  cells, flight controller, winch drum and wiring; drag to turn, click a
  part for its health and reading); parts the sim reports broken are shown
  broken (a broken arm droops, a rotor loses a blade, a cracked shell, red
  labels) with a **Damage report**; Drone overview, Component view
  close-ups, flight data (battery gauge, altitude / speed, time left,
  distance home, map with its track, pod and drop point), system tiles and
  every sensor's card. **Open CARLA view** opens a CARLA window for that
  drone (see Drone CARLA view below).
- **Statistics tab** (`#stats`): current-session tiles (crashes,
  out-of-lane, red lights, hard brakes, distance, duration), a per-session
  bar chart, and a table of all saved sessions grouped by day.
- **EN / 日本語** buttons in the header switch the language (also
  `?lang=ja` in the URL). The choice is remembered.
- **⬇ CSV log** downloads the current session's telemetry.

ブラウザで **http://localhost:5000** を開きます。

表示と操作を分け、操作はすべて**設定**タブにまとめています。

- **運転**タブ：速度・ペダル・安全指標（TTC/THW）・運転
  パフォーマンス（SDLP、ステア反転、ジャークRMS、速度遵守率）・
  ホバー表示付きグラフ・ミニマップ・イベントログ。
- **車両状態**タブ：車両の状態・センサー・衝突。
- **配送**タブ：地図・配達・ドローン・ポッド・イベント・LastMile ポッドの
  アニメーション再生と甲板。
- **設定**タブ：運転（ペダル調整・車両性能）・フォースフィードバック・
  配送シミュ（風・ポッドの外殻表示）。即時反映・保存。
- **ドローン**タブ（`#drones/<id>`）：配送ドローンのフリート画面。機体タブ、
  中央にリアルタイム 3D の機体（実モデル・物理マテリアル・実回転数で回る
  ローター・内部が見える **X 線**表示・部品クリックで健全性と値）、壊れた部品は
  壊れた姿で表示（アームが垂れる・翼の欠落・外殻のひび・赤いラベル）と
  **損傷レポート**、概要・部品ビュー・飛行データ（電池ゲージ・高度/速度・残り
  時間・帰還距離・航跡の地図）・システムタイル・全センサーのカード。
  「CARLA ビューを開く」でその機体の CARLA ウィンドウ（下記）。
- **統計**タブ（`#stats`）：現在セッションの集計タイル（衝突・車線逸脱・
  信号無視・急ブレーキ・距離・時間）、セッション別棒グラフ、日付ごとの
  セッション一覧表。
- ヘッダーの **EN / 日本語** ボタンで言語切替（URLに `?lang=ja` でも可）。
  設定は保存されます。
- **⬇ CSVログ** で現在のセッションのテレメトリをダウンロード。

---

## Data / 計測データ

Every run automatically saves to `data/<YYYY-MM-DD>/<HH-MM-SS>/`:

- `telemetry.csv` — 30 columns: position, speed & limit, pedals, steering,
  accelerations, jerk, yaw rate, slip, lane offset/width, TTC/THW, SDLP,
  steering reversal rate, hard brakes, lane invasions, red-light
  violations, speed compliance, collision count.
- `summary.json` — refreshed every 5 s while driving: crashes, out-of-lane
  count, red lights, hard brakes, distance, duration, average speed,
  compliance, SDLP. The Statistics page is built from these files.

実行のたびに `data/<日付>/<開始時刻>/` へ自動保存されます。

- `telemetry.csv` — 30列：位置、速度と制限速度、ペダル、ステア、加速度、
  ジャーク、ヨーレート、スリップ角、車線オフセット/幅、TTC/THW、SDLP、
  ステア反転率、急ブレーキ、車線逸脱、信号無視、速度遵守率、衝突回数。
- `summary.json` — 走行中5秒ごとに更新：衝突・車線逸脱・信号無視・
  急ブレーキ・距離・時間・平均速度・遵守率・SDLP。統計ページはこの
  ファイルから作られます。

---

## Pedal & steering tuning / ペダル・ステア調整

Pedal feel lives in `pedal_tuning.json` and can be changed **while
driving**:

- **Dashboard → "Pedal Tuning" card**: sliders for brake/throttle
  *strength* (output at full press) and *response curve* (higher = gentler
  at light press), with a live curve preview and a reset button. Changes
  reach the car within ~1 second.
- **In the driving window**: `[` / `]` = throttle strength down/up,
  `Shift+[` / `Shift+]` = brake strength down/up (5 % steps).
- Mapping: `output = strength × press^curve` (press = 0..1 pedal
  position). Defaults: throttle 80 % / curve 1.8, brake 100 % / curve 2.2.

Brakes too hard? → raise *brake response curve* or lower *brake strength*.
Acceleration too fast? → lower *throttle strength*.

Steering feel (gain / curve / clamp) is in `wheel_config.ini`
`[Sensitivity]`; axis and button indices are in `[G29 Racing Wheel]`.

ペダル感度は `pedal_tuning.json` に保存され、**走行中に**変更できます。

- **ダッシュボード →「ペダル調整」カード**：ブレーキ/アクセルの
  「強度」（全踏み時の出力）と「応答カーブ」（大きいほど浅い踏込みで
  穏やか）をスライダーで調整。カーブのプレビューとリセットボタン付き。
  変更は約1秒で車両に反映されます。
- **運転ウィンドウ内**：`[` / `]` = アクセル強度の増減、
  `Shift+[` / `Shift+]` = ブレーキ強度の増減（5%刻み）。
- 変換式：`出力 = 強度 × 踏込量^カーブ`（踏込量は0〜1）。
  初期値：アクセル80%/カーブ1.8、ブレーキ100%/カーブ2.2。
- ブレーキが強すぎる → ブレーキ応答カーブを上げるか強度を下げる。
  加速が速すぎる → アクセル強度を下げる。

ステアリング感度（gain / curve / clamp）は `wheel_config.ini` の
`[Sensitivity]`、軸・ボタン番号は `[G29 Racing Wheel]` にあります。

---

## Driver alerts / 運転警告

Bad driving triggers a large banner + warning beep, both in the driving
window and on the dashboard (thresholds live in `ALERT_RULES` in the G923
script and are the same rules used as RL reward penalties — see
`docs/RL_DESIGN.md`):

- **Danger (red, double beep)**: collision, crossing a **solid** line,
  forward collision risk (TTC < 1.5 s above 20 km/h), red-light violation.
- **Warning (amber, single beep)**: crossing a broken line, speeding
  (> limit + 10 km/h for 2 s), hard braking.
- Test the dashboard banner with `http://localhost:5000/?alerttest=1`.

危険な運転をすると、運転ウィンドウとダッシュボードの両方に大きな警告
バナーとビープ音が出ます（しきい値はG923スクリプトの `ALERT_RULES` に
あり、RL報酬ペナルティと同一ルール。`docs/RL_DESIGN.md` 参照）。

- **危険（赤・2連ビープ）**：衝突、**実線**の車線逸脱、前方衝突リスク
  （20km/h以上でTTC < 1.5秒）、信号無視。
- **注意（黄・1回ビープ）**：破線またぎ、速度超過（制限+10km/hを2秒）、
  急ブレーキ。
- ダッシュボードの警告テスト：`http://localhost:5000/?alerttest=1`

### Opening and closing windows / ウィンドウの開閉

The Drive tab's **Windows** card lists the optional windows: the **sensor
viewer** and the **drone cameras** (the latter needs `--delivery N`). Each row
has a status dot (green = open, with its pid), **Open** and **Close**. Started
with `--no-sensor-viewer`, or without `--drone-cams`? Press Open whenever you
want it; the pipeline starts it with the same options it would have used at
launch. Close stops only that window. Closing a window with its own close
button / Esc / Q is just as safe: the pipeline marks it closed instead of
restarting it or shutting down. A window that *crashes* is still restarted (3
times). Which windows are open is kept across F5: after a map switch the same
ones come back, whatever the launch flags said.

Not offered: the driving window (closing it ends the session), the dashboard
itself, traffic (the tick master), the delivery vans and force feedback --
stopping those changes or ends the drive. The controls only work under
`run_pipeline.py` (the card says so otherwise). Changes to the pipeline take
effect after its next restart.

How it works: the dashboard writes `pipeline_control.json`
(`{"drone_cams": {"action": "open", "t": ...}}`), run_pipeline.py carries it
out within a second and writes `pipeline_status.json` (open windows + pids).
`POST /windows {"window": "sensor_viewer", "action": "close"}` does the same
from a script (`windows_control.py`, `tests/test_windows_control.py`).

運転タブの**ウィンドウ**カードに任意ウィンドウ（**センサービューア**と
**ドローンカメラ**。後者は `--delivery N` が必要）が並び、状態の丸（緑 = 表示中、
pid 付き）と**開く**・**閉じる**ボタンがあります。`--no-sensor-viewer` や
`--drone-cams` なしで起動しても、いつでも「開く」で起動時と同じオプションで
開けます。「閉じる」はそのウィンドウだけを止めます。ウィンドウ自身の閉じる
ボタン / Esc / Q で閉じても安全で、再起動もシャットダウンもせず「閉じている」
扱いになります。*クラッシュ*した場合は従来どおり再起動（3 回）します。開閉状態は
F5 のマップ切替後も維持されます（起動オプションではなく現在の状態に従う）。

対象外：運転ウィンドウ（閉じるとセッション終了）、ダッシュボード自身、交通
（tick マスタ）、配送車、フォースフィードバック。止めると運転が変わるか終わる
ためです。操作は `run_pipeline.py` 配下でのみ有効です。パイプライン側の変更は
次回の再起動から反映されます。

仕組み：ダッシュボードが `pipeline_control.json` を書き、run_pipeline.py が
1 秒以内に実行して `pipeline_status.json`（表示中のウィンドウと pid）を書きます。

---

## Day, night and weather / 昼夜と天気

`weather_cycle.py` (started by `run_pipeline.py` on every map; `--no-weather-cycle`
turns it off) runs a whole day in 5 minutes and changes the weather by itself:

- **Clock and sun:** the sun's height and direction come from Oita's latitude and
  today's date, so nights are as long as they really are this season. The street
  lamps come on below 3° of sun (or in heavy rain / fog) and go off at dawn: on
  OitaKosen all 1,613 of our own lamps (BP_JpGaito, map guide ch. 28), on CARLA's
  towns their own. Your car's headlights come
  on at dusk, at night and in heavy rain or fog, and so do the NPC cars' lights.
- **Weather:** spells of 1.5-4 game hours: clear, partly cloudy, cloudy, rain or
  storm. Rain always comes after clouds, puddles grow while it rains and dry after.
  The wind is left to the seasons card and the delivery sim.
- **Rain and drones:** from 20 % rain (`delivery_sim.py --rain-ground`) every drone
  in the air comes down (logged as a `rain` crash) and the pods keep theirs inside
  until the rain stops (`rain_hold` / `rain_over` events).
- **Dashboard, Drive tab, card "Day & weather":** the time, the weather, whether
  drones may fly; day/night on/off, pause the clock, a time slider (any time, 15 min
  steps) and dawn / noon / dusk / night buttons, day length with − / + (1 min to
  real time: 1, 2, 3, 5, 10, 15, 20, 30, 45 min, 1, 2, 6, 24 h), street lamps Auto /
  On / Off and their brightness, force a weather (Auto, Clear, Partly cloudy, Cloudy,
  Rain, Storm), weather changes by itself or holds, rain chance. Saved in
  `weather_cycle.json`, read live; the state is in `data/weather_status.json`.

The C key's weather presets in the driving window are overwritten within a second
while the cycle runs (turn both off on the card to use them).
Test: `tests/test_weather_cycle.py`. Run by hand: `../../carla-venv/bin/python weather_cycle.py`.

`weather_cycle.py`（パイプラインが全マップで起動）は 1 日を 5 分で回し、天気も自動で変わります。
太陽は大分の緯度と今日の日付から計算、夜は自車と NPC のヘッドライトが点きます。雨（20 % 以上）では
飛行中のドローンはすべて墜落し、ポッドは雨が止むまでドローンを出しません。ダッシュボードの
「昼夜と天気」カードで、昼夜のオン/オフ・時計の停止・1 日の長さ・時刻・天気の固定・雨の確率を変更できます。

## Drone cameras / ドローンカメラ

`--drone-cams` (with `--delivery N`) opens a CCTV-style window with one tile
per drone. A drone in the air gets a small camera looking straight down, the
top of the picture towards the drone's nose; it sits 25 cm off the body's
centre so the parcel shows towards the lower left rather than in the middle; each tile
shows the drone, its van, status / flight phase, altitude, battery and order,
with a blinking REC. Drones that are docked, charging or waiting have no
camera at all (nothing is rendered for them) and read OFFLINE. Each camera
runs at 10 fps (`drone_viewer.py --cam-fps`). F11 fullscreen, Esc / Q closes.

`--drone-cams`（`--delivery` と併用）で、ドローンごとのタイルを並べた CCTV 風
ウィンドウを開きます。飛行中の機体だけに下向きカメラが付き（荷物は画面左下の
隅に映る）、飛んでいない機体は描画せず OFFLINE と表示します。

**Drone CARLA view.** **Open CARLA view** on the dashboard's Drones tab
opens a native CARLA window for that drone (`drone_window.py`, started
detached by the dashboard through `drone_windows.py`, one per drone; a
second click brings it forward when `wmctrl` / `xdotool` is installed,
otherwise reopens it). The big view is a chase camera behind and above
the drone body, so you see the 3D drone itself fly; a strip shows its four
onboard cameras (FPV gimbal, delivery camera looking down, front and rear
stereo). Keys: 1 chase, 2 orbit, 3–6 the onboard cameras, Tab next, H HUD,
F11, Esc / Q close. The HUD shows id, status, battery, altitude, speed,
heading and damage warnings. Docked, the camera stays on the pod; when
the sim respawns the body it re-attaches; closing destroys every sensor
(`role_name` `hil_dronewin_<id>`, leftovers swept at start). Nothing is
streamed into the browser. Test: `drone_window.py --drone 3 --snapshot
/tmp/d3.png --duration 12`. The 3D page model comes from the GLB:
`python3 -I delivery/assets/make_drone_web.py
~/Downloads/unreal/unreal/LastMilePod_Drone.glb web/src/views/drones/three/lmp_drone_parts.glb`.

**ドローンの CARLA ビュー**：ドローンタブの「CARLA ビューを開く」で、その機体の
CARLA ウィンドウ（`drone_window.py`）が開きます。大画面は機体の後方上から追う
チェイスカメラ（機体そのものが映る）、下段に機載カメラ 4 台。1〜6 キーで切替、
HUD に状態・電池・高度・速度・損傷を表示。格納中はポッドを映し、機体が変われば
付け直し、閉じるとセンサーをすべて破棄します。ブラウザには映像を流しません。

**Tooltips** (order / hub markers, station boxes, labels over drones and pods,
drone flight trails in the CARLA window) are off by default; turn them on in
the dashboard's Settings > Delivery sim ("CARLA tooltips"), or start with
`--delivery-args "--tooltips"`. The drones' winch tether always shows.

**Mini Pods tab.** The dashboard's **Mini Pods** tab (`#minipods`) watches the
LastMile Hubs' mini delivery pods like the Drones tab watches drones: a tab per
pod, the real pod in 3D (wheels, lid, shutter and flag move with the live
state, X-ray, broken parts shown broken), its hub and bays, drive data, a mini
map and every sensor (cards built from whatever the sim sends). **Open CARLA
view** opens `minipod_window.py` (chase camera behind the robot, orbit, six
onboard cameras, keys 1-8, HUD; docked pods are followed on the hub). On the
Delivery map, **Edit zones > Distribution points** places each LMH hub's
distribution points (square "D" badges, max 24 per hub); an LMH hub delivers in
the no-fly zones until you draw a zone for it. Details: `docs/DELIVERY.md`.

**ミニポッドタブ**：LastMile ハブのミニポッドを 1 台ずつ 3D で監視（車輪・蓋・
シャッター・旗が状態どおりに動く、X 線、損傷表示、ハブとベイ、走行データ、全セン
サー）。「CARLA ビューを開く」で `minipod_window.py`。配送ページの地図の「ゾーン
編集 > 配布地点」で LMH ハブの配布地点を置けます。

**ツールチップ**（マーカー・ラベル・飛行軌跡）は既定でオフ。ダッシュボードの
設定 > 配送シミュレーション「CARLA ツールチップ」でオン／オフできます。

---

## Skid marks / スキッドマーク

`traction_fx.py` (started by `run_pipeline.py`) draws a mark on the road
behind each sliding tyre: a band of thin dark lines across the tyre's width,
denser the harder it slides, building up over the first 2 m, and thinning out
until it is gone after 25 s (`--mark-life`; each line lives 35-100 % of it). Above 5 km/h the car is sliding when:

- the slip angle at the rear axle is over 8° (a gripping car stays under 7°
  even in tight junction turns; a drift or spin goes past 10°), or
- a tyre's slip ratio stays over 0.3 for 0.3 s with throttle or brake at 90 %
  or more, or the handbrake on (a full-throttle launch, an emergency stop).

CARLA's per-tyre slip readings alone jump to 0.3–0.5 on a gripping car when
throttle and brake change every tick, which is how the autopilot drives; the
old rule (tyre slip over 9° or 0.3) drew marks behind a car on autopilot.

タイヤが滑ると路面に黒いスキッドマークが出ます（強く滑るほど太く、25 秒で
消える）。判定は 5 km/h 以上で後軸の横滑り角 8° 超、または強いアクセル・ブレーキ・
サイドブレーキでタイヤのスリップ率 0.3 超が 0.3 秒続いた時（自動運転中の誤表示を修正）。

## Road logic overlay / 道路ロジック表示

Press **O** in the driving window (or use the Drive tab card **Road logic
overlay**) to see what the Traffic Manager, the autopilot and the NPCs drive
on: the OpenDRIVE map, which on your own maps is built from the OSM streets.
`road_overlay.py` (started by `run_pipeline.py`, idle until turned on;
`--no-road-overlay` skips it) draws it around your car with CARLA debug lines,
like CARLA's OpenDRIVE view in the UE4 editor, and your camera view shows them:

| Layer | In the world |
|---|---|
| Road edges (on) | red lines along both edges of every Driving lane; your lane thicker and pink |
| Driving direction (on) | blue lane centre + an arrow every 10 m near your car (keep-left shows here) |
| Junctions (on) | the paths through a junction orange (one line per lane), junction boxes yellow |
| Other lane types (on) | sidewalk grey, shoulder brown, parking purple, biking pink (CARLA's towns; our maps have none) |
| Rules (on) | 止まれ red bar + octagon, give way white, speed signs; traffic lights with their stop bars in the light's colour (red / yellow / green); the lane speed limit written on the road every 60 m |
| Lane change (off) | lane boundaries: cyan = the AI may change lanes both ways, yellow = one way, white = not; dashed = broken paint |
| Ids (off) | road / lane / junction ids |
| Raw OSM ways (off) | the source .osm streets in green (generated maps only), to compare OSM with the road logic |

It is drawn in two places that share one line budget: around your car (what
your driving window sees) and, with **Spectator area** on (default), around
what the CarlaUE4 window looks at: larger and with thicker lines when that
camera is high, and with text labels (the window shows CARLA's text, cameras
do not). Arrows, lane centres, numbers and sign shapes are drawn only within
45 m of your car.

The card switches it on / off, sets the radius (120 m), the brightness (3),
the arrows-and-text radius (45 m), the lines per refresh (800, nearest first),
the spectator area and the layers, and shows the colours. Debug lines glow:
the colours are sent very dim (channel values 1-4 of 255; 6 and up bloom the
whole picture white), and dimmer still at night. It redraws every 2 s; live on
OitaKosen it kept CARLA at 30 frames/s (29.3 while drawing; 2000 lines a
second had cost a third of the frame rate). Turned off, the lines are gone
within ~2.5 s. Files:
`data/road_overlay.json` (settings), `data/road_overlay_status.json` (state).
Tests: `tests/test_road_overlay.py`; live check with a camera shot:
`maps/road_overlay_live.py` (`--calibrate 1,3,6` draws a brightness ladder).
On our generated maps every lane boundary reads "lane change allowed" to the
AI, even on solid lines, and a road with no lines on it is not in the road
logic at all (map guide ch. 37).

運転ウィンドウの **O**（またはダッシュボード運転タブの「道路ロジック表示」）で、
Traffic Manager・自動運転・NPC が使う道路情報（OpenDRIVE。自作マップでは OSM の
道路から作成）を自車の周りに線で表示します。赤：走行車線の両端、青：車線中心と
走行方向、橙：交差点、規制：止まれ・譲れ・速度標識・信号の停止線（信号の色）・
車線の制限速度。CarlaUE4 ウィンドウのカメラの周りにも表示します（スペクテーター範囲）。
線は自発光でにじむため、色はとても暗く送ります（明るさ 3、夜はさらに暗く）。
2 秒ごとに最大 800 本を近い順に描き、CARLA の速度はほぼ変わりません
（実測 30 → 29.3 fps）。オフにすると約 2.5 秒で消えます。

---

## Traffic rules (Japan profile) / 交通ルール（日本プロファイル）

On Japanese maps the NPC cars follow Japanese traffic law as far as CARLA's
Traffic Manager can. `generate_traffic.py` picks the profile when it starts,
from `traffic_profiles.json` + `maps/maps.json`. A map gets `japan` when its
entry has `"road_style": "japan"`, or `"keep_left": true` in the `Japan`
package (OitaKosen, OitaStation). **Every other map, Pasir Ris included,
keeps the old traffic: no extra Traffic Manager call is made.** A map entry can
also name a profile itself (`"traffic_profile": "japan"` / `"none"`), and
`generate_traffic.py --traffic-profile none|japan` overrides it. After F5 the
pipeline restarts generate_traffic on the new map, so the profile is chosen again.

| Rule (source) | How the NPCs follow it |
|---|---|
| Keep left (道交法 17, 18 条) | the map's lanes (netconvert `--lefthand`); keep-"slow"-lane rule 0, because CARLA counts these roads as right-hand and would push cars into the overtaking lane |
| Red / yellow / green (NPA signal sheet) | TM stops on red **and** yellow at the stop line; never runs a light (`ignore_lights` 0) |
| No turn on red | TM treats every movement the same: red = stop |
| 止まれ: full stop, then go (43 条) | RuleKeeper `stop_signs`, at every 206 stop line of the map (its landmarks, lanes facing the sign): slow down, stop with the front at the line, stand still 2 s, go first come first served once nothing is in the junction or coming along the crossing road (max hold 12 s). The TM itself never stops there: its all-way stop needs a non-green light state, and a car under no light reads green / 止まれ: 停止線で 2 秒完全停止し、交差点が空いてから先着順で発進 |
| Speed limit (22 条; 60 km/h, 30 km/h without a centre line since 2026-09-01) | desired speed = the limit of the lane the car is on (xodr lane speed) × 0.9; 30 km/h where a lane has none; bicycles ≤ 18 km/h |
| Following distance (26 条) | gap 3.5 m (other maps: 2.5 m) plus TM's speed-based braking |
| No overtaking 30 m before a junction (30 条) | auto lane change off within 30 m of a junction |
| Pedestrians on the road (38 条) | stop while a walker is on the carriageway ahead, either half, until they have left it (held at most 15 s; a walker standing still for 8 s counts as stuck and is ignored) |
| Blinkers when turning (53 条) | TM drives each car's lights and blinkers |
| Pedestrians waiting at a crossing without signals (38 条; NPA "Pedestrians First") | with the map's `hokousha.json`: stop when a walker stands at the kerb of an unsignalised crossing or walks into it / 信号の無い横断歩道で待つ歩行者に停止 |
| Delivery vans (`delivery_sim.py`, own BasicAgent, not TM) | `delivery/rules.py` runs the same RuleKeeper for the vans while they drive: full stop at 止まれ, stop for walkers on the road / waiting at unsignalised crossings, speed capped at the lane limit × 0.9 (never above `--speed`). Vans parked for a delivery, charger or yard are left alone. `delivery_sim.py --traffic-profile none` turns it off; other maps unchanged / 配送車も止まれ・歩行者優先・車線速度に従う |
| Pedestrian signals 青 → 青点滅 → 赤 (道交法施行令 2 条) | `hokousha.py`: one pedestrian phase per signalised crossing from the junction's vehicle phases; the heads (歩行者用信号機, countdown column) show it and walkers wait at the kerb on red and flashing green, finish when already crossing No walk shorter than 5 s (short side phases stay red); flashing only after a walk / 歩行者用信号: 灯器と歩行者が同じ現示に従う |
| Speed limits (施行令 11 条; maps/osm_city/jp_speeds.py, map guide ch. 29) | each car drives its lane's limit × 0.9: the outer ring 100 km/h (44 posts), JARTIC zones and real OSM limits where they exist, else the legal limit, 60 with a centre line / 30 without; cars slow for the bends ahead (`speed.curve`: the ring's 15 m corners ~24 km/h). The dashboard and the driving window show the lane's limit on your own maps (`lane_limits.py`; CARLA's sign-based limit stays 30 on these keep-left roads) / 速度制限: 外周 100、他は実際の規制・法定速度 |
| Real signal timing (JARTIC type C, Oita police) | `run_pipeline.py` starts `signal_timing.py apply --watch --only-map <Map>` on maps with maps.json `"signal_timing"` (OitaKosen): cycle and green / yellow 3 s / all-red 2 s by weekday or holiday and 5-minute slot, re-applied every slot; other maps and F5 switches away keep CARLA's times / 実際の信号時間を自動適用 |

Not possible with the TM (left as they are): yellow-light "go on if you
cannot stop safely" (the TM always stops), flashing yellow / red (CARLA lights
have no flashing state), right turns giving way to oncoming traffic (TM only
avoids collisions), stopping for a pedestrian *waiting* at a crossing
(`get_crosswalks()` is empty on our maps), level crossings (none on our maps),
giving way at unsignalised junctions without a 止まれ (TM collision avoidance only).
Live check of the stops: `carla-venv/bin/python maps/stop_live.py` (read-only;
`--role delivery` for the vans only, a per-role table at the end).
With `--driver-profile` (study arm B) only the fixed settings apply, then the
human-error profile overrides them; the helper is off.
Tests: `tests/test_traffic_profile.py`, `tests/test_delivery_rules.py`. Details: map guide ch. 23.

日本のマップ（maps.json の `road_style: japan`、または Japan パッケージの
`keep_left`）では NPC が日本の交通ルールに従います：信号・標識・歩行者を無視しない、
車線の最高速度 × 0.9 で走る（標識の無い車線は 30 km/h、自転車は 18 km/h 以下）、
車間 3.5 m、交差点手前 30 m は車線変更しない、前方の車道を歩行者が渡っている間は
停止、「止まれ」（206）の停止線で 2 秒完全停止して交差点が空いてから先着順で発進
（TM 自身は信号の無い交差点で止まらないため補助で行う）、ウインカーは TM が出す。
黄信号で止まれない場合の進行・点滅信号・右折時の対向車優先・踏切・止まれの無い
交差点での優先は TM では扱えません。
配送車（delivery_sim.py）も走行中は同じ補助で止まれ・歩行者・車線速度に従います（配送の停車には干渉しない）。
他のマップ（Pasir Ris を含む）の交通は従来どおりです。`--traffic-profile none` で無効。
パイプラインの次の起動（または F5 のマップ切替）から有効になります。

### Timetabled buses (Oita Station) / 路線バス（時刻表）

On OitaStation the real Oita buses (大分バス, 大分交通, highway buses) drive
into the map, call at their stops and leave it again on the published
timetable (GTFS-JP from Oita Prefecture's open data, `data/oita_bus/`,
`data/oita_bus/sources.md`). `generate_traffic.py` runs `bus_schedule.py` on
maps whose maps.json entry has `"bus_schedule": "data/oita_bus"` (only
`oita_station`); every other map is unchanged. It needs the **new** OitaStation
road logic (installed 2026-11-09; it refuses the old ±423 m level by itself).

| What | How |
|---|---|
| The bus | `vehicle.oitabus.erga` (Isuzu Erga, cars.json `oitabus`, npc_only; built and installed 2026-11-09); CARLA's `vehicle.mitsubishi.fusorosa` stands in only if that blueprint is missing. role_name `timetable_bus` / 大分バス（エルガ、2026-11-09 導入） |
| Where it drives | its route's real roads (the GTFS shape) on the lanes, kerb-side (left) lane at each stop, door at the pole; enters / leaves just inside the ring road; trips that start or end at the station come / go empty (回送) by the edge their route uses |
| When | spawned so it reaches its first stop on time (way in at 18 km/h, never before its last stop outside the map); dwell 20-40 s boarding, 10-20 s alighting; at Oita Station it waits for the departure time (max 5 min) |
| Rules | the Traffic Manager drives it with the Japan profile (lights, 止まれ, walkers, lane speeds), ≤ 40 km/h, no own lane changes; left blinker pulling in, right pulling out; it waits before a junction when a bus stands at the stop behind it |
| Clock | the time of day the sim shows (Day & weather), then **real time** (the day cycle runs a day in 5-15 minutes, which would flood the station); setting the clock on the dashboard re-anchors it. Day type from the date: weekday / Saturday / Sunday-holiday (every national holiday, also 11/3 and 11/23, which the feed misses) |
| Safety | ≤ 25 buses at once (`--bus-max`); a bus without progress for 3 min is removed and logged; all go at exit and on F5 |

Options: `generate_traffic.py --bus-schedule auto|none|<folder> --bus-time 08:00
--bus-day weekday|saturday|sunday_holiday --bus-max 25`. One log line per bus
event (`[bus] 08:01:16 depart ...`); status for the dashboard's "Timetabled
buses" card (Drive page, Environment; `/buses`) in `data/bus_status.json`.
Offline mapping report: `carla-venv/bin/python bus_schedule.py report`.
Live check (after the import): `maps/bus_live.py watch` (read-only) or
`maps/bus_live.py spawn --time 08:00 --day weekday --n 5` (own role and TM port,
removes its buses). Tests: `tests/test_bus_schedule.py`. Details: map guide ch. 36.
Credit / 出典：大分県オープンデータカタログ（BODIK）「大分バスGTFSデータ」「大分交通GTFSデータ」「高速バスGTFSデータ」（CC BY 4.0、大分県）を加工して作成; road names © OpenStreetMap contributors.

大分駅マップでは、公開時刻表どおりに路線バスがマップに入り、停留所（大分駅前 1〜8 番のりば・
降車専用・上野の森口・要町ほか）でポールに扉を合わせて停まり、出て行きます。時刻はシムの表示時刻
から実時間で進みます（昼夜サイクルは速すぎるため）。祝日は日祝ダイヤ。同時 25 台まで、3 分動けない
バスは削除。他のマップは従来どおり。新しい OitaStation（2026-11-09 取り込み済み）でパイプラインの次の起動から有効です（実機確認はこれから）。

### Parked cars / 駐車車両

On maps whose build has `maps/build/<Map>/parked_cars.json` (OitaStation: 440
spots where the v8 model had its parked cars, made by `maps/osm_city/core_cars.py`),
`generate_traffic.py` parks real vehicles there (`parked_cars.py`): CARLA's cars
and your own cars from `cars/cars.json` (not the F1 car, not `npc_only` vehicles
such as the bus), physics off, lowered onto the ground, nose either way; a car too
big for its spot is swapped for a smaller one. A spot whose ground does not match
the loaded level (a build newer than the installed map) stays empty. role_name
`parked`; removed at exit, leftovers of a killed run cleared at start.
Options: `--parked-cars auto|none`, `--parked-max N` (0 = every spot).
Test: `tests/test_parked_cars.py`.

駐車位置のあるマップ（OitaStation の 440 か所）では、CARLA の車と自作車両（F1 と
npc_only を除く）を物理なしで地面に置きます。`--parked-cars none` で無効、
`--parked-max N` で台数の上限。

### Lane changes follow the paint (Japan maps) / 車線変更は路面標示どおり（日本のマップ）

netconvert writes no `laneChange` into the road logic, so CARLA let the AI change
lanes everywhere, even across solid lines. On Japan maps `maps/osm_city/lane_change_jp.py`
(run by `make_osm_city.sh`) now writes every lane mark from the painted lines:
yellow solid between lanes (進路変更禁止) = no lane change, white solid or broken =
allowed (legal in Japan), solid kerb / centre lines = none. Installed on OitaStation
2026-11-09; OitaKosen's patched road logic is waiting for its install. See it in the
game with the road logic overlay (O, layer "lane change"). Map guide ch. 39.

道路ロジックに laneChange が無く、AI が実線でも車線変更していました。日本のマップでは
塗られた線から書き込みます（黄色実線 = 進路変更禁止、白線・破線 = 可）。OitaStation は
2026-11-09 導入済み、OitaKosen は導入待ち。

---

## Own maps / 自作マップ

Built with `maps/make_osm_city.sh <key> --lefthand` (OpenStreetMap -> road logic ->
generated 3-D model -> import; the carla-map-generator skill and the map guide
`../../map_guide/README.md`). They appear on the Launch page and in F5 by themselves.

| Map | What |
|---|---|
| OitaStation (`oita_station`) | the user's v8 model of JR Oita Station kept as the core (±400 m), inside a generated city to ±640 m: generated Japanese buildings and shop fronts, core ground in the generator's materials, lanes fitted to the model's asphalt, 440 parking spots, timetabled buses, Japanese signals, plates and speed posts. Imported 2026-11-09. `oita_station_v8` (the model alone) is the rollback |
| OitaKosen (`oita_kosen`) | hilly campus area: terrain, campus / dorm / 7-Eleven landmarks, JARTIC signs, real signal timing, seasonal trees, street lamps, ring road at 100 km/h |
| PasirRis (`pasir_ris`) | Singapore estate, keep-left, two school landmarks |
| MarinaBay, SpaFrancorchamps | race circuits (`maps/make_map.sh`) |

大分駅マップはユーザーの v8 モデルを中心に残し、周囲 ±640 m を OSM から生成した拡張版です
（2026-11-09 取り込み）。他に大分高専（起伏・ランドマーク）、パシリス、サーキット 2 つ。

---

## Your own cars / 自作車両

`cars/cars.json` lists your own car models: Mazda RX-7 (`vehicle.mazda.rx7`),
Toyota Supra A70 (`vehicle.toyota.supra_a70`), Mitsubishi i-MiEV
(`vehicle.mitsubishi.i_miev`), Honda NSX-R (`vehicle.honda.nsx_r`), Toyota
Soarer (`vehicle.toyota.soarer`), Nissan Sileighty (`vehicle.nissan.sileighty`),
all right-hand drive, and BMW 3.0 CSL (`vehicle.bmw.csl_e9`, left-hand drive
in the model), Red Bull RB16 (`vehicle.redbull.rb16`, centre seat), Ferrari SF90
XX (`vehicle.ferrari.sf90_xx`) and Rolls-Royce Dawn (`vehicle.rollsroyce.dawn`),
both converted to right-hand drive, Honda S800 (`vehicle.honda.s800`), and the
NPC-only Oita bus (`vehicle.oitabus.erga`, driven only by the bus timetable). All
12 were rebuilt on 2026-11-09. `cars/build_cars.sh` turns each into a CARLA vehicle plus its
steering wheel as a separate prop;
restart CARLA afterwards. In the driving window these cars get:

- a driver-eye camera at their own (right-hand) seat, measured from the
  model, so the view fits the car whatever its size;
- a steering wheel that turns 1:1 with the G923 (900° lock to lock,
  `wheel_range_deg` in `wheel_config.ini`);
- live eye adjustment: **Ctrl+I/K** forward/back, **Ctrl+J/L** left/right,
  **Ctrl+U/O** down/up, 2 cm a press, saved per car in `cars/car_views.json`.

To add a car: a model with separate wheels and a separate steering wheel
(FBX, glb or OBJ), an entry in `cars.json` (which objects are the wheels and
the steering wheel, which way the nose points, real length, mass, and the
window materials, which are made invisible so the view out is clear), then
`cars/build_cars.sh`. CARLA's own vehicles have their driver eye measured too
(`cars/carla_views.json`, checked with camera images: all left-hand drive,
the bus and trucks inside their cabs, bikes just ahead of the built-in rider),
and the same Ctrl keys tune and save it. Their steering wheels are part of the
body mesh, so only your own cars' wheels turn. Hood and Chase views scale with
every vehicle's size.

`cars/cars.json` の自作車両（右ハンドル）を `cars/build_cars.sh` で CARLA に
取り込みます（終了後 CARLA 再起動）。運転時は車両ごとの運転席視点、実ハンドル
と 1:1 で回るハンドル、Ctrl+I/K/J/L/U/O による視点調整（車両ごとに保存）が
使えます。

---

## RL research plan / RL研究計画

`docs/RL_DESIGN.md` describes how to use the collected human-error driving
data to train an autonomous agent: observation/action spaces, reward rules
shared with the driver alerts (§3.1 has the reward formula and how to
weight it), human-error scenario library, training recipe, and the full
run/evaluate workflow (§6). Implementation: `rl/` + `agent_drive.py`.

`docs/RL_DESIGN.md` に、収集した人間のエラー運転データで自動運転
エージェントを学習させる設計（観測・行動空間、運転警告と共通の報酬ルール
〈§3.1に報酬の式と重み付けの指針〉、人間エラーシナリオ集、学習レシピ、
§6に実行・評価手順）をまとめています。実装は `rl/` と `agent_drive.py`。

Quick start / クイックスタート:

```bash
../../carla-venv/bin/pip install stable-baselines3    # one-time / 初回のみ
python -m rl.extract_scenarios           # mine human errors / 人間エラー抽出
python -m rl.smoke_test                  # env check (CARLA running) / 環境確認
python -m rl.train_sac --timesteps 200000
python run_pipeline.py --agent model:rl/results/models/sac_ego.zip   # human vs agent
```

To benchmark the agent with the same dashboard metrics as human sessions,
run it as `hero`: `python agent_drive.py --rolename hero --policy model:...`.
/ 人間セッションと同一のダッシュボード指標で評価するには `hero` として
走らせます：`python agent_drive.py --rolename hero --policy model:...`

---

## Driving controls / 運転操作

These are the **default** bindings. Change them on the dashboard
(Settings > Keybinds, see below); H in the driving window always shows the
bindings in use. / 以下は **既定の** 割り当てです。ダッシュボードの
設定 > キー割り当て で変更できます（下記）。運転ウインドウの H で現在の
割り当てを表示します。

| Key / キー | Action / 動作 |
|---|---|
| Wheel + pedals / ホイールとペダル | steer, throttle, brake / 操舵・加速・制動 |
| W / ↑ | throttle / アクセル |
| S / ↓ | brake; reverse once stopped / ブレーキ（停止後はリバース） |
| A / D | steer left / right / 左右へ操舵 |
| SPACE | handbrake / ハンドブレーキ |
| TAB | camera: Driver Eye / Hood / Chase / カメラ切替 |
| N | toggle mirrors / ミラー表示切替 |
| Z / X (hold) | glance left / right / 左右目視（長押し） |
| Q | reverse / リバース |
| E (wheel: Share) | hazard lights on / off / ハザードランプ（ホイール：Share） |
| M , . | manual gears / MTモード・シフト |
| P | smooth autopilot (lane limits, slows for bends, turns, lights and cars; `--tm-autopilot` = old Traffic Manager) / なめらかな自動運転 |
| C / Shift+C | weather / 天候切替 |
| `[` `]` (Shift: brake) | pedal strength / ペダル強度調整 |
| Backspace | next vehicle model, alphabetical (Shift = previous) / 次の車種（アルファベット順、Shift で前） |
| F5 / Shift+F5 | next / previous map (see below) / 次・前のマップ（下記） |
| F11 | fullscreen / フルスクリーン |
| O | road logic overlay: what the AI drives on, drawn on the roads (see "Road logic overlay") / 道路ロジック表示（上記） |
| F1 | HUD info / HUD情報 |
| H | help / ヘルプ |
| ESC | quit / 終了 |

Wheel buttons (G923, PlayStation version) by default / ホイールボタンの既定値:
Cross (×) next car / 次の車種 · Square (□) HUD · Circle (○) camera / カメラ ·
Triangle (△) weather / 天候 · R / L paddle gear up / down / シフト ·
R2 reverse / リバース · L2 handbrake / ハンドブレーキ · Options automatic / AT ·
R3 / L3 right / left signal / 右・左ウインカー · Enter next sensor / 次のセンサー.

### Changing the map (F5) / マップ切替（F5）

F5 goes to the next installed map, Shift+F5 to the previous one. The cycle is
built from the server's own map list, installed maps only, in a fixed order:
Town01 … Town10HD, then your own maps by package (MarinaBay,
SpaFrancorchamps, OitaKosen, OitaStation, PasirRis …). Maps that are not
installed (e.g. Town06/07 in this package) never appear.

Under `run_pipeline.py` the driving window does **not** load the map itself:
it shows "Switching to <map>…", writes `map_request.json` and closes (exit
code 75). The pipeline then stops every client (traffic first), loads the
map with CarlaUE4 still running, and starts them all again with the same
options and the **same car** (`--start-vehicle` = the car you were driving).
Left-hand traffic / hybrid physics, the map's start position, delivery,
season and wind are set for the new map as at startup. The windows are gone
for ~20–60 s while that happens. If the load fails, the pipeline restarts
CarlaUE4 on that map (never a second one beside it). Run standalone, the
driving window still loads the map itself.

F5 で次、Shift+F5 で前のインストール済みマップへ切り替える。巡回順はサーバの
マップ一覧から作り（インストール済みのみ）、順序は固定：Town01〜Town10HD、
次に自作マップをパッケージ順に（MarinaBay、SpaFrancorchamps、OitaKosen、
OitaStation、PasirRis …）。未インストールのマップ（本パッケージの Town06/07
など）は出てこない。

`run_pipeline.py` 配下では運転ウインドウ自身はマップを読み込まない。
「Switching to <map>…」を表示し、`map_request.json` を書いて終了する（終了
コード 75）。パイプラインが全クライアントを止め（交通を最初に）、CarlaUE4 は
動かしたままマップを読み込み、同じオプション・**同じ車**（`--start-vehicle`
= 運転していた車）で全てを再起動する。左側通行・ハイブリッド物理、マップの
スタート位置、配送、季節と風は起動時と同様に新しいマップに合わせる。切替中の
約 20〜60 秒はウインドウが消える。読込に失敗した場合は CarlaUE4 をそのマップで
再起動する（2 つ目を並べて起動はしない）。単体起動の運転ウインドウは従来どおり
自分でマップを読み込む。

### Keybinds page / キー割り当てページ

Dashboard **Settings > Keybinds** (`http://localhost:5000/#settings-keybinds`)
lists every action of the driving window with two columns, **Keyboard** and
**Wheel (G923)**. Wheel buttons are shown by the name printed on the wheel
(Cross, Triangle, R paddle, Options, D-pad up...), never by number.

- Click a binding to change it, **+** to add a second one, **×** to remove it.
  Keyboard: press the key (Ctrl / Shift / Alt combinations work; actions you
  hold, like throttle, ignore modifiers). Wheel: press the button on the wheel.
  The driving window reads the wheel and reports the press, so it must be running.
- Two actions on the same input are flagged in yellow. **↺** resets one row,
  **reset all** every row; nothing changes until **save**.
- The driving window re-reads `keybinds.json` about once a second, so a saved
  change applies while driving.
- **Wheel button names** card: turn **identify buttons** on and press each
  button once. If the name shown is right click **Correct**, otherwise pick the
  right name (or type one) and **Rename**. While identifying, wheel buttons do
  nothing in the driving window. The names live in `g923_buttons.json`.
- Files: `keybinds.json` (bindings, created on first run from the defaults;
  the five button numbers in `wheel_config.ini` are the defaults for reverse,
  handbrake, gear up / down and automatic), `g923_buttons.json` (names),
  `wheel_last.json` (last wheel press) and `keybind_learn.json` (identify mode).
  All are regenerated when deleted. Throttle, brake and steering on the wheel
  are axes and stay in `wheel_config.ini`.
- Besides the old actions there are **Hazard lights** (default: E, wheel
  Share; also the hazard button on the dashboard's Drive and Vehicle pages)
  and keyboard keys for the turn signals (unbound until you bind them).

ダッシュボードの **設定 > キー割り当て** に運転ウインドウの全操作が
**キーボード** と **ホイール（G923）** の 2 列で並びます。ホイールのボタンは
番号ではなく刻印の名前（×・△・右パドル・OPTIONS・十字キー ↑ など）で表示します。

- 割り当てをクリックで変更、**＋** で 2 つ目を追加、**×** で削除。キーボードは
  キーを押す（Ctrl / Shift / Alt 併用可。アクセルなど押し続ける操作では修飾キーは
  無視）。ホイールはボタンを押す（ホイールを読むのは運転ウインドウなので起動が必要）。
- 同じ入力が複数の操作に割り当てられると黄色で警告。**↺** で 1 行、
  **すべて既定に戻す** で全行を既定に戻し、**保存** するまで反映されません。
- 運転ウインドウは約 1 秒ごとに `keybinds.json` を読み直すため、保存した変更は
  運転中に反映されます。
- **ホイールのボタン名** カード：**ボタンを識別** をオンにして各ボタンを 1 回ずつ
  押し、名前が正しければ **正しい**、違えば正しい名前を選ぶか入力して
  **名前を変更**。識別中はホイールのボタンを押しても運転ウインドウでは何も
  起きません。名前は `g923_buttons.json` に保存されます。
- アクセル・ブレーキ・操舵はホイールの軸なので引き続き `wheel_config.ini` で設定します。
- **ハザードランプ**（既定：E キー、ホイールの Share。ダッシュボードの運転・車両ページのボタンでも可）と
  ウインカーのキーボード操作（既定は未割り当て）があります。

### Three screens / 3 画面表示

Launch page → Display → **Screens: triple** (or `run_pipeline.py --screens
triple`) puts your view across three side-by-side monitors of the same size
(5760×1080 on three 1080p screens), fullscreen across all three
(`triple_screen.py`: monitors from `xrandr`, the EWMH
`_NET_WM_FULLSCREEN_MONITORS` hint, so GNOME's top bar does not cover it).

- **3-screen layout `angled`** (side monitors turned towards you): three
  cameras at the driver's eye, each a third of the **3-screen field of view**
  (default 150° = 50° each) and yawed by it, so the view runs on across the
  bezels. **`flat`** (monitors in one line): one wide camera.
- The HUD and the rear-view mirror are on the middle screen, the door
  mirrors on the side screens near their inner edges.
- Three 1080p cameras are three render passes for CARLA; lower the frame cap
  or mirror rate if the server slows down. Tests: `tests/test_triple_screen.py`.

横並び 3 画面に運転視点を表示（angled は画面ごとにカメラ、flat は横長カメラ 1 台）。

### Autopilot (P) / 自動運転

P hands the car to `hero_autopilot.py`: it roams like the Traffic Manager
did (a random way at each junction, CARLA's LocalPlanner steers), but drives
at the lane's own limit (100 on the OitaKosen ring, the real limits
elsewhere, the sign limit on CARLA's towns) and slows only where it must:
a lower limit ahead, a bend or a turn (2.6 m/s² sideways), a red light, the
car in front (1.4 s gap) or a pedestrian on its path. The pedals come from
a jerk-limited acceleration command with a coast band between throttle and
brake and rate limits on both, so they do not pump; the automatic gearbox
shifts. It stops at every 止まれ line (2 s, like the NPC rule), and when it is
wedged (throttle on, no progress for 6 s) it reverses 3 m with the nose turning
back to the lane and carries on (`recovery.py`, at most 3 times in 2 min). The
old Traffic Manager autopilot (30 km/h on these maps, on/off pedals) is
`--tm-autopilot`. Tests: `tests/test_hero_autopilot.py`, `tests/test_recovery.py`.

### Nothing stays stuck / 立ち往生の復帰

`recovery.py` runs inside the traffic script and the delivery sim: an NPC car
that has not moved 1 m in 20 s (and is not held by a rule, queued behind a car
or at a red light) is taken off autopilot, reversed 3.5 m out of its wedge and
handed back; after 3 failed tries in 3 min it is moved 12 m on along its lane.
A wedged pod reverses the same way before it is re-planned or nudged
(`reverse_out` event). A walker still for 90 s (not waiting at a signalised
crossing) gets a new destination, and the third time a new pavement point.

NPC 車・ポッド・歩行者が動けなくなったら後退（または経路変更）で復帰させる。

P で `hero_autopilot.py` が運転：車線の制限速度で走り、カーブ・曲がり角・信号・前車・
歩行者の手前だけ減速。ペダルは躍度制限と惰行帯で踏み直しをしない。

### Keyboard driving (no wheel) / キーボード運転（ホイール無し）

If no G923/G29 is detected the driving window falls back to the keyboard
automatically — W/A/S/D (or the arrow keys) steer and work the pedals, and
the terminal prints which mode was chosen. Pass `--keyboard` to
`run_pipeline.py` (or to the G923 script directly) to force it even with a
wheel plugged in, e.g. for a quick test run:

```bash
python run_pipeline.py --map Town02 --vehicles 20 --walkers 5 --keyboard
```

Throttle and brake ramp up over a few tenths of a second rather than
slamming to full, obey the same strength caps as the pedals
(`pedal_tuning.json`, still adjustable live with `[` / `]` and on the
dashboard), and S only engages reverse once the car has stopped — so
keyboard sessions produce telemetry comparable to wheel sessions.

G923/G29 が検出されない場合、運転ウィンドウは自動でキーボード操作に
切り替わります（W/A/S/D または矢印キー。どちらのモードになったかは
端末に表示されます）。ホイール接続時にも強制したい場合は
`run_pipeline.py`（または G923 スクリプト）に `--keyboard` を渡します。

アクセル・ブレーキは一気に全踏みせず数十分の一秒かけて立ち上がり、
ペダルと同じ強度上限（`pedal_tuning.json`。`[` / `]` とダッシュボードから
走行中に調整可能）に従います。S は車が停止してから初めてリバースに
入るため、キーボード走行でもホイール走行と比較可能なテレメトリが
得られます。

---

## Troubleshooting / トラブルシューティング

- **"port 2000 already in use" or signal-11 crash at start** — a previous
  CARLA is still running. The pipeline will reuse it automatically; for a
  fresh simulator run `pkill -f CarlaUE4-Linux-Shipping` first.
  / 起動時に「port 2000 already in use」やsignal-11クラッシュが出る場合、
  前回のCARLAが残っています。パイプラインは自動で再利用しますが、
  新規に起動したいときは先に `pkill -f CarlaUE4-Linux-Shipping` を実行。
- **Rear mirror black/missing** — mirror cameras spawn above the roof and
  auto-respawn if they die; toggle N if needed.
  / ルームミラーが映らない場合：ミラーカメラは屋根上方に生成され、
  停止時は自動再生成されます。必要ならNキーで表示を切替。
- **Wheel not detected** — the sim now keeps running on the keyboard
  (W/A/S/D) instead of exiting. To get the wheel back: the G923 needs the
  new-lg4ff DKMS driver; check axes with `jstest-gtk` and fix indices in
  `wheel_config.ini`.
  / ホイールが認識されない場合：終了せずキーボード（W/A/S/D）操作で
  続行します。ホイールを使うには new-lg4ff DKMSドライバが必要です。
  `jstest-gtk` で軸を確認し `wheel_config.ini` の番号を修正してください。
- **F5 (change map) used to crash everything** — fixed: the pipeline now
  switches the map for all clients (see *Changing the map (F5)* and
  `docs/TROUBLESHOOTING.md`). Restart the pipeline once after updating.
  / F5（マップ切替）で全体が落ちていた件は修正済み。パイプラインが全
  クライアントごと切り替える（上記と `docs/TROUBLESHOOTING.md` を参照）。
  更新後に一度パイプラインを再起動すること。
- **A viewer window keeps coming back after I close it** — that was the old
  restart-on-exit; now a window closed with its close button / Esc / Q (or the
  dashboard's Close) stays closed. Restart the pipeline once after updating.
  The Windows card says "run_pipeline.py is not running" when the dashboard
  was started on its own: the controls need the pipeline.
  / ビューアを閉じても再び開く場合：旧版の挙動です。現在は閉じるボタン / Esc /
  Q（またはダッシュボードの「閉じる」）で閉じたままになります。更新後に一度
  パイプラインを再起動すること。ダッシュボード単体起動ではカードに
  「run_pipeline.py が動いていません」と出ます（操作にはパイプラインが必要）。
- **Every client times out right after the map loads (big own maps)** — fixed
  2026-11-09: the pipeline loads the map with `load_world_map` (waits up to 120 s
  and checks the loaded map) instead of `PythonAPI/util/config.py` (10 s).
  / 大きな自作マップで読込直後に全クライアントがタイムアウト：修正済み（120 秒待って確認）。
- **Dashboard shows "waiting for hero"** — the driving window isn't up yet
  or the hero vehicle died; it recovers automatically.
  / ダッシュボードが「車両待機中」のまま：運転ウィンドウが未起動か
  自車が消滅しています。自動で復帰します。
