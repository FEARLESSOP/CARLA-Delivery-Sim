# RL Design: Autonomous Agent Robust to Human Driver Errors
# RL設計：人間ドライバーのエラーに頑健な自動運転エージェント

**Goal / 目的** — Train an autonomous-vehicle (AV) agent with reinforcement
learning so it can drive safely *alongside imperfect human drivers*. The
human driving data collected by this HIL rig (with its errors: lane
departures, hard brakes, red-light runs) is the key ingredient — the agent
must learn to anticipate and tolerate those errors, not just drive well on
rule-following traffic.

強化学習（RL）で、**エラーを起こす人間ドライバーと共存して**安全に走れる
自動運転エージェントを訓練する。本HIL環境で収集する人間の運転データ
（車線逸脱・急ブレーキ・信号無視などのエラーを含む）が鍵であり、
エージェントはルール通りのNPC交通ではなく「人間らしいエラー」を予測・
許容できるように学習する。

> **Experimental design** (arms, subjects, endpoints, statistics) is fixed
> in [`RESEARCH_PROTOCOL.md`](RESEARCH_PROTOCOL.md) (2026-07-10).
> / 実験設計（アーム・被験者・評価項目・統計）は
> [`RESEARCH_PROTOCOL.md`](RESEARCH_PROTOCOL.md) で確定（2026-07-10）。

---

## 1. Three-phase architecture / 3フェーズ構成

```
Phase 1: COLLECT            Phase 2: MODEL               Phase 3: TRAIN
┌───────────────────┐       ┌────────────────────┐       ┌─────────────────────┐
│ Human drives hero │       │ Human behavior     │       │ RL agent drives ego │
│ (this pipeline)   │  ───► │ model / replay     │  ───► │ AV; human(-like)    │
│ data/<day>/<time>/│       │ from telemetry.csv │       │ drivers around it   │
└───────────────────┘       └────────────────────┘       └─────────────────────┘
```

1. **Collect / 収集** — Already working: every session saves ~10 Hz
   telemetry (30 columns) + event counters. Keep collecting sessions with
   many different drivers and conditions.
   / 既に稼働中。セッションごとに約10Hzのテレメトリ（30列）とイベント集計を
   保存。ドライバーや条件を変えて収集を続ける。
2. **Model / モデル化** — Turn logs into *error-prone human driver agents*:
   - **Replay**: re-drive recorded trajectories with `carla.Client.replay_file`
     or by applying logged controls — exact but not reactive.
   - **Behavior cloning (BC)**: small policy net (input: same features as
     telemetry; output: throttle/brake/steer) fitted per driver — reactive
     and can be perturbed.
   - **Parameterized errors**: measure each driver's statistics (SDLP,
     reversal rate, THW distribution, red-light probability) and inject them
     into CARLA's TrafficManager (`vehicle_percentage_speed_difference`,
     `distance_to_leading_vehicle`, `ignore_lights_percentage`,
     `ignore_vehicles_percentage`, lane-offset noise) — cheap and scalable.
   / ログを「エラーを起こす人間ドライバーのエージェント」に変換する。
   リプレイ（正確だが非反応的）、行動クローン（反応的・摂動可能）、
   パラメータ化エラー（SDLP・車間分布・信号無視率などを測り、CARLAの
   TrafficManagerに注入。安価でスケールする）の3通り。
3. **Train / 訓練** — RL agent controls an ego AV in scenarios populated by
   the human(-like) drivers from Phase 2.
   / フェーズ2の人間（らしい）ドライバーを配置したシナリオで、RLエージェント
   がego車を制御して学習する。

**HIL co-simulation option / HIL共同シミュレーション** — Because the G923
pipeline and the agent are both just CARLA clients, the human can drive the
`hero` vehicle *live* in the same world while the agent drives an `ego`
vehicle (`role_name='ego'`). This is the highest-value evaluation: the agent
vs. a real human, today, with no extra infrastructure.
/ G923パイプラインもエージェントも同じCARLAクライアントなので、人間が
`hero` を運転しながら、同一ワールドでエージェントが `ego`
（`role_name='ego'`）を運転できる。追加設備なしで「本物の人間 vs
エージェント」を評価できる最重要構成。

---

## 2. Environment specification / 環境仕様

Gym-style episode loop over CARLA in **synchronous mode** (`fixed_delta_
seconds = 0.05`, 20 Hz; use asynchronous mode only for live-human HIL runs).
/ CARLAを**同期モード**（0.05秒刻み・20Hz）でGym形式のエピソードループに
する。人間が同時走行するHIL評価時のみ非同期モード。

**Observation / 観測** (all obtainable with the code already in
`dashboard.py`'s telemetry worker):
- Ego state: speed, lon/lat acceleration, yaw rate, lane offset & width,
  heading error to lane, distance to next junction, speed limit, traffic
  light state.
- Neighbors: K=6 nearest vehicles in ego frame — (dx, dy, relative speed,
  heading), the same math as the dashboard mini-map (`nearby_list`).
- Route: next N=10 waypoints in ego frame (from `GlobalRoutePlanner`).
- Optionally a semantic BEV image for a CNN policy; start with the vector
  observation — it trains far faster.

/ 観測：自車状態（速度・前後/横加速度・ヨーレート・車線オフセット/幅・
車線に対する方位誤差・制限速度・信号状態）、近傍K=6台の相対状態
（ダッシュボードのミニマップと同じ計算）、経路の先読みウェイポイントN=10。
まずはベクトル観測で開始（学習が速い）。

**Action / 行動** — continuous 2D: steering ∈ [-1,1], accel ∈ [-1,1]
(positive → throttle, negative → brake). Same interface as
`carla.VehicleControl`, so the trained policy can drop into this pipeline.
/ 行動：連続2次元（ステア、加減速）。`carla.VehicleControl` と同じ
インターフェースなので学習済み方策をこのパイプラインへそのまま接続できる。

**Episode / エピソード** — random route of 300–1000 m; ends on collision,
off-road > 3 s, route completion, or 90 s timeout.
/ 300〜1000mのランダム経路。衝突・3秒以上のコース外・経路完了・90秒
タイムアウトで終了。

---

## 3. Reward rules / 報酬ルール

One rule set, **shared with the driver alerts** (`ALERT_RULES` in the G923
script) so human feedback and agent training use identical definitions of
"bad driving". / 運転警告（G923スクリプトの `ALERT_RULES`）と**同じ
しきい値**を報酬に使い、「悪い運転」の定義を人間側と学習側で統一する。

| Rule / ルール | Trigger / 条件 | Driver alert / 警告 | RL reward / 報酬 |
|---|---|---|---|
| Collision / 衝突 | collision sensor | danger + beep×2 | −200, end episode / エピソード終了 |
| Red light / 信号無視 | leave red TL moving | danger | −100 |
| Off-road / コース外 | no driving waypoint | — | −50, end if > 3 s |
| Solid-line crossing / 実線逸脱 | lane_invasion (Solid) | danger | −20 |
| Broken-line crossing / 破線またぎ | lane_invasion (Broken) | warn | −5 |
| Forward risk / 前方リスク | TTC < 1.5 s & v > 20 km/h | danger "BRAKE!" | −10 × (1.5 − TTC)/1.5 per step |
| Tailgating / 車間不足 | THW < 1.0 s | (dashboard) | −2 × (1.0 − THW) per step |
| Speeding / 速度超過 | v > limit + 10 for 2 s | warn | −0.5 × over_kmh/10 per step |
| Hard braking / 急ブレーキ | brake>0.7 & decel>4 m/s² | warn | −2 per event (comfort) |
| Discomfort / 乗り心地 | \|jerk\| > 5 m/s³ | — | −0.1 × \|jerk\| per step |
| Lane keeping / 車線維持 | always | — | +0.5 × (1 − \|offset\|/(width/2)) per step |
| Progress / 前進 | always | — | +0.1 × Δroute_distance (m) |
| Completion / 完走 | route end | — | +100 |

Notes / 注: penalties per *step* assume 20 Hz — scale if the tick rate
changes. Tune the safety:progress ratio first; everything else is secondary.
/ ステップ毎ペナルティは20Hz前提。安全系と前進系の比率を最初に調整する。

### 3.1 Reward formula / 報酬の式

Implemented in `rl/rules.py` (`REWARD_RULES` + `step_reward()`); the Gym
env calls it every tick. The total reward at step *t* is
/ `rl/rules.py`（`REWARD_RULES` と `step_reward()`）に実装。Gym環境が
毎Tick呼び出す。ステップ *t* の総報酬：

```
R(t) =  w_prog · Δd(t)                                   progress / 前進
      + w_lane · (1 − |o(t)| / (w_lane_half))            lane keeping / 車線維持
      − w_fcw  · max(0, (TTC* − TTC)/TTC*)  [v > 20]     forward risk / 前方リスク
      − w_thw  · max(0, THW* − THW)                      tailgating / 車間不足
      − w_spd  · max(0, v − v_lim − 10)/10               speeding / 速度超過
      − w_jerk · min(|jerk|, 20)            [|jerk| > 5] comfort / 乗り心地
                  (capped: 1-tick physics artifacts spike to ~100 m/s³
                   / 上限あり：物理アーティファクト対策)
      + Σ E_i(t)                                         events / イベント
```

where / 記号:
- `Δd(t)` = route metres gained this step / このステップで進んだ経路距離 (m)
- `o(t)` = signed lane offset, `w_lane_half` = lane_width/2 / 車線オフセットと半車線幅
- `TTC* = 1.5 s`, `THW* = 1.0 s` — the SAME thresholds as the driver alerts
  (`ALERT_RULES`) / 運転警告と同一しきい値
- `E_i(t)` = one-shot event terms: collision −200 (episode ends), red light
  −100, off-road −50 (ends after 3 s), solid-line −20, broken-line −5,
  hard brake −2, route completion +100
- weights / 重み: `w_prog = 0.1 /m`, `w_lane = 0.5 /step`, `w_fcw = 10`,
  `w_thw = 2`, `w_spd = 0.5`, `w_jerk = 0.1`

**How to weight it / 重みの決め方**

1. **Safety ≫ progress, by construction / 安全 ≫ 前進を構造的に保証する.**
   The best possible per-step income is ~`w_lane + w_prog·Δd ≈ 0.6`/step
   (12/s at 20 Hz). A red-light run (−100) must erase several seconds of
   perfect driving — at the defaults it erases ~8 s. If the trained agent
   still runs lights or crashes for progress, raise the event penalty (or
   lower `w_prog`), not the per-step terms.
   / ステップ毎の最大収入は約0.6/step（20Hzで約12/s）。信号無視（−100）は
   完璧な運転数秒分を帳消しにする大きさが必要（既定では約8秒分）。学習後も
   信号無視や衝突で前進を稼ぐなら、イベント側を強くする（または `w_prog`
   を下げる）。
2. **Fix the "frozen robot" with progress, not safety / 停止する
   エージェントは前進側で直す.** If the agent learns to stop or crawl
   (safest policy!), increase `w_prog` or add a small idle penalty — do not
   weaken collision/TTC terms.
   / 停止・徐行ばかり学ぶ場合は `w_prog` を上げる（安全項は弱めない）。
3. **Keep per-step terms O(1) / ステップ項はO(1)に保つ.** Terminal events
   (±100–200) must dominate any single step; if a per-step term regularly
   exceeds ~2, its weight is too high (this is why jerk uses a threshold
   and the EMA filter in the env).
   / 終端イベント（±100〜200）が常に支配的であるべき。ステップ項が2を
   超え続けるなら重みが大きすぎる。
4. **Rescale with the tick rate / Tickレートに合わせて再スケール.**
   Per-step weights assume 20 Hz; at rate `f`, multiply per-step weights by
   `20/f` so the per-second pressure is unchanged.
   / ステップ毎の重みは20Hz前提。レート`f`なら `20/f` 倍して毎秒の圧力を
   一定に保つ。
5. **Verify empirically / 実測で確認する.** `python -m rl.smoke_test`
   prints the per-term totals of a scripted lane-keeping run — after any
   weight change, check no single term dwarfs the others unintentionally.
   / 重み変更後は `rl/smoke_test.py` の項別集計で、特定の項が意図せず
   支配していないか確認する。

---

## 4. Human-error scenario library / 人間エラーシナリオ集

Mine `data/*/*/summary.json` + `telemetry.csv` for the moments that matter
and turn them into training scenarios (this is what makes the agent robust
to *humans*, not generic traffic):

- **Cut-in / lane drift** — segments around each `lane_invasions` increment:
  spawn a BC/replay vehicle beside the ego that drifts the same way.
- **Sudden braking** — segments around `hard_brake_count` increments: lead
  vehicle replays the human's brake profile.
- **Red-light runner** — cross-traffic vehicle with the measured
  `red_light_violations` behavior at junctions.
- **Distracted following** — sample THW/SDLP from the human distributions
  instead of TrafficManager defaults.

Curriculum: start with 0 % error vehicles, ramp to the measured human error
rates, then overshoot (2×) for stress testing. Randomize weather using the
same presets as the pipeline.

/ `data/` からイベント前後の区間を切り出してシナリオ化する：割り込み・
ふらつき（lane_invasions前後）、急ブレーキ先行車（hard_brake前後の
ブレーキ波形を再生）、信号無視車（計測した無視率で交差交通に配置）、
脇見風追従（THW/SDLP分布からサンプリング）。カリキュラムはエラー車0%から
実測値まで漸増し、最後は2倍で過酷試験。天候もランダム化する。

---

## 5. Training recipe / 学習レシピ

- **Algorithm**: SAC or PPO (stable-baselines3) on the vector observation;
  20 Hz sync mode, action repeat 1, frame stack 4.
  / SACまたはPPO（stable-baselines3）。同期20Hz、フレームスタック4。
- **Warm start**: behavior-clone the agent from *good* human segments
  (speed_compliance > 95 %, no events) before RL — much faster than RL from
  scratch, and keeps early exploration safe.
  / 良好な人間区間（遵守率95%以上・イベントなし）でBC事前学習してからRL。
- **Evaluation**: the dashboard metrics are the benchmark — run the trained
  agent as `hero` through this very pipeline (`--autopilot` replaced by the
  policy) and compare crashes / out-of-lane / TTC / SDLP / compliance
  against human sessions on the Statistics page.
  / 評価はこのダッシュボードの指標そのもの。学習済み方策を `hero` として
  本パイプラインで走らせ、統計ページで人間セッションと比較する。
- **Live HIL test**: human drives `hero`, agent drives `ego` in the same
  town; count agent-side interventions and near-misses (TTC dips) when the
  human misbehaves on purpose.
  / 最終試験は同一マップでの人間×エージェント共走。人間が意図的にエラーを
  起こし、エージェント側のニアミス（TTC低下）を数える。

## 6. How to run, and how to see the agent improve / 実行方法と改善の確認方法

Implemented files / 実装ファイル: `rl/rules.py` (shared alert+reward rules),
`rl/carla_env.py` (Gym env), `rl/extract_scenarios.py`, `rl/train_sac.py`,
`rl/smoke_test.py`, `agent_drive.py` (ego client), pipeline `--agent` flag.

**Step 0 — one-time / 初回のみ**
```bash
cd PythonAPI/hil_sim
../../carla-venv/bin/pip install stable-baselines3   # gymnasium/networkx は導入済み
```

**Step 1 — collect human sessions / 人間セッションの収集** (already
routine / 従来どおり):
```bash
python run_pipeline.py --map Town02        # drive with the G923 / G923で運転
```
Each session lands in `data/<day>/<time>/telemetry.csv` + `summary.json`.

**Step 2 — mine human errors / 人間エラーの抽出**:
```bash
python -m rl.extract_scenarios
# -> data/scenarios/clips/*.json          (event clips / イベント切り出し)
# -> data/scenarios/driver_profiles.json  (TrafficManager error params)
```

**Step 3 — verify the env, then train / 環境確認と学習** (CARLA must be
running; headless is fastest / CARLAを起動しておく。ヘッドレスが最速):
```bash
../../CarlaUE4.sh -RenderOffScreen -quality-level=Low &   # headless server
python -m rl.smoke_test                    # env sanity check / 環境の動作確認
python -m rl.train_sac --timesteps 200000  # checkpoints every 20k steps
```

**Step 3b — the study arms / 研究アーム** (docs/RESEARCH_PROTOCOL.md §3;
CARLA running, headless is fastest / CARLA起動済み・ヘッドレス推奨):
```bash
python -m rl.run_arm --arm A --seed 0 --timesteps 60000   # default traffic
python -m rl.run_arm --arm B --seed 0 --timesteps 60000   # human-error traffic
python -m rl.run_arm --arm B --seed 0 --eval-only         # re-evaluate ckpts
python -m rl.plot_progress                                # result graphs
python -m rl.plot_improvement                             # how the agent improves
python -m rl.plot_town_events                             # Town02 event map
```
`run_arm` loads the map, spawns NPC traffic (`--vehicles`, default 20) in
passive mode under its own TrafficManager, trains SAC with checkpoints,
then evaluates every checkpoint in the SAME `--eval-traffic` (default
`human_x1` = measured human error rates) and appends rows to
`rl/results/eval_history.csv`. Repeat with `--seed 1 2 ...`; the plots
average seeds. Arm B's traffic comes from `data/scenarios/map_profiles.json`
(km-weighted per map, written by `rl.extract_scenarios`).
/ マップ読込 → 受動モードで NPC 交通生成 → チェックポイント付き SAC
学習 → 全チェックポイントを同一の評価交通で評価し eval_history.csv に
追記。シードを変えて繰り返すとプロットがシード平均する。アームBの交通は
`map_profiles.json`（マップ毎の距離加重プロファイル）から作る。

Outputs / 出力 — all under `rl/results/` (`rl/paths.py`; `models/`, `tb/`,
`eval_history.csv`, `figures/`, `logs/`): `figures/progress.png` (learning
curves: training reward and length, then Driving Score / Route Completion /
collision rate per checkpoint, mean ± std over seeds), `figures/results.png`
(final checkpoints as grouped bars with error bars, human sessions as a
reference line), `figures/results_table.md` (mean ± std table),
`figures/improvement_*.png` + `figures/improvement_table.md`
(`rl.plot_improvement`: the full metric battery vs. training steps with the
human drivers as a dashed reference, first-vs-final checkpoint bars with %
change, agent/human ratio per km, a speed-vs-safety trajectory that tells
"safer" from "just slower", and how episodes end per checkpoint),
`figures/Town02_events.png` + `figures/Town02_metrics.png` (where and how
often humans — and agent sessions
listed in `data/agent_sessions.json` — err on the map). Metrics follow the
CARLA Leaderboard: RC = route fraction completed, IS = product of
infraction penalties (collision 0.60, red light 0.70), DS = RC × IS;
infractions are also given per km. Figure text is English only. The town
and the evaluation traffic default to `STUDY_MAP` / `EVAL_TRAFFIC` in
`rl/config.py` — change the map there, not in each script.
/ 指標は CARLA Leaderboard の慣例（RC・IS・DS = RC×IS、km あたり違反数）。
図中の文字は英語のみ。

Machine safety / 機器の保護: a training run hard-locked this workstation
on 2026-09-14 (`docs/TROUBLESHOOTING.md`, "The whole computer freezes").
`run_arm` therefore checks the kernel log before starting (refuses on
`NRVM: Xid` / lockups in this boot, `--allow-faults` overrides), writes
`rl/results/logs/<run>_<time>.log` + `.health.json` from the first second,
runs the world in `no_rendering_mode` (the env has no camera, `--render`
disables), caps torch at `--torch-threads 4` and nices itself, and stores
the replay buffer with each checkpoint so `--resume` continues a cut-off
run. Keep the first run after a freeze short and watch `journalctl -kf`.
/ フリーズ事故を受け、`run_arm` は開始前にカーネルログを確認（障害があれば
拒否）、開始直後にログと機器スナップショットを書き、描画を止め、torch の
スレッドを絞り、リプレイバッファ付きチェックポイントから `--resume` できる。
フリーズ後の最初の run は短くし、`journalctl -kf` を見ながら走らせる。

**Step 4 — see it improve / 改善を確認する** — three lenses, weakest to
strongest / 弱い順に3つの観点:
1. **Training curve / 学習曲線**: `tensorboard --logdir rl/results/tb` → open
   http://localhost:6006. `rollout/ep_rew_mean` should climb and
   `ep_len_mean` grow (episodes end early on crashes, so longer = safer).
   / `ep_rew_mean` の上昇と `ep_len_mean` の伸び（衝突で早期終了する
   ため、長い＝安全）を見る。
2. **Rule score vs. baseline / ルールスコアの基線比較**: run the same
   world twice and compare the cumulative rule score `agent_drive.py`
   prints every 10 s / 同一条件で2回走らせ、10秒毎に表示される累積
   ルールスコアを比較:
   ```bash
   python agent_drive.py --policy autopilot                  # baseline / 基線
   python agent_drive.py --policy model:rl/results/models/sac_ego.zip
   ```
3. **Dashboard benchmark / ダッシュボード基準** (the honest test — same
   metrics as the human sessions / 人間セッションと同一指標での本試験):
   run the agent as `hero` with the dashboard logging, then compare
   crashes / out-of-lane / SDLP / compliance on the Statistics page
   against human sessions and against older checkpoints:
   ```bash
   python run_pipeline.py --no-sim --no-traffic &        # dashboard etc.
   python agent_drive.py --rolename hero --policy model:rl/results/models/sac_ego.zip
   ```
   Improvement = newer checkpoints produce sessions with fewer crashes,
   fewer out-of-lane events, lower SDLP at similar avg speed.
   / 改善＝新しいチェックポイントほど、平均速度が同等のまま衝突・
   車線逸脱・SDLPが減ること。

**Step 5 — live HIL / 人間との共走評価**: human drives hero, agent drives
ego in the same world / 同一ワールドで人間がhero、エージェントがegoを運転:
```bash
python run_pipeline.py --agent model:rl/results/models/sac_ego.zip
```

---

## 7. Suggested next steps / 次のステップ

1. ~~Add `role_name=ego` support + a `--policy` flag to the pipeline (agent
   client skeleton).~~ **Done 2026-07-07** — `agent_drive.py`, `--agent`.
2. ~~Script to slice `telemetry.csv` around events into scenario clips.~~
   **Done 2026-07-07** — `rl/extract_scenarios.py`.
3. ~~Gym wrapper implementing §2–§3.~~ **Done 2026-07-07** —
   `rl/carla_env.py` (+ `rl/rules.py` shared with the driver alerts).
4. BC baseline from good segments, then SAC fine-tuning with the scenario
   library. / 良区間BC→シナリオ集でSAC微調整。
5. ~~Traffic spawner that applies `driver_profiles.json` TrafficManager
   params so training traffic misbehaves like the measured humans.~~
   **Done 2026-09-14** — `generate_traffic.py --driver-profile
   data/scenarios/map_profiles.json --error-scale {0,1,2}` (km-weighted
   per-map pool from `rl/extract_scenarios.py`), driven by `rl/run_arm.py`.
   / `map_profiles.json` を適用する交通生成、`rl/run_arm.py` から使用。
6. Timing randomization + action latency in `rl/carla_env.py`
   (RESEARCH_PROTOCOL §7) and the confrontation runner (§8 item 3) are
   still open. / Tick時間ランダム化・行動遅延と共走評価ランナーは未着手。
