# Research Protocol: Error-Robust RL via HIL Human-Error Traffic
# 研究プロトコル：HIL人間エラー交通による頑健RL

Decisions fixed in the grilling session of 2026-07-10. Companion to
`RL_DESIGN.md` (which holds the environment/reward specification); this
document holds the *experimental design*. Change either only by explicit
revision, not silently.
/ 2026-07-10 のグリリングセッションで確定した決定事項。環境・報酬仕様は
`RL_DESIGN.md`、実験設計は本書が正。変更は明示的な改訂によること。

---

## 1. Primary claim / 主張

An RL agent trained against traffic that reproduces **measured human
errors** (mined from this rig's telemetry) is **safer around real humans**
than the same agent trained on default CARLA traffic.
/ 実測した人間エラーを再現する交通で訓練したRLエージェントは、既定の
CARLA交通で訓練した同一エージェントより**本物の人間の近くで安全**である。

## 2. Treatment: human-error traffic / 処置条件の実装

- **Primary**: TrafficManager parameter injection from
  `data/scenarios/driver_profiles.json` (SDLP, THW distribution,
  `ignore_lights_percentage`, speed difference) — produced by
  `rl/extract_scenarios.py`.
- **Secondary**: scripted vehicles replaying mined event clips (cut-in,
  hard brake, red-light run) spawned near the ego.
- Curriculum per `RL_DESIGN.md` §4: 0 % → measured rates → 2× overshoot.
- Behavior-cloned NPC drivers: **out of scope for the pilot**.

## 3. Study arms / 実験アーム

Matched in every respect except traffic: same SAC hyperparameters, same
reward (`rl/rules.py`), same timestep budget, same maps, same seeds count.
/ 交通以外は完全一致（SAC設定・報酬・ステップ数・マップ・シード数）。

| Arm | Training traffic / 訓練交通 | Learned? |
|---|---|---|
| A | Default TrafficManager | yes (SAC) |
| B | Human-error traffic (§2, with curriculum) | yes (SAC) |
| C | CARLA autopilot reference / 参照基線 | no |

3–5 seeds per learned arm. Single-seed results are not reportable.
/ 学習アームは3〜5シード。単一シードの結果は報告不可。

## 4. Primary evaluation / 主要評価

**Live HIL confrontation.** Human drives `hero` and performs *scripted*
errors from the §4 taxonomy of `RL_DESIGN.md` (cut-in, hard brake ahead,
red-light run, distracted following) while the arm's agent drives `ego`
nearby (async mode, `run_pipeline.py --agent`).
/ **人間との共走対決評価**。人間が hero を運転し、台本化したエラーを
実行。エージェントは ego を運転（非同期・`--agent`）。

- **Primary endpoint / 主要評価項目**: collisions + near-misses
  (TTC < 1.5 s dips, thresholds identical to `ALERT_RULES`) per
  confrontation.
- Protocol controls: every eval driver faces **all** arms; arm order
  randomized; the driver is **blinded** to which agent is running.
- Secondary (reported, not decisive): held-out error-scenario evaluation
  (split by driver) and the dashboard metric battery (Statistics page).

## 5. Subjects: pilot → full study / 被験者：パイロット→本試験

1. **Pilot**: 3–5 lab drivers, train/eval overlap allowed,
   ~5 confrontations × arm × driver. Purpose: debug protocol, estimate
   variance, exercise the full pipeline.
2. **Full study**: pre-registered; 8–12 drivers with a **disjoint split**
   (~6–8 collection-only → profiles/clips; 3–4 evaluation-only),
   ~10 confrontations × arm × eval driver (≥ 90 trials).
3. Check university human-subjects/IRB requirements **before the pilot**.
   / パイロット前に倫理審査（IRB）要否を確認すること。

## 6. Success criteria & statistics / 成功基準と統計

- **Pilot gates** (feasibility only): protocol completes for every
  driver×arm; arm B trends in the expected direction; variance estimate
  yields a power analysis that sets the full-study N.
- **Full study**: pre-registered one-sided hypothesis — arm B < arm A on
  the primary endpoint — tested with a **mixed-effects model** (driver =
  random effect, arm = fixed effect). Report effect size + CI, not only p.

## 7. Known-risk mitigations / 既知リスクへの対策

- **Sync→async transfer gap**: training stays synchronous but randomizes
  `fixed_delta_seconds` (0.04–0.07 s per episode) and injects 1–2 ticks of
  action latency; deployment (`agent_drive.py`) runs a 20 Hz control loop
  and its closed-loop latency is measured and reported.
  / 同期訓練にTick時間ランダム化＋行動遅延を入れ、非同期展開時の
  実測遅延を報告する。
- **BC warm-start**: excluded from the pilot. If pilot SAC cannot reach
  basic route-following within budget, add BC **to both learned arms
  equally** in the full study (never to one arm only).
- **Same-driver contamination**: full study uses disjoint
  collection/evaluation drivers (§5) to support the generalization claim.

## 8. Immediate build list (pending confirmation) / 実装項目（承認待ち）

1. Traffic spawner applying `driver_profiles.json` + event-clip vehicles
   (RL_DESIGN §7 item 5) — the arm-B environment.
   *Primary part done 2026-09-14*: `generate_traffic.py --driver-profile`
   (km-weighted per-map profile, `--error-scale` curriculum knob); the
   event-clip vehicles (§2 secondary) are still open.
2. Timing randomization in `rl/carla_env.py` (§7 above).
3. Confrontation runner: scripted-error prompts for the human +
   per-confrontation endpoint logging (reuses `ALERT_RULES`/dashboard).
4. Arm-matched training launcher (seeds × arms) + evaluation notebook
   (mixed-effects for the full study, gates for the pilot).
   *Launcher + secondary-endpoint plots done 2026-09-14*: `rl/run_arm.py`,
   `rl/plot_progress.py` (Leaderboard-style DS/RC/IS, per-km infractions,
   mean ± std over seeds), `rl/plot_town_events.py`. The mixed-effects
   analysis of the primary endpoint waits on item 3.
