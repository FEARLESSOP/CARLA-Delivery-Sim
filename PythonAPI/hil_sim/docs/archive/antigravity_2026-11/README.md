# Archive: Google Antigravity's write-ups, 2026-11-09 .. 11-13 / Antigravity の記録（アーカイブ）

Kept for the record, **not current guidance**. They were written by Google
Antigravity while it ran the RL work (2026-11-09 .. 11-13) and moved here on
2026-11-13 when the docs were organised. Several claims in them do not match
the training logs or the code; check before relying on any of them.

記録として保管。**現在の手順書ではない。** Antigravity が RL 作業中
（2026-11-09〜11-13）に書いたもので、2026-11-13 の整理でここへ移した。
学習ログやコードと合わない記述があるので、使う前に必ず確認すること。

| File / ファイル | What it is / 内容 |
|---|---|
| `CHAT_COMPLETE_ARCHIVE.md` | Antigravity's own ledger of its chat (was in the CARLA root) / チャット全記録（CARLA 直下にあった） |
| `OITA_LAST_MILE_MASTER_PLAN.md` | Research plan: two papers, phases / 研究計画（論文 2 本・段階） |
| `RL_DELIVERY_SPECIFICATION.md` | Theory/architecture spec of the RL delivery system / RL 配送の理論・構成仕様 |
| `OVERNIGHT_LOG.md` | 30-minute training checks, Nov 11-12 (was in `rl_delivery/results/`) / 夜間学習チェック記録 |

## Known mismatches (checked 2026-11-13) / 確認済みの食い違い

- **Drone navigation "verified":** `rl_delivery/results/nav_progress.csv` shows
  `reached = 0`, `crash = 1.0` from about 30 M steps on, so checkpoint
  `nav/step_099754348` is broken. / ナビ脳は約 3000 万ステップ以降、到達 0・墜落 100%。
- **Paper 1 benchmark** (`rl_delivery/ramppo.py`, `evaluate_scenario_benchmark`):
  the "PPO" baselines B1-B3 load no network (a fixed hub preference), scenarios
  S2-S5 differ only in wind and give identical numbers, and B4 does not beat B5
  or the heuristic. / ベースラインに学習済みネットワークが無く、S2〜S5 は同一結果。
- **Fleet brains** make 14-17 deliveries/h against 36.5 for the rule baseline.
  / フリート脳は配達数がルール基準の半分以下。
- `delivery/feasibility_engine.py` hard-codes a battery reserve, no-fly zones
  and masks, against the rule "nothing hardcoded, pure RL" (`docs/RL_DELIVERY.md`).
  / 電池予備・飛行禁止・マスクを固定値で持ち、「すべて学習」の規則に反する。

Current RL guidance: [`../../RL_DELIVERY.md`](../../RL_DELIVERY.md). Index of all
docs: [`../../README.md`](../../README.md).
現行の RL 手順書は `docs/RL_DELIVERY.md`、全ドキュメントの索引は `docs/README.md`。
