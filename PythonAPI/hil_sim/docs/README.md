# Documentation index / ドキュメント索引

Every document of the rig, by topic, with what it is for. Links are relative to
this file (`hil_sim/docs/`; `../../../` is the CARLA folder). Files stay where they are because code, guides and agent
instructions link to them; this page is the one place that lists them all.
Organised 2026-11-13.

リグの全ドキュメントを話題別に一覧にする。リンクはこのファイルからの相対
（`../../../` が CARLA フォルダ）。コードや手順書から参照されているため場所は動かさず、
この索引で全体をまとめる（2026-11-13 整理）。

## Start here / はじめに

| Doc | What it is / 内容 |
|---|---|
| [`../README.md`](../README.md) | The rig: run it, dashboard, driving controls, maps, cars, buses, weather (EN + JA) / リグの使い方 |
| [`../FILES.md`](../FILES.md) | Where is what: important files by topic / ファイルの場所 |
| [`../../../CARLA_RIG_SETUP_FROM_SCRATCH.md`](../../../CARLA_RIG_SETUP_FROM_SCRATCH.md) | Every change on top of stock CARLA, rebuild from scratch, open items (sec. 9), timeline (sec. 10) / 全変更・再構築・未解決・年表 |
| [`../../../NEW_PC_SETUP.md`](../../../NEW_PC_SETUP.md) | One-script setup of a new PC (`setup_new_pc.sh`) / 新 PC のセットアップ |
| [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) | Symptoms, causes, fixes (GPU Xid, PRIME, frozen world, slow loops) / トラブル対処 |

## Delivery simulation / 配送シミュレーション

| Doc | What it is / 内容 |
|---|---|
| [`DELIVERY.md`](DELIVERY.md) | `delivery_sim.py` + `delivery/`: pods, drones, hubs, Mini Pods, charging, camera / 配送ミッション |
| [`DELIVERY_KPI.md`](DELIVERY_KPI.md) | The pod and drone KPIs and how they are computed / 配送 KPI |

## RL for the pods and drones / ポッド・ドローンの強化学習

| Doc | What it is / 内容 |
|---|---|
| [`RL_DELIVERY.md`](RL_DELIVERY.md) | **The handbook**: brains, pretend world, user rules (nothing hardcoded, pure RL), training, checks / 手順書（最初に読む） |
| [`RL_CARLA_BRIDGE.md`](RL_CARLA_BRIDGE.md) | Running the trained brains in CARLA (`--policy-nav`, `--policy-drive`, `--policy-recovery`) and their status / CARLA への接続と現状 |
| [`../rl_delivery/research/README.md`](../rl_delivery/research/README.md) | The research record: runs, changes, baselines (CSV) for a paper / 研究記録 |
| [`../rl_delivery/HANDOFF.md`](../rl_delivery/HANDOFF.md) | Training log and requests between agents (status note on top) / 学習の引き継ぎ記録 |

## RL for the driving study / 運転研究の強化学習（`rl/`）

| Doc | What it is / 内容 |
|---|---|
| [`RL_DESIGN.md`](RL_DESIGN.md) | Agent robust to human driver errors: spaces, reward, scenarios, recipe / 設計 |
| [`RESEARCH_PROTOCOL.md`](RESEARCH_PROTOCOL.md) | Fixed experimental design (arms A/B, metrics); change only by explicit revision / 実験計画 |
| [`../rl/results/README.md`](../rl/results/README.md) | Where the training output goes / 出力の場所 |

## Maps, cars, buses / マップ・車・バス

| Doc | What it is / 内容 |
|---|---|
| `../../../map_guide/README.md` | Map guide (chapters 01-39), part of the map skill / マップガイド |
| [`../maps/queue/`](../maps/queue/) | Queued map jobs with status: 01 Pasir Ris (deferred) + brief, 02 and 03 OitaKosen (done) / マップ作業の予定 |
| `../cars/sources/<car>/survey*.md` | Per-car survey notes from the car importer / 車ごとの調査メモ |
| [`../data/oita_bus/sources.md`](../data/oita_bus/sources.md) | Sources of the Oita bus timetables (GTFS) / バス時刻表の出典 |

## Dashboard (web) / ダッシュボード

| Doc | What it is / 内容 |
|---|---|
| [`../web/DEV.md`](../web/DEV.md) | Rules for the React app and how to rebuild `web/dist` / 開発ルール |

## Plans (not built) / 計画（未実装）

| Doc | What it is / 内容 |
|---|---|
| [`NETWORK_SIM_PLAN.md`](NETWORK_SIM_PLAN.md) | ns-3 + CARLA wireless co-simulation (VANET, 5G, 6G); plan only / 無線ネットワーク連成の計画 |

## Archive / アーカイブ

| Doc | What it is / 内容 |
|---|---|
| [`archive/antigravity_2026-11/`](archive/antigravity_2026-11/README.md) | Google Antigravity's chat ledger, master plan, spec and overnight log (2026-11-09..13); not current, with a list of claims that do not match the logs / 旧記録（現行ではない） |

## For AI agents (not organised here) / AI エージェント向け（ここでは整理しない）

`CLAUDE.md`, `AGENTS.md`, `../../../AGENTS.md`, `../../../.agents/skills/`,
`~/.claude/skills/carla-map-generator/`, `~/.claude/skills/carla-car-importer/`.
