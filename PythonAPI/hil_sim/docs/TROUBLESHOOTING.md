# Troubleshooting / トラブルシューティング

Symptoms that look like bugs in this repo but are not, and the ones that are.
本リポジトリのバグに見えて実際は違う症状と、実際にバグである症状。

---

## "CPU goes over 100% and then the simulation crashes"
## 「CPU が 100% を超えてシミュレーションがクラッシュする」

**The CPU number is not the problem.** `CarlaUE4-Linux-Shipping` is a
multithreaded UE4 server with ~111 threads. In `top`, `%CPU` is *per core*,
so anything above 100% simply means "more than one core". Measured on this
rig during a healthy 60-vehicle session:

**CPU の数値は問題ではない。** `CarlaUE4-Linux-Shipping` は約111スレッドを
持つマルチスレッドの UE4 サーバである。`top` の `%CPU` は *1コアあたり* の
値なので、100% 超は単に「1コア以上を使っている」という意味でしかない。
本機で60台の正常なセッション中に実測した値：

| Metric / 指標 | Healthy value / 正常値 |
|---|---|
| `CarlaUE4` CPU | 185–400% (≈2–4 of 20 cores / 20コア中約2〜4) |
| System load average / システム負荷 | 2.6 on a 20-core box / 20コアで2.6 |
| GPU utilisation / GPU使用率 | 20–85% |
| GPU power / GPU電力 | 57–182 W |
| Heartbeat age / ハートビート経過 | 0–1 s |

A 20-core machine is ~20% busy at 400%. That is normal and healthy.
20コア機で400%はおよそ20%の使用率であり、正常である。

The crash has a separate cause — see the next section.
クラッシュの原因は別にある。次節を参照。

---

## The real crash: GPU device loss (NVRM Xid)
## 本当のクラッシュ原因：GPU デバイスロス（NVRM Xid）

### Signature / 兆候

In the CarlaUE4 output / CarlaUE4 の出力：

```
LowLevelFatalError [File:Unknown] [Line: 803]
Result failed, VkResult=-4 ... with error VK_ERROR_DEVICE_LOST
Signal 11 caught.
Segmentation fault (core dumped)
```

or, when the GPU dies without the engine noticing in time /
またはエンジンが気付く前に GPU が死んだ場合：

```
LowLevelFatalError ... GameThread timed out waiting for RenderThread after 60.00 secs
```

In the kernel log / カーネルログ：

```bash
journalctl -k --since today | grep "NVRM: Xid"
```

```
NVRM: Xid (PCI:0000:01:00): 13, Graphics Exception: MISSING_MACRO_DATA
NVRM: Xid (PCI:0000:01:00): 32, pid=..., name=CarlaUE4-Linux-, channel 0x00000003
```

**Always check the kernel log before debugging the Python side.** If an Xid
line blames `CarlaUE4-Linux-`, nothing in this repo caused the crash.
**Python 側を調べる前に必ずカーネルログを確認すること。** Xid 行が
`CarlaUE4-Linux-` を指しているなら、原因は本リポジトリにはない。

### Environment where this was observed / 観測環境

- RTX 5080 (Blackwell), `nvidia-driver-595-open` 595.84 **and**
  `nvidia-driver-610-open` 610.43.02 (see the driver log below)
- Ubuntu 24.04, CARLA 0.9.16 (UE 4.26, Vulkan RHI)
- 7 Xid faults in a single afternoon (2026-09-14) / 1日の午後に7回

### Trigger: burst asset load/unload / 誘因：アセットの一括ロード・解放

The fault fired reliably when `generate_traffic.py` spawned all 60 vehicles
in **one** `apply_batch_sync` — the server uploads every mesh, material and
shader in a single burst. Measured 2026-09-14:

`generate_traffic.py` が60台を **1回の** `apply_batch_sync` で生成したとき
（サーバがメッシュ・マテリアル・シェーダを一気に転送する）に確実に発生した。
2026-09-14 の実測：

| Spawn/teardown style / 生成・破棄方式 | Result / 結果 |
|---|---|
| One batch of 60 / 60台一括 | **3 crashes in 3 runs** / 3回中3回クラッシュ |
| Chunks of 5, 0.5 s apart / 5台ずつ0.5秒間隔 | **0 crashes in 3 runs** / 3回中0回 |
| One batch teardown on SIGTERM / SIGTERM時に一括破棄 | **Killed the server every time** / 毎回サーバ死亡 |
| Chunked teardown / チャンク化した破棄 | **Clean shutdown** / 正常終了 |

### Mitigation now in the repo / 現在の回避策

`generate_traffic.py` spawns and destroys actors in chunks by default:
`generate_traffic.py` は既定でアクターをチャンク単位で生成・破棄する：

```bash
--spawn-chunk 5      # actors per batch (0 = old one-shot behaviour)
--spawn-pause 0.5    # seconds between chunks
```

This is a **workaround for a driver bug, not a fix.** It spreads the GPU
work out so the fault is much less likely to fire; it cannot make a driver
that faults under burst load reliable.
これは **ドライバのバグに対する回避策であり、修正ではない。** GPU の負荷を
分散して発生確率を下げるだけで、バースト負荷で落ちるドライバを
信頼できるものに変えることはできない。

### The actual fix / 本来の対処

The lever is the **driver**, not CARLA settings and not this repo.
効くのは **ドライバ** であり、CARLA の設定でも本リポジトリでもない。

```bash
# currently installed / 現在の導入版
dpkg -l | grep nvidia-driver        # nvidia-driver-610-open 610.43.02
grep nvidia-driver /var/log/dpkg.log  # what was tried, when / 試行の履歴
```

What has been tried (2026-09-14, from `/var/log/dpkg.log` and
`journalctl -k`) / 試したこと：

| Driver / ドライバ | Result / 結果 |
|---|---|
| `595-open` 595.84 | Xid 13/32 faults, the original report / 当初の障害 |
| `610-open` 610.43.02 (10:10) | still faults / 依然として発生 |
| `610` proprietary (10:54) | **RTX 5080 disappears** — no DRM node, no outputs; Blackwell needs the open module / GPU が消える。Blackwell は open モジュール必須 |
| `610-open` again (11:21, current) | Xid 69 + 32 at 11:57 during a 20-vehicle session, spawn chunking on / 依然として発生 |

So a newer branch did not fix it and the proprietary module is not an
option on this GPU. Remaining levers, in order / 残る手段：

1. **Keep the burst-load workaround** (`--spawn-chunk`, chunked teardown)
   and keep sessions small; it lowers the rate, not to zero.
   / 回避策は維持。発生率は下がるが0にはならない。
2. **Rule out the CPU.** The same box throws random compiler SIGSEGVs
   under `make -j16`, which points at i5-14600K (Raptor Lake) degradation.
   A signal-11 *without* an Xid line is that, not the GPU. Run `stress-ng`
   / a kernel build to check, and apply the Intel microcode / BIOS
   voltage fix if it is. / Xid 行の無い signal-11 は CPU 側。マイクロコード
   と BIOS の電圧設定を確認。
3. **Watch for a driver release newer than 610.43** and retest the
   one-batch spawn (`--spawn-chunk 0`), which reproduces in minutes.
   / 610.43 より新しいドライバで一括生成を再試験。

Changing the driver is a system-level change and needs a reboot; it is the
operator's call, not something any script here should do.
ドライバ変更はシステム全体に影響し再起動を伴う。スクリプトが勝手に行うべき
ものではなく、運用者が判断すること。

Do **not** lower render quality to dodge this: `docs/RESEARCH_PROTOCOL.md`
fixes the experimental conditions, so that is an explicit-revision decision.
（The faults reproduced at `-quality-level=Low` anyway.）
これを避けるために描画品質を下げては **いけない**。`docs/RESEARCH_PROTOCOL.md`
が実験条件を固定しているため、明示的な改訂が必要な判断である。
（そもそも `-quality-level=Low` でも発生した。）

### After repeated faults / 障害が繰り返された後

The GPU can end up in a degraded state where CarlaUE4 segfaults *at startup*
with no Xid line at all. Reboot before concluding anything else is wrong.
GPU が劣化状態に陥り、Xid 行すら出ずに CarlaUE4 が *起動時* に segfault する
ことがある。他の原因を疑う前に再起動すること。

---

## The whole computer freezes / PC 全体が固まる

Different from the Xid crash above: there the *server* dies and the desktop
survives. Here nothing responds — no mouse, no SSH, no magic SysRq — and
the only way out is the power button. Observed 2026-09-14 13:34, five
minutes into `python -m rl.run_arm --arm A --seed 0` on CarlaUE4
`-RenderOffScreen`, 20 NPC vehicles, `nvidia-driver-610-open` 610.43.02,
kernel 7.0.0-31.
上記の Xid クラッシュとは別物。あちらは *サーバ* が死ぬだけでデスクトップは
生きているが、こちらはマウスも SSH も SysRq も効かず電源ボタンしかない。
2026-09-14 13:34、`rl.run_arm` 開始 5 分後に発生。

### Signature / 兆候

`journalctl -k -b -1` (the boot *before* the power cycle) shows, in this
order / 電源断 *前* のブートのカーネルログに次の順で出る：

```
watchdog: CPU0: Watchdog detected hard LOCKUP on cpu 0
RIP: native_queued_spin_lock_slowpath+0x2d7/0x320
Call Trace: ... down+0x1e/0x80
            nvkms_kthread_q_callback+0xb4/0x1b0 [nvidia_modeset]
            _main_loop+0x9c/0x170 [nvidia_modeset]
watchdog: BUG: soft lockup - CPU#13 stuck for 22s! [code:27500]
watchdog: BUG: soft lockup - CPU#17 stuck for 26s! [python:248883]
   ... (every other CPU, stuck in smp_call_function_many_cond
        <- native_flush_tlb_multi <- munmap)
```

Read it as: the NVIDIA **modeset** kernel thread (display/KMS side, not
the compute side) dead-locked on a spinlock with interrupts off on CPU 0.
Every other core then waited forever for CPU 0 to acknowledge a TLB-flush
IPI, so *all* processes froze — the python training process, VS Code,
this CLI. The user-space processes named in the soft-lockup lines are
victims, not causes.
読み方：NVIDIA の **modeset** カーネルスレッド（表示側）が CPU 0 上で
割り込み禁止のままスピンロックでデッドロックし、他の全コアが CPU 0 の
TLB フラッシュ応答を待ち続けて全プロセスが停止した。soft lockup 行に
出るプロセス名は被害者であって原因ではない。

Quick check after any freeze / フリーズ後の確認：

```bash
journalctl -k -b -1 --no-pager | grep -E "hard LOCKUP|soft lockup|NVRM: Xid|Oops" | head
python -m rl.health          # same check + GPU/memory state as JSON
```

### What the RL scripts do about it / RL スクリプト側の対策

A user process cannot stop a kernel driver from locking up. What
`rl/run_arm.py` does instead (all on by default) / ユーザプロセスには
ドライバのロックアップを防げない。代わりに `rl/run_arm.py` が行うこと：

| Measure / 対策 | Why / 理由 |
|---|---|
| Preflight (`rl/health.py`): refuse to start if this boot already logged `NVRM: Xid` / lockups; `--allow-faults` overrides | a GPU that has faulted once in a boot keeps faulting until reboot / 一度障害を出した GPU は再起動まで再発する |
| `rl/results/logs/<run>_<time>.log` (line-flushed) + `.health.json` at start and end | after a power cycle you know what was running and how far it got / 電源断後に何がどこまで動いていたか分かる |
| `world.no_rendering_mode = True` while the script owns the world (`--render` to disable) | the env has no camera; physics, TrafficManager, collision / lane sensors are unaffected, and the GPU is almost idle — fewer chances for the driver to misbehave / GPU をほぼ遊ばせる |
| `--torch-threads 4` + `OMP_NUM_THREADS`, `os.nice(5)` | the i5-14600K is suspected of instability under parallel load (below); a tiny MLP gains nothing from 20 threads / CPU の並列高負荷を避ける |
| replay buffer saved with every checkpoint (newest kept) → `--resume` | a frozen run resumes from the last checkpoint instead of restarting / 途中から再開できる |

Operational rules / 運用ルール:

1. **First run after a freeze is short** (`--timesteps 5000`) with
   `journalctl -kf` open in another terminal.
   / フリーズ後の最初の run は短く、別端末で `journalctl -kf` を開く。
2. **Nothing else heavy on the box** while training — no VS Code
   indexing, no browser video, no builds. The lockup was in the display
   driver; the fewer GPU clients, the better.
   / 学習中は他の重い処理（VS Code の索引、動画、ビルド）を走らせない。
3. Run headless: `CarlaUE4.sh -RenderOffScreen -nosound`, and do not open
   `sensor_viewer.py` / the dashboard against a training server.
   / ヘッドレスで動かし、学習中のサーバに viewer や dashboard を付けない。

### Levers outside this repo / リポジトリ外の手段

Same as for the Xid crash: the driver (`610-open` is the only branch that
sees the RTX 5080 at all) and the CPU (Raptor Lake, see below). Two
system-level options the operator may choose / 運用者が選べる選択肢：

* `kernel.hardlockup_panic=1` + `kernel.panic=30` (sysctl) turns a hang
  into an automatic reboot after 30 s instead of an all-night freeze.
  / ハングを 30 秒後の自動再起動に変える。
* Move the third monitor off the RTX 5080 so `nvidia_modeset` has no
  display to drive during training; the card then only does compute.
  / 3 枚目のモニタを RTX 5080 から外し、学習中は modeset に表示を持たせない。

Both are reboots and BIOS/cable work, not something a script here should do.
いずれも再起動や配線を伴う作業で、スクリプトが勝手に行うものではない。

---

## Which GPU is actually rendering? / 実際にどの GPU が描画しているか

This workstation is hybrid-graphics (Intel UHD 770 + RTX 5080,
`prime-select query` = `on-demand`), so GL/Vulkan apps land on the **iGPU**
by default and the run crawls. `run_pipeline.py` handles this itself via
`--gpu {auto,nvidia,off}`.

本機はハイブリッドグラフィックス構成（Intel UHD 770 + RTX 5080、
`prime-select query` は `on-demand`）のため、GL/Vulkan アプリは既定で
**iGPU** 上で動き、動作が極端に遅くなる。`run_pipeline.py` は
`--gpu {auto,nvidia,off}` で自動対応する。

Running any component **outside** the pipeline needs the variables by hand:
パイプライン **外** で各コンポーネントを動かす場合は手動で設定する：

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
__VK_LAYER_NV_optimus=NVIDIA_only \
./CarlaUE4.sh -quality-level=Low
```

Confirm with `nvidia-smi` that `CarlaUE4-Linux-Shipping` is listed and
drawing real power (~5.5 GB VRAM, 57–182 W). If it does not appear at all,
it is on the iGPU.
`nvidia-smi` に `CarlaUE4-Linux-Shipping` が現れ、実際に電力を消費して
いること（約5.5GB VRAM、57〜182W）を確認する。一覧に出ないなら iGPU 上で
動いている。

---

## `telemetry.csv` is 0 bytes / `telemetry.csv` が0バイト

Fixed. `dashboard.py` buffered the CSV header and flushed only every 2 s with
the data rows, so a session that died before its first flush — exactly what a
CarlaUE4 crash plus the pipeline's SIGKILL sweep produces — left a 0-byte file
with no header, and the run could not even be identified afterwards. The
header is now flushed at open.

修正済み。`dashboard.py` は CSV ヘッダをバッファに溜め、データ行と一緒に
2秒ごとにしかフラッシュしていなかった。そのため最初のフラッシュ前に
セッションが死ぬと（CarlaUE4 のクラッシュとパイプラインの SIGKILL 掃除が
まさにこれ）、ヘッダすら無い0バイトのファイルが残り、後から実行を識別する
ことすらできなかった。現在はオープン時にヘッダをフラッシュする。

If you still see 0-byte files, the dashboard never reached `_open_log` —
check that it started at all.
それでも0バイトのファイルが残る場合、ダッシュボードは `_open_log` に
到達していない。そもそも起動したかを確認すること。

---

## A frozen world / ワールドが停止する

See `CLAUDE.md` — with traffic enabled the world is **synchronous** and
`generate_traffic.py` is the only tick master. If it dies or wedges,
everything freezes at once and the driver sees a still frame rather than an
error. `run_pipeline.py` watches `traffic_heartbeat.json`
(`--stall-timeout`, `--stall-grace`) and restarts the tick master.

`CLAUDE.md` を参照。交通を有効にするとワールドは **同期モード** になり、
`generate_traffic.py` だけがティックマスタとなる。これが死ぬか詰まると
全体が同時に停止し、運転者にはエラーではなく静止画が見える。
`run_pipeline.py` は `traffic_heartbeat.json` を監視
（`--stall-timeout`、`--stall-grace`）してティックマスタを再起動する。

A still frame with a **fresh** heartbeat is not a tick-master problem —
check for the GPU crash above.
ハートビートが **新しい** のに静止している場合はティックマスタの問題では
ない。上記の GPU クラッシュを確認すること。

---

## A wheel button does the wrong thing, or nothing / ホイールのボタンが違う動作をする・反応しない

The bindings are in `keybinds.json` (dashboard Settings > Keybinds), not in
the script. Check, in order:

1. **Name wrong?** The names come from `g923_buttons.json`, a table of the
   usual new-lg4ff G923 / G29 layout (pygame numbers the 25 buttons in HID
   order; the D-pad is a hat). If the page says "Triangle" for the button
   printed ○, open the *Wheel button names* card, turn **identify buttons**
   on, press it, and rename it. The binding follows the physical button.
2. **Nothing arrives when binding / identifying?** The browser cannot see the
   wheel; the driving window writes each press to `wheel_last.json`. It must
   be running, and it must be a version with this feature (restart the
   pipeline once after updating).
3. **Buttons stopped working?** Identify mode makes the driving window ignore
   wheel buttons. It ends when you stop it, leave the page, or 30 s after the
   page stops renewing it. Delete `keybind_learn.json` to end it at once.
4. **Want the old layout back?** Settings > Keybinds > **reset all** > **save**,
   or delete `keybinds.json` (recreated from the defaults at the next start).

割り当てはスクリプト内ではなく `keybinds.json`（ダッシュボードの 設定 >
キー割り当て）にある。次の順に確認する。

1. **名前が違う：** 名前は `g923_buttons.json`（new-lg4ff の G923 / G29 の一般的な
   配置。pygame は 25 個のボタンを HID 順に番号付けし、十字キーはハット）。
   ○ のボタンが「△」と表示されるなら、*ホイールのボタン名* カードで
   **ボタンを識別** をオンにし、そのボタンを押して名前を変更する。
   割り当ては物理的なボタンに付いたまま。
2. **割り当て・識別で入力が届かない：** ブラウザはホイールを読めない。
   運転ウインドウが押下を `wheel_last.json` に書くため、運転ウインドウが
   起動していて、この機能を含む版である必要がある（更新後に一度パイプラインを再起動）。
3. **ボタンが効かなくなった：** 識別モード中は運転ウインドウがボタンを無視する。
   停止するか、ページを離れるか、更新が止まって 30 秒で終わる。すぐ終えるには
   `keybind_learn.json` を削除する。
4. **元の配置に戻したい：** 設定 > キー割り当て > **すべて既定に戻す** > **保存**、
   または `keybinds.json` を削除（次回起動時に既定値で再作成）。

---

## F5 (change map) crashes the whole rig / F5（マップ切替）でリグ全体が落ちる

### Signature / 兆候

F5 in the driving window, then every window closes and the pipeline log ends
with `manual_control exited ... -> shutting down`; FFB / dashboard logs show
`terminate called ... TimeoutException` (libcarla `abort()`). From
OitaKosen, F5 also went to Town01, and the list held Town06/07, which this
package does not have.

運転ウインドウで F5 を押すと全ウインドウが閉じ、パイプラインのログは
`manual_control exited ... -> shutting down` で終わる。FFB やダッシュボードの
ログに `terminate called ... TimeoutException`（libcarla の `abort()`）が出る。
また OitaKosen から F5 で Town01 に飛び、一覧には本パッケージに無い Town06/07 が
含まれていた。

### Cause / 原因

The driving window called `client.load_world()` itself while about eight
other clients shared the world: `generate_traffic.py` (synchronous tick
master, dt 1/30, and TrafficManager server), `delivery_sim.py`,
`dashboard.py`, `carla_ffb.py`, `traction_fx.py`, `drone_viewer.py`. The load
destroyed every actor they held while the world was synchronous and the TM
was still driving the old cars. libcarla aborts in those clients (Python
cannot catch it), the TM dies with the tick master, the driving window's own
`restart()` then waits on a world nobody steps or fails, and when it exits
the pipeline shuts everything down.

運転ウインドウが、他の約 8 クライアント（同期 tick マスタ兼 TM サーバの
`generate_traffic.py`（dt 1/30）、`delivery_sim.py`、`dashboard.py`、
`carla_ffb.py`、`traction_fx.py`、`drone_viewer.py`）が共有するワールドで
自ら `client.load_world()` を呼んでいた。ワールドが同期モードで TM が旧車両を
動かしている最中に、それらが持つアクターが全て消える。libcarla は各クライアント
で abort し（Python では捕捉不可）、TM は tick マスタと共に落ち、運転ウインドウ
の `restart()` は誰も進めないワールドを待つか失敗し、終了するとパイプラインが
全体を停止していた。

### Fix / 対処

`map_switch.py` + `run_pipeline.py`: F5 only writes `map_request.json`
(target map, current car) and exits with code 75. The pipeline stops the
clients (traffic first, 20 s grace, then the rest), forces async mode if
anything was left synchronous, loads the map in a throwaway client (up to
120 s, then confirms the loaded map's name — `util/config.py`'s 10 s is too
short for OitaKosen), and starts every client again with the same options
and `--start-vehicle` = the current car. If the load fails it restarts
CarlaUE4 on that map (kill first, then start; with `--no-sim` it does not
touch your CARLA and shuts down). The cycle comes from
`client.get_available_maps()`, installed maps only.

`map_switch.py` と `run_pipeline.py`：F5 は `map_request.json`（目的のマップと
現在の車）を書いて終了コード 75 で終わるだけ。パイプラインがクライアントを止め
（交通を最初に猶予 20 秒、次に残り）、同期モードが残っていれば非同期に戻し、
使い捨てクライアントでマップを読み込み（最大 120 秒、その後読込済みマップ名を
確認。`util/config.py` の 10 秒は OitaKosen には短い）、同じオプションと
`--start-vehicle` = 現在の車で全クライアントを再起動する。読込に失敗した場合は
CarlaUE4 をそのマップで再起動する（先に終了してから起動。`--no-sim` では
ユーザーの CARLA に触れずに終了する）。巡回順は `client.get_available_maps()`
から作り、インストール済みのマップのみ。

### If it still goes wrong / それでも失敗する場合

- Log says `asked for a map switch but map_request.json is missing or stale`:
  the window could not write the file (permissions, disk full).
  / ログに `asked for a map switch but map_request.json is missing or stale`：
  ウインドウが依頼ファイルを書けなかった（権限・ディスク容量）。
- Log says `load_world(<map>) failed -> restarting CarlaUE4`: the map did not
  load in 150 s; check the CarlaUE4 output for the map's own errors.
  / ログに `load_world(<map>) failed -> restarting CarlaUE4`：150 秒以内に
  読み込めなかった。CarlaUE4 の出力でマップ自体のエラーを確認する。
- Still the old behaviour: the pipeline was started before the update;
  restart it once. Test: `tests/test_map_switch.py`.
  / 以前の挙動のまま：更新前に起動したパイプラインである。一度再起動する。
  試験：`tests/test_map_switch.py`。

## The dashboard / launcher / ダッシュボードとランチャー

- **"The dashboard UI is not built yet"**: `web/dist` is missing (a fresh
  checkout without the build, or a deleted folder). Build it:
  `export PATH=~/.local/node/bin:$PATH && cd web && npm install && npm run build`.
  The JSON routes work meanwhile. / `web/dist` が無い: ビルドする。
- **The old dashboard still shows**: the browser kept the old page. Reload it.
  / ブラウザを再読込する。
- **Launch does nothing**: is the launcher running (`python3 run_pipeline.py`
  with no arguments)? With a pipeline started from the command line, the
  dashboard's live controls work, but Launch / Restart need the launcher.
  `data/launcher_state.json` shows its phase; the pipeline's output is in
  `data/pipeline.log`. / ランチャーが動いているか確認する。
- **A live change seems lost**: requests go through `data/pipeline_control.json`.
  The pipeline reads them from the moment it starts (since 2026-11-06; before
  that, one sent while CARLA was still booting was dropped) and logs
  `dashboard request: ...` in `data/pipeline.log`. A traffic change takes about
  10-20 s: the tick master destroys its cars and spawns the new set.
  / 依頼は pipeline.log に記録される。交通の変更は 10〜20 秒かかる。
- **A web change does not appear**: rebuild `web/` (`npm run build`), then
  reload the page. / web/ を再ビルドして再読込。


## Drones (or anything delivery_sim moves) judder / ドローンがカクつく

### Signature / 兆候

Pods drive smoothly but flying drones jump: a few metres in one frame, then
still for several frames. `data/pipeline.log` has a `[loop]` line every 20 s
(`delivery/loop_prof.py`); the loop rate there is well under the server's
~30 fps, and a second `[loop] blocking CARLA calls` line names the slow calls
with their `file:line`.
ポッドは滑らかだが飛行中のドローンが飛び飛びに動く。pipeline.log の `[loop]`
行でループ周波数がサーバの約 30 fps を大きく下回り、待ちの出る呼び出しが表示される。

### Cause / 原因

delivery_sim poses the drones itself (one `ApplyTransform` per drone per loop),
so they move only as often as its loop runs. In CARLA 0.9.16 a call that
**returns data** from the server (`cast_ray`, `get_weather`, a `bounding_box`
read, a first `get_actors` with new actors) waits ~30 ms when it is the first
such call in a tick; later ones in the same tick take ~0.1 ms. Fire-and-forget
calls (`apply_batch`, `set_transform`, debug drawing) and cached reads
(`get_transform`, `get_velocity`, the snapshot) do not wait. A few such calls
per tick kept the loop at 4-6 Hz (2026-11-13). Torch's default thread pool
(one per core) also made the RL brains ~3x slower per call.
ドローンは delivery_sim のループで動かすため、ループが遅いとカクつく。0.9.16 では
データを返す呼び出しが tick の最初だと約 30 ms 待つ（2026-11-13、ループ 4〜6 Hz）。

### Fix / 対処

Keep data-returning calls out of the per-tick path: read static things once
(`_BoxedCar` / `VEHICLES` in `delivery_sim.py` for BasicAgent's vehicle boxes,
the surface-ray grid in `make_surface_fn`, the half-length cache in
`traffic_profile.py`), or run them on a thread with its own client
(`RayWorker` in `delivery/backends.py`; the wait releases the GIL). Torch is set
to one thread in `delivery/rl_bridge.py`. After these the loop runs at 27-30 Hz
with drones flying. Use the `[loop]` lines to find the next culprit.
データを返す呼び出しを毎 tick の処理から外す（一度だけ読む・専用クライアントの
スレッドで投げる）。torch は 1 スレッド。修正後は飛行中でも 27〜30 Hz。
