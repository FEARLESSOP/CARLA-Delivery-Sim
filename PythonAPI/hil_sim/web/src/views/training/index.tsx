// AI training page: trainer control, learning charts, watch mode (replay of
// one recorded flight on a top-down map + the wind close-up).
// 強化学習ページ：学習の操作・進捗グラフ・ウォッチモード（記録した飛行の再生）
import { useEffect, useMemo, useRef, useState } from 'react'
import '@/i18n/training'
import { useT } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { postJSON } from '@/lib/api'
import { StaticLayer, useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { fmt, fmtDur } from '@/lib/fmt'
import { Badge, Button, Card, EmptyState, Grid, KV, SectionTitle, Segmented, Select, Slider, Stat, Stepper } from '@/components/ui'

interface ProgRow { steps: number; wall_s: number; wind_level: number; reached: number; crash: number; xtrack_mean: number; xtrack_max: number; guard_steps: number; battery_used: number; ep_reward: number }
interface TrainState { brain?: string; running?: boolean; steps?: number; wind_level?: number; deadline?: number; last?: Partial<ProgRow> }
type Brain = 'flight' | 'mission' | 'nav' | 'fleet' | 'drive'
interface RuleBase { deliveries: number; crashes_per_100: number; lead_mean_s: number; landing_min: number }
interface RlState { state: TrainState; alive: boolean; progress: ProgRow[]; checkpoints: string[]; drone_checkpoints?: string[]; rule?: RuleBase | null }
interface Frame { t: number; p: number[]; c: number[]; wind: number[]; cmd: number[]; dev: number; guard: boolean }
interface Rect { x0: number; y0: number; x1: number; y1: number }
interface Rec {
  checkpoint: string; wind_level: number; seed: number; path: number[][]; frames: Frame[]
  kpi: { reached: boolean; crash: boolean; xtrack_mean: number; xtrack_max: number; guard_steps: number; battery_used: number; time_s: number; path_m: number }
  box: number[]; nofly: Rect[]; zones: Record<string, Rect>; buildings: number[][]
}

const ZONE_COL = [C.cyan, C.s3, C.violet, C.accent]

function Trainer({ st, brain }: { st: RlState | null; brain: Brain }) {
  const t = useT()
  const [hours, setHours] = useState(3)
  const [busy, setBusy] = useState(false)
  const s = st?.state ?? {}
  const running = !!st?.alive
  const left = running && s.deadline ? Math.max(0, s.deadline - Date.now() / 1000) : 0
  const go = async (body: object) => { setBusy(true); try { await postJSON('/rl/train', body) } finally { setBusy(false) } }
  return (
    <Card title={t('tr_trainer')} actions={<Badge tone={running ? 'good' : 'neutral'} dot>{running ? t('tr_running') : t('tr_idle')}</Badge>}>
      <Select label={t('tr_brain')} value={brain} options={[{ value: brain, label: t(`tr_b_${brain}`) }]} onChange={() => {}} disabled />
      <Stepper label={t('tr_hours')} value={hours} min={0.5} max={24} step={0.5} onChange={setHours} />
      <div className="flex flex-wrap gap-2">
        <Button variant="primary" busy={busy} disabled={running} onClick={() => go({ action: 'start', brain, hours })}>{t('tr_start')}</Button>
        <Button busy={busy} disabled={running || !st?.checkpoints.length} onClick={() => go({ action: 'start', brain, hours, resume: true })}>{t('tr_resume')}</Button>
        <Button variant="danger" disabled={!running} onClick={() => go({ action: 'stop', brain })}>{t('tr_stop')}</Button>
      </div>
      <div className="grid grid-cols-2 gap-3">
        <Stat size="md" label={t('tr_steps')} value={fmt((s.steps ?? 0) / 1e6, 2)} unit="M" />
        {(brain === 'nav' || brain === 'flight') && <Stat size="md" label={t('tr_wind')} value={fmt((s.wind_level ?? 0) * 100, 0)} unit="%" />}
      </div>
      {running && <KV k={t('tr_left')} v={fmtDur(left)} />}
      {brain === 'flight' && s.last && <>
        <KV k={t('tr_reached')} v={`${fmt((s.last.reached ?? 0) * 100, 0)} %`} tone={(s.last.reached ?? 0) >= 0.8 ? 'good' : 'warn'} />
        <KV k={t('tr_xtrack')} v={`${fmt(s.last.xtrack_mean ?? 0, 2)} m`} />
        <KV k={t('tr_crash')} v={`${fmt((s.last.crash ?? 0) * 100, 0)} %`} tone={(s.last.crash ?? 0) > 0 ? 'bad' : 'good'} />
        <KV k={t('tr_guard')} v={String(s.last.guard_steps ?? 0)} />
      </>}
    </Card>
  )
}

function Learning({ rows, brain, rule }: { rows: Record<string, number>[]; brain: Brain; rule?: RuleBase | null }) {
  const t = useT()
  const series: [string, number, string, string][] = brain === 'flight' ? [
    ['reached', 100, C.accent, t('tr_s_reached')], ['crash', 100, C.danger, t('tr_s_crash')],
    ['xtrack_mean', 10, C.s1, t('tr_s_xtrack')], ['wind_level', 100, C.warn, t('tr_s_wind')],
  ] : brain === 'fleet' ? [
    ['deliveries_per_h', 1, C.accent, t('tr_f_deliv')], ['crashes_per_100', 1, C.danger, t('tr_f_crash')],
    ['stranded', 20, C.violet, t('tr_f_strand')], ['hover_s_per_h', 0.1, C.cyan, t('tr_f_hover')],
  ] : brain === 'drive' ? [
    ['reached', 100, C.accent, t('tr_d_reached')], ['kerb', 100, C.danger, t('tr_d_kerb')], ['collision', 100, C.violet, t('tr_d_coll')],
    ['speed', 10, C.cyan, t('tr_d_speed')], ['route_len', 0.1, C.s1, t('tr_d_route')], ['traffic', 10, C.warn, t('tr_d_traffic')],
  ] : brain === 'nav' ? [
    ['delivered', 100, C.cyan, t('tr_n_delivered')], ['reached', 100, C.accent, t('tr_n_reached')], ['crash_obstacle', 100, C.danger, t('tr_n_obst')],
    ['crash_nofly', 100, C.violet, t('tr_n_nofly')], ['dist', 0.1, C.s1, t('tr_n_dist')], ['wind_level', 100, C.warn, t('tr_s_wind')],
  ] : [
    ['deliveries', 5, C.accent, t('tr_m_deliv')], ['crashes_per_100', 5, C.danger, t('tr_m_crash')],
    ['lead_mean_s', 1 / 60, C.s1, t('tr_m_lead')], ['landing_min', 1, C.warn, t('tr_m_land')],
  ]
  const fr = rule as unknown as Record<string, number> | null | undefined
  const foot = brain === 'flight' ? t('tr_pid') : brain === 'nav' ? t('tr_n_base') : brain === 'drive' ? t('tr_d_base')
    : brain === 'fleet' ? (fr ? t('tr_f_base', { d: fmt(fr.deliveries_per_h, 1), c: fmt(fr.crashes_per_100, 1), s: fmt(fr.stranded, 1), h: fmt(fr.hover_s_per_h, 0) }) : '')
    : rule ? t('tr_m_base', {
    d: fmt(rule.deliveries, 1), c: fmt(rule.crashes_per_100, 1), l: fmt(rule.lead_mean_s / 60, 0), b: fmt(rule.landing_min, 0) }) : ''
  const ref = useCanvas((g, w, h) => {
    const L = 34, B = 20, T = 8
    g.strokeStyle = C.grid; g.fillStyle = C.mute; g.font = `10px ${FONT_MONO}`; g.lineWidth = 1
    for (let v = 0; v <= 100; v += 25) {
      const y = T + (h - T - B) * (1 - v / 100)
      g.beginPath(); g.moveTo(L, y); g.lineTo(w, y); g.stroke(); g.fillText(String(v), 4, y + 3)
    }
    if (rows.length < 1) return
    // fleet rounds alternate pod / drone, so plot by round, not steps / フリートはラウンド順
    const byRound = brain === 'fleet'
    const xs = rows.map((r, k) => (byRound ? k + 1 : r.steps))
    const mx = xs[xs.length - 1] || 1
    g.fillText(byRound ? `${rows.length} rounds` : `${fmt(mx / 1e6, 1)}M steps`, w - 80, h - 5)
    for (const [k, sc, col] of series) {
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath()
      rows.forEach((r, i) => {
        const x = L + (w - L - 4) * (xs[i] / mx), y = T + (h - T - B) * (1 - Math.min(100, Number(r[k] ?? 0) * sc) / 100)
        if (i) g.lineTo(x, y); else g.moveTo(x, y)
      })
      g.stroke()
    }
  }, [rows, brain])
  return (
    <Card title={t('tr_learning')} wide footer={foot}>
      {rows.length ? <canvas ref={ref} className="h-56 w-full" aria-label={t('tr_learning')} /> : <EmptyState>{t('tr_noprog')}</EmptyState>}
      <ul className="flex flex-wrap gap-3 text-xs text-fg-mute">
        {series.map(([k, , col, lab]) => <li key={k} className="flex items-center gap-1.5"><span className="inline-block h-0.5 w-4" style={{ background: col }} />{lab}</li>)}
      </ul>
    </Card>
  )
}

function useReplay(rec: Rec | null) {
  const [i, setI] = useState(0)
  const [play, setPlay] = useState(false)
  const [speed, setSpeed] = useState('5')
  const last = useRef(0)
  useEffect(() => { setI(0); setPlay(!!rec) }, [rec])
  useEffect(() => {
    if (!play || !rec) return
    let raf = 0
    last.current = performance.now()
    let acc = 0
    const step = (now: number) => {
      acc += (now - last.current) / 1000 * Number(speed) * 10        // frames are 0.1 s / 10 Hz
      last.current = now
      const k = Math.floor(acc)
      if (k > 0) { acc -= k; setI((v) => { const n = Math.min(rec.frames.length - 1, v + k); if (n === rec.frames.length - 1) setPlay(false); return n }) }
      raf = requestAnimationFrame(step)
    }
    raf = requestAnimationFrame(step)
    return () => cancelAnimationFrame(raf)
  }, [play, speed, rec])
  return { i, setI, play, setPlay, speed, setSpeed }
}

function MapView({ rec, i }: { rec: Rec; i: number }) {
  const layer = useRef(new StaticLayer())
  const ref = useCanvas((g, w, h) => {
    const [x0, y0, x1, y1] = rec.box
    const s = Math.min(w / (x1 - x0), h / (y1 - y0))
    const ox = (w - (x1 - x0) * s) / 2, oy = (h - (y1 - y0) * s) / 2
    const P = (x: number, y: number): [number, number] => [ox + (x - x0) * s, oy + (y - y0) * s]
    layer.current.draw(g, w, h, rec, (g) => {
    g.fillStyle = C.bg; g.fillRect(0, 0, w, h)
    Object.entries(rec.zones).forEach(([k, r], n) => {
      const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
      g.strokeStyle = ZONE_COL[n % 4] + '66'; g.lineWidth = 2; g.strokeRect(a, b, c - a, d - b)
      g.fillStyle = ZONE_COL[n % 4]; g.font = `11px ${FONT_MONO}`; g.fillText(`POD ${k}`, a + 4, b + 14)
    })
    for (const r of rec.nofly) {
      const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
      g.fillStyle = C.danger + '22'; g.fillRect(a, b, c - a, d - b); g.strokeStyle = C.danger; g.lineWidth = 2; g.strokeRect(a, b, c - a, d - b)
    }
    g.fillStyle = '#334155'
    for (const [bx, by, bl, bw, yaw] of rec.buildings) {
      const [cx, cy] = P(bx, by)
      g.save(); g.translate(cx, cy); g.rotate((yaw * Math.PI) / 180); g.fillRect(-bl * s / 2, -bw * s / 2, bl * s, bw * s); g.restore()
    }
    g.setLineDash([6, 5]); g.strokeStyle = C.mute; g.lineWidth = 1.5; g.beginPath()
    rec.path.forEach((p, k) => { const [a, b] = P(p[0], p[1]); if (k) g.lineTo(a, b); else g.moveTo(a, b) })
    g.stroke(); g.setLineDash([])
    })
    g.strokeStyle = C.accent; g.lineWidth = 2; g.beginPath()
    for (let k = 0; k <= i; k++) { const [a, b] = P(rec.frames[k].p[0], rec.frames[k].p[1]); if (k) g.lineTo(a, b); else g.moveTo(a, b) }
    g.stroke()
    g.fillStyle = C.danger
    for (let k = 0; k <= i; k++) if (rec.frames[k].guard) { const [a, b] = P(rec.frames[k].p[0], rec.frames[k].p[1]); g.fillRect(a - 2, b - 2, 4, 4) }
    const f = rec.frames[i]
    const [dx, dy] = P(f.p[0], f.p[1])
    const ws = Math.hypot(f.wind[0], f.wind[1])
    g.strokeStyle = C.warn; g.lineWidth = 3; g.beginPath(); g.moveTo(dx, dy); g.lineTo(dx + f.wind[0] * 4, dy + f.wind[1] * 4); g.stroke()
    g.fillStyle = C.fg; g.beginPath(); g.arc(dx, dy, 5, 0, Math.PI * 2); g.fill()
    g.fillStyle = C.fg2; g.font = `12px ${FONT_MONO}`
    g.fillText(`t ${fmt(f.t, 1)} s   alt ${fmt(f.p[2], 1)} m   off path ${fmt(f.dev, 2)} m   wind ${fmt(ws, 1)} m/s`, 10, h - 10)
  }, [rec, i])
  return <canvas ref={ref} className="h-[460px] w-full rounded-lg" aria-label="watch map" />
}

function CloseUp({ rec, i }: { rec: Rec; i: number }) {
  const t = useT()
  const data = useMemo(() => rec.frames.map((f, k) => {
    const n = rec.frames[Math.min(k + 1, rec.frames.length - 1)].c, c = f.c
    let tx = n[0] - c[0], ty = n[1] - c[1]
    const L = Math.hypot(tx, ty)
    if (L < 1e-3) { tx = 1; ty = 0 } else { tx /= L; ty /= L }
    const cross = f.wind[0] * -ty + f.wind[1] * tx
    const err = (f.p[0] - c[0]) * -ty + (f.p[1] - c[1]) * tx
    return [cross, err, Math.hypot(f.cmd[0], f.cmd[1]) * 30]
  }), [rec])
  const cols = [C.warn, C.s1, C.accent]
  const ref = useCanvas((g, w, h) => {
    const mx = Math.max(5, ...data.map((d) => Math.max(Math.abs(d[0]), Math.abs(d[1]), d[2] / 3)))
    const y = (v: number) => h / 2 - (v / mx) * (h / 2 - 6)
    g.strokeStyle = C.grid; g.beginPath(); g.moveTo(0, h / 2); g.lineTo(w, h / 2); g.stroke()
    ;[0, 1, 2].forEach((s) => {
      g.strokeStyle = cols[s]; g.lineWidth = 1.5; g.beginPath()
      data.forEach((d, k) => { const x = (k / Math.max(1, data.length - 1)) * w, v = s === 2 ? d[2] / 3 : d[s]; if (k) g.lineTo(x, y(v)); else g.moveTo(x, y(v)) })
      g.stroke()
    })
    const cx = (i / Math.max(1, data.length - 1)) * w
    g.strokeStyle = C.fg; g.beginPath(); g.moveTo(cx, 0); g.lineTo(cx, h); g.stroke()
  }, [data, i])
  return (
    <Card title={t('tr_closeup')} wide>
      <canvas ref={ref} className="h-40 w-full" aria-label={t('tr_closeup')} />
      <ul className="flex flex-wrap gap-3 text-xs text-fg-mute">
        {[t('tr_cross'), t('tr_err'), t('tr_tilt') + ' ÷3'].map((l, k) => <li key={l} className="flex items-center gap-1.5"><span className="inline-block h-0.5 w-4" style={{ background: cols[k] }} />{l}</li>)}
      </ul>
    </Card>
  )
}

function Watch({ ckpts }: { ckpts: string[] }) {
  const t = useT()
  const [ck, setCk] = useState('latest')
  const [wind, setWind] = useState(1.0)
  const [seed, setSeed] = useState(7)
  const [rec, setRec] = useState<Rec | null>(null)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState<string | null>(null)
  const r = useReplay(rec)
  const record = async () => {
    setBusy(true); setErr(null)
    try { setRec(await postJSON<Rec>('/rl/watch', { brain: 'flight', checkpoint: ck, wind, seed })) }
    catch (e) { setErr(String(e)) } finally { setBusy(false) }
  }
  const opts = [{ value: 'latest', label: t('tr_latest') }, { value: 'pid', label: t('tr_pidopt') },
    ...ckpts.slice().reverse().map((c) => ({ value: c, label: c.replace('step_', '').replace('.zip', '').replace(/^0+/, '') + ' steps' }))]
  return (
    <>
      <Card title={t('tr_watch')} className="xl:col-span-4" footer={t('tr_legend')}>
        <div className="grid gap-3 md:grid-cols-4">
          <Select label={t('tr_ckpt')} value={ck} options={opts} onChange={setCk} />
          <Slider label={t('tr_wwind')} value={wind} min={0} max={1} step={0.1} onChange={setWind} format={(v) => `${Math.round(v * 100)} %`} />
          <Stepper label={t('tr_seed')} value={seed} min={0} max={9999} onChange={setSeed} />
          <div className="flex items-end"><Button variant="primary" busy={busy} onClick={record}>{busy ? t('tr_recording') : t('tr_record')}</Button></div>
        </div>
        {err && <Badge tone="bad">{t('tr_err_msg')}: {err.slice(0, 160)}</Badge>}
        {rec ? <>
          <MapView rec={rec} i={r.i} />
          <div className="flex flex-wrap items-center gap-3">
            <Button size="sm" onClick={() => { if (r.i >= rec.frames.length - 1) r.setI(0); r.setPlay(!r.play) }}>{r.play ? t('tr_pause') : t('tr_play')}</Button>
            <Segmented size="sm" label={t('tr_speed')} value={r.speed} onChange={r.setSpeed}
              options={['1', '5', '20', '100'].map((v) => ({ value: v, label: `${v}×` }))} />
            <div className="min-w-[240px] flex-1">
              <Slider label={t('tr_time')} value={r.i} min={0} max={rec.frames.length - 1} onChange={(v) => { r.setPlay(false); r.setI(v) }}
                format={(v) => `${fmt(rec.frames[v]?.t ?? 0, 1)} s`} />
            </div>
          </div>
        </> : <EmptyState>{t('tr_nowatch')}</EmptyState>}
      </Card>
      {rec && <>
        <CloseUp rec={rec} i={r.i} />
        <Card title={t('tr_episode')}>
          <KV k={t('tr_done')} v={rec.kpi.reached ? t('tr_yes') : t('tr_no')} tone={rec.kpi.reached ? 'good' : 'bad'} />
          <KV k={t('tr_crash')} v={rec.kpi.crash ? t('tr_yes') : t('tr_no')} tone={rec.kpi.crash ? 'bad' : 'good'} />
          <KV k={t('tr_xtrack')} v={`${fmt(rec.kpi.xtrack_mean, 2)} m (max ${fmt(rec.kpi.xtrack_max, 1)})`} />
          <KV k={t('tr_guard')} v={String(rec.kpi.guard_steps)} tone={rec.kpi.guard_steps ? 'warn' : 'good'} />
          <KV k={t('tr_batt')} v={`${fmt(rec.kpi.battery_used, 1)} %`} />
          <KV k={t('tr_flight_t')} v={`${fmt(rec.kpi.time_s, 0)} s / ${fmt(rec.kpi.path_m, 0)} m`} />
        </Card>
      </>}
    </>
  )
}

interface MFrame { t: number; pod: number[]; moving: boolean; drones: number[][]; orders: number[][]; wind: number; rain: number }
interface MRec {
  checkpoint: string; seed: number; pod: string; frames: MFrame[]; events: { t: number; drone: number; what: string; batt: number }[]
  kpi: { deliveries: number; lead_mean_s: number | null; crashes: number; crash_why: Record<string, number>; crashes_per_100: number;
    landing_min: number | null; hold_s: number; queue_end: number }
  box: number[]; zone: Rect; nofly: Rect[]; lines: number[][][]
}
const DSTATE = [C.accent, C.info, C.cyan, C.danger]

function MissionMap({ rec, i }: { rec: MRec; i: number }) {
  const layer = useRef(new StaticLayer())
  const ref = useCanvas((g, w, h) => {
    const [x0, y0, x1, y1] = rec.box
    const s = Math.min(w / (x1 - x0), h / (y1 - y0))
    const ox = (w - (x1 - x0) * s) / 2, oy = (h - (y1 - y0) * s) / 2
    const P = (x: number, y: number): [number, number] => [ox + (x - x0) * s, oy + (y - y0) * s]
    layer.current.draw(g, w, h, rec, (g) => {
    g.fillStyle = C.bg; g.fillRect(0, 0, w, h)
    g.strokeStyle = C.line; g.lineWidth = 1
    for (const l of rec.lines) { g.beginPath(); l.forEach((p, k) => { const [a, b] = P(p[0], p[1]); if (k) g.lineTo(a, b); else g.moveTo(a, b) }); g.stroke() }
    const z = rec.zone, [za, zb] = P(z.x0, z.y0), [zc, zd] = P(z.x1, z.y1)
    g.strokeStyle = C.cyan; g.lineWidth = 2; g.strokeRect(za, zb, zc - za, zd - zb)
    for (const r of rec.nofly) {
      const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
      g.fillStyle = C.danger + '22'; g.fillRect(a, b, c - a, d - b); g.strokeStyle = C.danger; g.strokeRect(a, b, c - a, d - b)
    }
    })
    const f = rec.frames[i]
    if (!f) return
    g.fillStyle = C.warn
    for (const o of f.orders) { const [a, b] = P(o[0], o[1]); g.beginPath(); g.arc(a, b, 3, 0, Math.PI * 2); g.fill() }
    const [px, py] = P(f.pod[0], f.pod[1])
    g.fillStyle = C.fg; g.fillRect(px - 6, py - 6, 12, 12)
    f.drones.forEach((d, k) => {
      const [a, b] = P(d[0], d[1])
      g.strokeStyle = DSTATE[d[2]] ?? C.mute; g.lineWidth = 1; g.setLineDash([3, 3])
      if (d[2] === 1 || d[2] === 2) { g.beginPath(); g.moveTo(px, py); g.lineTo(a, b); g.stroke() }
      g.setLineDash([]); g.fillStyle = DSTATE[d[2]] ?? C.mute
      // drones in the pod sit round its square, not under it / ポッド内の機体は四角の周りに
      const [qx, qy] = d[2] === 0 ? [px + Math.cos(k * Math.PI / 2 + Math.PI / 4) * 13, py + Math.sin(k * Math.PI / 2 + Math.PI / 4) * 13] : [a, b]
      g.beginPath(); g.arc(qx, qy, 4.5, 0, Math.PI * 2); g.fill()
      g.fillStyle = C.fg2; g.font = `10px ${FONT_MONO}`; g.fillText(String(k + 1), qx + 5, qy - 5)
    })
    g.fillStyle = C.fg2; g.font = `12px ${FONT_MONO}`
    g.fillText(`t ${fmtDur(f.t)}   wind @pod ${fmt(f.wind, 1)} m/s   rain ${fmt(f.rain, 0)}   orders waiting ${f.orders.length}   batteries ${f.drones.map((d) => fmt(d[3], 0)).join('/')} %`, 10, h - 10)
  }, [rec, i])
  return <canvas ref={ref} className="h-[460px] w-full rounded-lg" aria-label="mission map" />
}

function MissionWatch({ ckpts }: { ckpts: string[] }) {
  const t = useT()
  const [ck, setCk] = useState('latest')
  const [seed, setSeed] = useState(7)
  const [rec, setRec] = useState<MRec | null>(null)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState<string | null>(null)
  const r = useReplay(rec as unknown as Rec | null)
  const record = async () => {
    setBusy(true); setErr(null)
    try { setRec(await postJSON<MRec>('/rl/watch', { brain: 'mission', checkpoint: ck, seed })) }
    catch (e) { setErr(String(e)) } finally { setBusy(false) }
  }
  const opts = [{ value: 'latest', label: t('tr_latest') }, { value: 'rule', label: t('tr_rule') },
    ...ckpts.slice().reverse().map((c) => ({ value: c, label: c.replace('step_', '').replace('.zip', '').replace(/^0+/, '') + ' steps' }))]
  const now = rec?.frames[r.i]?.t ?? 0
  const recent = rec ? rec.events.filter((e) => e.t <= now).slice(-8).reverse() : []
  return (
    <>
      <Card title={t('tr_watch')} className="xl:col-span-3" footer={t('tr_m_legend')}>
        <div className="grid gap-3 md:grid-cols-3">
          <Select label={t('tr_ckpt')} value={ck} options={opts} onChange={setCk} />
          <Stepper label={t('tr_seed')} value={seed} min={0} max={9999} onChange={setSeed} />
          <div className="flex items-end"><Button variant="primary" busy={busy} onClick={record}>{busy ? t('tr_recording') : t('tr_record')}</Button></div>
        </div>
        {err && <Badge tone="bad">{t('tr_err_msg')}: {err.slice(0, 160)}</Badge>}
        {rec ? <>
          <MissionMap rec={rec} i={r.i} />
          <div className="flex flex-wrap items-center gap-3">
            <Button size="sm" onClick={() => { if (r.i >= rec.frames.length - 1) r.setI(0); r.setPlay(!r.play) }}>{r.play ? t('tr_pause') : t('tr_play')}</Button>
            <Segmented size="sm" label={t('tr_speed')} value={r.speed} onChange={r.setSpeed}
              options={['1', '5', '20', '100'].map((v) => ({ value: v, label: `${Number(v) * 50}×` }))} />
            <div className="min-w-[240px] flex-1">
              <Slider label={t('tr_time')} value={r.i} min={0} max={rec.frames.length - 1} onChange={(v) => { r.setPlay(false); r.setI(v) }}
                format={(v) => fmtDur(rec.frames[v]?.t ?? 0)} />
            </div>
          </div>
        </> : <EmptyState>{t('tr_m_none')}</EmptyState>}
      </Card>
      <Card title={rec ? t('tr_episode') : t('tr_m_events')}>
        {rec && <>
          <KV k={t('tr_m_deliveries')} v={String(rec.kpi.deliveries)} />
          <KV k={t('tr_m_lead_k')} v={rec.kpi.lead_mean_s == null ? '–' : `${fmt(rec.kpi.lead_mean_s / 60, 1)} min`} />
          <KV k={t('tr_m_c100')} v={fmt(rec.kpi.crashes_per_100, 1)} tone={rec.kpi.crashes ? 'bad' : 'good'} />
          <KV k={t('tr_m_why')} v={Object.entries(rec.kpi.crash_why).map(([k, v]) => `${k} ${v}`).join(', ') || '–'} />
          <KV k={t('tr_m_landmin')} v={rec.kpi.landing_min == null ? '–' : `${fmt(rec.kpi.landing_min, 0)} %`} />
          <KV k={t('tr_m_hold')} v={fmtDur(rec.kpi.hold_s)} />
          <KV k={t('tr_m_queue')} v={String(rec.kpi.queue_end)} />
          <h4 className="mt-2 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('tr_m_events')}</h4>
          <ul className="flex flex-col gap-1 font-mono text-xs text-fg-2">
            {recent.map((e, k) => <li key={k}>{fmtDur(e.t)} · D{e.drone + 1} · {e.what} · {fmt(e.batt, 0)} %</li>)}
          </ul>
        </>}
      </Card>
    </>
  )
}

interface NFrame { t: number; p: number[]; wind: number[]; cmd: number[]; clear: number }
interface NRec {
  checkpoint: string; wind_level: number; seed: number; start: number[]; goal: number[]; meet?: number[]; frames: NFrame[]
  kpi: { reached: boolean; delivered?: boolean; crash: string; time_s: number; battery_used: number; min_clear: number; start_dist: number }
  box: number[]; nofly: Rect[]; cell: number; nx: number; ny: number; above: number[]; ground: number[]
}

function altColor(agl: number): string {
  const f = Math.max(0, Math.min(1, agl / 60))
  return `hsl(${Math.round(190 - 150 * f)} 90% 60%)`
}

function NavMap({ rec, i }: { rec: NRec; i: number }) {
  const bg = useMemo(() => {
    const c = document.createElement('canvas'); c.width = rec.nx; c.height = rec.ny
    const g = c.getContext('2d')!; const img = g.createImageData(rec.nx, rec.ny)
    for (let x = 0; x < rec.nx; x++) for (let y = 0; y < rec.ny; y++) {
      const a = rec.above[x * rec.ny + y], k = (y * rec.nx + x) * 4, f = Math.min(1, a / 30)
      img.data[k] = 18 + 200 * f; img.data[k + 1] = 24 + 140 * f; img.data[k + 2] = 38 + 40 * f; img.data[k + 3] = 255
    }
    g.putImageData(img, 0, 0); return c
  }, [rec])
  const layer = useRef(new StaticLayer())
  const ref = useCanvas((g, w, h) => {
    const [x0, y0, x1, y1] = rec.box
    const s = Math.min(w / (x1 - x0), h / (y1 - y0))
    const ox = (w - (x1 - x0) * s) / 2, oy = (h - (y1 - y0) * s) / 2
    const P = (x: number, y: number): [number, number] => [ox + (x - x0) * s, oy + (y - y0) * s]
    layer.current.draw(g, w, h, rec, (g) => {
    g.fillStyle = C.bg; g.fillRect(0, 0, w, h)
    g.imageSmoothingEnabled = false
    g.drawImage(bg, ox, oy, rec.nx * rec.cell * s, rec.ny * rec.cell * s)
    for (const r of rec.nofly) {
      const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
      g.fillStyle = C.danger + '30'; g.fillRect(a, b, c - a, d - b); g.strokeStyle = C.danger; g.lineWidth = 2; g.strokeRect(a, b, c - a, d - b)
    }
    const [sx, sy] = P(rec.start[0], rec.start[1]); g.strokeStyle = C.fg; g.lineWidth = 2; g.strokeRect(sx - 6, sy - 6, 12, 12)
    const [gx, gy] = P(rec.goal[0], rec.goal[1]); g.strokeStyle = C.accent; g.lineWidth = 3; g.beginPath(); g.arc(gx, gy, 8, 0, Math.PI * 2); g.stroke()
    if (rec.meet) {
      const [mx, my] = P(rec.meet[0], rec.meet[1])
      g.fillStyle = C.cyan; g.fillRect(mx - 6, my - 6, 12, 12)
      g.fillStyle = C.fg2; g.font = `11px ${FONT_MONO}`; g.fillText('POD', mx + 9, my + 4)
    }
    })
    for (let k = 1; k <= i; k++) {
      const a = rec.frames[k - 1], b = rec.frames[k]
      const gi = (p: number[]) => { const ix = Math.min(rec.nx - 1, Math.max(0, Math.floor((p[0] - x0) / rec.cell))), iy = Math.min(rec.ny - 1, Math.max(0, Math.floor((p[1] - y0) / rec.cell))); return rec.ground[ix * rec.ny + iy] }
      g.strokeStyle = altColor(b.p[2] - gi(b.p)); g.lineWidth = 3
      const [ax, ay] = P(a.p[0], a.p[1]), [bx, by] = P(b.p[0], b.p[1]); g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke()
    }
    const f = rec.frames[i]
    if (!f) return
    const [dx, dy] = P(f.p[0], f.p[1])
    g.strokeStyle = C.warn; g.lineWidth = 3; g.beginPath(); g.moveTo(dx, dy); g.lineTo(dx + f.wind[0] * 4, dy + f.wind[1] * 4); g.stroke()
    g.fillStyle = C.fg; g.beginPath(); g.arc(dx, dy, 5, 0, Math.PI * 2); g.fill()
    g.fillStyle = C.fg2; g.font = `12px ${FONT_MONO}`
    g.fillText(`t ${fmt(f.t, 1)} s   alt ${fmt(f.p[2], 1)} m   clearance ${fmt(f.clear, 1)} m   wind ${fmt(Math.hypot(f.wind[0], f.wind[1]), 1)} m/s`, 10, h - 10)
  }, [rec, i, bg])
  return <canvas ref={ref} className="h-[460px] w-full rounded-lg" aria-label="navigation map" />
}

function NavProfile({ rec, i }: { rec: NRec; i: number }) {
  const t = useT()
  const prof = useMemo(() => {
    let d = 0
    return rec.frames.map((f, k) => {
      if (k) d += Math.hypot(f.p[0] - rec.frames[k - 1].p[0], f.p[1] - rec.frames[k - 1].p[1])
      const ix = Math.min(rec.nx - 1, Math.max(0, Math.floor((f.p[0] - rec.box[0]) / rec.cell)))
      const iy = Math.min(rec.ny - 1, Math.max(0, Math.floor((f.p[1] - rec.box[1]) / rec.cell)))
      const gr = rec.ground[ix * rec.ny + iy]
      return { d, z: f.p[2], g: gr, top: gr + rec.above[ix * rec.ny + iy] }
    })
  }, [rec])
  const ref = useCanvas((g, w, h) => {
    if (prof.length < 2) return
    const D = Math.max(1, prof[prof.length - 1].d)
    const zmin = Math.min(...prof.map((p) => p.g)) - 2, zmax = Math.max(...prof.map((p) => Math.max(p.z, p.top))) + 5
    const X = (d: number) => (d / D) * (w - 8) + 4, Y = (z: number) => h - 6 - ((z - zmin) / (zmax - zmin)) * (h - 12)
    g.fillStyle = '#7c5a2a'
    prof.forEach((p, k) => { if (k) { const x = X(prof[k - 1].d), x2 = X(p.d); g.fillRect(x, Y(p.top), Math.max(1, x2 - x), Y(p.g) - Y(p.top)) } })
    g.fillStyle = '#334155'
    prof.forEach((p, k) => { if (k) { const x = X(prof[k - 1].d), x2 = X(p.d); g.fillRect(x, Y(p.g), Math.max(1, x2 - x), h - Y(p.g)) } })
    g.strokeStyle = C.cyan; g.lineWidth = 2; g.beginPath()
    prof.slice(0, i + 1).forEach((p, k) => { if (k) g.lineTo(X(p.d), Y(p.z)); else g.moveTo(X(p.d), Y(p.z)) }); g.stroke()
    const c = prof[Math.min(i, prof.length - 1)]
    g.fillStyle = C.fg; g.beginPath(); g.arc(X(c.d), Y(c.z), 4, 0, Math.PI * 2); g.fill()
  }, [prof, i])
  return (
    <Card title={t('tr_n_profile')} className="xl:col-span-3" footer={t('tr_n_prof_leg')}>
      <canvas ref={ref} className="h-40 w-full" aria-label={t('tr_n_profile')} />
    </Card>
  )
}

function NavWatch({ ckpts }: { ckpts: string[] }) {
  const t = useT()
  const [ck, setCk] = useState('latest')
  const [wind, setWind] = useState(0)
  const [seed, setSeed] = useState(7)
  const [rec, setRec] = useState<NRec | null>(null)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState<string | null>(null)
  const r = useReplay(rec as unknown as Rec | null)
  const record = async () => {
    setBusy(true); setErr(null)
    try { setRec(await postJSON<NRec>('/rl/watch', { brain: 'nav', checkpoint: ck, wind, seed })) }
    catch (e) { setErr(String(e)) } finally { setBusy(false) }
  }
  const opts = [{ value: 'latest', label: t('tr_latest') }, { value: 'scripted', label: t('tr_scripted') },
    ...ckpts.slice().reverse().map((c) => ({ value: c, label: c.replace('step_', '').replace('.zip', '').replace(/^0+/, '') + ' steps' }))]
  const k = rec?.kpi
  return (
    <>
      <Card title={t('tr_watch')} className="xl:col-span-3" footer={t('tr_n_legend')}>
        <div className="grid gap-3 md:grid-cols-4">
          <Select label={t('tr_ckpt')} value={ck} options={opts} onChange={setCk} />
          <Slider label={t('tr_wwind')} value={wind} min={0} max={1} step={0.1} onChange={setWind} format={(v) => `${Math.round(v * 100)} %`} />
          <Stepper label={t('tr_seed')} value={seed} min={0} max={9999} onChange={setSeed} />
          <div className="flex items-end"><Button variant="primary" busy={busy} onClick={record}>{busy ? t('tr_recording') : t('tr_record')}</Button></div>
        </div>
        {err && <Badge tone="bad">{t('tr_err_msg')}: {err.slice(0, 160)}</Badge>}
        {rec ? <>
          <NavMap rec={rec} i={r.i} />
          <div className="flex flex-wrap items-center gap-3">
            <Button size="sm" onClick={() => { if (r.i >= rec.frames.length - 1) r.setI(0); r.setPlay(!r.play) }}>{r.play ? t('tr_pause') : t('tr_play')}</Button>
            <Segmented size="sm" label={t('tr_speed')} value={r.speed} onChange={r.setSpeed}
              options={['1', '5', '20', '100'].map((v) => ({ value: v, label: `${v}×` }))} />
            <div className="min-w-[240px] flex-1">
              <Slider label={t('tr_time')} value={r.i} min={0} max={rec.frames.length - 1} onChange={(v) => { r.setPlay(false); r.setI(v) }}
                format={(v) => `${fmt(rec.frames[v]?.t ?? 0, 1)} s`} />
            </div>
          </div>
        </> : <EmptyState>{t('tr_n_none')}</EmptyState>}
      </Card>
      <Card title={t('tr_episode')}>
        {k && <>
          <KV k={t('tr_n_deliv')} v={k.delivered ? t('tr_yes') : t('tr_no')} tone={k.delivered ? 'good' : 'bad'} />
          <KV k={t('tr_n_result')} v={k.reached ? t('tr_n_landed') : (k.crash || 'timeout')} tone={k.reached ? 'good' : 'bad'} />
          <KV k={t('tr_n_distk')} v={`${fmt(k.start_dist, 0)} m`} />
          <KV k={t('tr_n_clear')} v={`${fmt(k.min_clear, 1)} m`} tone={k.min_clear < 2 ? 'warn' : 'good'} />
          <KV k={t('tr_batt')} v={`${fmt(k.battery_used, 1)} %`} />
          <KV k={t('tr_flight_t')} v={`${fmt(k.time_s, 0)} s`} />
        </>}
      </Card>
      {rec && <NavProfile rec={rec} i={r.i} />}
    </>
  )
}

// ---- fleet: 3 pods, 12 drones, standbys, lockers, stations / フリート ----
interface FFrame { t: number; pods: (number | number[] | null)[][]; drones: (number | string | null)[][]; orders: (number | string)[][]; wind: number; rain: number }
interface FRec {
  pod_checkpoint: string; drone_checkpoint: string; seed: number; frames: FFrame[]
  events: { t: number; what: string; [k: string]: unknown }[]
  kpi: Record<string, number | null | Record<string, number>>; bounds: number[]; zones: Record<string, Rect>; nofly: Rect[]
  stations: number[][]; lockers: number[][]; lines: number[][][]
}
const FSTATE: Record<string, string> = { fly: C.info, hover: C.cyan, standby: C.accent, down: C.danger }

function FleetMap({ rec, i }: { rec: FRec; i: number }) {
  const layer = useRef(new StaticLayer())
  const ref = useCanvas((g, w, h) => {
    const [x0, y0, x1, y1] = rec.bounds
    const s = Math.min(w / (x1 - x0), h / (y1 - y0))
    const ox = (w - (x1 - x0) * s) / 2, oy = (h - (y1 - y0) * s) / 2
    const P = (x: number, y: number): [number, number] => [ox + (x - x0) * s, oy + (y - y0) * s]
    layer.current.draw(g, w, h, rec, (g) => {
      g.fillStyle = C.bg; g.fillRect(0, 0, w, h)
      Object.entries(rec.zones).forEach(([k, r], n) => {
        const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
        g.fillStyle = ZONE_COL[n % 4] + '14'; g.fillRect(a, b, c - a, d - b)
        g.strokeStyle = ZONE_COL[n % 4] + '88'; g.lineWidth = 2; g.strokeRect(a, b, c - a, d - b)
        g.fillStyle = ZONE_COL[n % 4]; g.font = `11px ${FONT_MONO}`; g.fillText(`ZONE ${k}`, a + 4, b + 13)
      })
      g.strokeStyle = C.line; g.lineWidth = 1
      for (const l of rec.lines) { g.beginPath(); l.forEach((p, k) => { const [a, b] = P(p[0], p[1]); if (k) g.lineTo(a, b); else g.moveTo(a, b) }); g.stroke() }
      for (const r of rec.nofly) {
        const [a, b] = P(r.x0, r.y0), [c, d] = P(r.x1, r.y1)
        g.fillStyle = C.danger + '22'; g.fillRect(a, b, c - a, d - b); g.strokeStyle = C.danger; g.strokeRect(a, b, c - a, d - b)
      }
      g.fillStyle = C.accent
      for (const st of rec.stations) { const [a, b] = P(st[0], st[1]); g.fillRect(a - 6, b - 6, 12, 12) }
      g.fillStyle = C.warn
      for (const L of rec.lockers) { const [a, b] = P(L[0], L[1]); g.fillRect(a - 3, b - 3, 6, 6) }
    })
    const f = rec.frames[i]
    if (!f) return
    for (const o of f.orders) {
      const [a, b] = P(Number(o[0]), Number(o[1]))
      g.fillStyle = o[3] === 's' ? C.danger : o[2] === 'f' ? C.warn : C.info
      g.beginPath(); g.arc(a, b, 2.5, 0, Math.PI * 2); g.fill()
    }
    f.pods.forEach((p, k) => {
      const [a, b] = P(Number(p[0]), Number(p[1]))
      const meet = p[5] as number[] | null
      if (meet) { const [mx, my] = P(meet[0], meet[1]); g.strokeStyle = C.fg2; g.lineWidth = 1.5; g.beginPath(); g.arc(mx, my, 7, 0, Math.PI * 2); g.stroke() }
      g.fillStyle = p[4] ? C.danger : C.fg; g.fillRect(a - 6, b - 6, 12, 12)
      const bt = Number(p[2]) / 100
      g.fillStyle = C.grid; g.fillRect(a - 8, b + 8, 16, 3); g.fillStyle = bt < 0.25 ? C.danger : C.accent; g.fillRect(a - 8, b + 8, 16 * bt, 3)
      g.fillStyle = C.fg2; g.font = `10px ${FONT_MONO}`; g.fillText(`P${k + 1}${p[3] ? ' ⚡' : ''}`, a + 8, b - 6)
    })
    for (const d of f.drones) {
      const st = String(d[2])
      if (st === 'docked' || st === 'repair') continue
      const [a, b] = P(Number(d[0]), Number(d[1]))
      g.fillStyle = FSTATE[st] ?? C.mute
      if (st === 'down') { g.strokeStyle = C.danger; g.lineWidth = 2; g.beginPath(); g.moveTo(a - 4, b - 4); g.lineTo(a + 4, b + 4); g.moveTo(a + 4, b - 4); g.lineTo(a - 4, b + 4); g.stroke() }
      else { g.beginPath(); g.arc(a, b, 3.5, 0, Math.PI * 2); g.fill() }
    }
    const docked = f.drones.filter((d) => d[2] === 'docked').length
    g.fillStyle = C.fg2; g.font = `12px ${FONT_MONO}`
    g.fillText(`t ${fmtDur(f.t)}   in pods ${docked}   flying ${f.drones.filter((d) => d[2] === 'fly').length}   hovering ${f.drones.filter((d) => d[2] === 'hover').length}   standby ${f.drones.filter((d) => d[2] === 'standby').length}   parcels waiting ${f.orders.length}   wind ${fmt(f.wind, 1)} m/s   rain ${fmt(f.rain, 0)}`, 10, h - 10)
  }, [rec, i])
  return <canvas ref={ref} className="h-[560px] w-full rounded-lg" aria-label="fleet map" />
}

function ckOpts(list: string[], extra: { value: string; label: string }[]) {
  return [...extra, ...list.slice().reverse().map((c) => ({ value: c, label: c.replace('step_', '').replace('.zip', '').replace(/^0+/, '') + ' steps' }))]
}

function FleetWatch({ pods, drones }: { pods: string[]; drones: string[] }) {
  const t = useT()
  const [pc, setPc] = useState('latest')
  const [dc, setDc] = useState('latest')
  const [seed, setSeed] = useState(7)
  const [rec, setRec] = useState<FRec | null>(null)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState<string | null>(null)
  const r = useReplay(rec as unknown as Rec | null)
  const record = async () => {
    setBusy(true); setErr(null)
    try { setRec(await postJSON<FRec>('/rl/watch', { brain: 'fleet', checkpoint: pc, drone_checkpoint: dc, seed })) }
    catch (e) { setErr(String(e)) } finally { setBusy(false) }
  }
  const now = rec?.frames[r.i]?.t ?? 0
  const recent = rec ? rec.events.filter((e) => e.t <= now).slice(-10).reverse() : []
  const k = rec?.kpi ?? {}
  const num = (x: unknown, d = 1) => (typeof x === 'number' ? fmt(x, d) : '–')
  return (
    <>
      <Card title={t('tr_watch')} className="xl:col-span-3" footer={t('tr_f_legend')}>
        <div className="grid gap-3 md:grid-cols-4">
          <Select label={t('tr_f_pod')} value={pc} options={ckOpts(pods, [{ value: 'latest', label: t('tr_latest') }, { value: 'rule', label: t('tr_f_rule') }])} onChange={setPc} />
          <Select label={t('tr_f_drone')} value={dc} options={ckOpts(drones, [{ value: 'latest', label: t('tr_latest') }, { value: 'rule', label: t('tr_f_rule') }])} onChange={setDc} />
          <Stepper label={t('tr_seed')} value={seed} min={0} max={9999} onChange={setSeed} />
          <div className="flex items-end"><Button variant="primary" busy={busy} onClick={record}>{busy ? t('tr_recording') : t('tr_record')}</Button></div>
        </div>
        {err && <Badge tone="bad">{t('tr_err_msg')}: {err.slice(0, 160)}</Badge>}
        {rec ? <>
          <MissionMapWrap><FleetMap rec={rec} i={r.i} /></MissionMapWrap>
          <div className="flex flex-wrap items-center gap-3">
            <Button size="sm" onClick={() => { if (r.i >= rec.frames.length - 1) r.setI(0); r.setPlay(!r.play) }}>{r.play ? t('tr_pause') : t('tr_play')}</Button>
            <Segmented size="sm" label={t('tr_speed')} value={r.speed} onChange={r.setSpeed}
              options={['1', '5', '20', '100'].map((v) => ({ value: v, label: `${Number(v) * 50}×` }))} />
            <div className="min-w-[240px] flex-1">
              <Slider label={t('tr_time')} value={r.i} min={0} max={rec.frames.length - 1} onChange={(v) => { r.setPlay(false); r.setI(v) }}
                format={(v) => fmtDur(rec.frames[v]?.t ?? 0)} />
            </div>
          </div>
        </> : <EmptyState>{t('tr_f_none')}</EmptyState>}
      </Card>
      <Card title={rec ? t('tr_episode') : t('tr_f_events')}>
        {rec && <>
          <KV k={t('tr_f_pod')} v={rec.pod_checkpoint} />
          <KV k={t('tr_f_drone')} v={rec.drone_checkpoint} />
          <KV k={t('tr_f_deliv')} v={num(k.deliveries_per_h)} />
          <KV k={t('tr_m_lead_k')} v={typeof k.lead_mean_s === 'number' ? `${fmt(k.lead_mean_s / 60, 1)} min` : '–'} />
          <KV k={t('tr_m_c100')} v={num(k.crashes_per_100)} tone={Number(k.crashes_per_100) > 6 ? 'bad' : 'good'} />
          <KV k={t('tr_f_strandk')} v={num(k.stranded, 0)} tone={Number(k.stranded) > 0 ? 'bad' : 'good'} />
          <KV k={t('tr_f_swaps')} v={num(k.swaps, 0)} />
          <KV k={t('tr_f_recov')} v={num(k.recoveries, 0)} />
          <KV k={t('tr_f_repl')} v={num(k.replacements, 0)} />
          <KV k={t('tr_f_charges')} v={num(k.charges, 0)} />
          <KV k={t('tr_f_takeoff')} v={num(k.takeoff_crash, 0)} />
          <KV k={t('tr_f_waiting')} v={num(k.waiting, 0)} />
          <h4 className="mt-2 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('tr_f_events')}</h4>
          <ul className="flex flex-col gap-1 font-mono text-xs text-fg-2">
            {recent.map((e, n) => <li key={n}>{fmtDur(e.t)} · {e.what}{e.why ? ` (${String(e.why)})` : ''}{e.kind ? ` ${String(e.kind)}` : ''}</li>)}
          </ul>
        </>}
      </Card>
    </>
  )
}

function MissionMapWrap({ children }: { children: React.ReactNode }) { return <>{children}</> }

// ---- driving: one route, lane edges, stop lines, lights, cars / 運転 ----
interface DFrame { t: number; p: number[]; v: number; off: number; cars: number[][]; lights: number[] }
interface DRec {
  checkpoint: string; seed: number; route_len: number; traffic: number; frames: DFrame[]
  kpi: { end: string; reached: boolean; stop_violation: number; red_violation: number; centre_s: number; speeding_s: number; progress: number; time_s: number; length: number }
  edges: number[][]; marks: { xy: number[]; yaw: number; w: number; kind: string }[]; limit: number[]
}
const EDGE_COL = [C.danger, C.fg2, C.warn]
const LIGHT_COL = [C.accent, C.warn, C.danger]

function DriveMap({ rec, i }: { rec: DRec; i: number }) {
  const ref = useCanvas((g, w, h) => {
    const f = rec.frames[i]
    if (!f) return
    const view = 70
    const s = Math.min(w, h) / view
    const P = (x: number, y: number): [number, number] => [w / 2 + (x - f.p[0]) * s, h / 2 + (y - f.p[1]) * s]
    g.fillStyle = C.bg; g.fillRect(0, 0, w, h)
    g.fillStyle = '#1e293b'
    for (let k = 1; k < rec.edges.length; k++) {
      const a = rec.edges[k - 1], b = rec.edges[k]
      const pts = [P(a[0], a[1]), P(b[0], b[1]), P(b[3], b[4]), P(a[3], a[4])]
      g.beginPath(); pts.forEach(([x, y], n) => (n ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.fill()
    }
    for (let k = 1; k < rec.edges.length; k++) {
      const a = rec.edges[k - 1], b = rec.edges[k]
      for (const [xi, yi, ki] of [[0, 1, 2], [3, 4, 5]]) {
        const kind = b[6] ? 1 : b[ki]
        g.strokeStyle = EDGE_COL[kind] ?? C.mute; g.lineWidth = kind === 0 ? 3 : 1.5
        g.setLineDash(kind === 2 ? [4, 3] : [])
        const [x1, y1] = P(a[xi], a[yi]), [x2, y2] = P(b[xi], b[yi]); g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke()
      }
    }
    g.setLineDash([])
    rec.marks.forEach((m, n) => {
      const nx = -Math.sin(m.yaw) * m.w / 2, ny = Math.cos(m.yaw) * m.w / 2
      const [ax, ay] = P(m.xy[0] + nx, m.xy[1] + ny), [bx, by] = P(m.xy[0] - nx, m.xy[1] - ny)
      const lt = f.lights[n]
      g.strokeStyle = m.kind === 'stop' ? C.fg : (LIGHT_COL[lt] ?? C.mute); g.lineWidth = 4
      g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke()
      g.fillStyle = m.kind === 'stop' ? C.danger : (LIGHT_COL[lt] ?? C.mute); g.font = `11px ${FONT_MONO}`
      g.fillText(m.kind === 'stop' ? '止まれ' : '●', bx + 4, by)
    })
    const box = (x: number, y: number, yaw: number, col: string) => {
      const [cx, cy] = P(x, y)
      g.save(); g.translate(cx, cy); g.rotate(yaw); g.fillStyle = col; g.fillRect(-1.8 * s, -0.8 * s, 3.6 * s, 1.6 * s); g.restore()
    }
    for (const c of f.cars) box(c[0], c[1], c[2], '#64748b')
    box(f.p[0], f.p[1], f.p[2], C.cyan)
    g.fillStyle = C.fg2; g.font = `12px ${FONT_MONO}`
    g.fillText(`t ${fmt(f.t, 1)} s   ${fmt(f.v * 3.6, 0)} km/h   offset ${fmt(f.off, 2)} m`, 10, h - 10)
  }, [rec, i])
  return <canvas ref={ref} className="h-[460px] w-full rounded-lg" aria-label="driving view" />
}

function DriveSpeed({ rec, i }: { rec: DRec; i: number }) {
  const t = useT()
  const ref = useCanvas((g, w, h) => {
    const n = rec.frames.length
    if (n < 2) return
    const mx = 40
    const X = (k: number) => (k / (n - 1)) * w, Y = (v: number) => h - 4 - (v / mx) * (h - 8)
    g.strokeStyle = C.mute; g.lineWidth = 1; g.beginPath()
    rec.frames.forEach((_, k) => { const L = rec.limit[Math.min(rec.limit.length - 1, Math.floor((k / (n - 1)) * (rec.limit.length - 1)))] ?? 30; if (k) g.lineTo(X(k), Y(L)); else g.moveTo(X(k), Y(L)) }); g.stroke()
    g.strokeStyle = C.cyan; g.lineWidth = 2; g.beginPath()
    rec.frames.forEach((f, k) => { if (k) g.lineTo(X(k), Y(f.v * 3.6)); else g.moveTo(X(k), Y(f.v * 3.6)) }); g.stroke()
    g.strokeStyle = C.fg; g.beginPath(); g.moveTo(X(i), 0); g.lineTo(X(i), h); g.stroke()
  }, [rec, i])
  return <Card title={t('tr_d_speedchart')} className="xl:col-span-3"><canvas ref={ref} className="h-32 w-full" aria-label={t('tr_d_speedchart')} /></Card>
}

function DriveWatch({ ckpts }: { ckpts: string[] }) {
  const t = useT()
  const [ck, setCk] = useState('latest')
  const [seed, setSeed] = useState(7)
  const [rec, setRec] = useState<DRec | null>(null)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState<string | null>(null)
  const r = useReplay(rec as unknown as Rec | null)
  const record = async () => {
    setBusy(true); setErr(null)
    try { setRec(await postJSON<DRec>('/rl/watch', { brain: 'drive', checkpoint: ck, seed })) }
    catch (e) { setErr(String(e)) } finally { setBusy(false) }
  }
  const k = rec?.kpi
  return (
    <>
      <Card title={t('tr_watch')} className="xl:col-span-3" footer={t('tr_d_legend')}>
        <div className="grid gap-3 md:grid-cols-3">
          <Select label={t('tr_ckpt')} value={ck} options={ckOpts(ckpts, [{ value: 'latest', label: t('tr_latest') }, { value: 'scripted', label: t('tr_d_scripted') }])} onChange={setCk} />
          <Stepper label={t('tr_seed')} value={seed} min={0} max={9999} onChange={setSeed} />
          <div className="flex items-end"><Button variant="primary" busy={busy} onClick={record}>{busy ? t('tr_recording') : t('tr_record')}</Button></div>
        </div>
        {err && <Badge tone="bad">{t('tr_err_msg')}: {err.slice(0, 160)}</Badge>}
        {rec ? <>
          <DriveMap rec={rec} i={r.i} />
          <div className="flex flex-wrap items-center gap-3">
            <Button size="sm" onClick={() => { if (r.i >= rec.frames.length - 1) r.setI(0); r.setPlay(!r.play) }}>{r.play ? t('tr_pause') : t('tr_play')}</Button>
            <Segmented size="sm" label={t('tr_speed')} value={r.speed} onChange={r.setSpeed}
              options={['1', '5', '20', '100'].map((v) => ({ value: v, label: `${Number(v) * 2}×` }))} />
            <div className="min-w-[240px] flex-1">
              <Slider label={t('tr_time')} value={r.i} min={0} max={rec.frames.length - 1} onChange={(v) => { r.setPlay(false); r.setI(v) }}
                format={(v) => `${fmt(rec.frames[v]?.t ?? 0, 1)} s`} />
            </div>
          </div>
        </> : <EmptyState>{t('tr_d_none')}</EmptyState>}
      </Card>
      <Card title={t('tr_episode')}>
        {k && <>
          <KV k={t('tr_ckpt')} v={rec.checkpoint} />
          <KV k={t('tr_d_end')} v={k.end || '–'} tone={k.reached ? 'good' : 'bad'} />
          <KV k={t('tr_n_distk')} v={`${fmt(k.progress, 0)} / ${fmt(k.length, 0)} m`} />
          <KV k={t('tr_d_viol')} v={`${k.stop_violation} / ${k.red_violation}`} tone={k.stop_violation + k.red_violation ? 'bad' : 'good'} />
          <KV k={t('tr_d_centre')} v={`${fmt(k.centre_s, 1)} s`} tone={k.centre_s > 0 ? 'warn' : 'good'} />
          <KV k={t('tr_d_speeding')} v={`${fmt(k.speeding_s, 1)} s`} tone={k.speeding_s > 0 ? 'warn' : 'good'} />
          <KV k={t('tr_d_speedk')} v={`${fmt(k.progress / Math.max(k.time_s, 1) * 3.6, 0)} km/h`} />
        </>}
      </Card>
      {rec && <DriveSpeed rec={rec} i={r.i} />}
    </>
  )
}

export default function TrainingView() {
  const t = useT()
  const [brain, setBrain] = useState<Brain>('fleet')
  const { data } = usePoll<RlState>(`/rl/state?brain=${brain}`, 5000)
  const st = data && (data as RlState & { _b?: string }) ? data : null
  return (
    <div className="flex flex-col gap-4">
      <SectionTitle sub={t('tr_sub')}>{t('tr_title')}</SectionTitle>
      <Segmented label={t('tr_brain')} value={brain} onChange={setBrain}
        options={[{ value: 'fleet', label: t('tr_b_fleet') }, { value: 'drive', label: t('tr_b_drive') },
          { value: 'nav', label: t('tr_b_nav') }, { value: 'mission', label: t('tr_b_mission') + t('tr_retired') },
          { value: 'flight', label: t('tr_b_flight') + t('tr_retired') }]} />
      <Grid cols={4}>
        <Trainer key={`t-${brain}`} st={st} brain={brain} />
        <Learning rows={(st?.progress ?? []) as unknown as Record<string, number>[]} brain={brain} rule={st?.rule} />
        {brain === 'fleet' ? <FleetWatch pods={st?.checkpoints ?? []} drones={st?.drone_checkpoints ?? []} />
          : brain === 'drive' ? <DriveWatch ckpts={st?.checkpoints ?? []} />
          : brain === 'flight' ? <Watch ckpts={st?.checkpoints ?? []} />
          : brain === 'nav' ? <NavWatch ckpts={st?.checkpoints ?? []} /> : <MissionWatch ckpts={st?.checkpoints ?? []} />}
      </Grid>
    </div>
  )
}
