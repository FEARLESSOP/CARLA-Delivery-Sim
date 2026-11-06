import { useMemo, useState, type ReactNode } from 'react'
import { ArrowDown, ArrowUp, Download } from 'lucide-react'
import '@/i18n/stats'
import { useT, type TFunc } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { clamp, fmt, fmtDur } from '@/lib/fmt'
import { num, useTelemetry } from '@/lib/telemetry'
import { cn } from '@/lib/utils'
import { Badge, Button, Card, EmptyState, Segmented, Stat, type Tone } from '@/components/ui'

// Statistics: the current session's counters (telemetry), a bar chart of
// crashes / out-of-lane per saved session and the saved-session list
// (GET /sessions every 10 s while shown; dashboard.py).
// 統計ページ：現在のセッション、履歴グラフ、保存済みセッション一覧。

/** One summary.json from logs/<date>/<time>/ (driving) or a delivery run.
 *  Loosely typed: older runs lack fields. / 古い記録は項目が欠ける */
interface Session {
  date?: string
  time?: string
  map?: string
  current?: boolean
  trip_sec?: number | null
  trip_km?: number | null
  crashes?: number | null
  out_of_lane?: number | null
  red_light_violations?: number | null
  hard_brakes?: number | null
  speed_compliance?: number | null
  sim_seconds?: number | null
  vans?: number | null
}

type Kind = 'drive' | 'delivery'
interface Row {
  key: string
  s: Session
  kind: Kind
  ts: number          // ms since epoch, 0 = unknown
  day: string         // YYYY-MM-DD
  clock: string       // HH:MM:SS
  map: string
  dur: number | null
  km: number | null
  crash: number
  lane: number | null
  rlv: number | null
  hb: number | null
  comp: number | null
}

const n0 = (v: unknown): number | null => (typeof v === 'number' && isFinite(v) ? v : null)

/** Driving runs: date "2026-10-09", time "15-39-20"; delivery runs: date
 *  "delivery", time "20261106_103120". / 2 種類の日時形式を解釈 */
function toRow(s: Session, i: number): Row {
  const delivery = s.date === 'delivery' || s.vans != null
  let day = s.date ?? '', clock = (s.time ?? '').replace(/-/g, ':')
  const m = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(s.time ?? '')
  if (m) { day = `${m[1]}-${m[2]}-${m[3]}`; clock = `${m[4]}:${m[5]}:${m[6]}` }
  const ts = /^\d{4}-\d{2}-\d{2}$/.test(day) && /^\d{2}:\d{2}(:\d{2})?$/.test(clock)
    ? new Date(`${day}T${clock}`).getTime() || 0 : 0
  const comp = n0(s.speed_compliance)
  return {
    key: `${s.date}/${s.time}/${i}`, s, kind: delivery ? 'delivery' : 'drive', ts, day, clock,
    map: (s.map ?? '').split('/').pop() || '-',
    dur: n0(s.trip_sec) ?? (delivery ? n0(s.sim_seconds) : 0),
    km: n0(s.trip_km) ?? (delivery ? null : 0),
    crash: n0(s.crashes) ?? 0,
    lane: n0(s.out_of_lane) ?? (delivery ? null : 0),
    rlv: n0(s.red_light_violations) ?? (delivery ? null : 0),
    hb: n0(s.hard_brakes) ?? (delivery ? null : 0),
    // the old page showed 100 % when a driving run had no value
    comp: comp ?? (delivery ? null : 100),
  }
}

/* ------------------------------------------------------ current session */
function CurrentSession() {
  const t = useT()
  const s = useTelemetry()
  const live = Boolean(s?.connected && s?.hero_found)
  const crash = num(s, 'collision_count'), lane = num(s, 'lane_invasions')
  const rlv = num(s, 'red_light_violations'), hb = num(s, 'hard_brake_count')
  const comp = clamp(num(s, 'speed_compliance') || 100, 0, 100)
  const tiles: { k: string; v: string; unit?: string; tone: Tone }[] = [
    { k: 'tile_crash', v: String(crash), tone: crash > 0 ? 'bad' : 'neutral' },
    { k: 'tile_lane', v: String(lane), tone: lane > 0 ? 'info' : 'neutral' },
    { k: 'tile_rlv', v: String(rlv), tone: rlv > 0 ? 'warn' : 'neutral' },
    { k: 'tile_hb', v: String(hb), tone: hb > 0 ? 'warn' : 'neutral' },
    { k: 'tile_dist', v: fmt(num(s, 'trip_km'), 2), unit: 'km', tone: 'good' },
    { k: 'tile_dur', v: fmtDur(num(s, 'trip_sec')), tone: 'neutral' },
    { k: 'tile_avg', v: fmt(num(s, 'avg_kmh'), 0), unit: 'km/h', tone: 'neutral' },
    { k: 'tile_comp', v: fmt(comp, 0), unit: '%', tone: comp > 90 ? 'good' : comp > 70 ? 'warn' : 'bad' },
  ]
  return (
    <Card title={t('stats_session')}
      actions={live ? <Badge tone="good" dot>{t('badge_live')}</Badge> : null}
      footer={live ? t('st_session_h') : t('st_no_live')}>
      <div className="grid grid-cols-2 gap-3 sm:grid-cols-4 2xl:grid-cols-8">
        {tiles.map((x) => (
          <div key={x.k} className="rounded-lg border border-line/70 bg-surface/60 px-3 py-2.5">
            <Stat label={t(x.k)} value={x.v} unit={x.unit} tone={x.tone} size="md" />
          </div>
        ))}
      </div>
    </Card>
  )
}

/* -------------------------------------------------------- history chart */
const PAD_L = 30, PAD_B = 32, PAD_T = 18

function HistoryChart({ rows }: { rows: Row[] }) {
  const t = useT()
  const [hover, setHover] = useState<{ i: number; x: number; y: number; w: number } | null>(null)
  // latest 20, oldest -> newest / 最新 20 件を古い順に
  const list = useMemo(() => [...rows].sort((a, b) => b.ts - a.ts).slice(0, 20).reverse(), [rows])
  const hi = hover?.i ?? -1
  const ref = useCanvas((ctx, w, h) => {
    ctx.strokeStyle = C.grid
    ctx.lineWidth = 1
    if (!list.length) return
    const maxV = Math.max(1, ...list.map((r) => Math.max(r.crash, r.lane ?? 0)))
    const plotH = h - PAD_B - PAD_T
    // grid lines with values / 目盛り
    ctx.font = `10px ${FONT_MONO}`
    ctx.fillStyle = C.mute
    ctx.textAlign = 'right'
    for (const f of [0, 0.5, 1]) {
      const yy = Math.round(h - PAD_B - f * plotH) + 0.5
      ctx.beginPath(); ctx.moveTo(PAD_L, yy); ctx.lineTo(w - 4, yy); ctx.stroke()
      ctx.fillText(String(Math.round(f * maxV)), PAD_L - 5, yy + 3)
    }
    const gw = (w - PAD_L - 8) / list.length
    const bw = Math.max(4, Math.min(20, (gw - 12) / 2))
    let lastDay = '', lastDayX = -1e9
    const every = Math.max(1, Math.ceil(38 / gw))   // label spacing / ラベル間隔
    list.forEach((r, i) => {
      const gx = PAD_L + i * gw + gw / 2
      if (hi === i) {
        ctx.fillStyle = 'rgba(148,163,184,0.10)'
        ctx.fillRect(PAD_L + i * gw, PAD_T - 6, gw, plotH + 6)
      }
      const pairs: [number, string, number][] = [[r.crash, C.s2, gx - bw - 1], [r.lane ?? 0, C.s1, gx + 1]]
      for (const [v, col, x] of pairs) {
        const bh = (v / maxV) * plotH
        ctx.fillStyle = col
        ctx.beginPath()
        ctx.roundRect(x, h - PAD_B - bh, bw, Math.max(bh, v > 0 ? 3 : 0), [4, 4, 0, 0])
        ctx.fill()
        if (v > 0 && gw > 34) {
          ctx.fillStyle = C.fg2
          ctx.textAlign = 'center'
          ctx.fillText(String(v), x + bw / 2, h - PAD_B - bh - 4)
        }
      }
      ctx.fillStyle = C.mute
      ctx.textAlign = 'center'
      if ((list.length - 1 - i) % every === 0) ctx.fillText(r.clock.slice(0, 5), gx, h - PAD_B + 13)
      if (r.day !== lastDay && gx - lastDayX >= 40) { ctx.fillText(r.day.slice(5), gx, h - PAD_B + 26); lastDay = r.day; lastDayX = gx }
    })
  }, [list, hi])

  const hv = hover ? list[hover.i] : null
  return (
    <Card title={t('stats_history')} footer={t('hist_hint')}>
      {list.length ? (
        <div className="relative">
          <canvas ref={ref} className="block h-64 w-full 2xl:h-80" role="img"
            aria-label={`${t('stats_history')}: ${list.map((r) => `${r.day} ${r.clock} ${t('tile_crash')} ${r.crash}, ${t('tile_lane')} ${r.lane ?? '–'}`).join('; ')}`}
            onMouseMove={(e) => {
              const b = e.currentTarget.getBoundingClientRect()
              const gw = (b.width - PAD_L - 8) / list.length
              const i = Math.floor((e.clientX - b.left - PAD_L) / gw)
              setHover(i >= 0 && i < list.length ? { i, x: e.clientX - b.left, y: e.clientY - b.top, w: b.width } : null)
            }}
            onMouseLeave={() => setHover(null)} />
          {hv && hover && (
            <div className="pointer-events-none absolute z-10 min-w-44 rounded-lg border border-line bg-card-2 px-3 py-2 text-xs shadow-lg"
              style={{ left: hover.x, top: Math.max(0, hover.y - 80),
                transform: hover.x > hover.w / 2 ? 'translateX(calc(-100% - 12px))' : 'translateX(12px)' }}>
              <div className="num mb-0.5 font-medium text-fg">{hv.day} {hv.clock}</div>
              <div className="text-fg-mute">{hv.map}</div>
              <div className="mt-1 flex items-center gap-2"><i className="h-2 w-2 rounded-sm bg-s2" />{t('tile_crash')}<b className="num ml-auto text-fg">{hv.crash}</b></div>
              <div className="flex items-center gap-2"><i className="h-2 w-2 rounded-sm bg-s1" />{t('tile_lane')}<b className="num ml-auto text-fg">{hv.lane ?? '–'}</b></div>
            </div>
          )}
          <div className="mt-2 flex flex-wrap gap-x-4 gap-y-1 text-xs text-fg-mute">
            <span className="inline-flex items-center gap-1.5"><i className="h-2.5 w-2.5 rounded-sm bg-s2" />{t('tile_crash')} (1)</span>
            <span className="inline-flex items-center gap-1.5"><i className="h-2.5 w-2.5 rounded-sm bg-s1" />{t('tile_lane')} (2)</span>
          </div>
        </div>
      ) : <EmptyState>{t('st_hist_empty')}</EmptyState>}
    </Card>
  )
}

/* -------------------------------------------------------- session table */
type SortKey = 'ts' | 'map' | 'dur' | 'km' | 'crash' | 'lane' | 'rlv' | 'hb' | 'comp'
const COLS: { k: SortKey; label: string; num?: boolean }[] = [
  { k: 'ts', label: 'th_time' }, { k: 'map', label: 'th_map' }, { k: 'dur', label: 'th_dur', num: true },
  { k: 'km', label: 'th_dist', num: true }, { k: 'crash', label: 'th_crash', num: true },
  { k: 'lane', label: 'th_lane', num: true }, { k: 'rlv', label: 'th_rlv', num: true },
  { k: 'hb', label: 'th_hb', num: true }, { k: 'comp', label: 'th_comp', num: true },
]

function cmp(a: Row, b: Row, k: SortKey): number {
  if (k === 'map') return a.map.localeCompare(b.map)
  return (a[k] ?? -1) - (b[k] ?? -1)
}

const PAGE = 60

function SessionTable({ rows }: { rows: Row[] }) {
  const t = useT()
  const [sort, setSort] = useState<{ k: SortKey; desc: boolean }>({ k: 'ts', desc: true })
  const [shown, setShown] = useState(PAGE)
  const sorted = useMemo(() => {
    const dir = sort.desc ? -1 : 1
    return [...rows].sort((a, b) => dir * cmp(a, b, sort.k) || b.ts - a.ts)
  }, [rows, sort])
  const byDay = sort.k === 'ts'
  const out: ReactNode[] = []
  let lastDay: string | null = null
  for (const r of sorted.slice(0, shown)) {
    if (byDay && r.day !== lastDay) {
      lastDay = r.day
      out.push(
        <tr key={`d-${r.key}`} className="bg-surface/70">
          <th colSpan={COLS.length} scope="colgroup" className="num px-3 py-1.5 text-left text-xs font-semibold text-fg-mute">
            {r.day || t('st_unknown_date')}
          </th>
        </tr>,
      )
    }
    out.push(<SessionRow key={r.key} r={r} t={t} />)
  }
  return (
    <div className="-mx-4 overflow-x-auto px-4">
      <table className="w-full min-w-[720px] border-collapse text-sm">
        <thead>
          <tr className="border-b border-line">
            {COLS.map((c) => {
              const on = sort.k === c.k
              return (
                <th key={c.k} scope="col" aria-sort={on ? (sort.desc ? 'descending' : 'ascending') : 'none'}
                  className={cn('px-2 py-2 font-medium text-fg-mute', c.num ? 'text-right' : 'text-left')}>
                  <button type="button" title={t('st_sort_by', { c: t(c.label) })}
                    onClick={() => setSort((s) => ({ k: c.k, desc: s.k === c.k ? !s.desc : c.k !== 'map' }))}
                    className={cn('inline-flex items-center gap-1 rounded px-1 text-xs whitespace-nowrap hover:text-fg', on && 'text-fg')}>
                    {t(c.label)}
                    {on ? (sort.desc ? <ArrowDown className="h-3 w-3" aria-hidden /> : <ArrowUp className="h-3 w-3" aria-hidden />)
                      : <span className="w-3" aria-hidden />}
                  </button>
                </th>
              )
            })}
          </tr>
        </thead>
        <tbody>{out}</tbody>
      </table>
      {sorted.length > shown && (
        <div className="mt-3 flex justify-center">
          <Button size="sm" onClick={() => setShown((n) => n + PAGE)}>{t('st_more', { n: Math.min(PAGE, sorted.length - shown) })}</Button>
        </div>
      )}
    </div>
  )
}

function SessionRow({ r, t }: { r: Row; t: TFunc }) {
  const cell = 'px-2 py-1.5'
  const dash = (v: number | null, d = 0) => (v === null ? '–' : fmt(v, d))
  return (
    <tr className={cn('border-b border-line/50 hover:bg-card-2/60', r.s.current && 'bg-accent/5')}>
      <td className={cn(cell, 'num whitespace-nowrap')}>
        {r.clock || '-'}
        {r.s.current && <Badge tone="good" dot className="ml-2">{t('badge_live')}</Badge>}
        {r.kind === 'delivery' && <Badge tone="info" className="ml-2">{t('st_delivery')}</Badge>}
      </td>
      <td className={cn(cell, 'max-w-48 truncate')} title={r.s.map}>{r.map}</td>
      <td className={cn(cell, 'num text-right')}>{r.dur === null ? '–' : fmtDur(r.dur)}</td>
      <td className={cn(cell, 'num text-right')}>{dash(r.km, 2)}</td>
      <td className={cn(cell, 'num text-right', r.crash > 0 && 'font-semibold text-danger')}>{r.crash}</td>
      <td className={cn(cell, 'num text-right', (r.lane ?? 0) > 0 && 'font-semibold text-warn')}>{dash(r.lane)}</td>
      <td className={cn(cell, 'num text-right')}>{dash(r.rlv)}</td>
      <td className={cn(cell, 'num text-right')}>{dash(r.hb)}</td>
      <td className={cn(cell, 'num text-right', r.comp !== null && r.comp < 70 && 'text-warn')}>{r.comp === null ? '–' : `${fmt(r.comp, 0)}%`}</td>
    </tr>
  )
}

/* ---------------------------------------------------------------- page */
type KindFilter = 'all' | Kind

export default function StatsView() {
  const t = useT()
  const { data, error } = usePoll<Session[]>('/sessions', 10000)
  const [kind, setKind] = useState<KindFilter>('all')
  const all = useMemo(() => (Array.isArray(data) ? data.map(toRow) : []), [data])
  const rows = useMemo(() => (kind === 'all' ? all : all.filter((r) => r.kind === kind)), [all, kind])
  const hasDelivery = all.some((r) => r.kind === 'delivery')
  const totKm = rows.reduce((a, r) => a + (r.km ?? 0), 0)
  const totSec = rows.reduce((a, r) => a + (r.dur ?? 0), 0)
  const totCrash = rows.reduce((a, r) => a + r.crash, 0)

  return (
    <div className="space-y-5">
      <CurrentSession />
      <div className="grid gap-5 xl:grid-cols-[minmax(0,2fr)_minmax(0,3fr)]">
        <HistoryChart rows={rows} />
        <Card title={t('stats_sessions')}
          actions={
            <a href="/download" title={t('st_dl_csv')}
              className="inline-flex h-8 items-center gap-1.5 rounded-lg border border-line px-2.5 text-xs text-fg-2 hover:bg-muted">
              <Download className="h-3.5 w-3.5" aria-hidden />{t('csv')}
            </a>
          }
          footer={t('st_sessions_h')}>
          <div className="flex flex-wrap items-center justify-between gap-3">
            {hasDelivery ? (
              <Segmented size="sm" label={t('st_kind')} value={kind} onChange={setKind}
                options={[{ value: 'all', label: t('st_kind_all') }, { value: 'drive', label: t('st_kind_drive') },
                  { value: 'delivery', label: t('st_kind_delivery') }]} />
            ) : <span />}
            {rows.length > 0 && (
              <dl className="flex flex-wrap gap-x-5 gap-y-1 text-xs">
                <div className="text-fg-mute">{t('st_count', { n: rows.length })}</div>
                <div><dt className="inline text-fg-mute">{t('st_total_km')} </dt><dd className="num inline text-fg">{fmt(totKm, 1)} km</dd></div>
                <div><dt className="inline text-fg-mute">{t('st_total_time')} </dt><dd className="num inline text-fg">{fmtDur(totSec)}</dd></div>
                <div><dt className="inline text-fg-mute">{t('st_total_crash')} </dt><dd className="num inline text-fg">{totCrash}</dd></div>
              </dl>
            )}
          </div>
          {data === null
            ? <EmptyState>{error ? t('st_error') : t('st_loading')}</EmptyState>
            : rows.length ? <SessionTable rows={rows} /> : <EmptyState>{t('stats_empty')}</EmptyState>}
        </Card>
      </div>
    </div>
  )
}
