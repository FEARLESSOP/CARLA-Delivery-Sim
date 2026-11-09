// The Mini Pods page's right column: the battery gauge, speed and distance to
// the destination with canvas sparklines, ETA and trip counters, and the mini
// map (the delivery map's roads round the pod, its track and planned path,
// its destination, its hub, the hub's distribution and meeting points).
// 右列：電池ゲージ・速度/残り距離の推移・到着予想・小地図
import { useEffect, useState, type ReactNode } from 'react'
import { Clock, Gauge, MapPin, Route, Zap } from 'lucide-react'
import { getJSON } from '@/lib/api'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Card } from '@/components/ui'
import { MC, battTone, podColor } from '@/views/delivery/shared'
import type { DeliveryMap, DeliveryState, XY, ZoneSet } from '@/views/delivery/types'
import { OUT, distLeft, etaS, f, hubOf, mmss, podLabel, xyOf } from './data'
import { trackOf } from './history'
import type { MiniPod } from './types'

/** Semicircle battery gauge (0-100 %), ticks every 10. / 半円の電池ゲージ */
function BatteryGauge({ value }: { value: number }) {
  const t = useT()
  const v = Math.max(0, Math.min(100, value))
  const tone = battTone(v)
  const R = 80, cx = 100, cy = 96
  const pt = (pct: number, r = R) => {
    const a = Math.PI * (1 - pct / 100)
    return [cx + r * Math.cos(a), cy - r * Math.sin(a)] as const
  }
  const arc = (pct: number) => { const [x, y] = pt(pct); return `M ${cx - R} ${cy} A ${R} ${R} 0 0 1 ${x.toFixed(2)} ${y.toFixed(2)}` }
  const col = tone === 'good' ? 'stroke-accent' : tone === 'warn' ? 'stroke-warn' : 'stroke-danger'
  return (
    <svg viewBox="0 0 200 118" className="mx-auto block w-full max-w-[300px]" role="img" aria-label={`${t('mp_batt_level')} ${Math.round(v)} %`}>
      <path d={arc(100)} className="fill-none stroke-muted" strokeWidth={12} strokeLinecap="round" />
      {v > 0.5 && <path d={arc(v)} className={cn('fill-none transition-[d] duration-500', col)} strokeWidth={12} strokeLinecap="round" />}
      {Array.from({ length: 11 }, (_, i) => {
        const [x0, y0] = pt(i * 10, R - 13), [x1, y1] = pt(i * 10, R - (i % 5 === 0 ? 21 : 17))
        return <line key={i} x1={x0} y1={y0} x2={x1} y2={y1} className="stroke-line-strong" strokeWidth={1.2} />
      })}
      <text x={cx - R} y={cy + 16} textAnchor="middle" className="fill-fg-mute text-[9px]">0</text>
      <text x={cx + R} y={cy + 16} textAnchor="middle" className="fill-fg-mute text-[9px]">100</text>
      <text x={cx} y={cy - 18} textAnchor="middle" className="num fill-fg text-[30px] font-semibold">{Math.round(v)}%</text>
      <text x={cx} y={cy + 2} textAnchor="middle" className="fill-fg-mute text-[10px]">{t('mp_batt_level')}</text>
    </svg>
  )
}

/** A canvas sparkline with three y labels (min / mid / max). / 推移線 */
function Spark({ data, color, label }: { data: number[]; color: string; label: string }) {
  const ref = useCanvas((ctx, w, h) => {
    const vals = data.filter((v) => Number.isFinite(v))
    const padL = 26, padB = 4, padT = 4
    let a = Math.min(...vals, 0), b = Math.max(...vals, 1)
    if (!vals.length) { a = 0; b = 1 }
    const span = b - a
    const nice = span > 0 ? Math.pow(10, Math.floor(Math.log10(span))) / 2 : 1
    b = Math.ceil(b / nice) * nice
    a = Math.floor(a / nice) * nice
    ctx.font = `9px ${FONT_MONO}`
    ctx.fillStyle = C.mute
    ctx.textAlign = 'right'
    ctx.textBaseline = 'middle'
    for (const k of [0, 0.5, 1]) {
      const y = padT + (h - padT - padB) * (1 - k)
      const v = a + (b - a) * k
      ctx.fillText(b - a < 5 ? v.toFixed(1) : String(Math.round(v)), padL - 5, y)
      ctx.strokeStyle = C.grid
      ctx.lineWidth = 1
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w, y); ctx.stroke()
    }
    if (vals.length < 2) return
    const n = data.length
    const X = (i: number) => padL + ((w - padL) * i) / Math.max(n - 1, 1)
    const Y = (v: number) => padT + (h - padT - padB) * (1 - (v - a) / Math.max(b - a, 1e-6))
    const g = ctx.createLinearGradient(0, padT, 0, h)
    g.addColorStop(0, `${color}55`)
    g.addColorStop(1, `${color}00`)
    ctx.beginPath()
    let started = false, firstX = 0, lastX = 0
    data.forEach((v, i) => {
      if (!Number.isFinite(v)) return
      if (!started) { ctx.moveTo(X(i), Y(v)); started = true; firstX = X(i) } else ctx.lineTo(X(i), Y(v))
      lastX = X(i)
    })
    ctx.strokeStyle = color
    ctx.lineWidth = 1.8
    ctx.lineJoin = 'round'
    ctx.stroke()
    ctx.lineTo(lastX, h)
    ctx.lineTo(firstX, h)
    ctx.closePath()
    ctx.fillStyle = g
    ctx.fill()
  }, [data.length, data[data.length - 1], color])
  return <canvas ref={ref} className="h-16 w-full" role="img" aria-label={label} />
}

function MetricCard({ icon, title, value, unit, sub, children }: {
  icon: ReactNode; title: string; value: ReactNode; unit?: string; sub?: ReactNode; children?: ReactNode
}) {
  return (
    <section className="flex min-w-0 flex-col gap-1 rounded-[var(--radius-card)] border border-line bg-card/80 p-3 shadow-[var(--shadow-card)]">
      <h3 className="flex items-center gap-1.5 text-xs text-fg-mute"><span className="text-accent" aria-hidden>{icon}</span>{title}</h3>
      <div className="num text-2xl font-semibold leading-tight text-fg">{value}{unit && <span className="ml-1 text-sm font-normal text-fg-mute">{unit}</span>}</div>
      {sub && <div className="text-[11px] text-fg-mute">{sub}</div>}
      {children}
    </section>
  )
}

/** Battery gauge card with the Live badge. / 電池 */
export function BatteryCard({ p, live }: { p: MiniPod; live: boolean }) {
  const t = useT()
  return (
    <Card title={<span className="flex items-center gap-1.5"><Zap className="h-3.5 w-3.5 text-accent" aria-hidden />{t('mp_rt')}</span>}
      actions={<Badge tone={live ? 'good' : 'neutral'} dot>{live ? t('mp_live') : t('mp_paused')}</Badge>}>
      <BatteryGauge value={p.battery ?? 0} />
    </Card>
  )
}

const km = (m: number | null) => (m == null ? '–' : m >= 1000 ? f(m / 1000, 2) : f(m, 0))
const kmUnit = (m: number | null) => (m != null && m >= 1000 ? 'km' : 'm')

export function SpeedDist({ p, hist }: { p: MiniPod; hist: (k: string) => number[] }) {
  const t = useT()
  const d = distLeft(p)
  const sp = hist('_speed').map((v) => v * 3.6)
  return (
    <div className="grid grid-cols-2 gap-3">
      <MetricCard icon={<Gauge className="h-3.5 w-3.5" />} title={t('mp_speed')} value={p.speed == null ? '–' : f(p.speed * 3.6, 1)} unit="km/h"
        sub={`${f(p.speed, 2)} m/s`}>
        <Spark data={sp} color={C.s5} label={t('mp_speed_trend')} />
      </MetricCard>
      <MetricCard icon={<Route className="h-3.5 w-3.5" />} title={t('mp_dist_dest')} value={km(d)} unit={kmUnit(d)}
        sub={d == null ? t('mp_no_dest') : p.path?.length ? t('mp_along_path') : t('mp_straight')}>
        <Spark data={hist('_dist')} color={C.s1} label={t('mp_dist_trend')} />
      </MetricCard>
    </div>
  )
}

export function EtaTrips({ p }: { p: MiniPod }) {
  const t = useT()
  const eta = etaS(p)
  return (
    <div className="grid grid-cols-2 gap-3">
      <MetricCard icon={<Clock className="h-3.5 w-3.5" />} title={t('mp_eta')} value={mmss(eta)}
        sub={eta == null ? (OUT.includes(p.status) ? t('mp_no_dest') : t('mp_eta_na')) : t(p.status === 'returning' ? 'mp_eta_hub' : 'mp_eta_door')} />
      <MetricCard icon={<Route className="h-3.5 w-3.5" />} title={t('mp_trips')} value={p.trips ?? 0}
        sub={t('mp_km_total', { km: f(p.km, 2) })} />
    </div>
  )
}

/* ------------------------------------------------------------ mini map */
let MAP_CACHE: { name: string; roads: Path2D; lanes: Path2D | null; blds: Path2D } | null = null

function buildMap(name: string, m: DeliveryMap) {
  const roads = new Path2D()
  for (const r of m.roads ?? []) r.forEach(([x, y], i) => (i ? roads.lineTo(x, y) : roads.moveTo(x, y)))
  let lanes: Path2D | null = null
  if (m.lanes?.length) {
    lanes = new Path2D()
    for (const [kind, pts] of m.lanes) {
      if (kind !== 'd' || pts.length < 6) continue
      lanes.moveTo(pts[0], pts[1])
      for (let i = 2; i + 1 < pts.length; i += 2) lanes.lineTo(pts[i], pts[i + 1])
      lanes.closePath()
    }
  }
  const blds = new Path2D()
  for (const [x, y, hx, hy, yaw] of m.buildings ?? []) {
    const a = (yaw * Math.PI) / 180, c = Math.cos(a), s = Math.sin(a)
    const corners = [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]].map(([u, v]) => [x + u * c - v * s, y + u * s + v * c])
    corners.forEach(([px, py], i) => (i ? blds.lineTo(px, py) : blds.moveTo(px, py)))
    blds.closePath()
  }
  return { name, roads, lanes, blds }
}

/** /delivery/map once per map (the state names it). / 地図を一度だけ取得 */
function useMiniMap(name: string | undefined) {
  const [m, setM] = useState(MAP_CACHE && MAP_CACHE.name === name ? MAP_CACHE : null)
  useEffect(() => {
    if (!name) return
    if (MAP_CACHE && MAP_CACHE.name === name) { setM(MAP_CACHE); return }
    let alive = true
    getJSON<DeliveryMap>('/delivery/map').then((mp) => {
      if (!alive || !mp || !Array.isArray(mp.roads)) return
      MAP_CACHE = buildMap(name, mp)
      setM(MAP_CACHE)
    }).catch(() => {})
    return () => { alive = false }
  }, [name])
  return m
}

export function MiniMap({ p, st, zones }: { p: MiniPod; st: DeliveryState; zones: ZoneSet }) {
  const t = useT()
  const mp = useMiniMap(st.map)
  const hub = hubOf(st, p)
  const dest = xyOf(p.dest)
  const track = trackOf(p.id)
  const distPts: XY[] = p.hub != null ? (zones.dist[String(p.hub)] ?? []).map((q) => [q.x, q.y]) : []
  const meetPts: XY[] = p.hub != null ? (zones.meet[String(p.hub)] ?? []).map((q) => [q.x, q.y]) : []
  if (hub?.dist && !distPts.some(([x, y]) => Math.hypot(x - hub.dist![0], y - hub.dist![1]) < 1)) distPts.push(hub.dist)
  if (hub?.meet && !meetPts.some(([x, y]) => Math.hypot(x - hub.meet![0], y - hub.meet![1]) < 1)) meetPts.push(hub.meet)
  const hubCol = p.hub != null ? podColor(p.hub) : MC.pod
  const pathKey = (p.path ?? []).length
  const ref = useCanvas((ctx, w, h) => {
    ctx.fillStyle = C.bg
    ctx.fillRect(0, 0, w, h)
    // frame: the pod, its destination, its hub and the hub's points, at least 90 m across
    // 表示範囲：ポッド・目的地・ハブ・ハブの地点（最低 90 m）
    const pts: XY[] = [[p.x, p.y]]
    if (dest) pts.push(dest)
    if (hub) pts.push([hub.x, hub.y])
    for (const q of p.path ?? []) pts.push(q)
    let x0 = Math.min(...pts.map((q) => q[0])), x1 = Math.max(...pts.map((q) => q[0]))
    let y0 = Math.min(...pts.map((q) => q[1])), y1 = Math.max(...pts.map((q) => q[1]))
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2
    const span = Math.max(90, (x1 - x0) * 1.35, (y1 - y0) * 1.35 * (w / h))
    const k = w / span
    x0 = cx - w / 2 / k; y0 = cy - h / 2 / k
    const X = (x: number) => (x - x0) * k, Y = (y: number) => (y - y0) * k
    if (mp) {
      ctx.save()
      ctx.translate(-x0 * k, -y0 * k)
      ctx.scale(k, k)
      ctx.fillStyle = MC.oldBld
      ctx.fill(mp.blds)
      if (mp.lanes) { ctx.fillStyle = MC.oldRoad; ctx.fill(mp.lanes) }
      ctx.strokeStyle = mp.lanes ? C.axis : MC.oldRoad
      ctx.lineWidth = (mp.lanes ? 1 : 5) / k
      ctx.lineJoin = 'round'
      ctx.stroke(mp.roads)
      ctx.restore()
    }
    // travelled track / 走行軌跡
    if (track.length > 1) {
      ctx.strokeStyle = C.accent; ctx.lineWidth = 2; ctx.setLineDash([])
      ctx.beginPath()
      track.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))))
      ctx.stroke()
    }
    // planned path ahead / 予定経路
    if (p.path?.length) {
      ctx.strokeStyle = hubCol; ctx.lineWidth = 1.8; ctx.setLineDash([5, 4])
      ctx.beginPath(); ctx.moveTo(X(p.x), Y(p.y))
      for (const [x, y] of p.path) ctx.lineTo(X(x), Y(y))
      ctx.stroke(); ctx.setLineDash([])
    } else if (dest && OUT.includes(p.status)) {
      ctx.setLineDash([4, 4]); ctx.strokeStyle = C.mute; ctx.lineWidth = 1
      ctx.beginPath(); ctx.moveTo(X(p.x), Y(p.y)); ctx.lineTo(X(dest[0]), Y(dest[1])); ctx.stroke(); ctx.setLineDash([])
    }
    const disc = (x: number, y: number, r: number, fill: string, glyph: string) => {
      ctx.beginPath(); ctx.arc(X(x), Y(y), r, 0, Math.PI * 2)
      ctx.fillStyle = fill; ctx.fill()
      ctx.lineWidth = 1.5; ctx.strokeStyle = C.bg; ctx.stroke()
      ctx.fillStyle = C.bg; ctx.font = `bold 9px ${FONT_MONO}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(glyph, X(x), Y(y) + 0.5)
    }
    const square = (x: number, y: number, fill: string, glyph: string) => {
      ctx.fillStyle = fill; ctx.fillRect(X(x) - 6, Y(y) - 6, 12, 12)
      ctx.lineWidth = 1.5; ctx.strokeStyle = C.bg; ctx.strokeRect(X(x) - 6, Y(y) - 6, 12, 12)
      ctx.fillStyle = C.bg; ctx.font = `bold 9px ${FONT_MONO}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(glyph, X(x), Y(y) + 0.5)
    }
    for (const [x, y] of distPts) square(x, y, hubCol, 'D')
    for (const [x, y] of meetPts) disc(x, y, 6, hubCol, 'M')
    if (hub) {
      ctx.save(); ctx.translate(X(hub.x), Y(hub.y)); ctx.rotate((hub.yaw * Math.PI) / 180)
      ctx.fillStyle = hubCol; ctx.strokeStyle = C.bg; ctx.lineWidth = 1.5
      ctx.fillRect(-11, -5, 22, 10); ctx.strokeRect(-11, -5, 22, 10)
      ctx.restore()
      ctx.fillStyle = C.bg; ctx.font = `bold 8px ${FONT_MONO}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText('H', X(hub.x), Y(hub.y) + 0.5)
    }
    if (dest) disc(dest[0], dest[1], 7, MC.f2f, '★')
    // the pod: an arrow along its heading / ポッド（向きの矢印）
    ctx.save()
    ctx.translate(X(p.x), Y(p.y))
    ctx.rotate(((p.yaw ?? 0) * Math.PI) / 180)
    ctx.beginPath(); ctx.arc(0, 0, 11, 0, Math.PI * 2); ctx.fillStyle = `${C.accent}33`; ctx.fill()
    ctx.beginPath(); ctx.moveTo(8, 0); ctx.lineTo(-5, 5.5); ctx.lineTo(-2.5, 0); ctx.lineTo(-5, -5.5); ctx.closePath()
    ctx.fillStyle = C.accent; ctx.fill(); ctx.strokeStyle = C.bg; ctx.lineWidth = 1.2; ctx.stroke()
    ctx.restore()
    // scale bar / 縮尺
    const m = span > 400 ? 100 : span > 160 ? 50 : 20
    ctx.strokeStyle = C.fg2; ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(8, h - 10); ctx.lineTo(8 + m * k, h - 10); ctx.stroke()
    ctx.fillStyle = C.fg2; ctx.font = `9px ${FONT_MONO}`; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'
    ctx.fillText(`${m} m`, 8, h - 13)
  }, [p.x, p.y, p.yaw, hub?.x, hub?.y, dest?.[0], dest?.[1], mp, track.length, pathKey, distPts.length, meetPts.length, p.status])
  return (
    <Card title={<span className="flex items-center gap-1.5"><MapPin className="h-3.5 w-3.5 text-accent" aria-hidden />{t('mp_map')}</span>}>
      <canvas ref={ref} className="h-52 w-full rounded-lg border border-line" role="img"
        aria-label={t('mp_map_aria', { pod: podLabel(t, p.id) })} />
      <dl className="grid grid-cols-2 gap-x-3 text-[11px]">
        <div className="min-w-0"><dt className="text-fg-mute">{t('mp_pos')}</dt><dd className="num truncate text-fg">({f(p.x, 1)}, {f(p.y, 1)})</dd></div>
        <div className="min-w-0"><dt className="text-fg-mute">{t('mp_dest')}</dt><dd className="num truncate text-fg">{dest ? `(${f(dest[0], 1)}, ${f(dest[1], 1)})` : '–'}</dd></div>
      </dl>
      <ul className="flex flex-wrap gap-x-3 gap-y-1 text-[11px] text-fg-mute" aria-label={t('mp_map_legend')}>
        <li className="flex items-center gap-1"><i className="h-0.5 w-4 bg-accent" aria-hidden />{t('mp_map_track')}</li>
        <li className="flex items-center gap-1"><i className="h-0 w-4 border-t-2 border-dashed" style={{ borderColor: hubCol }} aria-hidden />{t('mp_map_path')}</li>
        <li className="flex items-center gap-1"><i className="h-2.5 w-2.5 rounded-full" style={{ background: MC.f2f }} aria-hidden />{t('mp_map_dest')}</li>
        <li className="flex items-center gap-1"><i className="h-2 w-3.5 rounded-[2px]" style={{ background: hubCol }} aria-hidden />{t('mp_map_hub')}</li>
        <li className="flex items-center gap-1"><i className="h-2.5 w-2.5 rounded-[2px]" style={{ background: hubCol }} aria-hidden />{t('mp_map_dist')}</li>
        <li className="flex items-center gap-1"><i className="h-2.5 w-2.5 rounded-full" style={{ background: hubCol }} aria-hidden />{t('mp_map_meet')}</li>
        {!mp && <li>{t('mp_map_none')}</li>}
      </ul>
    </Card>
  )
}
