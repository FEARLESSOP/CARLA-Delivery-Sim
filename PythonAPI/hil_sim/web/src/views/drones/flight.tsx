// The Drones page's right column: real-time flight data (battery gauge),
// altitude and speed with canvas sparklines, flight time left and distance
// from home, and the location / flight-path mini map (the delivery map's
// roads round the drone, its track, its pod and the drop point of its order).
// 右列：電池ゲージ・高度/速度の推移・残り時間/帰還距離・位置と航跡の小地図
import { useEffect, useState, type ReactNode } from 'react'
import { Clock, Gauge, MapPin, Mountain, Route, Zap } from 'lucide-react'
import { getJSON } from '@/lib/api'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Card } from '@/components/ui'
import { MC, battTone } from '@/views/delivery/shared'
import type { DeliveryMap, DeliveryState } from '@/views/delivery/types'
import { f, mmss } from './catalog'
import { trackOf } from './history'
import type { DroneS } from './types'

/** Semicircle battery gauge (0-100 %), ticks every 10. / 半円の電池ゲージ */
function BatteryGauge({ value, volts }: { value: number; volts?: number }) {
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
    <svg viewBox="0 0 200 118" className="mx-auto block w-full max-w-[300px]" role="img"
      aria-label={`${t('dn_batt_level')} ${Math.round(v)} %`}>
      <path d={arc(100)} className="fill-none stroke-muted" strokeWidth={12} strokeLinecap="round" />
      {v > 0.5 && <path d={arc(v)} className={cn('fill-none transition-[d] duration-500', col)} strokeWidth={12} strokeLinecap="round" />}
      {Array.from({ length: 11 }, (_, i) => {
        const [x0, y0] = pt(i * 10, R - 13), [x1, y1] = pt(i * 10, R - (i % 5 === 0 ? 21 : 17))
        return <line key={i} x1={x0} y1={y0} x2={x1} y2={y1} className="stroke-line-strong" strokeWidth={1.2} />
      })}
      <text x={cx - R} y={cy + 16} textAnchor="middle" className="fill-fg-mute text-[9px]">0</text>
      <text x={cx + R} y={cy + 16} textAnchor="middle" className="fill-fg-mute text-[9px]">100</text>
      <text x={cx} y={cy - 18} textAnchor="middle" className="num fill-fg text-[30px] font-semibold">{Math.round(v)}%</text>
      <text x={cx} y={cy + 2} textAnchor="middle" className="fill-fg-mute text-[10px]">
        {t('dn_batt_level')}{volts ? ` · ${volts.toFixed(1)} V` : ''}
      </text>
    </svg>
  )
}

/** A canvas sparkline with three y labels (min / mid / max). / 推移線 */
function Spark({ data, color, lo, label }: { data: number[]; color: string; lo?: number; label: string }) {
  const ref = useCanvas((ctx, w, h) => {
    const vals = data.filter((v) => Number.isFinite(v))
    const padL = 26, padB = 4, padT = 4
    let a = lo ?? Math.min(...vals, 0), b = Math.max(...vals, (lo ?? 0) + 1)
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
      ctx.fillText(String(Math.round(a + (b - a) * k)), padL - 5, y)
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
  }, [data.length, data[data.length - 1], color, lo])
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

/** Battery gauge card with the Live badge. / リアルタイム飛行データ */
export function FlightData({ d, live }: { d: DroneS; live: boolean }) {
  const t = useT()
  return (
    <Card title={<span className="flex items-center gap-1.5"><Zap className="h-3.5 w-3.5 text-accent" aria-hidden />{t('dn_rt')}</span>}
      actions={<Badge tone={live ? 'good' : 'neutral'} dot>{live ? t('dn_live') : t('dn_paused')}</Badge>}>
      <BatteryGauge value={d.sensors?.battery?.soc ?? d.battery} volts={d.sensors?.battery?.voltage} />
    </Card>
  )
}

export function AltSpeed({ d, hist }: { d: DroneS; hist: (k: 'hgt' | 'speed') => number[] }) {
  const t = useT()
  const s = d.sensors
  const hgt = s?.rangefinder?.ground ?? d.z
  const spd = s?.gnss?.speed
  const sp = hist('speed').map((v) => v * 3.6)
  return (
    <div className="grid grid-cols-2 gap-3">
      <MetricCard icon={<Mountain className="h-3.5 w-3.5" />} title={t('dn_alt_card')} value={f(hgt, 0)} unit="m"
        sub={t('dn_alt_sub', { msl: f(s?.gnss?.alt, 0) })}>
        <Spark data={hist('hgt')} color={C.s1} lo={0} label={t('dn_alt_trend')} />
      </MetricCard>
      <MetricCard icon={<Gauge className="h-3.5 w-3.5" />} title={t('dn_speed_card')} value={spd == null ? '–' : f(spd * 3.6, 0)} unit="km/h"
        sub={`${f(spd, 1)} m/s`}>
        <Spark data={sp} color={C.s5} lo={0} label={t('dn_speed_trend')} />
      </MetricCard>
    </div>
  )
}

export function TimeHome({ d }: { d: DroneS }) {
  const t = useT()
  const s = d.sensors
  const left = s?.battery?.left_s
  const dist = s?.link?.dist
  const docked = s?.link?.mode === 'wired'
  const home = d.van ? t('dn_home_sub_pod') : t('dn_home_sub_station')
  return (
    <div className="grid grid-cols-2 gap-3">
      <MetricCard icon={<Clock className="h-3.5 w-3.5" />} title={t('dn_remaining')} value={mmss(left)}
        sub={left == null ? t('dn_remaining_na') : t('dn_remaining_sub')} />
      <MetricCard icon={<Route className="h-3.5 w-3.5" />} title={t('dn_home')}
        value={docked ? '0' : dist == null ? '–' : dist >= 1000 ? f(dist / 1000, 2) : f(dist, 0)}
        unit={dist != null && dist >= 1000 && !docked ? 'km' : 'm'} sub={docked ? t('dn_home_docked') : home} />
    </div>
  )
}

/* ------------------------------------------------------------ mini map */
let MAP_CACHE: { name: string; map: DeliveryMap; roads: Path2D; lanes: Path2D | null; blds: Path2D } | null = null

function buildMap(name: string, m: DeliveryMap) {
  const roads = new Path2D()
  for (const r of m.roads ?? []) {
    r.forEach(([x, y], i) => (i ? roads.lineTo(x, y) : roads.moveTo(x, y)))
  }
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
  return { name, map: m, roads, lanes, blds }
}

/** /delivery/map once per map (the state names it). / 地図を一度だけ取得 */
function useDeliveryMap(name: string | undefined) {
  const [m, setM] = useState(MAP_CACHE && MAP_CACHE.name === name ? MAP_CACHE : null)
  useEffect(() => {
    if (!name) return
    if (MAP_CACHE && MAP_CACHE.name === name) { setM(MAP_CACHE); return }
    let alive = true
    getJSON<DeliveryMap>('/delivery/map').then((mp) => {
      if (!alive || !mp) return
      MAP_CACHE = buildMap(name, mp)
      setM(MAP_CACHE)
    }).catch(() => {})
    return () => { alive = false }
  }, [name])
  return m
}

export function MiniMap({ d, st }: { d: DroneS; st: DeliveryState }) {
  const t = useT()
  const mp = useDeliveryMap(st.map)
  const van = st.vans?.find((v) => v.id === d.van)
  const order = st.orders?.find((o) => o.id === d.order)
  const track = trackOf(d.id)
  const s = d.sensors
  const ref = useCanvas((ctx, w, h) => {
    ctx.fillStyle = C.bg
    ctx.fillRect(0, 0, w, h)
    // frame: the drone, its pod and its drop point, at least 260 m across / 表示範囲
    const pts: [number, number][] = [[d.x, d.y]]
    if (van) pts.push([van.x, van.y])
    if (order) pts.push([order.x, order.y])
    let x0 = Math.min(...pts.map((p) => p[0])), x1 = Math.max(...pts.map((p) => p[0]))
    let y0 = Math.min(...pts.map((p) => p[1])), y1 = Math.max(...pts.map((p) => p[1]))
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2
    const span = Math.max(260, (x1 - x0) * 1.35, (y1 - y0) * 1.35 * (w / h))
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
    // track / 航跡
    if (track.length > 1) {
      ctx.strokeStyle = C.accent
      ctx.lineWidth = 2
      ctx.setLineDash([])
      ctx.beginPath()
      track.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))))
      ctx.stroke()
    }
    // line to the drop point / 投下地点への線
    if (order && d.status === 'delivering') {
      ctx.setLineDash([4, 4])
      ctx.strokeStyle = C.mute
      ctx.lineWidth = 1
      ctx.beginPath(); ctx.moveTo(X(d.x), Y(d.y)); ctx.lineTo(X(order.x), Y(order.y)); ctx.stroke()
      ctx.setLineDash([])
    }
    const disc = (x: number, y: number, r: number, fill: string, glyph: string) => {
      ctx.beginPath(); ctx.arc(X(x), Y(y), r, 0, Math.PI * 2)
      ctx.fillStyle = fill; ctx.fill()
      ctx.lineWidth = 1.5; ctx.strokeStyle = C.bg; ctx.stroke()
      ctx.fillStyle = C.bg; ctx.font = `bold 9px ${FONT_MONO}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
      ctx.fillText(glyph, X(x), Y(y) + 0.5)
    }
    if (van) disc(van.x, van.y, 8, MC.pod, 'P')
    if (order) disc(order.x, order.y, 7, MC.f2f, '↓')
    // the drone: an arrow along its heading / 機体（機首方向の矢印）
    ctx.save()
    ctx.translate(X(d.x), Y(d.y))
    ctx.rotate(((d.yaw ?? 0) * Math.PI) / 180)
    ctx.beginPath(); ctx.arc(0, 0, 12, 0, Math.PI * 2); ctx.fillStyle = `${C.accent}33`; ctx.fill()
    ctx.beginPath(); ctx.moveTo(9, 0); ctx.lineTo(-6, 6); ctx.lineTo(-3, 0); ctx.lineTo(-6, -6); ctx.closePath()
    ctx.fillStyle = C.accent; ctx.fill(); ctx.strokeStyle = C.bg; ctx.lineWidth = 1.2; ctx.stroke()
    ctx.restore()
    // scale bar / 縮尺
    const m = span > 800 ? 200 : span > 400 ? 100 : 50
    ctx.strokeStyle = C.fg2; ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(8, h - 10); ctx.lineTo(8 + m * k, h - 10); ctx.stroke()
    ctx.fillStyle = C.fg2; ctx.font = `9px ${FONT_MONO}`; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'
    ctx.fillText(`${m} m`, 8, h - 13)
  }, [d.x, d.y, d.yaw, van?.x, van?.y, order?.x, order?.y, mp, track.length, d.status])
  const lat = s?.gnss?.lat, lon = s?.gnss?.lon
  const row = (k: string, v: string) => (
    <div className="min-w-0"><dt className="text-[10px] text-fg-mute">{k}</dt><dd className="num truncate text-sm text-fg">{v}</dd></div>
  )
  return (
    <Card title={<span className="flex items-center gap-1.5"><MapPin className="h-3.5 w-3.5 text-accent" aria-hidden />{t('dn_map')}</span>}>
      <div className="grid grid-cols-[minmax(0,1fr)_92px] gap-3">
        <canvas ref={ref} className="h-40 w-full rounded-lg border border-line" role="img"
          aria-label={t('dn_map_aria', { drone: d.id })} />
        <dl className="flex flex-col justify-between gap-1">
          {row(t('dn_lat'), lat == null ? '–' : `${f(Math.abs(lat), 4)}° ${lat >= 0 ? 'N' : 'S'}`)}
          {row(t('dn_lon'), lon == null ? '–' : `${f(Math.abs(lon), 4)}° ${lon >= 0 ? 'E' : 'W'}`)}
          {row(t('dn_height'), `${f(s?.rangefinder?.ground ?? d.z, 0)} m`)}
          {row(t('dn_speed_card'), `${s?.gnss?.speed == null ? '–' : f(s.gnss.speed * 3.6, 0)} km/h`)}
        </dl>
      </div>
      <ul className="flex flex-wrap gap-x-3 gap-y-1 text-[11px] text-fg-mute" aria-label={t('dn_map_legend')}>
        <li className="flex items-center gap-1"><i className="h-0.5 w-4 bg-accent" aria-hidden />{t('dn_map_track')}</li>
        <li className="flex items-center gap-1"><i className="h-2.5 w-2.5 rounded-full" style={{ background: MC.pod }} aria-hidden />{t('dn_map_pod')}</li>
        <li className="flex items-center gap-1"><i className="h-2.5 w-2.5 rounded-full" style={{ background: MC.f2f }} aria-hidden />{t('dn_map_drop')}</li>
        {!mp && <li>{t('dn_map_none')}</li>}
      </ul>
    </Card>
  )
}

