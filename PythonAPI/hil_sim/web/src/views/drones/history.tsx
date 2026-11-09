// Short per-drone history of a few sensor values (the last minute at the
// sim's 2 Hz publish rate) for the cards' sparklines, and the sparkline.
// 各機のセンサー値の直近 1 分（スパークライン用）
import { useEffect } from 'react'
import type { DroneS } from './types'

export const HIST_N = 120
export type HistKey =
  | 'soc' | 'current' | 'volt' | 'btemp' | 'alt' | 'agl' | 'speed' | 'vs' | 'rssi' | 'rpm'
  | 'vibe' | 'near' | 'tether' | 'load' | 'wind' | 'airspeed' | 'hdop' | 'heading' | 'hgt'

type Hist = Map<number, Map<HistKey, number[]>>

function sample(d: DroneS): Partial<Record<HistKey, number | null | undefined>> {
  const s = d.sensors
  if (!s) return { soc: d.battery, alt: d.z, hgt: d.z }
  const rpm = s.motors?.rpm
  return {
    soc: s.battery?.soc, current: s.battery?.current, volt: s.battery?.voltage, btemp: s.battery?.temp,
    alt: s.baro?.alt, agl: s.rangefinder?.agl, speed: s.gnss?.speed, vs: s.baro?.vs, rssi: s.link?.rssi,
    rpm: rpm && rpm.length ? rpm.reduce((a, b) => a + b, 0) / rpm.length : undefined,
    vibe: s.imu?.vibe, near: s.obstacle?.nearest, tether: s.winch?.tether, load: s.winch?.load,
    wind: s.air?.wind, airspeed: s.air?.airspeed, hdop: s.gnss?.hdop, heading: s.mag?.heading,
    hgt: s.rangefinder?.ground ?? d.z,
  }
}

// module level: survives page switches while the sim keeps its time / ページ切替でも保持
const HIST: Hist = new Map()
let lastT: number | undefined
/** each drone's recent ground track (world x, y), ~4 min / 各機の航跡 */
export const TRACK_N = 480
const TRACK = new Map<number, [number, number][]>()

function record(drones: DroneS[], simT: number) {
  if (simT === lastT) return
  if (lastT !== undefined && simT < lastT) { HIST.clear(); TRACK.clear() }   // sim restarted / 再起動
  lastT = simT
  for (const d of drones) {
    let tr = TRACK.get(d.id)
    if (!tr) { tr = []; TRACK.set(d.id, tr) }
    const last = tr[tr.length - 1]
    // a jump (re-docked, respawned) starts a new track / 瞬間移動で航跡を切る
    if (last && Math.hypot(d.x - last[0], d.y - last[1]) > 80) tr.length = 0
    if (!last || Math.hypot(d.x - last[0], d.y - last[1]) > 0.5) tr.push([d.x, d.y])
    if (tr.length > TRACK_N) tr.shift()
    let m = HIST.get(d.id)
    if (!m) { m = new Map(); HIST.set(d.id, m) }
    for (const [k, v] of Object.entries(sample(d)) as [HistKey, number | null | undefined][]) {
      let a = m.get(k)
      if (!a) { a = []; m.set(k, a) }
      a.push(typeof v === 'number' && Number.isFinite(v) ? v : NaN)
      if (a.length > HIST_N) a.shift()
    }
  }
}

/** Records each new sim time once (after the render that brought it; the
 *  next poll shows it); returns a getter for one drone's series.
 *  新しいシミュ時刻ごとに 1 回記録し、系列の取得関数を返す */
export function useSensorHistory(drones: DroneS[] | undefined, simT: number | undefined) {
  useEffect(() => { if (drones && simT !== undefined) record(drones, simT) }, [drones, simT])
  return (id: number, k: HistKey): number[] => HIST.get(id)?.get(k) ?? []
}

/** A drone's recent ground track. / 航跡 */
export const trackOf = (id: number): [number, number][] => TRACK.get(id) ?? []

/** Tiny line of the last minute; gaps where there was no reading. Decorative
 *  (the value is printed next to it). / 小さな推移線（値は横に表示） */
export function Spark({ data, className = 'stroke-info', min, max }: {
  data: number[]; className?: string; min?: number; max?: number
}) {
  const vals = data.filter((v) => Number.isFinite(v))
  if (vals.length < 2) return <svg className="h-6 w-full" aria-hidden />
  let lo = min ?? Math.min(...vals), hi = max ?? Math.max(...vals)
  if (hi - lo < 1e-6) { lo -= 1; hi += 1 }
  const W = 100, H = 24, n = HIST_N
  const off = n - data.length
  const segs: string[] = []
  let cur = ''
  data.forEach((v, i) => {
    if (!Number.isFinite(v)) { if (cur) segs.push(cur); cur = ''; return }
    const x = ((off + i) / (n - 1)) * W
    const y = H - 2 - ((Math.min(Math.max(v, lo), hi) - lo) / (hi - lo)) * (H - 4)
    cur += `${cur ? 'L' : 'M'}${x.toFixed(1)} ${y.toFixed(1)}`
  })
  if (cur) segs.push(cur)
  return (
    <svg viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" className="h-6 w-full" aria-hidden>
      {segs.map((d, i) => <path key={i} d={d} fill="none" strokeWidth={1.5} vectorEffect="non-scaling-stroke" className={className} />)}
    </svg>
  )
}
