// Short per-pod history (the last minute at the sim's 2 Hz publish rate) for
// the sparklines: speed, distance to the destination, battery, and every
// numeric sensor value by its key; plus each pod's recent ground track.
// Module level, so it survives page switches while the sim keeps its time.
// 各ミニポッドの直近 1 分の値（スパークライン用）と走行軌跡
import { useEffect } from 'react'
import { distLeft } from './data'
import type { MiniPod } from './types'

export const HIST_N = 120
export const TRACK_N = 600

const HIST = new Map<number, Map<string, number[]>>()
const TRACK = new Map<number, [number, number][]>()
let lastT: number | undefined

function push(m: Map<string, number[]>, k: string, v: unknown) {
  let a = m.get(k)
  if (!a) { a = []; m.set(k, a) }
  a.push(typeof v === 'number' && Number.isFinite(v) ? v : NaN)
  if (a.length > HIST_N) a.shift()
}

function record(pods: MiniPod[], simT: number) {
  if (simT === lastT) return
  if (lastT !== undefined && simT < lastT) { HIST.clear(); TRACK.clear() }   // sim restarted / 再起動
  lastT = simT
  for (const p of pods) {
    let tr = TRACK.get(p.id)
    if (!tr) { tr = []; TRACK.set(p.id, tr) }
    const last = tr[tr.length - 1]
    if (last && Math.hypot(p.x - last[0], p.y - last[1]) > 60) tr.length = 0     // jump: new track / 瞬間移動
    if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.3) tr.push([p.x, p.y])
    if (tr.length > TRACK_N) tr.shift()
    let m = HIST.get(p.id)
    if (!m) { m = new Map(); HIST.set(p.id, m) }
    push(m, '_speed', p.speed)
    push(m, '_dist', distLeft(p))
    push(m, '_batt', p.battery)
    for (const [k, s] of Object.entries(p.sensors ?? {})) {
      if (s && typeof s.value === 'number') push(m, k, s.value)
    }
  }
}

/** Records each new sim time once; returns a getter for one pod's series.
 *  ``_speed``, ``_dist``, ``_batt`` or a sensor key. / 系列の取得関数 */
export function usePodHistory(pods: MiniPod[] | undefined, simT: number | undefined) {
  useEffect(() => { if (pods && simT !== undefined) record(pods, simT) }, [pods, simT])
  return (id: number, k: string): number[] => HIST.get(id)?.get(k) ?? []
}

export const trackOf = (id: number): [number, number][] => TRACK.get(id) ?? []

/** Tiny line of the last minute; gaps where there was no reading. Decorative
 *  (the value is printed next to it). / 小さな推移線 */
export function MiniSpark({ data, className = 'stroke-info' }: { data: number[]; className?: string }) {
  const vals = data.filter((v) => Number.isFinite(v))
  if (vals.length < 2) return <svg className="h-6 w-full" aria-hidden />
  let lo = Math.min(...vals), hi = Math.max(...vals)
  if (hi - lo < 1e-6) { lo -= 1; hi += 1 }
  const W = 100, H = 24, off = HIST_N - data.length
  const segs: string[] = []
  let cur = ''
  data.forEach((v, i) => {
    if (!Number.isFinite(v)) { if (cur) segs.push(cur); cur = ''; return }
    const x = ((off + i) / (HIST_N - 1)) * W
    const y = H - 2 - ((v - lo) / (hi - lo)) * (H - 4)
    cur += `${cur ? 'L' : 'M'}${x.toFixed(1)} ${y.toFixed(1)}`
  })
  if (cur) segs.push(cur)
  return (
    <svg viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" className="h-6 w-full" aria-hidden>
      {segs.map((d, i) => <path key={i} d={d} fill="none" strokeWidth={1.5} vectorEffect="non-scaling-stroke" className={className} />)}
    </svg>
  )
}
