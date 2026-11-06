import { C } from '@/lib/colors'
import { clamp, fmt } from '@/lib/fmt'
import type { TFunc } from '@/lib/i18n'
import type { BB, Hit, Kind, Near, Sensor, VehData } from './data'

// Pure logic of the Vehicle page, ported 1:1 from the old dashboard JS
// (vzone, snMeta, snValue, snPos, snName, spreadPts, sensDupKeys, updatePdc,
// renderNear, renderVeh). Stage geometry: viewBox 300×600, centre (150,300),
// the car block drawn at 50 % so the ring distances stay honest.
// 旧ページのロジックをそのまま移植（図の座標系も同じ）

export const CX = 150, CY = 300
export const VSCALE = 0.5              // the car block's scale (#vscale)
export const CAR_A = 78, CAR_B = 34    // hero silhouette half extents on stage (px)
export const PDC_THR = [0.9, 1.8, 3.0] as const
export const NEAR_RANGE = 50

export type VehState = 'ok' | 'warn' | 'crash'

/** Overall status pill: crash if an impact is < 5 s old, caution when off
 *  the road, stability < 0.6, ABS or TCS. Reasons are listed under the pill. */
export interface Reason { key: string; vars?: Record<string, number> }
export function vehState(v: VehData): { st: VehState; why: Reason[] } {
  const why: Reason[] = []
  const fresh = v.hits.filter((h) => h.age < 5)
  if (fresh.length) why.push({ key: 'vx_why_crash', vars: { s: Math.round(Math.min(...fresh.map((h) => h.age))) } })
  if (v.hero && !v.onRoad) why.push({ key: 'vx_why_offroad' })
  if (v.stability < 0.6) why.push({ key: 'vx_why_stab' })
  if (v.abs) why.push({ key: 'vx_why_abs' })
  if (v.tcs) why.push({ key: 'vx_why_tcs' })
  return { st: fresh.length ? 'crash' : why.length ? 'warn' : 'ok', why }
}

const zoneKey = (f: number, r: number, th: number) => {
  const fz = f > th ? 'front' : f < -th ? 'rear' : ''
  const rz = r > th ? 'right' : r < -th ? 'left' : ''
  return 'zone_' + (fz && rz ? `${fz}_${rz}` : (fz || rz || 'center'))
}
/** Impact zone name from the vehicle-frame impact point. */
export const vzone = (t: TFunc, f: number, r: number) => t(zoneKey(f, r, 0.4))
export const fmtAgo = (t: TFunc, a: number) => (a < 60 ? fmt(a, 0) + t('veh_ago') : fmt(a / 60, 0) + 'min')
/** Impact marker position in car-block coordinates (before the 50 % scale). */
export const hitPos = (h: Hit): [number, number] => [CX + h.r * 50, CY - h.f * 140]
export const hitOpacity = (h: Hit) => clamp(1 - h.age / 150, 0.25, 1)

/* ---------------------------------------------------------------- sensors */
export type Shape = 'circle' | 'square' | 'diamond' | 'tri' | 'tri-down' | 'hex' | 'pent' | 'star' | 'plus' | 'ring'
export interface SnMeta { key: string; color: string; shape: Shape }
// colour + a distinct shape per sensor family (colour is never the only cue)
const SN_META: [string, SnMeta][] = [
  ['sensor.other.gnss', { key: 'sn_gnss', color: C.cyan, shape: 'circle' }],
  ['sensor.other.imu', { key: 'sn_imu', color: C.violet, shape: 'square' }],
  ['sensor.other.collision', { key: 'sn_coll', color: C.danger, shape: 'diamond' }],
  ['sensor.other.lane_invasion', { key: 'sn_lane', color: C.info, shape: 'tri' }],
  ['sensor.other.obstacle', { key: 'sn_ultra', color: C.warn, shape: 'tri-down' }],
  ['sensor.lidar.ray_cast_semantic', { key: 'sn_semlidar', color: C.s4, shape: 'hex' }],
  ['sensor.lidar', { key: 'sn_lidar', color: C.accent, shape: 'hex' }],
  ['sensor.other.radar', { key: 'sn_radar', color: C.s2, shape: 'pent' }],
  ['sensor.camera', { key: 'sn_cam', color: C.s3, shape: 'star' }],
]
const SN_OTHER: SnMeta = { key: 'sn_other', color: C.mute, shape: 'plus' }
export function snMeta(type: string): SnMeta {
  for (const [p, m] of SN_META) if (type.startsWith(p)) return m
  return SN_OTHER
}

/** Live readout per sensor type (same as the old snValue). */
export function snValue(t: TFunc, v: VehData, type: string): string {
  if (type.startsWith('sensor.other.gnss')) return `${fmt(v.gnss.lat, 4)}, ${fmt(v.gnss.lon, 4)}`
  if (type.startsWith('sensor.other.imu')) return `${fmt(v.imu.ax, 1)} / ${fmt(v.imu.ay, 1)} m/s²`
  if (type.startsWith('sensor.other.collision')) return String(v.collisionCount || 0)
  if (type.startsWith('sensor.other.lane_invasion')) return String(v.laneInvasions || 0)
  if (type.startsWith('sensor.other.obstacle')) return v.ultra > 0 ? `${fmt(v.ultra, 1)} m` : '—'
  return t('sn_active')
}

/** Mounting-zone name from the vehicle-frame offset (reuses zone_*). */
export const snPos = (t: TFunc, x: Sensor, bb: BB) =>
  t(zoneKey(x.f / Math.max(bb.x, 0.1), x.r / Math.max(bb.y, 0.1), 0.35))

/** Sensor families present more than once. */
export function sensDupKeys(sens: Sensor[]): Set<string> {
  const cnt: Record<string, number> = {}
  sens.forEach((x) => { const k = snMeta(x.type).key; cnt[k] = (cnt[k] || 0) + 1 })
  return new Set(Object.keys(cnt).filter((k) => cnt[k] > 1))
}

/** Cameras (and any duplicated sensor family) get their position appended. */
export function snName(t: TFunc, x: Sensor, bb: BB, dup: Set<string>): string {
  const { key } = snMeta(x.type)
  return key === 'sn_cam' || dup.has(key) ? `${t(key)} · ${snPos(t, x, bb)}` : t(key)
}

const sensPos = (x: Sensor, bb: BB): [number, number] => [
  CX + clamp(x.r / bb.y, -1.15, 1.15) * 49,
  CY - clamp(x.f / bb.x, -1.15, 1.15) * 140,
]
/** Marker positions (car-block coordinates); sensors at the same spot are
 *  fanned out sideways so every marker stays visible. */
export function spreadPts(sens: Sensor[], bb: BB): [number, number][] {
  const pts = sens.map((x) => sensPos(x, bb))
  const groups: Record<string, number[]> = {}
  pts.forEach((p, i) => { const k = `${Math.round(p[0] / 10)}:${Math.round(p[1] / 10)}`; (groups[k] ??= []).push(i) })
  Object.values(groups).forEach((g) => g.forEach((idx, j) => { pts[idx][0] += (j - (g.length - 1) / 2) * 15 }))
  return pts
}
/** Car-block point → stage point (undo the 50 % scale). */
export const toStage = ([x, y]: [number, number]): [number, number] => [CX + (x - CX) * VSCALE, CY + (y - CY) * VSCALE]

/** SVG path of a marker shape centred on (0,0) with radius r. */
export function shapePath(s: Shape, r: number): string {
  const poly = (k: number, rot: number, rr = r) => Array.from({ length: k }, (_, i) => {
    const a = rot + (i * 2 * Math.PI) / k
    return `${i ? 'L' : 'M'}${fmt(Math.cos(a) * rr, 2)},${fmt(Math.sin(a) * rr, 2)}`
  }).join('') + 'Z'
  switch (s) {
    case 'circle': case 'ring': return `M${-r},0a${r},${r} 0 1,0 ${2 * r},0a${r},${r} 0 1,0 ${-2 * r},0Z`
    case 'square': { const q = r * 0.85; return `M${-q},${-q}H${q}V${q}H${-q}Z` }
    case 'diamond': return poly(4, -Math.PI / 2, r * 1.15)
    case 'tri': return poly(3, -Math.PI / 2, r * 1.2)
    case 'tri-down': return poly(3, Math.PI / 2, r * 1.2)
    case 'hex': return poly(6, 0, r * 1.05)
    case 'pent': return poly(5, -Math.PI / 2, r * 1.1)
    case 'star': return Array.from({ length: 10 }, (_, i) => {
      const a = -Math.PI / 2 + (i * Math.PI) / 5, rr = i % 2 ? r * 0.5 : r * 1.25
      return `${i ? 'L' : 'M'}${fmt(Math.cos(a) * rr, 2)},${fmt(Math.sin(a) * rr, 2)}`
    }).join('') + 'Z'
    case 'plus': { const w = r * 0.38; return `M${-w},${-r}H${w}V${-w}H${r}V${w}H${w}V${r}H${-w}V${w}H${-r}V${-w}H${-w}Z` }
  }
}

/* -------------------------------------------------------------------- PDC */
export type Zone = 'f' | 'b' | 'l' | 'r'
export const ZONES: Zone[] = ['f', 'b', 'l', 'r']
/** Park-distance margin per zone from the ground-truth nearby objects:
 *  margin = object distance minus the car body extent. */
export function pdcMargins(list: Near[], bb: BB): Record<Zone, number> {
  const z: Record<Zone, number> = { f: Infinity, b: Infinity, l: Infinity, r: Infinity }
  list.forEach((v) => {
    const m = Math.hypot(Math.max(0, Math.abs(v.fx) - bb.x), Math.max(0, Math.abs(v.rx) - bb.y))
    const th = (Math.atan2(v.rx, v.fx) * 180) / Math.PI
    const k: Zone = Math.abs(th) <= 45 ? 'f' : Math.abs(th) >= 135 ? 'b' : th > 0 ? 'r' : 'l'
    if (m < z[k]) z[k] = m
  })
  return z
}
/** Band colours inner → outer, and the readout colour for a margin. */
export const PDC_COL = [C.danger, C.warn, C.accent] as const
export const pdcTextCol = (m: number) => (m <= 0.9 ? C.s2 : m <= 1.8 ? C.s3 : C.s4)
export const pdcLevel = (m: number) => (m <= 0.9 ? 0 : m <= 1.8 ? 1 : m <= 3.0 ? 2 : 3)

/* ----------------------------------------------------------------- nearby */
export const KIND_C: Record<Kind, string> = { car: C.s2, truck: C.warn, bike: C.s5, walker: C.cyan }
/** √ distance scale: r = 240·√(d/50). */
export const distPix = (d: number) => 240 * Math.sqrt(clamp(d, 0, 50) / 50)
export const fmtDist = (d: number) => (d < 10 ? fmt(d, 1) : fmt(d, 0)) + ' m'

export interface NearPlaced { v: Near; x: number; y: number; clamped: boolean; opacity: number }
/** Radar position; very close objects sit just outside the car silhouette
 *  (an ellipse), like a real parking display. */
export function placeNear(v: Near): NearPlaced {
  const th = Math.atan2(v.rx, v.fx)
  const rmin = (CAR_A * CAR_B) / Math.hypot(CAR_B * Math.cos(th), CAR_A * Math.sin(th)) + 8
  const rd = distPix(v.d), r = Math.max(rd, rmin)
  return {
    v, x: CX + Math.sin(th) * r, y: CY - Math.cos(th) * r,
    clamped: rd < rmin, opacity: clamp(1.15 - v.d / 60, 0.4, 1),
  }
}

/** Ultrasonic arc colour: < 3 m critical, < 8 m caution, else clear. */
export const ultraCol = (d: number) => (d < 3 ? C.danger : d < 8 ? C.warn : C.accent)
