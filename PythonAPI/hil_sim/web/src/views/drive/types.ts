import type { Telemetry } from '@/lib/telemetry'

// The /telemetry fields the Drive page reads (dashboard.py, TELEMETRY dict),
// read with defaults so a missing field never breaks the page.
// 運転ページが読む /telemetry の項目（欠けても既定値で表示）

export interface Weather {
  cloudiness: number; precipitation: number; wetness: number
  fog_density: number; wind_intensity: number; sun_altitude: number
}
export interface Nearby { rx: number; fx: number; kind: string; kmh?: number; d?: number }
export interface DriveEvent { t: string; type: string; msg: string }
export interface Vec3 { x: number; y: number; z: number }
export interface Imu { ax: number; ay: number; az: number; gx: number; gy: number; gz: number }

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null
const n = (o: Record<string, unknown>, k: string, d = 0): number => {
  const v = o[k]
  return typeof v === 'number' && isFinite(v) ? v : d
}
const obj = (s: Telemetry | null, k: string): Record<string, unknown> => {
  const v = s ? s[k] : undefined
  return isObj(v) ? v : {}
}

export const bool = (s: Telemetry | null, k: string): boolean => Boolean(s && s[k])
export const str = (s: Telemetry | null, k: string, d = ''): string => {
  const v = s ? s[k] : undefined
  return typeof v === 'string' ? v : typeof v === 'number' ? String(v) : d
}

export function weatherOf(s: Telemetry | null): Weather {
  const w = obj(s, 'weather')
  return {
    cloudiness: n(w, 'cloudiness'), precipitation: n(w, 'precipitation'), wetness: n(w, 'wetness'),
    fog_density: n(w, 'fog_density'), wind_intensity: n(w, 'wind_intensity'), sun_altitude: n(w, 'sun_altitude'),
  }
}

export function vec3Of(s: Telemetry | null, k: string): Vec3 {
  const o = obj(s, k)
  return { x: n(o, 'x'), y: n(o, 'y'), z: n(o, 'z') }
}

export function gnssOf(s: Telemetry | null): { lat: number; lon: number } {
  const o = obj(s, 'gnss')
  return { lat: n(o, 'lat'), lon: n(o, 'lon') }
}

export function imuOf(s: Telemetry | null): Imu {
  const o = obj(s, 'imu')
  return { ax: n(o, 'ax'), ay: n(o, 'ay'), az: n(o, 'az'), gx: n(o, 'gx'), gy: n(o, 'gy'), gz: n(o, 'gz') }
}

export function nearbyOf(s: Telemetry | null): Nearby[] {
  const v = s ? s.nearby_list : undefined
  if (!Array.isArray(v)) return []
  return v.filter(isObj).map((o) => ({
    rx: n(o, 'rx'), fx: n(o, 'fx'), kind: typeof o.kind === 'string' ? o.kind : 'car',
    kmh: typeof o.kmh === 'number' ? o.kmh : undefined, d: typeof o.d === 'number' ? o.d : undefined,
  }))
}

export function eventsOf(s: Telemetry | null): DriveEvent[] {
  const v = s ? s.events : undefined
  if (!Array.isArray(v)) return []
  return v.filter(isObj).map((o) => ({
    t: String(o.t ?? ''), type: String(o.type ?? ''), msg: String(o.msg ?? ''),
  }))
}

export function pointsOf(s: Telemetry | null, k: string): [number, number][] {
  const v = s ? s[k] : undefined
  if (!Array.isArray(v)) return []
  const out: [number, number][] = []
  for (const p of v) if (Array.isArray(p) && typeof p[0] === 'number' && typeof p[1] === 'number') out.push([p[0], p[1]])
  return out
}

export function numsOf(s: Telemetry | null, k: string): number[] {
  const v = s ? s[k] : undefined
  return Array.isArray(v) ? v.filter((x): x is number => typeof x === 'number' && isFinite(x)) : []
}
