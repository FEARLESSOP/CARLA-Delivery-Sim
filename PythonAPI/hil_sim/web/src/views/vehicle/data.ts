import { num, type Telemetry } from '@/lib/telemetry'

// The /telemetry fields this page reads (dashboard.py, STATE / the sample in
// _telemetry_loop), parsed with defaults so a missing or odd field never
// breaks the page. / このページが読むテレメトリ項目（既定値付きで解析）

export type Kind = 'car' | 'truck' | 'bike' | 'walker'
export type Signal = 'off' | 'left' | 'right' | 'hazard'

/** collision_hits[]: impact point in the vehicle frame (f, r in −1..1). */
export interface Hit { age: number; f: number; r: number; i: number; o: string; key: string }
/** nearby_list[]: actor relative to the hero (fx forward, rx right, metres). */
export interface Near { fx: number; rx: number; d: number; yaw: number; kmh: number; kind: Kind }
/** sensors[]: an attached sensor's mounting offset (metres, vehicle frame). */
export interface Sensor { type: string; f: number; r: number }
/** bb: hero bounding-box half extents (metres). */
export interface BB { x: number; y: number }

export interface VehData {
  hero: boolean
  onRoad: boolean
  speed: number
  gear: number
  reverse: boolean
  handBrake: boolean
  steer: number         // −1..1
  heading: number       // degrees
  stability: number     // 0..1
  abs: boolean
  tcs: boolean
  brake: number         // 0..1
  collisionCount: number
  laneInvasions: number
  laneWidth: number
  laneOffset: number    // + = right of the lane centre
  ultra: number         // front obstacle distance, ≤ 0 = nothing
  bb: BB
  signal: Signal
  gnss: { lat: number; lon: number }
  imu: { ax: number; ay: number }
  hits: Hit[]
  near: Near[]          // whole list (≤ 80 m), sorted by distance
  sensors: Sensor[]
}

type Obj = Record<string, unknown>
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v)
const n = (o: Obj, k: string, d = 0): number => { const v = o[k]; return typeof v === 'number' && isFinite(v) ? v : d }
const b = (s: Telemetry, k: string): boolean => !!s[k]
const arr = (v: unknown): Obj[] => (Array.isArray(v) ? v.filter(isObj) : [])
const KINDS: Kind[] = ['car', 'truck', 'bike', 'walker']
const SIGNALS: Signal[] = ['off', 'left', 'right', 'hazard']

export function readVeh(s: Telemetry): VehData {
  const bbRaw = isObj(s.bb) ? s.bb : {}
  const gnss = isObj(s.gnss) ? s.gnss : {}
  const imu = isObj(s.imu) ? s.imu : {}
  const sig = s.turn_signal
  const seen = new Map<string, number>()
  const hits = arr(s.collision_hits).map((h): Hit => {
    const f = n(h, 'f'), r = n(h, 'r'), i = n(h, 'i'), o = String(h.o ?? '')
    // the old page keyed impacts by f|r|i; make duplicates unique for React
    const base = `${f}|${r}|${i}|${o}`
    const c = seen.get(base) ?? 0
    seen.set(base, c + 1)
    return { age: n(h, 'age'), f, r, i, o, key: c ? `${base}#${c}` : base }
  })
  return {
    hero: b(s, 'hero_found'),
    onRoad: b(s, 'on_road'),
    speed: num(s, 'speed_kmh'),
    gear: num(s, 'gear'),
    reverse: b(s, 'reverse'),
    handBrake: b(s, 'hand_brake'),
    steer: num(s, 'steer'),
    heading: num(s, 'heading_deg'),
    stability: num(s, 'stability', 1),
    abs: b(s, 'abs_active'),
    tcs: b(s, 'tcs_active'),
    brake: num(s, 'brake'),
    collisionCount: num(s, 'collision_count'),
    laneInvasions: num(s, 'lane_invasions'),
    laneWidth: num(s, 'lane_width'),
    laneOffset: num(s, 'lane_offset'),
    ultra: num(s, 'ultrasonic_m', -1),
    bb: { x: n(bbRaw, 'x', 2.4), y: n(bbRaw, 'y', 1.0) },
    signal: SIGNALS.includes(sig as Signal) ? (sig as Signal) : 'off',
    gnss: { lat: n(gnss, 'lat'), lon: n(gnss, 'lon') },
    imu: { ax: n(imu, 'ax'), ay: n(imu, 'ay') },
    hits,
    near: arr(s.nearby_list).map((v): Near => ({
      fx: n(v, 'fx'), rx: n(v, 'rx'), d: n(v, 'd'), yaw: n(v, 'yaw'), kmh: n(v, 'kmh'),
      kind: KINDS.includes(v.kind as Kind) ? (v.kind as Kind) : 'car',
    })),
    sensors: arr(s.sensors).map((x): Sensor => ({ type: String(x.type ?? ''), f: n(x, 'f'), r: n(x, 'r') })),
  }
}

/** /keybinds (only what the turn-signal hint needs). */
export interface KeybindsInfo {
  bindings: Record<string, { keys?: string[]; wheel?: string[] }>
  labels: Record<string, string>
  label_ja: Record<string, string>
}
