// LastMile charging stations on the City Map: the station's footprint and a
// pre-launch check of a spot against the map the editor draws (map.json:
// building footprints and driving-lane polygons). A spot inside a building or
// on a road is marked red with the reason; the sim makes the final check
// (free ground, flat, off the road) and reports what it rejects in
// state.station_errors.
// 充電ステーション：敷地の形と、地図データ（建物・車線）による起動前の地点確認。
// 最終確認はシミュ側（state.station_errors）。
import type { ChargeSpot, DeliveryMap, XY } from './types'

/** The station footprint, 14 m x 9.8 m, in its own CARLA axes (x forward,
 *  y right) around the spot / 敷地（ステーション座標、地点が原点） */
export const FOOT = { x0: -4.02, x1: 9.98, y0: -7.21, y1: 2.6 } as const
/** radius drawn round a spot without a yaw (the sim turns it to the kerb) / 向き未指定の円 */
export const AUTO_R = 8

/** CARLA local (lx, ly) of a station at (x, y, yaw deg) -> world. / ローカル -> 世界 */
export function toWorld(x: number, y: number, yaw: number, lx: number, ly: number): XY {
  const r = yaw * Math.PI / 180, c = Math.cos(r), s = Math.sin(r)
  return [x + lx * c - ly * s, y + lx * s + ly * c]
}

/** The four footprint corners of a station (world metres). / 敷地の四隅 */
export function footprint(x: number, y: number, yaw: number): XY[] {
  return [[FOOT.x0, FOOT.y0], [FOOT.x1, FOOT.y0], [FOOT.x1, FOOT.y1], [FOOT.x0, FOOT.y1]]
    .map(([a, b]) => toWorld(x, y, yaw, a, b))
}

/** Point in polygon (flat [x0, y0, x1, y1, ...] or XY[]), even-odd rule. */
export function inPoly(px: number, py: number, f: number[]): boolean {
  let inside = false
  const n = f.length >> 1
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const xi = f[2 * i], yi = f[2 * i + 1], xj = f[2 * j], yj = f[2 * j + 1]
    if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) inside = !inside
  }
  return inside
}

/** why a spot cannot take a station (pre-launch check) / 地点が使えない理由 */
export type SpotIssue = 'building' | 'road'

const CELL = 25               // bucket size in metres / 格子の大きさ (m)
const key = (cx: number, cy: number) => (cx + 32768) * 65536 + (cy + 32768)

/** Buildings and driving lanes of one map in a grid of buckets, so a check
 *  costs a few polygons (built once per map: ~1200 buildings, ~3000 lanes on
 *  OitaKosen). Results are cached per spot. / 建物と車線の格子索引（地図ごと） */
export class SpotChecker {
  private bld: [number, number, number, number, number][]
  private lanes: number[][] = []
  private bCells = new Map<number, number[]>()
  private lCells = new Map<number, number[]>()
  private cache = new Map<string, SpotIssue | null>()

  constructor(m: DeliveryMap) {
    this.bld = m.buildings || []
    this.bld.forEach(([x, y, ex, ey], i) => {
      const r = Math.hypot(ex, ey)
      this.insert(this.bCells, i, x - r, y - r, x + r, y + r)
    })
    for (const [k, f] of m.lanes || []) {
      if (k !== 'd' || f.length < 6) continue
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity
      for (let i = 0; i + 1 < f.length; i += 2) {
        x0 = Math.min(x0, f[i]); x1 = Math.max(x1, f[i]); y0 = Math.min(y0, f[i + 1]); y1 = Math.max(y1, f[i + 1])
      }
      this.insert(this.lCells, this.lanes.length, x0, y0, x1, y1)
      this.lanes.push(f)
    }
  }

  /** the map has lane polygons (older sims draw centre lines only) / 車線ポリゴンあり */
  get hasLanes() { return this.lanes.length > 0 }

  private insert(cells: Map<number, number[]>, i: number, x0: number, y0: number, x1: number, y1: number) {
    for (let cx = Math.floor(x0 / CELL); cx <= Math.floor(x1 / CELL); cx++) {
      for (let cy = Math.floor(y0 / CELL); cy <= Math.floor(y1 / CELL); cy++) {
        const k = key(cx, cy), c = cells.get(k)
        if (c) c.push(i); else cells.set(k, [i])
      }
    }
  }

  /** inside a building footprint / 建物の中 */
  inBuilding(px: number, py: number): boolean {
    for (const i of this.bCells.get(key(Math.floor(px / CELL), Math.floor(py / CELL))) ?? []) {
      const [x, y, ex, ey, yaw] = this.bld[i]
      const r = yaw * Math.PI / 180, c = Math.cos(r), s = Math.sin(r), dx = px - x, dy = py - y
      if (Math.abs(dx * c + dy * s) <= ex && Math.abs(-dx * s + dy * c) <= ey) return true
    }
    return false
  }

  /** on a driving lane / 車道の上 */
  onRoad(px: number, py: number): boolean {
    for (const i of this.lCells.get(key(Math.floor(px / CELL), Math.floor(py / CELL))) ?? []) {
      if (inPoly(px, py, this.lanes[i])) return true
    }
    return false
  }

  /** What is wrong with a station at this spot, or null. The spot itself;
   *  with a yaw also a 6 x 5 grid over the whole footprint (without one the
   *  sim picks the direction, so only the spot is known). / 地点（向きがあれば敷地全体）を確認 */
  check(p: ChargeSpot): SpotIssue | null {
    const k = `${p.x}|${p.y}|${p.yaw ?? ''}`
    const hit = this.cache.get(k)
    if (hit !== undefined) return hit
    const pts: XY[] = [[p.x, p.y]]
    if (p.yaw != null) {
      for (let a = 0; a <= 5; a++) for (let b = 0; b <= 4; b++) {
        pts.push(toWorld(p.x, p.y, p.yaw, FOOT.x0 + (FOOT.x1 - FOOT.x0) * a / 5, FOOT.y0 + (FOOT.y1 - FOOT.y0) * b / 4))
      }
    }
    let out: SpotIssue | null = null
    if (pts.some(([x, y]) => this.inBuilding(x, y))) out = 'building'
    else if (pts.some(([x, y]) => this.onRoad(x, y))) out = 'road'
    if (this.cache.size > 500) this.cache.clear()
    this.cache.set(k, out)
    return out
  }
}

/** The sim's rejection of a placed spot (the nearest error within 3 m). */
export function simError(errors: { x: number; y: number; why: string }[] | undefined, p: ChargeSpot): string | null {
  let best: string | null = null, bd = 9
  for (const e of errors ?? []) {
    const d = (e.x - p.x) ** 2 + (e.y - p.y) ** 2
    if (d <= bd && typeof e.why === 'string') { bd = d; best = e.why }
  }
  return best
}
