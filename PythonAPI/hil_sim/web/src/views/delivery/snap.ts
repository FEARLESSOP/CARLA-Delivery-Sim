// Snap targets for pod meeting points: the kerb-side stop points of map.json
// ("stops", 4 m apart), or, from an older delivery sim, the vertices of the
// lane centre lines (roads), else of the driving-lane polygons. A grid of
// buckets built once per map keeps the nearest-point query cheap enough for
// every pointer move (~9000 stops on PasirRis, ~40 000 vertices as fallback).
// 合流地点の吸着先（停車点、無ければ車線の頂点）と格子の空間索引
import type { DeliveryMap } from './types'

const CELL = 20               // bucket size in metres / 格子の大きさ (m)

export class SnapIndex {
  /** true when the map has the exact stop points / 正確な停車点あり */
  readonly exact: boolean
  readonly count: number
  private xs: Float64Array
  private ys: Float64Array
  private cells = new Map<number, number[]>()

  constructor(pts: [number, number][], exact: boolean) {
    this.exact = exact
    this.count = pts.length
    this.xs = new Float64Array(pts.length)
    this.ys = new Float64Array(pts.length)
    pts.forEach(([x, y], i) => {
      this.xs[i] = x; this.ys[i] = y
      const k = SnapIndex.key(Math.floor(x / CELL), Math.floor(y / CELL))
      const c = this.cells.get(k)
      if (c) c.push(i); else this.cells.set(k, [i])
    })
  }

  private static key(cx: number, cy: number) { return (cx + 32768) * 65536 + (cy + 32768) }

  /** The nearest point within ``maxR`` metres, or null. Searches rings of
   *  cells outwards and stops once no closer point can exist.
   *  最も近い点（maxR 以内）、無ければ null */
  nearest(x: number, y: number, maxR: number): [number, number] | null {
    const cx = Math.floor(x / CELL), cy = Math.floor(y / CELL)
    const rings = Math.ceil(maxR / CELL) + 1
    let best = -1, bd = maxR * maxR
    for (let r = 0; r <= rings; r++) {
      // every point in ring r is at least (r - 1) * CELL away / 環 r の点は (r-1)*CELL 以上離れる
      if (best >= 0 && ((r - 1) * CELL) ** 2 > bd) break
      for (let i = -r; i <= r; i++) {
        for (let j = -r; j <= r; j++) {
          if (Math.max(Math.abs(i), Math.abs(j)) !== r) continue
          const c = this.cells.get(SnapIndex.key(cx + i, cy + j))
          if (!c) continue
          for (const n of c) {
            const d = (this.xs[n] - x) ** 2 + (this.ys[n] - y) ** 2
            if (d < bd) { bd = d; best = n }
          }
        }
      }
    }
    return best >= 0 ? [this.xs[best], this.ys[best]] : null
  }
}

/** Build the index of a map: stops if present, else lane vertices. */
export function buildSnap(m: DeliveryMap): SnapIndex {
  if (Array.isArray(m.stops) && m.stops.length) return new SnapIndex(m.stops, true)
  const pts: [number, number][] = []
  if (m.roads?.length) {
    for (const pl of m.roads) for (const p of pl) pts.push(p)
  } else {
    for (const [k, f] of m.lanes || []) {
      if (k !== 'd') continue
      for (let i = 0; i + 1 < f.length; i += 2) pts.push([f[i], f[i + 1]])
    }
  }
  return new SnapIndex(pts, false)
}
