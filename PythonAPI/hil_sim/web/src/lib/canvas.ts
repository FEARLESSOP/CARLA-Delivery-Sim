import { useEffect, useRef } from 'react'

export type DrawFn = (ctx: CanvasRenderingContext2D, w: number, h: number) => void

/** A canvas sized to its CSS box (device pixels), redrawn by ``draw`` whenever
 *  ``deps`` change or the box resizes. Unlike the old dashboard it only
 *  reallocates the bitmap when the size really changes.
 *  CSS の大きさに合わせたキャンバス（サイズ変更時だけ再確保）。 */
export function useCanvas(draw: DrawFn, deps: unknown[]) {
  const ref = useRef<HTMLCanvasElement>(null)
  const drawRef = useRef(draw)
  drawRef.current = draw

  const paint = () => {
    const c = ref.current
    if (!c) return
    const w = c.clientWidth, h = c.clientHeight
    if (w < 2 || h < 2) return
    const dpr = window.devicePixelRatio || 1
    const W = Math.round(w * dpr), H = Math.round(h * dpr)
    if (c.width !== W || c.height !== H) { c.width = W; c.height = H }
    const ctx = c.getContext('2d')
    if (!ctx) return
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    ctx.clearRect(0, 0, w, h)
    drawRef.current(ctx, w, h)
  }

  useEffect(() => {
    const c = ref.current
    if (!c) return
    const ro = new ResizeObserver(() => paint())
    ro.observe(c)
    return () => ro.disconnect()
  }, [])
  // eslint-disable-next-line react-hooks/exhaustive-deps
  useEffect(paint, deps)
  return ref
}

/** Fixed-length ring buffer for chart history. / チャート履歴のリングバッファ */
export class Series {
  readonly data: number[] = []
  readonly len: number
  constructor(len = 300) { this.len = len }
  push(v: number) { this.data.push(v); if (this.data.length > this.len) this.data.shift() }
  clear() { this.data.length = 0 }
}

/** A cached layer for what never moves (roads, buildings): drawn once per
 *  ``key`` and canvas size, then copied in with one drawImage per frame.
 *  動かない層（道路・建物）を一度だけ描いて使い回す。 */
export class StaticLayer {
  private c: HTMLCanvasElement | null = null
  private sig = ''

  draw(ctx: CanvasRenderingContext2D, w: number, h: number, key: unknown, paint: DrawFn) {
    const dpr = window.devicePixelRatio || 1
    const sig = `${w}x${h}@${dpr}`
    if (!this.c || this.sig !== sig || this.key !== key) {
      this.c = this.c ?? document.createElement('canvas')
      this.c.width = Math.round(w * dpr); this.c.height = Math.round(h * dpr)
      const g = this.c.getContext('2d')!
      g.setTransform(dpr, 0, 0, dpr, 0, 0)
      paint(g, w, h)
      this.sig = sig; this.key = key
    }
    ctx.drawImage(this.c, 0, 0, w, h)
  }

  private key: unknown = null
}
