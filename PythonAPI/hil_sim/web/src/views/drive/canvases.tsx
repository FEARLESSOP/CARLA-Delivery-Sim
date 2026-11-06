import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { clamp } from '@/lib/fmt'
import { GG } from './history'
import type { Nearby } from './types'

// The Drive page's special canvases (old renderGG / renderMinimap /
// renderLane / renderTrail / renderColl). / 運転ページの特殊キャンバス

const G = 9.81
const font = (px = 11) => `${px}px ${FONT_MONO}`

/** G-G diagram: last 150 (lat, lon) points fading with age, rings every 0.5 g. */
export function GGDiagram({ tick, label, height = 220 }: { tick: number; label: string; height?: number }) {
  const ref = useCanvas((ctx, w, h) => {
    const cx = w / 2, cy = h / 2
    const scale = Math.min(w, h) / 2 / (1.5 * G)
    ctx.strokeStyle = C.grid
    ctx.lineWidth = 1
    ctx.font = font(10)
    ctx.fillStyle = C.mute
    for (const g of [0.5, 1, 1.5]) {
      ctx.beginPath(); ctx.arc(cx, cy, g * G * scale, 0, Math.PI * 2); ctx.stroke()
      ctx.fillText(`${g}g`, cx + g * G * scale * 0.71 + 2, cy - g * G * scale * 0.71 - 2)
    }
    ctx.strokeStyle = C.axis
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke()
    ctx.fillStyle = C.mute
    ctx.font = font(11)
    ctx.fillText('lon+', cx + 4, 12)
    ctx.textAlign = 'right'
    ctx.fillText('lat+', w - 4, cy - 5)
    ctx.textAlign = 'left'
    const n = GG.length
    ctx.fillStyle = C.s1
    GG.forEach(([lon, lat], i) => {
      const age = (n - i) / n
      ctx.globalAlpha = clamp(1 - age, 0.06, 1)
      ctx.beginPath(); ctx.arc(cx + lat * scale, cy - lon * scale, 2.2, 0, Math.PI * 2); ctx.fill()
    })
    ctx.globalAlpha = 1
    if (n) {
      const [lon, lat] = GG[n - 1]
      ctx.fillStyle = C.warn
      ctx.strokeStyle = C.bg
      ctx.lineWidth = 2
      ctx.beginPath(); ctx.arc(cx + lat * scale, cy - lon * scale, 4.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
    }
  }, [tick])
  return <canvas ref={ref} role="img" aria-label={label} className="block w-full" style={{ height }} />
}

/** Colour and shape per road-user kind (shape so colour is not the only cue). */
export const KIND: Record<string, { color: string; shape: 'circle' | 'square' | 'diamond' | 'dot' }> = {
  car: { color: C.s2, shape: 'circle' },
  truck: { color: C.warn, shape: 'square' },
  bike: { color: C.s5, shape: 'diamond' },
  walker: { color: C.cyan, shape: 'dot' },
}

function mark(ctx: CanvasRenderingContext2D, x: number, y: number, shape: string, r: number) {
  ctx.beginPath()
  if (shape === 'square') ctx.rect(x - r, y - r, r * 2, r * 2)
  else if (shape === 'diamond') { ctx.moveTo(x, y - r - 1); ctx.lineTo(x + r + 1, y); ctx.lineTo(x, y + r + 1); ctx.lineTo(x - r - 1, y); ctx.closePath() }
  else ctx.arc(x, y, shape === 'dot' ? r - 1 : r, 0, Math.PI * 2)
}

/** 80 m radar around the hero: rings every 20 m, front cone, road users with speed. */
export function Minimap({ list, label, height = 280 }: { list: Nearby[]; label: string; height?: number }) {
  const ref = useCanvas((ctx, w, h) => {
    const cx = w / 2, cy = h / 2, R = 80
    const sc = Math.min(w, h) / 2 / R
    ctx.lineWidth = 1
    ctx.font = font(10)
    for (const r of [20, 40, 60, 80]) {
      ctx.strokeStyle = r === 80 ? C.axis : C.grid
      ctx.beginPath(); ctx.arc(cx, cy, r * sc, 0, Math.PI * 2); ctx.stroke()
      ctx.fillStyle = C.mute
      ctx.fillText(`${r}m`, cx + 4, cy - r * sc + 12)
    }
    ctx.strokeStyle = C.grid
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke()
    // front cone (±20°, 60 m) / 前方コーン
    const ang = (20 * Math.PI) / 180
    ctx.fillStyle = C.info
    ctx.globalAlpha = 0.12
    ctx.beginPath(); ctx.moveTo(cx, cy)
    ctx.lineTo(cx + Math.sin(-ang) * 60 * sc, cy - Math.cos(-ang) * 60 * sc)
    ctx.lineTo(cx + Math.sin(ang) * 60 * sc, cy - Math.cos(ang) * 60 * sc)
    ctx.closePath(); ctx.fill()
    ctx.globalAlpha = 1
    for (const v of list) {
      const x = cx + v.rx * sc, y = cy - v.fx * sc
      const k = KIND[v.kind] ?? KIND.car
      ctx.fillStyle = k.color
      mark(ctx, x, y, k.shape, 4)
      ctx.fill()
      ctx.strokeStyle = C.bg
      ctx.lineWidth = 2
      ctx.stroke()
      if (v.kmh !== undefined) {
        ctx.fillStyle = C.fg2
        ctx.font = font(10)
        ctx.fillText(String(Math.round(v.kmh)), x + 7, y - 4)
      }
    }
    // hero / 自車
    ctx.fillStyle = C.accent
    ctx.strokeStyle = C.bg
    ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(cx, cy - 9); ctx.lineTo(cx - 6, cy + 7); ctx.lineTo(cx + 6, cy + 7); ctx.closePath()
    ctx.fill(); ctx.stroke()
  }, [list])
  return <canvas ref={ref} role="img" aria-label={label} className="block w-full" style={{ height }} />
}

/** Top view of the lane with the car at its lateral offset. */
export function LaneVis({ width, offset, onRoad, label }: { width: number; offset: number; onRoad: boolean; label: string }) {
  const ref = useCanvas((ctx, w, h) => {
    const lw = width || 3.5
    const cx = w / 2
    const scale = (w * 0.8) / Math.max(lw, 3.5)
    const x0 = cx - (lw * scale) / 2
    ctx.fillStyle = C.card
    ctx.fillRect(x0, 4, lw * scale, h - 8)
    ctx.strokeStyle = C.fg2
    ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(x0, 4); ctx.lineTo(x0, h - 4); ctx.moveTo(x0 + lw * scale, 4); ctx.lineTo(x0 + lw * scale, h - 4); ctx.stroke()
    ctx.strokeStyle = C.axis
    ctx.lineWidth = 1
    ctx.setLineDash([6, 6])
    ctx.beginPath(); ctx.moveTo(cx, 6); ctx.lineTo(cx, h - 6); ctx.stroke()
    ctx.setLineDash([])
    const off = clamp(offset || 0, -lw / 2, lw / 2)
    const vx = cx + off * scale
    ctx.fillStyle = onRoad ? C.accent : C.danger
    ctx.beginPath()
    ctx.roundRect(vx - 9, h / 2 - 13, 18, 26, 4)
    ctx.fill()
  }, [width, offset, onRoad])
  return <canvas ref={ref} role="img" aria-label={label} className="block w-full" style={{ height: 64 }} />
}

/** World X/Y path history, fitted to the box. */
export function Trail({ pts, label, height = 220 }: { pts: [number, number][]; label: string; height?: number }) {
  const ref = useCanvas((ctx, w, h) => {
    if (!pts.length) return
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
    for (const [x, y] of pts) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y) }
    const pad = 16
    const dx = maxX - minX || 1, dy = maxY - minY || 1
    const sc = Math.min((w - pad * 2) / dx, (h - pad * 2) / dy)
    // centre the path in the box / 中央に配置
    const ox = (w - dx * sc) / 2, oy = (h - dy * sc) / 2
    const X = (x: number) => ox + (x - minX) * sc, Y = (y: number) => oy + (y - minY) * sc
    ctx.strokeStyle = C.s1
    ctx.lineWidth = 2
    ctx.lineJoin = 'round'
    ctx.beginPath()
    pts.forEach(([x, y], i) => { if (i === 0) ctx.moveTo(X(x), Y(y)); else ctx.lineTo(X(x), Y(y)) })
    ctx.stroke()
    const [fx, fy] = pts[0]
    ctx.fillStyle = C.mute
    ctx.beginPath(); ctx.arc(X(fx), Y(fy), 3, 0, Math.PI * 2); ctx.fill()
    const [lx, ly] = pts[pts.length - 1]
    ctx.fillStyle = C.warn
    ctx.beginPath(); ctx.arc(X(lx), Y(ly), 4.5, 0, Math.PI * 2); ctx.fill()
    ctx.fillStyle = C.mute
    ctx.font = font(10)
    ctx.fillText(`${Math.round(dx)} × ${Math.round(dy)} m`, 6, h - 6)
  }, [pts])
  return <canvas ref={ref} role="img" aria-label={label} className="block w-full" style={{ height }} />
}

/** Collision impulse over the last 10 s, scaled to its own peak. */
export function CollChart({ vals, label, height = 140 }: { vals: number[]; label: string; height?: number }) {
  const ref = useCanvas((ctx, w, h) => {
    ctx.strokeStyle = C.grid
    ctx.lineWidth = 1
    ctx.beginPath(); ctx.moveTo(0, h - 6.5); ctx.lineTo(w, h - 6.5); ctx.stroke()
    if (!vals.length) return
    const maxV = Math.max(1, ...vals)
    ctx.fillStyle = C.mute
    ctx.font = font(10)
    ctx.fillText(maxV.toFixed(1), 4, 12)
    ctx.strokeStyle = C.danger
    ctx.lineWidth = 2
    ctx.beginPath()
    vals.forEach((v, i) => {
      const x = (i / (vals.length - 1 || 1)) * w
      const y = h - (v / maxV) * h * 0.85 - 6
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
    })
    ctx.stroke()
  }, [vals])
  return <canvas ref={ref} role="img" aria-label={label} className="block w-full" style={{ height }} />
}
