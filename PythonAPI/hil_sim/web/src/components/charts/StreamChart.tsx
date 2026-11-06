import { useState } from 'react'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { fmt } from '@/lib/fmt'
import { cn } from '@/lib/utils'

export interface ChartSeries {
  label: string
  color: string
  data: number[]
  /** dashed line (also tells series apart without colour) */
  dash?: boolean
}

interface Props {
  series: ChartSeries[]
  min: number
  max: number
  unit?: string
  decimals?: number
  /** history length the x axis spans (samples) */
  len?: number
  height?: number
  className?: string
  /** bump to redraw (e.g. a sample counter) */
  tick?: number
}

const PADL = 38

/** A live line chart of the last ``len`` samples with a hover readout and a
 *  legend (colour + name + latest value, so colour is never the only cue).
 *  直近のサンプルの折れ線（ホバーで値表示、凡例に最新値）。 */
export function StreamChart({ series, min, max, unit = '', decimals = 1, len = 300, height = 150, className, tick }: Props) {
  const [hover, setHover] = useState<number | null>(null)
  const ref = useCanvas((ctx, w, h) => {
    const span = Math.max(1e-6, max - min)
    const y = (v: number) => h - 8 - ((v - min) / span) * (h - 16)
    ctx.strokeStyle = C.grid
    ctx.lineWidth = 1
    ctx.font = `11px ${FONT_MONO}`
    ctx.fillStyle = C.mute
    ctx.textAlign = 'right'
    for (const f of [0, 0.5, 1]) {
      const v = min + f * span, yy = Math.round(y(v)) + 0.5
      ctx.beginPath(); ctx.moveTo(PADL, yy); ctx.lineTo(w - 4, yy); ctx.stroke()
      ctx.fillText(fmt(v, Math.abs(span) < 10 ? 1 : 0), PADL - 6, yy + 4)
    }
    if (min < 0 && max > 0) {
      const yy = Math.round(y(0)) + 0.5
      ctx.strokeStyle = C.axis
      ctx.beginPath(); ctx.moveTo(PADL, yy); ctx.lineTo(w - 4, yy); ctx.stroke()
    }
    const x = (i: number) => PADL + (i / (len - 1)) * (w - PADL - 6)
    for (const s of series) {
      if (!s.data.length) continue
      ctx.strokeStyle = s.color
      ctx.lineWidth = 2
      ctx.lineJoin = 'round'
      ctx.setLineDash(s.dash ? [5, 4] : [])
      ctx.beginPath()
      const off = len - s.data.length
      s.data.forEach((v, i) => {
        const yy = y(Math.max(min, Math.min(max, v)))
        if (i === 0) ctx.moveTo(x(i + off), yy); else ctx.lineTo(x(i + off), yy)
      })
      ctx.stroke()
    }
    ctx.setLineDash([])
    if (hover !== null) {
      const xx = x(hover)
      ctx.strokeStyle = C.mute
      ctx.beginPath(); ctx.moveTo(xx, 4); ctx.lineTo(xx, h - 4); ctx.stroke()
    }
  }, [series, min, max, len, hover, tick])

  const readout = hover === null ? null : series.map((s) => {
    const i = hover - (len - s.data.length)
    return { s, v: i >= 0 && i < s.data.length ? s.data[i] : null }
  })

  return (
    <div className={cn('relative', className)}>
      <canvas
        ref={ref}
        style={{ height }}
        className="w-full block"
        role="img"
        aria-label={series.map((s) => `${s.label} ${s.data.length ? fmt(s.data[s.data.length - 1], decimals) : '–'}${unit}`).join(', ')}
        onMouseMove={(e) => {
          const r = e.currentTarget.getBoundingClientRect()
          const f = (e.clientX - r.left - PADL) / (r.width - PADL - 6)
          setHover(f < 0 || f > 1 ? null : Math.round(f * (len - 1)))
        }}
        onMouseLeave={() => setHover(null)}
      />
      <div className="mt-2 flex flex-wrap gap-x-4 gap-y-1 text-xs text-fg-mute">
        {series.map((s, k) => {
          const v = readout ? readout[k].v : s.data.length ? s.data[s.data.length - 1] : null
          return (
            <span key={s.label} className="inline-flex items-center gap-1.5">
              <i className="inline-block h-0.5 w-3.5 rounded" style={{ background: s.color, opacity: s.dash ? 0.7 : 1 }} />
              {s.label}
              <b className="num text-fg-2 font-medium">{v === null ? '–' : fmt(v, decimals)}{unit}</b>
            </span>
          )
        })}
      </div>
    </div>
  )
}
