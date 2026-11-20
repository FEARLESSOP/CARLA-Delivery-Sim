// Cursor-following SVG charts for the Paper 1 results page: every mark highlights
// under the pointer and shows its exact numbers; legends highlight / hide a series.
// Paper 1 結果ページの図（カーソルに反応：強調表示と数値のツールチップ、凡例で強調・非表示）
import { useLayoutEffect, useMemo, useRef, useState, type ReactNode } from 'react'
import { C, FONT_MONO, FONT_SANS } from '@/lib/colors'

export function useWidth<T extends HTMLElement>(): [React.RefObject<T | null>, number] {
  const ref = useRef<T>(null)
  const [w, setW] = useState(0)
  useLayoutEffect(() => {
    const el = ref.current
    if (!el) return
    setW(el.clientWidth)
    const ro = new ResizeObserver(([e]) => setW(e.contentRect.width))
    ro.observe(el)
    return () => ro.disconnect()
  }, [])
  return [ref, w]
}

export function niceTicks(min: number, max: number, count = 5): number[] {
  if (!isFinite(min) || !isFinite(max)) return []
  if (min === max) { min -= 1; max += 1 }
  const raw = (max - min) / Math.max(1, count)
  const mag = Math.pow(10, Math.floor(Math.log10(raw)))
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? raw
  // ticks cover the whole range (first <= min, last >= max), so no mark falls off the axis
  // 目盛りは範囲全体を覆う（点が軸の外に出ない）
  const out: number[] = []
  const first = Math.floor(min / step + 1e-9), last = Math.ceil(max / step - 1e-9)
  for (let i = first; i <= last && out.length < 50; i++) { const v = i * step; out.push(Math.abs(v) < step * 1e-9 ? 0 : v) }
  return out
}

export const fmtN = (v: number | null | undefined, d = 1, sign = false) =>
  v == null || !isFinite(v) ? '–' : `${sign && v > 0 ? '+' : ''}${v.toFixed(d)}`

interface TipState { x: number; y: number; content: ReactNode }

function Tip({ tip, width }: { tip: TipState | null; width: number }) {
  if (!tip) return null
  const left = Math.min(Math.max(8, tip.x + 14), Math.max(8, width - 260))
  return (
    <div role="tooltip" className="pointer-events-none absolute z-20 max-w-[260px] rounded-lg border border-line-strong bg-card-2/95 px-3 py-2 text-xs text-fg shadow-[var(--shadow-card)] backdrop-blur"
      style={{ left, top: Math.max(4, tip.y - 12) }}>{tip.content}</div>
  )
}

function rel(ev: React.PointerEvent, el: HTMLElement | null) {
  const r = el?.getBoundingClientRect()
  return r ? { x: ev.clientX - r.left, y: ev.clientY - r.top } : { x: 0, y: 0 }
}

export interface LegendItem { key: string; label: string; color: string }
export function Legend({ items, hover, hidden, onHover, onToggle }: {
  items: LegendItem[]; hover: string | null; hidden: Set<string>; onHover: (k: string | null) => void; onToggle: (k: string) => void
}) {
  return (
    <div className="flex flex-wrap gap-x-3 gap-y-1.5 text-xs">
      {items.map((it) => (
        <button key={it.key} type="button" onMouseEnter={() => onHover(it.key)} onMouseLeave={() => onHover(null)}
          onFocus={() => onHover(it.key)} onBlur={() => onHover(null)} onClick={() => onToggle(it.key)}
          aria-pressed={!hidden.has(it.key)}
          className={`inline-flex items-center gap-1.5 rounded-md px-1.5 py-0.5 transition-opacity hover:bg-muted ${hidden.has(it.key) ? 'opacity-40 line-through' : hover && hover !== it.key ? 'opacity-50' : ''}`}>
          <span className="h-2.5 w-2.5 rounded-full" style={{ background: it.color }} />
          <span className="text-fg-2">{it.label}</span>
        </button>
      ))}
    </div>
  )
}

// ---- forest plot: differences with 95 % intervals / 差と 95 % 区間 ----
export interface ForestRow { key: string; label: string; color: string; diff: number; lo: number; hi: number; sig: boolean; n: number; note?: string }

export function ForestPlot({ rows, unit, digits = 2, lowerBetter = false, aria, labelWidth = 90 }: {
  rows: ForestRow[]; unit: string; digits?: number; lowerBetter?: boolean; aria: string; labelWidth?: number
}) {
  const [ref, w] = useWidth<HTMLDivElement>()
  const [hi, setHi] = useState<number | null>(null)
  const [tip, setTip] = useState<TipState | null>(null)
  const rowH = 34, mT = 8, mB = 28, mR = 64
  const H = mT + mB + rowH * rows.length
  const ext = Math.max(1e-9, ...rows.flatMap((r) => [Math.abs(r.lo), Math.abs(r.hi)])) * 1.1
  const ticks = niceTicks(-ext, ext, w < 420 ? 4 : 6)
  const lo = Math.min(-ext, ticks[0] ?? -ext), hiV = Math.max(ext, ticks[ticks.length - 1] ?? ext)
  const W = Math.max(280, w)
  const x = (v: number) => labelWidth + ((v - lo) / (hiV - lo)) * (W - labelWidth - mR)
  const tone = (r: ForestRow) => (!r.sig ? C.mute : (lowerBetter ? r.diff < 0 : r.diff > 0) ? C.accent : C.danger)
  const move = (ev: React.PointerEvent) => {
    const p = rel(ev, ref.current)
    const i = Math.floor((p.y - mT) / rowH)
    if (i < 0 || i >= rows.length) { setHi(null); setTip(null); return }
    const r = rows[i]
    setHi(i)
    setTip({ x: p.x, y: p.y, content: <>
      <div className="font-semibold">{r.label}</div>
      <div className="num">{fmtN(r.diff, digits, true)} {unit} <span className="text-fg-mute">(95 % {fmtN(r.lo, digits, true)} … {fmtN(r.hi, digits, true)})</span></div>
      <div style={{ color: tone(r) }}>{r.sig ? ((lowerBetter ? r.diff < 0 : r.diff > 0) ? 'better · not luck' : 'worse · not luck') : 'could be luck (interval crosses 0)'}</div>
      <div className="text-fg-mute">{r.n} paired days{r.note ? ` · ${r.note}` : ''}</div></> })
  }
  return (
    <div ref={ref} className="relative w-full" onPointerMove={move} onPointerLeave={() => { setHi(null); setTip(null) }}>
      {w > 0 && <svg width="100%" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={aria} style={{ fontFamily: FONT_SANS }}>
        {ticks.map((t) => <g key={t}>
          <line x1={x(t)} x2={x(t)} y1={mT} y2={H - mB} stroke={t === 0 ? C.fg2 : C.grid} strokeWidth={t === 0 ? 1.4 : 1} />
          <text x={x(t)} y={H - 9} textAnchor="middle" fontSize={11} fill={C.mute} fontFamily={FONT_MONO}>{t > 0 ? '+' : ''}{+t.toFixed(4)}</text>
        </g>)}
        {rows.map((r, i) => {
          const y = mT + rowH * i + rowH / 2
          const dim = hi != null && hi !== i
          return <g key={r.key} opacity={dim ? 0.35 : 1}>
            {hi === i && <rect x={0} y={y - rowH / 2} width={W} height={rowH} fill={C.line} opacity={0.35} rx={6} />}
            <text x={6} y={y + 4} fontSize={12} fill={C.fg} fontWeight={600}>{r.label}</text>
            <line x1={x(r.lo)} x2={x(r.hi)} y1={y} y2={y} stroke={r.color} strokeWidth={hi === i ? 4 : 2.6} strokeLinecap="round" />
            <circle cx={x(r.diff)} cy={y} r={hi === i ? 7 : 5.5} fill={r.sig ? r.color : C.card} stroke={r.color} strokeWidth={2} />
            <text x={W - 6} y={y + 4} textAnchor="end" fontSize={12} fill={tone(r)} fontFamily={FONT_MONO}>{fmtN(r.diff, digits, true)}</text>
          </g>
        })}
      </svg>}
      <Tip tip={tip} width={w} />
    </div>
  )
}

// ---- dot plot: one value per category and series / カテゴリ × 系列 ----
export interface DotSeries { key: string; label: string; color: string; values: (number | null)[] }

export function DotPlot({ cats, series, unit, digits = 1, aria, height = 320, lowerBetter = false }: {
  cats: string[]; series: DotSeries[]; unit: string; digits?: number; aria: string; height?: number; lowerBetter?: boolean
}) {
  const [ref, w] = useWidth<HTMLDivElement>()
  const [hover, setHover] = useState<string | null>(null)
  const [hidden, setHidden] = useState<Set<string>>(new Set())
  const [hc, setHc] = useState<number | null>(null)
  const [tip, setTip] = useState<TipState | null>(null)
  const vis = series.filter((s) => !hidden.has(s.key))
  const vals = vis.flatMap((s) => s.values.filter((v): v is number => v != null))
  const mn = Math.min(...vals), mx = Math.max(...vals)
  const pad = (mx - mn) * 0.12 || 1
  const ticks = niceTicks(mn - pad, mx + pad, 5)
  const y0 = ticks[0] ?? mn, y1 = ticks[ticks.length - 1] ?? mx
  const W = Math.max(300, w), H = height, mL = 54, mR = 10, mT = 10, mB = 30
  const band = (W - mL - mR) / Math.max(1, cats.length)
  const y = (v: number) => mT + (1 - (v - y0) / (y1 - y0 || 1)) * (H - mT - mB)
  const xd = (ci: number, si: number) => mL + band * ci + band * (si + 1) / (vis.length + 1)
  const move = (ev: React.PointerEvent) => {
    const p = rel(ev, ref.current)
    const ci = Math.floor((p.x - mL) / band)
    if (ci < 0 || ci >= cats.length || p.y > H - mB + 4) { setHc(null); setTip(null); return }
    setHc(ci)
    let near: string | null = null, best = 1e9
    vis.forEach((s, si) => { const v = s.values[ci]; if (v == null) return; const d = Math.hypot(p.x - xd(ci, si), p.y - y(v)); if (d < best) { best = d; near = s.key } })
    const nearKey = best < 26 ? near : null
    setHover(nearKey)
    const sorted = vis.map((s) => ({ s, v: s.values[ci] })).filter((o) => o.v != null)
      .sort((a, b) => (lowerBetter ? a.v! - b.v! : b.v! - a.v!))
    setTip({ x: p.x, y: p.y, content: <>
      <div className="mb-1 font-semibold">{cats[ci]}</div>
      {sorted.map(({ s, v }) => <div key={s.key} className={`flex justify-between gap-4 ${nearKey === s.key ? 'font-semibold text-fg' : 'text-fg-2'}`}>
        <span className="inline-flex items-center gap-1.5"><span className="h-2 w-2 rounded-full" style={{ background: s.color }} />{s.label}</span>
        <span className="num">{fmtN(v, digits)} {unit}</span></div>)}</> })
  }
  return (
    <div className="flex flex-col gap-2">
      <Legend items={series} hover={hover} hidden={hidden} onHover={setHover}
        onToggle={(k) => setHidden((h) => { const n = new Set(h); if (n.has(k)) n.delete(k); else n.add(k); return n })} />
      <div ref={ref} className="relative w-full" onPointerMove={move} onPointerLeave={() => { setHc(null); setTip(null); setHover(null) }}>
        {w > 0 && vals.length > 0 && <svg width="100%" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={aria} style={{ fontFamily: FONT_SANS }}>
          {ticks.map((t) => <g key={t}><line x1={mL} x2={W - mR} y1={y(t)} y2={y(t)} stroke={C.grid} />
            <text x={mL - 6} y={y(t) + 4} textAnchor="end" fontSize={11} fill={C.mute} fontFamily={FONT_MONO}>{+t.toFixed(3)}</text></g>)}
          {hc != null && <rect x={mL + band * hc} y={mT} width={band} height={H - mT - mB} fill={C.line} opacity={0.3} rx={6} />}
          {cats.map((c, ci) => <text key={c} x={mL + band * ci + band / 2} y={H - 10} textAnchor="middle" fontSize={11}
            fill={hc === ci ? C.fg : C.mute}>{band < 70 ? `S${ci + 1}` : c}</text>)}
          {vis.map((s, si) => s.values.map((v, ci) => v == null ? null : (
            <g key={s.key + ci} opacity={hover && hover !== s.key ? 0.25 : 1}>
              <line x1={xd(ci, si)} x2={xd(ci, si)} y1={y(v)} y2={H - mB} stroke={s.color} strokeOpacity={0.25} strokeWidth={2} />
              <circle cx={xd(ci, si)} cy={y(v)} r={hover === s.key ? 6 : 4.5} fill={s.color} />
            </g>)))}
        </svg>}
        <Tip tip={tip} width={w} />
      </div>
    </div>
  )
}

// ---- line chart with a crosshair / 十字線つき折れ線 ----
export interface LineSeries { key: string; label: string; color: string; dash?: boolean; points: { x: number; y: number }[] }

export function LineChart({ series, xLabel, unit, digits = 1, aria, height = 300, xFmt }: {
  series: LineSeries[]; xLabel: string; unit: string; digits?: number; aria: string; height?: number; xFmt?: (v: number) => string
}) {
  const [ref, w] = useWidth<HTMLDivElement>()
  const [hover, setHover] = useState<string | null>(null)
  const [hidden, setHidden] = useState<Set<string>>(new Set())
  const [cx, setCx] = useState<number | null>(null)
  const [tip, setTip] = useState<TipState | null>(null)
  const vis = series.filter((s) => !hidden.has(s.key) && s.points.length)
  const xs = useMemo(() => [...new Set(vis.flatMap((s) => s.points.map((p) => p.x)))].sort((a, b) => a - b), [vis])
  const ys = vis.flatMap((s) => s.points.map((p) => p.y))
  const mn = Math.min(...ys), mx = Math.max(...ys), pad = (mx - mn) * 0.1 || 1
  const yt = niceTicks(mn - pad, mx + pad, 5)
  const y0 = yt[0] ?? mn, y1 = yt[yt.length - 1] ?? mx
  const W = Math.max(300, w), H = height, mL = 54, mR = 14, mT = 10, mB = 34
  const x0 = xs[0] ?? 0, x1 = xs[xs.length - 1] ?? 1
  const X = (v: number) => mL + ((v - x0) / (x1 - x0 || 1)) * (W - mL - mR)
  const Y = (v: number) => mT + (1 - (v - y0) / (y1 - y0 || 1)) * (H - mT - mB)
  const xt = niceTicks(x0, x1, w < 500 ? 3 : 6)
  const fx = xFmt ?? ((v: number) => String(v))
  const move = (ev: React.PointerEvent) => {
    const p = rel(ev, ref.current)
    if (!xs.length || p.x < mL - 10 || p.x > W - mR + 10) { setCx(null); setTip(null); return }
    let nx = xs[0]
    for (const v of xs) if (Math.abs(X(v) - p.x) < Math.abs(X(nx) - p.x)) nx = v
    setCx(nx)
    const at = vis.map((s) => ({ s, pt: s.points.find((q) => q.x === nx) })).filter((o) => o.pt)
    let near: string | null = null, best = 1e9
    at.forEach(({ s, pt }) => { const d = Math.abs(Y(pt!.y) - p.y); if (d < best) { best = d; near = s.key } })
    const nearKey = best < 18 ? near : null
    setHover(nearKey)
    setTip({ x: p.x, y: p.y, content: <>
      <div className="mb-1 font-semibold">{xLabel} {fx(nx)}</div>
      {at.sort((a, b) => b.pt!.y - a.pt!.y).map(({ s, pt }) => <div key={s.key} className={`flex justify-between gap-4 ${nearKey === s.key ? 'font-semibold text-fg' : 'text-fg-2'}`}>
        <span className="inline-flex items-center gap-1.5"><span className="h-2 w-2 rounded-full" style={{ background: s.color }} />{s.label}</span>
        <span className="num">{fmtN(pt!.y, digits)} {unit}</span></div>)}</> })
  }
  return (
    <div className="flex flex-col gap-2">
      <Legend items={series} hover={hover} hidden={hidden} onHover={setHover}
        onToggle={(k) => setHidden((h) => { const n = new Set(h); if (n.has(k)) n.delete(k); else n.add(k); return n })} />
      <div ref={ref} className="relative w-full" onPointerMove={move} onPointerLeave={() => { setCx(null); setTip(null); setHover(null) }}>
        {w > 0 && ys.length > 0 && <svg width="100%" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={aria} style={{ fontFamily: FONT_SANS }}>
          {yt.map((t) => <g key={t}><line x1={mL} x2={W - mR} y1={Y(t)} y2={Y(t)} stroke={C.grid} />
            <text x={mL - 6} y={Y(t) + 4} textAnchor="end" fontSize={11} fill={C.mute} fontFamily={FONT_MONO}>{+t.toFixed(3)}</text></g>)}
          {xt.map((t) => <text key={t} x={X(t)} y={H - 12} textAnchor="middle" fontSize={11} fill={C.mute} fontFamily={FONT_MONO}>{fx(t)}</text>)}
          {vis.map((s) => <polyline key={s.key} fill="none" stroke={s.color} strokeWidth={hover === s.key ? 3.2 : 1.8}
            strokeDasharray={s.dash ? '5 4' : undefined} opacity={hover && hover !== s.key ? 0.2 : 0.95}
            points={s.points.map((p) => `${X(p.x)},${Y(p.y)}`).join(' ')} />)}
          {cx != null && <line x1={X(cx)} x2={X(cx)} y1={mT} y2={H - mB} stroke={C.fg2} strokeDasharray="3 3" />}
          {cx != null && vis.map((s) => { const p = s.points.find((q) => q.x === cx); return p ? <circle key={s.key} cx={X(p.x)} cy={Y(p.y)}
            r={hover === s.key ? 5.5 : 3.5} fill={s.color} stroke={C.bg} strokeWidth={1.5} opacity={hover && hover !== s.key ? 0.35 : 1} /> : null })}
        </svg>}
        <Tip tip={tip} width={w} />
      </div>
    </div>
  )
}

// ---- horizontal (stacked) bars / 横棒（積み上げ） ----
export interface BarSeg { key: string; label: string; color: string; value: number }
export interface BarRow { key: string; label: string; segs: BarSeg[]; note?: string }

export function Bars({ rows, unit, digits = 1, aria, labelWidth = 110 }: { rows: BarRow[]; unit: string; digits?: number; aria: string; labelWidth?: number }) {
  const [ref, w] = useWidth<HTMLDivElement>()
  const [hk, setHk] = useState<string | null>(null)
  const [tip, setTip] = useState<TipState | null>(null)
  const rowH = 30, mT = 4, mB = 22, mR = 56
  const H = mT + mB + rowH * rows.length
  const W = Math.max(280, w)
  const max = Math.max(1e-9, ...rows.map((r) => r.segs.reduce((a, s) => a + s.value, 0)))
  const ticks = niceTicks(0, max, w < 420 ? 3 : 5)
  const top = Math.max(max, ticks[ticks.length - 1] ?? max)
  const X = (v: number) => labelWidth + (v / top) * (W - labelWidth - mR)
  const move = (ev: React.PointerEvent) => {
    const p = rel(ev, ref.current)
    const i = Math.floor((p.y - mT) / rowH)
    const r = rows[i]
    if (!r) { setHk(null); setTip(null); return }
    let acc = 0, seg: BarSeg | null = null
    for (const s of r.segs) { if (p.x >= X(acc) && p.x <= X(acc + s.value)) seg = s; acc += s.value }
    setHk(`${r.key}|${seg?.key ?? ''}`)
    const total = r.segs.reduce((a, s) => a + s.value, 0)
    setTip({ x: p.x, y: p.y, content: <>
      <div className="mb-1 font-semibold">{r.label}</div>
      {r.segs.map((s) => <div key={s.key} className={`flex justify-between gap-4 ${seg?.key === s.key ? 'font-semibold text-fg' : 'text-fg-2'}`}>
        <span className="inline-flex items-center gap-1.5"><span className="h-2 w-2 rounded-sm" style={{ background: s.color }} />{s.label}</span>
        <span className="num">{fmtN(s.value, digits)} {unit}</span></div>)}
      {r.segs.length > 1 && <div className="mt-1 flex justify-between gap-4 border-t border-line pt-1 text-fg-mute"><span>total</span><span className="num">{fmtN(total, digits)} {unit}</span></div>}
      {r.note && <div className="mt-1 text-fg-mute">{r.note}</div>}</> })
  }
  return (
    <div ref={ref} className="relative w-full" onPointerMove={move} onPointerLeave={() => { setHk(null); setTip(null) }}>
      {w > 0 && <svg width="100%" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={aria} style={{ fontFamily: FONT_SANS }}>
        {ticks.map((t) => <g key={t}><line x1={X(t)} x2={X(t)} y1={mT} y2={H - mB} stroke={C.grid} />
          <text x={X(t)} y={H - 6} textAnchor="middle" fontSize={11} fill={C.mute} fontFamily={FONT_MONO}>{+t.toFixed(3)}</text></g>)}
        {rows.map((r, i) => {
          const y = mT + rowH * i
          let acc = 0
          const rowHot = hk?.startsWith(r.key + '|')
          const total = r.segs.reduce((a, s) => a + s.value, 0)
          return <g key={r.key} opacity={hk && !rowHot ? 0.4 : 1}>
            <text x={6} y={y + rowH / 2 + 4} fontSize={12} fill={C.fg} fontWeight={600}>{r.label}</text>
            {r.segs.map((s) => { const x0 = X(acc); acc += s.value; const hot = hk === `${r.key}|${s.key}`
              return <rect key={s.key} x={x0} y={y + 6} width={Math.max(0, X(acc) - x0)} height={rowH - 12} rx={3} fill={s.color}
                stroke={hot ? C.fg : 'none'} strokeWidth={1.5} /> })}
            <text x={X(total) + 6} y={y + rowH / 2 + 4} fontSize={11} fill={C.fg2} fontFamily={FONT_MONO}>{fmtN(total, digits)}</text>
          </g>
        })}
      </svg>}
      <Tip tip={tip} width={w} />
    </div>
  )
}
