// The City Map card: canvas (MapEngine), floating legend, zoom / fit /
// fullscreen controls, hover tooltips. Wheel zooms at the cursor, drag (or one
// finger) pans, two fingers pinch, double-click fits; arrow keys / + / - / 0
// when the map has focus.
// 市街地図：ホイールでカーソル位置ズーム、ドラッグで移動、ダブルクリックで全体
import { useEffect, useRef, useState, type KeyboardEvent as ReactKeyboardEvent, type ReactNode } from 'react'
import { Expand, Layers, Minus, Plus, Scan, Shrink } from 'lucide-react'
import { getJSON } from '@/lib/api'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge } from '@/components/ui'
import { MapEngine } from './mapEngine'
import { DRONE_COL, MC } from './shared'
import type { DeliveryMap, DeliveryState } from './types'

const WHEEL_K = 1.18

/** Load /delivery/map once per map (when the state names a new one). */
function useDeliveryMap(state: DeliveryState | null): DeliveryMap | null {
  const [map, setMap] = useState<DeliveryMap | null>(null)
  const loading = useRef(false)
  const want = state?.available ? state.map : undefined
  useEffect(() => {
    if (!want || (map && map.map === want) || loading.current) return
    loading.current = true
    getJSON<DeliveryMap>('/delivery/map')
      .then((m) => { if (m && Array.isArray(m.roads)) setMap(m) })
      .catch(() => { /* not written yet: retried on the next state / 次の状態で再試行 */ })
      .finally(() => { loading.current = false })
    // re-run on every state so a failed load retries / 失敗時は次の状態で再試行
  }, [want, map, state])
  return map
}

type Shape = 'sq' | 'circle' | 'ring' | 'diamond' | 'box' | 'tall'
function Swatch({ shape, fill, edge }: { shape: Shape; fill: string; edge?: string }) {
  const base = 'inline-block shrink-0'
  if (shape === 'circle') return <i className={cn(base, 'h-2.5 w-2.5 rounded-full')} style={{ background: fill, boxShadow: edge ? `0 0 0 2px ${edge}` : undefined }} />
  if (shape === 'ring') return <i className={cn(base, 'h-2.5 w-2.5 rounded-full')} style={{ background: fill, boxShadow: `0 0 0 1.5px ${MC.ring}` }} />
  if (shape === 'diamond') return <i className={cn(base, 'h-2 w-2 rotate-45 border-2')} style={{ borderColor: fill }} />
  if (shape === 'box') return <i className={cn(base, 'h-2 w-3.5 rounded-[2px] border-2')} style={{ background: fill, borderColor: edge ?? fill }} />
  if (shape === 'tall') return <i className={cn(base, 'h-3 w-2 rounded-[1px]')} style={{ background: fill }} />
  return <i className={cn(base, 'h-2.5 w-2.5 rounded-[2px] border')} style={{ background: `${fill}33`, borderColor: fill }} />
}

function Legend() {
  const t = useT()
  const items: [Shape, string, string, string?][] = [
    ['box', MC.pod, 'dr_lg_pod', MC.battGood],
    ['box', MC.car, 'dr_lg_car'],
    ['circle', MC.battMid, 'dr_lg_drone', DRONE_COL.delivering],
    ['sq', MC.station, 'dr_lg_station'],
    ['sq', MC.ev, 'dr_lg_ev'],
    ['tall', MC.box, 'dr_lg_box'],
    ['circle', MC.f2f, 'dr_lg_f2f'],
    ['circle', MC.droneOrder, 'dr_lg_dr'],
    ['ring', MC.droneOrder, 'dr_lg_waiting'],
    ['diamond', MC.hub, 'dr_lg_hub'],
    ['circle', MC.done, 'dr_lg_done'],
    ['circle', MC.fail, 'dr_lg_fail'],
  ]
  return (
    <ul className="grid gap-x-4 gap-y-1 text-xs text-fg-2 sm:grid-cols-2">
      {items.map(([shape, fill, key, edge]) => (
        <li key={key} className="flex items-center gap-2">
          <Swatch shape={shape} fill={fill} edge={edge} />{t(key)}
        </li>
      ))}
    </ul>
  )
}

function MapButton({ label, onClick, children, pressed }: { label: string; onClick: () => void; children: ReactNode; pressed?: boolean }) {
  return (
    <button type="button" aria-label={label} title={label} aria-pressed={pressed} onClick={onClick}
      className={cn('grid h-10 w-10 place-items-center rounded-lg border border-line bg-surface/85 text-fg-2 shadow-lg backdrop-blur',
        'transition-colors duration-150 hover:bg-muted hover:text-fg', pressed && 'border-accent/60 text-accent')}>
      {children}
    </button>
  )
}

export default function CityMap({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const map = useDeliveryMap(state)
  const wrap = useRef<HTMLDivElement>(null)
  const cvRef = useRef<HTMLCanvasElement>(null)
  const tipRef = useRef<HTMLDivElement>(null)
  const [engine] = useState(() => new MapEngine())
  // phones start with the legend folded / スマホでは凡例を畳んで開始
  const [legend, setLegend] = useState(() => typeof window === 'undefined' || window.innerWidth >= 640)
  const [full, setFull] = useState(false)
  const [grab, setGrab] = useState(false)

  useEffect(() => { engine.setT(t) }, [engine, t])
  useEffect(() => { engine.setMap(map) }, [engine, map])
  useEffect(() => { engine.setState(state) }, [engine, state])

  useEffect(() => {
    const cv = cvRef.current, e = engine
    if (!cv) return
    e.attach(cv)
    if (import.meta.env.DEV) (window as unknown as { __drMap?: MapEngine }).__drMap = e
    const tip = tipRef.current!
    const hideTip = () => { tip.style.display = 'none' }
    const local = (ev: { clientX: number; clientY: number }) => {
      const r = cv.getBoundingClientRect()
      return { x: ev.clientX - r.left, y: ev.clientY - r.top, w: r.width, h: r.height }
    }

    const onWheel = (ev: WheelEvent) => {
      if (!e.hasView) return
      ev.preventDefault()
      const p = local(ev)
      e.zoomAt(ev.deltaY < 0 ? WHEEL_K : 1 / WHEEL_K, p.x - p.w / 2, p.y - p.h / 2)
    }

    // pointers: one = pan, two = pinch / 1 本でパン、2 本でピンチ
    const pts = new Map<number, { x: number; y: number }>()
    const g = { pinch: 0 }
    const onDown = (ev: PointerEvent) => {
      if (!e.hasView || ev.button > 0) return
      cv.setPointerCapture(ev.pointerId)
      pts.set(ev.pointerId, { x: ev.clientX, y: ev.clientY })
      if (pts.size === 2) { const [a, b] = [...pts.values()]; g.pinch = Math.hypot(a.x - b.x, a.y - b.y) }
      setGrab(true); hideTip()
    }
    const onMove = (ev: PointerEvent) => {
      const prev = pts.get(ev.pointerId)
      if (prev) {
        if (pts.size === 1) {
          e.panBy(ev.clientX - prev.x, ev.clientY - prev.y)
        } else if (pts.size === 2) {
          pts.set(ev.pointerId, { x: ev.clientX, y: ev.clientY })
          const [a, b] = [...pts.values()]
          const d = Math.hypot(a.x - b.x, a.y - b.y)
          const mid = local({ clientX: (a.x + b.x) / 2, clientY: (a.y + b.y) / 2 })
          if (g.pinch > 0 && d > 0) e.zoomAt(d / g.pinch, mid.x - mid.w / 2, mid.y - mid.h / 2)
          g.pinch = d
        }
        pts.set(ev.pointerId, { x: ev.clientX, y: ev.clientY })
        return
      }
      if (ev.pointerType !== 'mouse') return
      const p = local(ev)
      const hit = e.hitTest(p.x, p.y)
      if (!hit) { hideTip(); return }
      tip.textContent = hit
      tip.style.display = 'block'
      tip.style.left = `${Math.max(4, Math.min(p.x + 14, p.w - tip.offsetWidth - 4))}px`
      tip.style.top = `${Math.min(p.y + 14, p.h - tip.offsetHeight - 4)}px`
    }
    const onUp = (ev: PointerEvent) => {
      pts.delete(ev.pointerId)
      if (pts.size < 2) g.pinch = 0
      if (!pts.size) setGrab(false)
    }
    const onDbl = () => e.fit()

    cv.addEventListener('wheel', onWheel, { passive: false })
    cv.addEventListener('pointerdown', onDown)
    cv.addEventListener('pointermove', onMove)
    cv.addEventListener('pointerup', onUp)
    cv.addEventListener('pointercancel', onUp)
    cv.addEventListener('pointerleave', hideTip)
    cv.addEventListener('dblclick', onDbl)
    return () => {
      cv.removeEventListener('wheel', onWheel)
      cv.removeEventListener('pointerdown', onDown)
      cv.removeEventListener('pointermove', onMove)
      cv.removeEventListener('pointerup', onUp)
      cv.removeEventListener('pointercancel', onUp)
      cv.removeEventListener('pointerleave', hideTip)
      cv.removeEventListener('dblclick', onDbl)
      e.detach()
    }
  }, [engine])

  useEffect(() => {
    const on = () => setFull(document.fullscreenElement === wrap.current)
    document.addEventListener('fullscreenchange', on)
    return () => document.removeEventListener('fullscreenchange', on)
  }, [])
  const toggleFull = () => {
    if (document.fullscreenElement) void document.exitFullscreen()
    else void wrap.current?.requestFullscreen?.()
  }

  const onKey = (ev: ReactKeyboardEvent) => {
    const e = engine, step = 60
    const act: Record<string, () => void> = {
      ArrowLeft: () => e.panBy(step, 0), ArrowRight: () => e.panBy(-step, 0),
      ArrowUp: () => e.panBy(0, step), ArrowDown: () => e.panBy(0, -step),
      '+': () => e.zoomAt(WHEEL_K * WHEEL_K), '=': () => e.zoomAt(WHEEL_K * WHEEL_K),
      '-': () => e.zoomAt(1 / (WHEEL_K * WHEEL_K)), '0': () => e.fit(),
    }
    const f = act[ev.key]
    if (f) { ev.preventDefault(); f() }
  }

  const mapName = (state?.map || map?.map || '').split('/').pop() || '–'

  return (
    <div ref={wrap} className={cn('relative overflow-hidden rounded-[var(--radius-card)] border border-line bg-surface shadow-[var(--shadow-card)]',
      full ? 'h-screen w-screen rounded-none' : 'h-[62vh] min-h-[380px] lg:h-[calc(100vh-15rem)] lg:min-h-[560px] lg:max-h-[1400px]')}>
      <canvas ref={cvRef} tabIndex={0} role="img" aria-label={`${t('dr_map')}: ${mapName}. ${t('dr_map_hint')}`}
        onKeyDown={onKey}
        className={cn('block h-full w-full touch-none select-none outline-none focus-visible:ring-2 focus-visible:ring-accent',
          grab ? 'cursor-grabbing' : 'cursor-grab')} />
      <div ref={tipRef} role="tooltip"
        className="pointer-events-none absolute z-20 hidden max-w-[320px] rounded-lg border border-line bg-bg/95 px-2.5 py-1.5 text-xs text-fg shadow-xl num" />

      {/* top-left: title, map, hint / 左上：題名・地図名 */}
      <div className="pointer-events-none absolute left-3 top-3 z-10 flex max-w-[calc(100%-8rem)] flex-wrap items-center gap-2">
        <h3 className="rounded-lg bg-bg/75 px-2.5 py-1 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-2 backdrop-blur">{t('dr_map')}</h3>
        <Badge className="bg-bg/75 backdrop-blur">{mapName}</Badge>
      </div>

      {/* bottom-right: zoom / fit / fullscreen / 右下：操作 */}
      <div className="absolute bottom-3 right-3 z-10 flex flex-col gap-1.5">
        <MapButton label={t('dr_zoom_in')} onClick={() => engine.zoomAt(WHEEL_K * WHEEL_K)}><Plus className="h-4 w-4" aria-hidden /></MapButton>
        <MapButton label={t('dr_zoom_out')} onClick={() => engine.zoomAt(1 / (WHEEL_K * WHEEL_K))}><Minus className="h-4 w-4" aria-hidden /></MapButton>
        <MapButton label={t('dr_fit')} onClick={() => engine.fit()}><Scan className="h-4 w-4" aria-hidden /></MapButton>
        <MapButton label={full ? t('dr_full_exit') : t('dr_full')} onClick={toggleFull}>
          {full ? <Shrink className="h-4 w-4" aria-hidden /> : <Expand className="h-4 w-4" aria-hidden />}
        </MapButton>
      </div>

      {/* bottom-left: legend + hint / 左下：凡例 */}
      <div className="absolute bottom-3 left-3 z-10 max-w-[calc(100%-5rem)]">
        {legend ? (
          <div className="rounded-xl border border-line bg-bg/85 p-3 shadow-xl backdrop-blur-md">
            <div className="mb-2 flex items-center justify-between gap-3">
              <span className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('dr_legend')}</span>
              <button type="button" onClick={() => setLegend(false)} aria-expanded="true"
                className="rounded-md px-1.5 text-xs text-fg-mute hover:bg-muted hover:text-fg">{t('dr_hide')}</button>
            </div>
            <Legend />
            <p className="mt-2 border-t border-line pt-2 text-[11px] text-fg-mute">{t('dr_map_hint')}</p>
          </div>
        ) : (
          <button type="button" onClick={() => setLegend(true)} aria-expanded="false"
            className="flex h-10 items-center gap-2 rounded-lg border border-line bg-surface/85 px-3 text-xs text-fg-2 shadow-lg backdrop-blur hover:bg-muted">
            <Layers className="h-4 w-4" aria-hidden />{t('dr_legend')}
          </button>
        )}
      </div>
    </div>
  )
}
