// The City Map card: canvas (MapEngine), floating legend, zoom / fit /
// fullscreen controls, hover tooltips. Wheel zooms at the cursor, drag (or one
// finger) pans, two fingers pinch, double-click fits; arrow keys / + / - / 0
// when the map has focus.
// Edit zones mode: pick a pod (or No-fly) and drag to draw its rectangle;
// without a tool, click selects, drag moves, edges / corners resize, empty map
// pans; middle button or Space + drag always pans. Saved on pointer-up.
// Meeting points (mode switch in the tools): pick a pod, click near a road to
// add a point snapped to the nearest stop; drag a pin to move it (re-snapped).
// Distribution points (third mode, LMH hubs only): the same gestures, drawn
// as square "D" badges; the hub parks there and sends its mini pods out.
// Charging stations (fourth mode): click free ground to place a LastMile
// charging station (<= 12 per map), drag one to move it, R / Shift+R or the
// toolbar turns it 15 degrees, Auto lets the sim face it to the kerb; a spot
// in a building or on a road turns red with the reason. #delivery/charge
// opens the editor in this mode (the Launch page links here).
// Layers: show / hide each kind of thing the map draws (saved per browser).
// 市街地図：ホイールでカーソル位置ズーム、ドラッグで移動、ダブルクリックで全体
// ゾーン編集：道具を選んでドラッグで描画、選択・移動・辺や角で大きさ変更
import { useCallback, useEffect, useId, useLayoutEffect, useMemo, useRef, useState,
  type KeyboardEvent as ReactKeyboardEvent, type ReactNode, type Ref } from 'react'
import { Expand, Info, Layers as LayersIcon, Minus, Plus, Scan, Shrink, SquarePen } from 'lucide-react'
import { getJSON } from '@/lib/api'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge } from '@/components/ui'
import { MapEngine } from './mapEngine'
import { DRONE_COL, MAP_CENTRE_EVENT, MC } from './shared'
import { MP_COL, type DeliveryMap, type DeliveryState, type Rect, type ZoneSet } from './types'
import { loadLayers, saveLayers, LAYER_KEYS, type Layers } from './layers'
import { MAX_CHARGE, bigEnough, chargeRef, dragRect, getCharge, getPoint, getRect, isPointKind, mapKey, maxPoints, normRect, refExists, removeRect,
  sameRef, setCharge, setPoint, setRect, useZones, type Handle, type PointKind, type PointRef, type Tool, type ZoneRef } from './zones'
import { ChargeGlyph, LayersMenu, ZoneStatusBadge, ZoneSwatch, ZoneToolbar, type EditMode, type PodChip } from './MapControls'
import { podColor } from './shared'
import { buildSnap } from './snap'
import { SpotChecker, simError } from './stations'

const WHEEL_K = 1.18
const CURSOR: Record<Handle, string> = {
  move: 'move', n: 'ns-resize', s: 'ns-resize', e: 'ew-resize', w: 'ew-resize',
  nw: 'nwse-resize', se: 'nwse-resize', ne: 'nesw-resize', sw: 'nesw-resize',
}

/** what the current drag does / 現在のドラッグの役割 */
type Gesture =
  | { k: 'pan'; moved: number; deselect: boolean; add?: { id: string; kind: PointKind }; addCharge?: boolean }
  | { k: 'charge'; i: number; dx: number; dy: number; moved: number; work: ZoneSet | null }
  | { k: 'meet'; ref: PointRef; moved: number; work: ZoneSet | null }
  | { k: 'draw'; ref: ZoneRef; ax: number; ay: number; work: ZoneSet | null }
  | { k: 'edit'; ref: ZoneRef; h: Handle; start: Rect; wx: number; wy: number; moved: number; work: ZoneSet | null }

/** the page was opened as #delivery/charge (the Launch page's link) / 充電所リンク */
const chargeLink = () => typeof window !== 'undefined' && /\/charge$/.test(window.location.hash)

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
      <li className="flex items-center gap-2"><ZoneSwatch />{t('dr_lg_zone')}</li>
      <li className="flex items-center gap-2"><ZoneSwatch nofly />{t('dr_lg_nofly')}</li>
      <li className="flex items-center gap-2"><ZoneSwatch nofly lmh />{t('dr_lg_nofly_lmh')}</li>
      <li className="flex items-center gap-2"><ZoneSwatch meet />{t('dr_lg_meet')}</li>
      <li className="flex items-center gap-2"><ZoneSwatch dist />{t('dr_lg_dist')}</li>
      <li className="flex items-center gap-2"><ChargeGlyph />{t('dr_lg_charge')}</li>
      <li className="flex items-center gap-2"><ChargeGlyph bad />{t('dr_lg_charge_bad')}</li>
      <li className="flex items-center gap-2"><Swatch shape="circle" fill={podColor(4)} edge={MP_COL.to_door} />{t('dr_lg_minipod')}</li>
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

/** Labelled overlay button (Edit zones, Layers). / 文字付きの地図ボタン */
function OverlayButton({ label, icon, onClick, pressed, expanded, controls, extra, btnRef, ...rest }: {
  label: string; icon: ReactNode; onClick: () => void; pressed?: boolean; expanded?: boolean; controls?: string
  extra?: ReactNode; btnRef?: Ref<HTMLButtonElement>; 'data-layers-btn'?: boolean
}) {
  return (
    <button ref={btnRef} type="button" onClick={onClick} aria-pressed={pressed} aria-expanded={expanded} aria-controls={controls} {...rest}
      className={cn('pointer-events-auto inline-flex h-9 items-center gap-1.5 rounded-lg border border-line bg-surface/85 px-2.5 text-xs font-semibold text-fg-2',
        'shadow-lg backdrop-blur transition-colors duration-150 hover:bg-muted hover:text-fg',
        (pressed || expanded) && 'border-accent/60 text-accent')}>
      {icon}<span>{label}</span>{extra}
    </button>
  )
}

export default function CityMap({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const map = useDeliveryMap(state)
  const wrap = useRef<HTMLDivElement>(null)
  const cvRef = useRef<HTMLCanvasElement>(null)
  const tipRef = useRef<HTMLDivElement>(null)
  const layersBtn = useRef<HTMLButtonElement>(null)
  const [engine] = useState(() => new MapEngine())
  // phones start with the legend folded / スマホでは凡例を畳んで開始
  const [legend, setLegend] = useState(() => typeof window === 'undefined' || window.innerWidth >= 640)
  const [full, setFull] = useState(false)
  const [layers, setLayers] = useState<Layers>(loadLayers)
  const [layersOpen, setLayersOpen] = useState(false)
  const [edit, setEdit] = useState(chargeLink)
  const [tool, setTool] = useState<Tool>(() => (chargeLink() ? { kind: 'charge' } : null))
  const [mode, setMode] = useState<EditMode>(() => (chargeLink() ? 'charge' : 'zone'))
  const [sel, setSel] = useState<ZoneRef | null>(null)
  const toolsId = useId(), layersId = useId()

  const mapName = (state?.map || map?.map || '').split('/').pop() || '–'
  const { zones, status, error, commit, chargeOk } = useZones(mapKey(state?.map || map?.map))

  // pods to offer: running ones, plus any with a saved zone (dimmed)
  // 稼働中のポッド＋ゾーンだけ保存されたポッド（薄く表示）
  // "id:kind" so a hub turning LMH re-renders the chips / LMH かどうかも含める
  const vanIds = (state?.vans || []).map((v) => `${v.id}:${v.kind === 'lmh' ? 'h' : 'p'}:${v.area ?? ''}`).join(',')
  const chips = useMemo<PodChip[]>(() => {
    const vans = vanIds ? vanIds.split(',').map((s) => s.split(':')) : []
    const run = new Set(vans.map(([id]) => id))
    const lmh = new Set(vans.filter(([, k]) => k === 'h').map(([id]) => id))
    const area = new Map(vans.map(([id, , a]) => [id, a]))
    const ids = new Set([...run, ...Object.keys(zones.pods), ...Object.keys(zones.meet), ...Object.keys(zones.dist)])
    return [...ids].sort((a, b) => Number(a) - Number(b))
      .map((id) => ({ id, running: run.has(id), lmh: lmh.has(id) || (!run.has(id) && !!zones.dist[id]), area: area.get(id) || undefined }))
  }, [vanIds, zones.pods, zones.meet, zones.dist])

  // snap targets for meeting points, indexed once per map / 合流地点の吸着先（地図ごとに 1 回）
  const snap = useMemo(() => (map ? buildSnap(map) : null), [map])
  // pre-launch check of charging-station spots, once per map / 充電所地点の確認（地図ごと）
  const checker = useMemo(() => (map ? new SpotChecker(map) : null), [map])

  // the pointer handlers read the latest edit state from here / ハンドラ用の最新状態
  const live = useRef({ edit, tool, sel, zones, commit, setSel, setTool, snap })
  useLayoutEffect(() => { live.current = { edit, tool, sel, zones, commit, setSel, setTool, snap } })

  useEffect(() => { engine.setT(t) }, [engine, t])
  useEffect(() => { engine.setMap(map) }, [engine, map])
  useEffect(() => { engine.setChecker(checker) }, [engine, checker])
  // #delivery/charge (the Launch page's link) while the page is open: the
  // charging-station editor (on first render the initial state does it)
  // 起動ページからのリンクで充電ステーションの編集を開く
  useEffect(() => {
    const on = () => {
      if (!chargeLink()) return
      setEdit(true); setMode('charge'); setTool({ kind: 'charge' }); setSel(null)
    }
    window.addEventListener('hashchange', on)
    return () => window.removeEventListener('hashchange', on)
  }, [])
  // the Charging stations panel asks to centre on a station / 充電ステーション一覧からの移動
  useEffect(() => {
    const on = (ev: Event) => {
      const d = (ev as CustomEvent<{ x: number; y: number }>).detail
      if (!d) return
      engine.centreOn(d.x, d.y)
      // scroll the map into view below the sticky top bar (~56 px) / 上部バーの下へ
      const r = wrap.current?.getBoundingClientRect()
      if (r && (r.top < 56 || r.bottom > window.innerHeight)) window.scrollTo({ top: Math.max(0, r.top + window.scrollY - 64), behavior: 'smooth' })
    }
    window.addEventListener(MAP_CENTRE_EVENT, on)
    return () => window.removeEventListener(MAP_CENTRE_EVENT, on)
  }, [engine])
  useEffect(() => { engine.setState(state) }, [engine, state])
  useEffect(() => { engine.setLayers(layers); saveLayers(layers) }, [engine, layers])
  useEffect(() => { engine.setZoneView(zones, sel, edit) }, [engine, zones, sel, edit])
  // a selection whose zone is gone (deleted, rolled back) is dropped / 消えたゾーンの選択を解除
  useEffect(() => { if (sel && !refExists(zones, sel)) setSel(null) }, [zones, sel])
  useEffect(() => {
    const cv = cvRef.current
    if (cv) cv.style.cursor = !edit ? 'grab' : tool ? 'crosshair' : 'default'
    if (!edit || !tool || !isPointKind(tool.kind)) engine.setSnap(null)
  }, [engine, edit, tool])

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
    /** nearest stop to a world point: within 80 px on screen, at least 12 m
     *  画面上 80 px（最低 12 m）以内の最も近い停車点 */
    const snapAt = (wx: number, wy: number) => live.current.snap?.nearest(wx, wy, Math.max(12, 80 / e.scale)) ?? null
    const preview = (px: number, py: number, pod: string, kind: PointKind) => {
      const [wx, wy] = e.toWorld(px, py), q = snapAt(wx, wy)
      e.setSnap(q ? { cx: wx, cy: wy, sx: q[0], sy: q[1], pod, kind } : null)
    }
    const pointTool = (tl: Tool) => (tl && (tl.kind === 'meet' || tl.kind === 'dist') ? tl : null)
    const idleCursor = () => {
      const L = live.current
      cv.style.cursor = !L.edit ? 'grab' : L.tool ? 'crosshair' : 'default'
    }

    const onWheel = (ev: WheelEvent) => {
      if (!e.hasView) return
      ev.preventDefault()
      const p = local(ev)
      e.zoomAt(ev.deltaY < 0 ? WHEEL_K : 1 / WHEEL_K, p.x - p.w / 2, p.y - p.h / 2)
    }

    // pointers: one = pan or zone gesture, two = pinch / 1 本でパンかゾーン操作、2 本でピンチ
    const pts = new Map<number, { x: number; y: number }>()
    const g: { pinch: number; act: Gesture | null; space: boolean } = { pinch: 0, act: null, space: false }
    const cancelZone = () => {
      const L = live.current
      if (g.act && g.act.k !== 'pan') e.setZoneView(L.zones, L.sel, L.edit)
      g.act = { k: 'pan', moved: 99, deselect: false }
    }
    const onDown = (ev: PointerEvent) => {
      if (!e.hasView || ev.button === 2) return
      if (ev.button === 1) ev.preventDefault()            // no autoscroll / 自動スクロール無効
      cv.setPointerCapture(ev.pointerId)
      pts.set(ev.pointerId, { x: ev.clientX, y: ev.clientY })
      hideTip()
      if (pts.size === 2) {
        const [a, b] = [...pts.values()]; g.pinch = Math.hypot(a.x - b.x, a.y - b.y)
        cancelZone()
        return
      }
      const L = live.current, p = local(ev)
      if (L.edit && ev.button === 0 && !g.space) {
        const [wx, wy] = e.toWorld(p.x, p.y)
        // points first (select / move) unless drawing a zone / ゾーン描画中以外は点を優先
        const pt = pointTool(L.tool)
        const ct = L.tool?.kind === 'charge'
        if (!L.tool || pt || ct) {
          const mh = e.pointAt(p.x, p.y)
          if (mh) {
            if (!sameRef(mh, L.sel)) { L.setSel(mh); e.setZoneView(L.zones, mh, true) }
            g.act = { k: 'meet', ref: mh, moved: 0, work: null }
            e.setSnap(null)
            cv.style.cursor = 'move'
            return
          }
          // a charging station: select it, a drag moves it / 充電所：選択、ドラッグで移動
          const ci = e.chargeAt(p.x, p.y)
          const cs = ci != null ? L.zones.charge[ci] : null
          if (ci != null && cs) {
            const ref = chargeRef(ci)
            if (!sameRef(ref, L.sel)) { L.setSel(ref); e.setZoneView(L.zones, ref, true) }
            g.act = { k: 'charge', i: ci, dx: wx - cs.x, dy: wy - cs.y, moved: 0, work: null }
            cv.style.cursor = 'move'
            return
          }
        }
        if (ct) {
          // a click places a station, a drag pans / クリックで設置、ドラッグで移動
          g.act = { k: 'pan', moved: 0, deselect: false, addCharge: true }
          return
        }
        if (pt) {
          // a click adds a point, a drag pans / クリックで追加、ドラッグで移動
          g.act = { k: 'pan', moved: 0, deselect: false, add: { id: pt.id, kind: pt.kind } }
          return
        }
        const hit = e.zoneAt(p.x, p.y)
        const resize = !!hit && hit.handle !== 'move' && sameRef(hit.ref, L.sel)
        if (L.tool && !resize) {
          const ref: ZoneRef = L.tool.kind === 'pod' ? { kind: 'pod', id: L.tool.id } : { kind: 'nofly', id: `nf${Date.now()}` }
          g.act = { k: 'draw', ref, ax: wx, ay: wy, work: null }
          cv.style.cursor = 'crosshair'
          return
        }
        if (hit) {
          const start = getRect(L.zones, hit.ref)
          if (start) {
            if (!sameRef(hit.ref, L.sel)) { L.setSel(hit.ref); e.setZoneView(L.zones, hit.ref, true) }
            g.act = { k: 'edit', ref: hit.ref, h: hit.handle, start, wx, wy, moved: 0, work: null }
            cv.style.cursor = CURSOR[hit.handle]
            return
          }
        }
        g.act = { k: 'pan', moved: 0, deselect: true }
      } else {
        g.act = { k: 'pan', moved: 0, deselect: false }
      }
      cv.style.cursor = 'grabbing'
    }
    const onMove = (ev: PointerEvent) => {
      const prev = pts.get(ev.pointerId)
      if (prev) {
        if (pts.size === 1 && g.act) {
          const a = g.act, L = live.current
          if (a.k === 'pan') {
            e.panBy(ev.clientX - prev.x, ev.clientY - prev.y)
            a.moved += Math.abs(ev.clientX - prev.x) + Math.abs(ev.clientY - prev.y)
            if (a.add && a.moved >= 4) { e.setSnap(null); cv.style.cursor = 'grabbing' }
          } else if (a.k === 'charge') {
            a.moved += Math.abs(ev.clientX - prev.x) + Math.abs(ev.clientY - prev.y)
            const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y), cs = L.zones.charge[a.i]
            if (cs) {
              a.work = setCharge(L.zones, a.i, { ...cs, x: wx - a.dx, y: wy - a.dy })
              e.setZoneView(a.work, chargeRef(a.i), true)
            }
          } else if (a.k === 'meet') {
            // the pin follows the cursor; the preview shows where it will snap / ピンは追従、吸着先を表示
            a.moved += Math.abs(ev.clientX - prev.x) + Math.abs(ev.clientY - prev.y)
            const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y)
            a.work = setPoint(L.zones, a.ref.kind, a.ref.id, a.ref.i, { x: wx, y: wy })
            e.setZoneView(a.work, a.ref, true)
            preview(p.x, p.y, a.ref.id, a.ref.kind)
          } else {
            const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y)
            if (a.k === 'draw') {
              a.work = setRect(L.zones, a.ref, normRect(a.ax, a.ay, wx, wy))
            } else {
              a.moved += Math.abs(ev.clientX - prev.x) + Math.abs(ev.clientY - prev.y)
              a.work = setRect(L.zones, a.ref, dragRect(a.start, a.h, wx - a.wx, wy - a.wy))
            }
            e.setZoneView(a.work, a.ref, true)
          }
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
      const p = local(ev), L = live.current
      const ptl = pointTool(L.tool)
      if (L.edit && L.tool?.kind === 'charge' && !g.space) {
        cv.style.cursor = e.pointAt(p.x, p.y) != null || e.chargeAt(p.x, p.y) != null ? 'move' : 'crosshair'
      } else if (L.edit && ptl && !g.space) {
        const over = e.pointAt(p.x, p.y)
        cv.style.cursor = over ? 'move' : 'crosshair'
        if (over) e.setSnap(null); else preview(p.x, p.y, ptl.id, ptl.kind)
      } else if (L.edit && !L.tool && !g.space) {
        const over = !!e.pointAt(p.x, p.y) || e.chargeAt(p.x, p.y) != null
        const z = over ? null : e.zoneAt(p.x, p.y)
        cv.style.cursor = over ? 'move' : z ? CURSOR[z.handle] : 'default'
      } else if (L.edit && L.tool) {
        const z = e.zoneAt(p.x, p.y)
        cv.style.cursor = z && z.handle !== 'move' && sameRef(z.ref, L.sel) ? CURSOR[z.handle] : 'crosshair'
      }
      const hit = e.hitTest(p.x, p.y)
      if (!hit) { hideTip(); return }
      tip.textContent = hit
      tip.style.display = 'block'
      tip.style.left = `${Math.max(4, Math.min(p.x + 14, p.w - tip.offsetWidth - 4))}px`
      tip.style.top = `${Math.min(p.y + 14, p.h - tip.offsetHeight - 4)}px`
    }
    const onUp = (ev: PointerEvent) => {
      const had = pts.delete(ev.pointerId)
      if (pts.size < 2) g.pinch = 0
      if (pts.size || !had) return
      const a = g.act, L = live.current
      g.act = null
      idleCursor()
      if (!a || ev.type === 'pointercancel') { if (a && a.k !== 'pan') e.setZoneView(L.zones, L.sel, L.edit); e.setSnap(null); return }
      if (a.k === 'pan') {
        if (a.deselect && a.moved < 4) L.setSel(null)
        if (a.addCharge && a.moved < 4) {
          // place a station where clicked (the sim checks it again) / クリック位置に設置
          const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y), n = L.zones.charge.length
          if (n < MAX_CHARGE) {
            L.commit(setCharge(L.zones, n, { x: wx, y: wy }))
            L.setSel(chargeRef(n))
          }
        }
        if (a.add && a.moved < 4) {
          // add the stop nearest the click (or select the point already there)
          // クリック位置に最も近い停車点を追加（既にあれば選択）
          const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y), q = snapAt(wx, wy)
          const { id, kind } = a.add
          const pts = L.zones[kind][id] ?? []
          if (q) {
            const dup = pts.findIndex((m) => Math.hypot(m.x - q[0], m.y - q[1]) < 1)
            if (dup >= 0) L.setSel({ kind, id, i: dup })
            else if (pts.length < maxPoints(kind)) {
              L.commit(setPoint(L.zones, kind, id, pts.length, { x: q[0], y: q[1] }))
              L.setSel({ kind, id, i: pts.length })
            }
          }
          preview(p.x, p.y, id, kind)
        }
      } else if (a.k === 'charge') {
        if (a.work && a.moved > 3) L.commit(a.work)
        else e.setZoneView(L.zones, L.sel, L.edit)
      } else if (a.k === 'meet') {
        e.setSnap(null)
        const p = local(ev), [wx, wy] = e.toWorld(p.x, p.y), q = a.moved > 3 ? snapAt(wx, wy) : null
        if (q) L.commit(setPoint(L.zones, a.ref.kind, a.ref.id, a.ref.i, { x: q[0], y: q[1] }))
        else e.setZoneView(L.zones, L.sel, L.edit)          // a click, or no road near: put it back / 戻す
      } else if (a.k === 'draw') {
        const r = a.work ? getRect(a.work, a.ref) : null
        if (a.work && r && bigEnough(r)) {
          L.commit(a.work); L.setSel(a.ref); L.setTool(null)
        } else {
          // too small: a click; select what is under it / 小さすぎ：クリック扱い
          e.setZoneView(L.zones, L.sel, L.edit)
          const p = local(ev), hit = e.zoneAt(p.x, p.y)
          L.setSel(hit ? hit.ref : null)
        }
      } else if (a.work && a.moved > 2) {
        L.commit(a.work)
      } else {
        e.setZoneView(L.zones, L.sel, L.edit)
      }
    }
    const onDbl = () => { if (!live.current.edit) e.fit() }
    const onAux = (ev: MouseEvent) => { if (ev.button === 1) ev.preventDefault() }
    // Space + drag pans while the map has focus / フォーカス中は Space + ドラッグでパン
    const onKeyDown = (ev: KeyboardEvent) => { if (ev.key === ' ' && document.activeElement === cv) { g.space = true; ev.preventDefault(); if (live.current.edit) cv.style.cursor = 'grab' } }
    const onKeyUp = (ev: KeyboardEvent) => { if (ev.key === ' ') { g.space = false; if (!g.act) idleCursor() } }

    cv.addEventListener('wheel', onWheel, { passive: false })
    cv.addEventListener('pointerdown', onDown)
    cv.addEventListener('pointermove', onMove)
    cv.addEventListener('pointerup', onUp)
    cv.addEventListener('pointercancel', onUp)
    const onLeave = () => { hideTip(); if (!g.act) e.setSnap(null) }
    cv.addEventListener('pointerleave', onLeave)
    cv.addEventListener('dblclick', onDbl)
    cv.addEventListener('auxclick', onAux)
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      cv.removeEventListener('wheel', onWheel)
      cv.removeEventListener('pointerdown', onDown)
      cv.removeEventListener('pointermove', onMove)
      cv.removeEventListener('pointerup', onUp)
      cv.removeEventListener('pointercancel', onUp)
      cv.removeEventListener('pointerleave', onLeave)
      cv.removeEventListener('dblclick', onDbl)
      cv.removeEventListener('auxclick', onAux)
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
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

  const deleteSel = useCallback(() => {
    if (!sel || !refExists(zones, sel)) return
    commit(removeRect(zones, sel)); setSel(null)
  }, [sel, zones, commit])

  const pickTool = (tl: Tool) => {
    setTool(tl)
    // a pod that already has a zone: select it too (Delete works, drawing replaces it)
    // ゾーンのあるポッドは選択も（削除可、描き直しで置き換え）
    if (tl && tl.kind === 'pod' && zones.pods[tl.id]) setSel({ kind: 'pod', id: tl.id })
    if (tl && isPointKind(tl.kind) && 'id' in tl && sel && (sel.id !== tl.id || sel.kind !== tl.kind)) setSel(null)
  }
  // the charging-station mode has one tool: picked at once / 充電所モードは道具を即選択
  const pickMode = (m: EditMode) => { setMode(m); setTool(m === 'charge' ? { kind: 'charge' } : null) }
  const toggleEdit = () => {
    setEdit((on) => !on)
    setTool(null); setSel(null)
  }
  const selMeet = getPoint(zones, sel)
  const selCharge = getCharge(zones, sel)
  const selIssue = selCharge && checker ? checker.check(selCharge) : null
  const selWhy = selCharge && state?.available && state.running ? simError(state.station_errors, selCharge) : null
  /** turn the selected station by ``d`` degrees; null = automatic (the sim
   *  faces it to the kerb) / 選択中の充電所を回す（null は自動） */
  const rotateSel = useCallback((d: number | null) => {
    const c = getCharge(zones, sel)
    if (!c || !sel || sel.kind !== 'charge') return
    commit(setCharge(zones, sel.i, { x: c.x, y: c.y, yaw: d == null ? undefined : (c.yaw ?? 0) + d }), d == null ? 0 : 400)
  }, [zones, sel, commit])

  const onKey = (ev: ReactKeyboardEvent) => {
    const e = engine, step = 60
    if (edit) {
      if (ev.key === 'Escape') { ev.preventDefault(); if (tool) setTool(null); else setSel(null); return }
      if ((ev.key === 'Delete' || ev.key === 'Backspace') && sel) { ev.preventDefault(); deleteSel(); return }
      const nudge: Record<string, [number, number]> = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }
      const r = getRect(zones, sel), d = nudge[ev.key]
      const cs = getCharge(zones, sel)
      if (sel && sel.kind === 'charge' && cs) {
        // arrows move the station 10 m (Shift: 1 m), R / Shift+R turn it 15 degrees
        // 矢印で 10 m（Shift: 1 m）移動、R / Shift+R で 15° 回転
        if (d) {
          ev.preventDefault()
          const k = ev.shiftKey ? 1 : 10
          commit(setCharge(zones, sel.i, { ...cs, x: cs.x + d[0] * k, y: cs.y + d[1] * k }), 500)
          return
        }
        if (ev.key === 'r' || ev.key === 'R') { ev.preventDefault(); rotateSel(ev.shiftKey ? -15 : 15); return }
      }
      if (sel && r && d) {
        // arrows move the selected zone 10 m (Shift: 1 m), saved once the keys rest
        // 矢印で選択ゾーンを 10 m（Shift: 1 m）移動、キーが止まってから保存
        ev.preventDefault()
        const k = ev.shiftKey ? 1 : 10
        commit(setRect(zones, sel, dragRect(r, 'move', d[0] * k, d[1] * k)), 500)
        return
      }
    }
    const act: Record<string, () => void> = {
      ArrowLeft: () => e.panBy(step, 0), ArrowRight: () => e.panBy(-step, 0),
      ArrowUp: () => e.panBy(0, step), ArrowDown: () => e.panBy(0, -step),
      '+': () => e.zoomAt(WHEEL_K * WHEEL_K), '=': () => e.zoomAt(WHEEL_K * WHEEL_K),
      '-': () => e.zoomAt(1 / (WHEEL_K * WHEEL_K)), '0': () => e.fit(),
    }
    const f = act[ev.key]
    if (f) { ev.preventDefault(); f() }
  }
  // Esc / Delete also work while a toolbar button has focus / 道具パネル上でも Esc・Delete
  const onToolsKey = (ev: ReactKeyboardEvent) => {
    if (ev.key === 'Escape') { ev.preventDefault(); if (tool) setTool(null); else if (sel) setSel(null); else toggleEdit() }
    else if (ev.key === 'Delete' && sel) { ev.preventDefault(); deleteSel() }
  }
  const closeLayers = useCallback(() => { setLayersOpen(false); layersBtn.current?.focus() }, [])
  const hidden = LAYER_KEYS.length - LAYER_KEYS.filter((k) => layers[k]).length

  return (
    <div ref={wrap} className={cn('relative overflow-hidden rounded-[var(--radius-card)] border border-line bg-surface shadow-[var(--shadow-card)]',
      full ? 'h-screen w-screen rounded-none' : 'h-[62vh] min-h-[380px] lg:h-[calc(100vh-15rem)] lg:min-h-[560px] lg:max-h-[1400px]')}>
      <canvas ref={cvRef} tabIndex={0} role="img"
        aria-label={`${t('dr_map')}: ${mapName}. ${edit ? t(mode === 'meet' ? 'dr_meet_hint' : mode === 'dist' ? 'dr_dist_hint' : mode === 'charge' ? 'dr_cs_hint' : 'dr_zone_hint') : t('dr_map_hint')}`}
        onKeyDown={onKey}
        className="block h-full w-full touch-none select-none outline-none focus-visible:ring-2 focus-visible:ring-accent" />
      <div ref={tipRef} role="tooltip"
        className="pointer-events-none absolute z-20 hidden max-w-[320px] rounded-lg border border-line bg-bg/95 px-2.5 py-1.5 text-xs text-fg shadow-xl num" />

      {/* top-left: title, map, edit zones, layers; the zone tools and the
          layers list stack below / 左上：題名・地図名・ゾーン編集・レイヤー */}
      <div className="pointer-events-none absolute left-3 top-3 z-20 flex max-h-[calc(100%-1.5rem)] max-w-[calc(100%-8rem)] flex-col items-start gap-2">
        <div className="flex flex-wrap items-center gap-2">
          <h3 className="rounded-lg bg-bg/75 px-2.5 py-1 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-2 backdrop-blur">{t('dr_map')}</h3>
          <Badge className="bg-bg/75 backdrop-blur">{mapName}</Badge>
          <OverlayButton label={t('dr_zones_edit')} icon={<SquarePen className="h-3.5 w-3.5" aria-hidden />}
            pressed={edit} controls={edit ? toolsId : undefined} onClick={toggleEdit} />
          <OverlayButton btnRef={layersBtn} label={t('dr_layers')} icon={<LayersIcon className="h-3.5 w-3.5" aria-hidden />} data-layers-btn
            expanded={layersOpen} controls={layersOpen ? layersId : undefined} onClick={() => setLayersOpen((o) => !o)}
            extra={hidden > 0 ? <span className="rounded bg-warn/20 px-1 text-[10px] text-warn num">−{hidden}<span className="sr-only"> {t('dr_ly_hidden', { n: hidden })}</span></span> : undefined} />
          {!edit && (status === 'error' || status === 'offline') && <span className="pointer-events-auto max-w-full"><ZoneStatusBadge status={status} error={error} /></span>}
        </div>
        {edit && (
          <div className="pointer-events-auto max-w-full shrink-0" onKeyDown={onToolsKey}>
            <ZoneToolbar id={toolsId} chips={chips} zones={zones} tool={tool} sel={sel} status={status} error={error}
              mode={mode} onMode={pickMode} snapExact={snap ? snap.exact : null} selMeet={selMeet}
              charge={{ ok: chargeOk, map: !!checker, lanes: !!checker?.hasLanes, sel: selCharge, issue: selIssue, why: selWhy,
                bad: zones.charge.filter((c) => !!checker?.check(c) || (!!state?.running && !!simError(state?.station_errors, c))).length,
                live: state?.available && state.running && Array.isArray(state.stations) ? state.stations.length : null }}
              onRotate={rotateSel} onTool={pickTool} onDelete={deleteSel} onDone={toggleEdit} />
          </div>
        )}
        {layersOpen && (
          <LayersMenu id={layersId} layers={layers} onChange={setLayers} onClose={closeLayers} className="pointer-events-auto min-h-0" />
        )}
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
            <p className="mt-2 border-t border-line pt-2 text-[11px] text-fg-mute">{edit ? t('dr_zone_hint_short') : t('dr_map_hint')}</p>
          </div>
        ) : (
          <button type="button" onClick={() => setLegend(true)} aria-expanded="false"
            className="flex h-10 items-center gap-2 rounded-lg border border-line bg-surface/85 px-3 text-xs text-fg-2 shadow-lg backdrop-blur hover:bg-muted">
            <Info className="h-4 w-4" aria-hidden />{t('dr_legend')}
          </button>
        )}
      </div>
    </div>
  )
}
