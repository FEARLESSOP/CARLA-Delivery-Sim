// The City Map renderer, kept out of React: it redraws at most once per screen
// frame (requestAnimationFrame), whatever arrives (state polls, wheel, drag).
// The map itself (lanes, markings, buildings) is a few Path2D objects in world
// metres built once per map, painted into an oversized cached image (the view
// plus half a view on every side): panning only slides that image, zooming
// stretches it and re-renders it sharp 140 ms after the wheel stops. That is
// the design that took panning from 7.7 to 100 frames/s on OitaKosen.
// 地図描画（React 外）：1 フレーム 1 回、地図は世界座標の Path2D と大きめの
// キャッシュ画像（移動はずらすだけ、拡大縮小は伸縮して 140 ms 後に描き直す）
// Layers (layers.ts) switch each kind of thing off; zones (pod delivery zones,
// no-fly zones) are a few rectangles drawn every frame on top of the base.
// Points: meeting points are pins, LMH distribution points square "D" badges.
// Mini pods (LastMile Hub robots) are small dots in their hub's colour with
// their planned path dashed behind them.
// LastMile charging stations: planned spots (footprint or, without a yaw, a
// dashed 8 m circle, a bolt marker, red when the pre-launch check or the sim
// rejects them) and, while the sim runs, the stations it built with their
// bays coloured by occupancy; rejected spots get a red X.
// 充電ステーション：予定地点（敷地か破線の円、稲妻の印、不可なら赤）と稼働中の実物
// レイヤーで表示を切替、ゾーンは毎フレーム描く数個の矩形
import type { TFunc } from '@/lib/i18n'
import { C, FONT_SANS } from '@/lib/colors'
import { fmt, clamp } from '@/lib/fmt'
import { MP_COL, type DeliveryMap, type DeliveryState, type Rect, type XY, type ZoneSet } from './types'
import { MC, BAY_COL, DRONE_COL, battCol, droneSt, orderSt, podColor } from './shared'
import { BASE_LAYERS, allLayers, type Layers } from './layers'
import { chargeRef, getRect, inNoFly, lmhArea, sameRef, type Handle, type PointKind, type PointRef, type ZoneRef } from './zones'
import { AUTO_R, footprint, inPoly, simError, toWorld, type SpotChecker } from './stations'

interface View { cx: number; cy: number; s: number }
/** a point tool's preview: the cursor, the stop it snaps to (world), whose
 *  point it adds and which kind / 点の道具のプレビュー */
export interface SnapPreview { cx: number; cy: number; sx: number; sy: number; pod: string; kind: PointKind }
interface Hit { x: number; y: number; r: number; tip: string }
interface RectHit { x0: number; y0: number; x1: number; y1: number; area: number; tip: string }
const HANDLE_PX = 9          // grab tolerance of a zone edge / corner / 辺・角の掴み幅
const PIN_UP = 13            // meeting-point pin head above its tip (px) / ピンの頭の高さ
interface Paths { s: Path2D; h: Path2D; d: Path2D; bld: Path2D; w: Path2D; wb: Path2D; y: Path2D; yb: Path2D; roads: Path2D }
interface Base { cv: HTMLCanvasElement; map: DeliveryMap; s: number; cx: number; cy: number; BW: number; BH: number; W: number; H: number; dpr: number }

const MARGIN = 0.5          // base image covers the view + half a view each side / 周囲半画面分
const ZMIN = 0.05, ZMAX = 40
const FLYING_ALL = ['delivering', 'to_station', 'to_van', 'falling']
/** mini pods inside their hub (not drawn on the street) / ハブ内のミニポッド */
const MP_IN_HUB = ['docked', 'loading']


export class MapEngine {
  map: DeliveryMap | null = null
  state: DeliveryState | null = null
  t: TFunc = (k) => k
  /** frames drawn (for fps measurement) / 描画回数 */
  frames = 0
  private cv: HTMLCanvasElement | null = null
  private view: View | null = null
  private paths: Paths | null = null
  private pathsMap: DeliveryMap | null = null
  private base: Base | null = null
  private zoomTimer = 0
  private raf = 0
  private hits: Hit[] = []
  private rectHits: RectHit[] = []
  private ro: ResizeObserver | null = null
  private layers: Layers = allLayers(true)
  private zones: ZoneSet | null = null
  private sel: ZoneRef | null = null
  private editing = false
  private hatch: CanvasPattern | null = null
  /** point tool preview: cursor and the stop it snaps to (world) / 吸着プレビュー */
  private snap: SnapPreview | null = null
  /** pre-launch check of charging-station spots (per map) / 充電所地点の確認 */
  private checker: SpotChecker | null = null

  attach(cv: HTMLCanvasElement) {
    this.cv = cv
    this.ro = new ResizeObserver(() => this.request())
    this.ro.observe(cv)
    this.request()
  }

  detach() {
    this.ro?.disconnect()
    this.ro = null
    cancelAnimationFrame(this.raf); this.raf = 0
    clearTimeout(this.zoomTimer)
    this.cv = null
    this.base = null
  }

  setMap(m: DeliveryMap | null) {
    if (m === this.map) return
    this.map = m; this.view = null; this.base = null
    this.request()
  }

  setState(s: DeliveryState | null) { this.state = s; this.request() }

  setT(t: TFunc) { this.t = t; this.request() }

  /** Show / hide layers; a base-image layer re-renders the cache once. */
  setLayers(l: Layers) {
    if (BASE_LAYERS.some((k) => l[k] !== this.layers[k])) this.base = null
    this.layers = l
    this.request()
  }

  /** Zones to draw, the selected one (handles while editing). Cheap: called
   *  on every pointer move of a zone drag. / ドラッグ中も毎回呼ばれる */
  setZoneView(z: ZoneSet | null, sel: ZoneRef | null, editing: boolean) {
    this.zones = z; this.sel = sel; this.editing = editing
    this.request()
  }

  /** The map's charging-station spot checker (built per map by CityMap). */
  setChecker(c: SpotChecker | null) { this.checker = c; this.request() }

  /** The charging-station spot under (x, y) CSS px: its marker first, then
   *  the footprint (or the 8 m circle) holding the point. / 充電所地点の当たり判定 */
  chargeAt(x: number, y: number): number | null {
    const z = this.zones
    if (!z || !this.view || !this.cv || !this.zoneOn('charge')) return null
    let best = -1, bd = 13 * 13
    for (let i = 0; i < z.charge.length; i++) {
      const c = z.charge[i], p = this.P(c.x, c.y), d = (p[0] - x) ** 2 + (p[1] - y) ** 2
      if (d < bd) { bd = d; best = i }
    }
    if (best >= 0) return best
    const [wx, wy] = this.toWorld(x, y)
    let ba = Infinity
    for (let i = 0; i < z.charge.length; i++) {
      const c = z.charge[i], d = Math.hypot(wx - c.x, wy - c.y)
      const inside = c.yaw != null ? inPoly(wx, wy, footprint(c.x, c.y, c.yaw).flat()) : d <= AUTO_R
      if (inside && d < ba) { ba = d; best = i }
    }
    return best >= 0 ? best : null
  }

  /** Snap preview of a point tool (null hides it). / 吸着プレビュー */
  setSnap(p: SnapPreview | null) {
    if (!p && !this.snap) return
    this.snap = p
    this.request()
  }

  /** The meeting or distribution point under (x, y) CSS px (pin head / badge
   *  or its foot), nearest first. / ピン・バッジの下にある点 */
  pointAt(x: number, y: number): PointRef | null {
    const z = this.zones
    if (!z || !this.view || !this.cv) return null
    let best: PointRef | null = null, bd = 11 * 11
    for (const kind of ['meet', 'dist'] as const) {
      if (!this.zoneOn(kind)) continue
      for (const [id, pts] of Object.entries(z[kind])) {
        pts.forEach((m, i) => {
          const p = this.P(m.x, m.y)
          const d = Math.min((p[0] - x) ** 2 + (p[1] - y) ** 2, (p[0] - x) ** 2 + (p[1] - PIN_UP - y) ** 2)
          if (d < bd) { bd = d; best = { kind, id, i } }
        })
      }
    }
    return best
  }
  /** older name: any point kind / 互換名 */
  meetAt(x: number, y: number): PointRef | null { return this.pointAt(x, y) }

  /** CSS px from the canvas corner -> world metres. / 画面 -> 世界座標 */
  toWorld(x: number, y: number): [number, number] {
    const v = this.view, cv = this.cv
    if (!v || !cv) return [0, 0]
    return [(x - cv.clientWidth / 2) / v.s + v.cx, (y - cv.clientHeight / 2) / v.s + v.cy]
  }

  /** zone layers are always on while editing / 編集中はゾーンを常に表示 */
  private zoneOn(kind: ZoneRef['kind']) {
    return this.editing || (kind === 'pod' ? this.layers.podZones : kind === 'meet' ? this.layers.meet
      : kind === 'dist' ? this.layers.dist : kind === 'charge' ? this.layers.charge : this.layers.nofly)
  }

  /** The zone (and which part of it) under (x, y) CSS px: the selected zone's
   *  edges / corners first, then the smallest zone holding the point.
   *  選択中の辺・角を優先、次に点を含む最小のゾーン */
  zoneAt(x: number, y: number): { ref: ZoneRef; handle: Handle } | null {
    const z = this.zones
    if (!z || !this.view || !this.cv) return null
    const scr = (r: Rect) => { const a = this.P(r.x0, r.y0), b = this.P(r.x1, r.y1); return [a[0], a[1], b[0], b[1]] }
    const sr = this.sel && this.zoneOn(this.sel.kind) ? getRect(z, this.sel) : null
    if (sr && this.sel) {
      const [x0, y0, x1, y1] = scr(sr), T = HANDLE_PX
      const inX = x > x0 - T && x < x1 + T, inY = y > y0 - T && y < y1 + T
      if (inX && inY) {
        const n = Math.abs(y - y0) < T, s = Math.abs(y - y1) < T, w = Math.abs(x - x0) < T, e = Math.abs(x - x1) < T
        const h = `${n ? 'n' : s ? 's' : ''}${w ? 'w' : e ? 'e' : ''}`
        if (h) return { ref: this.sel, handle: h as Handle }
      }
    }
    let best: ZoneRef | null = null, ba = Infinity
    const consider = (ref: ZoneRef, r: Rect) => {
      const [x0, y0, x1, y1] = scr(r)
      const a = (x1 - x0) * (y1 - y0)
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1 && a < ba) { best = ref; ba = a }
    }
    if (this.zoneOn('pod')) for (const [id, r] of Object.entries(z.pods)) consider({ kind: 'pod', id }, r)
    if (this.zoneOn('nofly')) for (const n of z.nofly) consider({ kind: 'nofly', id: n.id }, n)
    return best ? { ref: best, handle: 'move' } : null
  }

  /** One redraw per screen frame however many events arrive. / 1 フレームに 1 回 */
  request() {
    if (this.raf) return
    this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw() })
  }

  get hasView() { return !!this.view }
  /** CSS px per metre / 1 m あたりの画素 */
  get scale() { return this.view?.s ?? 1 }

  fit() { this.computeFit(); this.request() }

  private computeFit() {
    const m = this.map, cv = this.cv
    if (!m || !cv) return
    let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9
    for (const pl of m.roads) for (const [x, y] of pl) {
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y
    }
    if (x0 > x1) { x0 = -100; x1 = 100; y0 = -100; y1 = 100 }
    const W = cv.clientWidth || 800, H = cv.clientHeight || 600
    this.view = { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2,
      s: 0.94 * Math.min(W / Math.max(x1 - x0, 1), H / Math.max(y1 - y0, 1)) }
  }

  /** Zoom by ``k`` keeping the world point under (mx, my) (CSS px from the
   *  canvas centre) still. / カーソル位置を固定して拡大縮小 */
  zoomAt(k: number, mx = 0, my = 0) {
    const v = this.view
    if (!v) return
    const wx = v.cx + mx / v.s, wy = v.cy + my / v.s
    v.s = clamp(v.s * k, ZMIN, ZMAX)
    v.cx = wx - mx / v.s; v.cy = wy - my / v.s
    this.request()
  }

  /** Centre the view on a world point, zooming in to at least ``minS`` px
   *  per metre (a station list click). / 世界座標の地点を中央に */
  centreOn(x: number, y: number, minS = 4) {
    if (!this.view) this.computeFit()
    const v = this.view
    if (!v || !Number.isFinite(x) || !Number.isFinite(y)) return
    v.cx = x; v.cy = y; v.s = clamp(Math.max(v.s, minS), ZMIN, ZMAX)
    this.request()
  }

  /** Pan by a screen delta in CSS px. / 画面上の移動量でパン */
  panBy(dx: number, dy: number) {
    const v = this.view
    if (!v) return
    v.cx -= dx / v.s; v.cy -= dy / v.s
    this.request()
  }

  /** The tooltip under (x, y) in CSS px from the canvas corner, if any. */
  hitTest(x: number, y: number): string | null {
    let best: Hit | null = null, bd = 1e9
    for (const h of this.hits) {
      const d = Math.hypot(h.x - x, h.y - y)
      if (d < h.r && d < bd) { best = h; bd = d }
    }
    if (best) return best.tip
    let rb: RectHit | null = null
    for (const h of this.rectHits) {
      if (x >= h.x0 && x <= h.x1 && y >= h.y0 && y <= h.y1 && (!rb || h.area < rb.area)) rb = h
    }
    return rb ? rb.tip : null
  }

  /* ---------------------------------------------------------- internals */
  private P(x: number, y: number): [number, number] {
    const v = this.view!, cv = this.cv!
    return [(x - v.cx) * v.s + cv.clientWidth / 2, (y - v.cy) * v.s + cv.clientHeight / 2]
  }

  private box(ctx: CanvasRenderingContext2D, x: number, y: number, ex: number, ey: number, yaw: number) {
    const r = yaw * Math.PI / 180, c = Math.cos(r), s = Math.sin(r)
    ctx.beginPath()
    const pts: [number, number][] = [[ex, ey], [-ex, ey], [-ex, -ey], [ex, -ey]]
    pts.forEach(([a, b], i) => {
      const p = this.P(x + a * c - b * s, y + a * s + b * c)
      if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1])
    })
    ctx.closePath()
  }

  private buildPaths(m: DeliveryMap): Paths {
    if (this.paths && this.pathsMap === m) return this.paths
    const P: Paths = { s: new Path2D(), h: new Path2D(), d: new Path2D(), bld: new Path2D(),
      w: new Path2D(), wb: new Path2D(), y: new Path2D(), yb: new Path2D(), roads: new Path2D() }
    const add = (p: Path2D, f: number[], close: boolean) => {
      for (let i = 0; i + 1 < f.length; i += 2) { if (i) p.lineTo(f[i], f[i + 1]); else p.moveTo(f[i], f[i + 1]) }
      if (close) p.closePath()
    }
    for (const [x, y, ex, ey, yaw] of m.buildings || []) {
      const r = yaw * Math.PI / 180, c = Math.cos(r), sn = Math.sin(r)
      const q: [number, number][] = [[ex, ey], [-ex, ey], [-ex, -ey], [ex, -ey]]
      add(P.bld, q.flatMap(([a, b]) => [x + a * c - b * sn, y + a * sn + b * c]), true)
    }
    for (const [k, f] of m.lanes || []) add(k === 's' ? P.s : k === 'd' ? P.d : P.h, f, true)
    for (const [col, broken, f] of m.marks || []) add(col === 'y' ? (broken ? P.yb : P.y) : (broken ? P.wb : P.w), f, false)
    if (!m.lanes) for (const pl of m.roads || []) add(P.roads, pl.flat(), false)
    this.paths = P; this.pathsMap = m
    return P
  }

  private baseRender(W: number, H: number, dpr: number) {
    const m = this.map!, v = this.view!, P = this.buildPaths(m)
    const BW = Math.round(W * (1 + 2 * MARGIN)), BH = Math.round(H * (1 + 2 * MARGIN))
    const cv = this.base?.cv ?? document.createElement('canvas')
    cv.width = Math.round(BW * dpr); cv.height = Math.round(BH * dpr)
    const ctx = cv.getContext('2d')!
    ctx.setTransform(1, 0, 0, 1, 0, 0)
    ctx.clearRect(0, 0, cv.width, cv.height)
    // world metres -> base pixels / 世界座標 -> 画像の画素
    ctx.setTransform(dpr * v.s, 0, 0, dpr * v.s, dpr * (BW / 2 - v.cx * v.s), dpr * (BH / 2 - v.cy * v.s))
    const px = 1 / v.s                                // one CSS pixel in metres
    const L = this.layers
    ctx.lineJoin = 'round'
    if (L.buildings) {
      ctx.fillStyle = m.lanes ? MC.bld : MC.oldBld
      ctx.strokeStyle = m.lanes ? MC.bldEdge : MC.oldBldEdge; ctx.lineWidth = px
      ctx.fill(P.bld); ctx.stroke(P.bld)
    }
    if (m.lanes) {
      // pavements and shoulders under the carriageways, each outlined in its
      // own colour so neighbouring lanes meet without a seam / 継ぎ目なし
      ctx.lineWidth = 1.2 * px
      const layers: [Path2D, string][] = [[P.s, MC.side], [P.h, MC.shoulder], [P.d, MC.road]]
      if (L.roads) for (const [p, col] of layers) { ctx.fillStyle = ctx.strokeStyle = col; ctx.fill(p); ctx.stroke(p) }
      // lane markings, only once big enough to read / 区画線（拡大時のみ）
      if (L.marks && v.s > 0.6) {
        ctx.lineCap = 'butt'
        ctx.lineWidth = Math.max(1, Math.min(3, 0.15 * v.s)) * px
        const marks: [Path2D, string, boolean][] = [[P.w, MC.white, false], [P.wb, MC.white, true],
          [P.y, MC.yellow, false], [P.yb, MC.yellow, true]]
        for (const [p, col, broken] of marks) { ctx.strokeStyle = col; ctx.setLineDash(broken ? [3, 5] : []); ctx.stroke(p) }
        ctx.setLineDash([])
      }
    } else {
      // older delivery_sim: lane centre lines only / 旧形式：中心線のみ
      ctx.strokeStyle = MC.oldRoad; ctx.lineCap = 'round'
      ctx.lineWidth = Math.max(1.2, 3.4 * v.s) * px
      if (L.roads) ctx.stroke(P.roads)
    }
    this.base = { cv, map: m, s: v.s, cx: v.cx, cy: v.cy, BW, BH, W, H, dpr }
  }

  /** Reuse (pan), stretch (zoom; sharp again once the wheel stops) or render
   *  anew. / 移動は再利用、拡大縮小は伸縮後に描き直し */
  private baseDraw(ctx: CanvasRenderingContext2D, W: number, H: number, dpr: number) {
    const v = this.view!
    let b = this.base
    const fresh = !!b && b.map === this.map && b.dpr === dpr && b.W === W && b.H === H
    if (b && fresh && b.s === v.s) {
      if (Math.abs(b.cx - v.cx) * v.s > (b.BW - W) / 2 || Math.abs(b.cy - v.cy) * v.s > (b.BH - H) / 2) {
        this.baseRender(W, H, dpr); b = this.base
      }
    } else if (b && fresh) {
      clearTimeout(this.zoomTimer)
      this.zoomTimer = window.setTimeout(() => {
        if (!this.cv || !this.view) return
        this.baseRender(W, H, dpr); this.request()
      }, 140)
    } else {
      this.baseRender(W, H, dpr); b = this.base
    }
    if (!b) return
    const k = v.s / b.s                                 // 1 unless zooming
    const ox = (b.cx - b.BW / (2 * b.s) - v.cx) * v.s + W / 2
    const oy = (b.cy - b.BH / (2 * b.s) - v.cy) * v.s + H / 2
    ctx.drawImage(b.cv, ox, oy, b.BW * k, b.BH * k)
  }

  private arrow(ctx: CanvasRenderingContext2D, x: number, y: number, dx: number, dy: number, col: string, w: number) {
    const a = Math.atan2(dy, dx), L = Math.hypot(dx, dy)
    if (L < 1) return
    ctx.strokeStyle = ctx.fillStyle = col; ctx.lineWidth = w
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke()
    const h = Math.min(10, L * 0.4)
    ctx.beginPath()
    ctx.moveTo(x + dx, y + dy)
    ctx.lineTo(x + dx - h * Math.cos(a - 0.45), y + dy - h * Math.sin(a - 0.45))
    ctx.lineTo(x + dx - h * Math.cos(a + 0.45), y + dy - h * Math.sin(a + 0.45))
    ctx.closePath(); ctx.fill()
  }

  private draw() {
    const cv = this.cv
    if (!cv) return
    const dpr = window.devicePixelRatio || 1
    const W = cv.clientWidth, H = cv.clientHeight
    if (W < 30 || H < 30) return                       // hidden / 非表示
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
      cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr)
    }
    const ctx = cv.getContext('2d')
    if (!ctx) return
    this.frames++
    const t = this.t
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    const m = this.map, s = this.state
    ctx.fillStyle = m && m.lanes ? MC.ground : MC.oldGround; ctx.fillRect(0, 0, W, H)
    const hits: Hit[] = []
    this.hits = hits
    this.rectHits = []
    const font = (w: string, px: number) => `${w} ${px}px ${FONT_SANS}`
    if (!m) {
      ctx.fillStyle = C.mute; ctx.font = font('400', 13); ctx.textAlign = 'center'
      ctx.fillText(t('dr_nomap'), W / 2, H / 2)
      return
    }
    if (!this.view) this.computeFit()
    const v = this.view!
    this.baseDraw(ctx, W, H, dpr)
    const L = this.layers
    const R = (k: number) => Math.max(k, k * v.s / 2.2)  // markers grow when zoomed in
    this.drawZones(ctx, W, H)
    this.drawCharge(ctx, W, H)

    // charge station / 充電所
    if (L.station) {
      ctx.fillStyle = MC.stationFill; ctx.strokeStyle = MC.station; ctx.lineWidth = 1
      for (const [x, y] of m.station || []) {
        const p = this.P(x, y), r = R(3.5)
        ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r)
      }
      if (m.station?.length) {
        const p = this.P(m.station[0][0], m.station[0][1])
        if (L.labels) {
          ctx.fillStyle = MC.station; ctx.font = font('600', 11); ctx.textAlign = 'left'
          ctx.fillText(t('dr_lg_station'), p[0] + 10, p[1] - 8)
        }
        hits.push({ x: p[0], y: p[1], r: 12, tip: `${t('dr_lg_station')} · ${t('dr_tip_pads', { n: m.station.length })}` })
      }
    }
    // EV chargers for the pods / ポッド用 EV 充電器
    if (L.ev) for (const [x, y] of m.chargers || []) {
      const p = this.P(x, y), r = R(4.5)
      ctx.fillStyle = MC.evFill; ctx.strokeStyle = MC.ev; ctx.lineWidth = 1.5
      ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r)
      ctx.fillStyle = MC.ev; ctx.font = font('700', 11); ctx.textAlign = 'center'
      ctx.fillText('EV', p[0], p[1] + 3)
      hits.push({ x: p[0], y: p[1], r: 9, tip: t('dr_lg_ev') })
    }
    // LastMile box stations on the pavements / 歩道の箱型ステーション
    if (L.boxes) (m.boxes || []).forEach(([x, y], i) => {
      const p = this.P(x, y), r = R(3.5)
      ctx.fillStyle = MC.box; ctx.strokeStyle = MC.boxEdge; ctx.lineWidth = 1.5
      ctx.fillRect(p[0] - r, p[1] - r * 1.3, 2 * r, 2.6 * r); ctx.strokeRect(p[0] - r, p[1] - r * 1.3, 2 * r, 2.6 * r)
      hits.push({ x: p[0], y: p[1], r: 8, tip: `${t('dr_lg_box')} ${i + 1}` })
    })
    // the live part: needs a running delivery sim / 配送シミュ稼働中のみ
    const live = () => {
      if (!s || !s.available) return

      const drones = s.drones || []
      // drones on the pads, listed beside the station / 充電パッド上の機体
      const parked = drones.filter((d) => d.status === 'charging' || d.status === 'standby')
      if (L.drones && L.labels && L.station && parked.length && m.station?.length) {
        const p = this.P(m.station[0][0], m.station[0][1])
        ctx.font = font('600', 11.5); ctx.textAlign = 'left'
        parked.forEach((d, i) => {
          ctx.fillStyle = battCol(d.battery)
          ctx.fillText(`D${d.id} ${Math.round(d.battery)}% ${droneSt(t, d.status)}`, p[0] + 10, p[1] + 7 + 13 * i)
        })
      }
      // hubs / ドローン地点
      if (L.hubs) for (const h of s.hubs || []) {
        const p = this.P(h.x, h.y), r = R(5)
        ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(Math.PI / 4)
        ctx.strokeStyle = MC.hub; ctx.lineWidth = 2; ctx.strokeRect(-r, -r, 2 * r, 2 * r)
        ctx.restore()
        hits.push({ x: p[0], y: p[1], r: 8, tip: `${t('dr_lg_hub')} · ${t('pd_pod')} ${h.van}` })
      }
      // orders: kind (face to face / drone) and outcome (delivered / failed)
      // filters both apply / 種類と結果の両方で絞り込み
      for (const o of s.orders || []) {
        if (!(o.kind === 'drone' ? L.ordDrone : L.ordF2f)) continue
        if ((o.status === 'delivered' && !L.ordDone) || (o.status === 'failed' && !L.ordFail)) continue
        const p = this.P(o.x, o.y), r = R(3.2)
        let col: string = o.kind === 'drone' ? MC.droneOrder : MC.f2f
        if (o.status === 'delivered') col = MC.done
        if (o.status === 'failed') col = MC.fail
        ctx.globalAlpha = o.status === 'delivered' ? 0.55 : 1
        ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill()
        if (o.status === 'queued' || o.status === 'in_flight') {
          ctx.strokeStyle = MC.ring; ctx.lineWidth = o.status === 'in_flight' ? 2 : 1
          ctx.beginPath(); ctx.arc(p[0], p[1], r + 2.5, 0, 7); ctx.stroke()
        }
        if (o.status === 'failed') {
          ctx.strokeStyle = MC.failCross; ctx.lineWidth = 1.6; ctx.beginPath()
          ctx.moveTo(p[0] - r - 2, p[1] - r - 2); ctx.lineTo(p[0] + r + 2, p[1] + r + 2)
          ctx.moveTo(p[0] + r + 2, p[1] - r - 2); ctx.lineTo(p[0] - r - 2, p[1] + r + 2); ctx.stroke()
        }
        ctx.globalAlpha = 1
        hits.push({ x: p[0], y: p[1], r: 7, tip: `#${o.id} ${o.kind === 'drone' ? t('dr_tip_drone') : t('dr_tip_f2f')} · ${orderSt(t, o.status)}` +
          (o.van ? ` · ${t('pd_pod')} ${o.van}` : '') + (o.zone != null ? ` · ${t('dr_tip_zone_of', { n: o.zone })}` : '') +
          (o.reason ? ` · ${o.reason}` : '') })
      }
      // other cars, at least 7 x 3.5 px however far out / 他の車（縮小時も最小サイズ）
      if (L.cars) {
        ctx.fillStyle = MC.car
        for (const [x, y, yaw] of s.cars || []) {
          this.box(ctx, x, y, Math.max(2.3, 3.5 / v.s), Math.max(1.0, 1.75 / v.s), yaw); ctx.fill()
        }
      }
      // pods: body in the pod's own colour, edge = battery / ポッド（塗り=ポッド色、縁=電池）
      const vanPos = new Map<number, [number, number]>()
      for (const van of s.vans || []) {
        vanPos.set(van.id, [van.x, van.y])
        if (!L.pods) continue
        const pc = podColor(van.id)
        ctx.fillStyle = pc; ctx.strokeStyle = battCol(van.battery); ctx.lineWidth = 2.2
        this.box(ctx, van.x, van.y, Math.max(2.3, 7 / v.s), Math.max(1.1, 3.5 / v.s), van.yaw)
        ctx.fill(); ctx.stroke()
        const p = this.P(van.x, van.y)
        if (L.labels) {
          ctx.font = font('700', 11); ctx.textAlign = 'center'
          ctx.lineWidth = 3; ctx.strokeStyle = MC.zoneText; ctx.strokeText(`V${van.id} ${Math.round(van.battery)}%`, p[0], p[1] - 9)
          ctx.fillStyle = pc; ctx.fillText(`V${van.id} ${Math.round(van.battery)}%`, p[0], p[1] - 9)
        }
        const lmh = van.kind === 'lmh'
        const docked = Array.isArray(van.pods_docked) ? van.pods_docked.length : van.pods_docked
        hits.push({ x: p[0], y: p[1], r: 10, tip: `${lmh ? t('dr_tip_lmh', { n: van.id }) : `${t('pd_pod')} ${van.id}`} · ${t('dr_th_batt')} ${Math.round(van.battery)}% · ${van.state} · ` +
          (lmh ? t('dr_tip_lmh_docked', { n: docked ?? 0 }) : `${van.drone ? `D${van.drone}` : t('dr_nodrone')}`) + ` · ${van.delivered} ✓ ${van.failed} ✗` +
          (van.zone === true || this.zones?.pods[String(van.id)] ? ` · ${t('dr_tip_haszone')}` : '') })
      }
      // mini pods (LastMile Hub robots): planned path, then a dot in the hub's
      // colour ringed by status / ミニポッド：経路と、ハブ色の点（縁=状態）
      const mps = (s.minipods || []).filter((q) => !MP_IN_HUB.includes(q.status))
      if (L.minipodPaths) for (const q of mps) {
        if (!q.path || q.path.length < 1) continue
        ctx.strokeStyle = q.hub != null ? podColor(q.hub) : MC.ring
        ctx.globalAlpha = 0.85; ctx.lineWidth = 1.6; ctx.setLineDash([5, 4])
        ctx.beginPath()
        const a = this.P(q.x, q.y); ctx.moveTo(a[0], a[1])
        for (const [x, y] of q.path) { const b = this.P(x, y); ctx.lineTo(b[0], b[1]) }
        ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1
        const end = q.path[q.path.length - 1], e2 = this.P(end[0], end[1])
        ctx.fillStyle = q.hub != null ? podColor(q.hub) : MC.ring
        ctx.beginPath(); ctx.arc(e2[0], e2[1], 2.5, 0, 7); ctx.fill()
      }
      if (L.minipods) for (const q of mps) {
        const p = this.P(q.x, q.y), r = R(2.8)
        const col = q.hub != null ? podColor(q.hub) : MC.car
        ctx.fillStyle = col; ctx.strokeStyle = MP_COL[q.status] ?? MC.ring; ctx.lineWidth = 2
        ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill(); ctx.stroke()
        // heading tick / 向き
        if (q.yaw != null) {
          const a = q.yaw * Math.PI / 180
          ctx.strokeStyle = MC.zoneText; ctx.lineWidth = 1.5
          ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(p[0] + Math.cos(a) * r, p[1] + Math.sin(a) * r); ctx.stroke()
        }
        if (q.status === 'crashed' || q.status === 'stuck') {
          ctx.strokeStyle = MC.ring; ctx.lineWidth = 1.4; ctx.beginPath()
          ctx.moveTo(p[0] - r, p[1] - r); ctx.lineTo(p[0] + r, p[1] + r)
          ctx.moveTo(p[0] + r, p[1] - r); ctx.lineTo(p[0] - r, p[1] + r); ctx.stroke()
        }
        if (L.labels && v.s > 0.9) {
          ctx.font = font('700', 10.5); ctx.textAlign = 'left'
          ctx.lineWidth = 3; ctx.strokeStyle = MC.zoneText; ctx.strokeText(`M${q.id}`, p[0] + r + 3, p[1] + 4)
          ctx.fillStyle = col; ctx.fillText(`M${q.id}`, p[0] + r + 3, p[1] + 4)
        }
        const st = t(`mps_${q.status}`)
        hits.push({ x: p[0], y: p[1], r: r + 4, tip: t('dr_tip_minipod', { n: q.id }) +
          (q.hub != null ? ` · ${t('dr_tip_lmh', { n: q.hub })}${q.bay != null ? ` ${t('dr_tip_bay', { n: q.bay })}` : ''}` : '') +
          ` · ${st === `mps_${q.status}` ? q.status : st}` + (q.battery != null ? ` · ${Math.round(q.battery)}%` : '') +
          (q.order != null ? ` · #${q.order}` : '') })
      }
      // drones / ドローン
      for (const d of drones) {
        const p = this.P(d.x, d.y)
        const flying = FLYING_ALL.includes(d.status)
        const vp = d.van != null ? vanPos.get(d.van) : undefined
        if (L.links && flying && vp && d.status !== 'to_station') {
          const q = this.P(vp[0], vp[1])
          ctx.strokeStyle = MC.link; ctx.setLineDash([4, 4]); ctx.lineWidth = 1
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(p[0], p[1]); ctx.stroke(); ctx.setLineDash([])
        }
        if (!L.drones) continue
        if (d.status === 'docked' || d.status === 'reloading') continue   // inside its pod
        const r = flying ? R(5) : R(3.6)
        ctx.fillStyle = battCol(d.battery)
        ctx.strokeStyle = DRONE_COL[d.status] ?? MC.ring; ctx.lineWidth = 2.2
        ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 7); ctx.fill(); ctx.stroke()
        if (d.status === 'crashed' || d.status === 'falling') {
          ctx.strokeStyle = MC.ring; ctx.lineWidth = 1.5; ctx.beginPath()
          ctx.moveTo(p[0] - r, p[1] - r); ctx.lineTo(p[0] + r, p[1] + r)
          ctx.moveTo(p[0] + r, p[1] - r); ctx.lineTo(p[0] - r, p[1] + r); ctx.stroke()
        }
        if (L.labels && (flying || d.status === 'crashed')) {      // pad drones are listed by the station
          ctx.fillStyle = MC.label; ctx.font = font('700', 11.5); ctx.textAlign = 'left'
          ctx.fillText(`D${d.id} ${Math.round(d.battery)}%`, p[0] + r + 3, p[1] + 4)
        }
        hits.push({ x: p[0], y: p[1], r: r + 4, tip: `D${d.id} · ${droneSt(t, d.status)} · ${Math.round(d.battery)}% · z ${fmt(d.z, 0)} m` +
          (d.note ? ` · ${d.note}` : '') })
      }
      // wind compass in the corner (map orientation) / 隅に風向（地図と同じ向き）
      if (!L.wind) return
      const w = s.wind || {}
      const cx = W - 54, cy = 54
      ctx.fillStyle = MC.compassBg; ctx.strokeStyle = MC.compassEdge; ctx.lineWidth = 1
      ctx.beginPath(); ctx.arc(cx, cy, 42, 0, 7); ctx.fill(); ctx.stroke()
      const Lw = Math.min(36, 6 + (w.speed || 0) * 2.2), a = (w.dir || 0) * Math.PI / 180
      this.arrow(ctx, cx - Lw / 2 * Math.cos(a), cy - Lw / 2 * Math.sin(a), Lw * Math.cos(a), Lw * Math.sin(a),
        w.gust ? MC.gust : MC.windArrow, 3)
      ctx.fillStyle = w.gust ? MC.gust : MC.windText; ctx.textAlign = 'center'; ctx.font = font('700', 11)
      ctx.fillText(`${fmt(w.speed || 0, 1)} m/s${w.gust ? ` ${t('dr_gust')}` : ''}`, cx, cy + 58)
    }
    live()
    this.drawMeet(ctx, W, H)
  }

  /** Meeting points: a pin in the pod's colour (dark outline, its number in
   *  the head) standing on the stop; inside a no-fly zone the pin is dark with
   *  a dashed red outline and "!" (the sim ignores it). Then the snap preview of
   *  the meeting-point tool. / 合流地点のピン（禁止区域内は破線と「!」）と吸着プレビュー */
  private drawMeet(ctx: CanvasRenderingContext2D, W: number, H: number) {
    const z = this.zones, t = this.t
    if (!z || !this.view) return
    const pin = (x: number, y: number, col: string, txt: string, bad: boolean, sel: boolean, ghost = false) => {
      const hx = x, hy = y - PIN_UP, r = 7.5
      ctx.globalAlpha = ghost ? 0.75 : 1
      ctx.beginPath()
      ctx.moveTo(x, y)
      ctx.lineTo(hx - r * 0.72, hy + r * 0.7)
      ctx.arc(hx, hy, r, Math.PI * 0.75, Math.PI * 0.25)
      ctx.closePath()
      ctx.fillStyle = bad ? MC.compassBg : col; ctx.fill()
      ctx.lineWidth = 2; ctx.strokeStyle = bad ? MC.nofly : MC.zoneText
      if (bad || ghost) ctx.setLineDash([3, 2])
      ctx.stroke(); ctx.setLineDash([])
      ctx.globalAlpha = 1
      ctx.fillStyle = bad ? MC.nofly : MC.zoneText; ctx.font = `800 10px ${FONT_SANS}`; ctx.textAlign = 'center'
      ctx.fillText(bad ? '!' : txt, hx, hy + 3.5)
      if (sel) {
        ctx.strokeStyle = MC.handle; ctx.lineWidth = 2
        ctx.beginPath(); ctx.arc(hx, hy, r + 3.5, 0, 7); ctx.stroke()
        ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.stroke()
      }
    }
    // distribution point: a square "D" badge on a short post (vs the round
    // pin), so the two read apart without colour / 配布地点：棒の上の四角い「D」
    const badge = (x: number, y: number, col: string, txt: string, sel: boolean, ghost = false) => {
      ctx.font = `800 9.5px ${FONT_SANS}`
      const label = `D${txt}`
      const w = Math.max(16, ctx.measureText(label).width + 7), h = 14, top = y - PIN_UP - h / 2
      ctx.globalAlpha = ghost ? 0.75 : 1
      ctx.strokeStyle = MC.zoneText; ctx.lineWidth = 2
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, top + h); ctx.stroke()
      ctx.beginPath(); ctx.rect(x - w / 2, top, w, h)
      ctx.fillStyle = col; ctx.fill()
      if (ghost) ctx.setLineDash([3, 2])
      ctx.stroke(); ctx.setLineDash([])
      ctx.globalAlpha = 1
      ctx.fillStyle = MC.zoneText; ctx.textAlign = 'center'
      ctx.fillText(label, x, top + h - 3.5)
      ctx.fillRect(x - 1.5, y - 1.5, 3, 3)
      if (sel) {
        ctx.strokeStyle = MC.handle; ctx.lineWidth = 2
        ctx.strokeRect(x - w / 2 - 3.5, top - 3.5, w + 7, h + 7)
        ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.stroke()
      }
    }
    // an LMH hub's meeting points are for its ground robots: no-fly zones do
    // not matter to them / LMH ハブの合流地点は飛行禁止区域でも有効
    const lmh = new Set((this.state?.vans ?? []).filter((v) => v.kind === 'lmh').map((v) => String(v.id)))
    if (this.zoneOn('meet')) for (const [id, pts] of Object.entries(z.meet)) {
      const col = podColor(id)
      pts.forEach((m, i) => {
        const p = this.P(m.x, m.y)
        if (p[0] < -20 || p[1] < -30 || p[0] > W + 20 || p[1] > H + 20) return
        const bad = !lmh.has(id) && inNoFly(z, m.x, m.y)
        pin(p[0], p[1], col, String(i + 1), bad, this.editing && sameRef(this.sel, { kind: 'meet', id, i }))
        this.hits.push({ x: p[0], y: p[1] - PIN_UP, r: 10,
          tip: `${t('dr_meet_tip', { n: id, i: i + 1, x: fmt(m.x, 1), y: fmt(m.y, 1) })}${bad ? ` · ${t('dr_meet_bad')}` : ''}` })
      })
    }
    if (this.zoneOn('dist')) for (const [id, pts] of Object.entries(z.dist)) {
      const col = podColor(id)
      pts.forEach((m, i) => {
        const p = this.P(m.x, m.y)
        if (p[0] < -24 || p[1] < -30 || p[0] > W + 24 || p[1] > H + 20) return
        badge(p[0], p[1], col, String(i + 1), this.editing && sameRef(this.sel, { kind: 'dist', id, i }))
        this.hits.push({ x: p[0], y: p[1] - PIN_UP, r: 10,
          tip: t('dr_dist_tip', { n: id, i: i + 1, x: fmt(m.x, 1), y: fmt(m.y, 1) }) })
      })
    }
    const sn = this.snap
    if (sn) {
      const c = this.P(sn.cx, sn.cy), q = this.P(sn.sx, sn.sy)
      ctx.strokeStyle = MC.ring; ctx.lineWidth = 1; ctx.setLineDash([3, 3])
      ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); ctx.setLineDash([])
      if (sn.kind === 'dist') badge(q[0], q[1], podColor(sn.pod), '+', false, true)
      else pin(q[0], q[1], podColor(sn.pod), '+', !lmh.has(sn.pod) && inNoFly(z, sn.sx, sn.sy), false, true)
    }
  }

  /** A lightning bolt filling a box of half-size ``s`` at (cx, cy). / 稲妻 */
  private bolt(ctx: CanvasRenderingContext2D, cx: number, cy: number, s: number, col: string) {
    ctx.beginPath()
    ctx.moveTo(cx + 0.2 * s, cy - s); ctx.lineTo(cx - 0.6 * s, cy + 0.15 * s); ctx.lineTo(cx - 0.05 * s, cy + 0.15 * s)
    ctx.lineTo(cx - 0.25 * s, cy + s); ctx.lineTo(cx + 0.6 * s, cy - 0.2 * s); ctx.lineTo(cx + 0.05 * s, cy - 0.2 * s)
    ctx.closePath(); ctx.fillStyle = col; ctx.fill()
  }

  /** Charging stations: the live ones the sim built (footprint polygon from
   *  its corners, bays coloured by phase with P / H letters), the spots the
   *  sim rejected (red X), then the planned spots of the editor on top
   *  (footprint, or a dashed 8 m circle when the sim picks the direction;
   *  bolt marker + number; red + dashed when unusable). Missing state keys =
   *  an older sim: only the planned spots. / 充電ステーションの描画 */
  private drawCharge(ctx: CanvasRenderingContext2D, W: number, H: number) {
    const z = this.zones, s = this.state, t = this.t, v = this.view
    if (!v || !this.zoneOn('charge')) return
    const run = !!s && s.available && s.running
    const live = run && Array.isArray(s.stations) ? s.stations : []
    const errors = run && Array.isArray(s.station_errors) ? s.station_errors : []
    const font = (w: string, px: number) => `${w} ${px}px ${FONT_SANS}`
    const path = (pts: XY[]) => {
      ctx.beginPath()
      pts.forEach(([x, y], i) => { const p = this.P(x, y); if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]) })
      ctx.closePath()
    }
    const off = (p: [number, number], m: number) => p[0] < -m || p[1] < -m || p[0] > W + m || p[1] > H + m
    const text = (txt: string, x: number, y: number, col: string, px = 10.5) => {
      ctx.font = font('700', px); ctx.textAlign = 'left'; ctx.lineJoin = 'round'
      ctx.lineWidth = 3; ctx.strokeStyle = MC.zoneText; ctx.strokeText(txt, x, y)
      ctx.fillStyle = col; ctx.fillText(txt, x, y)
    }
    const cross = (p: [number, number], r: number) => {
      ctx.strokeStyle = MC.nofly; ctx.lineWidth = 2.6; ctx.beginPath()
      ctx.moveTo(p[0] - r, p[1] - r); ctx.lineTo(p[0] + r, p[1] + r)
      ctx.moveTo(p[0] + r, p[1] - r); ctx.lineTo(p[0] - r, p[1] + r); ctx.stroke()
    }

    // live stations / 稼働中のステーション
    for (const st of live) {
      if (!Number.isFinite(st.x) || !Number.isFinite(st.y)) continue
      const corners = Array.isArray(st.corners) && st.corners.length >= 3 ? st.corners : footprint(st.x, st.y, st.yaw ?? 0)
      const c = this.P(st.x, st.y)
      if (off(c, 14 * v.s + 40)) continue
      path(corners)
      ctx.fillStyle = MC.chargeFill; ctx.fill()
      ctx.strokeStyle = MC.chargeLive; ctx.lineWidth = 2; ctx.stroke()
      const bays = Array.isArray(st.bays) ? st.bays : []
      const busy = bays.filter((b) => b.van != null || (b.phase && b.phase !== 'free')).length
      const scr = corners.map(([x, y]) => this.P(x, y))
      const xs = scr.map((p) => p[0]), ys = scr.map((p) => p[1])
      const bx0 = Math.min(...xs), bx1 = Math.max(...xs), by0 = Math.min(...ys), by1 = Math.max(...ys)
      const src = st.source === 'manual' || st.source === 'random' ? t(`dr_cs_src_${st.source}`) : st.source ?? ''
      this.rectHits.push({ x0: bx0, y0: by0, x1: bx1, y1: by1, area: (bx1 - bx0) * (by1 - by0),
        tip: t('dr_cs_live_tip', { n: st.id, src, busy, total: bays.length }) })
      if (this.layers.labels && v.s > 0.7) text(t('dr_cs_live_label', { n: st.id }), bx1 + 3, by0 + 10, MC.chargeLive)
      for (const b of bays) {
        if (!Number.isFinite(b.x) || !Number.isFinite(b.y)) continue
        const p = this.P(b.x, b.y), r = Math.max(5, 1.4 * v.s)
        const col = BAY_COL[b.phase] ?? MC.ring, free = b.phase === 'free' && b.van == null
        ctx.beginPath(); ctx.roundRect(p[0] - r, p[1] - r, 2 * r, 2 * r, 2)
        ctx.fillStyle = free ? MC.compassBg : col; ctx.fill()
        ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.stroke()
        if (r >= 6) {
          ctx.fillStyle = free ? col : MC.zoneText; ctx.font = font('800', Math.min(12, r * 1.2)); ctx.textAlign = 'center'
          ctx.fillText(b.kind === 'lmh' ? 'H' : 'P', p[0], p[1] + Math.min(12, r * 1.2) * 0.36)
        }
        if (b.van != null && this.layers.labels) text(`V${b.van}`, p[0] + r + 2, p[1] + 4, podColor(b.van))
        const ph = t(`dr_cs_ph_${b.phase}`)
        this.hits.push({ x: p[0], y: p[1], r: r + 3, tip: t('dr_cs_bay_tip', {
          n: st.id, kind: t(b.kind === 'lmh' ? 'dr_cs_bay_lmh' : 'dr_cs_bay_lmp'),
          phase: ph === `dr_cs_ph_${b.phase}` ? String(b.phase) : ph,
        }) + (b.van != null ? ` · ${t('pd_pod')} ${b.van}` : '') })
      }
    }

    // spots the sim rejected / シミュが却下した地点
    for (const e of errors) {
      if (!Number.isFinite(e.x) || !Number.isFinite(e.y)) continue
      const p = this.P(e.x, e.y)
      if (off(p, 20)) continue
      ctx.beginPath(); ctx.arc(p[0], p[1], 9, 0, 7); ctx.fillStyle = MC.compassBg; ctx.fill()
      cross(p, 5.5)
      this.hits.push({ x: p[0], y: p[1], r: 11, tip: t('dr_cs_rejected', { why: String(e.why || '?') }) })
    }

    // planned spots (the editor's) / 予定地点
    if (!z) return
    z.charge.forEach((c, i) => {
      const p = this.P(c.x, c.y)
      if (off(p, Math.max(AUTO_R, 14) * v.s + 30)) return
      const issue = this.checker?.check(c) ?? null
      const why = simError(errors, c)
      const bad = !!issue || !!why
      const built = live.some((st) => Math.hypot(st.x - c.x, st.y - c.y) < 2.5)
      const sel = this.editing && sameRef(this.sel, chargeRef(i))
      const edge = bad ? MC.nofly : MC.charge
      if (!built) {
        if (c.yaw != null) {
          path(footprint(c.x, c.y, c.yaw))
          ctx.fillStyle = bad ? MC.noflyFill : MC.chargeFill; ctx.fill()
          ctx.strokeStyle = edge; ctx.lineWidth = sel ? 3 : 2
          if (bad) ctx.setLineDash([6, 4])
          ctx.stroke(); ctx.setLineDash([])
          // its front (+x) / 正面
          if (v.s > 1.2) {
            const a = toWorld(c.x, c.y, c.yaw, 9.98, -2.3), b = toWorld(c.x, c.y, c.yaw, 13.5, -2.3)
            const pa = this.P(a[0], a[1]), pb = this.P(b[0], b[1])
            this.arrow(ctx, pa[0], pa[1], pb[0] - pa[0], pb[1] - pa[1], edge, 2)
          }
        } else {
          ctx.beginPath(); ctx.arc(p[0], p[1], Math.max(AUTO_R * v.s, 4), 0, 7)
          ctx.fillStyle = bad ? MC.noflyFill : MC.chargeFill; ctx.fill()
          ctx.strokeStyle = edge; ctx.lineWidth = sel ? 2.5 : 1.6; ctx.setLineDash([5, 4]); ctx.stroke(); ctx.setLineDash([])
        }
      }
      // bolt marker + number / 稲妻の印と番号
      const k = 9
      ctx.beginPath(); ctx.roundRect(p[0] - k, p[1] - k, 2 * k, 2 * k, 4)
      ctx.fillStyle = bad ? MC.compassBg : MC.charge; ctx.fill()
      ctx.strokeStyle = bad ? MC.nofly : MC.zoneText; ctx.lineWidth = 2; ctx.stroke()
      this.bolt(ctx, p[0], p[1], 6, bad ? MC.nofly : MC.zoneText)
      if (bad) cross(p, k + 2)
      text(String(i + 1), p[0] + k + 3, p[1] - k + 6, bad ? MC.nofly : MC.charge, 11)
      if (sel) {
        ctx.strokeStyle = MC.handle; ctx.lineWidth = 2
        ctx.beginPath(); ctx.roundRect(p[0] - k - 4, p[1] - k - 4, 2 * k + 8, 2 * k + 8, 6); ctx.stroke()
      }
      const yaw = c.yaw != null ? t('dr_cs_yaw', { d: fmt(c.yaw, 0) }) : t('dr_cs_auto')
      this.hits.push({ x: p[0], y: p[1], r: 13, tip: `${t('dr_cs_tip', { i: i + 1, x: fmt(c.x, 1), y: fmt(c.y, 1) })} · ${yaw}` +
        (issue ? ` · ${t(`dr_cs_bad_${issue}`)}` : '') + (why ? ` · ${t('dr_cs_rejected', { why })}` : '') +
        (built ? ` · ${t('dr_cs_built')}` : '') })
    })
  }

  /** Diagonal red hatch for no-fly zones, made once (screen space, so it
   *  keeps its spacing at any zoom). / 斜線パターン（一度だけ作成） */
  private hatchPattern(ctx: CanvasRenderingContext2D) {
    if (this.hatch) return this.hatch
    const n = 10, c = document.createElement('canvas')
    c.width = c.height = n
    const g = c.getContext('2d')!
    g.strokeStyle = MC.noflyHatch; g.lineWidth = 1.6
    g.beginPath()
    g.moveTo(0, n); g.lineTo(n, 0)
    g.moveTo(-n / 2, n / 2); g.lineTo(n / 2, -n / 2)
    g.moveTo(n / 2, n * 1.5); g.lineTo(n * 1.5, n / 2)
    g.stroke()
    this.hatch = ctx.createPattern(c, 'repeat')
    return this.hatch
  }

  /** Pod zones (translucent fill, solid outline, "Pod N") and no-fly zones
   *  (red hatch, dashed outline, "No-fly"); handles on the selected one while
   *  editing. A handful of rects: cheap enough for every frame.
   *  ゾーン描画（数個の矩形なので毎フレームでも軽い） */
  private drawZones(ctx: CanvasRenderingContext2D, W: number, H: number) {
    const z = this.zones, t = this.t
    if (!z) return
    const label = (txt: string, x: number, y: number, col: string) => {
      ctx.font = `700 12px ${FONT_SANS}`; ctx.textAlign = 'left'
      ctx.lineWidth = 3.5; ctx.strokeStyle = MC.zoneText; ctx.lineJoin = 'round'
      ctx.strokeText(txt, x, y); ctx.fillStyle = col; ctx.fillText(txt, x, y)
    }
    const size = (r: Rect) => ({ w: fmt(r.x1 - r.x0, 0), h: fmt(r.y1 - r.y0, 0) })
    let selBox: [number, number, number, number, string] | null = null
    const one = (ref: ZoneRef, r: Rect, col: string, nofly: boolean, txt: string, tip: string) => {
      const a = this.P(r.x0, r.y0), b = this.P(r.x1, r.y1)
      if (b[0] < 0 || b[1] < 0 || a[0] > W || a[1] > H) return     // off screen / 画面外
      const w = b[0] - a[0], h = b[1] - a[1]
      const sel = sameRef(ref, this.sel)
      if (nofly) {
        ctx.fillStyle = MC.noflyFill; ctx.fillRect(a[0], a[1], w, h)
        const pat = this.hatchPattern(ctx)
        if (pat) { ctx.fillStyle = pat; ctx.fillRect(a[0], a[1], w, h) }
        ctx.setLineDash([7, 4])
      } else {
        ctx.globalAlpha = sel ? 0.24 : 0.15
        ctx.fillStyle = col; ctx.fillRect(a[0], a[1], w, h)
        ctx.globalAlpha = 1
      }
      ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2
      ctx.strokeRect(a[0], a[1], w, h)
      ctx.setLineDash([])
      label(txt, Math.max(a[0], 0) + 6, Math.max(a[1], 0) + 16, col)
      this.rectHits.push({ x0: a[0], y0: a[1], x1: b[0], y1: b[1], area: w * h, tip })
      if (sel && this.editing) selBox = [a[0], a[1], b[0], b[1], col]
    }
    const lmhIds = new Set((this.state?.vans ?? []).filter((v) => v.kind === 'lmh').map((v) => String(v.id)))
    if (this.zoneOn('pod')) for (const [id, r] of Object.entries(z.pods)) {
      const name = lmhIds.has(id) ? t('dr_hub_label', { n: id }) : t('dr_zone_label', { n: id })
      one({ kind: 'pod', id }, r, podColor(id), false, name, t('dr_zone_tip', { n: id, ...size(r) }).replace(t('dr_zone_label', { n: id }), name))
    }
    // LMH hubs on the default area deliver inside the no-fly zones: a thin
    // inset outline per hub in its colour / 既定（飛行禁止区域）を使う LMH ハブの細い枠
    const lmhDefault = (this.state?.vans ?? []).filter((v) => v.kind === 'lmh' && lmhArea(z, v.id, v.area) === 'nofly')
    if (this.zoneOn('nofly')) for (const n of z.nofly) {
      const name = n.name ? ` · ${n.name}` : ''
      const hubs = lmhDefault.length ? ` · ${t('dr_nofly_lmh_tip', { hubs: lmhDefault.map((v) => v.id).join(', ') })}` : ''
      one({ kind: 'nofly', id: n.id }, n, MC.nofly, true, `${t('dr_zone_nofly')}${name}`, `${t('dr_nofly_tip', size(n))}${name}${hubs}`)
      const a = this.P(n.x0, n.y0), b = this.P(n.x1, n.y1)
      if (b[0] < 0 || b[1] < 0 || a[0] > W || a[1] > H) continue
      lmhDefault.forEach((v, i) => {
        const d = 4 + i * 3
        if (b[0] - a[0] < 2 * d + 4 || b[1] - a[1] < 2 * d + 4) return
        ctx.strokeStyle = podColor(v.id); ctx.lineWidth = 1.2
        ctx.strokeRect(a[0] + d, a[1] + d, b[0] - a[0] - 2 * d, b[1] - a[1] - 2 * d)
      })
    }
    if (selBox) {
      const [x0, y0, x1, y1, col] = selBox as [number, number, number, number, string]
      const xm = (x0 + x1) / 2, ym = (y0 + y1) / 2, k = 4.5
      ctx.fillStyle = MC.handle; ctx.strokeStyle = col; ctx.lineWidth = 2
      for (const [hx, hy] of [[x0, y0], [xm, y0], [x1, y0], [x1, ym], [x1, y1], [xm, y1], [x0, y1], [x0, ym]]) {
        ctx.fillRect(hx - k, hy - k, 2 * k, 2 * k); ctx.strokeRect(hx - k, hy - k, 2 * k, 2 * k)
      }
    }
  }
}
