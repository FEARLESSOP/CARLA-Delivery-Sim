// The City Map renderer, kept out of React: it redraws at most once per screen
// frame (requestAnimationFrame), whatever arrives (state polls, wheel, drag).
// The map itself (lanes, markings, buildings) is a few Path2D objects in world
// metres built once per map, painted into an oversized cached image (the view
// plus half a view on every side): panning only slides that image, zooming
// stretches it and re-renders it sharp 140 ms after the wheel stops. That is
// the design that took panning from 7.7 to 100 frames/s on OitaKosen.
// 地図描画（React 外）：1 フレーム 1 回、地図は世界座標の Path2D と大きめの
// キャッシュ画像（移動はずらすだけ、拡大縮小は伸縮して 140 ms 後に描き直す）
import type { TFunc } from '@/lib/i18n'
import { C, FONT_SANS } from '@/lib/colors'
import { fmt, clamp } from '@/lib/fmt'
import type { DeliveryMap, DeliveryState } from './types'
import { MC, DRONE_COL, battCol, droneSt, orderSt } from './shared'

interface View { cx: number; cy: number; s: number }
interface Hit { x: number; y: number; r: number; tip: string }
interface Paths { s: Path2D; h: Path2D; d: Path2D; bld: Path2D; w: Path2D; wb: Path2D; y: Path2D; yb: Path2D; roads: Path2D }
interface Base { cv: HTMLCanvasElement; map: DeliveryMap; s: number; cx: number; cy: number; BW: number; BH: number; W: number; H: number; dpr: number }

const MARGIN = 0.5          // base image covers the view + half a view each side / 周囲半画面分
const ZMIN = 0.05, ZMAX = 40
const FLYING_ALL = ['delivering', 'to_station', 'to_van', 'falling']

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
  private ro: ResizeObserver | null = null

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

  /** One redraw per screen frame however many events arrive. / 1 フレームに 1 回 */
  request() {
    if (this.raf) return
    this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw() })
  }

  get hasView() { return !!this.view }

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
    return best ? best.tip : null
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
    ctx.lineJoin = 'round'
    ctx.fillStyle = m.lanes ? MC.bld : MC.oldBld
    ctx.strokeStyle = m.lanes ? MC.bldEdge : MC.oldBldEdge; ctx.lineWidth = px
    ctx.fill(P.bld); ctx.stroke(P.bld)
    if (m.lanes) {
      // pavements and shoulders under the carriageways, each outlined in its
      // own colour so neighbouring lanes meet without a seam / 継ぎ目なし
      ctx.lineWidth = 1.2 * px
      const layers: [Path2D, string][] = [[P.s, MC.side], [P.h, MC.shoulder], [P.d, MC.road]]
      for (const [p, col] of layers) { ctx.fillStyle = ctx.strokeStyle = col; ctx.fill(p); ctx.stroke(p) }
      // lane markings, only once big enough to read / 区画線（拡大時のみ）
      if (v.s > 0.6) {
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
      ctx.stroke(P.roads)
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
    const font = (w: string, px: number) => `${w} ${px}px ${FONT_SANS}`
    if (!m) {
      ctx.fillStyle = C.mute; ctx.font = font('400', 13); ctx.textAlign = 'center'
      ctx.fillText(t('dr_nomap'), W / 2, H / 2)
      return
    }
    if (!this.view) this.computeFit()
    const v = this.view!
    this.baseDraw(ctx, W, H, dpr)
    const R = (k: number) => Math.max(k, k * v.s / 2.2)  // markers grow when zoomed in

    // charge station / 充電所
    ctx.fillStyle = MC.stationFill; ctx.strokeStyle = MC.station; ctx.lineWidth = 1
    for (const [x, y] of m.station || []) {
      const p = this.P(x, y), r = R(3.5)
      ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r)
    }
    if (m.station?.length) {
      const p = this.P(m.station[0][0], m.station[0][1])
      ctx.fillStyle = MC.station; ctx.font = font('600', 11); ctx.textAlign = 'left'
      ctx.fillText(t('dr_lg_station'), p[0] + 10, p[1] - 8)
      hits.push({ x: p[0], y: p[1], r: 12, tip: `${t('dr_lg_station')} · ${t('dr_tip_pads', { n: m.station.length })}` })
    }
    // EV chargers for the pods / ポッド用 EV 充電器
    for (const [x, y] of m.chargers || []) {
      const p = this.P(x, y), r = R(4.5)
      ctx.fillStyle = MC.evFill; ctx.strokeStyle = MC.ev; ctx.lineWidth = 1.5
      ctx.fillRect(p[0] - r, p[1] - r, 2 * r, 2 * r); ctx.strokeRect(p[0] - r, p[1] - r, 2 * r, 2 * r)
      ctx.fillStyle = MC.ev; ctx.font = font('700', 11); ctx.textAlign = 'center'
      ctx.fillText('EV', p[0], p[1] + 3)
      hits.push({ x: p[0], y: p[1], r: 9, tip: t('dr_lg_ev') })
    }
    // LastMile box stations on the pavements / 歩道の箱型ステーション
    ;(m.boxes || []).forEach(([x, y], i) => {
      const p = this.P(x, y), r = R(3.5)
      ctx.fillStyle = MC.box; ctx.strokeStyle = MC.boxEdge; ctx.lineWidth = 1.5
      ctx.fillRect(p[0] - r, p[1] - r * 1.3, 2 * r, 2.6 * r); ctx.strokeRect(p[0] - r, p[1] - r * 1.3, 2 * r, 2.6 * r)
      hits.push({ x: p[0], y: p[1], r: 8, tip: `${t('dr_lg_box')} ${i + 1}` })
    })
    if (!s || !s.available) return

    const drones = s.drones || []
    // drones on the pads, listed beside the station / 充電パッド上の機体
    const parked = drones.filter((d) => d.status === 'charging' || d.status === 'standby')
    if (parked.length && m.station?.length) {
      const p = this.P(m.station[0][0], m.station[0][1])
      ctx.font = font('600', 11.5); ctx.textAlign = 'left'
      parked.forEach((d, i) => {
        ctx.fillStyle = battCol(d.battery)
        ctx.fillText(`D${d.id} ${Math.round(d.battery)}% ${droneSt(t, d.status)}`, p[0] + 10, p[1] + 7 + 13 * i)
      })
    }
    // hubs / ドローン地点
    for (const h of s.hubs || []) {
      const p = this.P(h.x, h.y), r = R(5)
      ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(Math.PI / 4)
      ctx.strokeStyle = MC.hub; ctx.lineWidth = 2; ctx.strokeRect(-r, -r, 2 * r, 2 * r)
      ctx.restore()
      hits.push({ x: p[0], y: p[1], r: 8, tip: `${t('dr_lg_hub')} · ${t('pd_pod')} ${h.van}` })
    }
    // orders / 注文
    for (const o of s.orders || []) {
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
        (o.van ? ` · ${t('pd_pod')} ${o.van}` : '') + (o.reason ? ` · ${o.reason}` : '') })
    }
    // other cars, at least 7 x 3.5 px however far out / 他の車（縮小時も最小サイズ）
    ctx.fillStyle = MC.car
    for (const [x, y, yaw] of s.cars || []) {
      this.box(ctx, x, y, Math.max(2.3, 3.5 / v.s), Math.max(1.0, 1.75 / v.s), yaw); ctx.fill()
    }
    // pods / 配送ポッド
    const vanPos = new Map<number, [number, number]>()
    for (const van of s.vans || []) {
      vanPos.set(van.id, [van.x, van.y])
      ctx.fillStyle = MC.pod; ctx.strokeStyle = battCol(van.battery); ctx.lineWidth = 2.2
      this.box(ctx, van.x, van.y, Math.max(2.3, 7 / v.s), Math.max(1.1, 3.5 / v.s), van.yaw)
      ctx.fill(); ctx.stroke()
      const p = this.P(van.x, van.y)
      ctx.fillStyle = MC.label; ctx.font = font('700', 11); ctx.textAlign = 'center'
      ctx.fillText(`V${van.id} ${Math.round(van.battery)}%`, p[0], p[1] - 9)
      hits.push({ x: p[0], y: p[1], r: 10, tip: `${t('pd_pod')} ${van.id} · ${t('dr_th_batt')} ${Math.round(van.battery)}% · ${van.state} · ` +
        `${van.drone ? `D${van.drone}` : t('dr_nodrone')} · ${van.delivered} ✓ ${van.failed} ✗` })
    }
    // drones / ドローン
    for (const d of drones) {
      const p = this.P(d.x, d.y)
      const flying = FLYING_ALL.includes(d.status)
      const vp = d.van != null ? vanPos.get(d.van) : undefined
      if (flying && vp && d.status !== 'to_station') {
        const q = this.P(vp[0], vp[1])
        ctx.strokeStyle = MC.link; ctx.setLineDash([4, 4]); ctx.lineWidth = 1
        ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(p[0], p[1]); ctx.stroke(); ctx.setLineDash([])
      }
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
      if (flying || d.status === 'crashed') {      // pad drones are listed by the station
        ctx.fillStyle = MC.label; ctx.font = font('700', 11.5); ctx.textAlign = 'left'
        ctx.fillText(`D${d.id} ${Math.round(d.battery)}%`, p[0] + r + 3, p[1] + 4)
      }
      hits.push({ x: p[0], y: p[1], r: r + 4, tip: `D${d.id} · ${droneSt(t, d.status)} · ${Math.round(d.battery)}% · z ${fmt(d.z, 0)} m` +
        (d.note ? ` · ${d.note}` : '') })
    }
    // wind compass in the corner (map orientation) / 隅に風向（地図と同じ向き）
    const w = s.wind || {}
    const cx = W - 54, cy = 54
    ctx.fillStyle = MC.compassBg; ctx.strokeStyle = MC.compassEdge; ctx.lineWidth = 1
    ctx.beginPath(); ctx.arc(cx, cy, 42, 0, 7); ctx.fill(); ctx.stroke()
    const L = Math.min(36, 6 + (w.speed || 0) * 2.2), a = (w.dir || 0) * Math.PI / 180
    this.arrow(ctx, cx - L / 2 * Math.cos(a), cy - L / 2 * Math.sin(a), L * Math.cos(a), L * Math.sin(a),
      w.gust ? MC.gust : MC.windArrow, 3)
    ctx.fillStyle = w.gust ? MC.gust : MC.windText; ctx.textAlign = 'center'; ctx.font = font('700', 11)
    ctx.fillText(`${fmt(w.speed || 0, 1)} m/s${w.gust ? ` ${t('dr_gust')}` : ''}`, cx, cy + 58)
  }
}
