// Pod delivery zones and no-fly zones: rectangle maths for the City Map editor
// and the /delivery/zones load / save hook (optimistic, rolled back on error).
// Points: pod meeting points ("meet") and LastMile Hub distribution points
// ("dist", where an LMH hub parks to send its mini pods out); both are posted
// together with pods / nofly every time (the server replaces the entry).
// 点：ポッドの合流地点（meet）と LMH ハブの配布地点（dist）
// Charging stations ("charge", one list per map, <= 12): LastMile charging
// station spots with an optional yaw, placed before a launch.
// 配送ゾーン・飛行禁止区域：矩形の計算と /delivery/zones の読み書き
// 充電ステーション（charge、地図ごとに最大 12）
import { useCallback, useEffect, useRef, useState } from 'react'
import { getJSON, postJSON } from '@/lib/api'
import type { ChargeSpot, MeetPoint, NoFly, Rect, ZoneSet, ZonesReply } from './types'

/** the server drops smaller rectangles / これより小さい矩形はサーバーが捨てる */
export const MIN_SIDE = 5
export const MAX_NOFLY = 32
export const MAX_MEET = 24
/** distribution points per LastMile Hub (LMH) / ハブごとの配布地点の上限 */
export const MAX_DIST = 24
/** charging stations per map (zones.MAX_CHARGE) / 地図ごとの充電ステーション上限 */
export const MAX_CHARGE = 12

/** the two point kinds: pod meeting points and LMH distribution points / 点の種類 */
export type PointKind = 'meet' | 'dist'
export const isPointKind = (k: string): k is PointKind => k === 'meet' || k === 'dist'

/** a pod zone, a no-fly zone, meeting point ``i`` of pod ``id``, or
 *  distribution point ``i`` of LMH hub ``id`` / ゾーン・合流地点・配布地点 */
export type ZoneRef = { kind: 'pod'; id: string } | { kind: 'nofly'; id: string }
  | { kind: 'meet'; id: string; i: number } | { kind: 'dist'; id: string; i: number }
  | { kind: 'charge'; id: 'charge'; i: number }
export type PointRef = Extract<ZoneRef, { kind: PointKind }>
/** charging-station spot ``i`` of the map / 充電ステーション地点 i */
export type ChargeRef = Extract<ZoneRef, { kind: 'charge' }>
export const chargeRef = (i: number): ChargeRef => ({ kind: 'charge', id: 'charge', i })
/** what a drag grabs: the body, an edge or a corner (n = y0 side, w = x0 side) */
export type Handle = 'move' | 'n' | 's' | 'e' | 'w' | 'ne' | 'nw' | 'se' | 'sw'
/** the drawing tool; null = select / move / pan; meet / dist = add meeting
 *  points of pod ``id`` / distribution points of LMH hub ``id``
 *  null は選択・移動、meet は合流地点、dist は配布地点を追加 */
export type Tool = { kind: 'pod'; id: string } | { kind: 'nofly' } | { kind: PointKind; id: string } | { kind: 'charge' } | null

export const EMPTY_ZONES: ZoneSet = { pods: {}, nofly: [], meet: {}, dist: {}, charge: [] }

const isPointRef = (r: ZoneRef): r is PointRef => isPointKind(r.kind)
/** a ref with an index (points, charging stations) / 番号付きの参照 */
const hasIndex = (r: ZoneRef): r is PointRef | ChargeRef => isPointKind(r.kind) || r.kind === 'charge'

export const sameRef = (a: ZoneRef | null, b: ZoneRef | null) =>
  !!a && !!b && a.kind === b.kind && a.id === b.id && (!hasIndex(a) || (hasIndex(b) && a.i === b.i))

/** The meeting / distribution point ``ref`` names, or null. / 指定の点 */
export function getPoint(z: ZoneSet, ref: ZoneRef | null): MeetPoint | null {
  return ref && isPointRef(ref) ? z[ref.kind][ref.id]?.[ref.i] ?? null : null
}
/** @deprecated name kept for older callers: any point kind / 互換名 */
export const getMeet = getPoint

/** The charging-station spot ``ref`` names, or null. / 指定の充電ステーション地点 */
export function getCharge(z: ZoneSet, ref: ZoneRef | null): ChargeSpot | null {
  return ref && ref.kind === 'charge' ? z.charge[ref.i] ?? null : null
}

/** the zone or point still exists / まだ存在する */
export const refExists = (z: ZoneSet, ref: ZoneRef) =>
  (ref.kind === 'charge' ? !!getCharge(z, ref) : isPointRef(ref) ? !!getPoint(z, ref) : !!getRect(z, ref))

/** a yaw in degrees wrapped to [-180, 180), 0.1 deg (as zones.clean_charge) / 向きを正規化 */
export const wrapYaw = (a: number) => r1((((a + 180) % 360) + 360) % 360 - 180)

/** A copy of ``z`` with charging-station spot ``i`` set (i = length appends);
 *  ``yaw`` undefined = automatic (the sim faces the kerb). / 充電ステーション地点を設定 */
export function setCharge(z: ZoneSet, i: number, p: ChargeSpot): ZoneSet {
  const charge = z.charge.slice()
  const q: ChargeSpot = { x: r1(p.x), y: r1(p.y) }
  if (p.yaw != null && Number.isFinite(p.yaw)) q.yaw = wrapYaw(p.yaw)
  charge[i] = q
  return { ...z, charge }
}

/** A copy of ``z`` with point ``i`` of kind ``kind`` for ``id`` set (i = length appends). */
export function setPoint(z: ZoneSet, kind: PointKind, id: string, i: number, p: MeetPoint): ZoneSet {
  const pts = (z[kind][id] ?? []).slice()
  pts[i] = { x: r1(p.x), y: r1(p.y) }
  return { ...z, [kind]: { ...z[kind], [id]: pts } }
}

/** A copy of ``z`` with meeting point ``i`` of pod ``id`` set (i = length appends). */
export const setMeet = (z: ZoneSet, id: string, i: number, p: MeetPoint) => setPoint(z, 'meet', id, i, p)

/** The delivery area an LMH hub uses: its own pod zone once one is drawn
 *  (wins), else what the sim says, else the default: the no-fly zones.
 *  LMH ハブの配達範囲：ゾーンがあればゾーン、無ければ既定の飛行禁止区域 */
export function lmhArea(z: ZoneSet, id: string | number, simArea?: string): 'nofly' | 'zone' | 'any' {
  if (z.pods[String(id)]) return 'zone'
  return simArea === 'any' || simArea === 'zone' ? simArea : 'nofly'
}

/** the most points one owner may have / 1 つの持ち主の点の上限 */
export const maxPoints = (kind: PointKind) => (kind === 'dist' ? MAX_DIST : MAX_MEET)

/** the point ``(x, y)`` lies inside a no-fly zone / 飛行禁止区域の中 */
export const inNoFly = (z: ZoneSet, x: number, y: number) =>
  z.nofly.some((n) => x >= n.x0 && x <= n.x1 && y >= n.y0 && y <= n.y1)

export const mapKey = (m?: string) => (m || '').replace(/\\/g, '/').replace(/\/+$/, '').split('/').pop() || ''

const r1 = (v: number) => Math.round(v * 10) / 10

export function normRect(ax: number, ay: number, bx: number, by: number): Rect {
  return { x0: r1(Math.min(ax, bx)), y0: r1(Math.min(ay, by)), x1: r1(Math.max(ax, bx)), y1: r1(Math.max(ay, by)) }
}

export const bigEnough = (r: Rect) => r.x1 - r.x0 >= MIN_SIDE && r.y1 - r.y0 >= MIN_SIDE

/** ``start`` dragged by (dx, dy) metres on ``h``; edges never come closer than
 *  MIN_SIDE to the opposite edge. / 反対側の辺に MIN_SIDE 以上近づかない */
export function dragRect(start: Rect, h: Handle, dx: number, dy: number): Rect {
  if (h === 'move') return { x0: r1(start.x0 + dx), y0: r1(start.y0 + dy), x1: r1(start.x1 + dx), y1: r1(start.y1 + dy) }
  let { x0, y0, x1, y1 } = start
  if (h.includes('w')) x0 = Math.min(start.x0 + dx, start.x1 - MIN_SIDE)
  if (h.includes('e')) x1 = Math.max(start.x1 + dx, start.x0 + MIN_SIDE)
  if (h.includes('n')) y0 = Math.min(start.y0 + dy, start.y1 - MIN_SIDE)
  if (h.includes('s')) y1 = Math.max(start.y1 + dy, start.y0 + MIN_SIDE)
  return { x0: r1(x0), y0: r1(y0), x1: r1(x1), y1: r1(y1) }
}

export function getRect(z: ZoneSet, ref: ZoneRef | null): Rect | null {
  if (!ref || hasIndex(ref)) return null
  if (ref.kind === 'pod') return z.pods[ref.id] ?? null
  return z.nofly.find((n) => n.id === ref.id) ?? null
}

/** A copy of ``z`` with ``ref`` set to ``r`` (a new no-fly id is appended). */
export function setRect(z: ZoneSet, ref: ZoneRef, r: Rect): ZoneSet {
  const rect = { x0: r.x0, y0: r.y0, x1: r.x1, y1: r.y1 }
  if (hasIndex(ref)) return z
  if (ref.kind === 'pod') return { ...z, pods: { ...z.pods, [ref.id]: rect } }
  const i = z.nofly.findIndex((n) => n.id === ref.id)
  const nofly = z.nofly.slice()
  if (i < 0) nofly.push({ ...rect, id: ref.id })
  else nofly[i] = { ...nofly[i], ...rect }
  return { ...z, nofly }
}

/** A copy of ``z`` without the zone or point ``ref``. */
export function removeRect(z: ZoneSet, ref: ZoneRef): ZoneSet {
  if (ref.kind === 'charge') return { ...z, charge: z.charge.filter((_, j) => j !== ref.i) }
  if (isPointRef(ref)) {
    const all = { ...z[ref.kind] }
    const pts = (all[ref.id] ?? []).filter((_, j) => j !== ref.i)
    if (pts.length) all[ref.id] = pts; else delete all[ref.id]
    return { ...z, [ref.kind]: all }
  }
  if (ref.kind === 'pod') {
    const pods = { ...z.pods }
    delete pods[ref.id]
    return { ...z, pods }
  }
  return { ...z, nofly: z.nofly.filter((n) => n.id !== ref.id) }
}

const num = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : NaN)
function readRect(v: unknown): Rect | null {
  if (!v || typeof v !== 'object') return null
  const o = v as Record<string, unknown>
  const r = { x0: num(o.x0), y0: num(o.y0), x1: num(o.x1), y1: num(o.y1) }
  return Object.values(r).every(Number.isFinite) ? normRect(r.x0, r.y0, r.x1, r.y1) : null
}

/** A loosely typed reply (or state.zones) as a clean ZoneSet. / 応答を検証 */
export function readZones(r: Partial<ZoneSet> | null | undefined): ZoneSet {
  const pods: Record<string, Rect> = {}
  for (const [k, v] of Object.entries(r?.pods ?? {})) { const c = readRect(v); if (c) pods[k] = c }
  const nofly: NoFly[] = []
  for (const v of Array.isArray(r?.nofly) ? r.nofly : []) {
    const c = readRect(v)
    if (c && typeof v.id === 'string') nofly.push({ ...c, id: v.id, ...(typeof v.name === 'string' ? { name: v.name } : {}) })
  }
  // older servers have no "meet" / "dist": none / 古いサーバーは meet・dist 無し
  const points = (src: Record<string, MeetPoint[]> | undefined, max: number) => {
    const all: Record<string, MeetPoint[]> = {}
    for (const [k, pts] of Object.entries(src ?? {})) {
      const out: MeetPoint[] = []
      for (const p of Array.isArray(pts) ? pts : []) {
        const x = num(p?.x), y = num(p?.y)
        if (Number.isFinite(x) && Number.isFinite(y)) out.push({ x: r1(x), y: r1(y) })
      }
      if (out.length) all[k] = out.slice(0, max)
    }
    return all
  }
  // older servers have no "charge": none / 古いサーバーは charge 無し
  const charge: ChargeSpot[] = []
  for (const p of Array.isArray(r?.charge) ? r.charge : []) {
    const x = num(p?.x), y = num(p?.y), yaw = num(p?.yaw)
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue
    charge.push(Number.isFinite(yaw) ? { x: r1(x), y: r1(y), yaw: wrapYaw(yaw) } : { x: r1(x), y: r1(y) })
  }
  return { pods, nofly, meet: points(r?.meet, MAX_MEET), dist: points(r?.dist, MAX_DIST), charge: charge.slice(0, MAX_CHARGE) }
}

export type ZoneStatus = 'idle' | 'loading' | 'saving' | 'saved' | 'error' | 'offline' | 'nomap'

/** Zones of the map ``name`` (the sim's current map when empty): load on map
 *  change (retry every 5 s while the route is missing), ``commit`` = set
 *  locally at once and POST (after ``delay`` ms, for key repeats); an error
 *  rolls back to the last saved set. / 楽観的更新、失敗時は元に戻す */
export function useZones(name: string) {
  const [zones, setZones] = useState<ZoneSet>(EMPTY_ZONES)
  const [key, setKey] = useState('')
  const [status, setStatus] = useState<ZoneStatus>('loading')
  const [error, setError] = useState('')
  const saved = useRef<ZoneSet>(EMPTY_ZONES)
  const seq = useRef(0)
  const timer = useRef(0)
  const [retry, setRetry] = useState(0)
  // false: the server's zones module predates charging stations (it drops
  // "charge"): restart the dashboard / false：古いサーバー（charge を捨てる）
  const [chargeOk, setChargeOk] = useState(true)

  useEffect(() => {
    const ac = new AbortController()
    let again = 0
    setStatus('loading')
    getJSON<ZonesReply>(`/delivery/zones${name ? `?map=${encodeURIComponent(name)}` : ''}`, ac.signal)
      .then((r) => {
        const z = readZones(r)
        saved.current = z
        setZones(z)
        setChargeOk(r.ok === false || Array.isArray(r.charge))
        setKey(r.map || name)
        setStatus(r.map || name ? 'idle' : 'nomap')
        if (!(r.map || name)) again = window.setTimeout(() => setRetry((n) => n + 1), 5000)
      })
      .catch((e: unknown) => {
        if (ac.signal.aborted) return
        setKey(name)
        setError(e instanceof Error ? e.message : String(e))
        setStatus('offline')
        again = window.setTimeout(() => setRetry((n) => n + 1), 5000)
      })
    return () => { ac.abort(); clearTimeout(again) }
  }, [name, retry])

  useEffect(() => () => clearTimeout(timer.current), [])

  const commit = useCallback((next: ZoneSet, delay = 0) => {
    setZones(next)
    clearTimeout(timer.current)
    const my = ++seq.current
    if (!key) { setZones(saved.current); setStatus('nomap'); return }
    setStatus('saving')
    const send = () => {
      // the whole entry every time: the server replaces it / サーバーは全体を置き換える
      postJSON<ZonesReply>('/delivery/zones', { map: key, pods: next.pods, nofly: next.nofly, meet: next.meet, dist: next.dist, charge: next.charge })
        .then((r) => {
          if (r.ok === false) throw new Error(r.error || 'error')
          const z = readZones(r)
          saved.current = z
          setChargeOk(Array.isArray(r.charge))
          if (my !== seq.current) return        // a newer edit is on its way / 新しい編集あり
          setZones(z)
          setStatus('saved')
        })
        .catch((e: unknown) => {
          if (my !== seq.current) return
          setZones(saved.current)
          setError(e instanceof Error ? e.message : String(e))
          setStatus('error')
        })
    }
    if (delay > 0) timer.current = window.setTimeout(send, delay)
    else send()
  }, [key])

  return { zones, key, status, error, commit, chargeOk }
}
