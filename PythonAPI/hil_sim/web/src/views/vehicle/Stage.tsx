import { memo, useRef } from 'react'
import { C } from '@/lib/colors'
import { clamp, fmt } from '@/lib/fmt'
import type { TFunc } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { HERO_BODY, HERO_DEFS, NEAR_SYMBOLS } from './assets'
import type { Hit, Kind, VehData } from './data'
import {
  CX, CY, KIND_C, PDC_COL, PDC_THR, ZONES, distPix, fmtDist, hitOpacity, hitPos, placeNear, pdcMargins,
  pdcTextCol, sensDupKeys, shapePath, snMeta, snName, spreadPts, toStage, ultraCol, vzone, type VehState, type Zone,
} from './logic'

// The top-down vehicle visual (SVG, viewBox 300×600, ported from the old
// page's #vsvg): radar rings, own-lane guide, the hero car with steering
// wheels / brake + reverse lights / blinkers / speed streaks, sensor markers,
// impact ripples, park-distance bands, lane-change chevrons, the ultrasonic
// arc and nearby cars / bikes / pedestrians.
// 車両の上面図（旧ページの SVG を JSX に移植）

/* ---------------------------------------------------------- static parts */
/** Nearby-object icon symbols (public-domain drawings, injected once). */
const NearSymbols = memo(function NearSymbols() {
  return <defs dangerouslySetInnerHTML={{ __html: NEAR_SYMBOLS }} />
})
const HeroDefs = memo(function HeroDefs() {
  return <g dangerouslySetInnerHTML={{ __html: HERO_DEFS }} />
})
const HeroBody = memo(function HeroBody() {
  return <g dangerouslySetInnerHTML={{ __html: HERO_BODY }} />
})

// [radius, label, dashed, label y] (label positions as on the old page)
const RINGS: [number, string, boolean, number][] = [[75.9, '5 m', false, 227], [131.5, '15 m', true, 172], [185.9, '30 m', true, 117], [240, '50 m', false, 63]]

const Grid = memo(function Grid({ front, rear }: { front: string; rear: string }) {
  return (
    <g aria-hidden>
      {/* radar grid: sqrt distance scale, r = 240·√(d/50) / √距離スケール */}
      <g className="vx-rings" style={{ color: C.info }}>
        {RINGS.map(([r, , dash]) => <circle key={r} cx={CX} cy={CY} r={r} className={dash ? 'r2' : undefined} />)}
      </g>
      {RINGS.map(([, l, , y]) => (
        <text key={l} className="vx-mono vx-halo-text" x={CX + 4} y={y} fontSize={12} fill={C.mute} strokeWidth={3}>{l}</text>
      ))}
      <line x1={CX} y1={18} x2={CX} y2={582} stroke={C.line} strokeOpacity={0.6} strokeDasharray="2 6" />
      <line x1={8} y1={CY} x2={292} y2={CY} stroke={C.line} strokeOpacity={0.6} strokeDasharray="2 6" />
      <path d="M150,32 l-7,11 h14 Z" fill={C.mute} />
      <text className="vx-mono" x={CX} y={24} textAnchor="middle" fontSize={13} fontWeight={600} fill={C.fg2} letterSpacing={0.6}>{front}</text>
      <text className="vx-mono" x={CX} y={594} textAnchor="middle" fontSize={13} fontWeight={600} fill={C.fg2} letterSpacing={0.6}>{rear}</text>
    </g>
  )
})

/* --------------------------------------------------------------- hero car */
function HeroCar({ v, st, shakeKey }: { v: VehData; st: VehState; shakeKey: number }) {
  const sa = v.steer * 35                                   // approx. road-wheel angle
  const brk = v.brake > 0.03 ? 0.25 + 0.75 * v.brake : 0.12
  const sigL = v.signal === 'left' || v.signal === 'hazard'
  const sigR = v.signal === 'right' || v.signal === 'hazard'
  return (
    <g key={shakeKey} className={cn('vx-car', st, st === 'crash' && 'vx-shake')}>
      {/* car: "car topview" (openclipart.org, CC0); wheel pivots in its local coords */}
      <g transform="translate(86.35,160) scale(0.35471)">
        <HeroDefs />
        <rect rx={8.6} ry={8.6} height={78.7} width={27.8} y={623} x={16.3} fill={C.bg} />
        <rect rx={8.6} ry={8.6} height={78.7} width={27.8} y={613} x={311.3} fill={C.bg} />
        <rect rx={8.6} ry={8.6} height={78.7} width={27.8} y={98} x={318.8} fill={C.bg} transform={`rotate(${fmt(sa, 1)} 332.7 137.4)`} />
        <rect rx={8.6} ry={8.6} height={78.7} width={27.8} y={101.1} x={8.6} fill={C.bg} transform={`rotate(${fmt(sa, 1)} 22.5 140.5)`} />
        <HeroBody />
      </g>
      <g fill={C.fg} opacity={0.35} style={{ filter: `drop-shadow(0 0 5px ${C.s1})` }}>
        <rect x={107} y={164} width={24} height={6} rx={3} />
        <rect x={169} y={164} width={24} height={6} rx={3} />
      </g>
      <g className="vx-tail" fill={C.s2} opacity={brk} style={{ filter: `drop-shadow(0 0 6px ${C.s2})` }}>
        <rect x={110} y={429} width={26} height={6} rx={3} />
        <rect x={164} y={429} width={26} height={6} rx={3} />
      </g>
      <rect className="vx-rev" x={143} y={429} width={14} height={5} rx={2.5} fill={C.fg} opacity={v.reverse ? 0.9 : 0}
        style={{ filter: `drop-shadow(0 0 6px ${C.fg})` }} />
      {/* blinker lamps / ウインカーランプ */}
      <g className={cn('vx-lamp', sigL && 'on')} fill={C.warn} style={{ color: C.warn }}>
        <rect x={99} y={170} width={7} height={6} rx={2} />
        <rect x={100} y={427} width={7} height={6} rx={2} />
      </g>
      <g className={cn('vx-lamp', sigR && 'on')} fill={C.warn} style={{ color: C.warn }}>
        <rect x={194} y={170} width={7} height={6} rx={2} />
        <rect x={193} y={427} width={7} height={6} rx={2} />
      </g>
    </g>
  )
}

function Streaks({ speed }: { speed: number }) {
  const op = clamp((speed - 4) / 40, 0, 0.75)
  const sd = `${clamp(60 / Math.max(speed, 1), 0.25, 3)}s`
  const L: [number, number, number, string][] = [[70, 130, 490, '0s'], [84, 110, 500, '-.4s'], [216, 110, 500, '-.8s'], [230, 130, 490, '-.2s']]
  return (
    <g className="vx-streaks" style={{ opacity: op, ['--sd' as string]: sd }} aria-hidden>
      {L.map(([x, y1, y2, d]) => <line key={x} className="vx-streak" x1={x} y1={y1} x2={x} y2={y2} stroke={C.info} style={{ animationDelay: d }} />)}
    </g>
  )
}

/* ------------------------------------------------------------ the overlays */
// park-distance bands: inner red at < 0.9 m clearance, amber < 1.8 m, green < 3.0 m
const PDC_GEOM: Record<Zone, ({ d: string } | { x1: number; y1: number; x2: number; y2: number })[]> = {
  f: [{ d: 'M130.1,225.3 A26,26 0 0 1 169.9,225.3' }, { d: 'M120.9,217.6 A38,38 0 0 1 179.1,217.6' }, { d: 'M111.7,209.9 A50,50 0 0 1 188.3,209.9' }],
  b: [{ d: 'M169.9,374.7 A26,26 0 0 1 130.1,374.7' }, { d: 'M179.1,382.4 A38,38 0 0 1 120.9,382.4' }, { d: 'M188.3,390.1 A50,50 0 0 1 111.7,390.1' }],
  l: [{ x1: 116, y1: 272, x2: 116, y2: 328 }, { x1: 105, y1: 272, x2: 105, y2: 328 }, { x1: 94, y1: 272, x2: 94, y2: 328 }],
  r: [{ x1: 184, y1: 272, x2: 184, y2: 328 }, { x1: 195, y1: 272, x2: 195, y2: 328 }, { x1: 206, y1: 272, x2: 206, y2: 328 }],
}
const PDC_TEXT: Record<Zone, [number, number]> = { f: [150, 198], b: [150, 412], l: [98, 350], r: [202, 350] }

function Pdc({ v }: { v: VehData }) {
  const m = pdcMargins(v.near, v.bb)
  return (
    <g>
      {ZONES.map((z) => PDC_GEOM[z].map((g, i) => {
        const p = { className: cn('vx-pdc', m[z] <= PDC_THR[i] && 'lit'), stroke: PDC_COL[i], style: { color: PDC_COL[i] } }
        return 'd' in g ? <path key={`${z}${i}`} {...p} d={g.d} /> : <line key={`${z}${i}`} {...p} {...g} />
      }))}
      {ZONES.map((z) => m[z] <= 3.0 && (
        <text key={z} className="vx-mono vx-halo-text" x={PDC_TEXT[z][0]} y={PDC_TEXT[z][1]} textAnchor="middle"
          fontSize={14} fontWeight={700} strokeWidth={3.5} fill={pdcTextCol(m[z])}>{fmt(m[z], 1)} m</text>
      ))}
    </g>
  )
}

function Chevrons({ signal }: { signal: VehData['signal'] }) {
  return (
    <g aria-hidden style={{ color: C.accent }}>
      <g className={signal === 'left' ? 'on' : undefined} stroke={C.accent}>
        <path className="vx-chev" d="M96,284 L80,300 L96,316" />
        <path className="vx-chev" d="M78,284 L62,300 L78,316" />
        <path className="vx-chev" d="M60,284 L44,300 L60,316" />
      </g>
      <g className={signal === 'right' ? 'on' : undefined} stroke={C.accent}>
        <path className="vx-chev" d="M204,284 L220,300 L204,316" />
        <path className="vx-chev" d="M222,284 L238,300 L222,316" />
        <path className="vx-chev" d="M240,284 L256,300 L240,316" />
      </g>
    </g>
  )
}

function Ultra({ d }: { d: number }) {
  if (!(d > 0)) return null
  const r = distPix(d), a = (18 * Math.PI) / 180
  const xw = Math.sin(a) * r, y = CY - Math.cos(a) * r, c = ultraCol(d)
  return (
    <g>
      <path className="vx-ultra" stroke={c} style={{ color: c }}
        d={`M${fmt(CX - xw, 1)},${fmt(y, 1)} A${fmt(r, 1)},${fmt(r, 1)} 0 0 1 ${fmt(CX + xw, 1)},${fmt(y, 1)}`} />
      {/* readout only once the arc clears the PDC labels / PDC 表示と重ならない距離だけ */}
      {d >= 8 && <text className="vx-mono vx-halo-text" x={CX + xw + 5} y={y + 4} fontSize={12} fontWeight={600} strokeWidth={3} fill={c}>
        {fmt(d, 1)} m
      </text>}
    </g>
  )
}

function Lane({ v }: { v: VehData }) {
  if (!(v.onRoad && v.laneWidth > 0.5)) return null
  const lw2 = v.laneWidth / 2
  const ex = (e: number) => CX + Math.sign(e) * distPix(Math.abs(e))
  const x1 = ex(-v.laneOffset - lw2), x2 = ex(-v.laneOffset + lw2)
  return (
    <g aria-hidden>
      <rect x={Math.min(x1, x2)} y={130} width={Math.abs(x2 - x1)} height={340} fill={C.fg} opacity={0.025} />
      <line className="vx-lane" x1={x1} y1={130} x2={x1} y2={470} stroke={C.mute} strokeOpacity={0.45} />
      <line className="vx-lane" x1={x2} y1={130} x2={x2} y2={470} stroke={C.mute} strokeOpacity={0.45} />
    </g>
  )
}

/** Downloaded top-view icon for an object kind. The car drawing is
 *  horizontal (nose right) so it gets a −90° base rotation. */
export function NearIcon({ kind, color }: { kind: Kind; color: string }) {
  const st = { color }
  if (kind === 'walker') return <use href="#vx-ic-walk" x={-4.7} y={-7.5} width={9.4} height={15} style={st} />
  if (kind === 'bike') return <use href="#vx-ic-bike" x={-4.4} y={-11} width={8.8} height={22} style={st} />
  if (kind === 'truck') return <use href="#vx-ic-truck" x={-7.7} y={-17} width={15.5} height={34} style={st} />
  return <g transform="rotate(-90)"><use href="#vx-ic-car" x={-15} y={-7.4} width={30} height={14.9} style={st} /></g>
}

function NearObjects({ v }: { v: VehData }) {
  const list = v.near.filter((x) => x.d <= 50)
  return (
    <g>
      {list.map((x, i) => {
        const p = placeNear(x)
        const right = p.x >= CX
        const ty = p.clamped ? 4 : x.kind === 'walker' ? 18 : 26
        return (
          <g key={i} className="vx-near" transform={`translate(${fmt(p.x, 1)},${fmt(p.y, 1)})`} opacity={fmt(p.opacity, 2)}>
            <g transform={`rotate(${fmt(x.kind === 'walker' ? 0 : x.yaw, 0)})`}><NearIcon kind={x.kind} color={KIND_C[x.kind]} /></g>
            <text className="vx-mono vx-halo-text" y={ty} x={p.clamped ? (right ? 16 : -16) : 0}
              textAnchor={p.clamped ? (right ? 'start' : 'end') : 'middle'} fontSize={13} strokeWidth={3} fill={C.fg}>
              {fmtDist(x.d)}
            </text>
          </g>
        )
      })}
    </g>
  )
}

/* ------------------------------------------------------------------- stage */
export interface StageProps {
  v: VehData
  st: VehState
  t: TFunc
  selSensor: number
  onSensor: (i: number) => void
  selHit: string | null
}

export default function Stage({ v, st, t, selSensor, onSensor, selHit }: StageProps) {
  // replay the shake whenever a new impact arrives while in the crash state
  const shake = useRef({ n: v.hits.length, key: 0 })
  if (v.hits.length !== shake.current.n) {
    if (st === 'crash') shake.current.key++
    shake.current.n = v.hits.length
  }
  const pts = spreadPts(v.sensors, v.bb)
  const sel = selSensor >= 0 && selSensor < v.sensors.length ? selSensor : -1
  const hit: Hit | undefined = selHit ? v.hits.find((h) => h.key === selHit) : undefined

  return (
    <svg viewBox="0 0 300 600" role="img" aria-label={t('vx_map_aria')} className="block h-auto w-full select-none">
      <NearSymbols />
      <Grid front={t('zone_front')} rear={t('zone_rear')} />
      <Lane v={v} />

      {/* car block drawn at 50 % so ring distances stay honest / 50% 縮小 */}
      <g transform={`translate(${CX},${CY}) scale(0.5) translate(${-CX},${-CY})`}>
        <Streaks speed={v.speed} />
        <HeroCar v={v} st={st} shakeKey={shake.current.key} />
        {/* attached sensors from the live CARLA actor list / 実取付センサー */}
        <g className={cn(sel >= 0 && 'vx-dim')}>
          {v.sensors.map((x, i) => {
            const m = snMeta(x.type), [px, py] = pts[i]
            const col = v.hero ? m.color : C.mute
            return (
              <g key={i} className={cn('vx-smark cursor-pointer', i === sel && 'sel')} transform={`translate(${fmt(px, 1)},${fmt(py, 1)})`}
                onClick={() => onSensor(i)}>
                <path className="vx-halo" d={shapePath(m.shape, 6.5)} stroke={col} />
                <path d={shapePath(m.shape, i === sel ? 9 : 6.5)} fill={col} stroke={C.bg} strokeWidth={1.5} />
              </g>
            )
          })}
        </g>
        {/* collision impact markers / 衝突位置 */}
        <g>
          {v.hits.map((h) => {
            const [x, y] = hitPos(h)
            return (
              <g key={h.key} className="vx-hit" opacity={h.key === selHit ? 1 : hitOpacity(h)}>
                <circle className="vx-hitring" cx={x} cy={y} r={7} stroke={C.danger} />
                <circle className="vx-hitring" cx={x} cy={y} r={7} stroke={C.danger} style={{ animationDelay: '.55s' }} />
                <circle cx={x} cy={y} r={5} fill={C.danger} stroke={C.s2} strokeWidth={1.5} style={{ filter: `drop-shadow(0 0 8px ${C.danger})` }} />
              </g>
            )
          })}
        </g>
      </g>

      <Pdc v={v} />
      <Chevrons signal={v.signal} />
      <Ultra d={v.ultra} />
      <NearObjects v={v} />

      {/* selected sensor: pulse ring + leader label (unscaled) / 選択センサー */}
      {sel >= 0 && pts[sel] && (() => {
        const [mx, my] = toStage(pts[sel])
        const lx = clamp(mx, 72, 228), ly = my < CY ? my - 17 : my + 25
        return (
          <g pointerEvents="none">
            <circle className="vx-locate" cx={mx} cy={my} r={8} stroke={C.fg} strokeOpacity={0.9} />
            <text className="vx-mono vx-halo-text" x={lx} y={ly} textAnchor="middle" fontSize={13} fontWeight={700} letterSpacing={0.6}
              fill={C.fg} strokeWidth={4}>
              {snName(t, v.sensors[sel], v.bb, sensDupKeys(v.sensors)).toUpperCase()}
            </text>
          </g>
        )
      })()}
      {/* selected impact (from the log) / 選択した衝突 */}
      {hit && (() => {
        const [mx, my] = toStage(hitPos(hit))
        const lx = clamp(mx, 80, 220), ly = my < CY ? my - 16 : my + 24
        return (
          <g pointerEvents="none">
            <circle cx={mx} cy={my} r={9} fill="none" stroke={C.fg} strokeWidth={1.5} strokeDasharray="3 2" />
            <text className="vx-mono vx-halo-text" x={lx} y={ly} textAnchor="middle" fontSize={13} fontWeight={700} fill={C.s2} strokeWidth={4}>
              {vzone(t, hit.f, hit.r)} · {hit.i}
            </text>
          </g>
        )
      })()}
    </svg>
  )
}
