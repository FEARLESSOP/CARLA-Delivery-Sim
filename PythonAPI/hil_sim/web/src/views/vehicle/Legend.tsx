import type { ReactNode } from 'react'
import { C } from '@/lib/colors'
import { useT } from '@/lib/i18n'
import type { Kind, Sensor } from './data'
import { KIND_C, PDC_COL, PDC_THR, shapePath, snMeta } from './logic'
import { NearIcon } from './Stage'

// Legend of the vehicle visual: every mark on the map with its meaning, so
// colour is never the only cue (shapes, positions and text back it up).
// 図の凡例（色だけに頼らず形・位置・文字で区別）

/** A small sensor marker (same shape and colour as on the map). */
export function SensorGlyph({ type, dim, size = 14 }: { type: string; dim?: boolean; size?: number }) {
  const m = snMeta(type)
  return (
    <svg viewBox="-10 -10 20 20" width={size} height={size} aria-hidden className="shrink-0">
      <path d={shapePath(m.shape, 7)} fill={dim ? C.mute : m.color} stroke={C.bg} strokeWidth={1.2} />
    </svg>
  )
}

/** Small top-view icon of a nearby object kind. */
export function KindGlyph({ kind, size = 20 }: { kind: Kind; size?: number }) {
  return (
    <svg viewBox="-18 -18 36 36" width={size} height={size} aria-hidden className="shrink-0">
      <NearIcon kind={kind} color={KIND_C[kind]} />
    </svg>
  )
}

function Row({ icon, children }: { icon: ReactNode; children: ReactNode }) {
  return (
    <li className="flex items-center gap-2.5 py-1 text-sm text-fg-2">
      <span className="grid w-9 shrink-0 place-items-center">{icon}</span>
      <span className="min-w-0">{children}</span>
    </li>
  )
}

const KINDS: Kind[] = ['car', 'truck', 'bike', 'walker']
const PDC_TXT = ['text-danger', 'text-warn', 'text-accent']

export default function Legend({ sensors }: { sensors: Sensor[] }) {
  const t = useT()
  // one entry per sensor family that is actually mounted
  const fams = Array.from(new Map(sensors.map((s) => [snMeta(s.type).key, s.type])).entries())
  return (
    <div className="min-w-0">
      <h4 className="mb-1 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('vx_legend')}</h4>
      <ul>
        <Row icon={
          <svg viewBox="0 0 36 20" width={36} height={20} aria-hidden>
            <circle cx={18} cy={10} r={8} fill="none" stroke={C.danger} strokeWidth={1.5} opacity={0.5} />
            <circle cx={18} cy={10} r={4.5} fill={C.danger} stroke={C.s2} strokeWidth={1.2} />
          </svg>}>{t('vx_lg_impact')}</Row>
        <Row icon={
          <svg viewBox="0 0 36 22" width={36} height={22} aria-hidden>
            {[0, 1, 2].map((i) => (
              <path key={i} d={`M${12 - i * 5},${18 - i * 4} A${8 + i * 6},${8 + i * 6} 0 0 1 ${24 + i * 5},${18 - i * 4}`}
                fill="none" stroke={PDC_COL[i]} strokeWidth={2.5} strokeLinecap="round" />
            ))}
          </svg>}>
          <span className="block">{t('vx_lg_pdc')}</span>
          <span className="num flex flex-wrap gap-x-2 text-xs">
            {PDC_THR.map((th, i) => <span key={th} className={PDC_TXT[i]}>≤ {th.toFixed(1)} m</span>)}
          </span>
        </Row>
        <Row icon={
          <svg viewBox="0 0 36 16" width={36} height={16} aria-hidden>
            <path d="M6,13 A26,26 0 0 1 30,13" fill="none" stroke={C.accent} strokeWidth={2.5} strokeLinecap="round" />
          </svg>}>
          <span className="block">{t('vx_lg_ultra')}</span>
          <span className="num flex flex-wrap gap-x-2 text-xs">
            <span className="text-danger">&lt; 3 m</span><span className="text-warn">&lt; 8 m</span>
            <span className="text-accent">≥ 8 m</span>
          </span>
        </Row>
        <Row icon={
          <svg viewBox="0 0 36 20" width={36} height={20} aria-hidden>
            <path d="M14,3 L6,10 L14,17 M24,3 L16,10 L24,17" fill="none" stroke={C.accent} strokeWidth={2.5} strokeLinecap="round" strokeLinejoin="round" />
          </svg>}>{t('vx_lg_signal')}</Row>
        <Row icon={
          <svg viewBox="0 0 36 20" width={36} height={20} aria-hidden>
            <line x1={10} y1={1} x2={10} y2={19} stroke={C.mute} strokeWidth={2} strokeDasharray="4 3" />
            <line x1={26} y1={1} x2={26} y2={19} stroke={C.mute} strokeWidth={2} strokeDasharray="4 3" />
          </svg>}>{t('vx_lg_lane')}</Row>
        <Row icon={
          <svg viewBox="0 0 36 20" width={36} height={20} aria-hidden>
            <circle cx={18} cy={20} r={8} fill="none" stroke={C.info} strokeOpacity={0.6} />
            <circle cx={18} cy={20} r={16} fill="none" stroke={C.info} strokeOpacity={0.6} strokeDasharray="2 3" />
          </svg>}>{t('vx_lg_rings')}</Row>
      </ul>
      <h5 className="mb-0.5 mt-2 text-xs text-fg-mute">{t('vx_lg_objects')}</h5>
      <ul className="grid grid-cols-2 gap-x-2">
        {KINDS.map((k) => <Row key={k} icon={<KindGlyph kind={k} />}>{t(`kind_${k}`)}</Row>)}
      </ul>
      {fams.length > 0 && <>
        <h5 className="mb-0.5 mt-2 text-xs text-fg-mute">{t('vx_lg_sensor')}</h5>
        <ul className="grid grid-cols-1 gap-x-2 sm:grid-cols-2 @xl:grid-cols-1">
          {fams.map(([k, type]) => <Row key={k} icon={<SensorGlyph type={type} />}>{t(k)}</Row>)}
        </ul>
      </>}
    </div>
  )
}
