// The Drones page's top and left column: the drone tabs, the Drone Overview
// card (thumbnail, key stats, Open CARLA view, Diagnostics), the Component
// View tiles (close-ups of the 3D model) and the Damage report.
// タブ・概要カード・部品ビュー・損傷レポート
import { lazy, Suspense, useEffect, useRef, type ReactNode } from 'react'
import { BatteryMedium, CircleCheck, MonitorPlay, Stethoscope, X } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Button, Card } from '@/components/ui'
import { DRONE_COL, MC, droneSt, failReason } from '@/views/delivery/shared'
import { DroneBadge } from '@/views/delivery/widgets'
import { HEALTH_MARK, cardHealth, f, SENSOR_IDS } from './catalog'
import { LEVEL_TONE, partName, partReading, stateText, type PartId, type PartState } from './parts'
import type { DroneS } from './types'

const Thumb = lazy(() => import('./three/Drone3D').then((m) => ({ default: m.Thumb })))

const TONE_TEXT = { good: 'text-accent', warn: 'text-warn', bad: 'text-danger', neutral: 'text-fg-mute', info: 'text-info', accent: 'text-accent' } as const
const TONE_DOT = { good: 'bg-accent', warn: 'bg-warn', bad: 'bg-danger', neutral: 'bg-fg-mute', info: 'bg-info', accent: 'bg-accent' } as const
export { TONE_DOT, TONE_TEXT }

const pad2 = (n: number) => String(n).padStart(2, '0')
export const droneLabel = (t: (k: string, v?: Record<string, string | number>) => string, id: number) =>
  t('dn_drone_n', { n: pad2(id) })

/** Drone tabs: status dot + label, battery %, the selected one filled. / 機体タブ */
export function DroneTabs({ drones, sel, onPick }: { drones: DroneS[]; sel: number | null; onPick: (id: number) => void }) {
  const t = useT()
  const nav = useRef<HTMLElement>(null)
  // keep the selected tab in view (many drones, narrow screens) / 選択中のタブを表示
  useEffect(() => {
    nav.current?.querySelector('[aria-pressed="true"]')?.scrollIntoView({ block: 'nearest', inline: 'nearest' })
  }, [sel])
  return (
    <nav ref={nav} aria-label={t('dn_tabs_aria')} className="-mx-1 overflow-x-auto px-1 pb-1">
      <ul className="flex gap-2">
        {drones.map((d) => {
          const on = d.id === sel
          const broken = (d.sensors?.damage?.broken?.length ?? 0) > 0
          return (
            <li key={d.id} className="shrink-0">
              <button type="button" aria-pressed={on} onClick={() => onPick(d.id)}
                className={cn('flex min-w-40 items-center gap-3 rounded-xl border px-3 py-2 text-left transition-colors duration-150',
                  on ? 'border-accent bg-accent text-accent-ink shadow-[0_8px_24px_-10px] shadow-accent/60'
                    : 'border-line bg-card/80 text-fg hover:bg-card-2')}>
                <span className="min-w-0">
                  <span className="flex items-center gap-1.5 text-sm font-semibold">
                    <i className="h-2 w-2 shrink-0 rounded-full ring-1 ring-black/30" aria-hidden
                      style={{ background: DRONE_COL[d.status] ?? MC.car }} />
                    {droneLabel(t, d.id)}
                    {broken && <span className={cn('text-xs', on ? 'text-accent-ink' : 'text-danger')} title={t('dn_dmg_broken')}>✕</span>}
                  </span>
                  <span className={cn('block truncate text-[11px]', on ? 'text-accent-ink/80' : 'text-fg-mute')}>
                    {droneSt(t, d.status)}
                  </span>
                </span>
                <span className={cn('num ml-auto flex items-center gap-1 text-xs', on ? 'text-accent-ink' : 'text-fg-2')}>
                  <BatteryMedium className="h-3.5 w-3.5" aria-hidden />{Math.round(d.battery)}%
                </span>
              </button>
            </li>
          )
        })}
      </ul>
    </nav>
  )
}

function Mini({ v, unit, label }: { v: ReactNode; unit?: string; label: string }) {
  return (
    <div className="min-w-0">
      <div className="num text-xl font-semibold text-fg">{v}{unit && <span className="ml-1 text-xs font-normal text-fg-mute">{unit}</span>}</div>
      <div className="text-[11px] text-fg-mute">{label}</div>
    </div>
  )
}

/** Drone Overview: id, model, status, thumbnail, key stats, the two buttons. / 概要 */
export function Overview({ d, states, view, onDiag, live }: {
  d: DroneS; states: Record<PartId, PartState>; live: boolean
  view: { open: boolean; busy: boolean; error: string | null; act: (a: 'open' | 'close') => void }
  onDiag: () => void
}) {
  const t = useT()
  const air = d.air_s
  return (
    <Card title={t('dn_overview')}>
      <div className="flex items-start justify-between gap-2">
        <div className="min-w-0">
          <div className="text-xs text-fg-mute">{droneLabel(t, d.id)}{d.sensors?.remote_id?.serial ? ` · ${d.sensors.remote_id.serial}` : ''}</div>
          <div className="truncate text-lg font-semibold text-fg">{t('dn_model')}</div>
        </div>
        <DroneBadge t={t} status={d.status} />
      </div>
      <div className="relative h-36 overflow-hidden rounded-xl bg-[radial-gradient(closest-side,color-mix(in_srgb,var(--color-accent)_16%,transparent),transparent)]">
        <Suspense fallback={null}><Thumb focus="all" states={states} /></Suspense>
      </div>
      <div className="grid grid-cols-3 gap-2">
        <Mini v={air == null ? '–' : Math.round(air / 60)} unit={t('dn_min')} label={t('dn_flight_time')} />
        <Mini v={f(d.km, 1)} unit="km" label={t('dn_distance')} />
        <Mini v={Math.round(d.battery)} unit="%" label={t('dn_batt')} />
      </div>
      <Button variant="primary" className="w-full" busy={view.busy} disabled={!live}
        onClick={() => view.act('open')} aria-describedby="dn-view-help">
        <MonitorPlay className="h-4 w-4" aria-hidden />{view.open ? t('dn_view_focus') : t('dn_open_view')}
      </Button>
      {view.open && (
        <Button variant="secondary" size="sm" className="w-full" onClick={() => view.act('close')}>
          <X className="h-3.5 w-3.5" aria-hidden />{t('dn_view_close')}
        </Button>
      )}
      <Button variant="secondary" className="w-full" onClick={onDiag}>
        <Stethoscope className="h-4 w-4" aria-hidden />{t('dn_diag')}
      </Button>
      <p id="dn-view-help" className="text-[11px] leading-snug text-fg-mute" aria-live="polite">
        {view.error ? <span className="text-danger">{t('dn_view_err', { e: view.error })}</span>
          : view.open ? t('dn_view_is_open') : t('dn_view_help')}
      </p>
    </Card>
  )
}

const TILES: { part: PartId; key: string }[] = [
  { part: 'cam_fpv', key: 'dn_comp_fpv' }, { part: 'motor1', key: 'dn_comp_motor' },
  { part: 'battery', key: 'dn_comp_battery' }, { part: 'winch', key: 'dn_comp_winch' },
]

/** Component View: a close-up render of four parts; click = select. / 部品ビュー */
export function ComponentTiles({ d, states, selected, onPick }: {
  d: DroneS; states: Record<PartId, PartState>; selected: PartId | null; onPick: (p: PartId) => void
}) {
  const t = useT()
  return (
    <Card title={t('dn_components')}>
      <div className="grid grid-cols-2 gap-2">
        {TILES.map(({ part, key }) => {
          const st = states[part] ?? {}
          const tone = st.level ? LEVEL_TONE[st.level] : 'neutral'
          const on = selected === part || (part === 'motor1' && /^(motor|rotor|esc|arm)\d$/.test(selected ?? ''))
          return (
            <button key={part} type="button" id={`dn-tile-${part}`} aria-pressed={on} onClick={() => onPick(part)}
              aria-label={`${t(key)}: ${stateText(t, st)}. ${partReading(t, part, d)}`}
              className={cn('group flex min-w-0 flex-col overflow-hidden rounded-xl border bg-card-2 text-left transition-colors duration-150',
                on ? 'border-accent ring-2 ring-accent/40' : 'border-line hover:border-line-strong')}>
              <span className="relative block h-20 bg-[radial-gradient(closest-side,color-mix(in_srgb,var(--color-fg)_7%,transparent),transparent)]">
                <Suspense fallback={null}><Thumb focus={part} states={states} /></Suspense>
                <span className={cn('absolute right-1.5 top-1.5 flex items-center gap-1 rounded-full bg-bg/70 px-1.5 text-[10px]', TONE_TEXT[tone])}>
                  <i className={cn('h-1.5 w-1.5 rounded-full', TONE_DOT[tone])} aria-hidden />{stateText(t, st)}
                </span>
              </span>
              <span className="px-2 py-1.5">
                <span className="block truncate text-xs font-medium text-fg">{t(key)}</span>
                <span className="num block truncate text-[10px] text-fg-mute">{partReading(t, part, d) || '–'}</span>
              </span>
            </button>
          )
        })}
      </div>
    </Card>
  )
}

/** Damage report: what is broken and why, plus sensor faults. / 損傷レポート */
export function DamageReport({ d, onPick }: { d: DroneS; onPick: (p: PartId) => void }) {
  const t = useT()
  const s = d.sensors
  const dm = s?.damage
  const rows = Object.entries(dm?.parts ?? {}).sort((a, b) => (a[1].level === b[1].level ? a[0].localeCompare(b[0]) : a[1].level === 'broken' ? -1 : 1))
  const faults = SENSOR_IDS.filter((id) => cardHealth(s, id) === 'fail')
  const whole = !rows.length && !faults.length
  return (
    <Card title={t('dn_damage')} actions={whole ? <Badge tone="good"><CircleCheck className="h-3 w-3" aria-hidden />{t('dn_intact')}</Badge>
      : <Badge tone={dm?.broken?.length ? 'bad' : 'warn'}>{rows.length + faults.length}</Badge>}>
      {whole ? <p className="text-sm text-fg-mute">{s ? t('dn_dmg_none') : t('dn_nosensors_short')}</p> : (
        <>
          {dm?.cause && (
            <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5 text-xs">
              <dt className="text-fg-mute">{t('dn_cause')}</dt><dd className="text-fg">{t(`dn_cause_${dm.cause}`) === `dn_cause_${dm.cause}` ? failReason(t, dm.cause) : t(`dn_cause_${dm.cause}`)}</dd>
              {dm.impact && <><dt className="text-fg-mute">{t('dn_impact')}</dt><dd className="text-fg">{t(`dn_imp_${dm.impact}`)}</dd></>}
              {(dm.fall_h ?? 0) > 0 && <><dt className="text-fg-mute">{t('dn_fall')}</dt><dd className="num text-fg">{f(dm.fall_h, 1)} m</dd></>}
              {dm.detail && <><dt className="text-fg-mute">{t('dn_detail')}</dt><dd className="truncate text-fg-2" title={dm.detail}>{dm.detail}</dd></>}
            </dl>
          )}
          <ul className="flex flex-col gap-1" aria-label={t('dn_damage')}>
            {rows.map(([part, p]) => (
              <li key={part}>
                <button type="button" onClick={() => onPick(part)}
                  className="flex w-full items-center gap-2 rounded-lg border border-line bg-card-2 px-2 py-1.5 text-left text-xs hover:border-line-strong">
                  <span className={cn('grid h-5 w-5 shrink-0 place-items-center rounded-full text-[11px] font-bold text-bg',
                    p.level === 'broken' ? 'bg-danger' : 'bg-warn')} aria-hidden>{p.level === 'broken' ? '✕' : '!'}</span>
                  <span className="min-w-0 flex-1">
                    <span className="block truncate font-medium text-fg">{partName(t, part)}</span>
                    <span className="block truncate text-fg-mute">{t(`dn_why_${p.why}`)}</span>
                  </span>
                  <Badge tone={p.level === 'broken' ? 'bad' : 'warn'}>{t(`dn_dmg_${p.level}`)}</Badge>
                </button>
              </li>
            ))}
            {faults.map((id) => (
              <li key={id} className="flex items-center gap-2 rounded-lg border border-danger/40 bg-danger/5 px-2 py-1.5 text-xs">
                <span className="grid h-5 w-5 shrink-0 place-items-center rounded-full bg-danger text-[11px] font-bold text-bg" aria-hidden>{HEALTH_MARK.fail}</span>
                <span className="min-w-0 flex-1 truncate text-fg">{t(`dn_s_${id}`)}</span>
                <Badge tone="bad">{t('dn_h_fail')}</Badge>
              </li>
            ))}
          </ul>
          <p className="text-[11px] text-fg-mute">{t('dn_dmg_help')}</p>
        </>
      )}
    </Card>
  )
}

