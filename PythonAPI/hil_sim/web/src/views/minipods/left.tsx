// The Mini Pods page's top and left column: the pod tabs, the Pod Overview
// card (thumbnail, key stats, Open CARLA view, Diagnostics), the Component
// View tiles (close-ups of the 3D model), the Hub card (the LMH hub's state,
// its three bays, distribution / meeting point, Send to charge) and the Damage
// report.
// タブ・概要カード・部品ビュー・ハブカード・損傷レポート
import { lazy, Suspense, useEffect, useRef, type ReactNode } from 'react'
import { BatteryMedium, CircleCheck, MonitorPlay, Stethoscope, Truck, X } from 'lucide-react'
import { useT, type TFunc } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Button, Card, KV } from '@/components/ui'
import { MP_COL, type DeliveryState, type ZoneSet } from '@/views/delivery/types'
import { lmhArea } from '@/views/delivery/zones'
import { MC, podColor } from '@/views/delivery/shared'
import { ChargeButton } from '@/views/delivery/ChargeButton'
import { HEALTH_MARK, IN_HUB, STATUS_TONE, TONE_DOT, TONE_TEXT, brokenCount, f, podLabel, sensorName, statusText } from './data'
import { LEVEL_TONE, partName, partReading, stateText, type PartId, type PartState } from './parts'
import type { CarlaView } from './carlaView'
import type { MiniPod } from './types'

const Thumb = lazy(() => import('./three/Pod3D').then((m) => ({ default: m.Thumb })))

/** Pod tabs: status dot + label, hub / bay, battery, ✕ when broken. / ポッドのタブ */
export function PodTabs({ pods, sel, onPick }: { pods: MiniPod[]; sel: number | null; onPick: (id: number) => void }) {
  const t = useT()
  const nav = useRef<HTMLElement>(null)
  useEffect(() => {
    nav.current?.querySelector('[aria-pressed="true"]')?.scrollIntoView({ block: 'nearest', inline: 'nearest' })
  }, [sel])
  return (
    <nav ref={nav} aria-label={t('mp_tabs_aria')} className="-mx-1 overflow-x-auto px-1 pb-1">
      <ul className="flex gap-2">
        {pods.map((p) => {
          const on = p.id === sel
          const broken = brokenCount(p) > 0
          return (
            <li key={p.id} className="shrink-0">
              <button type="button" aria-pressed={on} onClick={() => onPick(p.id)}
                className={cn('flex min-w-44 items-center gap-3 rounded-xl border px-3 py-2 text-left transition-colors duration-150',
                  on ? 'border-accent bg-accent text-accent-ink shadow-[0_8px_24px_-10px] shadow-accent/60'
                    : 'border-line bg-card/80 text-fg hover:bg-card-2')}>
                <span className="min-w-0">
                  <span className="flex items-center gap-1.5 text-sm font-semibold">
                    <i className="h-2 w-2 shrink-0 rounded-full ring-1 ring-black/30" aria-hidden
                      style={{ background: MP_COL[p.status] ?? MC.car }} />
                    {podLabel(t, p.id)}
                    {broken && <span className={cn('text-xs', on ? 'text-accent-ink' : 'text-danger')} title={t('mp_dmg_broken')}>✕<span className="sr-only"> {t('mp_dmg_broken')}</span></span>}
                  </span>
                  <span className={cn('block truncate text-[11px]', on ? 'text-accent-ink/80' : 'text-fg-mute')}>
                    {statusText(t, p.status)}
                    {p.hub != null && <> · <span className="num">{t('mp_hub_bay_short', { h: p.hub, b: p.bay ?? '–' })}</span></>}
                  </span>
                </span>
                <span className={cn('num ml-auto flex items-center gap-1 text-xs', on ? 'text-accent-ink' : 'text-fg-2')}>
                  <BatteryMedium className="h-3.5 w-3.5" aria-hidden />{p.battery == null ? '–' : `${Math.round(p.battery)}%`}
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

/** Pod Overview: id, model, status, thumbnail, key stats, the buttons. / 概要 */
export function Overview({ p, states, view, onDiag, live }: {
  p: MiniPod; states: Record<PartId, PartState>; live: boolean; view: CarlaView; onDiag: () => void
}) {
  const t = useT()
  return (
    <Card title={t('mp_overview')}>
      <div className="flex items-start justify-between gap-2">
        <div className="min-w-0">
          <div className="text-xs text-fg-mute">{podLabel(t, p.id)}{p.hub != null ? ` · ${t('mp_hub_n', { n: p.hub })}` : ''}</div>
          <div className="truncate text-lg font-semibold text-fg">{t('mp_model')}</div>
        </div>
        <Badge tone={STATUS_TONE[p.status] ?? 'neutral'} dot>{statusText(t, p.status)}</Badge>
      </div>
      <div className="relative h-36 overflow-hidden rounded-xl bg-[radial-gradient(closest-side,color-mix(in_srgb,var(--color-accent)_16%,transparent),transparent)]">
        <Suspense fallback={null}><Thumb focus="all" states={states} flag={IN_HUB.includes(p.status) ? 0 : 1} /></Suspense>
      </div>
      <div className="grid grid-cols-3 gap-2">
        <Mini v={p.trips ?? 0} label={t('mp_trips')} />
        <Mini v={f(p.km, 2)} unit="km" label={t('mp_distance')} />
        <Mini v={p.battery == null ? '–' : Math.round(p.battery)} unit="%" label={t('mp_batt')} />
      </div>
      <dl className="flex flex-col">
        <KV k={t('mp_hub')} v={p.hub == null ? '–' : t('mp_hub_bay', { h: p.hub, b: p.bay ?? '–' })} />
        <KV k={t('mp_order')} v={p.order == null ? t('mp_no_order') : `#${p.order}`} />
        {p.fail && <KV k={t('mp_fail')} v={p.fail} tone="bad" />}
      </dl>
      <Button variant="primary" className="w-full" busy={view.busy} disabled={!live}
        onClick={() => view.act('open')} aria-describedby="mp-view-help">
        <MonitorPlay className="h-4 w-4" aria-hidden />{view.open ? t('mp_view_focus') : t('mp_open_view')}
      </Button>
      {view.open && (
        <Button variant="secondary" size="sm" className="w-full" onClick={() => view.act('close')}>
          <X className="h-3.5 w-3.5" aria-hidden />{t('mp_view_close')}
        </Button>
      )}
      <Button variant="secondary" className="w-full" onClick={onDiag}>
        <Stethoscope className="h-4 w-4" aria-hidden />{t('mp_diag')}
      </Button>
      <p id="mp-view-help" className="text-[11px] leading-snug text-fg-mute" aria-live="polite">
        {view.error ? <span className="text-danger">{t('mp_view_err', { e: view.error })}</span>
          : view.open ? t('mp_view_is_open') : t('mp_view_help')}
      </p>
    </Card>
  )
}

const TILES: { part: PartId; key: string }[] = [
  { part: 'lidar', key: 'mp_comp_lidar' }, { part: 'cam_front', key: 'mp_comp_cam' },
  { part: 'wheel5', key: 'mp_comp_wheels' }, { part: 'lid', key: 'mp_comp_lid' },
]
const tileOn = (tile: PartId, sel: PartId | null) =>
  sel === tile || (tile === 'wheel5' && /^wheel\d$/.test(sel ?? '')) || (tile === 'lid' && ['lid_lock', 'payload', 'cam_lid', 'shutter'].includes(sel ?? ''))

/** Component View: a close-up render of four parts; click = select. / 部品ビュー */
export function ComponentTiles({ p, states, selected, onPick }: {
  p: MiniPod; states: Record<PartId, PartState>; selected: PartId | null; onPick: (part: PartId) => void
}) {
  const t = useT()
  // the wheel tile shows the worst wheel / 車輪タイルは最悪の車輪
  const wheelSt = (): PartState => {
    const all = [1, 2, 3, 4, 5, 6].map((n) => states[`wheel${n}`]).filter(Boolean)
    return all.find((s) => s.dmg === 'broken') ?? all.find((s) => s.level === 'fail') ?? all.find((s) => s.dmg || s.level === 'warn') ?? states.wheel5 ?? { sensors: [] }
  }
  return (
    <Card title={t('mp_components')}>
      <div className="grid grid-cols-2 gap-2">
        {TILES.map(({ part, key }) => {
          const st = part === 'wheel5' ? wheelSt() : states[part] ?? { sensors: [] }
          const tone = st.level ? LEVEL_TONE[st.level] : 'neutral'
          const on = tileOn(part, selected)
          const reading = partReading(t, part, p, st)
          return (
            <button key={part} type="button" aria-pressed={on} onClick={() => onPick(part)}
              aria-label={`${t(key)}: ${stateText(t, st)}. ${reading}`}
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
                <span className="num block truncate text-[10px] text-fg-mute">{reading || '–'}</span>
              </span>
            </button>
          )
        })}
      </div>
    </Card>
  )
}

/** Hub card: the LMH hub of this pod, its 3 bays (which pod is docked),
 *  its distribution and meeting point. / ハブカード */
export function HubCard({ p, st, zones, onPick }: { p: MiniPod; st: DeliveryState; zones: ZoneSet; onPick: (id: number) => void }) {
  const t = useT()
  const hub = st.vans?.find((v) => v.id === p.hub)
  const pods = (st.minipods ?? []) as MiniPod[]
  const mine = pods.filter((q) => q.hub === p.hub)
  const nBays = Math.max(3, ...mine.map((q) => q.bay ?? 0))
  const xy = (v: [number, number] | null | undefined) => (v ? `(${f(v[0], 1)}, ${f(v[1], 1)})` : '–')
  const docked = hub ? (Array.isArray(hub.pods_docked) ? hub.pods_docked.length : hub.pods_docked) : undefined
  return (
    <Card title={<span className="flex items-center gap-1.5"><Truck className="h-3.5 w-3.5 text-accent" aria-hidden />{t('mp_hub_card')}</span>}
      actions={hub ? <Badge>{hub.state}</Badge> : undefined}>
      {p.hub == null || !hub ? <p className="text-sm text-fg-mute">{t('mp_hub_none')}</p> : (
        <>
          <div className="flex items-center gap-2">
            <i aria-hidden className="h-3 w-3 shrink-0 rounded-[3px] border" style={{ background: `${podColor(hub.id)}55`, borderColor: podColor(hub.id) }} />
            <span className="text-sm font-semibold text-fg">{t('mp_hub_n', { n: hub.id })}</span>
            <span className="text-xs text-fg-mute">{t('mp_hub_model')}</span>
          </div>
          <dl className="flex flex-col">
            {hub.phase && <KV k={t('mp_hub_phase')} v={hub.phase.replace(/_/g, ' ')} />}
            <KV k={t('mp_batt')} v={`${Math.round(hub.battery)} %`} />
            <KV k={t('mp_hub_docked')} v={docked == null ? '–' : `${docked} / ${nBays}`} />
            <KV k={t('mp_hub_area')} v={t(`mp_area_${lmhArea(zones, hub.id, hub.area)}`)} />
            <KV k={t('mp_hub_dist')} v={xy(hub.dist)} />
            <KV k={t('mp_hub_meet')} v={xy(hub.meet)} />
          </dl>
          <ul className="grid grid-cols-3 gap-1.5" aria-label={t('mp_bays')}>
            {Array.from({ length: nBays }, (_, i) => i + 1).map((b) => {
              const q = mine.find((x) => x.bay === b)
              const inBay = !!q && IN_HUB.includes(q.status)
              return (
                <li key={b}>
                  <button type="button" disabled={!q} onClick={() => q && onPick(q.id)}
                    aria-label={`${t('mp_bay_n', { n: b })}: ${q ? `${podLabel(t, q.id)}, ${inBay ? t('mp_bay_in') : t('mp_bay_out')}` : t('mp_bay_empty')}`}
                    className={cn('flex w-full flex-col items-center gap-0.5 rounded-lg border px-1 py-1.5 text-[11px] transition-colors',
                      q?.id === p.id ? 'border-accent bg-accent/10' : 'border-line bg-card-2 hover:border-line-strong',
                      'disabled:cursor-default disabled:hover:border-line')}>
                    <span className="text-fg-mute">{t('mp_bay_n', { n: b })}</span>
                    <span className="num font-semibold text-fg">{q ? `M${q.id}` : '–'}</span>
                    <span className={cn('flex items-center gap-1', inBay ? 'text-info' : q ? 'text-accent' : 'text-fg-mute')}>
                      <i className={cn('h-1.5 w-1.5 rounded-full', inBay ? 'bg-info' : q ? 'bg-accent' : 'bg-fg-mute')} aria-hidden />
                      {q ? (inBay ? t('mp_bay_in') : t('mp_bay_out')) : t('mp_bay_empty')}
                    </span>
                  </button>
                </li>
              )
            })}
          </ul>
          <ChargeButton van={hub} state={st} size="md" full />
          <p className="text-[11px] leading-snug text-fg-mute">{t(`mp_area_help_${lmhArea(zones, hub.id, hub.area)}`)}</p>
        </>
      )}
    </Card>
  )
}

/** What the damage looks like ("wheel missing", "lid torn"...), or nothing. */
function dmgWhat(t: TFunc, part: PartId, lv: string | undefined): string {
  const k = `mp_dmg_what_${lv === 'broken' ? 'b' : 'd'}_${/^wheel\d$/.test(part) ? 'wheel' : part}`
  const s = t(k)
  return s === k ? '' : s
}

/** Damage report: what is broken and why, plus sensor faults. / 損傷レポート */
export function DamageReport({ p, states, onPick }: { p: MiniPod; states: Record<PartId, PartState>; onPick: (part: PartId) => void }) {
  const t = useT()
  const rows = Object.entries(states).filter(([, s]) => s.dmg)
    .sort((a, b) => (a[1].dmg === b[1].dmg ? a[0].localeCompare(b[0]) : a[1].dmg === 'broken' ? -1 : 1))
  const faults = Object.entries(p.sensors ?? {}).filter(([, s]) => s?.health === 'fail')
  const whole = !rows.length && !faults.length
  return (
    <Card title={t('mp_damage')} actions={whole ? <Badge tone="good"><CircleCheck className="h-3 w-3" aria-hidden />{t('mp_intact')}</Badge>
      : <Badge tone={rows.some(([, s]) => s.dmg === 'broken') ? 'bad' : 'warn'}>{rows.length + faults.length}</Badge>}>
      {whole ? <p className="text-sm text-fg-mute">{p.sensors ? t('mp_dmg_none') : t('mp_nosensors_short')}</p> : (
        <>
          {p.damage?.why && (
            <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5 text-xs">
              <dt className="text-fg-mute">{t('mp_cause')}</dt>
              <dd className="text-fg">{(() => { const k = `mp_why_${p.damage.why}`, s = t(k); return s === k ? p.damage.why.replace(/_/g, ' ') : s })()}</dd>
            </dl>
          )}
          <ul className="flex flex-col gap-1" aria-label={t('mp_damage')}>
            {rows.map(([part, s]) => (
              <li key={part}>
                <button type="button" onClick={() => onPick(part)}
                  className="flex w-full items-center gap-2 rounded-lg border border-line bg-card-2 px-2 py-1.5 text-left text-xs hover:border-line-strong">
                  <span className={cn('grid h-5 w-5 shrink-0 place-items-center rounded-full text-[11px] font-bold text-bg',
                    s.dmg === 'broken' ? 'bg-danger' : 'bg-warn')} aria-hidden>{s.dmg === 'broken' ? '✕' : '!'}</span>
                  <span className="min-w-0 flex-1">
                    <span className="block truncate font-medium text-fg">{partName(t, part)}</span>
                    <span className="block truncate text-fg-mute">{dmgWhat(t, part, s.dmg)}</span>
                  </span>
                  <Badge tone={s.dmg === 'broken' ? 'bad' : 'warn'}>{t(`mp_dmg_${s.dmg}`)}</Badge>
                </button>
              </li>
            ))}
            {faults.map(([k, s]) => (
              <li key={k} className="flex items-center gap-2 rounded-lg border border-danger/40 bg-danger/5 px-2 py-1.5 text-xs">
                <span className="grid h-5 w-5 shrink-0 place-items-center rounded-full bg-danger text-[11px] font-bold text-bg" aria-hidden>{HEALTH_MARK.fail}</span>
                <span className="min-w-0 flex-1 truncate text-fg">{sensorName(t, k, s)}</span>
                <Badge tone="bad">{t('mp_h_fail')}</Badge>
              </li>
            ))}
          </ul>
          <p className="text-[11px] text-fg-mute">{t('mp_dmg_help')}</p>
        </>
      )}
    </Card>
  )
}
