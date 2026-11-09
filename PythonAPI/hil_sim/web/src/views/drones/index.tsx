// The Drones page: a drone dashboard in the style of a fleet console. Tabs for
// every drone; on the left the Drone Overview (Open CARLA view, Diagnostics),
// the Component View close-ups and the Damage report; in the centre the drone
// itself in real-time 3D (three.js, lazy-loaded: physical materials, rotors at
// the live RPM, X-ray cutaway, clickable parts, broken parts shown broken);
// on the right the flight data (battery gauge, altitude / speed, time left,
// distance home, location and flight path); then every sensor's card.
// Data: /delivery/state (drones[].sensors + damage from drone_sensors.py).
// "Open CARLA view" opens a native CARLA window (drone_window.py) through
// dashboard.py; no camera picture is shown in the browser.
// Deep link: #drones/<id>.
// ドローンページ：タブ・概要・部品ビュー・損傷・中央の 3D・飛行データ・全センサー
import { lazy, Suspense, useMemo, useState } from 'react'
import { Layers, RotateCcw } from 'lucide-react'
import '@/i18n/delivery'
import '@/i18n/drones'
import { useT } from '@/lib/i18n'
import { Badge, Button, EmptyState, Segmented, SectionTitle } from '@/components/ui'
import { useDeliveryState, DOWN, FLYING, IN_POD } from '@/views/delivery/shared'
import { SensorCards } from './SensorCards'
import { CameraSpecs } from './CameraSpecs'
import { cardDomId, type CardId } from './catalog'
import { useSensorHistory } from './history'
import { CARD_PART, partCard, partStates, type PartId } from './parts'
import { ComponentTiles, DamageReport, DroneTabs, Overview, droneLabel } from './left'
import { AltSpeed, FlightData, MiniMap, TimeHome } from './flight'
import { BrokenLabel, PartList, PartTooltip, StatusTiles } from './centre'
import { useCarlaView } from './carlaView'
import type { Pose } from './three/Drone3D'
import type { DroneS } from './types'

const Drone3D = lazy(() => import('./three/Drone3D'))

const reduced = () => window.matchMedia?.('(prefers-reduced-motion: reduce)').matches

const poseOf = (d: DroneS): Pose =>
  d.status === 'crashed' ? 'crashed' : d.status === 'falling' ? 'falling' : FLYING.includes(d.status) ? 'flying' : 'ground'

export default function DronesView({ section }: { section?: string }) {
  const t = useT()
  const st = useDeliveryState()
  const drones = (st?.available ? st.drones ?? [] : []) as DroneS[]
  const live = !!(st?.available && st.running)
  const hist = useSensorHistory(drones, st?.t)
  const want = section ? Number(section) : NaN
  const fallback = drones.find((d) => FLYING.includes(d.status))?.id ?? drones[0]?.id ?? null
  const selId = drones.some((d) => d.id === want) ? want : fallback
  const d = drones.find((x) => x.id === selId)
  const [part, setPart] = useState<PartId | null>(null)
  const [xray, setXray] = useState<'solid' | 'xray'>('solid')
  const [resetKey, setResetKey] = useState(0)
  const view = useCarlaView(selId)
  const states = useMemo(() => partStates(d?.sensors), [d?.sensors])
  const card: CardId | null = part ? partCard(part) ?? null : null

  const pick = (id: number) => { window.location.hash = `drones/${id}` }
  const scrollTo = (id: string, focus = false) => requestAnimationFrame(() => {
    const el = document.getElementById(id)
    el?.scrollIntoView({ behavior: reduced() ? 'auto' : 'smooth', block: focus ? 'center' : 'start' })
    if (focus) el?.focus({ preventScroll: true })
  })
  // a part picked (3D, tile, list, damage row): select it; its card and tile light up
  // 部品の選択：カードとタイルが強調される
  const select = (p: PartId) => setPart((cur) => (cur === p ? null : p))
  const toCard = () => { if (card) scrollTo(cardDomId(card), true) }
  // "Show on drone" on a sensor card: select its part, bring the 3D into view
  // カードの「図で表示」：部品を選び 3D へ
  const locate = (id: CardId) => { setPart(CARD_PART[id]); scrollTo('dn-3d') }

  const run = !st?.available ? t('dr_none') : st.running ? t('dr_live') : t('dr_stopped')
  const s = d?.sensors
  const rpm = s?.motors?.rpm
  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <div>
          <h1 className="text-xl font-semibold text-fg">{t('dn_title')}</h1>
          <p className="text-sm text-fg-mute">{t('dn_sub')}</p>
        </div>
        <Badge tone={live ? 'good' : 'bad'} dot><span className="text-fg-mute">{t('dr_chip_run')}</span> {run}</Badge>
      </div>

      {!st?.available || !drones.length || !d || !st ? (
        <EmptyState>
          <p className="font-medium text-fg-2">{!st?.available ? t('dn_none') : t('dn_nodrones')}</p>
          <p className="mt-1">{t('dn_none_help')}</p>
        </EmptyState>
      ) : (
        <>
          <DroneTabs drones={drones} sel={selId} onPick={pick} />
          {!s && <div role="status" className="rounded-lg border border-warn/40 bg-warn/10 px-3 py-2 text-sm text-warn">{t('dn_nosensors')}</div>}

          <div className="grid gap-4 [grid-template-areas:'c'_'l'_'r'] md:grid-cols-2 md:[grid-template-areas:'c_c'_'l_r'] xl:grid-cols-[300px_minmax(0,1fr)_340px] xl:[grid-template-areas:'l_c_r']">
            {/* left / 左 */}
            <div className="flex min-w-0 flex-col gap-4 [grid-area:l]">
              <Overview d={d} states={states} view={view} live={live} onDiag={() => scrollTo('dn-sensors')} />
              <ComponentTiles d={d} states={states} selected={part} onPick={select} />
            </div>

            {/* centre: the drone / 中央：3D */}
            <div className="flex min-w-0 flex-col gap-3 [grid-area:c]">
              <section id="dn-3d" aria-labelledby="dn-3d-h"
                className="relative h-[440px] scroll-mt-24 md:h-[540px] xl:h-[640px] overflow-hidden rounded-[var(--radius-card)] border border-line bg-[radial-gradient(ellipse_at_50%_55%,color-mix(in_srgb,var(--color-accent)_14%,transparent)_0%,color-mix(in_srgb,var(--color-info)_6%,transparent)_38%,transparent_70%)]">
                <h2 id="dn-3d-h" className="sr-only">{t('dn_3d_title', { drone: droneLabel(t, d.id) })}</h2>
                <Suspense fallback={<div className="absolute inset-0 grid place-items-center text-sm text-fg-mute">{t('dn_3d_loading')}</div>}>
                  <Drone3D states={states} rpm={rpm} spinning={FLYING.includes(d.status)} pose={poseOf(d)}
                    xray={xray === 'xray'} selected={part} onPick={select} resetKey={resetKey}
                    ariaLabel={t('dn_3d_aria', { drone: droneLabel(t, d.id) })}
                    tooltip={(p) => <PartTooltip t={t} part={p} st={states[p] ?? {}} d={d} />}
                    label={(p) => <BrokenLabel t={t} part={p} st={states[p] ?? {}} />} />
                </Suspense>
                {/* overlay controls / 操作 */}
                <div className="pointer-events-none absolute inset-x-0 top-0 flex items-start justify-between gap-2 p-3">
                  <div className="pointer-events-auto flex flex-col gap-1">
                    <span className="text-sm font-semibold text-fg">{droneLabel(t, d.id)}</span>
                    <span className="text-[11px] text-fg-mute">
                      {d.phase ? t(`dn_ph_${d.phase}`) : IN_POD.includes(d.status) ? t('dn_in_pod') : DOWN.includes(d.status) ? t('dn_down') : ''}
                    </span>
                  </div>
                  <div className="pointer-events-auto flex items-center gap-2">
                    <Segmented size="sm" label={t('dn_view_mode')} value={xray} onChange={setXray}
                      options={[{ value: 'solid', label: t('dn_solid') }, { value: 'xray', label: <span className="flex items-center gap-1"><Layers className="h-3 w-3" aria-hidden />{t('dn_xray')}</span> }]} />
                    <Button variant="ghost" size="sm" className="h-8 w-8 px-0" aria-label={t('dn_reset')} title={t('dn_reset')}
                      onClick={() => setResetKey((k) => k + 1)}>
                      <RotateCcw className="h-3.5 w-3.5" aria-hidden />
                    </Button>
                  </div>
                </div>
                <div className="pointer-events-none absolute inset-x-0 bottom-0 flex flex-wrap items-end justify-between gap-2 p-3">
                  <p className="text-[11px] text-fg-mute">{t('dn_3d_hint')}</p>
                  {part && (
                    <div className="pointer-events-auto flex items-center gap-2 rounded-lg border border-accent/50 bg-surface/90 px-2.5 py-1.5 text-xs" aria-live="polite">
                      <PartTooltip t={t} part={part} st={states[part] ?? {}} d={d} hint={false} />
                      {card && <Button size="sm" variant="secondary" onClick={toCard}>{t('dn_to_card')}</Button>}
                    </div>
                  )}
                </div>
              </section>
              <StatusTiles s={s} onPick={select} />
              <DamageReport d={d} onPick={select} />
              <PartList states={states} selected={part} onPick={select} />
            </div>

            {/* right / 右 */}
            <div className="flex min-w-0 flex-col gap-3 [grid-area:r]">
              <FlightData d={d} live={live} />
              <AltSpeed d={d} hist={(k) => hist(d.id, k)} />
              <TimeHome d={d} />
              <MiniMap d={d} st={st} />
            </div>
          </div>

          {s && (
            <section id="dn-sensors" aria-labelledby="dn-sens-h" className="scroll-mt-24">
              <div id="dn-sens-h"><SectionTitle sub={t('dn_sensors_sub')}>{t('dn_sensors')}</SectionTitle></div>
              <div className="grid grid-cols-1 gap-4 md:grid-cols-2 xl:grid-cols-3 2xl:grid-cols-4">
                <SensorCards d={d} s={s} hist={(k) => hist(d.id, k)} selected={card} onLocate={locate} />
                <CameraSpecs d={d} selected={card} onLocate={locate} onOpen={() => view.act('open')} live={live} />
              </div>
            </section>
          )}
        </>
      )}
    </div>
  )
}
