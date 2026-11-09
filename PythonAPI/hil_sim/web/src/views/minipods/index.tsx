// The Mini Pods page: the LastMile Mini Delivery Pods (the six-wheeled robots
// a LastMile Hub carries in three curb-side bays) in the style of the Drones
// page. Tabs for every pod; on the left the Pod Overview (Open CARLA view,
// Diagnostics), the Component View close-ups and the Hub card; in the centre
// the pod itself in real-time 3D (three.js, lazy-loaded: physical materials,
// wheels at the live speed, lid / shutter / flag from the state, X-ray,
// clickable parts, broken parts shown broken), the status tiles, the Damage
// report; on the right the battery gauge, speed and distance to go, ETA and
// the mini map; then every sensor's card, grouped.
// Data: /delivery/state (minipods[] and vans[] with kind "lmh"). "Open CARLA
// view" opens a native CARLA window (minipod_window.py) through dashboard.py.
// Deep link: #minipods/<id>.
// ミニポッドページ：タブ・概要・部品ビュー・ハブ・中央の 3D・走行データ・全センサー
import { lazy, Suspense, useMemo, useState } from 'react'
import { Layers, RotateCcw } from 'lucide-react'
import '@/i18n/delivery'
import '@/i18n/minipods'
import { useT } from '@/lib/i18n'
import { Badge, Button, EmptyState, Segmented, SectionTitle } from '@/components/ui'
import { useDeliveryState } from '@/views/delivery/shared'
import { mapKey, readZones, useZones } from '@/views/delivery/zones'
import { DOWN, IN_HUB, OUT, lmhVans, openFrac, podLabel, statusText } from './data'
import { usePodHistory } from './history'
import { partStates, type PartId } from './parts'
import { ComponentTiles, DamageReport, HubCard, Overview, PodTabs } from './left'
import { BatteryCard, EtaTrips, MiniMap, SpeedDist } from './right'
import { BrokenLabel, PartList, PartTooltip, StatusTiles } from './centre'
import { SensorCards, cardDomId } from './SensorCards'
import { useCarlaView } from './carlaView'
import type { Pose } from './three/Pod3D'
import type { MiniPod } from './types'

const Pod3D = lazy(() => import('./three/Pod3D'))

const reduced = () => window.matchMedia?.('(prefers-reduced-motion: reduce)').matches

const poseOf = (p: MiniPod): Pose => (p.status === 'crashed' ? 'crashed' : p.status === 'stuck' ? 'stuck' : 'ground')

export default function MiniPodsView({ section }: { section?: string }) {
  const t = useT()
  const st = useDeliveryState()
  const published = !!st?.available && Array.isArray(st.minipods)
  const pods = useMemo(() => (published ? (st?.minipods ?? []) as MiniPod[] : []), [published, st?.minipods])
  const live = !!(st?.available && st.running)
  const hist = usePodHistory(pods, st?.t)
  const want = section ? Number(section) : NaN
  const fallback = pods.find((p) => OUT.includes(p.status))?.id ?? pods[0]?.id ?? null
  const selId = pods.some((p) => p.id === want) ? want : fallback
  const p = pods.find((x) => x.id === selId)
  const [part, setPart] = useState<PartId | null>(null)
  const [xray, setXray] = useState<'solid' | 'xray'>('solid')
  const [resetKey, setResetKey] = useState(0)
  const view = useCarlaView(selId)
  const states = useMemo(() => partStates(p), [p])
  // distribution / meeting points of the hubs (the editor's zones) / ハブの地点
  const z = useZones(mapKey(st?.map))
  const zones = useMemo(() => {
    const fromState = readZones(st?.zones)
    return Object.keys(fromState.dist).length || Object.keys(fromState.meet).length ? fromState : z.zones
  }, [st?.zones, z.zones])

  const pick = (id: number) => { window.location.hash = `minipods/${id}` }
  const scrollTo = (id: string, focus = false) => requestAnimationFrame(() => {
    const el = document.getElementById(id)
    el?.scrollIntoView({ behavior: reduced() ? 'auto' : 'smooth', block: focus ? 'center' : 'start' })
    if (focus) el?.focus({ preventScroll: true })
  })
  const select = (q: PartId) => setPart((cur) => (cur === q ? null : q))
  // the first sensor card of the selected part / 選択部品の最初のカード
  const cardKey = part ? states[part]?.sensors[0] : undefined
  const toCard = () => { if (cardKey) scrollTo(cardDomId(cardKey), true) }
  const locate = (q: PartId) => { setPart(q); scrollTo('mp-3d') }

  const run = !st?.available ? t('dr_none') : st.running ? t('dr_live') : t('dr_stopped')
  const hubs = lmhVans(st)
  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <div>
          <h1 className="text-xl font-semibold text-fg">{t('mp_title')}</h1>
          <p className="text-sm text-fg-mute">{t('mp_sub')}</p>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          {published && <Badge><span className="text-fg-mute">{t('mp_chip_hubs')}</span> <span className="num">{hubs.length}</span></Badge>}
          {published && <Badge><span className="text-fg-mute">{t('mp_chip_pods')}</span> <span className="num">{pods.length}</span></Badge>}
          <Badge tone={live ? 'good' : 'bad'} dot><span className="text-fg-mute">{t('dr_chip_run')}</span> {run}</Badge>
        </div>
      </div>

      {st?.available && !published && (
        <div role="status" className="rounded-lg border border-warn/40 bg-warn/10 px-3 py-2 text-sm text-warn">{t('mp_restart')}</div>
      )}

      {!st?.available || !pods.length || !p || !st ? (
        <EmptyState>
          <p className="font-medium text-fg-2">{!st?.available ? t('mp_none') : !published ? t('mp_not_published') : t('mp_nopods')}</p>
          <p className="mt-1">{t('mp_none_help')}</p>
        </EmptyState>
      ) : (
        <>
          <PodTabs pods={pods} sel={selId} onPick={pick} />
          {!p.sensors && <div role="status" className="rounded-lg border border-warn/40 bg-warn/10 px-3 py-2 text-sm text-warn">{t('mp_nosensors')}</div>}

          <div className="grid gap-4 [grid-template-areas:'c'_'l'_'r'] md:grid-cols-2 md:[grid-template-areas:'c_c'_'l_r'] xl:grid-cols-[300px_minmax(0,1fr)_340px] xl:[grid-template-areas:'l_c_r']">
            {/* left / 左 */}
            <div className="flex min-w-0 flex-col gap-4 [grid-area:l]">
              <Overview p={p} states={states} view={view} live={live} onDiag={() => scrollTo('mp-sensors')} />
              <ComponentTiles p={p} states={states} selected={part} onPick={select} />
              <HubCard p={p} st={st} zones={zones} onPick={pick} />
            </div>

            {/* centre: the pod / 中央：3D */}
            <div className="flex min-w-0 flex-col gap-3 [grid-area:c]">
              <section id="mp-3d" aria-labelledby="mp-3d-h"
                className="relative h-[440px] scroll-mt-24 overflow-hidden rounded-[var(--radius-card)] border border-line bg-[radial-gradient(ellipse_at_50%_55%,color-mix(in_srgb,var(--color-accent)_14%,transparent)_0%,color-mix(in_srgb,var(--color-info)_6%,transparent)_38%,transparent_70%)] md:h-[540px] xl:h-[640px]">
                <h2 id="mp-3d-h" className="sr-only">{t('mp_3d_title', { pod: podLabel(t, p.id) })}</h2>
                <Suspense fallback={<div className="absolute inset-0 grid place-items-center text-sm text-fg-mute">{t('mp_3d_loading')}</div>}>
                  <Pod3D states={states} speed={p.speed ?? 0} lid={openFrac(p.lid)} shutter={openFrac(p.shutter)}
                    flag={p.flag === undefined ? (IN_HUB.includes(p.status) ? 0 : 1) : openFrac(p.flag)}
                    lights={OUT.includes(p.status) || DOWN.includes(p.status)} scanning={!IN_HUB.includes(p.status) && p.status !== 'crashed'}
                    pose={poseOf(p)} xray={xray === 'xray'} selected={part} onPick={select} resetKey={resetKey}
                    ariaLabel={t('mp_3d_aria', { pod: podLabel(t, p.id) })}
                    tooltip={(q) => <PartTooltip t={t} part={q} st={states[q]} p={p} />}
                    label={(q) => <BrokenLabel t={t} part={q} st={states[q]} />} />
                </Suspense>
                <div className="pointer-events-none absolute inset-x-0 top-0 flex items-start justify-between gap-2 p-3">
                  <div className="pointer-events-auto flex flex-col gap-1">
                    <span className="text-sm font-semibold text-fg">{podLabel(t, p.id)}</span>
                    <span className="text-[11px] text-fg-mute">{statusText(t, p.status)}{p.order != null ? ` · #${p.order}` : ''}</span>
                  </div>
                  <div className="pointer-events-auto flex items-center gap-2">
                    <Segmented size="sm" label={t('mp_view_mode')} value={xray} onChange={setXray}
                      options={[{ value: 'solid', label: t('mp_solid') }, { value: 'xray', label: <span className="flex items-center gap-1"><Layers className="h-3 w-3" aria-hidden />{t('mp_xray')}</span> }]} />
                    <Button variant="ghost" size="sm" className="h-8 w-8 px-0" aria-label={t('mp_reset')} title={t('mp_reset')}
                      onClick={() => setResetKey((k) => k + 1)}>
                      <RotateCcw className="h-3.5 w-3.5" aria-hidden />
                    </Button>
                  </div>
                </div>
                <div className="pointer-events-none absolute inset-x-0 bottom-0 flex flex-wrap items-end justify-between gap-2 p-3">
                  <p className="text-[11px] text-fg-mute">{t('mp_3d_hint')}</p>
                  {part && (
                    <div className="pointer-events-auto flex items-center gap-2 rounded-lg border border-accent/50 bg-surface/90 px-2.5 py-1.5 text-xs" aria-live="polite">
                      <PartTooltip t={t} part={part} st={states[part]} p={p} hint={false} />
                      {cardKey && <Button size="sm" variant="secondary" onClick={toCard}>{t('mp_to_card')}</Button>}
                    </div>
                  )}
                </div>
              </section>
              <StatusTiles p={p} onPick={select} />
              <DamageReport p={p} states={states} onPick={select} />
              <PartList states={states} selected={part} onPick={select} />
            </div>

            {/* right / 右 */}
            <div className="flex min-w-0 flex-col gap-3 [grid-area:r]">
              <BatteryCard p={p} live={live} />
              <SpeedDist p={p} hist={(k) => hist(p.id, k)} />
              <EtaTrips p={p} />
              <MiniMap p={p} st={st} zones={zones} />
            </div>
          </div>

          {p.sensors && (
            <section id="mp-sensors" aria-labelledby="mp-sens-h" className="scroll-mt-24">
              <div id="mp-sens-h"><SectionTitle sub={t('mp_sensors_sub')}>{t('mp_sensors')}</SectionTitle></div>
              <SensorCards sensors={p.sensors} hist={(k) => hist(p.id, k)} selected={part} onLocate={locate} />
            </section>
          )}
        </>
      )}
    </div>
  )
}
