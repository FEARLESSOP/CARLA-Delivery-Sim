// The Delivery page: car + drone delivery sim (delivery_sim.py). The City Map
// is the hero; KPIs above it, wind beside it, the fleet tables, totals, the
// event feed and the LastMile pod showcase below.
// 配送ページ：市街地図を中心に、KPI・風・機体一覧・集計・イベント・ポッド
import type { ReactNode } from 'react'
import '@/i18n/delivery'
import { useT } from '@/lib/i18n'
import { fmt } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Badge, SectionTitle, Stat, type Tone } from '@/components/ui'
import CountUp from '@/components/bits/CountUp'
import CityMap from './CityMap'
import WindCard from './WindCard'
import PodPanel from './PodPanel'
import { DronesTable, EventsFeed, PodsTable, StatsCard } from './tables'
import { AT_STATION, DOWN, FLYING, IN_POD, useDeliveryState } from './shared'
import type { DeliveryState } from './types'

function Tile({ children, className }: { children: ReactNode; className?: string }) {
  return (
    <div className={cn('min-w-0 rounded-[var(--radius-card)] border border-line bg-card/80 px-4 py-3 shadow-[var(--shadow-card)]', className)}>
      {children}
    </div>
  )
}

const Count = ({ n }: { n: number }) => <CountUp to={n} duration={0.8} trigger="load" />
const TONE_TEXT: Record<Tone, string> = {
  good: 'text-accent', info: 'text-info', warn: 'text-warn', bad: 'text-danger', neutral: 'text-fg', accent: 'text-accent',
}

/** Drone fleet: total, a stacked bar and the four groups (text + colour). */
function Fleet({ s }: { s: DeliveryState | null }) {
  const t = useT()
  const ds = s?.available ? s.drones ?? [] : []
  const n = ds.length
  const parts: { key: string; label: string; v: number; bg: string; tone: Tone }[] = [
    { key: 'fly', label: t('dr_chip_flying'), v: ds.filter((d) => FLYING.includes(d.status)).length, bg: 'bg-accent', tone: 'good' },
    { key: 'pod', label: t('dr_chip_vans'), v: ds.filter((d) => IN_POD.includes(d.status)).length, bg: 'bg-info', tone: 'info' },
    { key: 'st', label: t('dr_chip_station'), v: ds.filter((d) => AT_STATION.includes(d.status)).length, bg: 'bg-warn', tone: 'warn' },
    { key: 'down', label: t('dr_chip_down'), v: ds.filter((d) => DOWN.includes(d.status)).length, bg: 'bg-danger', tone: 'bad' },
  ]
  return (
    <Tile className="col-span-2">
      <div className="flex items-baseline justify-between gap-2">
        <span className="text-xs text-fg-mute">{t('dr_chip_drones')}</span>
        <span className="num text-xl font-semibold text-fg">{n}</span>
      </div>
      <div className="mt-2 flex h-2 w-full overflow-hidden rounded-full bg-muted" aria-hidden>
        {parts.map((p) => p.v > 0 && <div key={p.key} className={cn('h-full', p.bg)} style={{ width: `${(p.v / Math.max(1, n)) * 100}%` }} />)}
      </div>
      <dl className="mt-2 grid grid-cols-4 gap-1">
        {parts.map((p) => (
          <div key={p.key} className="min-w-0">
            <dt className="flex items-center gap-1 truncate text-[11px] text-fg-mute"><i className={cn('h-2 w-2 shrink-0 rounded-sm', p.bg)} aria-hidden />{p.label}</dt>
            <dd className={cn('num text-base font-semibold', p.v ? TONE_TEXT[p.tone] : 'text-fg-mute')}>{p.v}</dd>
          </div>
        ))}
      </dl>
    </Tile>
  )
}

function Kpis({ s }: { s: DeliveryState | null }) {
  const t = useT()
  const st = s?.available ? s.stats ?? {} : {}
  const w = s?.available ? s.wind ?? {} : {}
  const vans = s?.available ? s.vans ?? [] : []
  return (
    <div className="grid grid-cols-2 gap-3 sm:grid-cols-4 xl:grid-cols-8">
      <Tile><Stat label={t('dr_chip_deliv')} tone="good" value={<Count n={st.delivered || 0} />}
        sub={`${st.delivered_drone || 0} ${t('dr_s_bydrone')} · ${st.delivered_face_to_face || 0} ${t('dr_s_f2f')}`} /></Tile>
      <Tile><Stat label={t('dr_chip_fail')} tone={(st.failed || 0) > 0 ? 'bad' : 'neutral'} value={<Count n={st.failed || 0} />}
        sub={`${st.queued || 0} ${t('dr_s_queued_short')}`} /></Tile>
      <Tile><Stat label={t('dr_s_trips')} value={<Count n={st.trips || 0} />} sub={`${fmt(st.drone_km, 2)} km`} /></Tile>
      <Tile><Stat label={t('dr_chip_busy')} value={<>{st.busy_vans || 0}<span className="text-lg text-fg-mute"> / {vans.length}</span></>}
        sub={`${fmt(st.pod_km, 2)} km`} /></Tile>
      <Fleet s={s} />
      <Tile><Stat label={t('dr_chip_wind')} tone={w.grounded ? 'bad' : w.gust ? 'warn' : 'neutral'}
        value={fmt(w.speed, 1)} unit="m/s"
        sub={`→ ${Math.round(w.dir || 0)}°${w.gust ? ` · ${t('dr_gust')}` : ''}${w.grounded ? ` · ${t('dr_grounded')}` : ''}`} /></Tile>
      <Tile><Stat label={t('chip_simt')} value={fmt(s?.t, 0)} /></Tile>
    </div>
  )
}

export default function DeliveryView() {
  const t = useT()
  const s = useDeliveryState()
  const live = !!(s?.available && s.running)
  const run = !s?.available ? t('dr_none') : s.running ? t('dr_live') : t('dr_stopped')
  const pod = s?.pod
  const sc = pod?.showcase

  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <h1 className="text-xl font-semibold text-fg">{t('dr_title')}</h1>
        <div className="flex flex-wrap items-center gap-2">
          <Badge tone={live ? 'good' : 'bad'} dot><span className="text-fg-mute">{t('dr_chip_run')}</span> {run}</Badge>
          {pod && <Badge><span className="text-fg-mute">{t('pd_chip_shell')}</span> {t(`pd_shell_${pod.shell}`)}</Badge>}
          {sc && <Badge tone="info"><span className="text-fg-mute">{t('pd_chip_on')}</span> {t('pd_pod')} {sc.van}</Badge>}
          {sc && <Badge tone="info"><span className="text-fg-mute">{t('pd_chip_scen')}</span> {t(sc.scenario === 'drone' ? 'pd_scen_drone' : 'pd_scen_direct')}</Badge>}
          {sc && <Badge tone="info" className="max-w-64 truncate"><span className="text-fg-mute">{t('pd_chip_step')}</span> {sc.title}</Badge>}
        </div>
      </div>

      <Kpis s={s} />

      <div className="grid gap-4 lg:grid-cols-[minmax(0,1fr)_360px] 2xl:grid-cols-[minmax(0,1fr)_420px]">
        <CityMap state={s} />
        <div className="min-w-0 lg:max-h-[calc(100vh-15rem)] lg:min-h-[560px] lg:overflow-y-auto lg:pr-1">
          <WindCard state={s} />
        </div>
      </div>

      <div className="grid items-start gap-4 xl:grid-cols-[minmax(0,2fr)_minmax(0,1fr)]">
        <DronesTable state={s} />
        <StatsCard state={s} />
      </div>
      <div className="grid items-start gap-4 lg:grid-cols-2">
        <PodsTable state={s} />
        <EventsFeed state={s} />
      </div>

      <section aria-labelledby="dr-pod-sect" className="flex flex-col">
        <div id="dr-pod-sect"><SectionTitle>{t('sect_pod')}</SectionTitle></div>
        <PodPanel state={s} />
      </section>
    </div>
  )
}
