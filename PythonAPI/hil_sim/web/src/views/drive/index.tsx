import type { ReactNode } from 'react'
import { Activity, ArrowBigLeft, ArrowBigRight, Car, Gauge, MapPin, Navigation, Route, Satellite, ShieldCheck, Timer, TrendingUp } from 'lucide-react'
import '@/i18n/drive'
import { useT } from '@/lib/i18n'
import { num, useTelemetry, type Telemetry } from '@/lib/telemetry'
import { C } from '@/lib/colors'
import { clamp, fmt, fmtDur } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Badge, Bar, Card, EmptyState, Grid, Indicator, KV, SectionTitle, Stat, type Tone } from '@/components/ui'
import { StreamChart } from '@/components/charts/StreamChart'
import CountUp from '@/components/bits/CountUp'
import { H, HIST_LEN, samples } from './history'
import './alerts'   // alert rules + beeps (the banner is mounted by App on every page)
import { CollChart, GGDiagram, KIND, LaneVis, Minimap, Trail } from './canvases'
import { DayCycleCard, SeasonCard, WeatherCard, WindowsCard } from './environment'
import { EventsCard } from './events'
import { bool, gnssOf, imuOf, nearbyOf, numsOf, pointsOf, str, vec3Of } from './types'

// The Drive page: what the hero car is doing right now. A hero strip with the
// numbers a driver checks first, then Driving, Dynamics & safety,
// Environment, Location and Session. Port of the old dashboard's view-dash.
// 運転ページ：上部に速度・制限・ギア・表示灯、下に分類したカード群

type S = Telemetry | null

const Title = ({ i: I, children }: { i: typeof Gauge; children: ReactNode }) =>
  <span className="inline-flex items-center gap-2"><I className="h-3.5 w-3.5" aria-hidden />{children}</span>

/** Old pillFor(): limit + 10 is OVER, anything above is "+". */
function speedPill(delta: number): [Tone, string] {
  if (delta > 10) return ['bad', 'dv_pill_over']
  if (delta > 0) return ['warn', '+']
  return ['good', 'dv_pill_ok']
}

const ttcTone = (v: number): Tone => v > 0 && v < 2 ? 'bad' : v > 0 && v < 4 ? 'warn' : 'neutral'
const thwTone = (v: number): Tone => v > 0 && v < 1 ? 'bad' : v > 0 && v < 2 ? 'warn' : 'neutral'
const pctTone = (v: number, good: number, ok: number): Tone => v > good ? 'good' : v > ok ? 'warn' : 'bad'
const tlTone = (tls: string): Tone => tls === 'Red' ? 'bad' : tls === 'Yellow' ? 'warn' : tls === 'Green' ? 'good' : 'neutral'
const dash = (v: number, d: number) => v > 0 ? fmt(v, d) : '—'
const tlState = (s: S) => bool(s, 'at_traffic_light') ? str(s, 'traffic_light', 'None') : 'None'

/* ---------------------------------------------------------------- hero */
function Chip({ k, v }: { k: string; v: ReactNode }) {
  return (
    <span className="inline-flex items-baseline gap-1.5 rounded-md border border-line bg-surface/60 px-2 py-0.5 text-xs text-fg-mute">
      {k}<b className="num font-medium text-fg-2">{v}</b>
    </span>
  )
}

function Signal({ on, side, label }: { on: boolean; side: 'l' | 'r'; label: string }) {
  const I = side === 'l' ? ArrowBigLeft : ArrowBigRight
  return (
    <span role="img" aria-label={`${label}${on ? ' ●' : ''}`} title={label}
      className={cn('grid h-10 w-10 place-items-center rounded-lg border transition-colors',
        on ? 'animate-pulse border-accent/60 bg-accent/15 text-accent' : 'border-line text-fg-mute/40')}>
      <I className="h-6 w-6" fill={on ? 'currentColor' : 'none'} aria-hidden />
    </span>
  )
}

function Hero({ s }: { s: S }) {
  const t = useT()
  const v = num(s, 'speed_kmh'), lim = num(s, 'speed_limit_kmh'), delta = num(s, 'speed_delta_kmh')
  const [pTone, pKey] = speedPill(delta)
  const gearN = num(s, 'gear')
  const gear = gearN === -1 ? 'R' : gearN === 0 ? 'N' : String(gearN)
  const sig = str(s, 'turn_signal', 'off')
  const tls = tlState(s)
  const vtype = str(s, 'vehicle_type', '-').replace('vehicle.', '')
  const speedTone = pTone === 'good' ? 'text-fg' : pTone === 'warn' ? 'text-warn' : 'text-danger'
  return (
    <section aria-label={t('dv_card_speed')}
      className="relative overflow-hidden rounded-[var(--radius-card)] border border-line bg-card/80 p-4 shadow-[var(--shadow-card)] md:p-5">
      <div className="flex flex-wrap gap-1.5">
        <Chip k={t('dv_chip_map')} v={str(s, 'map', '-')} />
        <Chip k={t('dv_chip_vehicle')} v={vtype} />
        <Chip k={t('dv_chip_mass')} v={fmt(num(s, 'mass_kg'), 0)} />
        <Chip k={t('dv_chip_nearby')} v={num(s, 'nearby_vehicles')} />
        <Chip k={t('dv_fps')} v={fmt(num(s, 'sim_fps'), 1)} />
        <Chip k={t('dv_chip_simt')} v={fmt(num(s, 'sim_time_sec'), 0)} />
      </div>

      <div className="mt-4 grid grid-cols-2 items-center gap-x-6 gap-y-5 sm:grid-cols-[auto_auto_auto_1fr]">
        {/* speed / 速度 */}
        <div className="col-span-2 min-w-0 sm:col-span-1">
          <div className="flex items-center gap-2 text-xs text-fg-mute">
            {t('dv_card_speed')}
            <Badge tone={pTone}>{t(pKey)}</Badge>
          </div>
          <div className="flex items-baseline gap-2">
            <span className={cn('num text-7xl font-semibold leading-none tracking-tight md:text-8xl', speedTone)}>{fmt(v, 0)}</span>
            <span className="text-lg text-fg-mute">{t('dv_kmh')}</span>
          </div>
          <div className="mt-1 text-sm text-fg-mute">
            {t('dv_lbl_avg')} <b className="num font-medium text-fg-2">{fmt(num(s, 'avg_kmh'), 0)}</b> {t('dv_kmh')}
            {delta > 0 && lim > 0 && <span className={cn('ml-2', pTone === 'bad' ? 'text-danger' : 'text-warn')}>{t('dv_speed_vs', { d: fmt(delta, 0) })}</span>}
          </div>
        </div>

        {/* limit sign / 制限速度標識 */}
        <div className="flex flex-col items-center gap-1">
          <div role="img" aria-label={`${t('dv_lbl_limit')} ${lim > 0 ? fmt(lim, 0) : t('dv_no_limit')}`}
            className={cn('grid h-20 w-20 place-items-center rounded-full border-[7px] md:h-24 md:w-24',
              lim > 0 ? 'border-danger bg-fg text-bg' : 'border-line bg-surface text-fg-mute')}>
            <span className="num text-3xl font-bold md:text-4xl">{lim > 0 ? fmt(lim, 0) : '–'}</span>
          </div>
          <span className="text-xs text-fg-mute">{t('dv_lbl_limit')}</span>
        </div>

        {/* gear + heading / ギアと方位 */}
        <div className="flex min-w-0 flex-wrap items-center gap-x-5 gap-y-2">
          <div>
            <div className="text-xs text-fg-mute">{t('dv_lbl_gear')}</div>
            <div className={cn('num text-6xl font-semibold leading-none', gear === 'R' ? 'text-warn' : 'text-fg')}>{gear}</div>
          </div>
          <div>
            <div className="text-xs text-fg-mute">{t('dv_lbl_heading')}</div>
            <div className="flex items-center gap-1.5">
              <Navigation className="h-5 w-5 text-info" style={{ transform: `rotate(${num(s, 'compass_deg')}deg)` }} aria-hidden />
              <span className="num text-xl md:text-2xl font-semibold text-fg">{fmt(num(s, 'heading_deg'), 1)}°</span>
            </div>
          </div>
        </div>

        {/* indicators / 表示灯 */}
        <div className="col-span-2 flex flex-wrap items-center gap-3 sm:col-span-1 sm:justify-end">
          <div className="flex items-center gap-1.5" role="group" aria-label={t('dv_signals')}>
            <Signal side="l" on={sig === 'left' || sig === 'hazard'} label={sig === 'hazard' ? t('dv_sig_hazard') : t('dv_sig_left')} />
            <Signal side="r" on={sig === 'right' || sig === 'hazard'} label={sig === 'hazard' ? t('dv_sig_hazard') : t('dv_sig_right')} />
          </div>
          <div className="grid grid-cols-2 gap-1.5">
            <Indicator on={bool(s, 'reverse')}>{t('dv_ind_rev')}</Indicator>
            <Indicator on={bool(s, 'hand_brake')}>{t('dv_ind_hb')}</Indicator>
            <Indicator on={bool(s, 'abs_active')}>{t('dv_ind_abs')}</Indicator>
            <Indicator on={bool(s, 'tcs_active')}>{t('dv_ind_tcs')}</Indicator>
          </div>
          <div className="min-w-24">
            <div className="text-xs text-fg-mute">{t('dv_lbl_tl')}</div>
            <Badge tone={tlTone(tls)} dot>{t(`dv_tl_${tls}`)}</Badge>
          </div>
        </div>
      </div>
    </section>
  )
}

/* ------------------------------------------------------------- driving */
function SteerBar({ v, label }: { v: number; label: string }) {
  const sp = clamp(v, -1, 1)
  return (
    <div className="relative h-1.5 w-full overflow-hidden rounded-full bg-muted" role="meter" aria-label={label}
      aria-valuemin={-100} aria-valuemax={100} aria-valuenow={Math.round(sp * 100)}>
      <i className="absolute inset-y-0 left-1/2 w-px bg-line-strong" aria-hidden />
      <i className="absolute inset-y-0 rounded-full bg-s3 transition-[width,left] duration-200"
        style={sp >= 0 ? { left: '50%', width: `${sp * 50}%` } : { left: `${50 + sp * 50}%`, width: `${-sp * 50}%` }} />
    </div>
  )
}

function InputsCard({ s }: { s: S }) {
  const t = useT()
  const th = num(s, 'throttle'), br = num(s, 'brake'), st = num(s, 'steer')
  const row = (k: string, val: string, bar: ReactNode) => (
    <div>
      <div className="flex items-baseline justify-between text-sm"><span className="text-fg-mute">{t(k)}</span><b className="num font-medium text-fg">{val}</b></div>
      {bar}
    </div>
  )
  return (
    <Card title={<Title i={Gauge}>{t('dv_card_inputs')}</Title>}>
      <div className="space-y-2">
        {row('dv_lbl_throttle', `${Math.round(th * 100)}%`, <Bar value={th} tone="good" label={t('dv_lbl_throttle')} />)}
        {row('dv_lbl_brake', `${Math.round(br * 100)}%`, <Bar value={br} tone="bad" label={t('dv_lbl_brake')} />)}
        {row('dv_lbl_steer', fmt(st, 2), <SteerBar v={st} label={t('dv_lbl_steer')} />)}
      </div>
      <StreamChart min={-100} max={100} unit="%" decimals={0} len={HIST_LEN} tick={samples} height={130}
        series={[
          { label: t('dv_lbl_throttle'), color: C.s4, data: H.throttle.data },
          { label: t('dv_lbl_brake'), color: C.s2, data: H.brake.data },
          { label: t('dv_lbl_steer'), color: C.s3, data: H.steer.data, dash: true },
        ]} />
    </Card>
  )
}

function SpeedHistCard() {
  const t = useT()
  let hi = 60
  for (const v of H.speed.data) if (v > hi) hi = v
  for (const v of H.limit.data) if (v > hi) hi = v
  return (
    <Card title={<Title i={TrendingUp}>{t('dv_card_speedhist')}</Title>} wide>
      <StreamChart min={0} max={Math.ceil(hi * 1.1)} unit=" km/h" decimals={0} len={HIST_LEN} tick={samples} height={210}
        series={[
          { label: t('dv_lg_speed'), color: C.s1, data: H.speed.data },
          { label: t('dv_lg_limit'), color: C.s3, data: H.limit.data, dash: true },
          { label: t('dv_lg_avg'), color: C.s4, data: H.avg.data },
        ]} />
    </Card>
  )
}

function LaneCard({ s }: { s: S }) {
  const t = useT()
  const on = bool(s, 'on_road')
  const junc = bool(s, 'at_junction')
  return (
    <Card title={<Title i={Route}>{t('dv_card_lane')}</Title>}>
      <div>
        <KV k={t('dv_lbl_onroad')} v={on ? t('dv_yes') : t('dv_offroad')} tone={on ? 'good' : 'bad'} />
        <KV k={t('dv_lbl_roadlane')} v={`${str(s, 'road_id', '-')} / ${str(s, 'lane_id', '-')}`} />
        <KV k={t('dv_lbl_lanew')} v={`${fmt(num(s, 'lane_width'), 2)} m`} />
        <KV k={t('dv_lbl_laneoff')} v={`${fmt(num(s, 'lane_offset'), 2)} m`} />
        <KV k={t('dv_lbl_junction')} v={junc ? t('dv_yes') : t('dv_no')} tone={junc ? 'info' : 'neutral'} />
      </div>
      <LaneVis width={num(s, 'lane_width')} offset={num(s, 'lane_offset')} onRoad={on} label={t('dv_card_lane')} />
    </Card>
  )
}

function LanePosCard({ s }: { s: S }) {
  const t = useT()
  return (
    <Card title={<Title i={Route}>{t('dv_card_lanepos')}</Title>} wide
      actions={<span className="num text-sm text-fg-2">SDLP {fmt(num(s, 'sdlp'), 3)} m</span>}>
      <StreamChart min={-2.5} max={2.5} unit=" m" decimals={2} len={HIST_LEN} tick={samples} height={180}
        series={[
          { label: t('dv_lg_laneoff'), color: C.s1, data: H.laneoff.data },
          { label: t('dv_lane_edge_l'), color: C.s2, data: H.laneEdgeP.data, dash: true },
          { label: t('dv_lane_edge_r'), color: C.s2, data: H.laneEdgeN.data, dash: true },
        ]} />
    </Card>
  )
}

function YawCard() {
  const t = useT()
  return (
    <Card title={<Title i={Activity}>{t('dv_card_yaw')}</Title>} wide>
      <StreamChart min={-90} max={90} decimals={1} len={HIST_LEN} tick={samples} height={180}
        series={[
          { label: t('dv_lg_yaw'), color: C.s1, data: H.yaw.data },
          { label: t('dv_lg_steer100'), color: C.s3, data: H.steerY.data, dash: true },
        ]} />
    </Card>
  )
}

/* ------------------------------------------------- dynamics & safety */
function SafetyCard({ s }: { s: S }) {
  const t = useT()
  const fd = num(s, 'front_distance', -1), ttc = num(s, 'ttc', -1), thw = num(s, 'thw', -1)
  const crashes = num(s, 'collision_count')
  const tls = tlState(s)
  const stab = Math.round(num(s, 'stability') * 100)
  return (
    <Card title={<Title i={ShieldCheck}>{t('dv_card_safety')}</Title>} wide>
      <div className="grid gap-4 sm:grid-cols-[minmax(0,0.9fr)_minmax(0,1.1fr)]">
        <div className="space-y-2">
          <div className="grid grid-cols-3 gap-3">
            <Stat size="md" label={t('dv_lbl_fdist')} value={dash(fd, 1)} unit="m" />
            <Stat size="md" label="TTC" value={dash(ttc, 1)} unit="s" tone={ttcTone(ttc)} />
            <Stat size="md" label="THW" value={dash(thw, 1)} unit="s" tone={thwTone(thw)} />
          </div>
          <div>
            <KV k={t('dv_lbl_crashes')} v={crashes} tone={crashes > 0 ? 'bad' : 'neutral'} />
            <KV k={t('dv_lbl_hardbrakes')} v={num(s, 'hard_brake_count')} />
            <KV k={t('dv_lbl_tl')} v={t(`dv_tl_${tls}`)} tone={tlTone(tls)} />
            <KV k={t('dv_lbl_stability')} v={`${stab}%`} tone={pctTone(stab, 70, 40)} />
            <Bar value={stab / 100} tone={pctTone(stab, 70, 40)} label={t('dv_lbl_stability')} />
          </div>
        </div>
        <StreamChart min={0} max={15} unit=" s" decimals={1} len={HIST_LEN} tick={samples} height={170}
          series={[
            { label: 'THW', color: C.s1, data: H.thw.data },
            { label: 'TTC', color: C.s2, data: H.ttc.data, dash: true },
          ]} />
      </div>
    </Card>
  )
}

function AccCard() {
  const t = useT()
  return (
    <Card title={<Title i={Activity}>{t('dv_card_acc')}</Title>} wide>
      <StreamChart min={-10} max={10} unit=" m/s²" decimals={2} len={HIST_LEN} tick={samples} height={210}
        series={[
          { label: t('dv_lg_lon'), color: C.s1, data: H.lon.data },
          { label: t('dv_lg_lat'), color: C.s2, data: H.lat.data },
          { label: t('dv_lg_jerk'), color: C.s3, data: H.jerk.data, dash: true },
        ]} />
    </Card>
  )
}

function DynamicsCard({ s }: { s: S }) {
  const t = useT()
  const a = 'm/s²'
  return (
    <Card title={<Title i={Car}>{t('dv_card_dyn')}</Title>}>
      <div>
        <KV k={t('dv_lbl_lon')} v={`${fmt(num(s, 'lon_accel'))} ${a}`} />
        <KV k={t('dv_lbl_lat')} v={`${fmt(num(s, 'lat_accel'))} ${a}`} />
        <KV k={t('dv_lbl_vert')} v={`${fmt(num(s, 'vert_accel'))} ${a}`} />
        <KV k={t('dv_lbl_jerk')} v={`${fmt(num(s, 'jerk'))} m/s³`} />
        <KV k={t('dv_lbl_g')} v={`${fmt(num(s, 'g_force'))} g`} />
        <KV k={t('dv_lbl_yawrate')} v={`${fmt(num(s, 'yaw_rate_deg'), 1)}°/s`} />
        <KV k={t('dv_lbl_slip')} v={`${fmt(num(s, 'slip_angle_deg'), 1)}°`} />
        <KV k={t('dv_lbl_curv')} v={fmt(num(s, 'curvature'), 4)} />
      </div>
    </Card>
  )
}

function GGCard() {
  const t = useT()
  return (
    <Card title={<Title i={Activity}>{t('dv_card_gg')}</Title>} footer={t('dv_gg_hint')}>
      <GGDiagram tick={samples} label={t('dv_card_gg')} height={220} />
    </Card>
  )
}

function CollCard({ s }: { s: S }) {
  const t = useT()
  const vals = numsOf(s, 'collision_recent')
  const any = vals.some((v) => v > 0)
  const peak = num(s, 'collision_peak')
  return (
    <Card title={<Title i={ShieldCheck}>{t('dv_card_coll')}</Title>} wide footer={t('dv_coll_hint')}
      actions={<span className="text-sm text-fg-mute">{t('dv_lbl_collpeak')} <b className={cn('num', peak > 0 ? 'text-danger' : 'text-fg-2')}>{fmt(peak, 1)}</b></span>}>
      <div className="relative">
        <CollChart vals={vals} label={t('dv_card_coll')} height={150} />
        {!any && <div className="pointer-events-none absolute inset-0 grid place-items-center text-sm text-fg-mute">{t('dv_coll_none')}</div>}
      </div>
    </Card>
  )
}

/* ------------------------------------------------------------ location */
function Legend({ items }: { items: { label: string; color: string; shape?: string }[] }) {
  return (
    <div className="flex flex-wrap gap-x-4 gap-y-1 text-xs text-fg-mute">
      {items.map((it) => (
        <span key={it.label} className="inline-flex items-center gap-1.5">
          <i className={cn('inline-block h-2.5 w-2.5', it.shape === 'square' ? 'rounded-[2px]' : it.shape === 'diamond' ? 'rotate-45 rounded-[1px]'
            : it.shape === 'triangle' ? '[clip-path:polygon(50%_0,100%_100%,0_100%)]' : 'rounded-full')} style={{ background: it.color }} />
          {it.label}
        </span>
      ))}
    </div>
  )
}

function MinimapCard({ s }: { s: S }) {
  const t = useT()
  const list = nearbyOf(s)
  return (
    <Card title={<Title i={MapPin}>{t('dv_card_map')}</Title>} wide
      actions={<Badge tone={list.length ? 'info' : 'neutral'}>{t('dv_nearby_n', { n: list.length })}</Badge>}>
      <Minimap list={list} label={`${t('dv_card_map')}: ${list.length}`} height={300} />
      <Legend items={[
        { label: t('dv_lg_hero'), color: C.accent, shape: 'triangle' },
        { label: t('dv_lg_nearby'), color: KIND.car.color },
        { label: t('dv_lg_truck'), color: KIND.truck.color, shape: 'square' },
        { label: t('dv_lg_bike'), color: KIND.bike.color, shape: 'diamond' },
        { label: t('dv_lg_walker'), color: KIND.walker.color },
        { label: t('dv_lg_cone'), color: C.info, shape: 'square' },
      ]} />
    </Card>
  )
}

function TrailCard({ s }: { s: S }) {
  const t = useT()
  const pts = pointsOf(s, 'trail')
  return (
    <Card title={<Title i={Route}>{t('dv_card_trail')}</Title>} footer={t('dv_trail_hint')}>
      {pts.length ? <Trail pts={pts} label={t('dv_card_trail')} height={240} /> : <EmptyState>{t('dv_trail_empty')}</EmptyState>}
    </Card>
  )
}

function GnssCard({ s }: { s: S }) {
  const t = useT()
  const p = vec3Of(s, 'location'), g = gnssOf(s), m = imuOf(s)
  return (
    <Card title={<Title i={Satellite}>{t('dv_card_gnss')}</Title>}>
      <div>
        <KV k={t('dv_lbl_pos')} v={`${fmt(p.x, 1)} / ${fmt(p.y, 1)} / ${fmt(p.z, 1)}`} />
        <KV k={t('dv_lbl_lat2')} v={fmt(g.lat, 6)} />
        <KV k={t('dv_lbl_lon2')} v={fmt(g.lon, 6)} />
        <KV k={t('dv_lbl_accel')} v={`${fmt(m.ax, 1)} / ${fmt(m.ay, 1)} / ${fmt(m.az, 1)}`} />
        <KV k={t('dv_lbl_gyro')} v={`${fmt(m.gx, 2)} / ${fmt(m.gy, 2)} / ${fmt(m.gz, 2)}`} />
        <KV k={t('dv_lbl_compass')} v={`${fmt(num(s, 'compass_deg'), 1)}°`} />
      </div>
    </Card>
  )
}

/* ------------------------------------------------------------- session */
// CountUp only on totals that change rarely / まれに変わる合計値だけ
const Count = ({ v }: { v: number }) => <CountUp to={v} duration={0.6} trigger="load" />

function TripCard({ s }: { s: S }) {
  const t = useT()
  const crashes = num(s, 'collision_count')
  return (
    <Card title={<Title i={Timer}>{t('dv_card_trip')}</Title>}>
      <div className="flex items-end justify-between gap-3">
        <Stat size="xl" label={t('dv_card_trip')} value={fmt(num(s, 'trip_km'), 2)} unit="km" />
        <Stat size="md" label={t('dv_lbl_elapsed')} value={fmtDur(num(s, 'trip_sec'))} />
      </div>
      <div>
        <KV k={t('dv_lbl_avgspeed')} v={`${fmt(num(s, 'avg_kmh'), 0)} km/h`} />
        <KV k={t('dv_lbl_crashes')} v={<Count v={crashes} />} tone={crashes > 0 ? 'bad' : 'neutral'} />
        <KV k={t('dv_lbl_hardbrakes')} v={<Count v={num(s, 'hard_brake_count')} />} />
        <KV k={t('dv_lbl_collpeak')} v={fmt(num(s, 'collision_peak'), 1)} />
      </div>
    </Card>
  )
}

function PerfCard({ s }: { s: S }) {
  const t = useT()
  const sdlp = num(s, 'sdlp'), rlv = num(s, 'red_light_violations')
  const comp = clamp(num(s, 'speed_compliance', 100) || 100, 0, 100)
  return (
    <Card title={<Title i={TrendingUp}>{t('dv_card_perf')}</Title>}>
      <div>
        <KV k={t('dv_lbl_sdlp')} v={`${fmt(sdlp, 3)} m`} tone={sdlp > 0.4 ? 'bad' : sdlp > 0.25 ? 'warn' : 'neutral'} />
        <KV k={t('dv_lbl_srr')} v={`${fmt(num(s, 'steer_reversal_rate'), 0)} /min`} />
        <KV k={t('dv_lbl_jerkrms')} v={`${fmt(num(s, 'jerk_rms'), 2)} m/s³`} />
        <KV k={t('dv_lbl_laneinv')} v={<Count v={num(s, 'lane_invasions')} />} />
        <KV k={t('dv_lbl_rlv')} v={<Count v={rlv} />} tone={rlv > 0 ? 'bad' : 'neutral'} />
      </div>
      <div>
        <KV k={t('dv_lbl_comp')} v={`${fmt(comp, 1)}%`} tone={pctTone(comp, 90, 70)} />
        <Bar value={comp / 100} tone={pctTone(comp, 90, 70)} label={t('dv_lbl_comp')} />
      </div>
    </Card>
  )
}

/* ---------------------------------------------------------------- page */
const SECTIONS = ['driving', 'dyn', 'env', 'loc', 'session'] as const
type SectionId = (typeof SECTIONS)[number]

function Section({ id, children }: { id: SectionId; children: ReactNode }) {
  const t = useT()
  return (
    <section id={`drive-${id}`} aria-labelledby={`drive-${id}-h`} className="scroll-mt-20">
      <div id={`drive-${id}-h`}><SectionTitle sub={t(`dv_sec_${id}_h`)}>{t(`dv_sec_${id}`)}</SectionTitle></div>
      {children}
    </section>
  )
}

export default function DriveView() {
  const t = useT()
  const s = useTelemetry()
  const connected = !!s?.connected
  const hero = connected && !!s?.hero_found
  return (
    <div className="space-y-6">
      {!hero && s && (
        <div role="status"
          className={cn('rounded-xl border px-4 py-3 text-sm', connected ? 'border-warn/40 bg-warn/10 text-warn' : 'border-danger/40 bg-danger/10 text-danger')}>
          {connected ? <><b>{t('dv_waiting')}</b> <span className="text-fg-2">{t('dv_waiting_h')}</span></> : t('dv_offline')}
        </div>
      )}
      <Hero s={s} />

      <nav aria-label={t('dv_sections')} className="-mt-2 flex flex-wrap gap-1.5">
        {SECTIONS.map((id) => (
          <button key={id} type="button"
            onClick={() => document.getElementById(`drive-${id}`)?.scrollIntoView({ behavior: 'smooth', block: 'start' })}
            className="rounded-full border border-line px-3 py-1 text-xs text-fg-2 transition-colors duration-150 hover:bg-card-2 hover:text-fg">
            {t(`dv_sec_${id}`)}
          </button>
        ))}
      </nav>

      <Section id="driving">
        <Grid>
          <InputsCard s={s} />
          <SpeedHistCard />
          <LaneCard s={s} />
          <LanePosCard s={s} />
          <YawCard />
        </Grid>
      </Section>

      <Section id="dyn">
        <Grid>
          <SafetyCard s={s} />
          <AccCard />
          <DynamicsCard s={s} />
          <GGCard />
          <CollCard s={s} />
        </Grid>
      </Section>

      <Section id="env">
        <Grid>
          <WeatherCard s={s} />
          <DayCycleCard />
          <div className="flex min-w-0 flex-col gap-4">
            <SeasonCard />
            <WindowsCard />
          </div>
        </Grid>
      </Section>

      <Section id="loc">
        <Grid>
          <MinimapCard s={s} />
          <TrailCard s={s} />
          <GnssCard s={s} />
        </Grid>
      </Section>

      <Section id="session">
        <Grid>
          <TripCard s={s} />
          <PerfCard s={s} />
          <EventsCard s={s} />
        </Grid>
      </Section>
    </div>
  )
}
