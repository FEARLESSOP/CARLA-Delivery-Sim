import { useEffect, useMemo, useRef, useState } from 'react'
import { Check, ChevronDown, Map as MapIcon, Play, RotateCcw, Search, TerminalSquare, Zap } from 'lucide-react'
import '@/i18n/launch'
import { launcherAPI, type CarEntry, type ControlRequest, type LaunchOptions, type LauncherInfo, type MapEntry, type OptionSpec } from '@/lib/api'
import { useT } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { refreshSim, simPhase, useSim } from '@/lib/sim'
import { ago } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Badge, Button, Card, EmptyState, Segmented, Select, Slider, Stepper, TextInput, Toggle } from '@/components/ui'
import Aurora from '@/components/bits/Aurora'
import ShinyText from '@/components/bits/ShinyText'
import StarBorder from '@/components/bits/StarBorder'
import SpotlightCard from '@/components/bits/SpotlightCard'

// The Simulation page: the launch form when nothing runs, live controls while
// it runs (launcher.py, pipeline_control.py). Built from launch_options.py's
// schema, so a new option there appears here without UI code.
// 起動ページ（停止中は起動フォーム、実行中はライブ操作）

type T = ReturnType<typeof useT>
const label = (t: T, o: OptionSpec) => { const k = `opt_${o.key}`; const s = t(k); return s === k ? o.label : s }
const help = (t: T, o: OptionSpec) => { const k = `opt_${o.key}_h`; const s = t(k); return s === k ? o.help : s }

function LiveTag({ kind }: { kind: OptionSpec['live'] }) {
  const t = useT()
  const tone = kind === 'live' ? 'good' : kind === 'map' ? 'info' : 'warn'
  return <Badge tone={tone} title={t(`l_${kind}_h`)}>{t(`l_${kind}`)}</Badge>
}

/* --------------------------------------------------------------- map */
function MapPicker({ maps, value, onPick, current }: {
  maps: MapEntry[]; value: string; onPick: (m: string) => void; current?: string
}) {
  const t = useT()
  return (
    <div className="grid grid-cols-2 gap-2.5 sm:grid-cols-3 2xl:grid-cols-4" role="radiogroup" aria-label={t('l_maps')}>
      {maps.map((m) => {
        const on = m.name === value
        return (
          <button key={m.name} type="button" role="radio" aria-checked={on} onClick={() => onPick(m.name)}
            className={cn('group relative overflow-hidden rounded-xl border p-3 text-left transition-all duration-200',
              on ? 'border-accent bg-accent/10 shadow-[0_0_0_1px_var(--color-accent)]' : 'border-line bg-surface/60 hover:border-line-strong hover:bg-card-2')}>
            <div className="flex items-center justify-between gap-2">
              <MapIcon className={cn('h-4 w-4', on ? 'text-accent' : 'text-fg-mute')} aria-hidden />
              {on && <Check className="h-4 w-4 text-accent" aria-hidden />}
            </div>
            <div className="mt-2 truncate font-medium text-fg">{m.name}</div>
            <div className="mt-0.5 flex items-center gap-1.5 text-xs text-fg-mute">
              <span>{m.own ? `${t('l_own')} · ${m.package}` : t('l_carla')}</span>
              {current === m.name && <Badge tone="good" className="ml-auto">{t('l_current')}</Badge>}
            </div>
          </button>
        )
      })}
    </div>
  )
}

/* --------------------------------------------------------------- car */
function CarPicker({ cars, value, onChange }: { cars: CarEntry[]; value: string; onChange: (v: string) => void }) {
  const t = useT()
  const [q, setQ] = useState('')
  const shown = cars.filter((c) => !q || `${c.name} ${c.id}`.toLowerCase().includes(q.toLowerCase()))
  return (
    <div className="space-y-2">
      <div className="relative">
        <Search className="pointer-events-none absolute left-3 top-1/2 h-4 w-4 -translate-y-1/2 text-fg-mute" aria-hidden />
        <input value={q} onChange={(e) => setQ(e.target.value)} placeholder={t('l_search')} aria-label={t('l_search')}
          className="h-9 w-full rounded-lg border border-line bg-surface pl-9 pr-3 text-sm outline-none focus:border-accent" />
      </div>
      <Select value={value} onChange={onChange}
        options={shown.map((c) => ({ value: c.id, label: c.own ? c.name : c.name, group: c.own ? t('l_own_cars') : t('l_carla_cars') }))
          .concat(shown.some((c) => c.id === value) ? [] : [{ value, label: value, group: '' }])} />
    </div>
  )
}

/* ------------------------------------------------------- one option */
function OptionField({ o, value, onChange, info }: {
  o: OptionSpec; value: LaunchOptions[string]; onChange: (v: LaunchOptions[string]) => void; info: LauncherInfo
}) {
  const t = useT()
  const L = <span className="inline-flex items-center gap-2">{label(t, o)}<LiveTag kind={o.live} /></span>
  switch (o.type) {
    case 'bool':
    case 'nbool':
      return <Toggle label={L} help={help(t, o)} checked={Boolean(value)} onChange={onChange} />
    case 'int':
      if ((o.max ?? 0) - (o.min ?? 0) > 60 && o.key !== 'vehicles' && o.key !== 'walkers')
        return <Slider label={L} help={help(t, o)} value={Number(value)} min={o.min ?? 0} max={o.max ?? 100} onChange={onChange} />
      return <Stepper label={L} help={help(t, o)} value={Number(value)} min={o.min ?? 0} max={o.max ?? 100}
        step={o.key === 'vehicles' || o.key === 'walkers' ? 5 : 1} onChange={onChange} />
    case 'float':
      return <Slider label={L} help={help(t, o)} value={Number(value)} min={o.min ?? 0} max={o.max ?? 100} step={1} unit="°" onChange={onChange} />
    case 'choice':
      return (
        <div className="py-1">
          <div className="mb-1.5 text-sm text-fg">{L}</div>
          <Segmented value={String(value)} onChange={onChange} options={(o.choices ?? []).map((c) => ({ value: c, label: c }))} />
          <div className="mt-1 text-xs text-fg-mute">{help(t, o)}</div>
        </div>
      )
    case 'vehicle':
      return (
        <div className="py-1">
          <div className="mb-1.5 text-sm text-fg">{L}</div>
          <CarPicker cars={info.catalog.cars} value={String(value)} onChange={onChange} />
          <div className="mt-1 text-xs text-fg-mute">{help(t, o)}</div>
        </div>
      )
    case 'map':
      return null           // the map has its own panel / マップは専用の欄
    default:
      return <TextInput label={L} help={help(t, o)} value={String(value)} onChange={onChange} />
  }
}

function GroupPanel({ g, title, info, opts, set, open: open0 = true }: {
  g: string; title: string; info: LauncherInfo; opts: LaunchOptions; set: (k: string, v: LaunchOptions[string]) => void; open?: boolean
}) {
  const [open, setOpen] = useState(open0)
  const fields = info.schema.filter((o) => o.group === g && o.type !== 'map')
  if (!fields.length) return null
  return (
    <section className="rounded-xl border border-line bg-surface/50">
      <button type="button" aria-expanded={open} onClick={() => setOpen(!open)}
        className="flex w-full items-center justify-between px-4 py-3 text-left">
        <span className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{title}</span>
        <ChevronDown className={cn('h-4 w-4 text-fg-mute transition-transform duration-200', open && 'rotate-180')} aria-hidden />
      </button>
      {open && (
        <div className="divide-y divide-line/60 px-4 pb-3">
          {fields.map((o) => <OptionField key={o.key} o={o} info={info} value={opts[o.key]} onChange={(v) => set(o.key, v)} />)}
        </div>
      )}
    </section>
  )
}

/* ---------------------------------------------------------------- log */
function LogPanel({ active }: { active: boolean }) {
  const t = useT()
  const { data } = usePoll<{ lines: string[] }>('/api/pipeline/log?n=300', 1000, active)
  const ref = useRef<HTMLPreElement>(null)
  const stick = useRef(true)
  useEffect(() => { const el = ref.current; if (el && stick.current) el.scrollTop = el.scrollHeight }, [data])
  return (
    <Card title={<span className="inline-flex items-center gap-2"><TerminalSquare className="h-3.5 w-3.5" aria-hidden />{t('l_log')}</span>}>
      <pre ref={ref} onScroll={(e) => { const el = e.currentTarget; stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 40 }}
        className="num h-72 overflow-auto rounded-lg bg-bg/80 p-3 text-[12px] leading-relaxed text-fg-2" aria-live="polite">
        {data?.lines.length ? data.lines.map((l, i) => (
          <div key={i} className={cn(/error|traceback|failed/i.test(l) ? 'text-danger' : /warning/i.test(l) ? 'text-warn' : '')}>{l}</div>
        )) : <span className="text-fg-mute">{t('l_log_empty')}</span>}
      </pre>
    </Card>
  )
}

/** What will start, in one glance. / 起動内容の要約 */
function Summary({ info, opts }: { info: LauncherInfo; opts: LaunchOptions }) {
  const t = useT()
  const car = info.catalog.cars.find((c) => c.id === opts.start_vehicle)
  const spec = (k: string) => info.schema.find((o) => o.key === k)!
  const items: [string, string][] = [
    [t('l_maps'), String(opts.map)],
    [label(t, spec('vehicles')), opts.traffic ? String(opts.vehicles) : '0'],
    [label(t, spec('walkers')), opts.traffic ? String(opts.walkers) : '0'],
    [label(t, spec('cyclists')), opts.traffic ? String(opts.cyclists) : '0'],
    [label(t, spec('delivery')), String(opts.delivery)],
    [label(t, spec('start_vehicle')), car?.name ?? String(opts.start_vehicle)],
    [label(t, spec('control_res')), String(opts.control_res)],
  ]
  return (
    <dl className="mt-5 grid grid-cols-2 gap-3 border-t border-line pt-4 sm:grid-cols-4">
      {items.map(([k, v]) => (
        <div key={k} className="min-w-0 rounded-lg bg-surface/70 px-3 py-2">
          <dt className="text-xs text-fg-mute">{k}</dt>
          <dd className="num truncate text-lg font-semibold text-fg">{v}</dd>
        </div>
      ))}
    </dl>
  )
}

/* ------------------------------------------------------- launch form */
function LaunchForm({ info }: { info: LauncherInfo }) {
  const t = useT()
  const [opts, setOpts] = useState<LaunchOptions>(info.saved)
  const [busy, setBusy] = useState(false)
  const set = (k: string, v: LaunchOptions[string]) => setOpts((o) => ({ ...o, [k]: v }))
  const defaults = useMemo(() => Object.fromEntries(info.schema.map((o) => [o.key, o.default])) as LaunchOptions, [info.schema])
  const last = info.launcher?.last_exit
  const launch = async () => { setBusy(true); try { await launcherAPI.launch(opts); await refreshSim() } finally { setTimeout(() => setBusy(false), 1500) } }
  return (
    <div className="space-y-5">
      <div className="relative overflow-hidden rounded-2xl border border-line bg-surface">
        <div className="pointer-events-none absolute inset-0 opacity-60" aria-hidden>
          <Aurora colorStops={['#0b1120', '#16a34a', '#1e3a8a']} amplitude={0.9} blend={0.6} speed={0.6} />
        </div>
        <div className="relative flex flex-col gap-4 p-6 md:flex-row md:items-end md:justify-between md:p-8">
          <div className="max-w-2xl">
            <h1 className="text-2xl font-semibold md:text-3xl">
              <ShinyText text={t('l_title')} color="#f1f5f9" shineColor="#86efac" speed={4} />
            </h1>
            <p className="mt-2 text-fg-2">{t('l_sub')}</p>
            {last && <p className={cn('mt-2 text-sm', last.rc ? 'text-warn' : 'text-fg-mute')}>
              {last.rc ? t('l_last_exit', { rc: last.rc }) : t('l_last_ok')} · {ago(last.t)}</p>}
          </div>
          <div className="flex items-center gap-3">
            <Button variant="ghost" onClick={() => setOpts(defaults)}><RotateCcw className="h-4 w-4" aria-hidden />{t('l_reset')}</Button>
            <StarBorder as="button" onClick={launch} disabled={busy} color="#22c55e" duration={3} radius={12}
              className="h-12 min-w-40 px-6 text-base font-semibold" backgroundColor="#22c55e" textColor="#052e16">
              <span className="inline-flex items-center gap-2"><Play className="h-4 w-4" aria-hidden />{busy ? t('l_launching') : t('l_launch')}</span>
            </StarBorder>
          </div>
        </div>
      </div>

      <div className="grid gap-5 xl:grid-cols-[1.4fr_1fr]">
        <SpotlightCard className="rounded-2xl !bg-card p-4" spotlightColor="rgba(34,197,94,0.18)">
          <div className="mb-3 flex items-center justify-between">
            <h2 className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('l_maps')}</h2>
            <LiveTag kind="map" />
          </div>
          <MapPicker maps={info.catalog.maps} value={String(opts.map)} onPick={(m) => set('map', m)} />
          <Summary info={info} opts={opts} />
        </SpotlightCard>
        <div className="space-y-3">
          {info.groups.filter(([g]) => g !== 'world').map(([g]) => (
            <GroupPanel key={g} g={g} title={t(`grp_${g}`)} info={info} opts={opts} set={set} open={g !== 'advanced' && g !== 'display'} />
          ))}
        </div>
      </div>
    </div>
  )
}

/* ------------------------------------------------------ live controls */
function useSent() {
  const [sent, setSent] = useState<string | null>(null)
  useEffect(() => { if (!sent) return; const id = setTimeout(() => setSent(null), 2500); return () => clearTimeout(id) }, [sent])
  return [sent, setSent] as const
}

function LiveControls({ info }: { info: LauncherInfo }) {
  const t = useT()
  const ps = info.pipeline!
  const base: LaunchOptions = { ...info.saved, ...(info.launcher?.options ?? {}) }
  const [opts, setOpts] = useState<LaunchOptions>({
    ...base, map: ps.map, vehicles: ps.vehicles, walkers: ps.walkers, cyclists: ps.cyclists, delivery: ps.delivery,
    start_vehicle: ps.start_vehicle || base.start_vehicle,
  })
  const set = (k: string, v: LaunchOptions[string]) => setOpts((o) => ({ ...o, [k]: v }))
  const [sent, setSent] = useSent()
  const [sureMap, setSureMap] = useState<string | null>(null)
  const send = async (req: ControlRequest, tag: string) => { await launcherAPI.control(req); setSent(tag); refreshSim() }
  const spec = (k: string) => info.schema.find((o) => o.key === k)!

  const trafficDirty = opts.vehicles !== ps.vehicles || opts.walkers !== ps.walkers || opts.cyclists !== ps.cyclists
  const helpers: [string, boolean][] = [
    ['ffb', ps.ffb], ['traction_fx', ps.traction_fx], ['weather_cycle', ps.weather_cycle],
    ['sensor_viewer', ps.running.includes('sensor_viewer')], ['drone_cams', ps.running.includes('drone_cams')],
  ]
  // restart-only options that differ from what runs / 実行中と異なる再起動項目
  const running = info.launcher?.options ?? null
  const restartDiff = running ? info.schema.filter((o) => o.live === 'restart' && opts[o.key] !== running[o.key]) : []

  return (
    <div className="space-y-5">
      <div className="flex flex-wrap items-end justify-between gap-3">
        <div>
          <h1 className="text-2xl font-semibold">{t('l_running')}</h1>
          <p className="text-fg-mute">{t('l_running_h')}</p>
        </div>
        {sent && <Badge tone="good" dot>{t('l_applied')}: {sent}</Badge>}
      </div>
      {!ps.launcher && <div className="rounded-xl border border-warn/40 bg-warn/10 p-3 text-sm text-warn">{t('l_cli')}</div>}

      <div className="grid gap-5 xl:grid-cols-[1.4fr_1fr]">
        <Card title={t('l_maps')} actions={<LiveTag kind="map" />}>
          <MapPicker maps={info.catalog.maps} value={String(opts.map)} current={ps.map}
            onPick={(m) => { set('map', m); setSureMap(null) }} />
          {opts.map !== ps.map && (
            <Button variant={sureMap ? 'primary' : 'secondary'} disabled={ps.phase === 'switching'}
              onClick={() => { if (sureMap) { send({ action: 'map', map: String(opts.map) }, String(opts.map)); setSureMap(null) } else setSureMap(String(opts.map)) }}>
              <MapIcon className="h-4 w-4" aria-hidden />{sureMap ? t('l_switch_sure', { m: String(opts.map) }) : t('l_switch', { m: String(opts.map) })}
            </Button>
          )}
        </Card>
        <div className="space-y-4">
          <Card title={t('grp_traffic')} actions={<LiveTag kind="live" />}>
            {(['vehicles', 'walkers', 'cyclists'] as const).map((k) => (
              <Stepper key={k} label={label(t, spec(k))} value={Number(opts[k])} min={0} max={spec(k).max ?? 100}
                step={k === 'cyclists' ? 1 : 5} onChange={(v) => set(k, v)} />
            ))}
            <div className="flex items-center justify-between gap-3">
              <span className="text-xs text-fg-mute">{t('l_traffic_h')}</span>
              <Button variant="primary" size="sm" disabled={!trafficDirty || !ps.traffic}
                onClick={() => send({ action: 'traffic', vehicles: Number(opts.vehicles), walkers: Number(opts.walkers), cyclists: Number(opts.cyclists) }, t('grp_traffic'))}>
                <Zap className="h-3.5 w-3.5" aria-hidden />{t('l_apply')}</Button>
            </div>
          </Card>
          <Card title={t('grp_delivery')} actions={<LiveTag kind="live" />}>
            <Stepper label={label(t, spec('delivery'))} value={Number(opts.delivery)} min={0} max={spec('delivery').max ?? 10}
              onChange={(v) => set('delivery', v)} />
            <div className="flex items-center justify-between gap-3">
              <span className="text-xs text-fg-mute">{t('l_pods_h')}</span>
              <Button variant="primary" size="sm" disabled={opts.delivery === ps.delivery}
                onClick={() => send({ action: 'delivery', vans: Number(opts.delivery) }, t('grp_delivery'))}>
                <Zap className="h-3.5 w-3.5" aria-hidden />{t('l_apply')}</Button>
            </div>
          </Card>
        </div>
      </div>

      <div className="grid gap-5 md:grid-cols-2 xl:grid-cols-3">
        <Card title={label(t, spec('start_vehicle'))} actions={<LiveTag kind="live" />}>
          <CarPicker cars={info.catalog.cars} value={String(opts.start_vehicle)} onChange={(v) => set('start_vehicle', v)} />
          <div className="flex items-center justify-between gap-3">
            <span className="text-xs text-fg-mute">{t('l_car_h')}</span>
            <Button size="sm" disabled={opts.start_vehicle === ps.start_vehicle}
              onClick={() => send({ action: 'vehicle', vehicle: String(opts.start_vehicle) }, label(t, spec('start_vehicle')))}>{t('l_car_apply')}</Button>
          </div>
        </Card>
        <Card title={t('grp_environment')} actions={<LiveTag kind="live" />}>
          {helpers.map(([k, on]) => (
            <Toggle key={k} label={label(t, spec(k))} help={help(t, spec(k))} checked={on}
              disabled={k === 'drone_cams' && ps.delivery <= 0}
              onChange={(v) => send({ action: 'helper', name: k, on: v }, label(t, spec(k)))} />
          ))}
        </Card>
        <Card title={t('l_need_restart')} actions={<LiveTag kind="restart" />}>
          <p className="text-xs text-fg-mute">{t('l_need_restart_h')}</p>
          <div className="divide-y divide-line/60">
            {info.schema.filter((o) => o.live === 'restart').map((o) => (
              <OptionField key={o.key} o={o} info={info} value={opts[o.key]} onChange={(v) => set(o.key, v)} />
            ))}
          </div>
          {ps.launcher ? (
            <Button variant={restartDiff.length ? 'primary' : 'secondary'} disabled={!restartDiff.length}
              onClick={() => launcherAPI.relaunch(opts).then(refreshSim)}>
              <RotateCcw className="h-4 w-4" aria-hidden />{restartDiff.length ? t('l_relaunch') : t('l_nothing')}
            </Button>
          ) : <p className="text-xs text-warn">{t('l_cli')}</p>}
        </Card>
      </div>
      <LogPanel active />
    </div>
  )
}

/* -------------------------------------------------------------- page */
export default function LaunchView() {
  const t = useT()
  const info = useSim()
  const phase = simPhase(info)
  if (!info) return <EmptyState>{t('l_offline')}</EmptyState>
  if (phase === 'running' || phase === 'switching') return <LiveControls key={info.pipeline?.pid} info={info} />
  if (phase === 'starting' || phase === 'stopping')
    return (
      <div className="space-y-5">
        <div className="flex items-center gap-4 rounded-2xl border border-line bg-surface p-6">
          <span className="h-8 w-8 animate-spin rounded-full border-[3px] border-accent border-t-transparent" aria-hidden />
          <div>
            <h1 className="text-xl font-semibold">{phase === 'starting' ? t('l_starting') : t('st_stopping')}</h1>
            <p className="text-fg-mute">{t('l_starting_h')}</p>
          </div>
          {phase === 'starting' && <Button variant="danger" className="ml-auto" onClick={() => launcherAPI.stop()}>{t('btn_stop')}</Button>}
        </div>
        <LogPanel active />
      </div>
    )
  return <LaunchForm info={info} />
}
