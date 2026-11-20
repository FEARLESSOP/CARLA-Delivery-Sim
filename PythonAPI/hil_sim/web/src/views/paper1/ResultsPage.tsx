// Paper 1 results page (/paper1, opened in a new tab from the dashboard). Every number
// comes from /rl/paper1/data, built locally from rl_delivery/research/paper1/.
// Paper 1 結果ページ（ダッシュボードから新しいタブで開く）。数値は全てローカルのファイルから。
import { useEffect, useMemo, useState } from 'react'
import '@/i18n/paper1'
import { useLang, useT } from '@/lib/i18n'
import { C } from '@/lib/colors'
import { Badge, EmptyState, Segmented, Select } from '@/components/ui'
import Aurora from '@/components/bits/Aurora'
import CountUp from '@/components/bits/CountUp'
import FadeContent from '@/components/bits/FadeContent'
import GradientText from '@/components/bits/GradientText'
import SpotlightCard from '@/components/bits/SpotlightCard'
import { Bars, DotPlot, ForestPlot, LineChart, fmtN, type ForestRow } from './charts'

type World = 'v1' | 'v2'
interface Row { world: World; method: string; seed: number | 'avg' | null; scenario: string; tag: string; n: number; [k: string]: unknown }
interface Diff { world: World; a: string; b: string; metric: string; scenario: string; diff: number; lo: number; hi: number; sig: boolean; n: number }
interface Tune { version: string; label: string; chosen: boolean; d_ontime: number; d_ontime_ci: number; d_ready: number; d_ready_ci: number; ontime30_per_h: number; ready_s: number }
interface Check { id: string; label: string; detail: string; summary: Record<string, Record<string, number | null>>; b0_minus_pod: Record<string, { diff: number; lo: number; hi: number; sig: boolean; n: number } | null> | null }
interface Data {
  scenarios: string[]; results: Row[]; diffs: Diff[]; training: Record<string, Record<string, number | null>[]>
  checks: Check[]; tuning: { B4: Tune[]; B5: Tune[] } | null; tuning2: { B4: (Tune & { carla_chargers_of_22: number | null })[] } | null
  carla: { file: string; hardware: string; b4: string; runs: Record<string, Record<string, number | null | Record<string, number>>> }[]
  diagnosis: { decisions: number; live_B4conf: Record<string, number>; replay: Record<string, Record<string, number>>; pod_battery_median: Record<string, number>
    drone_battery_median: number; B4conf_if_pod_battery: Record<string, Record<string, number>>; B2_sure_of_pod: number[]; causes: string[]; ruled_out: string[]; fix: string } | null
  causes: { crashes: Record<string, number>; recovery_failures: Record<string, number> } | null
  missing: string[]; files: number; updated: number; error?: string
}

const MCOL: Record<string, string> = { B0: C.mute, POD: '#64748b', B1: C.s2, B2: C.s1, B3: C.s3, B4: C.accent, B4conf: C.cyan, B5: C.violet }
const MNAME: Record<string, string> = { B0: 'B0 rule', POD: 'Always pod', B1: 'B1 PPO', B2: 'B2 Maskable', B3: 'B3 +forecast', B4: 'B4 proposed', B4conf: 'B4conf (first B4)', B5: 'B5 no network' }
const ORDER = ['B0', 'POD', 'B1', 'B2', 'B3', 'B5', 'B4conf', 'B4']
const SLAB = ['Normal', 'Rush hour', 'Pod delays', 'Station outage', 'High wind', 'Order bursts', 'Unseen customers']
const MEAS: Record<string, { field: string; diff: string; unit: string; digits: number; lowerBetter: boolean; key: string }> = {
  ontime: { field: 'ontime30_per_h', diff: 'ontime30_per_h', unit: '/h', digits: 2, lowerBetter: false, key: 'p1r_m_ontime' },
  ready: { field: 'ready_time_mean_s', diff: 'ready_time_mean_s', unit: 's', digits: 0, lowerBetter: true, key: 'p1r_m_ready' },
  deliv: { field: 'delivered_per_h', diff: 'delivered_per_h', unit: '/h', digits: 2, lowerBetter: false, key: 'p1r_m_deliv' },
  fail: { field: 'recovery_failure_rate', diff: 'recovery_failure_rate', unit: '', digits: 3, lowerBetter: true, key: 'p1r_m_fail' },
  crash: { field: 'crashes', diff: 'crashes', unit: '/day', digits: 2, lowerBetter: true, key: 'p1r_m_crash' },
}
const num = (v: unknown) => (typeof v === 'number' && isFinite(v) ? v : null)

function Section({ id, title, note, children }: { id: string; title: string; note?: string; children: React.ReactNode }) {
  return (
    <section id={id} className="flex flex-col gap-3 rounded-[var(--radius-card)] border border-line bg-card p-4 shadow-[var(--shadow-card)] md:p-5">
      <div>
        <h2 className="text-base font-semibold text-fg md:text-lg" style={{ textWrap: 'balance' }}>{title}</h2>
        {note && <p className="mt-1 max-w-[80ch] text-xs leading-relaxed text-fg-mute">{note}</p>}
      </div>
      {children}
    </section>
  )
}

function Headline({ label, d, unit, digits, lowerBetter }: { label: string; d: Diff | undefined; unit: string; digits: number; lowerBetter?: boolean }) {
  const t = useT()
  if (!d) return null
  const good = lowerBetter ? d.diff < 0 : d.diff > 0
  const shown = lowerBetter ? -d.diff : d.diff
  return (
    <SpotlightCard className="!rounded-[var(--radius-card)] border border-line !bg-card p-4" spotlightColor="rgba(34, 197, 94, 0.18)">
      <div className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{label}</div>
      <div className="mt-1 flex items-baseline gap-1.5">
        <span className="num text-3xl font-semibold" style={{ color: d.sig ? (good ? C.accent : C.danger) : C.fg2 }}>
          {shown >= 0 ? '+' : '−'}<CountUp to={Math.abs(shown)} decimals={digits} duration={1.2} />
        </span>
        <span className="text-sm text-fg-2">{unit}</span>
      </div>
      <div className="mt-1 flex flex-wrap items-center gap-2 text-xs text-fg-mute">
        <Badge tone={d.sig ? 'good' : 'neutral'}>{d.sig ? t('p1r_sig') : t('p1r_nosig')}</Badge>
        <span className="num">{t('p1r_ci', { lo: fmtN(lowerBetter ? -d.hi : d.lo, digits, true), hi: fmtN(lowerBetter ? -d.lo : d.hi, digits, true) })}</span>
      </div>
    </SpotlightCard>
  )
}

export default function ResultsPage() {
  const t = useT()
  const { lang, setLang } = useLang()
  const [data, setData] = useState<Data | null>(null)
  const [err, setErr] = useState<string | null>(null)
  const [world, setWorld] = useState<World>('v2')
  const [scen, setScen] = useState('ALL')
  const [meas, setMeas] = useState('ontime')
  const [curve, setCurve] = useState('val_ontime30_per_h')
  const [sort, setSort] = useState<{ k: string; up: boolean }>({ k: 'method', up: true })
  const [tScen, setTScen] = useState('ALL')
  // practice world (the fast simulation) and CARLA tests are kept apart / 練習世界と CARLA を分ける
  const [part, setPart] = useState<'practice' | 'carla'>(() => (window.location.hash === '#carla' ? 'carla' : 'practice'))
  const choosePart = (v: string) => { setPart(v as 'practice' | 'carla'); window.location.hash = v === 'carla' ? 'carla' : '' }
  useEffect(() => {
    const on = () => setPart(window.location.hash === '#carla' ? 'carla' : 'practice')
    window.addEventListener('hashchange', on)
    return () => window.removeEventListener('hashchange', on)
  }, [])

  useEffect(() => {
    fetch('/rl/paper1/data', { cache: 'no-cache' }).then((r) => r.json()).then((d: Data) => {
      if (d.error) setErr(d.error); else setData(d)
    }).catch((e) => setErr(String(e)))
  }, [])
  useEffect(() => { document.title = t('p1r_title') }, [t])

  const M = MEAS[meas]
  const diff = (w: World, a: string, b: string, metric: string, sc = 'ALL') =>
    data?.diffs.find((d) => d.world === w && d.a === a && d.b === b && d.metric === metric && d.scenario === sc)
  const res = (w: World, m: string, seed: Row['seed'], sc: string) =>
    data?.results.find((r) => r.world === w && r.method === m && r.seed === seed && r.scenario === sc)
  const methods = useMemo(() => ORDER.filter((m) => data?.results.some((r) => r.world === world && r.method === m)), [data, world])
  const avgRow = (m: string, sc: string) => res(world, m, 'avg', sc) ?? res(world, m, null, sc)

  if (err) return <div className="p-6 text-danger">{t('p1r_error', { e: err })}</div>
  if (!data) return <div className="p-6 text-fg-mute">{t('p1r_loading')}</div>

  const scOpts = [{ value: 'ALL', label: t('p1r_all') }, ...data.scenarios.map((s, i) => ({ value: s, label: `S${i + 1} ${SLAB[i]}` }))]
  const vs0: ForestRow[] = methods.filter((m) => m !== 'B0').map((m): ForestRow | null => {
    const d = diff(world, m, 'B0', M.diff, scen)
    return d ? { key: m, label: m === 'POD' ? 'Pod' : m, color: MCOL[m], diff: d.diff, lo: d.lo, hi: d.hi, sig: d.sig, n: d.n, note: MNAME[m] } : null
  }).filter((r): r is ForestRow => r != null)
  const h2h: ForestRow[] = data.diffs.filter((d) => d.world === world && d.metric === M.diff && d.scenario === scen && d.b !== 'B0')
    .map((d) => ({ key: d.a + d.b, label: `${d.a} − ${d.b}`, color: MCOL[d.a], diff: d.diff, lo: d.lo, hi: d.hi, sig: d.sig, n: d.n }))
  const perScen = methods.map((m) => ({ key: m, label: MNAME[m], color: MCOL[m],
    values: data.scenarios.map((s) => num(avgRow(m, s)?.[M.field])) }))
  const crashRows = methods.map((m) => avgRow(m, scen)).filter((r): r is Row => !!r)
  const choiceRows = methods.map((m) => {
    const r = avgRow(m, scen)
    return { key: m, label: m === 'POD' ? 'Pod' : m, segs: [
      { key: 'pod', label: t('p1r_k_pod'), color: C.s1, value: num(r?.to_pod) ?? 0 },
      { key: 'next', label: t('p1r_k_next'), color: C.accent, value: num(r?.to_next) ?? 0 },
      { key: 'stn', label: t('p1r_k_stn'), color: C.warn, value: num(r?.to_station) ?? 0 }] }
  })
  const trainSeries = Object.entries(data.training).filter(([k]) => k.startsWith(world + '|')).map(([k, pts]) => {
    const [, m, s] = k.split('|')
    return { key: k, label: `${m} s${s}`, color: MCOL[m], dash: s !== '0',
      points: pts.filter((p) => p.steps != null && p[curve] != null).map((p) => ({ x: p.steps as number, y: p[curve] as number })) }
  }).sort((a, b) => a.label.localeCompare(b.label))
  const tuneRows = (list: Tune[] | undefined, base: string, which: 'ontime' | 'ready'): ForestRow[] => (list ?? []).map((v) => {
    const d = which === 'ontime' ? v.d_ontime : v.d_ready, h = which === 'ontime' ? v.d_ontime_ci : v.d_ready_ci
    return { key: v.version, label: v.chosen ? `★ ${v.label}` : v.label, color: v.chosen ? C.accent : C.fg2, diff: d, lo: d - h, hi: d + h,
      sig: Math.abs(d) > h, n: 70, note: `vs ${base}` }
  })
  const tableRows = data.results.filter((r) => r.world === world && (tScen === 'ALL' ? true : r.scenario === tScen))
  const COLS: { k: string; label: string; d: number }[] = [
    { k: 'delivered_per_h', label: 'Deliv/h', d: 2 }, { k: 'ontime30_per_h', label: 'On time/h', d: 2 }, { k: 'ontime15_per_h', label: '≤15 min/h', d: 2 },
    { k: 'ready_time_mean_s', label: 'Ready s', d: 0 }, { k: 'ready_time_p95_s', label: 'Ready p95', d: 0 }, { k: 'recovery_failure_rate', label: 'Fail rate', d: 3 },
    { k: 'crashes', label: 'Crash/day', d: 2 }, { k: 'crashes_per100', label: 'Crash/100fl', d: 2 }, { k: 'to_pod', label: 'Pod', d: 1 },
    { k: 'to_next', label: 'Next stop', d: 1 }, { k: 'to_station', label: 'Charger', d: 1 }, { k: 'charger_conflicts', label: 'Queues', d: 2 },
    { k: 'infeasible_picks', label: 'Impossible', d: 2 }, { k: 'pod_stranded', label: 'Pod flat', d: 3 }, { k: 'waiting_orders_end', label: 'Waiting', d: 1 },
    { k: 'wh_per_parcel', label: 'Wh/parcel', d: 0 }, { k: 'pred_ready_mae_s', label: 'Pred err s', d: 0 }, { k: 'latency_mean_ms', label: 'ms/decision', d: 0 }]
  const sorted = [...tableRows].sort((a, b) => {
    const k = sort.k
    const va = k === 'method' ? `${ORDER.indexOf(a.method)}|${a.scenario}|${a.seed}` : num(a[k]) ?? -1e9
    const vb = k === 'method' ? `${ORDER.indexOf(b.method)}|${b.scenario}|${b.seed}` : num(b[k]) ?? -1e9
    return (va < vb ? -1 : va > vb ? 1 : 0) * (sort.up ? 1 : -1)
  })
  const csv = () => {
    const head = ['world', 'method', 'seed', 'scenario', 'days', ...COLS.map((c) => c.k)]
    const lines = [head.join(','), ...sorted.map((r) => [r.world, r.method, r.seed ?? '', r.scenario, r.n, ...COLS.map((c) => num(r[c.k]) ?? '')].join(','))]
    const a = document.createElement('a')
    a.href = URL.createObjectURL(new Blob([lines.join('\n')], { type: 'text/csv' }))
    a.download = `paper1_results_${world}${tScen === 'ALL' ? '' : '_' + tScen}.csv`
    a.click()
    setTimeout(() => URL.revokeObjectURL(a.href), 1000)
  }
  const f = (d: Diff | undefined, digits: number, neg = false) => fmtN(d ? (neg ? -d.diff : d.diff) : null, digits)
  const d43 = diff('v2', 'B4', 'B3', 'ontime30_per_h'), d43r = diff('v2', 'B4', 'B3', 'ready_time_mean_s')
  const d45 = diff('v2', 'B4', 'B5', 'ontime30_per_h'), d45r = diff('v2', 'B4', 'B5', 'ready_time_mean_s')
  const d50 = diff('v2', 'B5', 'B0', 'ontime30_per_h'), d53 = diff('v2', 'B5', 'B3', 'ontime30_per_h')
  const d30v1 = diff('v1', 'B3', 'B0', 'ontime30_per_h')
  const causeRows = data.causes ? (['crashes', 'recovery_failures'] as const).map((k) => ({ key: k, label: k === 'crashes' ? 'All crashes' : 'Recovery failures',
    segs: [{ key: 'wind', label: 'wind', color: C.info, value: data.causes![k].wind ?? 0 }, { key: 'battery', label: 'battery ran out', color: C.warn, value: data.causes![k].battery ?? 0 },
      { key: 'collision', label: 'collision', color: C.danger, value: data.causes![k].collision ?? 0 }] })) : []

  return (
    <div className="min-h-screen bg-bg">
      <header className="relative overflow-hidden border-b border-line">
        <div className="pointer-events-none absolute inset-0 opacity-40" aria-hidden>
          <Aurora colorStops={[C.info, C.accent, C.violet]} amplitude={0.8} blend={0.6} speed={0.4} />
        </div>
        <div className="relative mx-auto flex max-w-[1400px] flex-col gap-3 px-4 py-6 md:px-6 md:py-8">
          <div className="flex flex-wrap items-center justify-between gap-3">
            <a href="/" className="text-xs text-fg-2 hover:text-fg">← {t('p1r_back')}</a>
            <div className="flex items-center gap-3 text-xs text-fg-mute">
              <span className="num">{t('p1r_files', { n: data.files, t: new Date(data.updated * 1000).toLocaleString() })}</span>
              <Segmented size="sm" label="Language" value={lang} onChange={(v) => setLang(v as 'en' | 'ja')} options={[{ value: 'en', label: 'EN' }, { value: 'ja', label: '日本語' }]} />
            </div>
          </div>
          <h1 className="text-3xl font-semibold md:text-4xl">
            <GradientText colors={[C.accent, C.cyan, C.violet, C.accent]} animationSpeed={10}>{t('p1r_title')}</GradientText>
          </h1>
          <p className="max-w-[80ch] text-sm text-fg-2">{t('p1r_sub')}</p>
          {data.missing.length > 0 && <Badge tone="warn">{t('p1r_missing', { f: data.missing.join(', ') })}</Badge>}
          <Segmented label={t('p1r_part')} value={part} onChange={choosePart}
            options={[{ value: 'practice', label: t('p1r_part_practice') }, { value: 'carla', label: t('p1r_part_carla') }]} />
          <p className="max-w-[90ch] text-xs text-fg-mute">{t(part === 'carla' ? 'p1r_part_carla_n' : 'p1r_part_practice_n')}</p>
          {part === 'practice' && <div className="grid gap-3 sm:grid-cols-2 xl:grid-cols-4">
            <Headline label={t('p1r_h_b4b3')} d={d43} unit={t('p1r_per_h')} digits={2} />
            <Headline label={t('p1r_h_b4b3r')} d={d43r} unit={t('p1r_sec')} digits={0} lowerBetter />
            <Headline label={t('p1r_h_b4b5')} d={d45} unit={t('p1r_per_h')} digits={2} />
            <Headline label={t('p1r_h_b4b0')} d={diff('v2', 'B4', 'B0', 'ontime30_per_h')} unit={t('p1r_per_h')} digits={2} />
          </div>}
        </div>
      </header>

      <main className="mx-auto flex max-w-[1400px] flex-col gap-4 px-4 py-5 md:px-6">
        {part === 'carla' && <>
        {data.carla?.length > 0 ? <Section id="carla" title={t('p1r_s_carla')} note={t('p1r_s_carla_n')}>
          <div className="overflow-x-auto">
            <table className="num w-full border-collapse text-xs">
              <thead><tr className="text-fg-mute">{['p1r_cr_test', 'p1r_cr_method', 'p1r_cr_deliv', 'p1r_cr_drone', 'p1r_cr_failed', 'p1r_cr_crash', 'p1r_cr_charge', 'p1r_cr_wait', 'p1r_cr_ready'].map((k, i) =>
                <th key={k} className={`border-b border-line px-2 py-1.5 font-medium ${i > 1 ? 'text-right' : 'text-left'}`}>{t(k)}</th>)}</tr></thead>
              <tbody>{data.carla.flatMap((c) => Object.entries(c.runs).map(([m, r]) => {
                const isB4 = m === 'B4'
                const p = r.paper1 as Record<string, number> | null | undefined
                return <tr key={c.file + m} className={`hover:bg-muted/60 ${isB4 ? '' : 'text-fg-mute'}`}>
                  <td className="border-b border-line/50 px-2 py-1.5 font-sans">{t(c.hardware === 'typical' ? 'p1r_cr_typ' : 'p1r_cr_carla')}</td>
                  <td className="border-b border-line/50 px-2 py-1.5 font-sans font-semibold">
                    {isB4 ? <><span className="mr-1.5 inline-block h-2 w-2 rounded-full" style={{ background: MCOL[c.b4] }} />{MNAME[c.b4]}</> : t('p1r_cr_code')}
                    {isB4 && p && <span className="ml-2 font-normal text-fg-mute">({t('p1r_k_pod')} {p.pod} · {t('p1r_k_next')} {p.pod_next} · {t('p1r_k_stn')} {p.station})</span>}</td>
                  {[fmtN(num(r.delivered_per_h), 1), fmtN(num(r.drone_per_h), 1), String(r.failed ?? '–'), String(r.crashes ?? '–'), String(r.charge_trips ?? '–'),
                    `${fmtN(num(r.mean_wait_s), 0)} s`, num(r.ready_mean_s) != null ? `${fmtN(num(r.ready_mean_s), 0)} s (${r.ready_n})` : '–'].map((v, i) =>
                    <td key={i} className="border-b border-line/50 px-2 py-1.5 text-right">{v}</td>)}
                </tr>
              }))}</tbody>
            </table>
          </div>
          <div className="text-xs font-semibold text-fg-2">{t('p1r_cr_dwait')}</div>
          <Bars rows={data.carla.map((c) => {
            const k = Object.keys(c.runs)
            const b4 = c.runs.B4, code = c.runs[k.find((x) => x !== 'B4') ?? 'off']
            return { key: c.file, label: `${t(c.hardware === 'typical' ? 'p1r_cr_typ' : 'p1r_cr_carla')} · ${c.b4}`, segs: [
              { key: 'd', label: t('p1r_cr_dwait'), color: (num(b4?.mean_wait_s) ?? 0) < (num(code?.mean_wait_s) ?? 0) ? C.accent : C.danger,
                value: Math.abs((num(code?.mean_wait_s) ?? 0) - (num(b4?.mean_wait_s) ?? 0)) }] }
          })} unit="s" digits={0} aria="order waiting time saved by B4 in CARLA" labelWidth={210} />
          <p className="text-[11px] text-fg-mute">{t('p1r_cr_note')}</p>
          {data.diagnosis && <div className="rounded-xl border border-line bg-card-2 p-3 text-xs text-fg-2">
            <div className="mb-1.5 text-sm font-semibold text-fg">{t('p1r_dg_t')}</div>
            <ul className="flex list-disc flex-col gap-1 pl-4">
              <li>{t('p1r_dg_1', { n: data.diagnosis.decisions, s: data.diagnosis.live_B4conf.station ?? 0, b: data.diagnosis.drone_battery_median })}</li>
              <li>{t('p1r_dg_2', { a: data.diagnosis.pod_battery_median.charger_choices, b: data.diagnosis.pod_battery_median.pod_choices,
                lo: data.diagnosis.B4conf_if_pod_battery['30']?.station ?? '–', hi: data.diagnosis.B4conf_if_pod_battery['90']?.station ?? '–' })}</li>
              <li>{t('p1r_dg_3')}: {data.diagnosis.causes.join('; ')}.</li>
              <li>{t('p1r_dg_4')}: {data.diagnosis.ruled_out.join('; ')}.</li>
              <li className="text-accent">{t('p1r_dg_5')}: {data.diagnosis.fix}.</li>
            </ul>
          </div>}
        </Section> : <EmptyState>{t('p1r_nocarla')}</EmptyState>}
        </>}

        {part === 'practice' && <>
        <FadeContent trigger="mount" duration={0.6}>
          <Section id="findings" title={t('p1r_findings')}>
            <div className="grid gap-3 md:grid-cols-2 xl:grid-cols-4">
              {[
                { k: 'f1', c: C.accent, text: t('p1r_f1', { d: f(d43, 2), r: f(d43r, 0, true) }) },
                { k: 'f2', c: C.violet, text: t('p1r_f2', { d: f(d45, 2), r: f(d45r, 0, true) }) },
                { k: 'f3', c: C.s3, text: t('p1r_f3', { d: f(d50, 2), e: d53 ? `${fmtN(d53.diff, 2, true)} (${d53.sig ? t('p1r_sig') : t('p1r_nosig')})` : '–' }) },
                { k: 'f4', c: C.mute, text: t('p1r_f4', { d: d30v1 ? fmtN(d30v1.diff, 2, true) : '–' }) },
              ].map((x) => (
                <SpotlightCard key={x.k} className="!rounded-xl border border-line !bg-card-2 p-4" spotlightColor="rgba(148, 163, 184, 0.14)">
                  <div className="mb-1.5 flex items-center gap-2 text-sm font-semibold text-fg"><span className="h-2.5 w-2.5 rounded-full" style={{ background: x.c }} />{t(`p1r_${x.k}_t`)}</div>
                  <p className="text-xs leading-relaxed text-fg-2">{x.text}</p>
                </SpotlightCard>
              ))}
            </div>
          </Section>
        </FadeContent>

        <div className="sticky top-0 z-30 -mx-1 flex flex-wrap items-end gap-3 rounded-xl border border-line bg-bg/90 px-3 py-2 backdrop-blur-md">
          <Segmented label={t('p1r_world')} value={world} onChange={(v) => setWorld(v as World)}
            options={[{ value: 'v2', label: t('p1r_w2') }, { value: 'v1', label: t('p1r_w1') }]} />
          <div className="min-w-[200px]"><Select label={t('p1r_scen')} value={scen} options={scOpts} onChange={setScen} /></div>
          <div className="min-w-[200px]"><Select label={t('p1r_measure')} value={meas} onChange={setMeas}
            options={Object.entries(MEAS).map(([k, v]) => ({ value: k, label: t(v.key) }))} /></div>
        </div>

        <div className="grid gap-4 xl:grid-cols-2">
          <Section id="vs-b0" title={`${t('p1r_s_vs0')} · ${t(M.key)}`} note={t('p1r_s_vs0_n')}>
            <ForestPlot rows={vs0} unit={M.unit} digits={M.digits} lowerBetter={M.lowerBetter} aria={`difference from B0, ${M.field}`} />
          </Section>
          <Section id="h2h" title={`${t('p1r_s_h2h')} · ${t(M.key)}`} note={t('p1r_s_h2h_n')}>
            <ForestPlot rows={h2h} unit={M.unit} digits={M.digits} lowerBetter={M.lowerBetter} aria={`head to head, ${M.field}`} />
          </Section>
        </div>

        <Section id="per-scenario" title={`${t('p1r_s_scen')} · ${t(M.key)}`} note={t('p1r_s_scen_n')}>
          <DotPlot cats={SLAB} series={perScen} unit={M.unit} digits={M.digits} lowerBetter={M.lowerBetter} aria={`per situation, ${M.field}`} />
        </Section>

        <Section id="crashes" title={t('p1r_s_crash')} note={t('p1r_s_crash_n')}>
          <div className="grid gap-4 lg:grid-cols-2">
            <div className="overflow-x-auto">
              <table className="num w-full border-collapse text-xs">
                <thead><tr className="text-fg-mute">{['p1r_c_method', 'p1r_c_crash', 'p1r_c_c100', 'p1r_c_fail', 'p1r_c_rate', 'p1r_c_strand', 'p1r_c_wait'].map((k, i) =>
                  <th key={k} className={`border-b border-line px-2 py-1.5 font-medium ${i ? 'text-right' : 'text-left'}`}>{t(k)}</th>)}</tr></thead>
                <tbody>{crashRows.map((r) => <tr key={r.method} className="hover:bg-muted/60">
                  <td className="border-b border-line/60 px-2 py-1.5 font-sans font-semibold"><span className="mr-1.5 inline-block h-2 w-2 rounded-full" style={{ background: MCOL[r.method] }} />{MNAME[r.method]}</td>
                  {[fmtN(num(r.crashes), 2), fmtN(num(r.crashes_per100), 2), fmtN(num(r.recovery_failures), 2), `${fmtN((num(r.recovery_failure_rate) ?? NaN) * 100, 1)} %`,
                    fmtN((num(r.pod_stranded) ?? NaN) * 100, 1), fmtN(num(r.waiting_orders_end), 1)].map((v, i) =>
                    <td key={i} className="border-b border-line/60 px-2 py-1.5 text-right">{v}</td>)}</tr>)}</tbody>
              </table>
            </div>
            <div className="flex flex-col gap-2">
              <div className="text-xs font-semibold text-fg-2">{t('p1r_crash100')}</div>
              <Bars rows={crashRows.map((r) => ({ key: r.method, label: r.method === 'POD' ? 'Pod' : r.method,
                segs: [{ key: 'c', label: 'crashes / 100 flights', color: MCOL[r.method], value: num(r.crashes_per100) ?? 0 }] }))} unit="" digits={2} aria="crashes per 100 flights" labelWidth={52} />
              {causeRows.length > 0 && <>
                <div className="mt-2 text-xs font-semibold text-fg-2">{t('p1r_causes')}</div>
                <Bars rows={causeRows} unit="" digits={0} aria="crash causes for B0" labelWidth={120} />
                <p className="text-[11px] text-fg-mute">{t('p1r_causes_n')}</p>
              </>}
            </div>
          </div>
        </Section>

        <div className="grid gap-4 xl:grid-cols-2">
          <Section id="choices" title={t('p1r_s_choice')} note={t('p1r_s_choice_n')}>
            <Bars rows={choiceRows} unit="/day" digits={1} aria="recovery choices per method" labelWidth={52} />
          </Section>
          <Section id="prediction" title={t('p1r_s_pred')} note={t('p1r_s_pred_n')}>
            <div className="text-xs font-semibold text-fg-2">{t('p1r_mae')}</div>
            <Bars rows={methods.map((m) => avgRow(m, scen)).filter((r): r is Row => !!r && num(r.pred_ready_mae_s) != null).map((r) => ({ key: r.method, label: r.method === 'POD' ? 'Pod' : r.method,
              segs: [{ key: 'e', label: 'mean error', color: MCOL[r.method], value: num(r.pred_ready_mae_s) ?? 0 }] }))} unit="s" digits={0} aria="prediction error" labelWidth={52} />
            <div className="text-xs font-semibold text-fg-2">{t('p1r_lat')}</div>
            <Bars rows={methods.map((m) => avgRow(m, scen)).filter((r): r is Row => !!r && num(r.latency_mean_ms) != null).map((r) => ({ key: r.method, label: r.method === 'POD' ? 'Pod' : r.method,
              segs: [{ key: 'l', label: 'ms per decision', color: MCOL[r.method], value: num(r.latency_mean_ms) ?? 0 }] }))} unit="ms" digits={1} aria="decision time" labelWidth={52} />
          </Section>
        </div>

        <Section id="training" title={t('p1r_s_train')} note={t('p1r_s_train_n')}>
          <div className="max-w-[260px]"><Select label={t('p1r_tm')} value={curve} onChange={setCurve} options={[
            { value: 'val_ontime30_per_h', label: t('p1r_t_ontime') }, { value: 'val_ready_mean_s', label: t('p1r_t_ready') },
            { value: 'ep_reward', label: t('p1r_t_reward') }, { value: 'val_infeasible_picks', label: t('p1r_t_inf') }]} /></div>
          <LineChart series={trainSeries} xLabel={t('p1r_steps')} unit={curve === 'val_ready_mean_s' ? 's' : ''} digits={curve === 'val_ready_mean_s' ? 0 : 2}
            aria="training curves" xFmt={(v) => `${Math.round(v / 1000)}k`} />
        </Section>

        {data.tuning && <Section id="tuning" title={t('p1r_s_tune')} note={t('p1r_s_tune_n')}>
          <div className="grid gap-4 lg:grid-cols-2">
            {(['B4', 'B5'] as const).map((b) => <div key={b} className="flex flex-col gap-3">
              <div className="text-sm font-semibold" style={{ color: MCOL[b] }}>{b} · {t('p1r_m_ontime')}</div>
              <ForestPlot rows={tuneRows(data.tuning![b], b === 'B4' ? 'B2' : 'B0', 'ontime')} unit="/h" digits={2} aria={`${b} tuning, on time`} labelWidth={230} />
              <div className="text-sm font-semibold" style={{ color: MCOL[b] }}>{b} · {t('p1r_m_ready')}</div>
              <ForestPlot rows={tuneRows(data.tuning![b], b === 'B4' ? 'B2' : 'B0', 'ready')} unit="s" digits={0} lowerBetter aria={`${b} tuning, ready`} labelWidth={230} />
            </div>)}
          </div>
        </Section>}

        {data.tuning2 && <Section id="tuning2" title={t('p1r_s_tune2')} note={t('p1r_s_tune2_n')}>
          <div className="grid gap-4 lg:grid-cols-2">
            <div className="flex flex-col gap-2"><div className="text-sm font-semibold" style={{ color: MCOL.B4 }}>{t('p1r_m_ontime')}</div>
              <ForestPlot rows={tuneRows(data.tuning2.B4, 'B2', 'ontime')} unit="/h" digits={2} aria="revised B4 tuning, on time" labelWidth={290} /></div>
            <div className="flex flex-col gap-2"><div className="text-sm font-semibold" style={{ color: MCOL.B4 }}>{t('p1r_m_ready')}</div>
              <ForestPlot rows={tuneRows(data.tuning2.B4, 'B2', 'ready')} unit="s" digits={0} lowerBetter aria="revised B4 tuning, ready" labelWidth={290} /></div>
          </div>
          <div className="flex flex-wrap gap-2 text-xs">{data.tuning2.B4.map((v) => <Badge key={v.version} tone={v.carla_chargers_of_22 === 0 ? 'good' : v.carla_chargers_of_22 == null ? 'neutral' : 'warn'}>
            {v.label}: {v.carla_chargers_of_22 == null ? '–' : t('p1r_chg22', { n: v.carla_chargers_of_22 })}</Badge>)}</div>
        </Section>}


        <Section id="world" title={t('p1r_s_world')} note={t('p1r_s_world_n')}>
          <ol className="grid gap-3 md:grid-cols-2 xl:grid-cols-3">
            {data.checks.map((c, i) => {
              const b0 = c.summary.B0 ?? c.summary.here
              const pr = c.b0_minus_pod
              return <li key={c.id} className={`flex flex-col gap-2 rounded-xl border p-3 ${c.id === 'v2' ? 'border-accent/60 bg-accent/5' : 'border-line bg-card-2'}`}>
                <div className="flex items-baseline gap-2"><span className="num text-xs text-fg-mute">{i + 1}</span><span className="text-sm font-semibold text-fg">{c.label}</span></div>
                <div className="text-[11px] text-fg-mute">{c.detail}</div>
                {b0 && <div className="num grid grid-cols-3 gap-1 text-[11px] text-fg-2">
                  <span>deliv {fmtN(b0.delivered_per_h, 1)}/h</span><span>ready {fmtN(b0.ready_time_mean_s, 0)} s</span><span>charger {fmtN(b0.to_station, 1)}/day</span></div>}
                {pr ? <ForestPlot rows={(['ontime30_per_h', 'ready_time_mean_s'] as const).map((k): ForestRow | null => pr[k] ? { key: k, label: k === 'ontime30_per_h' ? 'on time /h' : 'ready s',
                  color: C.fg2, ...pr[k]!, note: t('p1r_b0pod') } : null).filter((r): r is ForestRow => r != null)}
                  unit="" digits={2} aria={`${c.label}: B0 minus always pod`} labelWidth={74} />
                  : c.id === 'meet' ? <div className="text-[11px] text-warn">{t('p1r_identical')}</div> : null}
              </li>
            })}
          </ol>
        </Section>

        <Section id="table" title={t('p1r_s_table')} note={t('p1r_s_table_n')}>
          <div className="flex flex-wrap items-end gap-3">
            <div className="min-w-[200px]"><Select label={t('p1r_scencol')} value={tScen} options={scOpts} onChange={setTScen} /></div>
            <button type="button" onClick={csv} className="h-9 rounded-lg border border-line px-3 text-xs text-fg-2 hover:bg-muted">{t('p1r_csv')}</button>
          </div>
          <div className="max-h-[560px] overflow-auto rounded-lg border border-line">
            <table className="num w-full border-collapse text-xs">
              <thead className="sticky top-0 bg-card-2"><tr>
                {[{ k: 'method', label: t('p1r_c_method') }, { k: 'seed', label: t('p1r_seedcol') }, { k: 'scenario', label: t('p1r_scencol') }, ...COLS].map((c) =>
                  <th key={c.k} className="cursor-pointer whitespace-nowrap border-b border-line px-2 py-1.5 text-right font-medium text-fg-mute first:text-left hover:text-fg"
                    onClick={() => setSort((s) => ({ k: c.k, up: s.k === c.k ? !s.up : true }))} aria-sort={sort.k === c.k ? (sort.up ? 'ascending' : 'descending') : 'none'}>
                    {c.label}{sort.k === c.k ? (sort.up ? ' ▲' : ' ▼') : ''}</th>)}
              </tr></thead>
              <tbody>{sorted.map((r) => <tr key={r.tag + r.scenario} className={`hover:bg-muted/60 ${r.seed === 'avg' ? 'bg-muted/30' : ''}`}>
                <td className="whitespace-nowrap border-b border-line/50 px-2 py-1 text-left font-sans"><span className="mr-1.5 inline-block h-2 w-2 rounded-full" style={{ background: MCOL[r.method] }} />{r.method}</td>
                <td className="border-b border-line/50 px-2 py-1 text-right">{r.seed === 'avg' ? t('p1r_avg') : r.seed ?? '–'}</td>
                <td className="whitespace-nowrap border-b border-line/50 px-2 py-1 text-right">{r.scenario === 'ALL' ? t('p1r_all') : r.scenario.split('_')[0]}</td>
                {COLS.map((c) => <td key={c.k} className="border-b border-line/50 px-2 py-1 text-right">{fmtN(num(r[c.k]), c.d)}</td>)}
              </tr>)}</tbody>
            </table>
          </div>
        </Section>

        </>}

        <details className="rounded-xl border border-line bg-card p-4 text-xs text-fg-2">
          <summary className="cursor-pointer text-sm font-semibold text-fg">{t('p1r_defs')}</summary>
          <dl className="mt-3 grid gap-x-4 gap-y-2 md:grid-cols-[max-content_1fr]">
            {ORDER.map((m) => <div key={m} className="contents"><dt className="font-semibold" style={{ color: MCOL[m] }}>{MNAME[m]}</dt><dd className="text-fg-mute">{t(`p1r_d_${m}`)}</dd></div>)}
          </dl>
        </details>
      </main>
    </div>
  )
}
