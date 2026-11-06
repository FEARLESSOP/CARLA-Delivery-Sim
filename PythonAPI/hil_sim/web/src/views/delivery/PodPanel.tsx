// LastMile pod showcase: take one pod over and scrub / play its whole cycle
// (POST /delivery/pod), and the drones on deck + battery magazine per pod.
// LastMile ポッド：1 台を乗っ取って全手順を再生、甲板と電池マガジン
import { useEffect, useId, useRef, useState } from 'react'
import { Hand, LogOut, Pause, Play } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Button, Card, EmptyState, KV, Select, type Tone } from '@/components/ui'
import { MC, sendCmd, TD, TDN, TH } from './shared'
import { Battery, TableFrame } from './widgets'
import type { DeliveryState, PodCmd, PodState, PodStep, Scenario } from './types'

const SPEEDS = ['0.5', '1', '2', '4'] as const
type Speed = typeof SPEEDS[number]
const podFmt = (t: number) => { const s = Math.max(0, t); return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}` }
const scenCol = (sc: Scenario) => (sc === 'drone' ? MC.pod : MC.f2f)
const toSpeed = (n: number): Speed => (SPEEDS.find((s) => Number(s) === n) ?? '1')

/** The animation timeline: a range input with the step ticks above it. */
function Scrubber({ pod, value, disabled, onInput, onCommit, label }: {
  pod: PodState; value: number; disabled: boolean; label: string
  onInput: (v: number) => void; onCommit: (v: number) => void
}) {
  const pct = (value / Math.max(1e-9, pod.total)) * 100
  return (
    <div className={cn('relative flex-1 pt-2', disabled && 'opacity-50')}>
      <div className="pointer-events-none absolute inset-x-0 top-0 h-2" aria-hidden>
        {pod.steps.map((s) => (
          <span key={s.id} className="absolute top-0 h-2 w-0.5 rounded-sm"
            style={{ left: `${(s.t0 / pod.total) * 100}%`, background: scenCol(s.scenario) }} />
        ))}
      </div>
      <input type="range" min={0} max={pod.total} step={0.05} value={value} disabled={disabled} aria-label={label}
        onChange={(e) => onInput(Number(e.target.value))}
        onPointerUp={(e) => onCommit(Number(e.currentTarget.value))}
        onKeyUp={(e) => onCommit(Number(e.currentTarget.value))}
        className="h-1.5 w-full appearance-none rounded-full accent-[var(--color-accent)] disabled:cursor-not-allowed"
        style={{ background: `linear-gradient(to right, var(--color-accent) ${pct}%, var(--color-muted) ${pct}%)` }} />
    </div>
  )
}

function StepList({ steps, current, onPick, title, scenario }: {
  steps: PodStep[]; current: string | null; onPick: (s: PodStep) => void; title: string; scenario: Scenario
}) {
  return (
    <div className="min-w-0">
      <h4 className="mb-1.5 flex items-center gap-2 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">
        <i className="h-2.5 w-2.5 rounded-sm" style={{ background: scenCol(scenario) }} aria-hidden />{title}
      </h4>
      <ol className="flex flex-col gap-0.5">
        {steps.map((s, i) => {
          const on = s.id === current
          return (
            <li key={s.id}>
              <button type="button" onClick={() => onPick(s)} aria-current={on ? 'step' : undefined}
                className={cn('grid w-full grid-cols-[1.5rem_1fr_auto] items-baseline gap-2 rounded-lg px-2 py-1.5 text-left text-sm transition-colors duration-150',
                  on ? 'bg-info/15 text-fg' : 'text-fg-2 hover:bg-muted/60')}>
                <b className={cn('num font-medium', on ? 'text-info' : 'text-fg-mute')}>{i + 1}</b>
                <span className="min-w-0">{s.title}</span>
                <span className="num text-xs text-fg-mute">{Math.round(s.t1 - s.t0)} s</span>
              </button>
            </li>
          )
        })}
      </ol>
    </div>
  )
}

export default function PodPanel({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const p = state?.pod ?? null
  const sc = p?.showcase ?? null
  const [selVan, setSelVan] = useState<string>('')
  const [speedSel, setSpeed] = useState<Speed>('1')
  // while a pod is shown the server's speed rules; else the user's pick
  // ショーケース中はサーバーの速度、それ以外は選択値
  const speed: Speed = sc ? toSpeed(sc.speed) : speedSel
  const [scrub, setScrub] = useState(0)
  const [dragging, setDragging] = useState(false)
  const [err, setErr] = useState<{ text: string; until: number } | null>(null)
  const lastSend = useRef(0)
  const speedId = useId()

  useEffect(() => {
    if (!err) return
    const id = window.setTimeout(() => setErr(null), Math.max(0, err.until - Date.now()))
    return () => window.clearTimeout(id)
  }, [err])

  const cmd = async (body: PodCmd) => {
    const r = await sendCmd('/delivery/pod', body)
    if (!r.ok) setErr({ text: r.error || 'error', until: Date.now() + 4000 })
  }

  if (!p || !p.vans.length) {
    return <Card title={t('pd_title')}><EmptyState>{t('pd_none')}</EmptyState></Card>
  }

  // the selected pod: keep the user's pick while it exists, else the showcase
  // 選択中のポッド（存在する限り維持、なければショーケース）
  const vanIds = p.vans.map((v) => String(v.id))
  const van = vanIds.includes(selVan) ? selVan : sc ? String(sc.van) : vanIds[0]
  const total = p.total
  const shown = sc ? (dragging ? scrub : sc.t) : 0
  const msg = err?.text || p.msg || (sc ? '' : t('pd_hint'))
  const pick = (s: PodStep) => {
    if (!sc) { setErr({ text: t('pd_take_first'), until: Date.now() + 4000 }); return }
    void cmd({ cmd: 'pod_time', t: s.t0 })
  }
  const slotName = (sl: number | null) => (sl == null ? t('pd_out') : sl === 1 ? t('pd_lift') : `B${sl === 0 ? 1 : sl}`)
  const deckTone = (b: number): Tone => (b < 30 ? 'bad' : b < 60 ? 'warn' : 'good')

  return (
    <div className="grid gap-4 2xl:grid-cols-[minmax(0,3fr)_minmax(0,2fr)]">
      <Card title={t('pd_anim')}
        actions={<div className="flex flex-wrap items-center justify-end gap-2">
          {sc ? <Badge tone="info" dot>{t('pd_pod')} {sc.van}</Badge> : null}
        </div>}>
        <div className="flex flex-wrap items-end gap-2">
          <div className="min-w-48 flex-1">
            <Select label={t('pd_pod')} value={van} onChange={setSelVan}
              options={p.vans.map((v) => ({ value: String(v.id), label: `${t('pd_pod')} ${v.id} · ${v.state}${v.free ? '' : ` · ${t('pd_busy')}`}` }))} />
          </div>
          <div className="py-1">
            {sc
              ? <Button onClick={() => cmd({ cmd: 'pod_release' })}><LogOut className="h-4 w-4" aria-hidden />{t('pd_release')}</Button>
              : <Button variant="primary" onClick={() => cmd({ cmd: 'pod_takeover', van: parseInt(van, 10), t: 0, speed: Number(speed) })}>
                  <Hand className="h-4 w-4" aria-hidden />{t('pd_take')}</Button>}
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-3 rounded-xl border border-line bg-surface/60 p-3">
          <Button size="md" disabled={!sc} aria-label={sc?.playing ? t('pd_pause') : t('pd_play')}
            onClick={() => sc && cmd({ cmd: 'pod_play', playing: !sc.playing, speed: Number(speed) })}>
            {sc?.playing ? <Pause className="h-4 w-4" aria-hidden /> : <Play className="h-4 w-4" aria-hidden />}
            <span className="hidden sm:inline">{sc?.playing ? t('pd_pause') : t('pd_play')}</span>
          </Button>
          <Scrubber pod={p} value={shown} disabled={!sc} label={t('pd_scrub')}
            onInput={(v) => {
              setDragging(true); setScrub(v)
              const now = Date.now()
              if (now - lastSend.current > 120) { lastSend.current = now; void cmd({ cmd: 'pod_time', t: v }) }
            }}
            onCommit={(v) => { setDragging(false); void cmd({ cmd: 'pod_time', t: v }) }} />
          <span className="num min-w-[6.5rem] text-right text-sm text-fg">{sc ? `${podFmt(shown)} / ${podFmt(total)}` : '—'}</span>
          <label htmlFor={speedId} className="sr-only">{t('pd_speed')}</label>
          <select id={speedId} value={speed} disabled={!sc}
            onChange={(e) => {
              const v = e.target.value as Speed; setSpeed(v)
              if (sc) void cmd({ cmd: 'pod_play', playing: sc.playing, speed: Number(v) })
            }}
            className="num h-10 rounded-lg border border-line bg-surface px-2 text-sm text-fg outline-none focus:border-accent disabled:opacity-50">
            {SPEEDS.map((s) => <option key={s} value={s}>{s}×</option>)}
          </select>
        </div>

        <div className="grid grid-cols-2 gap-x-6 sm:grid-cols-4">
          <KV k={t('pd_chip_shell')} v={t(`pd_shell_${p.shell}`)} mono={false} />
          <KV k={t('pd_chip_on')} v={sc ? `${t('pd_pod')} ${sc.van}` : '—'} mono={false} />
          <KV k={t('pd_chip_scen')} v={sc ? t(sc.scenario === 'drone' ? 'pd_scen_drone' : 'pd_scen_direct') : '—'} mono={false} />
          <KV k={t('pd_chip_step')} v={sc ? sc.title : '—'} mono={false} />
        </div>
        <p className={cn('min-h-5 text-sm', err ? 'text-warn' : 'text-fg-mute')} role="status" aria-live="polite">{msg}</p>

        <div className="grid gap-4 md:grid-cols-2">
          <StepList title={t('pd_scen_drone')} scenario="drone" steps={p.steps.filter((s) => s.scenario === 'drone')}
            current={sc?.step ?? null} onPick={pick} />
          <StepList title={t('pd_scen_direct')} scenario="direct" steps={p.steps.filter((s) => s.scenario === 'direct')}
            current={sc?.step ?? null} onPick={pick} />
        </div>
      </Card>

      <Card title={t('pd_deck_title')}>
        <TableFrame label={t('pd_deck_title')} maxH="max-h-[640px]">
          <thead><tr>
            <th scope="col" className={TH}>{t('pd_pod')}</th>
            <th scope="col" className={TH}>{t('pd_deck')}</th>
            <th scope="col" className={TH}>{t('pd_mag')}</th>
          </tr></thead>
          <tbody>
            {p.vans.map((v) => (
              <tr key={v.id} className="align-top hover:bg-muted/40">
                <th scope="row" className={cn(TDN, 'text-left font-semibold')}>{v.id}</th>
                <td className={cn(TD, 'whitespace-normal')}>
                  <ul className="flex flex-col gap-1">
                    {v.drones.map((d, i) => d ? (
                      <li key={i} className="flex flex-wrap items-center gap-x-2 text-xs">
                        <b className="num w-8 text-fg">D{d.id}</b>
                        <span className="w-10 text-fg-mute">{slotName(d.slot)}</span>
                        <Battery value={d.battery} tone={deckTone(d.battery)} w="w-12" label={`D${d.id} ${t('dr_th_batt')}`} />
                      </li>
                    ) : null)}
                  </ul>
                </td>
                <td className={cn(TD, 'whitespace-normal')}>
                  <div className="flex flex-wrap gap-1">
                    {v.packs.map((k, i) => k === null
                      ? <span key={i} className="num min-w-10 rounded-md border border-dashed border-line px-1 text-center text-xs text-fg-mute" aria-label={t('pd_empty')}>—</span>
                      : <span key={i} className={cn('num min-w-10 rounded-md border px-1 text-center text-xs',
                          k < 30 ? 'border-danger/40 text-danger' : k < 60 ? 'border-warn/40 text-warn' : 'border-line text-fg')}>{k}%</span>)}
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </TableFrame>
      </Card>
    </div>
  )
}
