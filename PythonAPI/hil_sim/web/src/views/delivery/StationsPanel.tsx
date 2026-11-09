// The Charging stations card of the Delivery page: one panel per LastMile
// charging station of this run (state.stations): where it came from, its two
// bays (03 for the LMP-4 drone pods, 04 for the hubs: who is plugged in, the
// phase, charge level, power, energy this visit), the energy it has delivered,
// the queue waiting for it, its 3 Mini Pod pads and 3 drone deck pads.
// Clicking a station's header centres the City Map on it. Every field may be
// missing (older sims): what is there is shown, the rest is a dash.
// 充電ステーション一覧：ベイ・順番待ち・パッド・デッキ。見出しで地図を移動
import type { ReactNode } from 'react'
import { MapPin, Zap } from 'lucide-react'
import { useT, type TFunc } from '@/lib/i18n'
import { fmt, fmtDur } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Badge, Card, EmptyState } from '@/components/ui'
import { BAY_COL, MC, centreMap, podColor } from './shared'
import { Battery } from './widgets'
import type { DeliveryState, LiveStation, StationBay } from './types'

const num = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v)
const dash = <span className="text-fg-mute">–</span>

/** Translated phase (bays, pads, deck), else the raw word. / 状態の訳 */
function phaseText(t: TFunc, ph: string | undefined | null): string {
  if (!ph) return '–'
  const k = `dr_cs_ph_${ph}`, s = t(k)
  return s === k ? ph.replace(/_/g, ' ') : s
}
const phaseCol = (ph: string | undefined | null) => (ph ? BAY_COL[ph] ?? '#6aa8f0' : MC.car)

function Phase({ t, ph }: { t: TFunc; ph?: string | null }) {
  return (
    <span className="inline-flex items-center gap-1 text-xs text-fg-2">
      <i className="h-2 w-2 shrink-0 rounded-full ring-1 ring-black/30" style={{ background: phaseCol(ph) }} aria-hidden />
      {phaseText(t, ph)}
    </span>
  )
}

/** "V3" with its pod colour swatch. / ポッド色つきの車両名 */
function VanChip({ id }: { id: number }) {
  return (
    <span className="inline-flex items-center gap-1 num text-sm font-semibold text-fg">
      <i className="h-2.5 w-2.5 rounded-[3px] border" style={{ background: `${podColor(id)}55`, borderColor: podColor(id) }} aria-hidden />V{id}
    </span>
  )
}

function SubTitle({ children, extra }: { children: ReactNode; extra?: ReactNode }) {
  return (
    <div className="mb-1 flex items-baseline justify-between gap-2">
      <h5 className="text-[11px] font-semibold uppercase tracking-wider text-fg-mute">{children}</h5>
      {extra && <span className="text-[11px] text-fg-mute">{extra}</span>}
    </div>
  )
}

/** One bay: label, vehicle (or free), phase; level bar, kW, kWh this visit. */
function BayRow({ t, b }: { t: TFunc; b: StationBay }) {
  const free = b.van == null && (!b.phase || b.phase === 'free')
  return (
    <li className="rounded-lg border border-line/70 bg-surface/50 px-2.5 py-2">
      <div className="flex flex-wrap items-center justify-between gap-x-3 gap-y-1">
        <span className="text-xs font-semibold text-fg-2">{t(b.kind === 'lmh' ? 'dr_ch_bay_lmh' : 'dr_ch_bay_lmp')}</span>
        <span className="flex items-center gap-2.5">
          {b.van != null ? <VanChip id={b.van} /> : <span className="text-xs text-fg-mute">{t('dr_ch_free')}</span>}
          {!free && <Phase t={t} ph={b.phase} />}
        </span>
      </div>
      {!free && (
        <dl className="mt-1.5 grid grid-cols-[minmax(0,1.4fr)_minmax(0,1fr)_minmax(0,1fr)] gap-2 text-xs">
          <div className="min-w-0">
            <dt className="text-[10px] text-fg-mute">{t('dr_ch_level')}</dt>
            <dd>{num(b.level) ? <Battery value={b.level} w="w-12" label={`V${b.van ?? ''} ${t('dr_ch_level')}`} /> : dash}</dd>
          </div>
          <div className="min-w-0">
            <dt className="text-[10px] text-fg-mute">{t('dr_ch_power')}</dt>
            <dd className={cn('num', num(b.kw) && b.kw > 0 ? 'text-warn' : 'text-fg')}>{num(b.kw) ? `${fmt(b.kw, 1)} kW` : dash}</dd>
          </div>
          <div className="min-w-0">
            <dt className="text-[10px] text-fg-mute">{t('dr_ch_energy')}</dt>
            <dd className="num text-fg">{num(b.energy_kwh) ? `${fmt(b.energy_kwh, 2)} kWh` : dash}</dd>
          </div>
        </dl>
      )}
    </li>
  )
}

/** 3 pad tiles (Mini Pod pads or the drone deck). / パッド 3 枚 */
function Pads({ t, title, label, prefix, items }: {
  t: TFunc; title: string; label: (n: number) => string; prefix: string
  items: { id: number | null; level?: number | null; phase?: string }[]
}) {
  return (
    <div>
      <SubTitle>{title}</SubTitle>
      <ul className="grid grid-cols-3 gap-1.5">
        {items.map((p, i) => {
          const empty = p.id == null
          return (
            <li key={i} className={cn('flex min-w-0 flex-col items-center gap-0.5 rounded-lg border px-1 py-1.5 text-[11px]',
              empty ? 'border-line/60 bg-transparent' : 'border-line bg-card-2')}
              aria-label={`${label(i + 1)}: ${empty ? t('dr_ch_empty') : `${prefix}${p.id}, ${num(p.level) ? `${Math.round(p.level)}%, ` : ''}${phaseText(t, p.phase)}`}`}>
              <span className="text-fg-mute">{label(i + 1)}</span>
              <span className={cn('num text-sm font-semibold', empty ? 'text-fg-mute' : 'text-fg')}>{empty ? '–' : `${prefix}${p.id}`}</span>
              <span className={cn('num', !num(p.level) ? 'text-fg-mute' : p.level > 50 ? 'text-accent' : p.level > 25 ? 'text-warn' : 'text-danger')}>
                {num(p.level) ? `${Math.round(p.level)}%` : '–'}
              </span>
              {empty && (!p.phase || p.phase === 'free')
                ? <span className="text-fg-mute">{t('dr_ch_empty')}</span>
                : <span className="max-w-full truncate"><Phase t={t} ph={p.phase} /></span>}
            </li>
          )
        })}
      </ul>
    </div>
  )
}

function StationCard({ st }: { st: LiveStation }) {
  const t = useT()
  // bay 03 (LMP-4) before bay 04 (hub) / ベイ 03 を先に
  const bays = (Array.isArray(st.bays) ? st.bays : []).slice().sort((a, b) => Number(a.kind === 'lmh') - Number(b.kind === 'lmh'))
  const queue = Array.isArray(st.queue) ? st.queue : []
  const pads = Array.isArray(st.pads) ? st.pads : null
  const deck = Array.isArray(st.deck) ? st.deck : null
  const busy = bays.filter((b) => b.van != null || (b.phase && b.phase !== 'free')).length
  const kwNow = bays.reduce((a, b) => a + (num(b.kw) ? b.kw : 0), 0)
  const src = st.source === 'manual' || st.source === 'random' ? t(`dr_cs_src_${st.source}`) : st.source
  const canGo = num(st.x) && num(st.y)
  const spot = st.queue_spot
  return (
    <li className="flex min-w-0 flex-col gap-3 rounded-xl border border-line bg-card-2/50 p-3">
      <button type="button" disabled={!canGo} onClick={() => canGo && centreMap(st.x, st.y)}
        title={canGo ? t('dr_ch_show', { n: st.id }) : undefined}
        aria-label={canGo ? t('dr_ch_show', { n: st.id }) : undefined}
        className="group -m-1 flex items-start justify-between gap-2 rounded-lg p-1 text-left transition-colors hover:bg-muted/50 disabled:hover:bg-transparent">
        <span className="min-w-0">
          <span className="flex flex-wrap items-center gap-2">
            <Zap className="h-4 w-4 shrink-0" style={{ color: MC.chargeLive }} aria-hidden />
            <span className="text-sm font-semibold text-fg">{t('dr_ch_station', { n: st.id })}</span>
            {src && <Badge tone={st.source === 'manual' ? 'info' : 'neutral'}>{src}</Badge>}
          </span>
          <span className="mt-1 block text-xs text-fg-mute num">
            {bays.length ? t('dr_ch_bays_busy', { busy, total: bays.length }) : ''}
            {bays.some((b) => num(b.kw)) ? ` · ${t('dr_ch_now', { kw: fmt(kwNow, 1) })}` : ''}
          </span>
        </span>
        <span className="flex shrink-0 flex-col items-end gap-1">
          <span className="num text-sm font-semibold" style={{ color: MC.chargeLive }}>
            {num(st.energy_kwh) ? `${fmt(st.energy_kwh, 1)} kWh` : '– kWh'}
          </span>
          <span className="text-[10px] text-fg-mute">{t('dr_ch_total')}</span>
          {canGo && <MapPin className="h-3.5 w-3.5 text-fg-mute transition-colors group-hover:text-accent" aria-hidden />}
        </span>
      </button>

      {bays.length > 0 && (
        <div>
          <SubTitle>{t('dr_ch_bays')}</SubTitle>
          <ul className="flex flex-col gap-1.5">{bays.map((b, i) => <BayRow key={`${b.kind}-${i}`} t={t} b={b} />)}</ul>
        </div>
      )}

      {(Array.isArray(st.queue) || spot) && (
        <div>
          <SubTitle extra={spot ? <>{t('dr_ch_spot')}: {spot.van != null ? `V${spot.van}` : t('dr_ch_spot_free')}</> : undefined}>
            {t('dr_ch_queue')}{queue.length ? <span className="num"> ({queue.length})</span> : null}
          </SubTitle>
          {!queue.length ? <p className="text-xs text-fg-mute">{t('dr_ch_queue_none')}</p> : (
            <ol className="flex flex-col gap-1">
              {queue.map((q, i) => (
                <li key={`${q.van}-${i}`} className="flex items-center justify-between gap-2 rounded-md bg-surface/50 px-2 py-1 text-xs">
                  <span className="flex items-center gap-2">
                    <span className="num w-4 text-fg-mute">{i + 1}.</span>
                    <VanChip id={q.van} />
                    {q.kind && <span className="text-fg-mute">{t(q.kind === 'lmh' ? 'dr_ch_kind_lmh' : 'dr_ch_kind_lmp')}</span>}
                  </span>
                  <span className={cn('num', num(q.wait_s) && q.wait_s > 120 ? 'text-warn' : 'text-fg-2')}>
                    {num(q.wait_s) ? t('dr_ch_waited', { t: fmtDur(q.wait_s) }) : '–'}
                  </span>
                </li>
              ))}
            </ol>
          )}
        </div>
      )}

      {(pads || deck) && (
        <div className="grid gap-3 sm:grid-cols-2">
          {pads && <Pads t={t} title={t('dr_ch_pads')} prefix="M" label={(n) => t('dr_ch_pad_n', { n })}
            items={pads.map((p) => ({ id: p.pod, level: p.level, phase: p.phase }))} />}
          {deck && <Pads t={t} title={t('dr_ch_deck')} prefix="D" label={(n) => t('dr_ch_deck_n', { n })}
            items={deck.map((p) => ({ id: p.drone, level: p.level, phase: p.phase }))} />}
        </div>
      )}
    </li>
  )
}

export function StationsPanel({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const list = state?.available && Array.isArray(state.stations) ? state.stations.slice() : []
  list.sort((a, b) => String(a.id).localeCompare(String(b.id), undefined, { numeric: true }))
  const total = list.reduce((a, s) => a + (num(s.energy_kwh) ? s.energy_kwh : 0), 0)
  const waiting = list.reduce((a, s) => a + (Array.isArray(s.queue) ? s.queue.length : 0), 0)
  return (
    <Card title={<span className="flex items-center gap-1.5"><Zap className="h-3.5 w-3.5" style={{ color: MC.chargeLive }} aria-hidden />{t('dr_ch_title')}</span>}
      actions={list.length ? <>
        {state && !state.running && <Badge tone="warn">{t('dr_ch_stopped')}</Badge>}
        {waiting > 0 && <Badge tone="warn" className="num">{t('dr_ch_waiting_n', { n: waiting })}</Badge>}
        <Badge className="num">{t('dr_ch_kwh_all', { kwh: fmt(total, 1) })}</Badge>
        <Badge className="num">{list.length}</Badge>
      </> : undefined}>
      {!state?.available ? <EmptyState>{t('dr_waiting')}</EmptyState>
        : !list.length ? <EmptyState>{t('dr_ch_none')}</EmptyState> : (
          <>
            <ul className="grid items-start gap-3 md:grid-cols-2 2xl:grid-cols-3">
              {list.map((st) => <StationCard key={String(st.id)} st={st} />)}
            </ul>
            <p className="text-[11px] text-fg-mute">{t('dr_ch_hint')}</p>
          </>
        )}
    </Card>
  )
}
