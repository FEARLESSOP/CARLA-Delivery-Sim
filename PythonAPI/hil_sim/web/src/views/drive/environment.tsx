import { useEffect, useState, type ReactNode } from 'react'
import { AppWindow, CloudSun, Leaf, Minus, Pause, Play, Plus, Sun } from 'lucide-react'
import { postJSON } from '@/lib/api'
import { useLang, useT } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { fmt } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import type { Telemetry } from '@/lib/telemetry'
import { Badge, Bar, Button, Card, KV, Segmented, Slider } from '@/components/ui'
import { weatherOf } from './types'

// Environment section: weather readout (telemetry), Season & wind (/season,
// seasons.py), Day & weather (/daycycle, weather_cycle.py) and Windows
// (/windows, windows_control.py).
// 環境：天候・季節と風・昼夜と天気・ウィンドウ

const Icon = ({ i: I }: { i: typeof Sun }) => <I className="h-3.5 w-3.5" aria-hidden />
const Title = ({ i, children }: { i: typeof Sun; children: string }) =>
  <span className="inline-flex items-center gap-2"><Icon i={i} />{children}</span>

/** Small group heading inside a card. / カード内の小見出し */
export function Sub({ children, right }: { children: ReactNode; right?: ReactNode }) {
  return (
    <div className="mt-1 flex items-center justify-between gap-2">
      <span className="text-xs font-medium text-fg-mute">{children}</span>
      {right}
    </div>
  )
}

/* ------------------------------------------------------------ weather */
export function WeatherCard({ s }: { s: Telemetry | null }) {
  const t = useT()
  const W = weatherOf(s)
  const row = (k: string, v: number, tone: 'info' | 'bad' | 'warn' | 'neutral') => (
    <div>
      <KV k={t(k)} v={`${Math.round(v)}%`} />
      <Bar value={v / 100} tone={tone} label={t(k)} />
    </div>
  )
  return (
    <Card title={<Title i={CloudSun}>{t('dv_card_weather')}</Title>}>
      <div className="space-y-1.5">
        {row('dv_lbl_cloud', W.cloudiness, 'neutral')}
        {row('dv_lbl_rain', W.precipitation, 'bad')}
        {row('dv_lbl_wet', W.wetness, 'info')}
        {row('dv_lbl_fog', W.fog_density, 'warn')}
      </div>
      <div className="border-t border-line pt-2">
        <KV k={t('dv_lbl_wind')} v={`${Math.round(W.wind_intensity)}%`} />
        <KV k={t('dv_lbl_sunalt')} v={`${fmt(W.sun_altitude, 0)}°`} />
      </div>
    </Card>
  )
}

/* ------------------------------------------------------------- season */
interface SeasonState {
  season: string
  by_date: string
  wind: number
  pending: boolean
  seasons: { key: string; en: string; ja: string }[]
  result: { ok: boolean; objects?: number; at?: string; error?: string } | null
}

export function SeasonCard() {
  const t = useT()
  const { lang } = useLang()
  const { data, setData } = usePoll<SeasonState>('/season', 2000)
  const post = async (body: Partial<{ season: string; wind: number }>) => {
    try { setData(await postJSON<SeasonState>('/season', body)) } catch { /* next poll shows the state */ }
  }
  const label = (k: string) => {
    const it = data?.seasons.find((x) => x.key === k)
    return it ? (lang === 'ja' ? it.ja : it.en) : k
  }
  const r = data?.result
  const msg = !data ? '' : data.pending ? t('dv_season_pending')
    : !r ? '' : r.ok ? t('dv_season_ok', { n: r.objects ?? 0, at: r.at ?? '' }) : (r.error ?? '')
  return (
    <Card title={<Title i={Leaf}>{t('dv_card_season')}</Title>}
      footer={msg && <span className={cn(r && !r.ok && !data?.pending && 'text-danger')} aria-live="polite">{msg}</span>}>
      {data ? (
        <>
          <Segmented size="sm" label={t('dv_season')} value={data.season} onChange={(k) => post({ season: k })}
            options={data.seasons.map((it) => ({ value: it.key, label: lang === 'ja' ? it.ja : it.en }))} />
          <KV k={t('dv_season_now')} v={label(data.season === 'auto' ? data.by_date : data.season)} mono={false} />
          <Slider label={t('dv_season_wind')} value={Math.round(data.wind)} min={0} max={100} step={5}
            onCommit={(v) => post({ wind: v })} />
        </>
      ) : <div className="text-sm text-fg-mute">…</div>}
    </Card>
  )
}

/* ---------------------------------------------------------- day cycle */
interface DayCycleState {
  settings: {
    day_minutes: number; day_night: boolean; dynamic: boolean; force: string; lamp_lumens: number
    lamps: 'auto' | 'on' | 'off'; paused: boolean; rain_chance: number
  }
  states: string[]
  status: {
    live?: boolean; clock?: string; hours?: number; state?: string; target?: string
    raining?: boolean; rain?: number; lamps_on?: boolean; lamps_n?: number
  } | null
}
type DayCycleBody = Partial<{
  day_night: boolean; paused: boolean; dynamic: boolean; clock: string; lamps: string
  day_minutes: number; rain_chance: number; lamp_lumens: number; force: string
}>

// day length steps (real minutes for 24 h) / 1 日の長さの段階
const DC_LENS = [1, 2, 3, 5, 10, 15, 20, 30, 45, 60, 120, 360, 1440]
const dcLenIndex = (m: number) => {
  let i = 0
  DC_LENS.forEach((v, k) => { if (Math.abs(v - m) < Math.abs(DC_LENS[i] - m)) i = k })
  return i
}
const dcHhmm = (q: number) => {
  const h = Math.floor(q / 4), mm = (q % 4) * 15
  return `${String(h).padStart(2, '0')}:${String(mm).padStart(2, '0')}`
}

export function DayCycleCard() {
  const t = useT()
  const { data, setData } = usePoll<DayCycleState>('/daycycle', 1000)
  const post = async (body: DayCycleBody) => {
    try { setData(await postJSON<DayCycleState>('/daycycle', body)) } catch { /* next poll */ }
  }
  const lenText = (m: number) => m >= 1440 ? t('dv_dc_realtime') : m >= 60 ? `${m / 60} h` : `${Math.round(m)} min`
  if (!data) return <Card title={<Title i={Sun}>{t('dv_card_daycycle')}</Title>}><div className="text-sm text-fg-mute">…</div></Card>
  const s = data.settings, st = data.status ?? {}
  const live = !!st.live
  const lbl = (k: string) => t(`dv_dc_${k}`)
  const stateText = !live ? '–' : st.target && st.target !== st.state
    ? `${lbl(st.state ?? 'clear')} ${t('dv_dc_to')} ${lbl(st.target)}` : lbl(st.state || 'clear')
  const stateTone = st.raining ? 'bad' : st.state === 'cloudy' ? 'warn' : 'good'
  const lenIdx = dcLenIndex(s.day_minutes)
  const lumK = Math.round((s.lamp_lumens || 150000) / 1000)
  return (
    <Card title={<Title i={Sun}>{t('dv_card_daycycle')}</Title>} wide
      footer={<span aria-live="polite">{live ? t('dv_dc_hint') : <span className="text-warn">{t('dv_dc_offline')}</span>}</span>}>
      {/* now / 現在 */}
      <div className="flex flex-wrap items-end gap-x-6 gap-y-2">
        <div>
          <div className="text-xs text-fg-mute">{t('dv_dc_clock')}</div>
          <div className="num text-4xl font-semibold leading-none text-fg">{st.clock || '--:--'}</div>
        </div>
        <div className="flex flex-wrap items-center gap-2 pb-0.5">
          <Badge tone={live ? stateTone : 'neutral'} dot>{stateText}</Badge>
          <Badge tone={st.raining ? 'bad' : 'good'} title={t('dv_dc_drones')}>
            {t('dv_dc_drones')}: {st.raining ? `${t('dv_dc_grounded')} (${Math.round(st.rain ?? 0)}%)` : t('dv_dc_fly')}
          </Badge>
        </div>
      </div>

      <div className="grid gap-x-6 gap-y-3 lg:grid-cols-2">
        {/* clock / 時計 */}
        <div className="space-y-2">
          <Sub>{t('dv_dc_daynight')}</Sub>
          <div className="flex flex-wrap items-center gap-2">
            <Segmented size="sm" label={t('dv_dc_daynight')} value={s.day_night ? 'on' : 'off'}
              onChange={(v) => post({ day_night: v === 'on' })}
              options={[{ value: 'on', label: t('dv_dc_on') }, { value: 'off', label: t('dv_dc_off') }]} />
            <Button size="sm" variant={s.paused ? 'primary' : 'secondary'} aria-pressed={s.paused}
              onClick={() => post({ paused: !s.paused })}>
              {s.paused ? <Play className="h-3.5 w-3.5" aria-hidden /> : <Pause className="h-3.5 w-3.5" aria-hidden />}
              {t(s.paused ? 'dv_dc_resume' : 'dv_dc_pause')}
            </Button>
          </div>
          <Slider label={t('dv_dc_settime')} value={st.hours != null ? Math.round(st.hours * 4) % 96 : 32} min={0} max={95}
            format={dcHhmm} onCommit={(v) => post({ clock: dcHhmm(v) })} />
          <div className="flex flex-wrap gap-1.5" role="group" aria-label={t('dv_dc_settime')}>
            {([['05:30', 'dawn'], ['12:00', 'noon'], ['18:00', 'dusk'], ['22:00', 'night']] as const).map(([c, k]) => (
              <Button key={k} size="sm" variant="ghost" className="border-line" onClick={() => post({ clock: c })}>
                {t(`dv_dc_${k}`)} <span className="num text-fg-mute">{c}</span>
              </Button>
            ))}
          </div>
          <div className="flex items-end gap-2">
            <Button size="sm" aria-label={t('dv_day_len_shorter')} disabled={lenIdx <= 0}
              onClick={() => post({ day_minutes: DC_LENS[Math.max(0, lenIdx - 1)] })}><Minus className="h-3.5 w-3.5" aria-hidden /></Button>
            <div className="min-w-0 flex-1">
              <Slider label={t('dv_dc_daylen')} value={lenIdx} min={0} max={DC_LENS.length - 1}
                format={(v) => lenText(DC_LENS[v])} onCommit={(v) => post({ day_minutes: DC_LENS[v] })} />
            </div>
            <Button size="sm" aria-label={t('dv_day_len_longer')} disabled={lenIdx >= DC_LENS.length - 1}
              onClick={() => post({ day_minutes: DC_LENS[Math.min(DC_LENS.length - 1, lenIdx + 1)] })}><Plus className="h-3.5 w-3.5" aria-hidden /></Button>
          </div>
        </div>

        {/* lamps + weather / 街灯と天気 */}
        <div className="space-y-2">
          <Sub right={<Badge tone={st.lamps_on ? 'warn' : 'neutral'}>
            {!live || !st.lamps_n ? '–' : `${st.lamps_on ? t('dv_dc_lit') : t('dv_dc_unlit')} · ${st.lamps_n}`}</Badge>}>
            {t('dv_dc_lamps')}
          </Sub>
          <Segmented size="sm" label={t('dv_dc_lamps')} value={s.lamps || 'auto'} onChange={(v) => post({ lamps: v })}
            options={[{ value: 'auto', label: t('dv_dc_lamps_auto') }, { value: 'on', label: t('dv_dc_on') }, { value: 'off', label: t('dv_dc_off') }]} />
          <Slider label={t('dv_dc_lampbright')} value={lumK} min={20} max={400} step={10} unit="k lm"
            onCommit={(v) => post({ lamp_lumens: v * 1000 })} />
          <Sub>{t('dv_dc_weather')}</Sub>
          <Segmented size="sm" label={t('dv_dc_weather')} value={s.force} onChange={(v) => post({ force: v })}
            options={['auto', ...data.states].map((k) => ({ value: k, label: t(`dv_dc_${k}`) }))} />
          <div>
            <Segmented size="sm" label={t('dv_dc_weather')} value={s.dynamic ? 'on' : 'off'} onChange={(v) => post({ dynamic: v === 'on' })}
              options={[{ value: 'on', label: t('dv_dc_dyn_on') }, { value: 'off', label: t('dv_dc_dyn_off') }]} />
          </div>
          <Slider label={t('dv_dc_rainchance')} value={Math.round(s.rain_chance * 100)} min={0} max={100} step={5} unit="%"
            onCommit={(v) => post({ rain_chance: v / 100 })} />
        </div>
      </div>
    </Card>
  )
}

/* ------------------------------------------------------------ windows */
interface WinInfo {
  name: string; en: string; ja: string; desc_en: string; desc_ja: string
  available: boolean; running: boolean; pid: number | null; reason: string
}
interface WindowsState { live: boolean; phase?: string; error?: string; windows: WinInfo[] }
type Pending = Record<string, { action: 'open' | 'close'; until: number }>

export function WindowsCard() {
  const t = useT()
  const { lang } = useLang()
  const { data, setData } = usePoll<WindowsState>('/windows', 1500)
  const [pending, setPending] = useState<Pending>({})
  // a request is done once the status shows it, or after 15 s
  // 状態に反映されたか 15 秒経てば応答待ちを解除
  useEffect(() => {
    if (!data) return
    const now = Date.now()
    setPending((p) => {
      const next = { ...p }
      let changed = false
      for (const w of data.windows) {
        const q = next[w.name]
        if (q && (now > q.until || (q.action === 'open') === w.running)) { delete next[w.name]; changed = true }
      }
      return changed ? next : p
    })
  }, [data])
  const post = async (name: string, action: 'open' | 'close') => {
    setPending((p) => ({ ...p, [name]: { action, until: Date.now() + 15000 } }))
    const drop = () => setPending((p) => { const n = { ...p }; delete n[name]; return n })
    try {
      const r = await fetch('/windows', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ window: name, action }),
      })
      const d = (await r.json()) as WindowsState
      if (!r.ok) drop()
      if (Array.isArray(d.windows)) setData(d)
    } catch { drop() }
  }
  const msg = !data ? '' : !data.live ? t('dv_win_offline')
    : data.phase === 'switching' ? t('dv_win_switching') : (data.error || t('dv_win_hint'))
  return (
    <Card title={<Title i={AppWindow}>{t('dv_card_windows')}</Title>}
      footer={<span className={cn(data && (!data.live || data.error) && 'text-warn')} aria-live="polite">{msg}</span>}>
      <ul className="divide-y divide-line/60">
        {(data?.windows ?? []).map((w) => {
          const p = pending[w.name]
          const name = lang === 'ja' ? w.ja : w.en
          const live = !!data?.live
          const st = p ? t(p.action === 'open' ? 'dv_win_opening' : 'dv_win_closing')
            : !live ? '' : !w.available && !w.running
              ? t('dv_win_unavail') + (w.reason.startsWith('needs --delivery') ? ` (${t('dv_win_need_delivery')})` : '')
              : w.running ? t('dv_win_running') + (w.pid ? ` · pid ${w.pid}` : '') : t('dv_win_closed')
          return (
            <li key={w.name} className="py-2.5 first:pt-0">
              <div className="flex items-center gap-2">
                <span className={cn('h-2 w-2 shrink-0 rounded-full', w.running ? 'bg-accent' : 'bg-muted ring-1 ring-line-strong')} aria-hidden />
                <div className="min-w-0 flex-1">
                  <div className="truncate text-sm font-medium text-fg">{name}</div>
                  <div className={cn('text-xs', w.running ? 'text-accent' : 'text-fg-mute')}>{st}</div>
                </div>
                <Button size="sm" variant="secondary" aria-label={`${t('dv_win_open')} ${name}`}
                  disabled={!live || !w.available || w.running || !!p} onClick={() => post(w.name, 'open')}>{t('dv_win_open')}</Button>
                <Button size="sm" variant="ghost" className="border-line" aria-label={`${t('dv_win_close')} ${name}`}
                  disabled={!live || !w.running || !!p} onClick={() => post(w.name, 'close')}>{t('dv_win_close')}</Button>
              </div>
              <p className="mt-1 pl-4 text-xs text-fg-mute">{lang === 'ja' ? w.desc_ja : w.desc_en}</p>
            </li>
          )
        })}
      </ul>
      {!data && <div className="text-sm text-fg-mute">…</div>}
    </Card>
  )
}
