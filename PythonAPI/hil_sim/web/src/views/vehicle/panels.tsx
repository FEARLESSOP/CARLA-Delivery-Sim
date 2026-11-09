import { useEffect, useMemo, useState } from 'react'
import { ArrowBigLeft, ArrowBigRight, MapPin, OctagonAlert, ShieldCheck, TriangleAlert } from 'lucide-react'
import { getJSON, postJSON } from '@/lib/api'
import { C } from '@/lib/colors'
import { fmt } from '@/lib/fmt'
import { useLang, useT, type Lang, type TFunc } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Bar, Button, Card, Indicator, KV, Stat } from '@/components/ui'
import AnimatedList, { type AnimatedListItem } from '@/components/bits/AnimatedList'
import type { Hit, Kind, KeybindsInfo, Signal, VehData } from './data'
import { fmtAgo, fmtDist, sensDupKeys, snName, snValue, vzone, type Reason, type VehState } from './logic'
import { KindGlyph, SensorGlyph } from './Legend'

// The cards around the vehicle visual: status + turn signals, sensors,
// nearby objects, last impact and the impact log.
// 図のまわりのカード（状態・ウインカー・センサー・周辺・衝突）

/* ----------------------------------------------------------------- status */
const PILL: Record<VehState, { cls: string; key: string; Icon: typeof ShieldCheck }> = {
  ok: { cls: 'border-accent/40 bg-accent/10 text-accent', key: 'veh_ok', Icon: ShieldCheck },
  warn: { cls: 'border-warn/40 bg-warn/10 text-warn', key: 'veh_warn', Icon: TriangleAlert },
  crash: { cls: 'border-danger/60 bg-danger/15 text-danger vx-crash-pulse', key: 'veh_crash', Icon: OctagonAlert },
}

export function StatusPill({ st, why }: { st: VehState; why: Reason[] }) {
  const t = useT()
  const p = PILL[st]
  return (
    <div className="flex flex-col items-center gap-1.5" role="status" aria-live="polite">
      <span className={cn('inline-flex items-center gap-2 rounded-full border px-5 py-2 font-mono text-sm font-bold tracking-[0.14em] transition-colors duration-200', p.cls)}>
        <p.Icon className="h-4.5 w-4.5" aria-hidden />{t(p.key)}
      </span>
      {why.length > 0 && (
        <span className="text-center text-xs text-fg-mute">{why.map((r) => t(r.key, r.vars)).join(' · ')}</span>
      )}
    </div>
  )
}

// keyboard key names as the old page printed them (kbPretty)
const KB_MODS = ['ctrl', 'shift', 'alt']
const KB_PRETTY: Record<string, string> = {
  up: '↑ Up', down: '↓ Down', left: '← Left', right: '→ Right', space: 'Space', tab: 'Tab', return: 'Enter',
  enter: 'Num Enter', escape: 'Esc', backspace: 'Backspace', delete: 'Delete', insert: 'Insert', home: 'Home',
  end: 'End', 'page up': 'PgUp', 'page down': 'PgDn', 'left shift': 'L-Shift', 'right shift': 'R-Shift',
  'left ctrl': 'L-Ctrl', 'right ctrl': 'R-Ctrl', 'left alt': 'L-Alt', 'right alt': 'R-Alt', 'caps lock': 'Caps Lock',
}
function kbPretty(spec: string): string {
  const parts = spec.split('+'), mods: string[] = []
  while (parts.length > 1 && KB_MODS.includes(parts[0])) mods.push(parts.shift() as string)
  const key = parts.join('+')
  let k = KB_PRETTY[key]
  if (!k) k = key.length === 1 || /^f\d+$/.test(key) ? key.toUpperCase() : /^\[.+\]$/.test(key) ? 'Num ' + key.slice(1, -1) : key
  return KB_MODS.filter((m) => mods.includes(m)).map((m) => m[0].toUpperCase() + m.slice(1)).concat([k]).join('+')
}
/** "turn signal · L3 (L) / R3 (R)" from the shared key / wheel bindings. */
function sigHint(t: TFunc, lang: Lang, kb: KeybindsInfo | null): string {
  if (!kb) return t('sig_hint')
  const one = (id: string): string | null => {
    const c = kb.bindings[id] ?? {}
    if (c.wheel?.length) {
      const name = kb.labels[c.wheel[0]]
      return name ? (lang === 'ja' ? kb.label_ja[name] || name : name) : t('kb_unknown')
    }
    if (c.keys?.length) return kbPretty(c.keys[0])
    return null
  }
  const l = one('signal_left'), r = one('signal_right'), h = one('hazard')
  const sig = l || r ? t('sig_hint_fmt', { l: l || '-', r: r || '-' }) : t('sig_hint')
  return h ? `${sig} · ${t('haz_hint_fmt', { h })}` : sig
}

type Side = 'left' | 'right' | 'hazard'
const SIG_KEY: Record<Signal, string> = { off: 'vx_sig_off', left: 'vx_sig_left', right: 'vx_sig_right', hazard: 'vx_sig_hazard' }

/** Turn-signal buttons: POST /signal toggles the server-side state (shared
 *  with the wheel's buttons). The lit state comes back with the telemetry. */
function TurnSignals({ signal }: { signal: Signal }) {
  const t = useT()
  const { lang } = useLang()
  const [kb, setKb] = useState<KeybindsInfo | null>(null)
  const [busy, setBusy] = useState<Side | null>(null)
  const [err, setErr] = useState(false)
  useEffect(() => {
    const ctl = new AbortController()
    getJSON<KeybindsInfo>('/keybinds', ctl.signal).then(setKb).catch(() => {})
    return () => ctl.abort()
  }, [])
  const post = async (side: Side) => {
    setBusy(side)
    try { await postJSON('/signal', { side }); setErr(false) } catch { setErr(true) } finally { setBusy(null) }
  }
  const btn = (side: 'left' | 'right') => {
    const on = signal === side
    const lamp = on || signal === 'hazard'
    const Icon = side === 'left' ? ArrowBigLeft : ArrowBigRight
    return (
      <Button size="lg" aria-pressed={on} aria-label={t(side === 'left' ? 'vx_sig_l_aria' : 'vx_sig_r_aria')}
        onClick={() => post(side)} disabled={busy !== null}
        className={cn('flex-1', on && 'border-accent/60 bg-accent/10 text-accent hover:bg-accent/15', !on && lamp && 'border-warn/50 text-warn')}>
        <Icon className={cn('h-6 w-6', lamp && 'vx-blink')} fill={lamp ? 'currentColor' : 'none'} aria-hidden />
        <span className="text-sm">{t(side === 'left' ? 'vx_sig_left' : 'vx_sig_right')}</span>
      </Button>
    )
  }
  return (
    <div>
      <div className="mb-1.5 flex items-center justify-between gap-2">
        <span className="text-xs text-fg-mute">{t('vx_sig')}</span>
        <Badge tone={signal === 'off' ? 'neutral' : signal === 'hazard' ? 'warn' : 'good'} dot={signal !== 'off'}>{t(SIG_KEY[signal])}</Badge>
      </div>
      <div className="flex gap-2">
        {btn('left')}
        <Button size="lg" aria-pressed={signal === 'hazard'} aria-label={t('vx_sig_h_aria')}
          onClick={() => post('hazard')} disabled={busy !== null}
          className={cn('flex-1', signal === 'hazard' && 'border-warn/60 bg-warn/10 text-warn hover:bg-warn/15')}>
          <TriangleAlert className={cn('h-6 w-6', signal === 'hazard' && 'vx-blink')} aria-hidden />
          <span className="text-sm">{t('vx_sig_hazard')}</span>
        </Button>
        {btn('right')}
      </div>
      <p className="mt-1.5 text-center text-xs text-fg-mute">{sigHint(t, lang, kb)}</p>
      {err && <p className="mt-1 text-center text-xs text-danger" role="alert">{t('vx_sig_err')}</p>}
    </div>
  )
}

export function StatusCard({ v, st, why }: { v: VehData; st: VehState; why: Reason[] }) {
  const t = useT()
  const sa = v.steer * 35
  const gear = v.reverse ? 'R' : v.gear === 0 ? 'N' : String(v.gear)
  return (
    <Card title={t('veh_status')}>
      <StatusPill st={st} why={why} />
      <div className="flex items-end justify-between gap-3">
        <Stat label={t('lbl_speed')} value={fmt(v.speed, 0)} unit="km/h" size="xl" />
        <Stat label={t('lbl_gear')} value={gear} size="xl" tone={v.reverse ? 'warn' : 'neutral'} />
      </div>
      <div>
        <KV k={t('lbl_steer')} v={`${fmt(sa, 1)}°`} />
        <KV k={t('lbl_heading')} v={`${fmt((v.heading + 360) % 360, 0)}°`} />
        <KV k={t('lbl_stability')} v={`${fmt(v.stability * 100, 0)}%`} tone={v.stability < 0.6 ? 'warn' : 'neutral'} />
        <div className="mt-1"><Bar value={v.stability} tone={v.stability < 0.6 ? 'warn' : 'good'} label={t('lbl_stability')} /></div>
      </div>
      <div className="flex flex-wrap justify-center gap-1.5">
        {([['REV', v.reverse, 'vx_ind_rev'], ['HB', v.handBrake, 'vx_ind_hb'], ['ABS', v.abs, 'vx_ind_abs'], ['TCS', v.tcs, 'vx_ind_tcs']] as const).map(([l, on, k]) => (
          <span key={l} title={t(k)} aria-label={`${t(k)}: ${t(on ? 'vx_on' : 'vx_off')}`}><Indicator on={on}>{l}</Indicator></span>
        ))}
      </div>
      <div className="border-t border-line pt-3"><TurnSignals signal={v.signal} /></div>
    </Card>
  )
}

/* ---------------------------------------------------------------- sensors */
export function SensorsCard({ v, sel, onSel }: { v: VehData; sel: number; onSel: (i: number) => void }) {
  const t = useT()
  const dup = sensDupKeys(v.sensors)
  return (
    <Card title={t('veh_sensors')} actions={<Badge>{v.sensors.length}</Badge>} footer={v.sensors.length ? t('veh_sn_hint') : undefined}>
      {v.sensors.length ? (
        <ul className="-mx-1.5 flex flex-col">
          {v.sensors.map((x, i) => (
            <li key={i}>
              <button type="button" aria-pressed={i === sel} onClick={() => onSel(i)}
                className={cn('flex w-full items-center gap-2.5 rounded-lg px-1.5 py-1.5 text-left text-sm transition-colors duration-150',
                  i === sel ? 'bg-info/15 text-fg' : 'text-fg-2 hover:bg-muted')}>
                <SensorGlyph type={x.type} dim={!v.hero} />
                <span className="min-w-0 flex-1 truncate">{snName(t, x, v.bb, dup)}</span>
                <span className="num shrink-0 text-right text-xs text-fg">{snValue(t, v, x.type)}</span>
                {i === sel && <MapPin className="h-3.5 w-3.5 shrink-0 text-info" aria-label={t('vx_located')} />}
              </button>
            </li>
          ))}
        </ul>
      ) : <p className="text-sm text-fg-mute">{t('veh_no_sensors')}</p>}
    </Card>
  )
}

/* ----------------------------------------------------------------- nearby */
const KIND_TXT: Record<Kind, string> = { car: 'text-s2', truck: 'text-warn', bike: 'text-s5', walker: 'text-cyan' }

export function NearbyCard({ v }: { v: VehData }) {
  const t = useT()
  const list = v.near.filter((x) => x.d <= 50)
  return (
    <Card title={t('veh_nearby')} actions={<Badge tone={list.length ? 'info' : 'neutral'}>{list.length}</Badge>} footer={t('veh_range')}>
      {list.length ? (
        <ul className="flex flex-col divide-y divide-line/60">
          {list.slice(0, 10).map((x, i) => (
            <li key={i} className="flex items-center gap-2.5 py-1.5 text-sm">
              <KindGlyph kind={x.kind} size={22} />
              <span className={cn('w-16 shrink-0 text-xs font-semibold tracking-wide', KIND_TXT[x.kind])}>{t(`kind_${x.kind}`)}</span>
              <span className="num ml-auto text-fg">{fmtDist(x.d)}</span>
              <span className="num w-20 text-right text-fg-mute">{fmt(x.kmh, 0)} km/h</span>
            </li>
          ))}
        </ul>
      ) : <p className="text-sm text-fg-mute">{t('veh_none_near')}</p>}
    </Card>
  )
}

/* ---------------------------------------------------------------- impacts */
/** Mini car outline with the impact point (position = a non-colour cue). */
function ImpactGlyph({ h }: { h: Hit }) {
  return (
    <svg viewBox="0 0 20 32" width={20} height={32} aria-hidden className="shrink-0">
      <rect x={4} y={2} width={12} height={28} rx={4} fill="none" stroke={C.mute} strokeWidth={1.2} />
      <circle cx={10 + h.r * 6} cy={16 - h.f * 14} r={2.8} fill={C.danger} stroke={C.s2} strokeWidth={0.8} />
    </svg>
  )
}

export function LastImpactCard({ v }: { v: VehData }) {
  const t = useT()
  const h = v.hits.length ? v.hits[v.hits.length - 1] : null
  return (
    <Card title={t('veh_last')}>
      {h ? (
        <div className="flex items-center justify-center gap-4 py-1">
          <ImpactGlyph h={h} />
          <div className="min-w-0 text-center">
            <div className="font-mono text-2xl font-semibold text-danger">{vzone(t, h.f, h.r)}</div>
            <div className="num text-xs text-fg-mute">{fmtAgo(t, h.age)} · {t('veh_impulse')} {h.i}</div>
            {h.o && <div className="truncate text-xs text-fg-2" title={h.o}>{h.o}</div>}
          </div>
        </div>
      ) : (
        <div className="py-1 text-center">
          <div className="font-mono text-2xl text-fg-mute">–</div>
          <div className="text-xs text-fg-mute">{t('veh_none')}</div>
        </div>
      )}
    </Card>
  )
}

export function ImpactLogCard({ v, sel, onSel }: { v: VehData; sel: string | null; onSel: (k: string | null) => void }) {
  const t = useT()
  const { lang } = useLang()
  // rebuild only when the list or a whole second of age changes (not at 5 Hz)
  const sig = v.hits.map((h) => `${h.key}@${Math.floor(h.age)}`).join(';')
  const rev = useMemo(() => v.hits.slice().reverse(), [sig]) // eslint-disable-line react-hooks/exhaustive-deps
  const items = useMemo<AnimatedListItem[]>(() => rev.map((h) => ({
    id: h.key,
    icon: <ImpactGlyph h={h} />,
    title: <span className="text-danger">{vzone(t, h.f, h.r)}{h.key === sel && <MapPin className="ml-1.5 inline h-3.5 w-3.5 text-fg" aria-label={t('vx_located')} />}</span>,
    description: <span className="break-all">{h.o} · {t('veh_impulse')} {h.i}</span>,
    meta: <span className="num">{fmtAgo(t, h.age)}</span>,
  })), [rev, sel, lang, t])
  return (
    <Card title={t('veh_impacts')} actions={<Badge tone={v.hits.length ? 'bad' : 'neutral'}>{v.hits.length}</Badge>}
      footer={v.hits.length ? t('vx_impact_sel') : undefined}>
      {v.hits.length ? (
        <AnimatedList
          items={items}
          onItemSelect={(_, i) => { const k = rev[i]?.key ?? null; onSel(k === sel ? null : k) }}
          selectOnHover={false}
          animation="slide"
          frame={false}
          showGradients
          fadeSize={28}
          width="100%"
          maxHeight={340}
          radius={10}
          gap={6}
          accentColor={C.s2}
          className="-mx-1.5 !w-auto"
          itemClassName="!py-2"
          style={{
            ['--al-tile' as string]: 'var(--color-card-2)', ['--al-tile-edge' as string]: 'var(--color-line)',
            ['--al-tile-highlight' as string]: 'transparent', ['--al-tile-shadow' as string]: 'none',
            ['--al-well' as string]: 'var(--color-surface)', ['--al-ink' as string]: 'var(--color-fg)',
            ['--al-muted' as string]: 'var(--color-fg-mute)',
          }}
        />
      ) : <p className="text-sm text-fg-mute">{t('veh_none')}</p>}
    </Card>
  )
}
