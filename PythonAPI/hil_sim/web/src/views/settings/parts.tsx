import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import { AlertTriangle, Check, Loader2, RotateCcw } from 'lucide-react'
import { postJSON } from '@/lib/api'
import { useLang, useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Slider, Toggle } from '@/components/ui'
import type { SchemaParam } from './types'

// Pieces shared by the settings sections (local, the kit lacks them):
// live-save hook + badge, a schema slider row with "reset to default",
// level meters (signed and unsigned).
// 設定ページの共通部品（保存状態、既定値に戻せるスライダー、メーター）

/* ----------------------------------------------------------- saving */
export type SaveState = 'idle' | 'pending' | 'saving' | 'saved' | 'error'

/** Debounced POSTs to a settings route. ``schedule`` (re)starts the timer;
 *  the body is built when it fires, so it carries every change made while
 *  waiting. A change still waiting when the section closes is sent then.
 *  ``busy()`` tells pollers not to overwrite what the user is editing.
 *  デバウンス付きの保存。編集中はポーリングで上書きしない。 */
export function useSaver<R>(url: string, apply: (r: R) => void) {
  const [state, setState] = useState<SaveState>('idle')
  const [error, setError] = useState<string | null>(null)
  const timer = useRef<number | null>(null)
  const make = useRef<(() => unknown) | null>(null)
  const inflight = useRef(0)
  const lastEdit = useRef(0)
  const applyRef = useRef(apply)
  applyRef.current = apply

  const post = useCallback(async (body: unknown): Promise<R | null> => {
    inflight.current++
    setState('saving')
    try {
      const r = await postJSON<R>(url, body)
      applyRef.current(r)
      setState('saved'); setError(null)
      return r
    } catch (e) {
      setState('error'); setError(e instanceof Error ? e.message : String(e))
      return null
    } finally {
      inflight.current--
    }
  }, [url])

  const flush = useCallback(() => {
    if (timer.current === null) return
    window.clearTimeout(timer.current)
    timer.current = null
    const m = make.current
    make.current = null
    if (m) void post(m())
  }, [post])

  const schedule = useCallback((body: () => unknown, ms: number) => {
    lastEdit.current = Date.now()
    make.current = body
    setState('pending')
    if (timer.current !== null) window.clearTimeout(timer.current)
    timer.current = window.setTimeout(flush, ms)
  }, [flush])

  /** Drop a waiting change (its values go out with the next explicit post). */
  const cancel = useCallback(() => {
    if (timer.current !== null) window.clearTimeout(timer.current)
    timer.current = null
    make.current = null
  }, [])

  /** the user is moving a control (or a change waits to be sent) */
  const editing = useCallback(() => timer.current !== null || Date.now() - lastEdit.current < 700, [])
  /** editing, or a POST is on its way: a poll answer may be stale */
  const busy = useCallback(() => editing() || inflight.current > 0, [editing])

  useEffect(() => () => flush(), [flush])
  return { state, error, post, schedule, cancel, busy, editing }
}

export function SaveBadge({ state, error }: { state: SaveState; error?: string | null }) {
  const t = useT()
  if (state === 'idle') return <Badge tone="neutral" title={t('set_note')}><Check className="h-3 w-3" aria-hidden />{t('s_live_saved')}</Badge>
  if (state === 'pending') return <Badge tone="warn" dot>{t('s_unsaved')}</Badge>
  if (state === 'saving') return <Badge tone="info"><Loader2 className="h-3 w-3 animate-spin" aria-hidden />{t('s_saving')}</Badge>
  if (state === 'saved') return <Badge tone="good"><Check className="h-3 w-3" aria-hidden />{t('s_saved')}</Badge>
  return <Badge tone="bad" title={error ?? undefined}><AlertTriangle className="h-3 w-3" aria-hidden />{t('s_failed')}</Badge>
}

/* ---------------------------------------------------- schema slider */
/** Decimals for a step (0.05 → 2, 5 → 0). / 刻みに合った小数桁 */
export const stepDecimals = (step: number) =>
  step >= 1 ? 0 : Math.min(4, Math.max(0, -Math.floor(Math.log10(step))))

/** "Torque at 100 % (Nm)" → ["Torque at 100 %", "Nm"]: a short trailing
 *  bracket becomes the slider's unit. / 末尾の (単位) を単位表示に */
export function splitUnit(label: string): [string, string | undefined] {
  const m = /^(.*\S)\s*[([［（]([^()[\]［］（）]{1,8})[)\]］）]\s*$/.exec(label)
  const u = m?.[2].trim() ?? ''
  if (!m || !/^(Nm|N|m|cm|deg|度|°|km\/h|m\/s²?|ms|s|\/s|%|kg|Hz)$/.test(u)) return [label, undefined]
  return [m[1], u === 'deg' || u === '度' ? '°' : u]
}

export function useParamLabel() {
  const { lang } = useLang()
  return (p: { en: string; ja?: string }) => (lang === 'ja' && p.ja) || p.en
}

/** A slider (or a switch for 0/1 params) with its value, unit, help and a
 *  reset-to-default button that lights up when the value differs.
 *  既定値と違うときに光る「既定に戻す」ボタン付きスライダー */
export function ParamRow({ label, help, value, def, min, max, step, unit, format, onChange }: {
  label: string; help?: ReactNode; value: number; def: number | undefined; min: number; max: number; step: number
  unit?: string; format?: (v: number) => string; onChange: (v: number) => void
}) {
  const t = useT()
  const changed = def !== undefined && Math.abs(value - def) > step / 2
  const fmtV = format ?? ((v: number) => Number(v).toFixed(stepDecimals(step)))
  const binary = step === 1 && min === 0 && max === 1
  const L = (
    <span className="inline-flex items-center gap-1.5">
      {changed && <i className="h-1.5 w-1.5 shrink-0 rounded-full bg-accent" title={t('s_changed')} aria-label={t('s_changed')} />}
      <span className={cn(changed && 'text-accent')}>{label}</span>
    </span>
  )
  return (
    <div className="flex items-start gap-1.5">
      <div className="min-w-0 flex-1">
        {binary
          ? <Toggle label={L} help={help} checked={value >= 0.5} onChange={(v) => onChange(v ? 1 : 0)} />
          : <Slider label={L} value={value} min={min} max={max} step={step} unit={unit} format={fmtV} help={help} onChange={onChange} />}
      </div>
      <button type="button" disabled={!changed || def === undefined}
        onClick={() => def !== undefined && onChange(def)}
        title={def !== undefined ? `${t('s_reset_one')} (${t('s_default', { v: binary ? String(def) : fmtV(def) })})` : undefined}
        aria-label={`${t('s_reset_one')}: ${label}`}
        className={cn('mt-1 grid h-7 w-7 shrink-0 place-items-center rounded-md border transition-colors duration-150',
          changed ? 'border-accent/40 text-accent hover:bg-accent/15' : 'border-transparent text-fg-mute/30')}>
        <RotateCcw className="h-3.5 w-3.5" aria-hidden />
      </button>
    </div>
  )
}

/** A schema param (FFB / vehicle tuning) as a ParamRow. */
export function SchemaRow({ p, value, def, onChange, format }: {
  p: SchemaParam; value: number; def: number | undefined; onChange: (v: number) => void; format?: (v: number) => string
}) {
  const label = useParamLabel()
  const [name, unit] = splitUnit(label(p))
  return (
    <ParamRow label={name} unit={unit} help={p.help || undefined} value={value} def={def}
      min={p.min} max={p.max} step={p.step} format={format} onChange={onChange} />
  )
}

/* ------------------------------------------------------------ meters */
/** -1..1 meter that grows from the centre. / 中央から伸びるメーター */
export function SignedMeter({ value, label, text }: { value: number; label: string; text: string }) {
  const v = Math.max(-1, Math.min(1, value || 0))
  return (
    <div>
      <div className="mb-1 flex items-baseline justify-between text-sm"><span className="text-fg-2">{label}</span><b className="num font-medium text-fg">{text}</b></div>
      <div className="relative h-2 overflow-hidden rounded-full bg-muted" role="meter" aria-label={label}
        aria-valuemin={-100} aria-valuemax={100} aria-valuenow={Math.round(v * 100)}>
        <span className="absolute inset-y-0 left-1/2 w-px bg-fg-mute/60" aria-hidden />
        <span className="absolute inset-y-0 rounded-full bg-info transition-[left,width] duration-200"
          style={{ left: `${50 + Math.min(0, v) * 50}%`, width: `${Math.abs(v) * 50}%` }} />
      </div>
    </div>
  )
}

export function Meter({ value, label, text }: { value: number; label: string; text: string }) {
  const v = Math.max(0, Math.min(1, value || 0))
  return (
    <div>
      <div className="mb-1 flex items-baseline justify-between text-sm"><span className="text-fg-2">{label}</span><b className="num font-medium text-fg">{text}</b></div>
      <div className="h-2 overflow-hidden rounded-full bg-muted" role="meter" aria-label={label}
        aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(v * 100)}>
        <span className="block h-full rounded-full bg-warn transition-[width] duration-200" style={{ width: `${v * 100}%` }} />
      </div>
    </div>
  )
}

/** Small "label value" chip. / 小さなラベル付きチップ */
export function Chip({ k, v, tone }: { k: ReactNode; v: ReactNode; tone?: 'good' | 'mute' }) {
  return (
    <span className="inline-flex items-baseline gap-2 rounded-lg border border-line bg-surface/70 px-3 py-1.5 text-xs">
      <span className="text-fg-mute">{k}</span>
      <b className={cn('num text-sm font-semibold', tone === 'good' ? 'text-accent' : tone === 'mute' ? 'text-fg-mute' : 'text-fg')}>{v}</b>
    </span>
  )
}
