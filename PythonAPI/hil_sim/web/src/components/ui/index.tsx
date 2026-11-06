// The dashboard's UI kit. Every view builds from these; no view styles its own
// buttons, sliders or cards. Tokens: src/index.css.
// ダッシュボード共通の UI 部品（各ビューはこれだけを使う）
import { useEffect, useId, useState, type ButtonHTMLAttributes, type ReactNode } from 'react'
import { cn } from '@/lib/utils'

/* ------------------------------------------------------------- layout */
export function Card({ title, actions, children, className, wide, tall, footer, id }: {
  title?: ReactNode; actions?: ReactNode; children: ReactNode; className?: string
  wide?: boolean; tall?: boolean; footer?: ReactNode; id?: string
}) {
  return (
    <section
      id={id}
      className={cn(
        'rounded-[var(--radius-card)] border border-line bg-card/80 p-4 shadow-[var(--shadow-card)] backdrop-blur-sm',
        'flex flex-col gap-3 min-w-0',
        wide && 'md:col-span-2', tall && 'row-span-2', className,
      )}
    >
      {(title || actions) && (
        <header className="flex items-center justify-between gap-2 min-h-6">
          {title && <h3 className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{title}</h3>}
          {actions && <div className="flex items-center gap-2">{actions}</div>}
        </header>
      )}
      {children}
      {footer && <footer className="text-xs text-fg-mute">{footer}</footer>}
    </section>
  )
}

/** Responsive card grid: 1 column on phones, up to 4 on wide screens. */
export function Grid({ children, className, cols = 4 }: { children: ReactNode; className?: string; cols?: 2 | 3 | 4 }) {
  const c = { 2: 'xl:grid-cols-2', 3: 'xl:grid-cols-3', 4: 'xl:grid-cols-4' }[cols]
  return <div className={cn('grid grid-cols-1 gap-4 md:grid-cols-2', c, className)}>{children}</div>
}

export function SectionTitle({ children, sub }: { children: ReactNode; sub?: ReactNode }) {
  return (
    <div className="mb-3 mt-2">
      <h2 className="text-lg font-semibold text-fg">{children}</h2>
      {sub && <p className="text-sm text-fg-mute">{sub}</p>}
    </div>
  )
}

/* ------------------------------------------------------------- values */
export type Tone = 'neutral' | 'good' | 'warn' | 'bad' | 'info' | 'accent'
const toneText: Record<Tone, string> = {
  neutral: 'text-fg', good: 'text-accent', warn: 'text-warn', bad: 'text-danger', info: 'text-info', accent: 'text-accent',
}

/** A big number with its label and unit. */
export function Stat({ label, value, unit, tone = 'neutral', sub, size = 'lg' }: {
  label: ReactNode; value: ReactNode; unit?: ReactNode; tone?: Tone; sub?: ReactNode; size?: 'md' | 'lg' | 'xl'
}) {
  const sz = { md: 'text-xl', lg: 'text-3xl', xl: 'text-5xl' }[size]
  return (
    <div className="min-w-0">
      <div className="text-xs text-fg-mute">{label}</div>
      <div className={cn('num font-semibold leading-tight', sz, toneText[tone])}>
        {value}{unit && <span className="ml-1 text-sm font-normal text-fg-mute">{unit}</span>}
      </div>
      {sub && <div className="text-xs text-fg-mute">{sub}</div>}
    </div>
  )
}

/** A label/value row. */
export function KV({ k, v, tone = 'neutral', mono = true }: { k: ReactNode; v: ReactNode; tone?: Tone; mono?: boolean }) {
  return (
    <div className="flex items-baseline justify-between gap-3 py-0.5 text-sm">
      <span className="text-fg-mute">{k}</span>
      <span className={cn(mono && 'num', 'text-right', toneText[tone])}>{v}</span>
    </div>
  )
}

/** A horizontal level bar (0..1). */
export function Bar({ value, tone = 'info', label }: { value: number; tone?: Tone; label?: string }) {
  const bg = { neutral: 'bg-fg-2', good: 'bg-accent', warn: 'bg-warn', bad: 'bg-danger', info: 'bg-info', accent: 'bg-accent' }[tone]
  const v = Math.max(0, Math.min(1, value || 0))
  return (
    <div className="h-1.5 w-full overflow-hidden rounded-full bg-muted" role="meter" aria-valuenow={Math.round(v * 100)}
      aria-valuemin={0} aria-valuemax={100} aria-label={label}>
      <div className={cn('h-full rounded-full transition-[width] duration-200', bg)} style={{ width: `${v * 100}%` }} />
    </div>
  )
}

export function Badge({ children, tone = 'neutral', dot, className, title }: {
  children: ReactNode; tone?: Tone; dot?: boolean; className?: string; title?: string
}) {
  const t = {
    neutral: 'bg-muted text-fg-2 border-line', good: 'bg-accent/15 text-accent border-accent/30',
    warn: 'bg-warn/15 text-warn border-warn/30', bad: 'bg-danger/15 text-danger border-danger/30',
    info: 'bg-info/15 text-info border-info/30', accent: 'bg-accent/15 text-accent border-accent/30',
  }[tone]
  const d = { neutral: 'bg-fg-mute', good: 'bg-accent', warn: 'bg-warn', bad: 'bg-danger', info: 'bg-info', accent: 'bg-accent' }[tone]
  return (
    <span title={title} className={cn('inline-flex items-center gap-1.5 rounded-full border px-2 py-0.5 text-xs font-medium whitespace-nowrap', t, className)}>
      {dot && <i className={cn('h-1.5 w-1.5 rounded-full', d)} />}
      {children}
    </span>
  )
}

/** Small on/off indicator (ABS, TCS, ...). */
export function Indicator({ on, children }: { on: boolean; children: ReactNode }) {
  return (
    <span className={cn('rounded-md border px-1.5 py-0.5 text-[11px] font-semibold num',
      on ? 'border-warn/50 bg-warn/15 text-warn' : 'border-line text-fg-mute/60')} aria-pressed={on}>
      {children}
    </span>
  )
}

export function EmptyState({ children }: { children: ReactNode }) {
  return <div className="rounded-lg border border-dashed border-line p-6 text-center text-sm text-fg-mute">{children}</div>
}

/* ------------------------------------------------------------- inputs */
type BtnVariant = 'primary' | 'secondary' | 'ghost' | 'danger'
export function Button({ variant = 'secondary', size = 'md', className, children, busy, ...rest }:
  ButtonHTMLAttributes<HTMLButtonElement> & { variant?: BtnVariant; size?: 'sm' | 'md' | 'lg'; busy?: boolean }) {
  const v = {
    primary: 'bg-accent text-accent-ink hover:bg-accent/90 border-accent font-semibold',
    secondary: 'bg-card-2 text-fg hover:bg-muted border-line',
    ghost: 'bg-transparent text-fg-2 hover:bg-muted border-transparent',
    danger: 'bg-danger/15 text-danger hover:bg-danger/25 border-danger/40',
  }[variant]
  const s = { sm: 'h-8 px-2.5 text-xs', md: 'h-10 px-3.5 text-sm', lg: 'h-12 px-5 text-base' }[size]
  return (
    <button
      {...rest}
      disabled={rest.disabled || busy}
      className={cn('inline-flex items-center justify-center gap-2 rounded-lg border transition-colors duration-150',
        'disabled:cursor-not-allowed disabled:opacity-50', v, s, className)}
    >
      {busy && <span className="h-3.5 w-3.5 animate-spin rounded-full border-2 border-current border-t-transparent" aria-hidden />}
      {children}
    </button>
  )
}

/** On/off switch with a visible label. */
export function Toggle({ checked, onChange, label, help, disabled }: {
  checked: boolean; onChange: (v: boolean) => void; label: ReactNode; help?: ReactNode; disabled?: boolean
}) {
  const id = useId()
  return (
    <label htmlFor={id} className={cn('flex items-start justify-between gap-3 py-1.5', disabled && 'opacity-50')}>
      <span className="min-w-0">
        <span className="block text-sm text-fg">{label}</span>
        {help && <span className="block text-xs text-fg-mute">{help}</span>}
      </span>
      <button
        id={id} type="button" role="switch" aria-checked={checked} disabled={disabled}
        onClick={() => onChange(!checked)}
        className={cn('relative mt-0.5 h-6 w-11 shrink-0 rounded-full border transition-colors duration-200',
          checked ? 'border-accent bg-accent' : 'border-line bg-muted')}
      >
        <span className={cn('absolute left-0 top-0.5 h-4.5 w-4.5 rounded-full bg-white shadow transition-transform duration-200',
          checked ? 'translate-x-5.5' : 'translate-x-0.5')} />
      </button>
    </label>
  )
}

/** Labelled slider with a typed value. ``onChange`` while dragging,
 *  ``onCommit`` once on release (send to the server there). */
export function Slider({ label, value, min, max, step = 1, unit, onChange, onCommit, help, format, disabled }: {
  label: ReactNode; value: number; min: number; max: number; step?: number; unit?: string
  onChange?: (v: number) => void; onCommit?: (v: number) => void; help?: ReactNode
  format?: (v: number) => string; disabled?: boolean
}) {
  const id = useId()
  const [v, setV] = useState(value)
  const [drag, setDrag] = useState(false)
  useEffect(() => { if (!drag) setV(value) }, [value, drag])
  const pct = ((v - min) / Math.max(1e-9, max - min)) * 100
  return (
    <div className={cn('py-1', disabled && 'opacity-50')}>
      <div className="mb-1 flex items-baseline justify-between gap-2">
        <label htmlFor={id} className="text-sm text-fg">{label}</label>
        <span className="num text-sm text-fg-2">{format ? format(v) : v}{unit && <span className="ml-0.5 text-fg-mute">{unit}</span>}</span>
      </div>
      <input
        id={id} type="range" min={min} max={max} step={step} value={v} disabled={disabled}
        onChange={(e) => { const n = Number(e.target.value); setV(n); onChange?.(n) }}
        onPointerDown={() => setDrag(true)}
        onPointerUp={() => { setDrag(false); onCommit?.(v) }}
        onKeyUp={() => onCommit?.(v)}
        className="h-1.5 w-full appearance-none rounded-full accent-[var(--color-accent)]"
        style={{ background: `linear-gradient(to right, var(--color-accent) ${pct}%, var(--color-muted) ${pct}%)` }}
      />
      {help && <div className="mt-1 text-xs text-fg-mute">{help}</div>}
    </div>
  )
}

/** Number with − / + buttons (counts like cars, pods). */
export function Stepper({ label, value, min, max, step = 1, onChange, help }: {
  label: ReactNode; value: number; min: number; max: number; step?: number; onChange: (v: number) => void; help?: ReactNode
}) {
  const set = (n: number) => onChange(Math.max(min, Math.min(max, n)))
  return (
    <div className="flex items-center justify-between gap-3 py-1">
      <span className="min-w-0">
        <span className="block text-sm text-fg">{label}</span>
        {help && <span className="block text-xs text-fg-mute">{help}</span>}
      </span>
      <div className="flex items-center rounded-lg border border-line bg-surface">
        <button type="button" aria-label="less" className="h-9 w-9 text-lg text-fg-2 hover:bg-muted rounded-l-lg disabled:opacity-40"
          disabled={value <= min} onClick={() => set(value - step)}>−</button>
        <input aria-label={typeof label === 'string' ? label : undefined} className="num h-9 w-14 bg-transparent text-center text-sm text-fg outline-none"
          value={value} inputMode="numeric"
          onChange={(e) => { const n = parseInt(e.target.value, 10); if (!isNaN(n)) set(n) }} />
        <button type="button" aria-label="more" className="h-9 w-9 text-lg text-fg-2 hover:bg-muted rounded-r-lg disabled:opacity-40"
          disabled={value >= max} onClick={() => set(value + step)}>+</button>
      </div>
    </div>
  )
}

export function Select<T extends string>({ label, value, options, onChange, help, disabled }: {
  label?: ReactNode; value: T; options: { value: T; label: string; group?: string }[]
  onChange: (v: T) => void; help?: ReactNode; disabled?: boolean
}) {
  const id = useId()
  const groups = Array.from(new Set(options.map((o) => o.group ?? '')))
  return (
    <div className="py-1">
      {label && <label htmlFor={id} className="mb-1 block text-sm text-fg">{label}</label>}
      <select id={id} value={value} disabled={disabled} onChange={(e) => onChange(e.target.value as T)}
        className="h-10 w-full rounded-lg border border-line bg-surface px-3 text-sm text-fg outline-none focus:border-accent">
        {groups.map((g) => g
          ? <optgroup key={g} label={g}>{options.filter((o) => o.group === g).map((o) => <option key={o.value} value={o.value}>{o.label}</option>)}</optgroup>
          : options.filter((o) => !o.group).map((o) => <option key={o.value} value={o.value}>{o.label}</option>))}
      </select>
      {help && <div className="mt-1 text-xs text-fg-mute">{help}</div>}
    </div>
  )
}

/** A row of mutually exclusive buttons. */
export function Segmented<T extends string>({ value, options, onChange, label, size = 'md' }: {
  value: T; options: { value: T; label: ReactNode }[]; onChange: (v: T) => void; label?: string; size?: 'sm' | 'md'
}) {
  return (
    <div role="radiogroup" aria-label={label} className="inline-flex flex-wrap gap-1 rounded-lg border border-line bg-surface p-1">
      {options.map((o) => (
        <button key={o.value} type="button" role="radio" aria-checked={o.value === value} onClick={() => onChange(o.value)}
          className={cn('rounded-md px-3 transition-colors duration-150', size === 'sm' ? 'h-7 text-xs' : 'h-8 text-sm',
            o.value === value ? 'bg-accent text-accent-ink font-semibold' : 'text-fg-2 hover:bg-muted')}>
          {o.label}
        </button>
      ))}
    </div>
  )
}

export function TextInput({ label, value, onChange, placeholder, help }: {
  label: ReactNode; value: string; onChange: (v: string) => void; placeholder?: string; help?: ReactNode
}) {
  const id = useId()
  return (
    <div className="py-1">
      <label htmlFor={id} className="mb-1 block text-sm text-fg">{label}</label>
      <input id={id} value={value} placeholder={placeholder} onChange={(e) => onChange(e.target.value)}
        className="num h-10 w-full rounded-lg border border-line bg-surface px-3 text-sm text-fg outline-none focus:border-accent" />
      {help && <div className="mt-1 text-xs text-fg-mute">{help}</div>}
    </div>
  )
}
