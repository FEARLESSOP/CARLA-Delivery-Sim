// Small widgets the Delivery tables share. / 表で共有する小部品
import type { ReactNode } from 'react'
import type { TFunc } from '@/lib/i18n'
import { Bar, Badge, type Tone } from '@/components/ui'
import { cn } from '@/lib/utils'
import { DRONE_COL, DRONE_TONE, MC, battTone, droneSt } from './shared'

/** Drone state as a badge: tone, a dot in its map ring colour, and text. */
export function DroneBadge({ t, status }: { t: TFunc; status: string }) {
  return (
    <Badge tone={DRONE_TONE[status] ?? 'neutral'}>
      <i className="h-2 w-2 rounded-full ring-1 ring-black/30" style={{ background: DRONE_COL[status] ?? MC.car }} aria-hidden />
      {droneSt(t, status)}
    </Badge>
  )
}

/** Battery as a bar plus the number. */
export function Battery({ value, label, tone, w = 'w-16' }: { value: number; label: string; tone?: Tone; w?: string }) {
  const v = Math.max(0, Math.min(100, value || 0))
  return (
    <span className="inline-flex items-center gap-2">
      <span className={cn('inline-block', w)}><Bar value={v / 100} tone={tone ?? battTone(v)} label={label} /></span>
      <span className="num w-10 text-right text-fg">{Math.round(v)}%</span>
    </span>
  )
}

/** Sticky-header table frame used by every table of the page. */
export function TableFrame({ children, maxH = 'max-h-[420px]', label }: { children: ReactNode; maxH?: string; label: string }) {
  return (
    <div className={cn('-mx-1 overflow-auto rounded-lg', maxH)} role="region" aria-label={label} tabIndex={0}>
      <table className="w-full border-collapse text-[13px]">{children}</table>
    </div>
  )
}
