import { useMemo, type CSSProperties } from 'react'
import { Activity, ArrowLeftRight, CarFront, OctagonX, TrafficCone, TriangleExclamationPoint } from 'lucide-react'
import { useT } from '@/lib/i18n'
import type { Telemetry } from '@/lib/telemetry'
import { cn } from '@/lib/utils'
import { Badge, Card, EmptyState } from '@/components/ui'
import AnimatedList, { type AnimatedListItem } from '@/components/bits/AnimatedList'
import { eventsOf, type DriveEvent } from './types'

// Driving Events: the newest-first list from /telemetry (events), shown with
// React Bits AnimatedList so a new event slides in. Items only change when the
// list does, not on every 5 Hz sample.
// 運転イベント（新しい順）。リストが変わった時だけ更新する。

const TYPE: Record<string, { cls: string; icon: typeof Activity }> = {
  collision: { cls: 'text-danger', icon: OctagonX },
  red_light: { cls: 'text-danger', icon: TrafficCone },
  hard_brake: { cls: 'text-warn', icon: TriangleExclamationPoint },
  lane: { cls: 'text-info', icon: CarFront },
  lane_change: { cls: 'text-fg-2', icon: ArrowLeftRight },
}

// The list's own theme variables mapped to the dashboard tokens.
// AnimatedList のテーマ変数をトークンに合わせる
const THEME = {
  '--al-tile': 'var(--color-card-2)', '--al-tile-edge': 'var(--color-line)', '--al-tile-highlight': 'transparent',
  '--al-tile-shadow': 'none', '--al-ink': 'var(--color-fg)', '--al-muted': 'var(--color-fg-mute)',
  '--al-well': 'var(--color-muted)', '--al-thumb': 'var(--color-line)', '--al-ring': 'var(--color-line-strong)',
} as CSSProperties

interface Row extends DriveEvent { id: string }

export function EventsCard({ s }: { s: Telemetry | null }) {
  const t = useT()
  const evs = eventsOf(s)
  const sig = JSON.stringify(evs)
  // stable ids: duplicates numbered from the oldest end, so prepending keeps keys
  // 安定キー：重複は古い側から番号付け（先頭追加でもキーが変わらない）
  const rows = useMemo<Row[]>(() => {
    const list = JSON.parse(sig) as DriveEvent[]
    const seen = new Map<string, number>()
    const out: Row[] = []
    for (let i = list.length - 1; i >= 0; i--) {
      const e = list[i], base = `${e.t}|${e.type}|${e.msg}`
      const k = (seen.get(base) ?? 0) + 1
      seen.set(base, k)
      out.unshift({ ...e, id: `${base}#${k}` })
    }
    return out
  }, [sig])
  const items = useMemo<AnimatedListItem[]>(() => rows.map((r) => ({ id: r.id, title: r.type })), [rows])

  return (
    <Card title={<span className="inline-flex items-center gap-2"><Activity className="h-3.5 w-3.5" aria-hidden />{t('dv_card_events')}</span>}
      wide actions={rows.length ? <Badge>{t('dv_events_n', { n: rows.length })}</Badge> : undefined}>
      {rows.length ? (
        <AnimatedList items={items} frame={false} width="100%" maxHeight={300} gap={6} radius={10} animation="slide"
          showGradients selectOnHover={false} style={THEME} itemClassName="!py-2 !cursor-default"
          renderItem={(_, i) => {
            const r = rows[i]
            if (!r) return null
            const ty = TYPE[r.type] ?? { cls: 'text-fg-2', icon: Activity }
            const I = ty.icon
            return (
              <div className="flex min-w-0 flex-1 items-center gap-3 text-sm">
                <I className={cn('h-4 w-4 shrink-0', ty.cls)} aria-hidden />
                <span className="num shrink-0 text-xs text-fg-mute">{r.t}</span>
                <span className={cn('shrink-0 font-semibold', ty.cls)}>{t(`dv_ev_${r.type}`)}</span>
                <span className="min-w-0 truncate text-fg-2">{r.msg}</span>
              </div>
            )
          }} />
      ) : <EmptyState>{t('dv_ev_none')}</EmptyState>}
    </Card>
  )
}
