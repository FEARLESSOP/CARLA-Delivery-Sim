import { Bus } from 'lucide-react'
import '@/i18n/buses'
import { useT } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { Badge, Card, EmptyState, KV } from '@/components/ui'

// Timetabled buses (/buses, bus_schedule.py via generate_traffic): the
// timetable clock, buses on the map and the next departures per Oita Station
// platform, with the data credit (CC BY 4.0). Read-only.
// 路線バス：時刻表の時刻・マップ上の台数・のりば別の次の発車・出典。

interface Departure { time: string; route: string; dest: string }
interface BusStatus {
  live?: boolean
  clock?: string
  date?: string
  day_type?: string
  clock_source?: string
  buses?: { id: number }[]
  next_departures?: Record<string, Departure[]>
  counts?: Record<string, number>
  credit?: string
}

const ORDER = ['1', '2', '3', '4', '5', '6', '7', '8', 'ueno_no_mori', 'kanamecho_highway']

export function BusCard() {
  const t = useT()
  const { data } = usePoll<BusStatus>('/buses', 3000)
  const live = Boolean(data?.live)
  const counts = data?.counts ?? {}
  const label = (p: string) =>
    p === 'ueno_no_mori' ? t('bus_ueno') : p === 'kanamecho_highway' ? t('bus_kaname')
      : p === 'kousha' ? t('bus_kousha') : t('bus_platform', { p })
  const deps = data?.next_departures ?? {}
  const plats = ORDER.filter((p) => (deps[p] ?? []).length > 0)
  return (
    <Card
      title={<span className="inline-flex items-center gap-2"><Bus className="h-3.5 w-3.5" aria-hidden />{t('bus_card')}</span>}
      actions={live ? <Badge tone="good" dot>{data?.clock ?? ''}</Badge> : undefined}
      footer={<span className="block text-[10px] leading-snug">{data?.credit ?? ''}</span>}
    >
      {!live ? <EmptyState>{t('bus_off')}</EmptyState> : (
        <div className="space-y-1.5">
          <KV k={t('bus_clock')} v={`${data?.clock ?? ''} ${t('bus_day_' + (data?.day_type ?? 'weekday'))}`} />
          <KV k={t('bus_on_map')} v={String(data?.buses?.length ?? 0)} />
          <KV k={t('bus_done')} v={`${counts.finished ?? 0} / ${(counts.stuck ?? 0) + (counts.lost ?? 0)}`}
              tone={(counts.stuck ?? 0) > 0 ? 'warn' : 'neutral'} />
          <div className="border-t border-line pt-2">
            <span className="text-xs font-medium text-fg-mute">{t('bus_next')}</span>
            {plats.length === 0 ? <p className="text-xs text-fg-mute">{t('bus_none')}</p> : (
              <ul className="mt-1 space-y-0.5 text-xs">
                {plats.map((p) => {
                  const d = (deps[p] ?? [])[0]
                  return (
                    <li key={p} className="flex min-w-0 items-baseline gap-2">
                      <span className="w-24 shrink-0 text-fg-mute">{label(p)}</span>
                      <span className="num shrink-0">{d.time}</span>
                      <span className="truncate" title={`${d.route} → ${d.dest}`}>{d.route} → {d.dest}</span>
                    </li>
                  )
                })}
              </ul>
            )}
          </div>
        </div>
      )}
    </Card>
  )
}
