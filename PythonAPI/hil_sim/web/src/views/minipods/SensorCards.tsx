// The sensor cards of the selected pod, rendered from the self-describing
// sensor dict (label, value, unit, health, group, part): one section per
// group (perception, navigation, drive, power, payload, comms, then any group
// the sim adds), one card per sensor with a health stripe and badge (glyph +
// text), the value with its unit, the part it lives in, a sparkline of the
// last minute for numbers, and "Show on pod" (selects the part on the 3D pod).
// No sensor key is hard-coded here.
// センサーカード（自己記述的な辞書から描画、キーは固定しない）
import { Crosshair } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Button } from '@/components/ui'
import { HEALTH_MARK, HEALTH_TONE, groupName, isHealth, sensorGroups, sensorName, valueText, worst } from './data'
import { MiniSpark } from './history'
import { partName, sensorPart, type PartId } from './parts'
import type { Health, PodSensor } from './types'

export const cardDomId = (key: string) => `mp-card-${key.replace(/[^a-zA-Z0-9_-]/g, '_')}`

function HealthBadge({ h }: { h: Health | undefined }) {
  const t = useT()
  if (!h) return null
  return <Badge tone={HEALTH_TONE[h]}><span aria-hidden>{HEALTH_MARK[h]}</span>{t(`mp_h_${h}`)}</Badge>
}

const SPARK_CLS: Record<string, string> = {
  perception: 'stroke-s2', navigation: 'stroke-s1', drive: 'stroke-s5', power: 'stroke-accent', payload: 'stroke-s4', comms: 'stroke-s3',
}

function SensorCard({ k, s, group, hist, selected, onLocate }: {
  k: string; s: PodSensor; group: string; hist: number[]; selected: boolean; onLocate: (p: PartId) => void
}) {
  const t = useT()
  const h = isHealth(s.health) ? s.health : undefined
  const part = sensorPart(k, s)
  const name = sensorName(t, k, s)
  const id = cardDomId(k)
  const num = typeof s.value === 'number'
  return (
    <section id={id} tabIndex={-1} aria-labelledby={`${id}-h`}
      className={cn('relative flex min-w-0 scroll-mt-24 flex-col gap-1.5 overflow-hidden rounded-[var(--radius-card)] border bg-card/80 p-3.5 pt-4 shadow-[var(--shadow-card)] outline-none transition-colors duration-200',
        selected ? 'border-accent ring-2 ring-accent/40' : 'border-line',
        h === 'fail' && !selected && 'border-danger/60', h === 'warn' && !selected && 'border-warn/50')}>
      <i aria-hidden className={cn('absolute inset-x-0 top-0 h-1',
        h === 'fail' ? 'bg-danger' : h === 'warn' ? 'bg-warn' : h === 'ok' ? 'bg-accent/70' : 'bg-line')} />
      <header className="flex items-start justify-between gap-2">
        <div className="min-w-0">
          <h4 id={`${id}-h`} className="truncate text-sm font-semibold text-fg" title={name}>{name}</h4>
          <p className="truncate text-[11px] leading-snug text-fg-mute">{partName(t, part)}</p>
        </div>
        <div className="flex shrink-0 items-center gap-1">
          <HealthBadge h={h} />
          <Button variant="ghost" size="sm" className="h-7 w-7 px-0" aria-label={`${t('mp_locate')}: ${name}`}
            title={t('mp_locate')} onClick={() => onLocate(part)}>
            <Crosshair className="h-3.5 w-3.5" aria-hidden />
          </Button>
        </div>
      </header>
      <div className={cn('num font-semibold leading-tight', num ? 'text-2xl' : 'text-lg', h === 'fail' ? 'text-danger' : h === 'warn' ? 'text-warn' : 'text-fg')}>
        {valueText(t, s.value)}
        {s.unit && s.value !== null && s.value !== undefined && typeof s.value !== 'boolean' && <span className="ml-1 text-xs font-normal text-fg-mute">{s.unit}</span>}
      </div>
      {num && <MiniSpark data={hist} className={SPARK_CLS[group] ?? 'stroke-info'} />}
    </section>
  )
}

export function SensorCards({ sensors, hist, selected, onLocate }: {
  sensors: Record<string, PodSensor>; hist: (k: string) => number[]; selected: PartId | null; onLocate: (p: PartId) => void
}) {
  const t = useT()
  const groups = sensorGroups(sensors)
  if (!groups.length) return <p className="text-sm text-fg-mute">{t('mp_nosensors_short')}</p>
  return (
    <div className="flex flex-col gap-5">
      {groups.map(([g, list]) => {
        let h: Health | undefined
        for (const [, s] of list) if (isHealth(s.health)) h = worst(h, s.health)
        return (
          <section key={g} aria-labelledby={`mp-g-${g}`}>
            <div className="mb-2 flex items-center gap-2">
              <h3 id={`mp-g-${g}`} className="text-xs font-semibold uppercase tracking-[0.12em] text-fg-2">{groupName(t, g)}</h3>
              <span className="text-[11px] text-fg-mute num">{list.length}</span>
              <HealthBadge h={h} />
            </div>
            <div className="grid grid-cols-1 gap-3 sm:grid-cols-2 xl:grid-cols-4 2xl:grid-cols-5">
              {list.map(([k, s]) => (
                <SensorCard key={k} k={k} s={s} group={g} hist={hist(k)} selected={!!selected && sensorPart(k, s) === selected} onLocate={onLocate} />
              ))}
            </div>
          </section>
        )
      })}
    </div>
  )
}
