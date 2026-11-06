import { useState } from 'react'
import { TriangleAlert } from 'lucide-react'
import '@/i18n/vehicle'
import './vehicle.css'
import { useT } from '@/lib/i18n'
import { useTelemetry } from '@/lib/telemetry'
import { Badge, Card, EmptyState } from '@/components/ui'
import { readVeh } from './data'
import { vehState } from './logic'
import Stage from './Stage'
import Legend from './Legend'
import { ImpactLogCard, LastImpactCard, NearbyCard, SensorsCard, StatusCard } from './panels'

// The Vehicle page: the top-down vehicle visual (sensors, impacts, parking
// distances, nearby objects, turn signals) in the middle, status and impact
// cards around it. Data: the shared 5 Hz telemetry (useTelemetry).
// 車両ページ：中央に上面図、周囲に状態・衝突カード（テレメトリ 5 Hz）

export default function VehicleView() {
  const t = useT()
  const tel = useTelemetry()
  const [selSensor, setSelSensor] = useState(-1)
  const [selHit, setSelHit] = useState<string | null>(null)

  if (!tel) return <EmptyState>{t('vx_waiting')}</EmptyState>
  const v = readVeh(tel)
  const { st, why } = vehState(v)
  const sel = selSensor < v.sensors.length ? selSensor : -1
  const toggleSensor = (i: number) => setSelSensor((s) => (s === i ? -1 : i))

  return (
    <div className="flex flex-col gap-4">
      {!v.hero && (
        <div className="flex items-center gap-2 rounded-lg border border-warn/40 bg-warn/10 px-3 py-2 text-sm text-warn" role="status">
          <TriangleAlert className="h-4 w-4 shrink-0" aria-hidden />{t('vx_no_hero')}
        </div>
      )}
      <div className="grid items-start gap-4 md:grid-cols-2 xl:grid-cols-[minmax(280px,1fr)_minmax(400px,1.7fr)_minmax(280px,1fr)]">
        {/* centrepiece first in the DOM: on phones and tablets it leads */}
        <Card title={t('veh_diagram')} className="@container md:col-span-2 xl:col-span-1 xl:col-start-2 xl:row-start-1"
          actions={<Badge tone={v.collisionCount > 0 ? 'bad' : 'neutral'} dot={v.collisionCount > 0}>{t('tile_crash')}: {v.collisionCount}</Badge>}>
          <div className="flex flex-col gap-4 @xl:flex-row @xl:items-start">
            <div className="min-w-0 flex-1">
              <div className="mx-auto" style={{ width: 'min(100%, max(340px, calc((100dvh - 180px) / 2)))' }}>
                <Stage v={v} st={st} t={t} selSensor={sel} onSensor={toggleSensor} selHit={selHit} />
              </div>
            </div>
            <div className="border-t border-line pt-3 @xl:w-60 @xl:shrink-0 @xl:border-l @xl:border-t-0 @xl:pl-4 @xl:pt-0">
              <Legend sensors={v.sensors} />
            </div>
          </div>
        </Card>
        <div className="flex min-w-0 flex-col gap-4 xl:col-start-1 xl:row-start-1">
          <StatusCard v={v} st={st} why={why} />
          <SensorsCard v={v} sel={sel} onSel={toggleSensor} />
        </div>
        <div className="flex min-w-0 flex-col gap-4 xl:col-start-3 xl:row-start-1">
          <NearbyCard v={v} />
          <LastImpactCard v={v} />
          <ImpactLogCard v={v} sel={selHit} onSel={setSelHit} />
        </div>
      </div>
    </div>
  )
}
