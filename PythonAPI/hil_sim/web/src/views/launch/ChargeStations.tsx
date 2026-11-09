import { useId } from 'react'
import { ArrowRight, Zap } from 'lucide-react'
import type { LaunchOptions, LauncherInfo } from '@/lib/api'
import { useT } from '@/lib/i18n'
import { Badge, Segmented, Stepper } from '@/components/ui'
import { MAX_STATIONS, chargeCount, chargeMode, mapKey, type Placed } from './charge'

// The "Charging stations" card of the Launch page and of the live delivery
// controls: Random (the sim places charge_stations of them on free ground
// beside a road) or Manual (every spot placed on the Delivery page's City Map
// for this map; the count is read from /delivery/zones and only shown).
// 0 / no spots = off: the old road-side chargers are used.
// 起動ページの「充電ステーション」：ランダム（シミュが配置）か手動（地図で配置した地点）


/** ``placed``: usePlacedStations(map, manual) of the parent (it also feeds
 *  the summary) / 親が取得した配置数 */
export default function ChargeStations({ info, opts, set, map, placed }: {
  info: LauncherInfo; opts: LaunchOptions; set: (k: string, v: LaunchOptions[string]) => void; map: string; placed: Placed
}) {
  const t = useT()
  const id = useId()
  const mode = chargeMode(opts), n = chargeCount(opts)
  const manual = mode === 'manual'
  const known = info.schema.some((o) => o.key === 'charge_mode')
  const on = manual ? (placed.n ?? 0) > 0 : n > 0
  return (
    <section aria-labelledby={id} className="rounded-lg border border-line bg-surface/60 p-3">
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 id={id} className="inline-flex items-center gap-1.5 text-sm font-medium text-fg">
          <Zap className="h-3.5 w-3.5 text-accent" aria-hidden />{t('l_cs_title')}
        </h3>
        <Badge tone={on ? 'good' : 'neutral'}>{on ? t('l_cs_on') : t('l_cs_off')}</Badge>
      </div>
      <Segmented size="sm" label={t('l_cs_mode')} value={mode} onChange={(v) => set('charge_mode', v)}
        options={[{ value: 'manual', label: t('l_cs_manual') }, { value: 'random', label: t('l_cs_random') }]} />
      {manual ? (
        <div className="mt-2 space-y-1">
          <div className="flex items-center justify-between gap-3 py-1">
            <span className="min-w-0">
              <span className="block text-sm text-fg">{t('l_cs_placed')}</span>
              <span className="block text-xs text-fg-mute">{t('l_cs_placed_h', { m: mapKey(map) || '–' })}</span>
            </span>
            <span className="num rounded-lg border border-line bg-surface px-3 py-1.5 text-sm text-fg" aria-live="polite">
              {placed.n ?? '…'}<span className="text-fg-mute"> / {MAX_STATIONS}</span>
            </span>
          </div>
          {placed.n === 0 && <p className="text-xs text-warn">{t('l_cs_none')}</p>}
          <a href="#delivery/charge" className="inline-flex items-center gap-1 rounded text-xs font-medium text-accent hover:underline focus-visible:outline-2 focus-visible:outline-accent">
            {t('l_cs_link')}<ArrowRight className="h-3 w-3" aria-hidden />
          </a>
        </div>
      ) : (
        <div className="mt-1">
          <Stepper label={t('l_cs_count')} help={t('l_cs_count_h')} value={n} min={0} max={MAX_STATIONS}
            onChange={(v) => set('charge_stations', v)} />
        </div>
      )}
      <p className="mt-2 text-xs text-fg-mute">{t('l_cs_help')}</p>
      {(!known || !placed.ok) && <p className="mt-1 text-xs text-warn" role="note">{t('l_cs_old')}</p>}
    </section>
  )
}
