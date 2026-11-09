// Charging-station launch options: helpers shared by the Launch page and its
// card (ChargeStations.tsx). / 充電ステーションの起動オプション用の部品
import type { ChargeMode, LaunchOptions } from '@/lib/api'
import type { TFunc } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import type { ZonesReply } from '@/views/delivery/types'

export const CHARGE_KEYS = ['charge_mode', 'charge_stations']
export const MAX_STATIONS = 12

export const chargeMode = (o: LaunchOptions): ChargeMode => (o.charge_mode === 'manual' ? 'manual' : 'random')
export const chargeCount = (o: LaunchOptions) => Math.max(0, Math.min(MAX_STATIONS, Math.round(Number(o.charge_stations ?? 0)) || 0))
export const mapKey = (m: string) => m.replace(/\\/g, '/').replace(/\/+$/, '').split('/').pop() || ''

/** Spots placed on the City Map for ``map`` (polled while shown); null while
 *  unknown; ``ok`` false = the dashboard predates charging stations.
 *  地図に置いた充電ステーション地点の数 */
export function usePlacedStations(map: string, enabled = true) {
  const key = mapKey(map)
  const { data } = usePoll<ZonesReply>(key ? `/delivery/zones?map=${encodeURIComponent(key)}` : null, 5000, enabled)
  const ok = !data || data.ok === false || Array.isArray(data.charge)
  const n = data && data.map === key && Array.isArray(data.charge) ? data.charge.length : data ? 0 : null
  return { n, ok }
}

/** One line for the Summary: "Random · 4", "Manual · 3", "Off". / 要約の表示 */
export function chargeSummary(t: TFunc, o: LaunchOptions, placed: number | null) {
  const mode = chargeMode(o), n = mode === 'manual' ? placed ?? 0 : chargeCount(o)
  return n > 0 ? `${t(mode === 'manual' ? 'l_cs_manual' : 'l_cs_random')} · ${n}` : t('l_cs_off_short')
}

export type Placed = ReturnType<typeof usePlacedStations>
