// Shared bits of the Delivery page and its settings section: the state poll,
// commands, the colours the map and the tables share, small widgets.
// 配送ページと設定で共有する部品
import { postJSON } from '@/lib/api'
import { usePoll } from '@/lib/poll'
import type { TFunc } from '@/lib/i18n'
import type { Tone } from '@/components/ui'
import { cn } from '@/lib/utils'
import type { ChargeCmd, CmdReply, DeliveryState, LiveStation, PodCmd, Van, WindCmd } from './types'

/** /delivery/state; the old page polled it every 500 ms (map, tables) and
 *  every 400 ms (pod); one poll at the faster rate feeds both now.
 *  旧ページは 500 ms と 400 ms で別々に取得、ここでは 400 ms の 1 本にまとめる */
export const STATE_MS = 400
export function useDeliveryState() {
  return usePoll<DeliveryState>('/delivery/state', STATE_MS).data
}

export async function sendCmd(url: '/delivery/wind' | '/delivery/pod' | '/delivery/charge', body: WindCmd | PodCmd | ChargeCmd): Promise<CmdReply> {
  try {
    return await postJSON<CmdReply>(url, body)
  } catch (e) {
    return { ok: false, error: e instanceof Error ? e.message : String(e) }
  }
}

/* Colours. The map keeps CARLA's own top-down palette (no_rendering_mode.py)
 * and the old dashboard's marker colours so both pages read the same; these
 * are canvas colours, used in JSX only for legend swatches.
 * 地図の色（CARLA の 2D 地図と旧ダッシュボードのマーカー色） */
export const MC = {
  ground: '#555753', road: '#2e3436', side: '#888a85', shoulder: '#423e40',
  bld: '#46484a', bldEdge: '#3a3c3d', white: '#babdb6', yellow: '#edd400',
  oldGround: '#0c1119', oldBld: '#18202d', oldBldEdge: '#243044', oldRoad: '#33415a',
  pod: '#3987e5', car: '#8a96aa', station: '#78ff3c', stationFill: '#78ff3c33',
  ev: '#ffd228', evFill: '#ffd22833', box: '#c084fc', boxEdge: '#0d1117',
  f2f: '#ff8c00', droneOrder: '#00a0ff', hub: '#dc00dc', done: '#0ca30c', fail: '#d03b3b',
  failCross: '#ff9c9c', label: '#e8eef6', link: '#ffffff30', ring: '#ffffff',
  compassBg: '#090c12cc', compassEdge: '#2c3a4e', windArrow: '#7fb8ff', windText: '#cfe3ff', gust: '#ff7a59',
  battGood: '#199e70', battMid: '#fab219', battLow: '#e66767',
  nofly: '#ef4444', noflyFill: '#ef44441f', noflyHatch: '#ef444499', zoneText: '#0b1120', handle: '#ffffff',
  // LastMile charging stations (teal, a bolt marker; red when unusable) / 充電ステーション
  charge: '#2dd4bf', chargeFill: '#2dd4bf2e', chargeLive: '#5eead4',
} as const

/** Charging-bay colour by phase (always with a letter / tooltip too).
 *  充電ベイの状態色 */
export const BAY_COL: Record<string, string> = {
  free: '#5fd45f', in: '#6aa8f0', out: '#6aa8f0', plugging: '#fab219', unplugging: '#fab219', charging: '#ffd228',
}

/** Pod colours for zones, van fills and chips: 8 hues ~40 degrees apart, bright
 *  on the slate map, none of them red (red means no-fly).
 *  ポッドの色（8 色、赤は飛行禁止用に使わない） */
export const POD_PALETTE = ['#22d3ee', '#fde047', '#e879f9', '#a3e635', '#fb923c', '#60a5fa', '#34d399', '#c4b5fd'] as const
export const podColor = (id: number | string) => {
  const n = Math.max(1, Math.round(Number(id)) || 1)
  return POD_PALETTE[(n - 1) % POD_PALETTE.length]
}

export const DRONE_COL: Record<string, string> = {
  docked: '#3987e5', reloading: '#6aa8f0', delivering: '#199e70', to_station: '#c98500', to_van: '#9085e9',
  charging: '#fab219', standby: '#5fd45f', falling: '#ec835a', crashed: '#d03b3b',
}
export const DRONE_TONE: Record<string, Tone> = {
  docked: 'info', reloading: 'info', delivering: 'good', to_station: 'warn', to_van: 'info',
  charging: 'warn', standby: 'good', falling: 'bad', crashed: 'bad',
}

export const FLYING = ['delivering', 'to_station', 'to_van']
export const IN_POD = ['docked', 'reloading']
export const AT_STATION = ['charging', 'standby']
export const DOWN = ['falling', 'crashed']

/** Battery colour on the map: > 50 green, > 25 amber, else red. */
export const battCol = (b: number) => (b > 50 ? MC.battGood : b > 25 ? MC.battMid : MC.battLow)
export const battTone = (b: number): Tone => (b > 50 ? 'good' : b > 25 ? 'warn' : 'bad')

/** Translated name of a drone state (raw name if there is no translation). */
export const droneSt = (t: TFunc, st: string) => { const k = `dst_${st}`; const s = t(k); return s === k ? st : s }
export const orderSt = (t: TFunc, st: string) => { const k = `ost_${st}`; const s = t(k); return s === k ? st : s }
export const failReason = (t: TFunc, r: string) => { const k = `fr_${r}`; const s = t(k); return s === k ? r : s }

export const TH = 'sticky top-0 z-10 bg-card-2 px-2.5 py-2 text-left text-[11px] font-semibold uppercase tracking-wider text-fg-mute whitespace-nowrap border-b border-line'
export const TD = 'px-2.5 py-1.5 border-b border-line/50 whitespace-nowrap text-fg-2'
export const TDN = cn(TD, 'num text-fg')

/* Charging business of a vehicle (the "Send to charge" button) / 充電の状況 */
/** van states that mean it is already on its way to, waiting at or at a
 *  charger / 充電関連の状態 */
export const CHARGE_STATES = ['to_charger', 'ev_charging', 'waiting_charger', 'queued', 'charging', 'plugging', 'unplugging']

/** "charging" = heading for, queued at or in a bay of a charger; "sent" =
 *  told to charge, goes when free; null = neither. / 充電中・指示済み・なし */
export function chargeStatus(v: Van, stations?: LiveStation[] | null): 'charging' | 'sent' | null {
  if (CHARGE_STATES.includes(v.state)) return 'charging'
  for (const st of Array.isArray(stations) ? stations : []) {
    if ((st.bays ?? []).some((b) => b.van === v.id && b.phase !== 'out')) return 'charging'
    if ((st.queue ?? []).some((q) => q.van === v.id)) return 'charging'
  }
  return v.charge_asap ? 'sent' : null
}

/** Ask the City Map to centre on a world point (it listens; nothing happens
 *  when the map is not on the page). / 市街地図をこの地点へ移動 */
export const MAP_CENTRE_EVENT = 'dr-map-centre'
export function centreMap(x: number, y: number) {
  window.dispatchEvent(new CustomEvent<{ x: number; y: number }>(MAP_CENTRE_EVENT, { detail: { x, y } }))
}
