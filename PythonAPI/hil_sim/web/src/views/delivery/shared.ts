// Shared bits of the Delivery page and its settings section: the state poll,
// commands, the colours the map and the tables share, small widgets.
// 配送ページと設定で共有する部品
import { postJSON } from '@/lib/api'
import { usePoll } from '@/lib/poll'
import type { TFunc } from '@/lib/i18n'
import type { Tone } from '@/components/ui'
import { cn } from '@/lib/utils'
import type { CmdReply, DeliveryState, PodCmd, WindCmd } from './types'

/** /delivery/state; the old page polled it every 500 ms (map, tables) and
 *  every 400 ms (pod); one poll at the faster rate feeds both now.
 *  旧ページは 500 ms と 400 ms で別々に取得、ここでは 400 ms の 1 本にまとめる */
export const STATE_MS = 400
export function useDeliveryState() {
  return usePoll<DeliveryState>('/delivery/state', STATE_MS).data
}

export async function sendCmd(url: '/delivery/wind' | '/delivery/pod', body: WindCmd | PodCmd): Promise<CmdReply> {
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
} as const

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
