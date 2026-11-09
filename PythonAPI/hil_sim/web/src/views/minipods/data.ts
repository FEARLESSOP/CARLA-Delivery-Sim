// Small helpers of the Mini Pods page: status groups and tones, the lid /
// shutter / flag open fraction from whatever the sim sends, distance and ETA
// along the planned path, the hub of a pod, sensors grouped for the cards.
// ミニポッドページの小物：状態・開閉量・経路距離と到着予想・ハブ・センサーの分類
import type { TFunc } from '@/lib/i18n'
import type { Tone } from '@/components/ui'
import type { DeliveryState, Van, XY } from '@/views/delivery/types'
import type { Health, MiniPod, OpenState, PodSensor } from './types'

/** inside the hub (bay) / ハブ内 */
export const IN_HUB = ['docked', 'loading', 'docking']
/** out on the street / 路上 */
export const OUT = ['rolling_out', 'to_door', 'handover', 'returning', 'waiting']
/** needs help / 要対応 */
export const DOWN = ['crashed', 'stuck']

export const STATUS_TONE: Record<string, Tone> = {
  docked: 'info', loading: 'info', docking: 'info', rolling_out: 'info', to_door: 'good', handover: 'good',
  returning: 'warn', waiting: 'warn', crashed: 'bad', stuck: 'bad',
}

/** cruise speed of a mini pod (LastMileHub README: 1.6 m/s) / 巡航速度 */
export const CRUISE = 1.6

export const HEALTH_RANK: Record<Health, number> = { off: 0, ok: 1, warn: 2, fail: 3 }
export const HEALTH_TONE: Record<Health, Tone> = { ok: 'good', warn: 'warn', fail: 'bad', off: 'neutral' }
/** glyph so colour is never the only cue / 色以外の手掛かり */
export const HEALTH_MARK: Record<Health, string> = { ok: '✓', warn: '!', fail: '✕', off: '–' }
export const isHealth = (h: unknown): h is Health => h === 'ok' || h === 'warn' || h === 'fail' || h === 'off'
export const worst = (a: Health | undefined, b: Health | undefined): Health | undefined =>
  !a ? b : !b ? a : HEALTH_RANK[a] >= HEALTH_RANK[b] ? a : b

export const TONE_TEXT: Record<Tone, string> = { good: 'text-accent', warn: 'text-warn', bad: 'text-danger', neutral: 'text-fg-mute', info: 'text-info', accent: 'text-accent' }
export const TONE_DOT: Record<Tone, string> = { good: 'bg-accent', warn: 'bg-warn', bad: 'bg-danger', neutral: 'bg-fg-mute', info: 'bg-info', accent: 'bg-accent' }

/** a number or an en dash / 数値または「–」 */
export const f = (v: number | null | undefined, d = 1): string =>
  typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '–'

/** mm:ss of seconds / 分:秒 */
export const mmss = (s: number | null | undefined) => {
  if (s == null || !Number.isFinite(s)) return '–'
  const v = Math.max(0, Math.round(s))
  return `${Math.floor(v / 60)}:${String(v % 60).padStart(2, '0')}`
}

const pad2 = (n: number) => String(n).padStart(2, '0')
export const podLabel = (t: TFunc, id: number) => t('mp_pod_n', { n: pad2(id) })

/** Translated status (raw if unknown). / 状態の文言 */
export const statusText = (t: TFunc, st: string) => { const k = `mps_${st}`; const s = t(k); return s === k ? st.replace(/_/g, ' ') : s }

const OPEN_WORDS = ['open', 'opened', 'raised', 'up', 'out', 'extended', 'unlocked']
const HALF_WORDS = ['opening', 'closing', 'raising', 'lowering', 'moving']
/** 0 (closed / stowed) .. 1 (open / raised) from a number, boolean or word. */
export function openFrac(v: OpenState): number {
  if (typeof v === 'number' && Number.isFinite(v)) return Math.max(0, Math.min(1, v > 1 ? v / 100 : v))
  if (typeof v === 'boolean') return v ? 1 : 0
  if (typeof v === 'string') {
    const w = v.toLowerCase()
    if (OPEN_WORDS.includes(w)) return 1
    if (HALF_WORDS.includes(w)) return 0.5
  }
  return 0
}

export const xyOf = (p: MiniPod['dest']): XY | null =>
  !p ? null : Array.isArray(p) ? (p.length >= 2 ? [p[0], p[1]] : null) : [p.x, p.y]

/** metres left: along the path when there is one, else straight to dest. */
export function distLeft(p: MiniPod): number | null {
  const path = p.path ?? []
  if (path.length) {
    let d = 0, cx = p.x, cy = p.y
    for (const [x, y] of path) { d += Math.hypot(x - cx, y - cy); cx = x; cy = y }
    return d
  }
  const dst = xyOf(p.dest)
  return dst ? Math.hypot(dst[0] - p.x, dst[1] - p.y) : null
}

/** seconds to the destination at the current speed (cruise when slower). */
export function etaS(p: MiniPod): number | null {
  const d = distLeft(p)
  if (d == null || !OUT.includes(p.status)) return null
  return d / Math.max(p.speed ?? 0, p.status === 'waiting' ? CRUISE : 0.6 * CRUISE)
}

export const lmhVans = (st: DeliveryState | null | undefined): Van[] => (st?.vans ?? []).filter((v) => v.kind === 'lmh')
export const hubOf = (st: DeliveryState | null | undefined, p: MiniPod | undefined) =>
  p?.hub == null ? undefined : st?.vans?.find((v) => v.id === p.hub)

/** broken parts in the damage dict / 破損した部品 */
export const brokenCount = (p: MiniPod) =>
  Object.values(p.damage?.parts ?? {}).filter((v) => v === 'broken').length

export const GROUPS = ['perception', 'navigation', 'drive', 'power', 'payload', 'comms'] as const

/** Sensors by group, in a fixed group order (unknown groups last, sorted). */
export function sensorGroups(s: Record<string, PodSensor> | null | undefined): [string, [string, PodSensor][]][] {
  const by = new Map<string, [string, PodSensor][]>()
  for (const [k, v] of Object.entries(s ?? {})) {
    if (!v || typeof v !== 'object') continue
    const g = typeof v.group === 'string' && v.group ? v.group : 'other'
    const a = by.get(g) ?? []
    a.push([k, v])
    by.set(g, a)
  }
  const order = (g: string) => { const i = (GROUPS as readonly string[]).indexOf(g); return i < 0 ? 99 : i }
  return [...by.entries()].sort((a, b) => order(a[0]) - order(b[0]) || a[0].localeCompare(b[0]))
}

/** A sensor value as text with its unit. / 値と単位 */
export function valueText(t: TFunc, v: PodSensor['value'], digits?: number): string {
  if (v === null || v === undefined) return '–'
  if (typeof v === 'boolean') return v ? t('mp_yes') : t('mp_no')
  if (typeof v === 'number') {
    if (!Number.isFinite(v)) return '–'
    const d = digits ?? (Number.isInteger(v) || Math.abs(v) >= 100 ? 0 : Math.abs(v) >= 10 ? 1 : 2)
    return v.toFixed(d)
  }
  const k = `mpv_${v}`, s = t(k)
  return s === k ? v : s
}

export const sensorName = (_t: TFunc, key: string, s: PodSensor) => s.label || key.replace(/_/g, ' ')
export const groupName = (t: TFunc, g: string) => { const k = `mp_g_${g}`; const s = t(k); return s === k ? g : s }
