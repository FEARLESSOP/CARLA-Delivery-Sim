// The parts of the 3D mini pod (three/PodModel.tsx): ids, where they sit, the
// sensors that live in them and their state. The sim names parts by GLB node
// (MiniPod_P1_Wheel3, Lid, Shutter, FlagMast...) or by mount key (lidar,
// cam_front, gnss...); normPart() turns either into one of our ids, so the
// sensor dict and the damage dict stay free-form.
// Model frame (lmh_minipod.glb): metres, +x nose, +y up, +z right (starboard).
// Body 0.70 x 0.54 x 0.67 m, lidar top 0.72 m; wheels 1/2 rear, 3/4 middle,
// 5/6 front, odd = left (-z), even = right (+z).
// 3D ミニポッドの部品：ID・位置・載っているセンサー・状態
import type { TFunc } from '@/lib/i18n'
import type { Tone } from '@/components/ui'
import { HEALTH_TONE, isHealth, worst } from './data'
import type { Health, MiniPod, PodSensor } from './types'

export type PartId = string
export type V3 = [number, number, number]
export type DamageLevel = 'broken' | 'damaged'

export const WHEEL_X = [-0.235, -0.235, 0, 0, 0.235, 0.235]
export const WHEEL_Y = 0.085
export const WHEEL_R = 0.085
export const wheelPos = (n: number): V3 => [WHEEL_X[n - 1], WHEEL_Y, n % 2 ? -0.235 : 0.235]

/** Every clickable part, grouped for the keyboard list. / クリックできる部品 */
export const PART_GROUPS: { key: string; parts: PartId[] }[] = [
  { key: 'perception', parts: ['lidar', 'cam_front', 'cam_rear', 'cam_left', 'cam_right', 'cam_down', 'cam_lid', 'bumper'] },
  { key: 'navigation', parts: ['gnss', 'compute'] },
  { key: 'drive', parts: ['wheel1', 'wheel2', 'wheel3', 'wheel4', 'wheel5', 'wheel6'] },
  { key: 'power', parts: ['battery', 'charge_port'] },
  { key: 'payload', parts: ['lid', 'lid_lock', 'shutter', 'payload'] },
  { key: 'comms', parts: ['link', 'flag'] },
  { key: 'body', parts: ['shell', 'headlights', 'taillights'] },
]
export const ALL_PARTS: PartId[] = PART_GROUPS.flatMap((g) => g.parts)

/** A sim part name (GLB node or mount key) as one of our part ids. / 部品名の正規化 */
export function normPart(raw: string | undefined | null): PartId {
  if (!raw) return ''
  let s = raw.trim().toLowerCase()
  s = s.replace(/^minipod_p\d+_+/, '').replace(/__m_.*$/, '').replace(/^minipod_+/, '')
  let m = /(?:wheel|motor|hub_motor|drive)[ _-]?(\d)/.exec(s)
  if (m && Number(m[1]) >= 1 && Number(m[1]) <= 6) return `wheel${m[1]}`
  m = /(?:cam(?:era)?)[ _-]?(front|rear|back|left|right|down|bottom|lid|bay|payload)/.exec(s)
    ?? /(front|rear|back|left|right|down|bottom|lid|bay|payload)[ _-]?cam(?:era)?/.exec(s)
  if (m) {
    const w = m[1]
    return `cam_${w === 'back' ? 'rear' : w === 'bottom' ? 'down' : w === 'bay' || w === 'payload' ? 'lid' : w}`
  }
  if (/lidar/.test(s)) return 'lidar'
  if (/lock|latch/.test(s)) return 'lid_lock'
  if (/^lid|_lid|lid$/.test(s)) return 'lid'
  if (/shutter/.test(s)) return 'shutter'
  if (/flag/.test(s)) return 'flag'
  if (/gnss|gps|rtk/.test(s)) return 'gnss'
  if (/imu|compute|computer|cpu|ecu|controller|brain/.test(s)) return 'compute'
  if (/link|lte|radio|antenna|modem|v2x|wifi|comms/.test(s)) return 'link'
  if (/batt|bms|pack|cell/.test(s)) return 'battery'
  if (/charge|socket|hv|contact|plug/.test(s)) return 'charge_port'
  if (/ultra|sonar|bumper|radar|obstacle|tof|cliff/.test(s)) return 'bumper'
  if (/head ?light|headlamp/.test(s)) return 'headlights'
  if (/tail|brake ?light/.test(s)) return 'taillights'
  if (/belt|payload|bay|parcel|cargo|load/.test(s)) return 'payload'
  if (/body|shell|chassis|frame|cover|panel/.test(s)) return 'shell'
  return s
}

/** The part a sensor lives in: its "part" field, else its key. / センサーの部品 */
export const sensorPart = (key: string, s: PodSensor): PartId => normPart(s.part || key)

/** Where a part sits (its centre, model frame) for labels and close-ups. / 部品の位置 */
export function partCentre(p: PartId): V3 {
  const m = /^wheel(\d)$/.exec(p)
  if (m) return wheelPos(Number(m[1]))
  const c: Record<string, V3> = {
    shell: [0, 0.45, 0], lid: [-0.08, 0.675, 0], lid_lock: [0.165, 0.655, 0], shutter: [-0.355, 0.45, 0],
    flag: [0.23, 1.1, -0.19], lidar: [0.27, 0.69, 0], cam_front: [0.36, 0.515, 0], cam_rear: [-0.36, 0.64, 0],
    cam_left: [0.08, 0.58, -0.276], cam_right: [0.08, 0.58, 0.276], cam_down: [0.36, 0.6, 0],
    cam_lid: [-0.09, 0.64, 0], bumper: [0.36, 0.245, 0], gnss: [0.21, 0.674, 0.12], link: [0.325, 0.7, 0.2],
    compute: [0.24, 0.25, 0], battery: [-0.05, 0.22, 0], charge_port: [-0.35, 0.2, 0], headlights: [0.352, 0.33, 0],
    taillights: [-0.354, 0.27, 0], payload: [-0.1, 0.42, 0],
  }
  return c[p] ?? [0, 0.45, 0]
}

/** i18n of a part's name; an unknown sim part shows its own name. / 部品名 */
export function partName(t: TFunc, p: PartId): string {
  const m = /^wheel(\d)$/.exec(p)
  if (m) return t('mp_part_wheel', { n: m[1], pos: t(`mp_wpos_${m[1]}`) })
  const k = `mp_part_${p}`, s = t(k)
  return s === k ? p.replace(/_/g, ' ') : s
}

export interface PartState {
  /** ok / warn / fail / off, the colour on the model / 表示色 */
  level?: Health
  /** the worst health of the sensors in it / センサーの健全性 */
  health?: Health
  /** the damage model's verdict / 損傷 */
  dmg?: DamageLevel
  /** sensor keys living in this part / この部品のセンサー */
  sensors: string[]
}

/** damage dict -> our part ids (worst level wins) / 損傷を部品 ID へ */
export function damageByPart(p: MiniPod | undefined): Record<PartId, DamageLevel> {
  const out: Record<PartId, DamageLevel> = {}
  for (const [k, v] of Object.entries(p?.damage?.parts ?? {})) {
    const id = normPart(k)
    if (v === 'broken') out[id] = 'broken'
    else if ((v === 'damaged' || v) && out[id] !== 'broken') out[id] = 'damaged'
  }
  return out
}

/** Every known part (plus any the sim names that we have no id for). */
export function partStates(p: MiniPod | undefined): Record<PartId, PartState> {
  const out: Record<PartId, PartState> = Object.fromEntries(ALL_PARTS.map((id) => [id, { sensors: [] }]))
  for (const [k, s] of Object.entries(p?.sensors ?? {})) {
    if (!s || typeof s !== 'object') continue
    const id = sensorPart(k, s)
    const st = out[id] ?? (out[id] = { sensors: [] })
    st.sensors.push(k)
    if (isHealth(s.health)) st.health = worst(st.health, s.health)
  }
  for (const [id, lv] of Object.entries(damageByPart(p))) {
    const st = out[id] ?? (out[id] = { sensors: [] })
    st.dmg = lv
  }
  for (const st of Object.values(out)) {
    st.level = st.dmg === 'broken' ? 'fail' : st.dmg === 'damaged' ? worst('warn', st.health) : st.health
  }
  return out
}

export const LEVEL_TONE: Record<Health, Tone> = HEALTH_TONE

/** "Broken", "Damaged", "Fault", "OK"... for a part / 部品の状態の文言 */
export function stateText(t: TFunc, st: PartState | undefined): string {
  if (st?.dmg) return t(`mp_dmg_${st.dmg}`)
  return st?.level ? t(`mp_h_${st.level}`) : t('mp_h_none')
}

/** The reading a tooltip shows: the first one or two sensors in the part. */
export function partReading(t: TFunc, p: PartId, pod: MiniPod, st: PartState | undefined): string {
  const s = pod.sensors ?? {}
  const parts = (st?.sensors ?? []).slice(0, 2).map((k) => {
    const v = s[k]
    if (!v) return ''
    const val = v.value
    const txt = val === null || val === undefined ? '–' : typeof val === 'number' ? (Number.isFinite(val) ? val.toFixed(Math.abs(val) >= 100 ? 0 : 1) : '–')
      : typeof val === 'boolean' ? (val ? t('mp_yes') : t('mp_no')) : String(val)
    return `${v.label || k} ${txt}${v.unit ? ` ${v.unit}` : ''}`
  }).filter(Boolean)
  if (parts.length) return parts.join(' · ')
  if (p === 'lid') return t('mp_lid_is', { s: t(pod.lid ? 'mp_open' : 'mp_closed') })
  if (p === 'flag') return t(pod.flag ? 'mp_flag_up' : 'mp_flag_down')
  if (p === 'battery' && pod.battery != null) return `${Math.round(pod.battery)} %`
  if (/^wheel\d$/.test(p) && pod.speed != null) return `${(pod.speed * 3.6).toFixed(1)} km/h`
  return ''
}
