// The parts of the 3D drone (three/DroneModel.tsx): their names, the sensor
// card each one opens, where they sit, and their state from the live sensors
// and the sim's damage model (delivery/drone_sensors.damage_model).
// Model frame (lmp_drone_parts.glb): metres, +x nose, +y up, +z starboard.
// Motors: PX4 quad-X, 1 front-right, 2 rear-left, 3 front-left, 4 rear-right.
// 3D ドローンの部品：名前・開くカード・位置・センサーと損傷から見た状態

import type { TFunc } from '@/lib/i18n'
import type { Tone } from '@/components/ui'
import { HEALTH_RANK, f, mmss, type CardId } from './catalog'
import type { DamagePart, DroneS, Health, Sensors } from './types'

export type PartId = string
export type V3 = [number, number, number]

/** (front +, right +) of each motor corner; right = +z in the model */
export const CORNER: Record<number, [number, number]> = { 1: [1, 1], 2: [-1, -1], 3: [1, -1], 4: [-1, 1] }
export const HUB_R = 0.2376
export const HUB_Y = 0.3036

const corner = (n: number, r: number, y: number): V3 => [CORNER[n][0] * r, y, CORNER[n][1] * r]

/** Every clickable part, grouped for the keyboard list. / クリックできる部品 */
export const PART_GROUPS: { key: string; parts: PartId[] }[] = [
  { key: 'flight', parts: ['rotor1', 'rotor2', 'rotor3', 'rotor4', 'motor1', 'motor2', 'motor3', 'motor4',
    'esc1', 'esc2', 'esc3', 'esc4', 'arm1', 'arm2', 'arm3', 'arm4'] },
  { key: 'avionics', parts: ['imu', 'baro', 'gnss', 'link', 'remote_id', 'lights'] },
  { key: 'sensors', parts: ['cam_fpv', 'cam_down', 'cam_front', 'cam_rear', 'rangefinder', 'flow'] },
  { key: 'power', parts: ['battery', 'pdb'] },
  { key: 'payload', parts: ['winch', 'hook', 'parachute'] },
  { key: 'frame', parts: ['shell', 'body', 'gear', 'lug'] },
]
export const ALL_PARTS: PartId[] = PART_GROUPS.flatMap((g) => g.parts)

/** part -> the sensor card it opens (structural parts: none) / 部品 -> カード */
export function partCard(p: PartId): CardId | undefined {
  if (/^(rotor|motor|esc|arm)\d$/.test(p)) return 'motors'
  const m: Record<string, CardId> = {
    imu: 'imu', baro: 'baro', gnss: 'gnss', link: 'link', remote_id: 'remote_id', cam_fpv: 'cam_fpv',
    cam_down: 'cam_down', cam_front: 'cam_front', cam_rear: 'cam_rear', rangefinder: 'rangefinder',
    flow: 'flow', battery: 'battery', pdb: 'battery', winch: 'winch', hook: 'winch', parachute: 'parachute',
  }
  return m[p]
}

/** card -> the part the "Show on drone" button selects / カード -> 部品 */
export const CARD_PART: Record<CardId, PartId> = {
  gnss: 'gnss', mag: 'gnss', imu: 'imu', baro: 'baro', rangefinder: 'rangefinder', flow: 'flow',
  obstacle: 'cam_front', air: 'imu', battery: 'battery', motors: 'motor1', winch: 'winch', link: 'link',
  remote_id: 'remote_id', parachute: 'parachute', cam_fpv: 'cam_fpv', cam_down: 'cam_down',
  cam_front: 'cam_front', cam_rear: 'cam_rear',
}

/** Where a part sits (its centre, model frame) for labels and close-ups. / 部品の位置 */
export function partCentre(p: PartId): V3 {
  const m = /^(rotor|motor|esc|arm)(\d)$/.exec(p)
  if (m) {
    const n = Number(m[2])
    return m[1] === 'rotor' ? corner(n, HUB_R, 0.31) : m[1] === 'motor' ? corner(n, HUB_R, 0.278)
      : m[1] === 'esc' ? corner(n, 0.185, 0.262) : corner(n, 0.14, 0.252)
  }
  const c: Record<string, V3> = {
    shell: [0, 0.305, 0], body: [0, 0.23, 0], gear: [0, 0.06, 0.1], battery: [0, 0.23, 0.09],
    pdb: [-0.075, 0.207, -0.03], imu: [0.035, 0.236, -0.045], baro: [-0.025, 0.238, -0.058],
    gnss: [0.08, 0.31, 0], rangefinder: [0.097, 0.178, 0], flow: [0.128, 0.18, -0.042],
    cam_fpv: [0.185, 0.193, 0], cam_down: [0.128, 0.18, 0.042], cam_front: [0.184, 0.282, 0],
    cam_rear: [-0.184, 0.282, 0], winch: [0, 0.18, 0], hook: [0, 0.15, 0], link: [-0.2, 0.22, 0],
    remote_id: [0, 0.312, 0.062], parachute: [-0.088, 0.33, 0], lights: [0.24, 0.255, 0], lug: [0, 0.345, 0],
  }
  return c[p] ?? [0, 0.25, 0]
}

/** i18n of a part's name / 部品名 */
export function partName(t: TFunc, p: PartId): string {
  const m = /^(rotor|motor|esc|arm)(\d)$/.exec(p)
  return m ? t(`dn_part_${m[1]}`, { n: m[2] }) : t(`dn_part_${p}`)
}

export interface PartState {
  /** ok / warn / fail / off, the colour on the model / 表示色 */
  level?: Health
  /** the sensor health behind it / センサーの健全性 */
  health?: Health
  /** the damage model's verdict / 損傷 */
  dmg?: DamagePart
}

const worst = (a: Health | undefined, b: Health | undefined): Health | undefined =>
  !a ? b : !b ? a : HEALTH_RANK[a] >= HEALTH_RANK[b] ? a : b

/** The sensor health a part shows: only the sensor that lives in it (a gust
 *  that saturates the wind estimate does not break the GNSS puck).
 *  部品に載っているセンサーの健全性だけを使う */
function ownHealth(s: Sensors, p: PartId): Health | undefined {
  const m = /^(rotor|motor|esc|arm)(\d)$/.exec(p)
  if (m) {
    const mo = s.motors
    if (!mo?.health) return undefined
    const n = Number(m[2])
    if (mo.health === 'fail') {
      // only the failed motor is at fault; the rest are just stopped / 停止中
      if (mo.dead === n && (m[1] === 'motor' || m[1] === 'esc')) return 'fail'
      return 'off'
    }
    return m[1] === 'arm' && mo.health === 'ok' ? 'ok' : mo.health
  }
  switch (p) {
    case 'gnss': return worst(s.gnss?.health, s.mag?.health)
    case 'imu': return s.imu?.health
    case 'baro': return s.baro?.health
    case 'rangefinder': return s.rangefinder?.health
    case 'flow': return s.flow?.health
    case 'cam_front': case 'cam_rear': return s.obstacle?.health
    case 'cam_fpv': case 'cam_down': return s.cams ? (s.cams.available ? 'ok' : 'off') : undefined
    case 'battery': case 'pdb': return s.battery?.health
    case 'winch': case 'hook': return s.winch?.health
    case 'link': return s.link?.health
    case 'remote_id': return s.remote_id?.health
    case 'parachute': return s.parachute?.health
    default: return undefined
  }
}

export function partState(s: Sensors | null | undefined, p: PartId): PartState {
  if (!s) return {}
  const health = ownHealth(s, p)
  const dmg = s.damage?.parts?.[p]
  const level = dmg?.level === 'broken' ? 'fail' : dmg?.level === 'damaged' ? worst('warn', health) : health
  return { level, health, dmg }
}

export function partStates(s: Sensors | null | undefined): Record<PartId, PartState> {
  return Object.fromEntries(ALL_PARTS.map((p) => [p, partState(s, p)]))
}

export const LEVEL_TONE: Record<Health, Tone> = { ok: 'good', warn: 'warn', fail: 'bad', off: 'neutral' }

/** "Broken", "Damaged", "Fault", "OK"... for a part / 部品の状態の文言 */
export function stateText(t: TFunc, st: PartState): string {
  if (st.dmg) return t(`dn_dmg_${st.dmg.level}`)
  return st.level ? t(`dn_h_${st.level}`) : t('dn_h_none')
}

/** The one reading a tooltip shows for a part. / ツールチップの値 */
export function partReading(t: TFunc, p: PartId, d: DroneS): string {
  const s = d.sensors
  if (!s) return ''
  const m = /^(rotor|motor|esc|arm)(\d)$/.exec(p)
  if (m) {
    const i = Number(m[2]) - 1
    const mo = s.motors ?? {}
    if (m[1] === 'esc') return `${f(mo.current?.[i], 1)} A · ${f(mo.throttle?.[i], 0)} %`
    if (m[1] === 'motor') return `${f(mo.rpm?.[i], 0)} rpm · ${f(mo.temp?.[i], 0)} °C`
    if (m[1] === 'rotor') return `${f(mo.rpm?.[i], 0)} rpm`
    return t(`dn_mo_pos${i + 1}`)
  }
  switch (p) {
    case 'gnss': return `${t(`dn_fix_${s.gnss?.fix ?? 'none'}`)} · ${s.gnss?.sats ?? '–'} ${t('dn_sats_short')}`
    case 'imu': return `${t('dn_i_roll')} ${f(s.imu?.roll, 0)}° · ${t('dn_i_pitch')} ${f(s.imu?.pitch, 0)}°`
    case 'baro': return `${f(s.baro?.alt, 1)} m · ${f(s.baro?.pressure, 1)} hPa`
    case 'rangefinder': return s.rangefinder?.in_range ? `${t('dn_agl')} ${f(s.rangefinder?.agl, 1)} m` : t('dn_out_of_range')
    case 'flow': return `${t('dn_f_q')} ${s.flow?.quality ?? 0}/255`
    case 'cam_front': return `${t('dn_o_front')} ${s.obstacle?.front == null ? '> 30' : f(s.obstacle.front, 1)} m`
    case 'cam_rear': return `${t('dn_o_rear')} ${s.obstacle?.rear == null ? '> 30' : f(s.obstacle.rear, 1)} m`
    case 'cam_fpv': case 'cam_down': return s.cams?.available ? t('dn_cam_in_window') : t('dn_cam_docked')
    case 'battery': return `${f(s.battery?.soc, 0)} % · ${f(s.battery?.voltage, 1)} V · ${f(s.battery?.temp, 0)} °C`
    case 'pdb': return `${f(s.battery?.current, 1)} A · ${f(s.battery?.power, 0)} W`
    case 'winch': case 'hook': return `${t(`dn_w_${s.winch?.state ?? 'idle'}`)} · ${f(s.winch?.tether, 1)} m`
    case 'link': return s.link?.mode === 'wired' ? t('dn_l_wired') : `${f(s.link?.rssi, 0)} dBm · ${f(s.link?.latency, 0)} ms`
    case 'remote_id': return s.remote_id?.broadcast ? t('dn_rid_on') : t('dn_rid_off')
    case 'parachute': return t(`dn_c_${s.parachute?.state ?? 'disarmed'}`)
    case 'shell': case 'body': return s.damage?.parts?.shell ? t(`dn_why_${s.damage.parts.shell.why}`) : t('dn_intact')
    default: return ''
  }
}

/** Battery minutes left as text / 残り時間 */
export const leftText = (s: Sensors | null | undefined) => mmss(s?.battery?.left_s)
