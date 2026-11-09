// The detail cards of the Drones page (sensors, then cameras), their health,
// and small shared helpers. The 3D model's parts and their link to these cards
// live in parts.ts.
// 詳細カードの一覧と健全性・共通の小物（3D 部品との対応は parts.ts）

import type { Tone } from '@/components/ui'
import type { Health, Sensors } from './types'

/** Detail cards, in page order. Sensors first, then the four cameras. */
export const SENSOR_IDS = [
  'gnss', 'mag', 'imu', 'baro', 'rangefinder', 'flow', 'obstacle', 'air',
  'battery', 'motors', 'winch', 'link', 'remote_id', 'parachute',
] as const
export const CAMERA_IDS = ['cam_fpv', 'cam_down', 'cam_front', 'cam_rear'] as const
export type SensorId = (typeof SENSOR_IDS)[number]
export type CameraCard = (typeof CAMERA_IDS)[number]
export type CardId = SensorId | CameraCard

/** card -> the camera id of drone_window.py / delivery/drone_sensors.CAMERAS */
export const CAM_OF: Record<CameraCard, 'fpv' | 'down' | 'front' | 'rear'> = {
  cam_fpv: 'fpv', cam_down: 'down', cam_front: 'front', cam_rear: 'rear',
}

/** Health of a card from the sensors payload. Cameras: available in the air, off in the pod. */
export function cardHealth(s: Sensors | null | undefined, id: CardId): Health | undefined {
  if (!s) return undefined
  if (id.startsWith('cam_')) return s.cams ? (s.cams.available ? 'ok' : 'off') : undefined
  const v = s[id as SensorId] as { health?: Health } | undefined
  return v?.health
}

export const HEALTH_RANK: Record<Health, number> = { off: 0, ok: 1, warn: 2, fail: 3 }
export const HEALTH_TONE: Record<Health, Tone> = { ok: 'good', warn: 'warn', fail: 'bad', off: 'neutral' }
/** Health glyph so colour is never the only cue / 色以外の手掛かり */
export const HEALTH_MARK: Record<Health, string> = { ok: '✓', warn: '!', fail: '✕', off: '–' }

/** a number or an en dash / 数値または「–」 */
export const f = (v: number | null | undefined, d = 1): string =>
  typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '–'

export const cardDomId = (id: CardId) => `dn-card-${id}`

/** delivery/drone_sensors.py CAMERAS (mount in the drone frame) / カメラ仕様 */
export const CAM_SPEC: Record<'fpv' | 'down' | 'front' | 'rear', { w: number; h: number; fov: number; pitch: number; yaw: number }> = {
  fpv: { w: 640, h: 360, fov: 90, pitch: -12, yaw: 0 },
  down: { w: 480, h: 360, fov: 100, pitch: -90, yaw: 0 },
  front: { w: 480, h: 270, fov: 110, pitch: -5, yaw: 0 },
  rear: { w: 480, h: 270, fov: 110, pitch: -5, yaw: 180 },
}
export const CAM_FPS = 10

/** mm:ss of seconds / 分:秒 */
export const mmss = (s: number | null | undefined) => {
  if (s == null || !Number.isFinite(s)) return '–'
  const v = Math.max(0, Math.round(s))
  return `${Math.floor(v / 60)}:${String(v % 60).padStart(2, '0')}`
}
