// The Mini Pods page's payloads: minipods[] and the LMH vans in
// /delivery/state (written by delivery_sim.py, the LastMile Hub logic) and
// dashboard.py's /minipods/window (the CARLA pop-up, minipod_window.py).
// Every field is optional on purpose: until the delivery sim is restarted it
// publishes no mini pods, and a sensor value is null while it has no reading.
// The sensor dict describes itself (label, unit, health, group, part): the
// page renders whatever keys arrive, it hard-codes none.
// ミニポッドページの型（delivery_sim.py と /minipods/window）

import type { Van, XY } from '@/views/delivery/types'

export type Health = 'ok' | 'warn' | 'fail' | 'off'

export type MpStatus =
  | 'docked' | 'loading' | 'rolling_out' | 'to_door' | 'handover' | 'returning'
  | 'docking' | 'waiting' | 'crashed' | 'stuck'

export type SensorGroup = 'perception' | 'navigation' | 'drive' | 'power' | 'payload' | 'comms'

/** One self-describing sensor reading. / 自己記述的なセンサー値 */
export interface PodSensor {
  label?: string
  value?: number | string | boolean | null
  unit?: string
  health?: Health
  group?: SensorGroup | string
  /** a GLB node name (MiniPod_P1_Wheel3, Lid...) or a mount key (lidar, cam_front...) */
  part?: string
}

export interface PodDamage {
  /** part (node name or mount key) -> "broken" | "damaged" */
  parts?: Record<string, 'broken' | 'damaged' | string>
  why?: string | null
}

/** lid / shutter / flag: 0..1 open fraction, a boolean, or a word (open, closed, raised, stowed) */
export type OpenState = number | boolean | string | null | undefined

export interface MiniPod {
  id: number
  hub: number | null
  bay?: number | null
  status: MpStatus | string
  x: number
  y: number
  z?: number
  yaw?: number
  /** m/s */
  speed?: number
  /** % */
  battery?: number
  order?: number | null
  dest?: XY | { x: number; y: number } | null
  lid?: OpenState
  shutter?: OpenState
  flag?: OpenState
  actor?: number | null
  trips?: number
  km?: number
  fail?: string | null
  /** the planned path ahead (world x, y) / 予定経路 */
  path?: XY[] | null
  sensors?: Record<string, PodSensor> | null
  damage?: PodDamage | null
}

/** A LastMile Hub (vans[] with kind "lmh"). / LastMile ハブ */
export type LmhVan = Van

/* ------------------------------------------------ /minipods/window (CARLA pop-up) */
export interface PodWindow { id: number; pid: number; running: boolean; started?: number }
export interface WindowReply {
  ok?: boolean
  error?: string
  /** opened, focused, replaced, closed, not_open, too_many */
  action?: string
  windows?: PodWindow[]
}
