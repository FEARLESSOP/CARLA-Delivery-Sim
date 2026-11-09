// The Drones page's payloads: drones[].sensors in /delivery/state (written by
// delivery_sim.py, delivery/drone_sensors.py, damage from its damage_model)
// and dashboard.py's /drones/window (the CARLA pop-up, drone_window.py).
// Every field is optional on purpose: an older sim publishes no sensors, and
// values are null while a sensor has no reading.
// ドローンページの型（drone_sensors.py と /drones/window）

import type { Drone } from '@/views/delivery/types'

export type Health = 'ok' | 'warn' | 'fail' | 'off'

export interface GnssS { lat?: number; lon?: number; alt?: number; fix?: 'rtk_fixed' | 'rtk_float' | '3d' | 'none' | string; sats?: number; hdop?: number; acc_h?: number | null; speed?: number; health?: Health }
export interface MagS { heading?: number; field?: number; interference?: boolean; health?: Health }
export interface ImuS { ax?: number; ay?: number; az?: number; gx?: number; gy?: number; gz?: number; roll?: number; pitch?: number; vibe?: number; health?: Health }
export interface BaroS { alt?: number; pressure?: number; vs?: number; temp?: number; health?: Health }
export interface RangeS { agl?: number | null; ground?: number | null; max?: number; in_range?: boolean; health?: Health }
export interface FlowS { vx?: number | null; vy?: number | null; quality?: number; health?: Health }
export interface ObstacleS { front?: number | null; rear?: number | null; down?: number | null; nearest?: number | null; max?: number; health?: Health }
export interface AirS { airspeed?: number; wind?: number; wind_dir?: number; est?: number; est_dir?: number; drift?: number; limit?: number; saturated?: boolean; health?: Health }
export interface BatteryS { soc?: number; voltage?: number; cell?: number; current?: number; power?: number; temp?: number; left_s?: number | null; capacity_ah?: number; cells?: number; charging?: boolean; health?: Health }
export interface MotorsS { rpm?: number[]; current?: number[]; temp?: number[]; throttle?: number[]; dead?: number | null; health?: Health }
export interface WinchS { state?: 'stowed' | 'idle' | 'lowering' | 'released' | 'retracted' | string; tether?: number; speed?: number; load?: number; payload?: number; max_tether?: number; health?: Health }
export interface LinkS { mode?: 'wired' | 'c2' | 'lte' | string; rssi?: number | null; snr?: number | null; latency?: number; loss?: number; dist?: number | null; lte?: number | null; health?: Health }
export interface RemoteIdS { broadcast?: boolean; serial?: string; traffic?: number; nearest?: number | null; health?: Health }
export interface ChuteS { state?: 'armed' | 'low' | 'disarmed' | 'deployed' | 'too_low' | string; min_agl?: number; health?: Health }
export interface CamsS { available?: boolean; list?: string[]; fps?: number }

export interface Sensors {
  t?: number
  health?: Health
  gnss?: GnssS
  mag?: MagS
  imu?: ImuS
  baro?: BaroS
  rangefinder?: RangeS
  flow?: FlowS
  obstacle?: ObstacleS
  air?: AirS
  battery?: BatteryS
  motors?: MotorsS
  winch?: WinchS
  link?: LinkS
  remote_id?: RemoteIdS
  parachute?: ChuteS
  cams?: CamsS
  damage?: Damage | null
}

export interface DroneS extends Drone {
  sensors?: Sensors | null
  km?: number
  /** seconds in the air this session (newer sims) / 飛行時間 */
  air_s?: number
}

/* ------------------------------------------------ damage (drone_sensors.damage_model) */
export type DamageLevel = 'broken' | 'damaged'
export interface DamagePart { level: DamageLevel; why: string }
export interface Damage {
  parts: Record<string, DamagePart>
  /** collision, motor_failure, battery_depleted, wind, parcel_lost, ... */
  cause?: string | null
  /** front_right, rear_left, front_left, rear_right */
  impact?: string | null
  fall_h?: number
  broken?: string[]
  detail?: string
}

/* ------------------------------------------------ /drones/window (CARLA pop-up) */
export interface DroneWindow { drone: number; pid: number; running: boolean; started?: number }
export interface WindowReply {
  ok?: boolean
  error?: string
  /** opened, focused, replaced, closed, not_open */
  action?: string
  windows?: DroneWindow[]
}
