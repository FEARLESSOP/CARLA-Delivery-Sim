// Payloads of dashboard.py's delivery routes (written by delivery_sim.py via
// delivery/live.py). World metres, x to the right and y down (CARLA top view).
// 配送ルートの型（世界座標 m、x 右・y 下）

export type XY = [number, number]

/* ------------------------------------------------------- /delivery/state */
export type DroneStatus =
  | 'docked' | 'reloading' | 'delivering' | 'to_station' | 'to_van'
  | 'charging' | 'standby' | 'falling' | 'crashed'

export interface Drone {
  id: number
  status: DroneStatus | string
  battery: number
  x: number
  y: number
  z: number
  yaw?: number
  van: number | null
  order: number | null
  drift: number
  trips: number
  crashes: number
  note: string
  fail: string | null
  phase?: string | null
  actor?: number
}

export interface Van {
  id: number
  x: number
  y: number
  yaw: number
  battery: number
  state: string
  drone: number | null
  incoming: number | null
  pending: number
  delivered: number
  failed: number
  ev_charges?: number
  pod_km?: number
  kinematic?: boolean
  actor?: number
}

export type OrderStatus = 'queued' | 'assigned' | 'in_flight' | 'delivered' | 'failed'
export interface Order {
  id: number
  kind: 'drone' | 'face_to_face' | string
  status: OrderStatus | string
  van: number | null
  x: number
  y: number
  reason?: string
}

export interface Hub { van: number; x: number; y: number }

export interface Wind {
  speed?: number
  dir?: number
  base_speed?: number
  base_dir?: number
  gust?: boolean
  gust_peak?: number
  gust_rate?: number
  grounded?: boolean
  hold?: number
  drone_max?: number
  rain?: number
  rain_ground?: number
  raining?: boolean
}

export interface DeliveryEvent {
  t: number
  event: string
  van: number | string | null
  order: number | string | null
  detail: string
}

export interface DeliveryStats {
  busy_vans?: number
  charge_trips?: number
  crashes?: number
  delivered?: number
  delivered_drone?: number
  delivered_face_to_face?: number
  drone_km?: number
  drones?: number
  ev_charges?: number
  failed?: number
  failed_by_reason?: Record<string, number>
  gusts?: number
  pod_km?: number
  pods_stranded?: number
  queued?: number
  repairs?: number
  swaps?: number
  trips?: number
}

/* LastMile pod (state.pod) / LastMile ポッド */
export type ShellMode = 'solid' | 'cutaway' | 'xray' | 'frame'
export type Scenario = 'drone' | 'direct'

export interface PodStep {
  id: string
  scenario: Scenario
  t0: number
  t1: number
  dur?: number
  title: string
}

/** slot: 1 = lift, 0/2.. = bays, null = away from the deck / 1 = リフト、null = 甲板外 */
export interface DeckDrone { id: number; slot: number | null; battery: number }

export interface PodVan {
  id: number
  state: string
  free: boolean
  job?: string | null
  out?: number
  drones: (DeckDrone | null)[]
  packs: (number | null)[]
}

export interface Showcase {
  van: number
  t: number
  playing: boolean
  speed: number
  step: string
  title: string
  scenario: Scenario
}

export interface PodState {
  shell: ShellMode
  showcase: Showcase | null
  steps: PodStep[]
  total: number
  vans: PodVan[]
  msg?: string
}

export interface DeliveryState {
  available: boolean
  running: boolean
  age?: number
  map?: string
  t?: number
  cars?: [number, number, number][]
  drones?: Drone[]
  vans?: Van[]
  orders?: Order[]
  hubs?: Hub[]
  events?: DeliveryEvent[]
  stats?: DeliveryStats
  wind?: Wind
  pod?: PodState | null
  tooltips?: boolean
}

/* --------------------------------------------------------- /delivery/map */
export type LaneKind = 'd' | 's' | 'h'          // driving, sidewalk, shoulder
export interface DeliveryMap {
  map: string
  /** lane centre lines (older delivery_sim draws only these) */
  roads: XY[][]
  /** lane polygons, flat [x0, y0, x1, y1, ...] */
  lanes?: [LaneKind | string, number[]][]
  /** markings: colour, broken, flat polyline */
  marks?: ['w' | 'y' | string, number, number[]][]
  /** [x, y, half-extent x, half-extent y, yaw deg] */
  buildings: [number, number, number, number, number][]
  station: XY[]
  chargers?: XY[]
  boxes?: XY[]
}

/* ------------------------------------------------------------- commands */
export type WindCmd =
  | { cmd: 'gust'; speed: number; dir: number; duration: number }
  | { cmd: 'wind'; speed: number; dir: number }
  | { cmd: 'gust_rate'; value: number }
  | { cmd: 'calm' }

export type PodCmd =
  | { cmd: 'shell'; mode: ShellMode }
  | { cmd: 'tooltips'; on: boolean }
  | { cmd: 'pod_takeover'; van: number; t: number; speed: number }
  | { cmd: 'pod_release' }
  | { cmd: 'pod_play'; playing: boolean; speed: number }
  | { cmd: 'pod_time'; t: number }

export interface CmdReply { ok: boolean; sent?: Record<string, unknown>; error?: string }
