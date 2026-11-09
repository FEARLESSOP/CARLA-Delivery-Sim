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
  /** the pod has a delivery zone (sim side, optional) / ゾーンあり */
  zone?: boolean
  /** "lmp" = LastMile drone pod, "lmh" = LastMile Hub carrying mini pods (newer sims) */
  kind?: 'lmp' | 'lmh' | string
  /** LMH: mini pods in their bays (a count, or their ids) / 格納中のミニポッド */
  pods_docked?: number | number[]
  /** LMH: the distribution point it is at / heading for, and its meeting point */
  dist?: XY | null
  meet?: XY | null
  phase?: string | null
  /** LMH: where its mini pods deliver: "nofly" (default: inside the no-fly
   *  zones, where drones may not fly), "zone" (its own pod zone, drawn by the
   *  user, wins) or "any" / 配達範囲 */
  area?: 'nofly' | 'zone' | 'any' | string
  /** sent to charge from the dashboard, not gone yet (newer sims) / 充電指示済み */
  charge_asap?: boolean
}

/** Mini pod ring / dot colour by status (map, tabs). Canvas colours like MC.
 *  ミニポッドの状態色 */
export const MP_COL: Record<string, string> = {
  docked: '#3987e5', loading: '#6aa8f0', rolling_out: '#9085e9', to_door: '#199e70', handover: '#5fd45f',
  returning: '#c98500', docking: '#6aa8f0', waiting: '#fab219', crashed: '#d03b3b', stuck: '#ec835a',
}

/** A LastMile Mini Delivery Pod as the City Map needs it (the Mini Pods page
 *  types the rest, views/minipods/types.ts). / 地図用のミニポッド */
export interface MiniPodLite {
  id: number
  hub: number | null
  bay?: number | null
  status: string
  x: number
  y: number
  yaw?: number
  battery?: number
  speed?: number
  order?: number | null
  path?: XY[] | null
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
  /** pod whose zone holds the drop point (sim side, optional) / 担当ゾーンのポッド */
  zone?: number | null
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
  /** LastMile Mini Delivery Pods (newer sims) / ミニポッド */
  minipods?: MiniPodLite[]
  orders?: Order[]
  hubs?: Hub[]
  events?: DeliveryEvent[]
  stats?: DeliveryStats
  wind?: Wind
  pod?: PodState | null
  tooltips?: boolean
  /** zones the sim applied (optional) / シミュが適用したゾーン */
  zones?: Partial<ZoneSet>
  /** LastMile charging stations standing in the world (newer sims; missing =
   *  an older sim: nothing drawn) / 設置済みの充電ステーション */
  stations?: LiveStation[]
  /** charging-station spots the sim could not use, with a short English
   *  reason / シミュが使えなかった地点と理由 */
  station_errors?: StationError[]
}

/* LastMile charging stations (state.stations) / LastMile 充電ステーション */
export type BayPhase = 'free' | 'in' | 'plugging' | 'charging' | 'unplugging' | 'out'
/** One charging bay: "lmp" (bay 03, the LMP-4 drone pods) or "lmh" (bay 04,
 *  the hubs); ``van`` = the pod using it. / 充電ベイ */
export interface StationBay {
  kind: 'lmp' | 'lmh' | string
  x: number
  y: number
  yaw?: number
  van: number | null
  phase: BayPhase | string
  /** the vehicle's battery while plugged in, % / 充電量 */
  level?: number | null
  /** current charging power, kW / 現在の電力 */
  kw?: number | null
  /** energy delivered to it this visit, kWh / 今回の充電量 */
  energy_kwh?: number | null
}
/** A vehicle waiting for a free bay of this station. / 順番待ちの車両 */
export interface StationQueued { van: number; kind?: 'lmp' | 'lmh' | string; wait_s?: number }
/** Where the next vehicle waits (and who is standing there). / 待機場所 */
export interface QueueSpot { x: number; y: number; yaw?: number; van: number | null }
/** A Mini Pod charging pad (3 per station). / ミニポッド用充電パッド */
export interface StationPad { pod: number | null; level?: number | null; phase?: string }
/** A drone deck pad (3 per station). / ドローン用デッキ */
export interface StationDeckPad { drone: number | null; level?: number | null; phase?: string }
export interface LiveStation {
  id: number | string
  x: number
  y: number
  yaw?: number
  /** "manual" = a spot placed on the map, "random" = placed by the sim */
  source?: 'manual' | 'random' | string
  /** footprint corners, world metres / 敷地の四隅 */
  corners?: XY[]
  bays?: StationBay[]
  /** total energy this station has delivered, kWh / 累計供給量 */
  energy_kwh?: number | null
  /** vehicles waiting for this station / 順番待ち */
  queue?: StationQueued[]
  queue_spot?: QueueSpot | null
  /** Mini Pod pads and drone deck pads / ミニポッドのパッドとドローンのデッキ */
  pads?: StationPad[]
  deck?: StationDeckPad[]
}
export interface StationError { x: number; y: number; why: string }

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
  /** kerb-side driving-lane points a pod may stop at, 4 m apart (newer sims) */
  stops?: XY[]
}

/* ------------------------------------------------------ /delivery/zones */
/** Axis-aligned rectangle in world metres (x0 < x1, y0 < y1). / 世界座標の矩形 */
export interface Rect { x0: number; y0: number; x1: number; y1: number }
export interface NoFly extends Rect { id: string; name?: string }
/** A pod meeting point (world metres). / 合流地点 */
export interface MeetPoint { x: number; y: number }
/** One map's zones: pod id (string, 1..99) -> rect, the no-fly zones,
 *  pod id -> meeting points (<= 24 each) and LMH hub id -> distribution
 *  points (<= 24 each); older servers omit meet / dist. */
export interface ZoneSet {
  pods: Record<string, Rect>; nofly: NoFly[]; meet: Record<string, MeetPoint[]>; dist: Record<string, MeetPoint[]>
  /** LastMile charging-station spots (<= 12); older servers omit it */
  charge: ChargeSpot[]
}
/** A charging-station spot picked on the map; ``yaw`` in degrees (CARLA,
 *  optional: the sim faces it to the nearest kerb lane). / 充電ステーション地点 */
export interface ChargeSpot { x: number; y: number; yaw?: number }
export interface ZonesReply extends Partial<ZoneSet> { ok?: boolean; map?: string; error?: string }

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

/** POST /delivery/charge: send a vehicle to charge when it is next free. / 充電へ */
export interface ChargeCmd { van: number }

export interface CmdReply { ok: boolean; sent?: Record<string, unknown>; error?: string }
