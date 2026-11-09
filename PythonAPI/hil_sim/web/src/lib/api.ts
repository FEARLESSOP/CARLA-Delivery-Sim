// Typed access to the Flask dashboard's routes (dashboard.py). Every view
// talks to the backend through these two helpers only.
// dashboard.py の API への型付きアクセス

export async function getJSON<T>(url: string, signal?: AbortSignal): Promise<T> {
  const r = await fetch(url, { signal, cache: 'no-store' })
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`)
  return (await r.json()) as T
}

export async function postJSON<T = unknown>(url: string, body: unknown): Promise<T> {
  const r = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  const data = (await r.json().catch(() => ({}))) as T & { error?: string }
  if (!r.ok) throw new Error((data && data.error) || `${url}: HTTP ${r.status}`)
  return data
}

/* ---------------------------------------------------------- launcher */
export type OptType = 'int' | 'float' | 'bool' | 'nbool' | 'choice' | 'str' | 'map' | 'vehicle'
export type LiveKind = 'live' | 'map' | 'restart'

export interface OptionSpec {
  key: string
  flag: string
  type: OptType
  default: number | string | boolean
  group: string
  label: string
  help: string
  live: LiveKind
  min?: number
  max?: number
  choices?: string[]
}

export type LaunchOptions = Record<string, number | string | boolean>

export interface MapEntry { name: string; own: boolean; package: string }
export interface CarEntry { id: string; name: string; own: boolean }

export interface LauncherState {
  phase: 'idle' | 'starting' | 'running' | 'stopping' | 'off'
  options: LaunchOptions
  pid: number | null
  started: number | null
  last_exit: { rc: number; t: number } | null
  t: number
}

export interface PipelineState {
  phase: 'running' | 'switching' | 'stopping' | 'stopped'
  pid: number
  t: number
  map: string
  vehicles: number
  walkers: number
  cyclists: number
  traffic: boolean
  delivery: number
  minipod_hubs?: number
  /** LastMile charging stations (newer pipelines) / 充電ステーション */
  charge_stations?: number
  charge_mode?: ChargeMode
  start_vehicle: string
  ffb: boolean
  traction_fx: boolean
  weather_cycle: boolean
  /** road_overlay.py runs (newer pipelines) / 道路ロジック表示ヘルパー */
  road_overlay?: boolean
  running: string[]
  launcher: boolean
}

export interface LauncherInfo {
  launcher: LauncherState | null
  pipeline: PipelineState | null
  schema: OptionSpec[]
  groups: [string, string][]
  saved: LaunchOptions
  catalog: { maps: MapEntry[]; cars: CarEntry[] }
}

/** where the LastMile charging stations come from: placed by the sim
 *  (random, charge_stations of them) or every spot placed on the Delivery map
 *  (manual) / 充電ステーションの配置方法 */
export type ChargeMode = 'random' | 'manual'

export type ControlRequest =
  | { action: 'traffic'; vehicles?: number; walkers?: number; cyclists?: number }
  | { action: 'delivery'; vans: number; lmh?: number; charge_stations?: number; charge_mode?: ChargeMode }
  | { action: 'map'; map: string }
  | { action: 'vehicle'; vehicle: string }
  | { action: 'helper'; name: string; on: boolean }
  | { action: 'stop' }

export const launcherAPI = {
  info: () => getJSON<LauncherInfo>('/api/launcher'),
  launch: (options: LaunchOptions) => postJSON('/api/launcher', { action: 'launch', options }),
  relaunch: (options: LaunchOptions) => postJSON('/api/launcher', { action: 'relaunch', options }),
  stop: () => postJSON('/api/launcher', { action: 'stop' }),
  control: (req: ControlRequest) => postJSON('/api/control', req),
  log: (n = 200) => getJSON<{ lines: string[] }>(`/api/pipeline/log?n=${n}`),
}
