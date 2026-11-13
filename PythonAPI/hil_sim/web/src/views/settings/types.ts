// Payloads of the settings routes in dashboard.py.
// dashboard.py の設定 API の型

/* /tuning (pedal curves) */
export interface PedalTuning {
  throttle_max: number
  throttle_curve: number
  brake_max: number
  brake_curve: number
}

/* one schema-built slider (ffb_settings.PARAMS, vehicle_tuning.PARAMS) */
export interface SchemaParam {
  key: string
  min: number
  max: number
  step: number
  group: string
  en: string
  ja?: string
  help?: string
}
export interface SchemaGroup { key: string; en: string; ja?: string }
export type Values = Record<string, number>

/* /vehicle_tuning */
export interface VtCar {
  id: number
  type: string
  hero: boolean
  waiting: boolean
  applied?: boolean
  brake_nm: number
  brake_base: number
  torque_nm: number
  torque_base: number
  top_kmh: number
  top_base: number
}
export interface VehicleTuning {
  params: SchemaParam[]
  groups?: SchemaGroup[]
  values: Values
  defaults: Values
  status: { cars?: VtCar[] } | null
}
export type VtPost = { values: Values } | { reset: true } | { neutral: true }

/* /ffb */
export interface FfbStatus {
  age: number
  hero: string
  kmh: number
  surface: string
  front_sat: number
  steer: number
  wheel: number
  output: number
  damper: number
  friction: number
}
export interface FfbInfo {
  params: SchemaParam[]
  groups: SchemaGroup[]
  values: Values
  defaults: Values
  presets: string[]
  user_presets?: string[]
  active: string | null
  status: FfbStatus | null
  error?: string | null
}
export interface FfbPost {
  values?: Values
  preset?: string
  save_preset?: string
  delete_preset?: string
}

/* /keybinds */
export interface KbAction {
  id: string
  en: string
  ja: string
  group: string
  kind: 'hold' | 'press'
  axis: boolean
  window?: string
}
export interface KbCell { keys: string[]; wheel: string[] }
export type KbBindings = Record<string, KbCell>
export interface KbData {
  actions: KbAction[]
  groups: SchemaGroup[]
  bindings: KbBindings
  defaults: KbBindings
  labels: Record<string, string>
  label_ja: Record<string, string>
  stock_labels: Record<string, string>
  stock_order: string[]
  confirmed: string[]
}
/** /keybinds/wheel: the last press the driving window reported. */
export interface WheelLast { spec: string; t: number; label?: string | null }
export interface WheelPoll { last: WheelLast | null; now: number; learning?: boolean }
export interface LearnReply { until?: number; now: number }
export type LabelPost = { spec: string; label: string } | { spec: string; confirm: true } | { reset: true }
