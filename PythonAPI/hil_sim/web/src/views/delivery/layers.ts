// What the City Map draws, layer by layer, for the Layers filter. The first
// three live in the cached base image (toggling them re-renders it once); the
// rest are drawn every frame. Saved per browser in localStorage.
// 地図のレイヤー（最初の 3 つはキャッシュ画像、残りは毎フレーム描画）
export const LAYER_GROUPS = [
  ['dr_lyg_map', ['roads', 'marks', 'buildings']],
  ['dr_lyg_places', ['station', 'ev', 'boxes', 'hubs']],
  ['dr_lyg_fleet', ['pods', 'drones', 'minipods', 'minipodPaths', 'links', 'cars', 'labels']],
  ['dr_lyg_orders', ['ordF2f', 'ordDrone', 'ordDone', 'ordFail']],
  ['dr_lyg_zones', ['podZones', 'nofly', 'meet', 'dist', 'charge']],
  ['dr_lyg_over', ['wind']],
] as const

export type LayerKey = (typeof LAYER_GROUPS)[number][1][number]
export type Layers = Record<LayerKey, boolean>

export const LAYER_KEYS: LayerKey[] = LAYER_GROUPS.flatMap(([, ks]) => [...ks])
/** layers painted into the cached base image / キャッシュ画像に入るレイヤー */
export const BASE_LAYERS: LayerKey[] = ['roads', 'marks', 'buildings']

const STORE = 'dr_map_layers'

export const allLayers = (on: boolean): Layers =>
  Object.fromEntries(LAYER_KEYS.map((k) => [k, on])) as Layers

export function loadLayers(): Layers {
  const l = allLayers(true)
  try {
    const raw: unknown = JSON.parse(localStorage.getItem(STORE) || '{}')
    if (raw && typeof raw === 'object') {
      for (const k of LAYER_KEYS) {
        const v = (raw as Record<string, unknown>)[k]
        if (typeof v === 'boolean') l[k] = v
      }
    }
  } catch { /* private mode / blocked storage: all on / 既定は全表示 */ }
  return l
}

export function saveLayers(l: Layers) {
  try { localStorage.setItem(STORE, JSON.stringify(l)) } catch { /* ignore / 無視 */ }
}
