import { getJSON } from './api'

// /delivery/map (~3 MB) shared by every page that draws it (City Map, Drones,
// Mini Pods): one download per map name, kept while the map stays the same.
// A failed download is forgotten so the next call retries.
// /delivery/map（約 3 MB）を全ページで共有：マップごとに 1 回だけ取得する。
let cache: { name: string; p: Promise<unknown> } | null = null

export function loadDeliveryMap<T>(name: string): Promise<T> {
  if (cache && cache.name === name) return cache.p as Promise<T>
  const p = getJSON<T>('/delivery/map')
  cache = { name, p }
  p.catch(() => { if (cache && cache.p === p) cache = null })
  return p
}
