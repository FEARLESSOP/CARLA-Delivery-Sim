import { useSyncExternalStore } from 'react'

// The hero car's telemetry (/telemetry, dashboard.py), polled once for the
// whole app at 5 Hz. Views read it with useTelemetry(); history buffers and
// other stores that must keep recording while their view is hidden subscribe
// with onTelemetry() at import time.
// テレメトリ（5 Hz）。表示していないビューの履歴も onTelemetry で記録する。

/** Loosely typed: dashboard.py adds fields over time; read with defaults. */
export interface Telemetry {
  connected: boolean
  hero_found: boolean
  map?: string
  speed_kmh?: number
  speed_limit_kmh?: number
  [key: string]: unknown
}

type Listener = (s: Telemetry) => void
let current: Telemetry | null = null
let online = false
const views = new Set<() => void>()
const listeners = new Set<Listener>()
let started = false

function emit() { views.forEach((f) => f()) }

let firstTick = true
async function tick() {
  if (document.hidden && !firstTick) return
  firstTick = false
  try {
    const r = await fetch('/telemetry', { cache: 'no-store' })
    const s = (await r.json()) as Telemetry
    current = s
    online = true
    listeners.forEach((f) => { try { f(s) } catch (e) { console.error(e) } })
  } catch {
    online = false
    current = current ? { ...current, connected: false } : null
  }
  emit()
}

export function startTelemetry(ms = 200) {
  if (started) return
  started = true
  tick()
  window.setInterval(tick, ms)
  document.addEventListener('visibilitychange', () => { if (!document.hidden) tick() })
}

/** Called with every sample, whichever view is shown. Returns an unsubscribe. */
export function onTelemetry(f: Listener): () => void {
  listeners.add(f)
  return () => listeners.delete(f)
}

function subscribe(f: () => void) { views.add(f); return () => views.delete(f) }

/** The latest sample (re-renders at 5 Hz). null before the first answer. */
export function useTelemetry(): Telemetry | null {
  return useSyncExternalStore(subscribe, () => current)
}

/** Is the dashboard backend answering at all? */
export function useBackendOnline(): boolean {
  return useSyncExternalStore(subscribe, () => online)
}

/** Number helper for loosely typed fields. */
export const num = (s: Telemetry | null, k: string, d = 0): number => {
  const v = s ? s[k] : undefined
  return typeof v === 'number' && isFinite(v) ? v : d
}
