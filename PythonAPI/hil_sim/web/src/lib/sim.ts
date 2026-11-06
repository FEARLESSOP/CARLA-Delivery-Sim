import { useSyncExternalStore } from 'react'
import { launcherAPI, type LauncherInfo } from './api'

// The launcher / pipeline status (/api/launcher), polled once for the app.
// ランチャーとパイプラインの状態（アプリ全体で 1 回だけ取得）
let info: LauncherInfo | null = null
const subs = new Set<() => void>()
let started = false

async function tick() {
  try { info = await launcherAPI.info() } catch { /* backend restarting */ }
  subs.forEach((f) => f())
}

export function startSim(ms = 1000) {
  if (started) return
  started = true
  tick()
  window.setInterval(tick, ms)
}
export const refreshSim = tick

export function useSim(): LauncherInfo | null {
  return useSyncExternalStore((f) => { subs.add(f); return () => subs.delete(f) }, () => info)
}

export type SimPhase = 'offline' | 'idle' | 'starting' | 'running' | 'switching' | 'stopping'

/** One phase for the UI from the launcher's and the pipeline's state. */
export function simPhase(i: LauncherInfo | null): SimPhase {
  if (!i) return 'offline'
  if (i.pipeline?.phase === 'switching') return 'switching'
  if (i.launcher?.phase === 'stopping' || i.pipeline?.phase === 'stopping') return 'stopping'
  if (i.pipeline?.phase === 'running') return 'running'
  if (i.launcher?.phase === 'starting') return 'starting'
  return 'idle'
}
