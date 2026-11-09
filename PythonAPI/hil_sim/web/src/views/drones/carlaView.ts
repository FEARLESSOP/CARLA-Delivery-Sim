// "Open CARLA view": dashboard.py starts drone_window.py, a native CARLA
// window for one drone (a chase camera following the drone body, its onboard
// cameras in a strip, a HUD). Nothing is rendered in the browser.
// GET  /drones/window                  -> {windows: [{drone, pid, running}]}
// POST /drones/window {drone, action}  action "open" (a second open focuses or
//                                      replaces it) | "close"
// CARLA のポップアップ窓（drone_window.py）を開く・閉じる
import { useState } from 'react'
import { postJSON } from '@/lib/api'
import { usePoll } from '@/lib/poll'
import type { WindowReply } from './types'

export function useCarlaView(drone: number | null) {
  const poll = usePoll<WindowReply>('/drones/window', 2000)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const open = (poll.data?.windows ?? []).some((w) => w.drone === drone && w.running)
  const act = async (action: 'open' | 'close') => {
    if (drone === null) return
    setBusy(true)
    setError(null)
    try {
      const r = await postJSON<WindowReply>('/drones/window', { drone, action })
      if (r.ok === false) setError(r.error ?? 'error')
      if (r.windows) poll.setData(r)
      else poll.refresh()
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }
  return { open, busy, error, available: poll.data !== null || !poll.error, act }
}
