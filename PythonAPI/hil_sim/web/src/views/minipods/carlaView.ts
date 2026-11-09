// "Open CARLA view": dashboard.py starts minipod_window.py, a native CARLA
// window for one mini pod (a chase camera behind / above the pod body, its six
// onboard cameras in a strip, a HUD). Nothing is rendered in the browser.
// GET  /minipods/window               -> {windows: [{id, pid, running}]}
// POST /minipods/window {id, action}  action "open" (a second open focuses or
//                                     replaces it) | "close"
// CARLA のポップアップ窓（minipod_window.py）を開く・閉じる
import { useState } from 'react'
import { postJSON } from '@/lib/api'
import { usePoll } from '@/lib/poll'
import type { WindowReply } from './types'

export interface CarlaView { open: boolean; busy: boolean; error: string | null; available: boolean; act: (a: 'open' | 'close') => void }

export function useCarlaView(id: number | null): CarlaView {
  const poll = usePoll<WindowReply>('/minipods/window', 2000)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const open = (poll.data?.windows ?? []).some((w) => w.id === id && w.running)
  const act = async (action: 'open' | 'close') => {
    if (id === null) return
    setBusy(true)
    setError(null)
    try {
      const r = await postJSON<WindowReply>('/minipods/window', { id, action })
      if (r.ok === false) setError(r.error ?? 'error')
      if (r.windows) poll.setData(r)
      else poll.refresh()
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }
  return { open, busy, error, available: poll.data !== null || !poll.error, act: (a) => { void act(a) } }
}
