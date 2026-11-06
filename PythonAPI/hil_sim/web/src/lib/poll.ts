import { useEffect, useRef, useState } from 'react'
import { getJSON } from './api'

/** Poll a JSON route every ``ms`` while ``enabled`` and the page is visible.
 *  Returns the latest data, the last error and a manual refresh. Requests
 *  never overlap. / 表示中だけ一定間隔で取得（重複しない） */
export function usePoll<T>(url: string | null, ms: number, enabled = true) {
  const [data, setData] = useState<T | null>(null)
  const [error, setError] = useState<string | null>(null)
  const tick = useRef<() => void>(() => {})

  useEffect(() => {
    if (!url || !enabled) return
    let alive = true
    let busy = false          // per effect run: a StrictMode remount must not inherit it / 実行ごと
    const ctl = new AbortController()
    let first = true
    const run = async () => {
      // hidden tabs skip polls, but the first answer always comes (a tab
      // opened in the background must not stay empty) / 初回は非表示でも取得
      if (busy || (document.hidden && !first)) return
      first = false
      busy = true
      try {
        const d = await getJSON<T>(url, ctl.signal)
        if (alive) { setData(d); setError(null) }
      } catch (e) {
        if (alive && !(e instanceof DOMException && e.name === 'AbortError')) setError(String(e))
      } finally {
        busy = false
      }
    }
    tick.current = run
    run()
    const id = window.setInterval(run, ms)
    const onVis = () => { if (!document.hidden) run() }
    document.addEventListener('visibilitychange', onVis)
    return () => { alive = false; ctl.abort(); window.clearInterval(id); document.removeEventListener('visibilitychange', onVis) }
  }, [url, ms, enabled])

  return { data, error, refresh: () => tick.current(), setData }
}
