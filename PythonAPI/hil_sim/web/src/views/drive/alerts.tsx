import { useSyncExternalStore } from 'react'
import { createPortal } from 'react-dom'
import { OctagonExclamationPoint, TriangleExclamationPoint } from 'lucide-react'
import '@/i18n/drive'
import { useT } from '@/lib/i18n'
import { fmt } from '@/lib/fmt'
import { num, onTelemetry, type Telemetry } from '@/lib/telemetry'
import { cn } from '@/lib/utils'

// Driver alerts: same rules as the in-sim alerts (ALERT_RULES in the G923
// script): lane crossing, collision, red light, hard brake, TTC < 1.5 s,
// limit + 10. A banner plus a WebAudio beep, each category with a cooldown.
// Checked at import time so the beep also sounds on other pages.
// 運転警告（バナー＋ビープ、種類ごとのクールダウン）。他のページでも鳴る。

export type Severity = 'warn' | 'danger'
export interface Alert {
  id: number
  key: string                                  // i18n key / 文言キー
  extra: string                                // appended text (TTC, speeds)
  severity: Severity
  until: number                                // ms epoch the banner hides
}

let current: Alert | null = null
let seq = 0
let hideTimer = 0
const views = new Set<() => void>()
const cool: Record<string, number> = {}
const prev: { lane: number | null; crash: number; rlv: number; hb: number } = { lane: null, crash: 0, rlv: 0, hb: 0 }
let ac: AudioContext | null = null

function emit() { views.forEach((f) => f()) }

function beep(freq: number, dur: number, times: number) {
  try {
    ac = ac ?? new AudioContext()
    if (ac.state === 'suspended') void ac.resume()
    for (let i = 0; i < times; i++) {
      const o = ac.createOscillator(), g = ac.createGain()
      o.type = 'sine'
      o.frequency.value = freq
      const t0 = ac.currentTime + i * (dur + 0.07)
      g.gain.setValueAtTime(0.0001, t0)
      g.gain.exponentialRampToValueAtTime(0.25, t0 + 0.01)
      g.gain.exponentialRampToValueAtTime(0.0001, t0 + dur)
      o.connect(g); g.connect(ac.destination)
      o.start(t0); o.stop(t0 + dur + 0.02)
    }
  } catch { /* no audio (autoplay policy) / 音声不可 */ }
}

export function showAlert(cat: string, key: string, severity: Severity, ms = 2500, cooldown = 3000, extra = '') {
  const now = Date.now()
  if (now < (cool[cat] ?? 0)) return
  cool[cat] = now + cooldown
  current = { id: ++seq, key, extra, severity, until: now + ms }
  beep(severity === 'danger' ? 880 : 660, 0.14, severity === 'danger' ? 2 : 1)
  window.clearTimeout(hideTimer)
  hideTimer = window.setTimeout(() => { current = current && { ...current, until: 0 }; emit() }, ms)
  emit()
}

function check(s: Telemetry) {
  const lane = num(s, 'lane_invasions'), crash = num(s, 'collision_count')
  const rlv = num(s, 'red_light_violations'), hb = num(s, 'hard_brake_count')
  if (prev.lane !== null && s.hero_found) {
    if (lane > prev.lane) showAlert('lane', 'dv_al_lane', 'warn')
    if (crash > prev.crash) showAlert('crash', 'dv_al_crash', 'danger', 3500, 2000)
    if (rlv > prev.rlv) showAlert('rlv', 'dv_al_redlight', 'danger')
    if (hb > prev.hb) showAlert('hb', 'dv_al_hardbrake', 'warn')
  }
  prev.lane = lane; prev.crash = crash; prev.rlv = rlv; prev.hb = hb
  const ttc = num(s, 'ttc', -1), v = num(s, 'speed_kmh'), lim = num(s, 'speed_limit_kmh')
  if (ttc > 0 && ttc < 1.5 && v > 20)
    showAlert('fcw', 'dv_al_fcw', 'danger', 2000, 3000, ` (TTC ${fmt(ttc, 1)}s)`)
  if (lim > 0 && v > lim + 10)
    showAlert('speed', 'dv_al_speeding', 'warn', 2500, 6000, ` ${fmt(v, 0)} / ${fmt(lim, 0)} km/h`)
}

onTelemetry(check)
// ?alerttest=1 shows a test alert (old page did the same) / テスト表示
if (new URLSearchParams(window.location.search).get('alerttest'))
  window.setTimeout(() => showAlert('test', 'dv_al_test', 'danger', 3000, 0), 800)

function subscribe(f: () => void) { views.add(f); return () => views.delete(f) }
export const useAlert = () => useSyncExternalStore(subscribe, () => current)

/** The alert banner, fixed under the app header. Mount once (the Drive page
 *  does; App could mount it to show banners on every page).
 *  警告バナー（ヘッダー直下に固定表示） */
export function DriveAlertBar() {
  const t = useT()
  const a = useAlert()
  const show = !!a && a.until > Date.now()
  const Icon = a?.severity === 'danger' ? OctagonExclamationPoint : TriangleExclamationPoint
  return createPortal(
    <div role="status" aria-live="assertive"
      className={cn('pointer-events-none fixed left-1/2 top-16 z-50 -translate-x-1/2 transition-all duration-150',
        show ? 'translate-y-0 opacity-100' : '-translate-y-2 opacity-0')}>
      {a && (
        <div className={cn('flex items-center gap-2.5 rounded-xl border px-5 py-3 text-base font-semibold shadow-[var(--shadow-card)] backdrop-blur-md',
          a.severity === 'danger' ? 'border-danger/60 bg-danger/25 text-fg' : 'border-warn/60 bg-warn/20 text-fg')}>
          <Icon className={cn('h-5 w-5 shrink-0', a.severity === 'danger' ? 'text-danger' : 'text-warn')} aria-hidden />
          <span>{t(a.key)}{a.extra}</span>
        </div>
      )}
    </div>,
    document.body,
  )
}
