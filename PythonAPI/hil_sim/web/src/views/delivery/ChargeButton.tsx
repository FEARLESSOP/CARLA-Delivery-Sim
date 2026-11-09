// "Send to charge" for one delivery vehicle (an LMP-4 drone pod or a LastMile
// Hub): POST /delivery/charge, the sim sends it to a charging station the
// next time it is free (charge_asap). Disabled while the vehicle is already
// charging (heading for / queued at / in a bay) or sent; the label says which.
// A failed send shows the error under the button.
// 「充電へ」ボタン：次に空いた時に充電ステーションへ。充電中・指示済みの間は押せない
import { useEffect, useState } from 'react'
import { BatteryCharging } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Button } from '@/components/ui'
import { chargeStatus, sendCmd } from './shared'
import type { DeliveryState, Van } from './types'

/** how long a sent request counts as "sent" before the sim's state confirms
 *  it (charge_asap or a charging state) / シミュの反映を待つ時間 */
const PENDING_MS = 6000

export function ChargeButton({ van, state, size = 'sm', className, full }: {
  van: Van; state: DeliveryState | null; size?: 'sm' | 'md'; className?: string; full?: boolean
}) {
  const t = useT()
  const [busy, setBusy] = useState(false)
  const [sentAt, setSentAt] = useState(0)
  const [err, setErr] = useState<string | null>(null)
  const live = !!(state?.available && state.running)
  const status = chargeStatus(van, state?.stations)
  const pending = sentAt > 0 && !status

  // the local "sent" gives way to the sim's state as soon as that confirms
  // (pending needs !status) and expires after PENDING_MS / 反映後または時間切れで解除
  useEffect(() => {
    if (!sentAt) return
    const id = window.setTimeout(() => setSentAt(0), Math.max(0, sentAt + PENDING_MS - Date.now()))
    return () => window.clearTimeout(id)
  }, [sentAt])
  useEffect(() => {
    if (!err) return
    const id = window.setTimeout(() => setErr(null), 8000)
    return () => window.clearTimeout(id)
  }, [err])

  const send = async () => {
    setBusy(true); setErr(null)
    const r = await sendCmd('/delivery/charge', { van: van.id })
    setBusy(false)
    if (r.ok) setSentAt(Date.now())
    else setErr(r.error || t('dr_ch_err'))
  }

  const label = status === 'charging' ? t('dr_ch_charging') : status === 'sent' || pending ? t('dr_ch_sent') : t('dr_ch_send')
  const why = !live ? t('dr_ch_off') : status === 'charging' ? t('dr_ch_charging_h') : status === 'sent' || pending ? t('dr_ch_sent_h') : t('dr_ch_send_h')
  return (
    <span className={cn('inline-flex flex-col items-start gap-0.5', full && 'w-full', className)}>
      <Button size={size} variant={status || pending ? 'ghost' : 'secondary'} busy={busy}
        disabled={!live || !!status || pending} onClick={send} title={why}
        aria-label={`${t('dr_ch_send')}: V${van.id}. ${why}`}
        className={cn('whitespace-nowrap', full && 'w-full', status === 'charging' && 'text-warn', (status === 'sent' || pending) && 'text-info')}>
        <BatteryCharging className="h-3.5 w-3.5 shrink-0" aria-hidden />{label}
      </Button>
      <span role="status" aria-live="polite" className={cn('max-w-48 text-[11px] leading-tight text-danger', !err && 'sr-only')}>
        {err ? `${t('dr_ch_err')}: ${err}` : pending ? t('dr_ch_sent_h') : ''}
      </span>
    </span>
  )
}
