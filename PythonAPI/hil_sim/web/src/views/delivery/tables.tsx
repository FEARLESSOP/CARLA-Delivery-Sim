// Deliveries & failures, Drones, Delivery pods, Drone events.
// 配達と失敗・ドローン一覧・配送ポッド・イベント
import { AnimatePresence, motion, useReducedMotion } from 'motion/react'
import { useT } from '@/lib/i18n'
import { fmt } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Badge, Card, EmptyState, KV } from '@/components/ui'
import { DOWN, TD, TDN, TH, failReason } from './shared'
import { Battery, DroneBadge, TableFrame } from './widgets'
import type { DeliveryState } from './types'

/* -------------------------------------------------- deliveries & failures */
export function StatsCard({ state, className }: { state: DeliveryState | null; className?: string }) {
  const t = useT()
  const st = state?.stats ?? {}
  const fr = st.failed_by_reason ?? {}
  return (
    <Card title={t('dr_stats')} className={className}>
      {!state?.available ? <EmptyState>{t('dr_waiting')}</EmptyState> : (
        <div className="grid gap-x-6 sm:grid-cols-2 lg:grid-cols-1 2xl:grid-cols-2">
          <div>
            <KV k={t('dr_s_deliv')} v={<>{st.delivered || 0} <span className="text-xs text-fg-mute">({st.delivered_drone || 0} {t('dr_s_bydrone')})</span></>} tone="good" />
            <KV k={t('dr_s_fail')} v={st.failed || 0} tone={(st.failed || 0) > 0 ? 'bad' : 'neutral'} />
            {Object.keys(fr).sort().map((k) => (
              <KV key={k} k={<span className="pl-3">· {failReason(t, k)}</span>} v={fr[k]} tone="bad" />
            ))}
            <KV k={t('dr_s_queued')} v={st.queued || 0} />
          </div>
          <div className="mt-2 border-t border-line pt-2 sm:mt-0 sm:border-0 sm:pt-0 lg:mt-2 lg:border-t lg:pt-2 2xl:mt-0 2xl:border-0 2xl:pt-0">
            <KV k={t('dr_s_trips')} v={st.trips || 0} />
            <KV k={t('dr_s_crash')} v={st.crashes || 0} tone={(st.crashes || 0) > 0 ? 'warn' : 'neutral'} />
            <KV k={t('dr_s_swaps')} v={st.swaps || 0} />
            <KV k={t('dr_s_charge')} v={st.charge_trips || 0} />
            <KV k={t('dr_s_repair')} v={st.repairs || 0} />
            <KV k={t('dr_s_gusts')} v={st.gusts || 0} />
            <KV k={t('dr_s_km')} v={`${fmt(st.drone_km, 2)} km`} />
            <KV k={t('dr_s_evch')} v={st.ev_charges || 0} />
            <KV k={t('dr_s_podkm')} v={`${fmt(st.pod_km, 2)} km`} />
            <KV k={t('dr_s_strand')} v={st.pods_stranded || 0} tone={(st.pods_stranded || 0) > 0 ? 'bad' : 'neutral'} />
          </div>
        </div>
      )}
    </Card>
  )
}

/* ------------------------------------------------------------ drones */
export function DronesTable({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const ds = state?.available ? state.drones ?? [] : []
  return (
    <Card title={t('dr_drones')} actions={<Badge className="num">{ds.length}</Badge>}>
      {!state?.available ? <EmptyState>{t('dr_waiting')}</EmptyState> : !ds.length ? <EmptyState>–</EmptyState> : (
        <TableFrame label={t('dr_drones')} maxH="max-h-[480px]">
          <thead><tr>
            {['dr_th_drone', 'dr_th_status', 'dr_th_batt', 'dr_th_van', 'dr_th_order', 'dr_th_alt', 'dr_th_drift',
              'dr_th_trips', 'dr_th_crash', 'dr_th_note'].map((k) => <th key={k} scope="col" className={TH}>{t(k)}</th>)}
          </tr></thead>
          <tbody>
            {ds.map((d) => (
              <tr key={d.id} className="hover:bg-muted/40">
                <th scope="row" className={cn(TDN, 'text-left font-semibold')}>D{d.id}</th>
                <td className={TD}><DroneBadge t={t} status={d.status} /></td>
                <td className={TD}><Battery value={d.battery} label={`D${d.id} ${t('dr_th_batt')}`} /></td>
                <td className={TDN}>{d.van ? `V${d.van}` : '–'}</td>
                <td className={TDN}>{d.order ? `#${d.order}` : '–'}</td>
                <td className={TDN}>{fmt(d.z, 1)} m</td>
                <td className={TDN}>{fmt(d.drift, 2)} m</td>
                <td className={TDN}>{d.trips}</td>
                <td className={cn(TDN, d.crashes > 0 && 'text-warn')}>{d.crashes}</td>
                <td className={cn(TD, 'min-w-40 whitespace-normal text-fg-mute')}>
                  {d.fail && DOWN.includes(d.status) ? <span className="text-danger">{failReason(t, d.fail)}</span> : d.note}
                </td>
              </tr>
            ))}
          </tbody>
        </TableFrame>
      )}
    </Card>
  )
}

/* ------------------------------------------------------------ pods */
export function PodsTable({ state }: { state: DeliveryState | null }) {
  const t = useT()
  const vans = state?.available ? state.vans ?? [] : []
  return (
    <Card title={t('dr_vans')} actions={<Badge className="num">{vans.length}</Badge>}>
      {!state?.available ? <EmptyState>{t('dr_waiting')}</EmptyState> : !vans.length ? <EmptyState>–</EmptyState> : (
        <TableFrame label={t('dr_vans')} maxH="max-h-[360px]">
          <thead><tr>
            {[t('dr_th_van'), t('dr_th_podbatt'), t('dr_th_state'), t('dr_th_drone'), t('dr_th_pend')].map((h) =>
              <th key={h} scope="col" className={TH}>{h}</th>)}
            <th scope="col" className={TH} title={t('dr_s_deliv')}><span aria-hidden>✓</span><span className="sr-only">{t('dr_s_deliv')}</span></th>
            <th scope="col" className={TH} title={t('dr_s_fail')}><span aria-hidden>✗</span><span className="sr-only">{t('dr_s_fail')}</span></th>
          </tr></thead>
          <tbody>
            {vans.map((v) => (
              <tr key={v.id} className="hover:bg-muted/40">
                <th scope="row" className={cn(TDN, 'text-left font-semibold')}>V{v.id}</th>
                <td className={TD}><Battery value={v.battery} label={`V${v.id} ${t('dr_th_podbatt')}`} /></td>
                <td className={TD}><Badge>{v.state}</Badge></td>
                <td className={TDN}>{v.drone ? `D${v.drone}` : v.incoming ? `→ D${v.incoming}` : '–'}</td>
                <td className={TDN}>{v.pending}</td>
                <td className={cn(TDN, 'text-accent')}>{v.delivered}</td>
                <td className={cn(TDN, v.failed > 0 && 'text-danger')}>{v.failed}</td>
              </tr>
            ))}
          </tbody>
        </TableFrame>
      )}
    </Card>
  )
}

/* ------------------------------------------------------------ events */
const EV_TONE: Record<string, string> = {
  failed: 'text-danger', drone_crash: 'text-danger',
  standby_sent: 'text-violet', drone_arrived: 'text-violet',
  drone_to_station: 'text-warn', drone_charging: 'text-warn', drone_charging_in_van: 'text-warn',
  wind: 'text-info', wind_hold: 'text-info',
  to_charger: 'text-s3', ev_charging: 'text-s3', ev_charged: 'text-s3', batch_declined: 'text-s3', waiting_for_charger: 'text-s3',
  pod_stranded: 'text-s2', orders_returned: 'text-s2', pod_towed: 'text-s2',
  delivered: 'text-accent', drone_repaired: 'text-accent',
}

export function EventsFeed({ state, className }: { state: DeliveryState | null; className?: string }) {
  const t = useT()
  const reduce = useReducedMotion()
  const evs = state?.available ? (state.events ?? []).slice().reverse() : []
  const seen = new Set<string>()
  const rows = evs.map((e) => {
    let k = `${e.t}|${e.event}|${e.van ?? ''}|${e.order ?? ''}|${e.detail}`
    while (seen.has(k)) k += '+'
    seen.add(k)
    return { k, e }
  })
  return (
    <Card title={t('dr_events')} className={className} actions={<Badge className="num">{evs.length}</Badge>}>
      {!state?.available ? <EmptyState>{t('dr_waiting')}</EmptyState> : !rows.length ? <EmptyState>{t('dr_noevents')}</EmptyState> : (
        <ol className="-mx-1 max-h-[360px] overflow-y-auto pr-1" aria-live="off" aria-label={t('dr_events')}>
          <AnimatePresence initial={false}>
            {rows.map(({ k, e }) => (
              <motion.li key={k} layout={!reduce}
                initial={reduce ? false : { opacity: 0, y: -8, scale: 0.98 }}
                animate={{ opacity: 1, y: 0, scale: 1 }}
                exit={{ opacity: 0 }}
                transition={{ duration: 0.25, ease: 'easeOut' }}
                className="grid grid-cols-[3.5rem_minmax(0,auto)_1fr] items-baseline gap-x-2.5 border-b border-line/50 px-1 py-1.5 text-[13px]">
                <span className="num text-xs text-fg-mute">{fmt(e.t, 0)}s</span>
                <span className={cn('num text-xs font-semibold', EV_TONE[e.event] ?? 'text-fg')}>
                  {e.event}{e.van !== '' && e.van != null ? ` V${e.van}` : ''}
                </span>
                <span className="min-w-0 break-words text-fg-2">
                  {e.order ? <span className="num text-fg-mute">#{e.order} </span> : null}{e.detail}
                </span>
              </motion.li>
            ))}
          </AnimatePresence>
        </ol>
      )}
    </Card>
  )
}
