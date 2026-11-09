// One card per sensor of the selected drone: live values with units, a health
// stripe and badge (glyph + text), what the hardware is, a sparkline where a
// trend helps, and "Show on drone" (selects the part on the 3D model).
// センサーごとのカード：値・単位・健全性・機器の説明・推移・図で表示
import type { ReactNode } from 'react'
import { Crosshair } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Bar, Button, KV, Stat, type Tone } from '@/components/ui'
import { HEALTH_MARK, HEALTH_TONE, cardDomId, f, type CardId, type SensorId } from './catalog'
import { Spark, type HistKey } from './history'
import type { DroneS, Health, Sensors } from './types'

export function HealthBadge({ h }: { h: Health | undefined }) {
  const t = useT()
  if (!h) return null
  return <Badge tone={HEALTH_TONE[h]}><span aria-hidden>{HEALTH_MARK[h]}</span>{t(`dn_h_${h}`)}</Badge>
}

/** The frame every sensor / camera card shares. / 共通のカード枠 */
export function SensorFrame({ id, health, selected, onLocate, children, extra }: {
  id: CardId; health: Health | undefined; selected: boolean; onLocate: (id: CardId) => void
  children: ReactNode; extra?: ReactNode
}) {
  const t = useT()
  return (
    <section id={cardDomId(id)} tabIndex={-1} aria-labelledby={`${cardDomId(id)}-h`}
      className={cn('relative flex min-w-0 scroll-mt-24 flex-col gap-2 overflow-hidden rounded-[var(--radius-card)] border bg-card/80 p-4 pt-5 shadow-[var(--shadow-card)] outline-none transition-colors duration-200',
        selected ? 'border-accent ring-2 ring-accent/40' : 'border-line',
        health === 'fail' && !selected && 'border-danger/60', health === 'warn' && !selected && 'border-warn/50')}>
      {/* health stripe (the badge says it in words too) / 健全性の帯 */}
      <i aria-hidden className={cn('absolute inset-x-0 top-0 h-1',
        health === 'fail' ? 'bg-danger' : health === 'warn' ? 'bg-warn' : health === 'ok' ? 'bg-accent/70' : 'bg-line')} />
      <header className="flex items-start justify-between gap-2">
        <div className="min-w-0">
          <h3 id={`${cardDomId(id)}-h`} className="text-sm font-semibold text-fg">{t(`dn_s_${id}`)}</h3>
          <p className="text-[11px] leading-snug text-fg-mute">{t(`dn_hw_${id}`)}</p>
        </div>
        <div className="flex shrink-0 items-center gap-1">
          <HealthBadge h={health} />
          <Button variant="ghost" size="sm" className="h-7 w-7 px-0" aria-label={`${t('dn_locate')}: ${t(`dn_s_${id}`)}`}
            title={t('dn_locate')} onClick={() => onLocate(id)}>
            <Crosshair className="h-3.5 w-3.5" aria-hidden />
          </Button>
        </div>
      </header>
      {children}
      {extra}
    </section>
  )
}

/** a label over a value and its unit, small / 小さな値表示 */
function Mini({ k, v, unit, tone }: { k: ReactNode; v: ReactNode; unit?: string; tone?: Tone }) {
  const c = tone === 'bad' ? 'text-danger' : tone === 'warn' ? 'text-warn' : tone === 'good' ? 'text-accent' : 'text-fg'
  return (
    <div className="min-w-0">
      <div className="truncate text-[11px] text-fg-mute">{k}</div>
      <div className={cn('num text-sm', c)}>{v}{unit && <span className="ml-0.5 text-[11px] text-fg-mute">{unit}</span>}</div>
    </div>
  )
}

function Trend({ label, data, cls, min, max }: { label: string; data: number[]; cls?: string; min?: number; max?: number }) {
  return (
    <div>
      <div className="text-[10px] uppercase tracking-wider text-fg-mute">{label}</div>
      <Spark data={data} className={cls} min={min} max={max} />
    </div>
  )
}

const dist = (v: number | null | undefined, max?: number) =>
  v === null || v === undefined ? (max ? `> ${max}` : '–') : f(v, 1)

export interface CardProps {
  d: DroneS
  s: Sensors
  hist: (k: HistKey) => number[]
  selected: CardId | null
  onLocate: (id: CardId) => void
}

/** All sensor cards in page order. / 全センサーカード */
export function SensorCards(p: CardProps) {
  const t = useT()
  const { s, hist } = p
  const card = (id: SensorId, body: ReactNode, extra?: ReactNode) => (
    <SensorFrame key={id} id={id} health={(s[id] as { health?: Health } | undefined)?.health}
      selected={p.selected === id} onLocate={p.onLocate} extra={extra}>{body}</SensorFrame>
  )
  const g = s.gnss ?? {}, m = s.mag ?? {}, imu = s.imu ?? {}, b = s.baro ?? {}, r = s.rangefinder ?? {}
  const fl = s.flow ?? {}, o = s.obstacle ?? {}, a = s.air ?? {}, bt = s.battery ?? {}, mo = s.motors ?? {}
  const w = s.winch ?? {}, l = s.link ?? {}, rid = s.remote_id ?? {}, ch = s.parachute ?? {}
  const near = (v: number | null | undefined): Tone => (v !== null && v !== undefined && v < 5 ? 'bad' : v !== null && v !== undefined && v < 10 ? 'warn' : 'neutral')
  const left = bt.left_s
  return (
    <>
      {card('gnss', <>
        <div className="flex items-end justify-between gap-2">
          <Stat size="md" label={t('dn_g_fix')} value={t(`dn_fix_${g.fix ?? 'none'}`)}
            tone={g.fix === 'rtk_fixed' ? 'good' : g.fix === 'rtk_float' ? 'info' : g.fix === '3d' ? 'warn' : 'neutral'} />
          <Mini k={t('dn_g_sats')} v={g.sats ?? '–'} />
          <Mini k="HDOP" v={f(g.hdop, 2)} />
        </div>
        <KV k={t('dn_g_lat')} v={`${f(g.lat, 7)}°`} />
        <KV k={t('dn_g_lon')} v={`${f(g.lon, 7)}°`} />
        <KV k={t('dn_g_alt')} v={`${f(g.alt, 2)} m`} />
        <KV k={t('dn_g_acc')} v={g.acc_h ? `± ${g.acc_h < 0.1 ? (g.acc_h * 100).toFixed(0) + ' cm' : f(g.acc_h, 2) + ' m'}` : '–'} />
        <KV k={t('dn_speed')} v={`${f(g.speed, 1)} m/s`} />
      </>, <Trend label={t('dn_speed')} data={hist('speed')} cls="stroke-s1" min={0} />)}

      {card('mag', <>
        <div className="flex items-center gap-4">
          <Compass deg={m.heading} />
          <div className="flex-1">
            <Stat size="md" label={t('dn_heading')} value={`${f(m.heading, 0)}°`} />
            <KV k={t('dn_m_field')} v={`${f(m.field, 1)} µT`} />
            <KV k={t('dn_m_interf')} v={m.interference ? t('dn_yes') : t('dn_no')} tone={m.interference ? 'warn' : 'neutral'} />
          </div>
        </div>
      </>)}

      {card('imu', <>
        <div className="grid grid-cols-3 gap-2">
          <Mini k={t('dn_i_ax')} v={f(imu.ax, 2)} unit="m/s²" />
          <Mini k={t('dn_i_ay')} v={f(imu.ay, 2)} unit="m/s²" />
          <Mini k={t('dn_i_az')} v={f(imu.az, 2)} unit="m/s²" />
          <Mini k={t('dn_i_gx')} v={f(imu.gx, 1)} unit="°/s" />
          <Mini k={t('dn_i_gy')} v={f(imu.gy, 1)} unit="°/s" />
          <Mini k={t('dn_i_gz')} v={f(imu.gz, 1)} unit="°/s" />
        </div>
        <div className="flex items-center gap-3">
          <Horizon roll={imu.roll ?? 0} pitch={imu.pitch ?? 0} />
          <div className="flex-1">
            <KV k={t('dn_i_roll')} v={`${f(imu.roll, 1)}°`} />
            <KV k={t('dn_i_pitch')} v={`${f(imu.pitch, 1)}°`} />
            <KV k={t('dn_i_vibe')} v={`${f(imu.vibe, 2)} m/s²`} tone={(imu.vibe ?? 0) > 3 ? 'warn' : 'neutral'} />
          </div>
        </div>
      </>, <Trend label={t('dn_i_vibe')} data={hist('vibe')} cls="stroke-s5" min={0} />)}

      {card('baro', <>
        <Stat size="md" label={t('dn_b_alt')} value={f(b.alt, 1)} unit="m" />
        <KV k={t('dn_b_press')} v={`${f(b.pressure, 2)} hPa`} />
        <KV k={t('dn_b_vs')} v={`${f(b.vs, 1)} m/s`} />
        <KV k={t('dn_b_temp')} v={`${f(b.temp, 1)} °C`} />
      </>, <Trend label={t('dn_b_alt')} data={hist('alt')} cls="stroke-s4" />)}

      {card('rangefinder', <>
        <Stat size="md" label={t('dn_agl')} value={r.in_range ? f(r.agl, 2) : `> ${r.max ?? 40}`} unit="m"
          sub={r.in_range ? undefined : t('dn_r_out', { m: f(r.ground, 0) })} />
        <Bar value={r.in_range && r.agl != null ? r.agl / (r.max || 40) : 1} tone={r.in_range ? 'info' : 'neutral'} label={t('dn_agl')} />
        <KV k={t('dn_r_range')} v={`0.2 – ${r.max ?? 40} m`} />
      </>, <Trend label={t('dn_agl')} data={hist('agl')} cls="stroke-s1" min={0} />)}

      {card('flow', <>
        <div className="grid grid-cols-2 gap-2">
          <Mini k={t('dn_f_vx')} v={f(fl.vx, 2)} unit="m/s" />
          <Mini k={t('dn_f_vy')} v={f(fl.vy, 2)} unit="m/s" />
        </div>
        <KV k={t('dn_f_q')} v={`${fl.quality ?? 0} / 255`} tone={(fl.quality ?? 0) < 60 ? 'warn' : 'neutral'} />
        <Bar value={(fl.quality ?? 0) / 255} tone={(fl.quality ?? 0) > 60 ? 'good' : 'warn'} label={t('dn_f_q')} />
        <p className="text-[11px] text-fg-mute">{t('dn_f_help')}</p>
      </>)}

      {card('obstacle', <>
        {(['front', 'rear', 'down'] as const).map((k) => {
          const v = o[k]
          const max = o.max ?? 30
          return (
            <div key={k}>
              <div className="flex items-baseline justify-between text-sm">
                <span className="text-fg-mute">{t(`dn_o_${k}`)}</span>
                <span className={cn('num', near(v) === 'bad' ? 'text-danger' : near(v) === 'warn' ? 'text-warn' : 'text-fg')}>
                  {dist(v, k === 'down' ? undefined : max)} m
                </span>
              </div>
              <Bar value={v == null ? 0 : 1 - Math.min(v, max) / max} tone={near(v) === 'neutral' ? 'info' : near(v)} label={t(`dn_o_${k}`)} />
            </div>
          )
        })}
        <p className="text-[11px] text-fg-mute">{t('dn_o_help')}</p>
      </>, <Trend label={t('dn_o_near')} data={hist('near')} cls="stroke-s2" min={0} />)}

      {card('air', <>
        <div className="grid grid-cols-2 gap-2">
          <Stat size="md" label={t('dn_a_air')} value={f(a.airspeed, 1)} unit="m/s" />
          <Stat size="md" label={t('dn_a_est')} value={f(a.est, 1)} unit="m/s" sub={`→ ${f(a.est_dir, 0)}°`} />
        </div>
        <KV k={t('dn_a_true')} v={`${f(a.wind, 1)} m/s → ${f(a.wind_dir, 0)}°`} />
        <KV k={t('dn_drift')} v={`${f(a.drift, 2)} m`} />
        <KV k={t('dn_a_limit')} v={`${f(a.limit, 0)} m/s`} />
        <Bar value={(a.wind ?? 0) / (a.limit || 11)} tone={a.saturated ? 'bad' : (a.wind ?? 0) > 0.8 * (a.limit || 11) ? 'warn' : 'good'} label={t('dn_a_limit')} />
        {a.saturated && <p className="text-xs text-danger">{t('dn_a_sat')}</p>}
      </>, <Trend label={t('dn_a_true')} data={hist('wind')} cls="stroke-s3" min={0} />)}

      {card('battery', <>
        <div className="flex items-end justify-between gap-2">
          <Stat label={t('dn_bt_soc')} value={f(bt.soc, 0)} unit="%" tone={(bt.soc ?? 0) > 50 ? 'good' : (bt.soc ?? 0) > 25 ? 'warn' : 'bad'} />
          <Mini k={t('dn_bt_left')} v={left == null ? '–' : `${Math.floor(left / 60)}:${String(Math.round(left % 60)).padStart(2, '0')}`} />
        </div>
        <Bar value={(bt.soc ?? 0) / 100} tone={(bt.soc ?? 0) > 50 ? 'good' : (bt.soc ?? 0) > 25 ? 'warn' : 'bad'} label={t('dn_bt_soc')} />
        <div className="grid grid-cols-2 gap-x-4">
          <KV k={t('dn_bt_v')} v={`${f(bt.voltage, 2)} V`} />
          <KV k={t('dn_bt_cell')} v={`${f(bt.cell, 3)} V`} />
          <KV k={t('dn_bt_i')} v={`${f(bt.current, 1)} A`} tone={bt.charging ? 'info' : 'neutral'} />
          <KV k={t('dn_bt_p')} v={`${f(bt.power, 0)} W`} />
          <KV k={t('dn_bt_t')} v={`${f(bt.temp, 1)} °C`} tone={(bt.temp ?? 0) > 60 ? 'warn' : 'neutral'} />
          <KV k={t('dn_bt_pack')} v={`${bt.cells ?? 6}S ${bt.capacity_ah ?? 10} Ah`} />
        </div>
        {bt.charging && <p className="text-xs text-info">{t('dn_bt_charging')}</p>}
      </>, <div className="grid grid-cols-2 gap-3">
        <Trend label={t('dn_bt_soc')} data={hist('soc')} cls="stroke-accent" min={0} max={100} />
        <Trend label={t('dn_bt_i')} data={hist('current')} cls="stroke-s3" />
      </div>)}

      {card('motors', <>
        <table className="w-full text-[13px]">
          <caption className="sr-only">{t('dn_s_motors')}</caption>
          <thead><tr className="text-left text-[11px] text-fg-mute">
            <th scope="col" className="font-normal">#</th><th scope="col" className="font-normal">{t('dn_mo_rpm')}</th>
            <th scope="col" className="w-1/4 font-normal">{t('dn_mo_thr')}</th><th scope="col" className="font-normal">A</th>
            <th scope="col" className="font-normal">°C</th>
          </tr></thead>
          <tbody>
            {[0, 1, 2, 3].map((i) => {
              const dead = mo.dead === i + 1
              return (
                <tr key={i} className={cn(dead && 'text-danger')}>
                  <th scope="row" className="py-0.5 text-left font-normal text-fg-mute">M{i + 1} <span className="text-[10px]">{t(`dn_mo_pos${i + 1}`)}</span></th>
                  <td className="num">{dead ? t('dn_mo_dead') : f(mo.rpm?.[i], 0)}</td>
                  <td className="pr-2"><Bar value={(mo.throttle?.[i] ?? 0) / 100} tone={dead ? 'bad' : (mo.throttle?.[i] ?? 0) > 92 ? 'warn' : 'good'} label={`M${i + 1} ${t('dn_mo_thr')}`} /></td>
                  <td className="num">{f(mo.current?.[i], 1)}</td>
                  <td className={cn('num', (mo.temp?.[i] ?? 0) > 85 && 'text-warn')}>{f(mo.temp?.[i], 0)}</td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </>, <Trend label={t('dn_mo_avg')} data={hist('rpm')} cls="stroke-s1" min={0} />)}

      {card('winch', <>
        <div className="flex items-end justify-between gap-2">
          <Stat size="md" label={t('dn_w_state')} value={t(`dn_w_${w.state ?? 'idle'}`)}
            tone={w.state === 'lowering' ? 'info' : w.state === 'released' ? 'good' : 'neutral'} />
          <Mini k={t('dn_w_payload')} v={w.payload ? f(w.payload, 2) : '–'} unit="kg" />
        </div>
        <KV k={t('dn_w_tether')} v={`${f(w.tether, 2)} m`} />
        <Bar value={(w.tether ?? 0) / (w.max_tether || 20)} tone="info" label={t('dn_w_tether')} />
        <KV k={t('dn_w_speed')} v={`${f(w.speed, 2)} m/s`} />
        <KV k={t('dn_w_load')} v={`${f(w.load, 1)} N (${f((w.load ?? 0) / 9.81, 2)} kg)`} />
      </>, <Trend label={t('dn_w_tether')} data={hist('tether')} cls="stroke-s4" min={0} />)}

      {card('link', <>
        <div className="flex items-end justify-between gap-2">
          <Stat size="md" label="RSSI" value={l.mode === 'wired' ? t('dn_l_wired') : f(l.rssi, 0)} unit={l.mode === 'wired' ? undefined : 'dBm'}
            tone={l.mode === 'wired' ? 'neutral' : (l.rssi ?? -120) > -70 ? 'good' : (l.rssi ?? -120) > -85 ? 'warn' : 'bad'} />
          <Badge tone={l.mode === 'lte' ? 'warn' : 'neutral'}>{t(`dn_l_${l.mode ?? 'c2'}`)}</Badge>
        </div>
        {l.mode !== 'wired' && <Bar value={((l.rssi ?? -100) + 100) / 60} tone={(l.rssi ?? -120) > -70 ? 'good' : 'warn'} label="RSSI" />}
        <div className="grid grid-cols-2 gap-x-4">
          <KV k="SNR" v={`${f(l.snr, 0)} dB`} />
          <KV k={t('dn_l_lat')} v={`${f(l.latency, 0)} ms`} />
          <KV k={t('dn_l_loss')} v={`${f(l.loss, 1)} %`} tone={(l.loss ?? 0) > 5 ? 'warn' : 'neutral'} />
          <KV k={t('dn_l_dist')} v={l.dist == null ? '–' : `${f(l.dist, 0)} m`} />
          <KV k="LTE" v={l.lte == null ? '–' : `${f(l.lte, 0)} dBm`} />
        </div>
      </>, <Trend label="RSSI" data={hist('rssi')} cls="stroke-s5" />)}

      {card('remote_id', <>
        <KV k={t('dn_rid_bc')} v={rid.broadcast ? t('dn_rid_on') : t('dn_rid_off')} tone={rid.broadcast ? 'good' : 'neutral'} />
        <KV k={t('dn_rid_serial')} v={rid.serial ?? '–'} />
        <KV k={t('dn_rid_traffic')} v={rid.traffic ? String(rid.traffic) : t('dn_rid_none')} />
        <p className="text-[11px] text-fg-mute">{t('dn_rid_help')}</p>
      </>)}

      {card('parachute', <>
        <Stat size="md" label={t('dn_c_state')} value={t(`dn_c_${ch.state ?? 'disarmed'}`)}
          tone={ch.state === 'armed' ? 'good' : ch.state === 'deployed' ? 'warn' : ch.state === 'too_low' ? 'bad' : 'neutral'} />
        <KV k={t('dn_c_min')} v={`${f(ch.min_agl, 0)} m`} />
        <p className="text-[11px] text-fg-mute">{t('dn_c_help')}</p>
      </>)}
    </>
  )
}

/** Compass card: north up, the drone's heading as an arrow. / 方位計 */
function Compass({ deg }: { deg: number | undefined }) {
  const t = useT()
  const a = deg ?? 0
  return (
    <svg viewBox="-50 -50 100 100" className="h-20 w-20 shrink-0" role="img" aria-label={`${t('dn_heading')} ${Math.round(a)}°`}>
      <circle r="44" className="fill-surface stroke-line" strokeWidth="2" />
      {['N', 'E', 'S', 'W'].map((c, i) => {
        const r = (i * Math.PI) / 2
        return <text key={c} x={Math.sin(r) * 34} y={-Math.cos(r) * 34} textAnchor="middle" dominantBaseline="central"
          className={cn('text-[12px]', c === 'N' ? 'fill-danger font-semibold' : 'fill-fg-mute')}>{c}</text>
      })}
      <g transform={`rotate(${a})`}>
        <path d="M0 -26 L7 8 L0 3 L-7 8 Z" className="fill-accent" />
      </g>
    </svg>
  )
}

/** Attitude indicator: the horizon tilts with roll and moves with pitch. / 姿勢指示器 */
function Horizon({ roll, pitch }: { roll: number; pitch: number }) {
  const t = useT()
  const y = Math.max(-40, Math.min(40, pitch * 1.2))
  return (
    <svg viewBox="-40 -40 80 80" className="h-20 w-20 shrink-0" role="img"
      aria-label={`${t('dn_i_roll')} ${roll.toFixed(0)}°, ${t('dn_i_pitch')} ${pitch.toFixed(0)}°`}>
      <defs><clipPath id="dn-hz"><circle r="36" /></clipPath></defs>
      <g clipPath="url(#dn-hz)">
        <g transform={`rotate(${-roll}) translate(0 ${y})`}>
          <rect x="-80" y="-80" width="160" height="80" className="fill-info/40" />
          <rect x="-80" y="0" width="160" height="80" className="fill-warn/30" />
          <line x1="-80" x2="80" y1="0" y2="0" className="stroke-fg" strokeWidth="1" />
        </g>
      </g>
      <circle r="36" className="fill-none stroke-line" strokeWidth="2" />
      <path d="M-18 0 H-6 L0 5 L6 0 H18" className="fill-none stroke-accent" strokeWidth="2.5" />
    </svg>
  )
}
