import { useEffect, useRef, useState } from 'react'
import { RotateCcw } from 'lucide-react'
import { usePoll } from '@/lib/poll'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { useT } from '@/lib/i18n'
import { Badge, Button, Card, EmptyState } from '@/components/ui'
import { ParamRow, SaveBadge, SchemaRow, useParamLabel, useSaver } from './parts'
import type { PedalTuning, Values, VehicleTuning, VtCar, VtPost } from './types'

// Settings > Driving: pedal curves (/tuning, polled every 5 s so the
// driving window's [ / ] keys show up) and vehicle performance multipliers
// (/vehicle_tuning, polled every second).
// 設定 > 運転：ペダル調整と車両性能。

/* ------------------------------------------------------- pedal tuning */
const TUNE_DEF: PedalTuning = { throttle_max: 0.8, throttle_curve: 1.8, brake_max: 1.0, brake_curve: 2.2 }
const TUNE_ROWS: { key: keyof PedalTuning; label: string; min: number; max: number; step: number; pct?: boolean }[] = [
  { key: 'brake_max', label: 'tune_bmax', min: 0.1, max: 1, step: 0.05, pct: true },
  { key: 'brake_curve', label: 'tune_bcurve', min: 0.5, max: 3, step: 0.1 },
  { key: 'throttle_max', label: 'tune_tmax', min: 0.1, max: 1, step: 0.05, pct: true },
  { key: 'throttle_curve', label: 'tune_tcurve', min: 0.5, max: 3, step: 0.1 },
]

function TuneCurve({ tune }: { tune: PedalTuning }) {
  const t = useT()
  const ref = useCanvas((ctx, w, h) => {
    const L = 8, T = 6, W = w - 14, H = h - 14
    ctx.strokeStyle = C.grid
    ctx.lineWidth = 1
    ctx.strokeRect(L + 0.5, T + 0.5, W, H)
    ctx.setLineDash([2, 4])
    for (const f of [0.25, 0.5, 0.75]) {
      ctx.beginPath(); ctx.moveTo(L + f * W, T); ctx.lineTo(L + f * W, T + H); ctx.stroke()
      ctx.beginPath(); ctx.moveTo(L, T + f * H); ctx.lineTo(L + W, T + f * H); ctx.stroke()
    }
    ctx.setLineDash([])
    const curve = (mx: number, cv: number, col: string, dash: number[]) => {
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash(dash); ctx.beginPath()
      for (let i = 0; i <= 60; i++) {
        const p = i / 60, v = mx * Math.pow(p, cv)
        const x = L + p * W, y = T + H - v * H
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
      }
      ctx.stroke(); ctx.setLineDash([])
    }
    curve(tune.throttle_max, tune.throttle_curve, C.accent, [])
    curve(tune.brake_max, tune.brake_curve, C.danger, [6, 4])
    ctx.fillStyle = C.mute
    ctx.font = `10px ${FONT_MONO}`
    ctx.fillText('100%', L + 4, T + 12)
  }, [tune])
  return (
    <div>
      <div className="mb-1 text-xs text-fg-mute">{t('tune_curve')}</div>
      <canvas ref={ref} className="block h-36 w-full" role="img"
        aria-label={`${t('lbl_throttle')} ${Math.round(tune.throttle_max * 100)}% ^${tune.throttle_curve}, ${t('lbl_brake')} ${Math.round(tune.brake_max * 100)}% ^${tune.brake_curve}`} />
      <div className="mt-1.5 flex flex-wrap items-center gap-x-4 gap-y-1 text-xs text-fg-mute">
        <span className="inline-flex items-center gap-1.5"><i className="inline-block h-0.5 w-4 rounded bg-accent" />{t('lbl_throttle')}</span>
        <span className="inline-flex items-center gap-1.5"><i className="inline-block w-4 border-t-2 border-dashed border-danger" />{t('lbl_brake')}</span>
        <span>{t('tune_hint')}</span>
      </div>
    </div>
  )
}

function PedalCard() {
  const t = useT()
  const [tune, setTune] = useState<PedalTuning>(TUNE_DEF)
  const ref = useRef(tune)
  ref.current = tune
  const saver = useSaver<PedalTuning>('/tuning', () => { /* the old page ignored the answer too */ })
  const { data } = usePoll<Partial<PedalTuning>>('/tuning', 5000)
  const { busy } = saver
  useEffect(() => { if (data && !busy()) setTune({ ...TUNE_DEF, ...data }) }, [data, busy])

  const update = (next: PedalTuning) => {
    ref.current = next
    setTune(next)
    saver.schedule(() => ref.current, 250)
  }
  return (
    <Card title={t('card_tuning')}
      actions={<>
        <SaveBadge state={saver.state} error={saver.error} />
        <Button size="sm" variant="ghost" onClick={() => update({ ...TUNE_DEF })}><RotateCcw className="h-3.5 w-3.5" aria-hidden />{t('tune_reset')}</Button>
      </>}>
      <p className="-mt-1 text-xs text-fg-mute">{t('tune_h')}</p>
      <div className="grid gap-x-6 gap-y-2 lg:grid-cols-2">
        <div>
          {TUNE_ROWS.map((r) => (
            <ParamRow key={r.key} label={t(r.label)} value={tune[r.key]} def={TUNE_DEF[r.key]}
              min={r.min} max={r.max} step={r.step}
              format={r.pct ? (v) => `${Math.round(v * 100)}%` : (v) => v.toFixed(1)}
              onChange={(v) => update({ ...ref.current, [r.key]: v })} />
          ))}
        </div>
        <TuneCurve tune={tune} />
      </div>
    </Card>
  )
}

/* ------------------------------------------------ vehicle performance */
function CarsTable({ cars }: { cars: VtCar[] }) {
  const t = useT()
  const cell = (now: number, base: number, unit: string) => (
    <td className="num px-2 py-1.5 text-right whitespace-nowrap">
      {now} <span className="text-fg-mute">{unit}</span>
      {now !== base && <span className="ml-1 text-fg-mute">({base})</span>}
    </td>
  )
  return (
    <div className="-mx-4 overflow-x-auto px-4">
      <table className="w-full min-w-[520px] border-collapse text-sm">
        <caption className="mb-1 text-left text-xs text-fg-mute">{t('vt_cars')}</caption>
        <thead>
          <tr className="border-b border-line text-xs text-fg-mute">
            <th scope="col" className="px-2 py-1.5 text-left font-medium">{t('vt_car')}</th>
            <th scope="col" className="px-2 py-1.5 text-right font-medium">{t('vt_brake')}</th>
            <th scope="col" className="px-2 py-1.5 text-right font-medium">{t('vt_torque')}</th>
            <th scope="col" className="px-2 py-1.5 text-right font-medium">{t('vt_top')}</th>
          </tr>
        </thead>
        <tbody>
          {cars.length ? cars.map((c) => (
            <tr key={c.id} className={c.hero ? 'border-b border-line/50 bg-accent/5' : 'border-b border-line/50'}>
              <td className="px-2 py-1.5">
                <span className="mr-1.5">{c.type.replace('vehicle.', '').replace('delivery.', '')}</span>
                {c.hero && <Badge tone="good">{t('vt_me')}</Badge>}
                {c.waiting && <Badge tone="warn" title={t('vt_wait_tip')}>{t('vt_wait')}</Badge>}
              </td>
              {cell(c.brake_nm, c.brake_base, 'Nm')}
              {cell(c.torque_nm, c.torque_base, 'Nm')}
              {cell(c.top_kmh, c.top_base, 'km/h')}
            </tr>
          )) : (
            <tr><td colSpan={4} className="px-2 py-3 text-center text-fg-mute">{t('vt_none')}</td></tr>
          )}
        </tbody>
      </table>
    </div>
  )
}

function VehicleCard() {
  const t = useT()
  const label = useParamLabel()
  const [info, setInfo] = useState<VehicleTuning | null>(null)
  const [values, setValues] = useState<Values>({})
  const ref = useRef(values)
  ref.current = values
  const saver = useSaver<VehicleTuning>('/vehicle_tuning', (d) => {
    if (!d?.params) return
    setInfo(d)
    if (!saver.editing()) setValues(d.values)
  })
  const { data, error } = usePoll<VehicleTuning>('/vehicle_tuning', 1000)
  const { busy } = saver
  useEffect(() => {
    if (!data?.params) return
    setInfo(data)
    if (!busy()) setValues(data.values)
  }, [data, busy])

  const set = (k: string, v: number) => {
    const next = { ...ref.current, [k]: v }
    ref.current = next
    setValues(next)
    saver.schedule((): VtPost => ({ values: ref.current }), 200)
  }
  const explicit = async (body: VtPost) => {
    saver.cancel()
    const r = await saver.post(body)
    if (r?.values) setValues(r.values)
  }
  const fmtVt = (step: number) => (v: number) => (step >= 1 ? String(v) : `×${Number(v).toFixed(2)}`)
  const groups = info?.groups?.length ? info.groups : [{ key: '', en: '' }]

  return (
    <Card title={t('vt_card')}
      actions={<>
        <SaveBadge state={saver.state} error={saver.error} />
        <Button size="sm" variant="ghost" disabled={!info} onClick={() => explicit({ neutral: true })}>{t('vt_stock')}</Button>
        <Button size="sm" variant="ghost" disabled={!info} onClick={() => explicit({ reset: true })}><RotateCcw className="h-3.5 w-3.5" aria-hidden />{t('tune_reset')}</Button>
      </>}
      footer={t('vt_hint')}>
      <p className="-mt-1 text-xs text-fg-mute">{t('vt_h')}</p>
      {!info ? <EmptyState>{error ? t('s_offline') : t('s_loading')}</EmptyState> : (
        <>
          <div className="grid gap-x-6 gap-y-3 lg:grid-cols-2">
            {groups.map((g) => {
              const ps = info.params.filter((p) => !g.key || p.group === g.key)
              if (!ps.length) return null
              return (
                <div key={g.key}>
                  {g.key && <h4 className="mb-1 text-xs font-semibold text-fg-2">{label(g)}</h4>}
                  {ps.map((p) => (
                    <SchemaRow key={p.key} p={p} value={values[p.key] ?? info.defaults[p.key] ?? p.min}
                      def={info.defaults[p.key]} format={fmtVt(p.step)} onChange={(v) => set(p.key, v)} />
                  ))}
                </div>
              )
            })}
          </div>
          <CarsTable cars={info.status?.cars ?? []} />
        </>
      )}
    </Card>
  )
}

export default function DrivingSection() {
  return (
    <div className="grid items-start gap-4 2xl:grid-cols-2">
      <PedalCard />
      <VehicleCard />
    </div>
  )
}
