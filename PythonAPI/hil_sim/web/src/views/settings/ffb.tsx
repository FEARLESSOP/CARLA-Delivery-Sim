import { useEffect, useRef, useState } from 'react'
import { Check, Save, X } from 'lucide-react'
import { usePoll } from '@/lib/poll'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_MONO } from '@/lib/colors'
import { useT } from '@/lib/i18n'
import { fmt } from '@/lib/fmt'
import { cn } from '@/lib/utils'
import { Button, Card, EmptyState, TextInput } from '@/components/ui'
import { Chip, Meter, SaveBadge, SchemaRow, SignedMeter, useParamLabel, useSaver } from './parts'
import type { FfbInfo, FfbPost, FfbStatus, Values } from './types'

// Settings > Force feedback: every carla_ffb.py setting as a slider built
// from the /ffb schema (ffb_settings.py), presets, live status and the
// centring-torque curve. Polled every 500 ms; changes are POSTed 150 ms after
// the last move and carla_ffb.py picks them up within half a second.
// 設定 > フォースフィードバック（スキーマから生成、0.5 秒ごとに更新）。

const isLive = (s: FfbStatus | null | undefined): s is FfbStatus => !!s && s.age < 3

/** Steady-state centring torque against steering position (kingpin +
 *  centre spring, through the output stage); the tyre's self-aligning torque
 *  comes on top while cornering. Same maths as the old ffCentring().
 *  舵角に対する定常センタリングトルク（旧 ffCentring と同じ式）。 */
function centring(V: Values, s: number, kmh: number): number {
  const g = (k: string) => V[k] ?? 0
  const ss = (x: number) => { x = Math.max(0, Math.min(1, x)); return x * x * (3 - 2 * x) }
  const fullKmh = g('spring_full_kmh') || 1
  const k = g('spring_standstill') + (1 - g('spring_standstill')) * ss(kmh / fullKmh)
  const spring = -g('center_spring') * k * Math.sign(s) * Math.pow(Math.abs(s), g('spring_linearity'))
  let f = (-g('kpi_nm') * s / (g('torque_full_nm') || 1) + spring) * g('strength')
  let a = Math.abs(f)
  const knee = g('soft_knee')
  if (a > knee && knee < 1) a = knee + (1 - knee) * Math.tanh((a - knee) / (1 - knee))
  f = Math.sign(f) * a
  const mf = g('min_force'), blend = g('min_force_blend')
  f = f * (1 - mf) + mf * (blend > 0 ? Math.tanh(f / blend) : Math.sign(f))
  return Math.max(-1, Math.min(1, f))
}

function CentringCurve({ values, status }: { values: Values; status: FfbStatus | null }) {
  const t = useT()
  const live = isLive(status)
  const kmh = live ? status.kmh : 60
  const ref = useCanvas((ctx, w, h) => {
    const X = (s: number) => 10 + (s + 1) / 2 * (w - 20), Y = (f: number) => h / 2 - f * (h / 2 - 10)
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1
    ctx.beginPath(); ctx.moveTo(X(-1), Y(0)); ctx.lineTo(X(1), Y(0))
    ctx.moveTo(X(0), Y(1)); ctx.lineTo(X(0), Y(-1)); ctx.stroke()
    const draw = (k: number, col: string, dash: number[]) => {
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash(dash); ctx.beginPath()
      for (let i = 0; i <= 120; i++) {
        const x = -1 + i / 60, y = centring(values, x, k)
        if (i === 0) ctx.moveTo(X(x), Y(y)); else ctx.lineTo(X(x), Y(y))
      }
      ctx.stroke(); ctx.setLineDash([])
    }
    draw(0, C.mute, [5, 4])
    draw(kmh, C.info, [])
    ctx.font = `11px ${FONT_MONO}`
    ctx.fillStyle = C.fg
    ctx.fillText(`${Math.round(kmh)} km/h`, 14, 16)
    ctx.fillStyle = C.mute
    ctx.fillText('L', 12, h / 2 - 4); ctx.fillText('R', w - 20, h / 2 - 4)
    if (live) {
      const sx = Math.max(-1, Math.min(1, status.steer))
      ctx.strokeStyle = C.warn; ctx.lineWidth = 1.5
      ctx.beginPath(); ctx.moveTo(X(sx), Y(1)); ctx.lineTo(X(sx), Y(-1)); ctx.stroke()
    }
  }, [values, kmh, live, live ? status.steer : 0])
  return (
    <div>
      <div className="mb-1 text-xs text-fg-mute">{t('ff_curve')}</div>
      <canvas ref={ref} className="block h-44 w-full 2xl:h-52" role="img"
        aria-label={`${t('ff_curve')} · ${Math.round(kmh)} km/h`} />
      <div className="mt-1.5 flex flex-wrap gap-x-4 gap-y-1 text-xs text-fg-mute">
        <span className="inline-flex items-center gap-1.5"><i className="inline-block h-0.5 w-4 rounded bg-info" />{t('ff_lg_now')}</span>
        <span className="inline-flex items-center gap-1.5"><i className="inline-block w-4 border-t-2 border-dashed border-fg-mute" />{t('ff_lg_still')}</span>
        <span className="inline-flex items-center gap-1.5"><i className="inline-block h-3 w-0.5 rounded bg-warn" />{t('ff_lg_wheel')}</span>
      </div>
    </div>
  )
}

function StatusChips({ s }: { s: FfbStatus | null }) {
  const t = useT()
  const live = isLive(s)
  return (
    <div className="flex flex-wrap gap-2" aria-label={t('ff_status')}>
      <Chip k={t('ff_chip_state')} v={!s ? t('ff_offline') : live ? t('ff_running') : t('ff_stale')} tone={live ? 'good' : 'mute'} />
      <Chip k={t('chip_vehicle')} v={s ? s.hero : '-'} />
      <Chip k="km/h" v={live ? fmt(s.kmh, 0) : '-'} />
      <Chip k={t('ff_chip_surface')} v={live ? s.surface : '-'} />
      <Chip k={t('ff_chip_sat')} v={live ? `${Math.round(s.front_sat * 100)}%` : '-'} />
    </div>
  )
}

function Meters({ s }: { s: FfbStatus | null }) {
  const t = useT()
  const live = isLive(s)
  const v = (x: number | undefined) => Math.max(-1, Math.min(1, live ? x ?? 0 : 0))
  const txt = (x: number | undefined) => (live ? `${fmt(v(x) * 100, 0)}%` : '-')
  return (
    <div className="space-y-3">
      <div className="text-xs text-fg-mute">{t('ff_meters')}</div>
      <SignedMeter label={t('ff_m_wheel')} value={v(s?.wheel)} text={txt(s?.wheel)} />
      <SignedMeter label={t('ff_m_out')} value={v(s?.output)} text={txt(s?.output)} />
      <Meter label={t('ff_m_damper')} value={v(s?.damper)} text={txt(s?.damper)} />
      <Meter label={t('ff_m_friction')} value={v(s?.friction)} text={txt(s?.friction)} />
    </div>
  )
}

/** Two-click confirmation (replaces the old confirm() dialogs). / 2 回押しで確定 */
function useArm(ms = 3500) {
  const [armed, setArmed] = useState<string | null>(null)
  useEffect(() => { if (!armed) return; const id = setTimeout(() => setArmed(null), ms); return () => clearTimeout(id) }, [armed, ms])
  return [armed, setArmed] as const
}

export default function FfbSection() {
  const t = useT()
  const label = useParamLabel()
  const [info, setInfo] = useState<FfbInfo | null>(null)
  const [values, setValues] = useState<Values>({})
  const pending = useRef<Values>({})
  const [pname, setPname] = useState('')
  const [pmsg, setPmsg] = useState('')
  const [armDel, setArmDel] = useArm()
  const [armRep, setArmRep] = useArm()

  const saver = useSaver<FfbInfo>('/ffb', (d) => {
    if (!d?.params) return
    setInfo(d)
    if (!saver.editing() && !Object.keys(pending.current).length) setValues(d.values)
  })
  const { busy } = saver
  const { data, error } = usePoll<FfbInfo>('/ffb', 500)
  useEffect(() => {
    if (!data?.params) return
    setInfo(data)
    if (!busy() && !Object.keys(pending.current).length) setValues(data.values)
  }, [data, busy])

  const set = (k: string, v: number) => {
    setValues((o) => ({ ...o, [k]: v }))
    pending.current = { ...pending.current, [k]: v }
    saver.schedule((): FfbPost => { const vals = pending.current; pending.current = {}; return { values: vals } }, 150)
  }
  /** An explicit action: send what waits along with it, take the answer. */
  const act = async (body: FfbPost) => {
    saver.cancel()
    const vals = pending.current
    pending.current = {}
    const r = await saver.post(Object.keys(vals).length && !body.preset ? { values: vals, ...body } : body)
    if (r?.values) setValues(r.values)
    return r
  }
  const savePreset = async () => {
    const name = pname.trim()
    if (!name) { setPmsg(t('ff_pname_need')); return }
    if (info?.user_presets?.includes(name) && armRep !== name) { setArmRep(name); return }
    setArmRep(null)
    const d = await act({ save_preset: name })
    setPmsg(d?.error ? d.error : d ? `${t('ff_psaved')} ${name}` : saver.error ?? t('s_failed'))
    if (d && !d.error) setPname('')
  }
  const delPreset = async (name: string) => {
    if (armDel !== name) { setArmDel(name); return }
    setArmDel(null)
    const d = await act({ delete_preset: name })
    if (d) setPmsg(`${t('ff_pdeleted')}: ${name}`)
  }

  if (!info) return <EmptyState>{error ? t('s_offline') : t('ff_wait')}</EmptyState>
  const s = info.status
  const presetLabel = (n: string) => { const k = `ff_p_${n}`; const v = t(k); return v === k ? n : v }

  return (
    <div className="space-y-4">
      <StatusChips s={s} />
      <Card title={t('ff_card_main')} actions={<SaveBadge state={saver.state} error={saver.error} />} footer={t('ff_hint')}>
        <div className="grid gap-5 xl:grid-cols-[minmax(0,1fr)_minmax(0,1fr)]">
          <div className="space-y-3">
            <div>
              <div className="mb-1.5 text-xs text-fg-mute">{t('ff_presets')}</div>
              <div className="flex flex-wrap gap-2" role="group" aria-label={t('ff_presets')}>
                {info.presets.map((n) => (
                  <Button key={n} size="sm" variant={info.active === n ? 'primary' : 'secondary'} aria-pressed={info.active === n}
                    className={cn(n === 'forza_horizon' && info.active !== n && 'border-info/50 text-info')}
                    onClick={() => act({ preset: n })}>
                    {info.active === n && <Check className="h-3.5 w-3.5" aria-hidden />}{presetLabel(n)}
                  </Button>
                ))}
              </div>
            </div>
            {!!info.user_presets?.length && (
              <div>
                <div className="mb-1.5 text-xs text-fg-mute">{t('ff_user')}</div>
                <div className="flex flex-wrap gap-2" role="group" aria-label={t('ff_user')}>
                  {info.user_presets.map((n) => (
                    <span key={n} className="inline-flex">
                      <Button size="sm" variant={info.active === n ? 'primary' : 'secondary'} aria-pressed={info.active === n}
                        className="rounded-r-none" onClick={() => act({ preset: n })}>
                        {info.active === n && <Check className="h-3.5 w-3.5" aria-hidden />}{n}
                      </Button>
                      <Button size="sm" variant={armDel === n ? 'danger' : 'secondary'} className="rounded-l-none border-l-0 px-2"
                        aria-label={`${t('ff_pdel_q')} "${n}"`} title={t('ff_pdel_q')} onClick={() => delPreset(n)}>
                        {armDel === n ? t('ff_pdel_sure', { p: n }) : <X className="h-3.5 w-3.5" aria-hidden />}
                      </Button>
                    </span>
                  ))}
                </div>
              </div>
            )}
            <form className="flex flex-wrap items-end gap-2" onSubmit={(e) => { e.preventDefault(); void savePreset() }}>
              <div className="min-w-40 flex-1">
                <TextInput label={t('ff_pname_ph')} value={pname} onChange={(v) => { setPname(v.slice(0, 40)); setArmRep(null) }} />
              </div>
              <Button type="submit" variant={armRep ? 'danger' : 'secondary'} className="mb-1">
                <Save className="h-4 w-4" aria-hidden />{armRep ? t('ff_replace_sure', { p: armRep }) : t('ff_psave')}
              </Button>
            </form>
            <div className="min-h-4 text-xs text-fg-mute" role="status" aria-live="polite">{pmsg || t('ff_save_h')}</div>
          </div>
          <div className="space-y-4">
            <CentringCurve values={values} status={s} />
            <Meters s={s} />
          </div>
        </div>
      </Card>
      <div className="grid gap-4 lg:grid-cols-2 2xl:grid-cols-3">
        {info.groups.map((g) => {
          const ps = info.params.filter((p) => p.group === g.key)
          if (!ps.length) return null
          return (
            <Card key={g.key} title={label(g)}>
              <div className="-mt-1">
                {ps.map((p) => (
                  <SchemaRow key={p.key} p={p} value={values[p.key] ?? info.defaults[p.key] ?? p.min}
                    def={info.defaults[p.key]} onChange={(v) => set(p.key, v)} />
                ))}
              </div>
            </Card>
          )
        })}
      </div>
    </div>
  )
}
