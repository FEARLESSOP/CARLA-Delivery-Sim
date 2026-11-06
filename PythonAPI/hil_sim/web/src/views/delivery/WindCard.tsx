// Wind: compass (click to pick the gust direction), readouts and the wind
// commands for delivery_sim (POST /delivery/wind: gust, wind, gust_rate, calm).
// 風：コンパス（クリックで方向）、表示、風コマンド
import { useState, type MouseEvent as ReactMouseEvent } from 'react'
import { Navigation, Wind as WindIcon } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { useCanvas } from '@/lib/canvas'
import { C, FONT_SANS } from '@/lib/colors'
import { fmt } from '@/lib/fmt'
import { Badge, Button, Card, KV, Slider } from '@/components/ui'
import { MC, sendCmd } from './shared'
import type { DeliveryState, WindCmd } from './types'

function arrow(ctx: CanvasRenderingContext2D, x: number, y: number, dx: number, dy: number, col: string, w: number) {
  const a = Math.atan2(dy, dx), L = Math.hypot(dx, dy)
  if (L < 1) return
  ctx.strokeStyle = ctx.fillStyle = col; ctx.lineWidth = w
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke()
  const h = Math.min(10, L * 0.4)
  ctx.beginPath()
  ctx.moveTo(x + dx, y + dy)
  ctx.lineTo(x + dx - h * Math.cos(a - 0.45), y + dy - h * Math.sin(a - 0.45))
  ctx.lineTo(x + dx - h * Math.cos(a + 0.45), y + dy - h * Math.sin(a + 0.45))
  ctx.closePath(); ctx.fill()
}

export default function WindCard({ state, className }: { state: DeliveryState | null; className?: string }) {
  const t = useT()
  const w = state?.wind ?? {}
  const live = !!(state?.available && state.running)
  const [speed, setSpeed] = useState(14)
  const [dirSel, setDirSel] = useState(0)
  const [dirTouched, setDirTouched] = useState(false)
  const [dur, setDur] = useState(6)
  const [base, setBase] = useState(3)
  const [rate, setRate] = useState(0.5)
  const [msg, setMsg] = useState('')
  const [busy, setBusy] = useState(false)
  // the direction follows the base wind until the user picks one / 選ぶまで基本風向に追従
  const dir = dirTouched ? dirSel : Math.round(w.base_dir || 0)
  const pickDir = (d: number) => { setDirSel(d); setDirTouched(true) }

  const send = async (cmd: WindCmd) => {
    setBusy(true)
    const j = await sendCmd('/delivery/wind', cmd)
    setBusy(false)
    setMsg(j.ok ? `${t('dr_sent')}: ${JSON.stringify(j.sent)}` : (j.error || 'error'))
  }

  const ref = useCanvas((ctx, W, H) => {
    ctx.fillStyle = MC.oldGround; ctx.fillRect(0, 0, W, H)
    const cx = W / 2, cy = H / 2, R0 = Math.min(W, H) / 2 - 14
    ctx.strokeStyle = MC.compassEdge; ctx.lineWidth = 1
    for (const k of [0.33, 0.66, 1]) { ctx.beginPath(); ctx.arc(cx, cy, R0 * k, 0, 7); ctx.stroke() }
    ctx.fillStyle = C.mute; ctx.font = `11px ${FONT_SANS}`; ctx.textAlign = 'center'
    const ticks: [string, number, number][] = [['0°', R0 + 8, 0], ['90°', 0, R0 + 9], ['180°', -R0 - 9, 0], ['270°', 0, -R0 - 5]]
    for (const [s, dx, dy] of ticks) ctx.fillText(s, cx + dx, cy + dy + 3)
    // the chosen gust direction / 選択中の突風方向
    const ga = dir * Math.PI / 180
    ctx.setLineDash([4, 4]); ctx.strokeStyle = `${MC.pod}88`; ctx.lineWidth = 1.5
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R0 * Math.cos(ga), cy + R0 * Math.sin(ga)); ctx.stroke()
    ctx.setLineDash([])
    // the wind now: 15 m/s reaches the rim / 現在の風（15 m/s で外周）
    const L = Math.min(R0, R0 * (w.speed || 0) / 15), a = (w.dir || 0) * Math.PI / 180
    arrow(ctx, cx - L / 2 * Math.cos(a), cy - L / 2 * Math.sin(a), L * Math.cos(a), L * Math.sin(a),
      w.gust ? MC.gust : MC.windArrow, 4)
    // the drones' limit (11 m/s) / ドローンが抗える限界
    ctx.strokeStyle = `${MC.fail}55`; ctx.setLineDash([2, 3])
    ctx.beginPath(); ctx.arc(cx, cy, R0 * 11 / 15, 0, 7); ctx.stroke(); ctx.setLineDash([])
  }, [w.speed, w.dir, w.gust, dir])

  const onCompass = (ev: ReactMouseEvent<HTMLCanvasElement>) => {
    const r = ev.currentTarget.getBoundingClientRect()
    const a = Math.atan2(ev.clientY - r.top - r.height / 2, ev.clientX - r.left - r.width / 2)
    pickDir(Math.round((a * 180 / Math.PI + 360) % 360))
  }

  const gustTone = w.grounded ? 'bad' : w.gust ? 'warn' : 'good'
  const gustText = (w.grounded ? `${t('dr_grounded')} · ` : '') + (w.gust ? `${t('dr_gust')} ${fmt(w.gust_peak, 0)} m/s` : t('dr_steady'))

  return (
    <Card className={className} title={<span className="inline-flex items-center gap-2"><WindIcon className="h-3.5 w-3.5" aria-hidden />{t('dr_wind')}</span>}
      actions={state?.available ? <Badge tone={gustTone} dot>{gustText}</Badge> : <Badge>–</Badge>}>
      <div className="@container">
      <div className="grid gap-4 @md:grid-cols-[minmax(0,200px)_1fr] @md:items-center">
        <canvas ref={ref} onClick={onCompass} aria-label={t('dr_wind_hint')} role="img"
          className="mx-auto aspect-square w-full max-w-[200px] cursor-crosshair rounded-xl border border-line" />
        <div>
          <KV k={t('dr_wind_now')} v={`${fmt(w.speed, 1)} m/s → ${Math.round(w.dir || 0)}°`} tone={w.gust ? 'warn' : 'neutral'} />
          <KV k={t('dr_wind_base')} v={`${fmt(w.base_speed, 1)} m/s → ${Math.round(w.base_dir || 0)}°`} />
          <KV k={t('dr_wind_rate')} v={`${fmt(w.gust_rate, 2)} / min`} />
          <KV k={t('dr_wind_hold')} v={w.hold ? `${fmt(w.hold, 0)} m/s${w.grounded ? ` — ${t('dr_grounded')}` : ''}` : '–'}
            tone={w.grounded ? 'bad' : 'neutral'} />
        </div>
      </div>
      </div>

      <fieldset className="rounded-xl border border-line bg-surface/50 p-3">
        <legend className="px-1 text-xs font-medium text-fg-mute">{t('dr_gust_grp')}</legend>
        <Slider label={t('dr_ctl_speed')} value={speed} min={4} max={25} step={0.5} onChange={setSpeed} />
        <Slider label={t('dr_ctl_dir')} value={dir} min={0} max={359} step={1} unit="°" onChange={pickDir} />
        <Slider label={t('dr_ctl_dur')} value={dur} min={2} max={20} step={1} unit=" s" onChange={setDur} />
        <div className="mt-2 flex flex-wrap gap-2">
          <Button variant="primary" size="sm" disabled={!live} busy={busy}
            onClick={() => send({ cmd: 'gust', speed, dir, duration: dur })}>
            <WindIcon className="h-3.5 w-3.5" aria-hidden />{t('dr_btn_gust')}
          </Button>
          <Button size="sm" onClick={() => setDirTouched(false)} disabled={!dirTouched}>
            <Navigation className="h-3.5 w-3.5" aria-hidden />{t('dr_btn_follow')}
          </Button>
        </div>
      </fieldset>

      <fieldset className="rounded-xl border border-line bg-surface/50 p-3">
        <legend className="px-1 text-xs font-medium text-fg-mute">{t('dr_base_grp')}</legend>
        <Slider label={t('dr_ctl_base')} value={base} min={0} max={15} step={0.5} onChange={setBase} />
        <Slider label={t('dr_ctl_rate')} value={rate} min={0} max={6} step={0.25} onChange={setRate} />
        <div className="mt-2 flex flex-wrap gap-2">
          <Button size="sm" disabled={!live} onClick={() => send({ cmd: 'wind', speed: base, dir })}>{t('dr_btn_base')}</Button>
          <Button size="sm" disabled={!live} onClick={() => send({ cmd: 'gust_rate', value: rate })}>{t('dr_btn_rate')}</Button>
          <Button size="sm" variant="ghost" disabled={!live} onClick={() => send({ cmd: 'calm' })}>{t('dr_btn_calm')}</Button>
        </div>
      </fieldset>
      <p className="min-h-4 text-xs text-fg-mute num" role="status" aria-live="polite">{msg}</p>
      <p className="text-xs text-fg-mute">{t('dr_wind_hint')}</p>
    </Card>
  )
}
