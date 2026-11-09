import { useId } from 'react'
import { Waypoints } from 'lucide-react'
import '@/i18n/roadOverlay'
import { postJSON } from '@/lib/api'
import { useLang, useT } from '@/lib/i18n'
import { usePoll } from '@/lib/poll'
import { cn } from '@/lib/utils'
import { Badge, Card, Slider, Toggle } from '@/components/ui'
import { Sub } from './environment'

// Road logic overlay (/road_overlay, road_overlay.py): the OpenDRIVE map the
// AI drives on, drawn around your car with CARLA debug lines. On/off (also
// key O), radius, line budget, layers, and the legend of the in-world colours
// (served by the backend, so the swatches always match what is drawn).
// 道路ロジック表示：入切（O キー）・半径・線数・レイヤー・凡例

interface LayerInfo { key: string; en: string; ja: string; default: boolean }
interface LegendItem { layer: string; key: string; rgb: [number, number, number]; en: string; ja: string }
interface OverlayState {
  settings: {
    on: boolean; radius: number; max_shapes: number; refresh: number; detail_radius: number; brightness: number
    spectator: boolean; layers: Record<string, boolean>
  }
  status: {
    live?: boolean; ready?: boolean; building?: boolean; on?: boolean; map?: string; shapes?: number
    ms?: number; precompute_s?: number; lanes?: number; signs?: number; lights?: number; hero?: boolean
    capped?: boolean; osm?: string; error?: string; areas?: number; spectator_area?: boolean
    spectator_radius?: number | null; level?: number; night?: boolean
  }
  layers: LayerInfo[]
  legend: LegendItem[]
  limits: Record<string, [number, number]>
}
type Body = Partial<{
  on: boolean; spectator: boolean; radius: number; max_shapes: number; detail_radius: number; brightness: number
  layers: Record<string, boolean>
}>

/** A labelled checkbox (the kit has switches only; eight of them would be too
 *  tall here). / ラベル付きチェックボックス */
function LayerCheck({ label, checked, onChange }: { label: string; checked: boolean; onChange: (v: boolean) => void }) {
  const id = useId()
  return (
    <label htmlFor={id} className="flex min-h-8 cursor-pointer items-center gap-2 rounded-md px-1.5 text-sm text-fg hover:bg-muted">
      <input id={id} type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)}
        className="h-4 w-4 shrink-0 accent-[var(--color-accent)]" />
      <span className="min-w-0 leading-snug">{label}</span>
    </label>
  )
}

export function RoadOverlayCard() {
  const t = useT()
  const { lang } = useLang()
  const { data, setData } = usePoll<OverlayState>('/road_overlay', 1500)
  const post = async (body: Body) => {
    try { setData(await postJSON<OverlayState>('/road_overlay', body)) } catch { /* next poll shows the state */ }
  }
  const title = <span className="inline-flex items-center gap-2"><Waypoints className="h-3.5 w-3.5" aria-hidden />{t('ro_card')}</span>
  if (!data) return <Card title={title}><div className="text-sm text-fg-mute">…</div></Card>
  const s = data.settings, st = data.status ?? {}
  const live = !!st.live
  const name = (x: { en: string; ja: string }) => (lang === 'ja' ? x.ja : x.en)
  const badge = !live ? <Badge tone="warn">{t('ro_not_running')}</Badge>
    : st.building ? <Badge tone="info" dot>{t('ro_building')}</Badge>
      : s.on ? <Badge tone="good" dot>{t('ro_drawing')}{st.shapes != null ? ` · ${st.shapes}` : ''}</Badge>
        : <Badge tone="neutral">{t('ro_idle')}</Badge>
  const fmtS = (v?: number) => (v == null ? '–' : v.toFixed(1))
  const status = !live ? <span className="text-warn">{t('ro_offline')}</span>
    : st.error ? <span className="text-danger">{st.error}</span>
      : !st.ready ? t('ro_building')
        : s.on && st.on
          ? <>
            {t('ro_status', { map: st.map ?? '', n: st.shapes ?? 0, ms: Math.round(st.ms ?? 0), s: fmtS(st.precompute_s) })}
            {st.hero === false && <> · {t('ro_no_hero')}</>}
            {st.capped && <span className="block text-warn">{t('ro_capped')}</span>}
            {st.spectator_area && <span className="block">{t('ro_spec_on', { r: Math.round(st.spectator_radius ?? 0) })}</span>}
            {st.night && <span className="block">{t('ro_night')}</span>}
            {s.layers.osm && st.osm && <span className="block">{t('ro_osm', { state: st.osm })}</span>}
          </>
          : t('ro_status_idle', { map: st.map ?? '', lanes: st.lanes ?? 0, signs: st.signs ?? 0, lights: st.lights ?? 0, s: fmtS(st.precompute_s) })
  const [rLo, rHi] = data.limits?.radius ?? [20, 400]
  return (
    <Card title={title} actions={badge} wide
      footer={<span aria-live="polite">{status}</span>}>
      <Toggle label={t('ro_show')} help={t('ro_show_h')} checked={s.on} onChange={(v) => post({ on: v })} />
      <Toggle label={t('ro_spec')} help={t('ro_spec_h')} checked={s.spectator} onChange={(v) => post({ spectator: v })} />
      <div className="grid gap-x-6 gap-y-3 lg:grid-cols-2">
        <div className="space-y-2">
          <Slider label={t('ro_radius')} value={Math.round(s.radius)} min={rLo} max={rHi} step={10} unit=" m"
            onCommit={(v) => post({ radius: v })} />
          <Slider label={t('ro_bright')} value={Math.round(s.brightness)} min={1} max={8} step={1}
            help={t('ro_bright_h')} onCommit={(v) => post({ brightness: v })} />
          <Slider label={t('ro_detail')} value={Math.round(s.detail_radius)} min={10} max={120} step={5} unit=" m"
            help={t('ro_detail_h')} onCommit={(v) => post({ detail_radius: v })} />
          <Slider label={t('ro_shapes')} value={s.max_shapes} min={200} max={3000} step={100}
            help={t('ro_shapes_h')} onCommit={(v) => post({ max_shapes: v })} />
          <Sub>{t('ro_layers')}</Sub>
          <div className="grid grid-cols-1 gap-x-2 sm:grid-cols-2" role="group" aria-label={t('ro_layers')}>
            {data.layers.map((L) => (
              <LayerCheck key={L.key} label={name(L)} checked={!!s.layers[L.key]}
                onChange={(v) => post({ layers: { [L.key]: v } })} />
            ))}
          </div>
        </div>
        <div className="space-y-2">
          <Sub>{t('ro_legend')}</Sub>
          <ul className="grid grid-cols-1 gap-x-3 gap-y-1 text-xs sm:grid-cols-2">
            {data.legend.map((it) => (
              <li key={it.key} className={cn('flex min-w-0 items-center gap-2', !s.layers[it.layer] && 'opacity-40')}>
                {/* the in-world colour, from road_overlay.py / ワールド内の色 */}
                <span className="h-1.5 w-5 shrink-0 rounded-full ring-1 ring-line"
                  style={{ backgroundColor: `rgb(${it.rgb.join(',')})` }} aria-hidden />
                <span className="min-w-0 leading-snug text-fg-2">{name(it)}</span>
              </li>
            ))}
          </ul>
          <p className="text-xs text-fg-mute">{t('ro_legend_h')}</p>
          <p className="text-xs text-fg-mute">{t('ro_hint')}</p>
        </div>
      </div>
    </Card>
  )
}
