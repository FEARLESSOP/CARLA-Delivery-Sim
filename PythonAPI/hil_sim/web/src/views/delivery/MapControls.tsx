// City Map overlays: the zone editor's tool panel (one chip per pod in its own
// colour, No-fly, delete, save status) and the Layers filter popover. The kit
// has no checkbox, so a compact one lives here (LayerCheck).
// Modes: zones, meeting points (every pod / hub), distribution points (LMH
// hubs only: where the hub parks and sends its mini pods out), charging
// stations (LastMile charging-station spots, one list per map).
// 地図の重ね表示：ゾーン編集の道具パネルとレイヤー絞り込み
import { useEffect, useId, useRef, type KeyboardEvent as ReactKeyboardEvent } from 'react'
import { Ban, Check, MapPin, RotateCcw, RotateCw, Trash2, X } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { fmt } from '@/lib/fmt'
import { Badge, Button, Segmented, type Tone } from '@/components/ui'
import { MC, podColor } from './shared'
import { LAYER_GROUPS, LAYER_KEYS, allLayers, type LayerKey, type Layers } from './layers'
import { MAX_CHARGE, MAX_NOFLY, inNoFly, lmhArea, maxPoints, type Tool, type ZoneRef, type ZoneStatus } from './zones'
import type { ChargeSpot, MeetPoint, ZoneSet } from './types'
import type { SpotIssue } from './stations'

/* ------------------------------------------------------------ zone tools */
/** lmh: the van is a LastMile Hub (carries mini pods); area: what the sim
 *  says its delivery area is / LMH ハブと配達範囲 */
export interface PodChip { id: string; running: boolean; lmh?: boolean; area?: string }

const STATUS_TONE: Record<ZoneStatus, Tone> = {
  idle: 'neutral', loading: 'neutral', saving: 'info', saved: 'good', error: 'bad', offline: 'warn', nomap: 'warn',
}

export function ZoneStatusBadge({ status, error }: { status: ZoneStatus; error: string }) {
  const t = useT()
  if (status === 'idle') return null
  return (
    <Badge tone={STATUS_TONE[status]} dot className="max-w-full truncate" title={error || undefined}>
      <span role="status" aria-live="polite" className="truncate">{t(`dr_zst_${status}`, { e: error })}</span>
    </Badge>
  )
}

/** what the pod chips do: draw the pod's zone, add its meeting points, or
 *  add an LMH hub's distribution points / ゾーン・合流地点・配布地点 */
export type EditMode = 'zone' | 'meet' | 'dist' | 'charge'

/** The charging-station mode's view of things: the server keeps "charge"
 *  (ok), a map is loaded (map) with lane polygons (lanes), the selected spot
 *  and what is wrong with it (pre-launch check / the sim's reason), how many
 *  are unusable, how many the sim built (null: not running or an older sim).
 *  充電ステーションモードの状態 */
export interface ChargeInfo {
  ok: boolean; map: boolean; lanes: boolean; sel: ChargeSpot | null; issue: SpotIssue | null; why: string | null
  bad: number; live: number | null
}

export function ZoneToolbar({ id, chips, zones, tool, sel, status, error, mode, onMode, snapExact, selMeet, charge, onRotate, onTool, onDelete, onDone }: {
  id: string; chips: PodChip[]; zones: ZoneSet; tool: Tool; sel: ZoneRef | null; status: ZoneStatus; error: string
  mode: EditMode; onMode: (m: EditMode) => void
  /** true: exact stop points, false: lane-vertex fallback, null: no map yet */
  snapExact: boolean | null; selMeet: MeetPoint | null
  charge: ChargeInfo; onRotate: (d: number | null) => void
  onTool: (t: Tool) => void; onDelete: () => void; onDone: () => void
}) {
  const t = useT()
  const nfFull = zones.nofly.length >= MAX_NOFLY
  const isTool = (k: 'pod' | 'nofly' | 'meet' | 'dist', pid?: string) =>
    !!tool && tool.kind === k && ('id' in tool ? tool.id === pid : true)
  const selName = !sel ? '' : sel.kind === 'pod' ? t(chips.find((c) => c.id === sel.id)?.lmh ? 'dr_hub_label' : 'dr_zone_label', { n: sel.id })
    : sel.kind === 'meet' ? t('dr_meet_name', { n: sel.id, i: sel.i + 1 })
      : sel.kind === 'dist' ? t('dr_dist_name', { n: sel.id, i: sel.i + 1 })
        : sel.kind === 'charge' ? t('dr_cs_name', { i: sel.i + 1 }) : t('dr_zone_nofly')
  const meetN = (pid: string) => zones.meet[pid]?.length ?? 0
  const distN = (pid: string) => zones.dist[pid]?.length ?? 0
  const kind = mode === 'meet' ? 'meet' : mode === 'dist' ? 'dist' : 'pod'
  // distribution points belong to LMH hubs only / 配布地点は LMH ハブのみ
  const shown = mode === 'dist' ? chips.filter((c) => c.lmh || distN(c.id) > 0) : chips
  const pointTool = tool && (tool.kind === 'meet' || tool.kind === 'dist') ? tool : null
  return (
    <div id={id} role="toolbar" aria-label={t('dr_zones_tools')}
      className="w-[min(36rem,calc(100vw-2.5rem))] max-w-full rounded-xl border border-line bg-bg/90 p-2.5 shadow-xl backdrop-blur-md">
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <Segmented size="sm" label={t('dr_edit_mode')} value={mode} onChange={onMode}
          options={[{ value: 'zone', label: t('dr_mode_zone') }, { value: 'meet', label: t('dr_mode_meet') },
            { value: 'dist', label: t('dr_mode_dist') }, { value: 'charge', label: t('dr_mode_charge') }]} />
        <span className="flex items-center gap-1.5">
          <Button size="sm" variant="danger" disabled={!sel} onClick={onDelete}
            aria-label={sel ? `${t('dr_zone_delete')}: ${selName}` : t('dr_zone_delete')}>
            <Trash2 aria-hidden className="h-3.5 w-3.5" /><span className="hidden sm:inline">{t('dr_zone_delete')}</span>
          </Button>
          <Button size="sm" variant="primary" onClick={onDone}>{t('dr_zones_done')}</Button>
        </span>
      </div>
      {mode === 'charge' ? <ChargeTools zones={zones} tool={tool} sel={sel} status={status} error={error} charge={charge}
        onRotate={onRotate} onTool={onTool} /> : <>
      <div className="flex flex-wrap items-center gap-1.5">
        {shown.map((c) => {
          const col = podColor(c.id), on = isTool(kind, c.id), has = !!zones.pods[c.id], n = meetN(c.id), dn = distN(c.id)
          const full = (mode === 'meet' && n >= maxPoints('meet') && !on) || (mode === 'dist' && dn >= maxPoints('dist') && !on)
          const what = mode === 'meet' ? t('dr_meet_pick', { n: c.id }) : mode === 'dist' ? t('dr_dist_pick', { n: c.id })
            : t('dr_zone_pick', { n: c.id })
          const count = mode === 'dist' ? t('dr_dist_count', { n: dn }) : t('dr_meet_count', { n })
          // an LMH hub without its own zone delivers in the no-fly zones / 既定は飛行禁止区域
          const dflt = c.lmh && mode === 'zone' && lmhArea(zones, c.id, c.area) === 'nofly'
          return (
            <button key={c.id} type="button" aria-pressed={on} disabled={full}
              aria-label={`${what}${c.lmh ? ` · ${t('dr_chip_lmh')}` : ''}${dflt ? ` · ${t('dr_lmh_default')}` : ''}${has ? ` · ${t('dr_zone_has')}` : ''} · ${count}${c.running ? '' : ` · ${t('dr_zone_off')}`}`}
              title={full ? t(mode === 'dist' ? 'dr_dist_full' : 'dr_meet_full') : !c.running ? t('dr_zone_off')
                : dflt ? t('dr_lmh_default_tip') : c.lmh ? (mode === 'zone' && has ? t('dr_lmh_zone_tip') : t('dr_chip_lmh')) : undefined}
              onClick={() => onTool(on ? null : { kind, id: c.id })}
              className={cn('inline-flex h-8 items-center gap-1.5 rounded-lg border px-2.5 text-xs font-semibold transition-colors duration-150',
                'disabled:cursor-not-allowed disabled:opacity-50',
                on ? 'bg-muted text-fg' : 'border-line bg-card-2 text-fg-2 hover:bg-muted', !c.running && 'opacity-60')}
              style={on ? { borderColor: col, boxShadow: `inset 0 0 0 1px ${col}` } : undefined}>
              <i aria-hidden className="h-3 w-3 shrink-0 rounded-[3px] border" style={{ background: `${col}55`, borderColor: col }} />
              {c.lmh ? t('dr_hub_label', { n: c.id }) : t('dr_zone_label', { n: c.id })}
              {has && <Check aria-hidden className="h-3 w-3 text-fg-mute" />}
              {n > 0 && <span aria-hidden className="inline-flex items-center gap-0.5 text-fg-mute num"><MapPin className="h-3 w-3" />{n}</span>}
              {dn > 0 && <span aria-hidden className="inline-flex items-center gap-0.5 text-fg-mute num"><DistGlyph />{dn}</span>}
              {dflt && <span aria-hidden className="rounded bg-danger/15 px-1 text-[10px] font-medium text-danger">{t('dr_lmh_default')}</span>}
            </button>
          )
        })}
        {mode === 'zone' && (
          <button type="button" aria-pressed={isTool('nofly')} disabled={nfFull && !isTool('nofly')}
            aria-label={nfFull ? `${t('dr_zone_pick_nf')} · ${t('dr_nofly_full')}` : t('dr_zone_pick_nf')}
            title={nfFull ? t('dr_nofly_full') : undefined}
            onClick={() => onTool(isTool('nofly') ? null : { kind: 'nofly' })}
            className={cn('inline-flex h-8 items-center gap-1.5 rounded-lg border px-2.5 text-xs font-semibold transition-colors duration-150',
              'disabled:cursor-not-allowed disabled:opacity-50',
              isTool('nofly') ? 'border-danger bg-danger/20 text-danger' : 'border-danger/40 bg-danger/10 text-danger hover:bg-danger/20')}>
            <Ban aria-hidden className="h-3.5 w-3.5" />{t('dr_zone_nofly')}
          </button>
        )}
        {!shown.length && <span className="text-xs text-fg-mute">{mode === 'dist' ? t('dr_no_hubs') : t('dr_no_pods')}</span>}
      </div>
      <div className="mt-2 flex min-h-6 flex-wrap items-center gap-2 text-xs text-fg-2">
        {pointTool ? (
          <span className="font-medium text-fg">{pointTool.kind === 'dist'
            ? t('dr_dist_adding', { n: pointTool.id, c: distN(pointTool.id), m: maxPoints('dist') })
            : t('dr_meet_adding', { n: pointTool.id, c: meetN(pointTool.id), m: maxPoints('meet') })}</span>
        ) : tool ? (
          <span className="font-medium text-fg">
            {t('dr_zone_drawing', { what: tool.kind === 'pod' ? t('dr_zone_label', { n: tool.id }) : t('dr_zone_nofly') })}
          </span>
        ) : sel?.kind === 'meet' || sel?.kind === 'dist' ? (
          <span className="font-medium text-fg">
            {t('dr_meet_selected', { what: selName })}
            {selMeet && <span className="ml-1 text-fg-mute num">({fmt(selMeet.x, 1)}, {fmt(selMeet.y, 1)})</span>}
            {selMeet && sel.kind === 'meet' && !chips.find((c) => c.id === sel.id)?.lmh && inNoFly(zones, selMeet.x, selMeet.y) && <span className="ml-1 text-danger">· {t('dr_meet_bad')}</span>}
          </span>
        ) : sel ? (
          <span className="font-medium text-fg">{t('dr_zone_selected', { what: selName })}</span>
        ) : (
          <span className="text-fg-mute">{mode === 'meet' ? t('dr_meet_none') : mode === 'dist' ? t('dr_dist_none') : t('dr_zone_none')}</span>
        )}
        <ZoneStatusBadge status={status} error={error} />
      </div>
      </>}
      {(mode === 'meet' || mode === 'dist') && snapExact === false && (
        <p className="mt-1.5 text-[11px] text-warn" role="note">{t('dr_meet_fallback')}</p>
      )}
      {(mode === 'meet' || mode === 'dist') && snapExact === null && (
        <p className="mt-1.5 text-[11px] text-warn" role="note">{t('dr_meet_nomap')}</p>
      )}
      <details className="mt-1.5 text-[11px] leading-snug text-fg-mute">
        <summary className="cursor-pointer select-none rounded text-fg-2 hover:text-fg">{t('dr_zone_howto')}</summary>
        {mode === 'charge' ? (
          <>
            <p className="mt-1">{t('dr_cs_hint')}</p>
            <p className="mt-1">{t('dr_cs_rules')}</p>
            <p className="mt-1">{t('dr_cs_launch')}</p>
          </>
        ) : mode === 'meet' ? (
          <>
            <p className="mt-1">{t('dr_meet_hint')}</p>
            <p className="mt-1">{t('dr_meet_rules')}</p>
            <p className="mt-1">{t('dr_meet_lmh')}</p>
          </>
        ) : mode === 'dist' ? (
          <>
            <p className="mt-1">{t('dr_dist_hint')}</p>
            <p className="mt-1">{t('dr_dist_rules')}</p>
          </>
        ) : (
          <>
            <p className="mt-1">{t('dr_zone_hint')}</p>
            <p className="mt-1">{t('dr_zone_rules')}</p>
            <p className="mt-1">{t('dr_lmh_area_help')}</p>
          </>
        )}
      </details>
    </div>
  )
}

/* ------------------------------------------------------ charging stations */
/** The charging-station mode of the tool panel: the add tool with its count,
 *  turn / automatic direction for the selected station, the check result.
 *  充電ステーションの道具（追加・回転・自動向き・確認結果） */
function ChargeTools({ zones, tool, sel, status, error, charge, onRotate, onTool }: {
  zones: ZoneSet; tool: Tool; sel: ZoneRef | null; status: ZoneStatus; error: string; charge: ChargeInfo
  onRotate: (d: number | null) => void; onTool: (t: Tool) => void
}) {
  const t = useT()
  const n = zones.charge.length, full = n >= MAX_CHARGE, on = tool?.kind === 'charge'
  const s = sel?.kind === 'charge' ? charge.sel : null
  const reason = s ? (charge.issue ? t(`dr_cs_bad_${charge.issue}`) : charge.why ? t('dr_cs_rejected', { why: charge.why }) : null) : null
  return (
    <>
      <div className="flex flex-wrap items-center gap-1.5">
        <button type="button" aria-pressed={on} disabled={(full && !on) || !charge.ok}
          aria-label={`${t('dr_cs_add')} · ${t('dr_cs_count', { n, m: MAX_CHARGE })}${full ? ` · ${t('dr_cs_full')}` : ''}`}
          title={!charge.ok ? t('dr_cs_old_server') : full ? t('dr_cs_full') : undefined}
          onClick={() => onTool(on ? null : { kind: 'charge' })}
          className={cn('inline-flex h-8 items-center gap-1.5 rounded-lg border px-2.5 text-xs font-semibold transition-colors duration-150',
            'disabled:cursor-not-allowed disabled:opacity-50',
            on ? 'bg-muted text-fg' : 'border-line bg-card-2 text-fg-2 hover:bg-muted')}
          style={on ? { borderColor: MC.charge, boxShadow: `inset 0 0 0 1px ${MC.charge}` } : undefined}>
          <ChargeGlyph />{t('dr_cs_add')}
          <span className="text-fg-mute num">{n}/{MAX_CHARGE}</span>
        </button>
        {s && (
          <span className="inline-flex items-center gap-1" role="group" aria-label={t('dr_cs_turn')}>
            <Button size="sm" onClick={() => onRotate(-15)} aria-label={t('dr_cs_left')} title={t('dr_cs_left')}>
              <RotateCcw aria-hidden className="h-3.5 w-3.5" />15°</Button>
            <Button size="sm" onClick={() => onRotate(15)} aria-label={t('dr_cs_right')} title={t('dr_cs_right')}>
              <RotateCw aria-hidden className="h-3.5 w-3.5" />15°</Button>
            <Button size="sm" variant={s.yaw == null ? 'primary' : 'secondary'} disabled={s.yaw == null}
              onClick={() => onRotate(null)} title={t('dr_cs_auto_h')}>{t('dr_cs_auto_btn')}</Button>
          </span>
        )}
      </div>
      <div className="mt-2 flex min-h-6 flex-wrap items-center gap-2 text-xs text-fg-2">
        {s && sel?.kind === 'charge' ? (
          <span className="font-medium text-fg">
            {t('dr_meet_selected', { what: t('dr_cs_name', { i: sel.i + 1 }) })}
            <span className="ml-1 text-fg-mute num">({fmt(s.x, 1)}, {fmt(s.y, 1)}) · {s.yaw != null ? t('dr_cs_yaw', { d: fmt(s.yaw, 0) }) : t('dr_cs_auto')}</span>
            {reason ? <span className="ml-1 text-danger">· {reason}</span>
              : charge.map && <span className="ml-1 text-accent">· {t('dr_cs_ok')}</span>}
          </span>
        ) : on ? (
          <span className="font-medium text-fg">{t('dr_cs_adding', { c: n, m: MAX_CHARGE })}</span>
        ) : (
          <span className="text-fg-mute">{t('dr_cs_none')}</span>
        )}
        {n > 0 && <Badge tone={charge.bad ? 'bad' : 'good'}>{charge.bad ? t('dr_cs_bad_n', { n: charge.bad, m: n }) : t('dr_cs_ok_n', { n })}</Badge>}
        {charge.live != null && <Badge tone="info">{t('dr_cs_live_n', { n: charge.live })}</Badge>}
        <ZoneStatusBadge status={status} error={error} />
      </div>
      {!charge.ok && <p className="mt-1.5 text-[11px] text-warn" role="note">{t('dr_cs_old_server')}</p>}
      {charge.ok && !charge.map && <p className="mt-1.5 text-[11px] text-warn" role="note">{t('dr_cs_nomap')}</p>}
      {charge.ok && charge.map && !charge.lanes && <p className="mt-1.5 text-[11px] text-warn" role="note">{t('dr_cs_nolanes')}</p>}
    </>
  )
}

/** The bolt marker of a charging station (tools, legend). / 充電ステーションの記号 */
export function ChargeGlyph({ bad }: { bad?: boolean }) {
  return (
    <svg aria-hidden viewBox="0 0 16 16" className="h-3.5 w-3.5 shrink-0">
      <rect x="1" y="1" width="14" height="14" rx="3" fill={bad ? MC.compassBg : MC.charge} stroke={bad ? MC.nofly : MC.zoneText} strokeWidth="1.5" />
      <path d="M9 3 L4.5 8.8 L7.6 8.8 L6.6 13 L11.5 7 L8.4 7 Z" fill={bad ? MC.nofly : MC.zoneText} />
    </svg>
  )
}

/* ---------------------------------------------------------------- layers */
function LayerCheck({ checked, onChange, label }: { checked: boolean; onChange: (v: boolean) => void; label: string }) {
  return (
    <label className="flex min-h-8 cursor-pointer items-center gap-2.5 rounded-md px-1.5 text-sm text-fg-2 hover:bg-muted hover:text-fg">
      <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)}
        className="h-4 w-4 shrink-0 cursor-pointer accent-accent" />
      <span className="min-w-0 truncate">{label}</span>
    </label>
  )
}

export function LayersMenu({ id, layers, onChange, onClose, className }: {
  id: string; layers: Layers; onChange: (l: Layers) => void; onClose: () => void; className?: string
}) {
  const t = useT()
  const ref = useRef<HTMLDivElement>(null)
  const head = useId()
  const shown = LAYER_KEYS.filter((k) => layers[k]).length

  // focus the first checkbox; close on a click outside / 最初の項目にフォーカス、外側クリックで閉じる
  useEffect(() => {
    ref.current?.querySelector<HTMLInputElement>('input')?.focus()
    const out = (ev: PointerEvent) => {
      const el = ev.target as Node
      if (ref.current && !ref.current.contains(el) && !(el instanceof Element && el.closest('[data-layers-btn]'))) onClose()
    }
    document.addEventListener('pointerdown', out)
    return () => document.removeEventListener('pointerdown', out)
  }, [onClose])

  const onKey = (ev: ReactKeyboardEvent) => {
    if (ev.key === 'Escape') { ev.stopPropagation(); onClose() }
  }
  const set = (k: LayerKey, v: boolean) => onChange({ ...layers, [k]: v })

  return (
    <div ref={ref} id={id} role="dialog" aria-labelledby={head} onKeyDown={onKey}
      className={cn('w-64 max-w-[calc(100vw-2.5rem)] overflow-y-auto rounded-xl border border-line bg-bg/95 p-2.5 shadow-xl backdrop-blur-md', className)}>
      <div className="mb-1.5 flex items-center justify-between gap-2 px-1">
        <span id={head} className="text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">{t('dr_layers_title')}</span>
        <span className="flex items-center gap-1">
          <span className="text-[11px] text-fg-mute num">{t('dr_ly_count', { n: shown, m: LAYER_KEYS.length })}</span>
          <button type="button" onClick={onClose} aria-label={t('dr_close')}
            className="grid h-7 w-7 place-items-center rounded-md text-fg-mute hover:bg-muted hover:text-fg">
            <X aria-hidden className="h-3.5 w-3.5" />
          </button>
        </span>
      </div>
      <div className="mb-2 grid grid-cols-2 gap-1.5">
        <Button size="sm" onClick={() => onChange(allLayers(true))} disabled={shown === LAYER_KEYS.length}>{t('dr_show_all')}</Button>
        <Button size="sm" onClick={() => onChange(allLayers(false))} disabled={shown === 0}>{t('dr_hide_all')}</Button>
      </div>
      {LAYER_GROUPS.map(([g, keys]) => (
        <fieldset key={g} className="border-t border-line pt-1.5 pb-1 first-of-type:border-t-0">
          <legend className="px-1 text-[11px] font-semibold uppercase tracking-wider text-fg-mute">{t(g)}</legend>
          {keys.map((k) => <LayerCheck key={k} checked={layers[k]} onChange={(v) => set(k, v)} label={t(`dr_ly_${k}`)} />)}
          {g === 'dr_lyg_orders' && <p className="px-1.5 pb-1 text-[11px] text-fg-mute">{t('dr_ly_orders_hint')}</p>}
          {g === 'dr_lyg_zones' && <p className="px-1.5 pb-1 text-[11px] text-fg-mute">{t('dr_ly_zones_hint')}</p>}
        </fieldset>
      ))}
    </div>
  )
}

/** The small square "D" of a distribution point (chips, legend). / 配布地点の記号 */
export function DistGlyph({ col = podColor(1), big }: { col?: string; big?: boolean }) {
  return (
    <svg aria-hidden viewBox="0 0 14 16" className={cn('shrink-0', big ? 'h-3.5 w-3' : 'h-3 w-2.5')}>
      <line x1="7" y1="10" x2="7" y2="15.5" stroke={MC.zoneText} strokeWidth="1.5" />
      <rect x="1" y="1" width="12" height="9.5" fill={col} stroke={MC.zoneText} strokeWidth="1.5" />
      <text x="7" y="8.6" textAnchor="middle" fontSize="7.5" fontWeight="800" fill={MC.zoneText}>D</text>
    </svg>
  )
}

/** Legend swatches for the zones: a few pod colours, the no-fly hatch, a
 *  meeting-point pin or a distribution-point badge. */
export function ZoneSwatch({ nofly, meet, dist, lmh }: { nofly?: boolean; meet?: boolean; dist?: boolean; lmh?: boolean }) {
  if (dist) return <DistGlyph col={podColor(4)} big />
  if (nofly && lmh) {
    return <i aria-hidden className="inline-block h-2.5 w-3.5 shrink-0 rounded-[2px] border border-dashed"
      style={{ borderColor: MC.nofly, background: `repeating-linear-gradient(45deg, ${MC.noflyHatch} 0 1.5px, transparent 1.5px 4px)`,
        boxShadow: `inset 0 0 0 1px ${podColor(4)}` }} />
  }
  if (meet) {
    return (
      <svg aria-hidden viewBox="0 0 12 16" className="h-3.5 w-2.5 shrink-0">
        <path d="M6 15 L1.8 8.6 A5 5 0 1 1 10.2 8.6 Z" fill={podColor(2)} stroke={MC.zoneText} strokeWidth="1.5" />
        <text x="6" y="8.4" textAnchor="middle" fontSize="6.5" fontWeight="800" fill={MC.zoneText}>1</text>
      </svg>
    )
  }
  if (nofly) {
    return <i aria-hidden className="inline-block h-2.5 w-3.5 shrink-0 rounded-[2px] border border-dashed"
      style={{ borderColor: MC.nofly, background: `repeating-linear-gradient(45deg, ${MC.noflyHatch} 0 1.5px, transparent 1.5px 4px)` }} />
  }
  return (
    <span aria-hidden className="inline-flex shrink-0 gap-px">
      {[1, 2, 3, 4].map((n) => <i key={n} className="inline-block h-2.5 w-1.5 rounded-[1px] border"
        style={{ background: `${podColor(n)}40`, borderColor: podColor(n) }} />)}
    </span>
  )
}
