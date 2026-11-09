// Pieces around the big 3D drone: the status tiles under it (GPS, obstacle
// avoidance, RTK, return to home, winch, parachute), the keyboard parts list,
// and the DOM bits the 3D view shows (hover tooltip, "broken" labels).
// 3D の周り：状態タイル・部品一覧（キーボード用）・ツールチップ・破損ラベル
import type { ComponentType } from 'react'
import { Cable, House, LocateFixed, Radar, Satellite, Umbrella } from 'lucide-react'
import { useT, type TFunc } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import type { Tone } from '@/components/ui'
import { HEALTH_MARK, f } from './catalog'
import { LEVEL_TONE, PART_GROUPS, partName, partReading, stateText, type PartId, type PartState } from './parts'
import { TONE_DOT, TONE_TEXT } from './left'
import type { DroneS, Health, Sensors } from './types'

const tone = (h: Health | undefined): Tone => (h ? LEVEL_TONE[h] : 'neutral')

interface Tile { key: string; part: PartId; icon: ComponentType<{ className?: string }>; text: string; tone: Tone }

function tiles(t: TFunc, s: Sensors | null | undefined): Tile[] {
  const g = s?.gnss, o = s?.obstacle, l = s?.link, w = s?.winch, c = s?.parachute
  const obst = !o?.health ? '–' : o.health === 'fail' ? t('dn_t_fault') : o.health === 'off' ? t('dn_t_standby')
    : o.health === 'warn' && o.nearest != null ? t('dn_t_obst_near', { m: f(o.nearest, 1) }) : t('dn_t_active')
  const link = !l?.health ? '–' : l.mode === 'wired' ? t('dn_t_wired') : l.health === 'fail' ? t('dn_t_lost')
    : l.health === 'warn' ? t('dn_t_weak') : t('dn_t_ready')
  const fixTone: Tone = g?.fix === 'rtk_fixed' ? 'good' : g?.fix === 'rtk_float' ? 'info' : g?.fix === '3d' ? 'warn'
    : g?.health === 'off' ? 'neutral' : g ? 'bad' : 'neutral'
  return [
    { key: 'dn_t_gps', part: 'gnss', icon: Satellite, tone: tone(g?.health),
      text: g ? (g.fix === 'none' ? t('dn_fix_none') : t('dn_t_sats', { n: g.sats ?? 0 })) : '–' },
    { key: 'dn_t_obst', part: 'cam_front', icon: Radar, tone: tone(o?.health), text: obst },
    { key: 'dn_t_rtk', part: 'gnss', icon: LocateFixed, tone: fixTone, text: g ? t(`dn_fix_${g.fix ?? 'none'}`) : '–' },
    { key: 'dn_t_rth', part: 'link', icon: House, tone: tone(l?.health), text: link },
    { key: 'dn_t_winch', part: 'winch', icon: Cable, tone: tone(w?.health), text: w ? t(`dn_w_${w.state ?? 'idle'}`) : '–' },
    { key: 'dn_t_chute', part: 'parachute', icon: Umbrella, tone: tone(c?.health), text: c ? t(`dn_c_${c.state ?? 'disarmed'}`) : '–' },
  ]
}

/** Status tiles: icon, name, a dot AND text (never colour alone). / 状態タイル */
export function StatusTiles({ s, onPick }: { s: Sensors | null | undefined; onPick: (p: PartId) => void }) {
  const t = useT()
  return (
    <ul className="grid grid-cols-2 gap-2 sm:grid-cols-3 2xl:grid-cols-6" aria-label={t('dn_status_tiles')}>
      {tiles(t, s).map((x) => {
        const Icon = x.icon
        return (
          <li key={x.key}>
            <button type="button" onClick={() => onPick(x.part)}
              className="flex h-full w-full flex-col items-center gap-1 rounded-xl border border-line bg-card/80 px-2 py-2.5 text-center shadow-[var(--shadow-card)] transition-colors duration-150 hover:border-line-strong">
              <Icon className="h-5 w-5 text-accent" aria-hidden />
              <span className="text-xs font-medium text-fg">{t(x.key)}</span>
              <span className={cn('flex items-center gap-1 text-[11px]', TONE_TEXT[x.tone])}>
                <i className={cn('h-1.5 w-1.5 shrink-0 rounded-full', TONE_DOT[x.tone])} aria-hidden />
                <span className="num truncate">{x.text}</span>
              </span>
            </button>
          </li>
        )
      })}
    </ul>
  )
}

/** Every part as a button (keyboard / screen reader way into the model). / 部品ボタン一覧 */
export function PartList({ states, selected, onPick }: {
  states: Record<PartId, PartState>; selected: PartId | null; onPick: (p: PartId) => void
}) {
  const t = useT()
  return (
    <details className="group rounded-xl border border-line bg-card/60 px-3 py-2">
      <summary className="cursor-pointer select-none text-xs text-fg-mute marker:text-fg-mute">
        {t('dn_parts')} <span className="text-fg-mute/80">· {t('dn_parts_help')}</span>
      </summary>
      <div className="mt-2 flex flex-col gap-2">
        {PART_GROUPS.map((g) => (
          <div key={g.key}>
            <div className="mb-1 text-[10px] uppercase tracking-wider text-fg-mute">{t(`dn_pg_${g.key}`)}</div>
            <div className="flex flex-wrap gap-1">
              {g.parts.map((p) => {
                const st = states[p] ?? {}
                const tn = st.level ? LEVEL_TONE[st.level] : 'neutral'
                return (
                  <button key={p} type="button" aria-pressed={selected === p} onClick={() => onPick(p)}
                    aria-label={`${partName(t, p)}: ${stateText(t, st)}`}
                    className={cn('flex items-center gap-1 rounded-md border px-1.5 py-0.5 text-[11px] transition-colors',
                      selected === p ? 'border-accent bg-accent/15 text-fg' : 'border-line bg-card-2 text-fg-2 hover:border-line-strong')}>
                    <span className={cn('num w-3 text-center', TONE_TEXT[tn])} aria-hidden>{st.level ? HEALTH_MARK[st.level] : '·'}</span>
                    {partName(t, p)}
                  </button>
                )
              })}
            </div>
          </div>
        ))}
      </div>
    </details>
  )
}

/** The hover tooltip in the 3D view: name, state, a reading. / ツールチップ */
export function PartTooltip({ t, part, st, d, hint = true }: { t: TFunc; part: PartId; st: PartState; d: DroneS; hint?: boolean }) {
  const tn = st.level ? LEVEL_TONE[st.level] : 'neutral'
  const reading = partReading(t, part, d)
  return (
    <div className={cn("pointer-events-none w-max max-w-56 text-xs", hint && "rounded-lg border border-line-strong bg-surface/95 px-2.5 py-1.5 shadow-lg")}>
      <div className="font-semibold text-fg">{partName(t, part)}</div>
      <div className={cn('flex items-center gap-1', TONE_TEXT[tn])}>
        <i className={cn('h-1.5 w-1.5 rounded-full', TONE_DOT[tn])} aria-hidden />{stateText(t, st)}
        {st.dmg && <span className="text-fg-mute">· {t(`dn_why_${st.dmg.why}`)}</span>}
      </div>
      {reading && <div className="num text-fg-2">{reading}</div>}
      {hint && <div className="mt-0.5 text-[10px] text-fg-mute">{t('dn_click_details')}</div>}
    </div>
  )
}

/** A red label over a broken / faulty part. / 破損ラベル */
export function BrokenLabel({ t, part, st }: { t: TFunc; part: PartId; st: PartState }) {
  return (
    <div className="pointer-events-none flex w-max items-center gap-1 rounded-full border border-danger/70 bg-bg/85 px-2 py-0.5 text-[10px] font-semibold text-danger shadow">
      <span aria-hidden>✕</span>{st.dmg?.level === 'broken' ? t('dn_dmg_broken') : t('dn_h_fail')} · <span className="font-normal text-fg">{partName(t, part)}</span>
    </div>
  )
}
