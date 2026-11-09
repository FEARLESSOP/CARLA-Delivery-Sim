// Pieces around the big 3D pod: the status tiles under it (Lidar, Cameras,
// GNSS, Obstacle stop, Lid lock, Link: found in the self-describing sensor
// dict by part / key / group, never by fixed keys), the keyboard parts list,
// and the DOM bits the 3D view shows (hover tooltip, "broken" labels).
// 3D の周り：状態タイル・部品一覧（キーボード用）・ツールチップ・破損ラベル
import type { ComponentType } from 'react'
import { Camera, Lock, OctagonX, Radar, RadioTower, Satellite } from 'lucide-react'
import { useT, type TFunc } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import type { Tone } from '@/components/ui'
import { HEALTH_MARK, HEALTH_TONE, TONE_DOT, TONE_TEXT, isHealth, valueText, worst } from './data'
import { LEVEL_TONE, PART_GROUPS, partName, partReading, sensorPart, stateText, type PartId, type PartState } from './parts'
import type { Health, MiniPod, PodSensor } from './types'

interface Tile { key: string; part: PartId; icon: ComponentType<{ className?: string }>; text: string; tone: Tone }

type Match = (k: string, s: PodSensor, part: PartId) => boolean
const TILE_DEFS: { key: string; part: PartId; icon: Tile['icon']; match: Match; count?: boolean }[] = [
  { key: 'mp_t_lidar', part: 'lidar', icon: Radar, match: (_k, _s, p) => p === 'lidar' },
  { key: 'mp_t_cams', part: 'cam_front', icon: Camera, match: (_k, _s, p) => p.startsWith('cam_'), count: true },
  { key: 'mp_t_gnss', part: 'gnss', icon: Satellite, match: (k, s, p) => p === 'gnss' || /gnss|gps|rtk/i.test(`${k} ${s.label ?? ''}`) },
  { key: 'mp_t_obst', part: 'bumper', icon: OctagonX,
    match: (k, s, p) => p === 'bumper' || /obstacle|e[-_ ]?stop|ultra|sonar|bumper/i.test(`${k} ${s.label ?? ''}`) },
  { key: 'mp_t_lock', part: 'lid_lock', icon: Lock, match: (k, s, p) => p === 'lid_lock' || /lock|latch/i.test(`${k} ${s.label ?? ''}`) },
  { key: 'mp_t_link', part: 'link', icon: RadioTower, match: (_k, s, p) => p === 'link' || s.group === 'comms' },
]

function tiles(t: TFunc, sensors: Record<string, PodSensor> | null | undefined): Tile[] {
  const all = Object.entries(sensors ?? {}).filter(([, s]) => s && typeof s === 'object')
  return TILE_DEFS.map((d) => {
    const hits = all.filter(([k, s]) => d.match(k, s, sensorPart(k, s)))
    let h: Health | undefined
    for (const [, s] of hits) if (isHealth(s.health)) h = worst(h, s.health)
    let text = '–'
    if (d.count && hits.length) {
      const ok = hits.filter(([, s]) => s.health === 'ok' || s.health === 'warn').length
      text = t('mp_t_n_ok', { n: ok, m: hits.length })
    } else if (hits.length) {
      // the worst sensor speaks for the tile / 最も悪いセンサーを表示
      const [, s] = [...hits].sort((a, b) => (b[1].health === 'fail' ? 1 : 0) - (a[1].health === 'fail' ? 1 : 0))[0]
      text = s.value === null || s.value === undefined ? (s.health ? t(`mp_h_${s.health}`) : '–')
        : `${valueText(t, s.value)}${s.unit && typeof s.value === 'number' ? ` ${s.unit}` : ''}`
    }
    return { key: d.key, part: d.part, icon: d.icon, text, tone: h ? HEALTH_TONE[h] : 'neutral' }
  })
}

/** Status tiles: icon, name, a dot AND text (never colour alone). / 状態タイル */
export function StatusTiles({ p, onPick }: { p: MiniPod; onPick: (part: PartId) => void }) {
  const t = useT()
  return (
    <ul className="grid grid-cols-2 gap-2 sm:grid-cols-3 2xl:grid-cols-6" aria-label={t('mp_status_tiles')}>
      {tiles(t, p.sensors).map((x) => {
        const Icon = x.icon
        return (
          <li key={x.key}>
            <button type="button" onClick={() => onPick(x.part)}
              className="flex h-full w-full flex-col items-center gap-1 rounded-xl border border-line bg-card/80 px-2 py-2.5 text-center shadow-[var(--shadow-card)] transition-colors duration-150 hover:border-line-strong">
              <Icon className="h-5 w-5 text-accent" aria-hidden />
              <span className="text-xs font-medium text-fg">{t(x.key)}</span>
              <span className={cn('flex max-w-full items-center gap-1 text-[11px]', TONE_TEXT[x.tone])}>
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
  // parts the sim names that we have no 3D place for / 3D に無い部品
  const known = new Set(PART_GROUPS.flatMap((g) => g.parts))
  const other = Object.keys(states).filter((k) => k && !known.has(k))
  const groups = other.length ? [...PART_GROUPS, { key: 'other', parts: other }] : PART_GROUPS
  return (
    <details className="group rounded-xl border border-line bg-card/60 px-3 py-2">
      <summary className="cursor-pointer select-none text-xs text-fg-mute marker:text-fg-mute">
        {t('mp_parts')} <span className="text-fg-mute/80">· {t('mp_parts_help')}</span>
      </summary>
      <div className="mt-2 flex flex-col gap-2">
        {groups.map((g) => (
          <div key={g.key}>
            <div className="mb-1 text-[10px] uppercase tracking-wider text-fg-mute">{t(`mp_g_${g.key}`)}</div>
            <div className="flex flex-wrap gap-1">
              {g.parts.map((p) => {
                const st = states[p]
                const tn = st?.level ? LEVEL_TONE[st.level] : 'neutral'
                return (
                  <button key={p} type="button" aria-pressed={selected === p} onClick={() => onPick(p)}
                    aria-label={`${partName(t, p)}: ${stateText(t, st)}`}
                    className={cn('flex items-center gap-1 rounded-md border px-1.5 py-0.5 text-[11px] transition-colors',
                      selected === p ? 'border-accent bg-accent/15 text-fg' : 'border-line bg-card-2 text-fg-2 hover:border-line-strong')}>
                    <span className={cn('num w-3 text-center', TONE_TEXT[tn])} aria-hidden>{st?.level ? HEALTH_MARK[st.level] : '·'}</span>
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
export function PartTooltip({ t, part, st, p, hint = true }: { t: TFunc; part: PartId; st: PartState | undefined; p: MiniPod; hint?: boolean }) {
  const tn = st?.level ? LEVEL_TONE[st.level] : 'neutral'
  const reading = partReading(t, part, p, st)
  return (
    <div className={cn('pointer-events-none w-max max-w-60 text-xs', hint && 'rounded-lg border border-line-strong bg-surface/95 px-2.5 py-1.5 shadow-lg')}>
      <div className="font-semibold text-fg">{partName(t, part)}</div>
      <div className={cn('flex items-center gap-1', TONE_TEXT[tn])}>
        <i className={cn('h-1.5 w-1.5 rounded-full', TONE_DOT[tn])} aria-hidden />{stateText(t, st)}
      </div>
      {reading && <div className="num text-fg-2">{reading}</div>}
      {hint && <div className="mt-0.5 text-[10px] text-fg-mute">{t('mp_click_details')}</div>}
    </div>
  )
}

/** A red label over a broken / faulty part. / 破損ラベル */
export function BrokenLabel({ t, part, st }: { t: TFunc; part: PartId; st: PartState | undefined }) {
  return (
    <div className="pointer-events-none flex w-max items-center gap-1 rounded-full border border-danger/70 bg-bg/85 px-2 py-0.5 text-[10px] font-semibold text-danger shadow">
      <span aria-hidden>✕</span>{st?.dmg === 'broken' ? t('mp_dmg_broken') : t('mp_h_fail')} · <span className="font-normal text-fg">{partName(t, part)}</span>
    </div>
  )
}
