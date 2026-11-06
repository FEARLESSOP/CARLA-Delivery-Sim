import { lazy, Suspense, type ComponentType } from 'react'
import { Car, Gauge, Keyboard, Package } from 'lucide-react'
import '@/i18n/settings'
import { useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { EmptyState, Segmented } from '@/components/ui'

// Settings: every control of the rig in four sections (#settings/<section>):
// driving (pedals, vehicle performance), force feedback, the delivery sim
// (DeliverySettings from the delivery view) and keybinds. Only the shown
// section is mounted, so only its routes are polled (like secVisible()).
// 設定ページ：4 つのセクション。表示中のセクションだけを読み込み・取得する。

const DrivingSection = lazy(() => import('./driving'))
const FfbSection = lazy(() => import('./ffb'))
const KeybindsSection = lazy(() => import('./keybinds'))
// written by the delivery port; loaded lazily so this page never waits on it
// 配送ビューの担当が作成（遅延読み込み）
const DeliverySettings = lazy(() => import('@/views/delivery/settings'))

type Section = 'driving' | 'ffb' | 'delivery' | 'keybinds'
const SECTIONS: { key: Section; icon: ComponentType<{ className?: string }>; Comp: ComponentType }[] = [
  { key: 'driving', icon: Car, Comp: DrivingSection },
  { key: 'ffb', icon: Gauge, Comp: FfbSection },
  { key: 'delivery', icon: Package, Comp: DeliverySettings },
  { key: 'keybinds', icon: Keyboard, Comp: KeybindsSection },
]
const isSection = (s: string | undefined): s is Section => SECTIONS.some((x) => x.key === s)

export default function SettingsView({ section }: { section?: string }) {
  const t = useT()
  const cur: Section = isSection(section) ? section : 'driving'
  const go = (s: Section) => { window.location.hash = `settings/${s}` }
  const Current = SECTIONS.find((s) => s.key === cur)!.Comp

  return (
    <div className="grid gap-5 lg:grid-cols-[13rem_minmax(0,1fr)] 2xl:grid-cols-[15rem_minmax(0,1fr)]">
      <aside className="lg:sticky lg:top-20 lg:self-start">
        <h1 className="mb-1 text-2xl font-semibold">{t('s_title')}</h1>
        <p className="mb-3 text-xs text-fg-mute">{t('set_note')}</p>
        {/* phones / tablets: segmented / スマホ：セグメント */}
        <div className="overflow-x-auto lg:hidden">
          <Segmented label={t('s_sections')} value={cur} onChange={go}
            options={SECTIONS.map((s) => ({ value: s.key, label: t(`set_${s.key}`) }))} />
        </div>
        {/* desktop: side nav / PC：サイドナビ */}
        <nav aria-label={t('s_sections')} className="hidden lg:block">
          <ul className="space-y-1">
            {SECTIONS.map(({ key, icon: Icon }) => (
              <li key={key}>
                <a href={`#settings/${key}`} aria-current={cur === key ? 'page' : undefined}
                  className={cn('flex h-10 items-center gap-2.5 rounded-lg border px-3 text-sm transition-colors duration-150',
                    cur === key ? 'border-accent/40 bg-accent/10 font-medium text-fg' : 'border-transparent text-fg-mute hover:bg-card hover:text-fg')}>
                  <Icon className={cn('h-4 w-4', cur === key ? 'text-accent' : '')} aria-hidden />
                  {t(`set_${key}`)}
                </a>
              </li>
            ))}
          </ul>
        </nav>
      </aside>
      <section aria-label={t(`set_${cur}`)} className="min-w-0">
        {cur !== 'delivery' && <h2 className="mb-3 text-lg font-semibold text-fg">{t(`set_${cur}`)}</h2>}
        <Suspense fallback={<EmptyState>{t('s_loading')}</EmptyState>}>
          <Current key={cur} />
        </Suspense>
      </section>
    </div>
  )
}
