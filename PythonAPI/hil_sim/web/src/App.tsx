import { lazy, Suspense, useEffect, useState, type ComponentType } from 'react'
import { Activity, BarChart3, Bot, BrainCircuit, CarFront, Download, Gauge, Package, Plane, Settings2, Square } from 'lucide-react'
import '@/i18n/common'
import { useLang, useT } from '@/lib/i18n'
import { simPhase, startSim, useSim, type SimPhase } from '@/lib/sim'
import { startTelemetry, useBackendOnline, useTelemetry } from '@/lib/telemetry'
import { launcherAPI } from '@/lib/api'
import { Badge, Button, Segmented, type Tone } from '@/components/ui'
import { cn } from '@/lib/utils'
import LaunchView from '@/views/launch'
// alert rules, beeps and the banner, on every page (not only Drive) / 警告は全ページで
import { DriveAlertBar } from '@/views/drive/alerts'

// Views load on first visit (the Delivery map and FFB pages are heavy).
// ビューは初回表示時に読み込む
const DriveView = lazy(() => import('@/views/drive'))
const VehicleView = lazy(() => import('@/views/vehicle'))
const DeliveryView = lazy(() => import('@/views/delivery'))
const DronesView = lazy(() => import('@/views/drones'))
const MiniPodsView = lazy(() => import('@/views/minipods'))
const StatsView = lazy(() => import('@/views/stats'))
const SettingsView = lazy(() => import('@/views/settings'))
const TrainingView = lazy(() => import('@/views/training'))

type ViewKey = 'sim' | 'drive' | 'vehicle' | 'delivery' | 'drones' | 'minipods' | 'stats' | 'training' | 'settings'
const VIEWS: { key: ViewKey; icon: ComponentType<{ className?: string }>; Comp: ComponentType }[] = [
  { key: 'sim', icon: Activity, Comp: LaunchView },
  { key: 'drive', icon: Gauge, Comp: DriveView },
  { key: 'vehicle', icon: CarFront, Comp: VehicleView },
  { key: 'delivery', icon: Package, Comp: DeliveryView },
  { key: 'drones', icon: Plane, Comp: DronesView },
  { key: 'minipods', icon: Bot, Comp: MiniPodsView },
  { key: 'stats', icon: BarChart3, Comp: StatsView },
  { key: 'training', icon: BrainCircuit, Comp: TrainingView },
  { key: 'settings', icon: Settings2, Comp: SettingsView },
]

/** #drive, #settings/ffb ... (deep links; the back button works). */
function useHashRoute(): [ViewKey, string, (v: ViewKey, sub?: string) => void] {
  const parse = (): [ViewKey, string] => {
    const [v, sub] = window.location.hash.replace(/^#/, '').split('/')
    return [VIEWS.some((x) => x.key === v) ? (v as ViewKey) : 'sim', sub ?? '']
  }
  const [route, setRoute] = useState(parse)
  useEffect(() => {
    const on = () => setRoute(parse())
    window.addEventListener('hashchange', on)
    return () => window.removeEventListener('hashchange', on)
  }, [])
  const go = (v: ViewKey, sub?: string) => { window.location.hash = sub ? `${v}/${sub}` : v }
  return [route[0], route[1], go]
}

const PHASE_TONE: Record<SimPhase, Tone> = {
  offline: 'bad', idle: 'neutral', starting: 'warn', running: 'good', switching: 'warn', stopping: 'warn',
}

function StopButton() {
  const t = useT()
  const [sure, setSure] = useState(false)
  useEffect(() => { if (!sure) return; const id = setTimeout(() => setSure(false), 3000); return () => clearTimeout(id) }, [sure])
  return (
    <Button variant="danger" size="sm" onClick={() => { if (sure) { launcherAPI.stop(); setSure(false) } else setSure(true) }}>
      <Square className="h-3.5 w-3.5" aria-hidden /> {sure ? t('btn_stop_sure') : t('btn_stop')}
    </Button>
  )
}

export default function App() {
  const t = useT()
  const { lang, setLang } = useLang()
  const sim = useSim()
  const online = useBackendOnline()
  const tel = useTelemetry()
  // no launcher endpoint (an older backend): judge by the telemetry connection
  // ランチャー API が無い場合はテレメトリの接続で判断
  const phase: SimPhase = sim ? simPhase(sim) : tel?.connected ? 'running' : online ? 'idle' : 'offline'
  const [view, sub, go] = useHashRoute()

  useEffect(() => { startSim(); startTelemetry() }, [])
  // first visit: the Launch page when nothing runs, else Drive
  // 初回: 停止中は起動ページ、実行中は運転ページ
  const [routed, setRouted] = useState(false)
  useEffect(() => {
    if (routed || !sim) return
    setRouted(true)
    if (!window.location.hash) go(phase === 'running' ? 'drive' : 'sim')
  }, [sim, routed, phase, go])

  const live = phase === 'running' || phase === 'switching'
  const status = t(`st_${phase}`)
  const Current = VIEWS.find((v) => v.key === view)!.Comp

  return (
    <div className="min-h-screen">
      <header className="sticky top-0 z-30 border-b border-line bg-bg/85 backdrop-blur-md">
        <div className="mx-auto flex max-w-[1800px] flex-wrap items-center gap-x-4 gap-y-2 px-4 py-2.5">
          <a href="#sim" className="flex items-center gap-2 font-mono text-sm font-semibold tracking-wider text-fg">
            <span className="grid h-7 w-7 place-items-center rounded-lg bg-accent/15 text-accent">▲</span>
            CARLA <span className="text-accent">HIL</span>
          </a>
          <nav aria-label="Views" className="order-3 w-full overflow-x-auto md:order-none md:w-auto">
            <ul className="flex gap-1">
              {VIEWS.map(({ key, icon: Icon }) => (
                <li key={key}>
                  <a href={`#${key}`} aria-current={view === key ? 'page' : undefined}
                    className={cn('flex h-9 items-center gap-2 rounded-lg px-3 text-sm whitespace-nowrap transition-colors duration-150',
                      view === key ? 'bg-card-2 text-fg' : 'text-fg-mute hover:bg-card hover:text-fg')}>
                    <Icon className="h-4 w-4" aria-hidden />{t(`nav_${key}`)}
                  </a>
                </li>
              ))}
            </ul>
          </nav>
          <div className="ml-auto flex items-center gap-2">
            <Badge tone={PHASE_TONE[phase]} dot>{status}</Badge>
            {live && sim?.pipeline?.map && <Badge>{sim.pipeline.map}</Badge>}
            {live && tel && !tel.hero_found && <Badge tone="warn">{t('st_hero_wait')}</Badge>}
            {live && <StopButton />}
            <a href="/download" className="hidden h-8 items-center gap-1.5 rounded-lg border border-line px-2.5 text-xs text-fg-2 hover:bg-muted sm:inline-flex">
              <Download className="h-3.5 w-3.5" aria-hidden />{t('csv')}
            </a>
            <Segmented size="sm" label={t('lang')} value={lang} onChange={setLang}
              options={[{ value: 'en', label: 'EN' }, { value: 'ja', label: '日本語' }]} />
          </div>
        </div>
      </header>
      <DriveAlertBar />
      <main className="mx-auto max-w-[1800px] px-4 py-5">
        <Suspense fallback={<div className="p-10 text-center text-fg-mute">…</div>}>
          <Current key={view} {...({ section: sub } as object)} />
        </Suspense>
      </main>
    </div>
  )
}
