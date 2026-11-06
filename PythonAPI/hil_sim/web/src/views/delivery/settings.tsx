// Settings → Delivery sim: wind, the LastMile pod shell view and the CARLA
// tooltips (POST /delivery/wind, /delivery/pod). Imported by the Settings page.
// 設定 → 配送シミュ：風、ポッドの外殻表示、CARLA ツールチップ
import { useState } from 'react'
import '@/i18n/delivery'
import { useT } from '@/lib/i18n'
import { Badge, Card, Grid, Segmented } from '@/components/ui'
import WindCard from './WindCard'
import { sendCmd, useDeliveryState } from './shared'
import type { PodCmd, ShellMode } from './types'

const SHELLS: ShellMode[] = ['solid', 'cutaway', 'xray', 'frame']

export default function DeliverySettings() {
  const t = useT()
  const s = useDeliveryState()
  const [err, setErr] = useState('')
  const cmd = async (body: PodCmd) => {
    const r = await sendCmd('/delivery/pod', body)
    setErr(r.ok ? '' : r.error || 'error')
  }
  const live = !!(s?.available && s.running)
  const shell = s?.pod?.shell
  // like the old page: the tooltip state shows only while the sim runs
  // 旧ページ同様、稼働中のみ現在値を表示
  const tips: '0' | '1' | '' = live ? (s?.tooltips ? '1' : '0') : ''
  const run = !s?.available ? t('dr_none') : s.running ? t('dr_live') : t('dr_stopped')

  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-wrap items-center gap-2 text-sm text-fg-mute">
        <span>{t('dr_chip_run')}</span>
        <Badge tone={live ? 'good' : 'bad'} dot>{run}</Badge>
      </div>
      <Grid cols={3}>
        <WindCard state={s} className="md:col-span-2 xl:col-span-1 xl:row-span-2" />
        <Card title={t('pd_view')} actions={shell ? <Badge tone="info">{t(`pd_shell_${shell}`)}</Badge> : null}>
          <Segmented<ShellMode | ''> label={t('pd_view')} value={shell ?? ''}
            options={SHELLS.map((m) => ({ value: m, label: t(`pd_shell_${m}`) }))}
            onChange={(m) => { if (m) void cmd({ cmd: 'shell', mode: m }) }} />
          <p className="text-xs text-fg-mute">{t('pd_view_hint')}</p>
        </Card>
        <Card title={t('tt_title')}>
          <Segmented<'0' | '1' | ''> label={t('tt_title')} value={tips}
            options={[{ value: '0', label: t('tt_off') }, { value: '1', label: t('tt_on') }]}
            onChange={(v) => { if (v) void cmd({ cmd: 'tooltips', on: v === '1' }) }} />
          <p className="text-xs text-fg-mute">{t('tt_hint')}</p>
        </Card>
      </Grid>
      {err && <p className="text-sm text-warn" role="alert">{err}</p>}
    </div>
  )
}
