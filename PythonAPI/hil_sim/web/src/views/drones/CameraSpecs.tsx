// One card per onboard camera (with the sensor cards): resolution, field of
// view, where it looks, what it is for. The pictures are not shown in the
// browser: "Show in CARLA view" opens the drone's CARLA window
// (drone_window.py), whose strip shows these cameras.
// 機載カメラの仕様カード（映像は CARLA の窓で表示）
import { MonitorPlay } from 'lucide-react'
import { useT } from '@/lib/i18n'
import { Button, KV } from '@/components/ui'
import { CAM_FPS as FPS, CAM_OF, CAM_SPEC, CAMERA_IDS, cardHealth, type CameraCard, type CardId } from './catalog'
import { SensorFrame } from './SensorCards'
import type { DroneS } from './types'

export function CameraSpecs({ d, selected, onLocate, onOpen, live }: {
  d: DroneS; selected: CardId | null; onLocate: (id: CardId) => void; onOpen: () => void; live: boolean
}) {
  const t = useT()
  return (
    <>
      {CAMERA_IDS.map((id: CameraCard) => {
        const c = CAM_OF[id]
        const sp = CAM_SPEC[c]
        return (
          <SensorFrame key={id} id={id} health={cardHealth(d.sensors, id)} selected={selected === id} onLocate={onLocate}>
            <KV k={t('dn_cam_res')} v={`${sp.w} × ${sp.h}`} />
            <KV k={t('dn_cam_fov')} v={`${sp.fov}°`} />
            <KV k={t('dn_cam_rate')} v={`${FPS} fps`} />
            <KV k={t('dn_cam_aim')} v={sp.pitch === -90 ? t('dn_cam_straight_down') : `${sp.yaw === 180 ? t('dn_cam_back') : t('dn_cam_fwd')}, ${sp.pitch}°`} />
            <p className="text-[11px] text-fg-mute">{t(`dn_cam_use_${c}`)}</p>
            <Button size="sm" variant="secondary" onClick={onOpen} disabled={!live} className="mt-auto">
              <MonitorPlay className="h-3.5 w-3.5" aria-hidden />{t('dn_cam_open_view')}
            </Button>
          </SensorFrame>
        )
      })}
    </>
  )
}
