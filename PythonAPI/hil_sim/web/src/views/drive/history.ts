import { Series } from '@/lib/canvas'
import { clamp } from '@/lib/fmt'
import { num, onTelemetry } from '@/lib/telemetry'

// Chart history for the Drive page (~60 s at 5 Hz). Fed at import time, so it
// keeps recording while another page is shown (old dashboard: H buffers).
// 運転ページのチャート履歴（約 60 秒）。他のページ表示中も記録を続ける。

export const HIST_LEN = 300

const mk = () => new Series(HIST_LEN)
export const H = {
  speed: mk(), limit: mk(), avg: mk(),
  throttle: mk(), brake: mk(), steer: mk(),
  lon: mk(), lat: mk(), jerk: mk(),
  yaw: mk(), steerY: mk(),
  ttc: mk(), thw: mk(),
  laneoff: mk(), laneEdgeP: mk(), laneEdgeN: mk(),
}

/** Last 150 (lon, lat) accelerations for the G-G diagram. / G-G 線図の点 */
export const GG: [number, number][] = []
const GG_LEN = 150

/** Number of samples recorded so far (redraw key for the charts). */
export let samples = 0

onTelemetry((s) => {
  if (!s.connected || !s.hero_found) return
  const steer = num(s, 'steer')
  const ttc = num(s, 'ttc', -1), thw = num(s, 'thw', -1)
  H.speed.push(num(s, 'speed_kmh'))
  H.limit.push(num(s, 'speed_limit_kmh'))
  H.avg.push(num(s, 'avg_kmh'))
  H.throttle.push(num(s, 'throttle') * 100)
  H.brake.push(num(s, 'brake') * 100)
  H.steer.push(steer * 100)
  H.lon.push(num(s, 'lon_accel'))
  H.lat.push(num(s, 'lat_accel'))
  H.jerk.push(clamp(num(s, 'jerk'), -20, 20))
  H.yaw.push(num(s, 'yaw_rate_deg'))
  H.steerY.push(steer * 100)
  H.ttc.push(ttc > 0 ? Math.min(ttc, 15) : 15)
  H.thw.push(thw > 0 ? Math.min(thw, 15) : 15)
  const lw2 = (num(s, 'lane_width') || 3.5) / 2
  H.laneoff.push(clamp(num(s, 'lane_offset'), -3, 3))
  H.laneEdgeP.push(lw2)
  H.laneEdgeN.push(-lw2)
  GG.push([num(s, 'lon_accel'), num(s, 'lat_accel')])
  if (GG.length > GG_LEN) GG.shift()
  samples++
})
