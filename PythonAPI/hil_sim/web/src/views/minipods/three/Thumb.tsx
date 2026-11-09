// A small still render of the same 3D pod: the overview thumbnail and the
// Component View close-ups (lidar, front camera, a wheel and its motor, the
// lid / payload bay). Rendered on demand, so the tiles cost nothing between
// updates.
// 同じ 3D モデルの小さな静止画（全体・部品のクローズアップ）
import { Suspense, useEffect } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { partCentre, type PartId, type PartState, type V3 } from '../parts'
import PodModel from './PodModel'
import { Studio } from './Pod3D'

/** Camera offset from the part it looks at, per close-up. / 視点 */
const VIEW: Record<string, { off: V3; fov: number; xray?: boolean; lid?: number }> = {
  all: { off: [1.05, 0.62, 1.1], fov: 30 },
  lidar: { off: [0.2, 0.14, 0.18], fov: 32 },
  cam_front: { off: [0.3, 0.06, 0.2], fov: 30 },
  wheel5: { off: [0.22, 0.12, -0.3], fov: 34 },
  lid: { off: [-0.5, 0.55, 0.45], fov: 34, lid: 1 },
}

const aimAt = (focus: string): V3 => (focus === 'all' ? [0, 0.36, 0] : partCentre(focus))

function Aim({ focus }: { focus: string }) {
  const camera = useThree((s) => s.camera)
  const invalidate = useThree((s) => s.invalidate)
  useEffect(() => { camera.lookAt(...aimAt(focus)); camera.updateProjectionMatrix(); invalidate() }, [camera, focus, invalidate])
  return null
}

export default function Thumb({ focus, states, flag = 1 }: { focus: PartId | 'all'; states: Record<PartId, PartState>; flag?: number }) {
  const v = VIEW[focus] ?? VIEW.all
  const target = aimAt(focus)
  const pos: V3 = [target[0] + v.off[0], target[1] + v.off[1], target[2] + v.off[2]]
  return (
    <Canvas frameloop="demand" dpr={[1, 2]} camera={{ position: pos, fov: v.fov, near: 0.01, far: 10 }}
      gl={{ antialias: true, alpha: true, toneMapping: THREE.ACESFilmicToneMapping }}
      style={{ pointerEvents: 'none' }} aria-hidden>
      <Aim focus={focus} />
      <Studio accentRim={false} />
      <Suspense fallback={null}>
        <PodModel states={states} speed={0} lid={v.lid ?? 0} shutter={0} flag={focus === 'all' ? flag : 0}
          lights={false} scanning={false} xray={!!v.xray} animate={false} />
      </Suspense>
    </Canvas>
  )
}
