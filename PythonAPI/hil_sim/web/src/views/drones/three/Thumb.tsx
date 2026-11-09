// A small still render of the same 3D drone: the overview thumbnail (whole
// drone) and the Component View tiles (a close-up of one part: FPV camera,
// motor, battery, winch). Rendered on demand (only when the part's state
// changes), so the tiles cost nothing between updates.
// 同じ 3D モデルの小さな静止画（全体・部品のクローズアップ）
import { Suspense, useEffect } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { partCentre, type PartId, type PartState, type V3 } from '../parts'
import DroneModel from './DroneModel'
import { Studio } from './Drone3D'

/** Camera offset from the part it looks at, per close-up. / クローズアップの視点 */
const VIEW: Record<string, { off: V3; fov: number; xray?: boolean }> = {
  all: { off: [0.62, 0.36, 0.66], fov: 30 },
  cam_fpv: { off: [0.17, 0.02, 0.12], fov: 30 },
  motor1: { off: [0.14, 0.09, 0.16], fov: 30 },
  battery: { off: [0.07, 0.16, 0.24], fov: 34, xray: true },
  winch: { off: [0.14, -0.07, 0.17], fov: 34 },
}

const aimAt = (focus: string): V3 => (focus === 'all' ? [0, 0.19, 0] : partCentre(focus))

function Aim({ focus }: { focus: string }) {
  const camera = useThree((s) => s.camera)
  const invalidate = useThree((s) => s.invalidate)
  useEffect(() => { camera.lookAt(...aimAt(focus)); camera.updateProjectionMatrix(); invalidate() }, [camera, focus, invalidate])
  return null
}

export default function Thumb({ focus, states }: { focus: PartId | 'all'; states: Record<PartId, PartState> }) {
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
        <DroneModel states={states} spinning={false} xray={!!v.xray} animate={false} />
      </Suspense>
    </Canvas>
  )
}
