// What the model has no geometry for inside the shell, sized and placed like
// a real delivery robot of this size (seen in X-ray): the 48 V battery pack
// (cells + BMS board) under the payload floor, the compute unit with its heat
// sink behind the nose, six in-wheel drive motors (housing + windings just
// inboard of each wheel), the motor wiring, the payload belt's drive roller and the lid lock
// actuator. Model frame: metres, +x nose, +y up, +z right; the payload bay
// floor (belt) is at y 0.30, the chassis rails at y 0.08..0.14.
// モデルに無い内部部品（X 線で見える）
import { useMemo } from 'react'
import { RoundedBox } from '@react-three/drei'
import * as THREE from 'three'
import { wheelPos } from '../parts'
import type { MatFor } from './PodModel'

type V = [number, number, number]

function Box({ part, paint, p, s, matFor, r }: { part: string; paint: string; p: V; s: V; matFor: MatFor; r?: V }) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, paint)} userData={{ part }}>
      <boxGeometry args={s} />
    </mesh>
  )
}
function Cyl({ part, paint, p, rad, h, matFor, r, seg = 20 }: {
  part: string; paint: string; p: V; rad: number; h: number; matFor: MatFor; r?: V; seg?: number
}) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, paint)} userData={{ part }}>
      <cylinderGeometry args={[rad, rad, h, seg]} />
    </mesh>
  )
}

export default function Internals({ matFor }: { matFor: MatFor }) {
  const o = { matFor }
  // wiring: battery -> each hub motor (red + black pair) / 配線
  const wires = useMemo(() => [1, 2, 3, 4, 5, 6].flatMap((n) => {
    const [x, , z] = wheelPos(n)
    const s = Math.sign(z)
    return [0, 1].map((k) => {
      const off = k ? 0.004 : -0.004
      const curve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(-0.05 + off, 0.2, s * 0.12),
        new THREE.Vector3(x * 0.8 + off, 0.16, s * 0.15),
        new THREE.Vector3(x + off, 0.11, s * 0.17),
        new THREE.Vector3(x + off, 0.088, s * 0.185),
      ])
      return { n, k, geo: new THREE.TubeGeometry(curve, 20, 0.0022, 6, false) }
    })
  }), [])
  return (
    <group>
      {/* ---- battery pack: 4 x 6 cylindrical cells in a case + BMS / 電池パック */}
      <group position={[-0.05, 0.215, 0]}>
        <RoundedBox args={[0.3, 0.1, 0.3]} radius={0.008} material={matFor('battery', 'aluDark')} userData={{ part: 'battery' }} />
        {Array.from({ length: 24 }, (_, i) => {
          const cx = (i % 6) - 2.5, cz = Math.floor(i / 6) - 1.5
          return <Cyl key={i} part="battery" paint="cells" p={[cx * 0.045, 0.012, cz * 0.065]} rad={0.019} h={0.09} seg={14} {...o} />
        })}
        <Box part="battery" paint="pcb" p={[0, 0.057, 0]} s={[0.24, 0.004, 0.14]} {...o} />
        {[-0.08, 0, 0.08].map((x) => <Box key={x} part="battery" paint="black" p={[x, 0.062, 0.03]} s={[0.018, 0.006, 0.018]} {...o} />)}
        <Box part="battery" paint="copper" p={[0.13, 0.058, -0.05]} s={[0.03, 0.004, 0.02]} {...o} />
      </group>
      {/* ---- compute unit behind the nose (perception + planning) / 計算機 */}
      <group position={[0.24, 0.245, 0]}>
        <RoundedBox args={[0.11, 0.05, 0.2]} radius={0.004} material={matFor('compute', 'aluDark')} userData={{ part: 'compute' }} />
        {[-3, -2, -1, 0, 1, 2, 3].map((i) => (
          <Box key={i} part="compute" paint="alu" p={[0, 0.033, i * 0.026]} s={[0.1, 0.018, 0.004]} {...o} />
        ))}
        <Box part="compute" paint="pcb" p={[0, -0.028, 0]} s={[0.1, 0.004, 0.18]} {...o} />
        <Box part="compute" paint="black" p={[0.02, -0.022, 0.04]} s={[0.03, 0.006, 0.03]} {...o} />
        <Box part="compute" paint="gold" p={[-0.04, -0.024, -0.06]} s={[0.012, 0.003, 0.04]} {...o} />
      </group>
      {/* ---- six hub motors: windings inside each wheel hub / ハブモーター */}
      {[1, 2, 3, 4, 5, 6].map((n) => {
        const [x, y, z] = wheelPos(n)
        const s = Math.sign(z)
        return (
          <group key={n} position={[x, y, s * 0.188]} rotation={[Math.PI / 2, 0, 0]}>
            <Cyl part={`wheel${n}`} paint="aluDark" p={[0, 0, 0]} rad={0.034} h={0.026} seg={24} {...o} />
            <Cyl part={`wheel${n}`} paint="copper" p={[0, 0, 0]} rad={0.027} h={0.032} seg={24} {...o} />
            <Cyl part={`wheel${n}`} paint="steel" p={[0, -s * 0.02, 0]} rad={0.01} h={0.03} seg={12} {...o} />
          </group>
        )
      })}
      {wires.map((w) => (
        <mesh key={`${w.n}-${w.k}`} geometry={w.geo} material={matFor('battery', w.k ? 'wireRed' : 'wire')} userData={{ part: 'battery' }} />
      ))}
      {/* ---- payload belt drive roller under the bay floor / ベルト駆動ローラー */}
      <Cyl part="payload" paint="alu" p={[0.13, 0.285, 0]} rad={0.014} h={0.4} r={[Math.PI / 2, 0, 0]} {...o} />
      <Cyl part="payload" paint="black" p={[0.13, 0.285, 0.215]} rad={0.022} h={0.03} r={[Math.PI / 2, 0, 0]} {...o} />
      {/* ---- lid lock solenoid at the front of the bay / 蓋ロック */}
      <Box part="lid_lock" paint="aluDark" p={[0.165, 0.62, 0]} s={[0.03, 0.04, 0.07]} {...o} />
      <Box part="lid_lock" paint="copper" p={[0.165, 0.62, 0]} s={[0.022, 0.03, 0.04]} {...o} />
    </group>
  )
}
