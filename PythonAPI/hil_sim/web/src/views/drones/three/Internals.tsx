// What the model has no geometry for, sized and placed like the real thing:
// inside the fuselage (seen in X-ray): the flight-controller stack with its
// heat sink, the barometer under its foam cap, the power distribution board,
// the radio module, the winch drum with its tether, the cells of the hot-swap
// battery and the wiring to the four ESCs; outside: the front / rear stereo
// pairs, the delivery and optical-flow cameras, glass for every lens, the
// datalink antennas and the Remote ID module. Positions: model frame
// (metres, +x nose, +y up, +z starboard), inside the core box
// x +-0.15, y 0.189..0.268, z -0.097..0.097 (the battery fills z > 0.018).
// モデルに無い内部部品・センサー（X 線で見える）
import { useMemo, type ReactNode } from 'react'
import { RoundedBox } from '@react-three/drei'
import * as THREE from 'three'
import { CORNER } from '../parts'
import type { MatFor } from './DroneModel'

type V = [number, number, number]

/** A box / cylinder of one part. / 部品の箱・円柱 */
function Box({ part, paint, p, s, matFor, outline, r }: {
  part: string; paint: string; p: V; s: V; matFor: MatFor; outline: (p: string) => ReactNode; r?: V
}) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, paint)} userData={{ part }}>
      <boxGeometry args={s} />
      {outline(part)}
    </mesh>
  )
}
function Cyl({ part, paint, p, rad, h, matFor, outline, r, seg = 20 }: {
  part: string; paint: string; p: V; rad: number; h: number; matFor: MatFor; outline: (p: string) => ReactNode; r?: V; seg?: number
}) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, paint)} userData={{ part }}>
      <cylinderGeometry args={[rad, rad, h, seg]} />
      {outline(part)}
    </mesh>
  )
}

/** A lens: dark coated glass, facing ``r`` (cylinder axis = y before rotating). */
function Lens({ part, p, rad, r, matFor }: { part: string; p: V; rad: number; r: V; matFor: MatFor }) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, 'lens')} userData={{ part }}>
      <sphereGeometry args={[rad, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2.4]} />
    </mesh>
  )
}

export default function Internals({ matFor, outline }: { matFor: MatFor; outline: (p: string) => ReactNode }) {
  const o = { matFor, outline }
  // wiring: power board -> each ESC (red + black pair) / 配線
  const wires = useMemo(() => [1, 2, 3, 4].flatMap((n) => {
    const [fx, fz] = CORNER[n]
    return [0, 1].map((k) => {
      const off = (k ? 0.003 : -0.003)
      const curve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(-0.07 + off, 0.21, -0.03 + off),
        new THREE.Vector3(fx * 0.06, 0.226, fz * 0.05 + off),
        new THREE.Vector3(fx * 0.125 + off, 0.25, fz * 0.125),
        new THREE.Vector3(fx * 0.175 + off, 0.258, fz * 0.18),
      ])
      return { n, k, geo: new THREE.TubeGeometry(curve, 24, 0.0013, 6, false) }
    })
  }), [])
  const tether = useMemo(() => new THREE.TorusGeometry(0.0175, 0.0025, 8, 32), [])
  return (
    <group>
      {/* ---- flight controller stack (IMU + compute) / フライトコントローラ */}
      <group position={[0.035, 0.226, -0.048]}>
        <RoundedBox args={[0.056, 0.003, 0.046]} radius={0.001} material={matFor('imu', 'pcb')} userData={{ part: 'imu' }}>{outline('imu')}</RoundedBox>
        <RoundedBox args={[0.056, 0.003, 0.046]} radius={0.001} position={[0, 0.012, 0]} material={matFor('imu', 'pcb')} userData={{ part: 'imu' }} />
        {[[-0.024, -0.019], [0.024, -0.019], [-0.024, 0.019], [0.024, 0.019]].map(([x, z], i) => (
          <Cyl key={i} part="imu" paint="gold" p={[x, 0.006, z]} rad={0.0014} h={0.012} seg={8} {...o} />
        ))}
        <Box part="imu" paint="M_black" p={[0.004, 0.0035, 0.002]} s={[0.014, 0.003, 0.014]} {...o} />
        {/* heat sink fins / ヒートシンク */}
        {[-2, -1, 0, 1, 2].map((i) => (
          <Box key={i} part="imu" paint="M_alu" p={[i * 0.007, 0.019, 0]} s={[0.0022, 0.009, 0.034]} {...o} />
        ))}
        <Box part="imu" paint="M_alu" p={[0, 0.0145, 0]} s={[0.036, 0.002, 0.036]} {...o} />
      </group>
      {/* ---- barometer under its foam cap / 気圧計 */}
      <Box part="baro" paint="pcb" p={[-0.025, 0.228, -0.06]} s={[0.022, 0.002, 0.02]} {...o} />
      <Box part="baro" paint="foam" p={[-0.025, 0.236, -0.06]} s={[0.014, 0.012, 0.014]} {...o} />
      {/* ---- power distribution board / 電源分配基板 */}
      <group position={[-0.075, 0.205, -0.03]}>
        <RoundedBox args={[0.06, 0.003, 0.07]} radius={0.001} material={matFor('pdb', 'pcb')} userData={{ part: 'pdb' }}>{outline('pdb')}</RoundedBox>
        {[[-0.02, -0.025], [0.02, -0.025], [-0.02, 0.025], [0.02, 0.025]].map(([x, z], i) => (
          <Box key={i} part="pdb" paint="copper" p={[x, 0.002, z]} s={[0.012, 0.0012, 0.01]} {...o} />
        ))}
        <Cyl part="pdb" paint="M_black" p={[0, 0.007, 0]} rad={0.005} h={0.011} {...o} />
        <Cyl part="pdb" paint="M_black" p={[0.012, 0.006, 0.006]} rad={0.004} h={0.009} {...o} />
      </group>
      {/* ---- radio (C2 + LTE) module / 無線モジュール */}
      <Box part="link" paint="M_aluDark" p={[-0.118, 0.232, -0.058]} s={[0.026, 0.012, 0.03]} {...o} />
      {[-0.008, 0.008].map((z, i) => (
        <Cyl key={i} part="link" paint="gold" p={[-0.133, 0.232, -0.058 + z]} rad={0.0022} h={0.007} r={[0, 0, Math.PI / 2]} seg={10} {...o} />
      ))}
      {/* datalink antennas on the rear arms / 後部アームのアンテナ */}
      {[-1, 1].map((s) => (
        <group key={s} position={[-0.2, 0.18, s * 0.2]}>
          <Cyl part="link" paint="M_black" p={[0, 0.03, 0]} rad={0.0032} h={0.06} seg={12} {...o} />
          <Cyl part="link" paint="M_white" p={[0, 0.063, 0]} rad={0.0035} h={0.006} seg={12} {...o} />
        </group>
      ))}
      {/* ---- winch drum with the tether wound on it / ウインチのドラム */}
      <group position={[0, 0.205, -0.01]}>
        <Cyl part="winch" paint="M_alu" p={[0, 0, 0]} rad={0.015} h={0.032} r={[Math.PI / 2, 0, 0]} {...o} />
        <Cyl part="winch" paint="M_aluDark" p={[0, 0, -0.018]} rad={0.02} h={0.003} r={[Math.PI / 2, 0, 0]} {...o} />
        <Cyl part="winch" paint="M_aluDark" p={[0, 0, 0.018]} rad={0.02} h={0.003} r={[Math.PI / 2, 0, 0]} {...o} />
        {[-0.01, -0.004, 0.002, 0.008].map((z, i) => (
          <mesh key={i} geometry={tether} position={[0, 0, z]} material={matFor('winch', 'M_safety')} userData={{ part: 'winch' }} />
        ))}
        <Cyl part="winch" paint="M_black" p={[0, 0, -0.03]} rad={0.011} h={0.02} r={[Math.PI / 2, 0, 0]} {...o} />
      </group>
      {/* ---- the hot-swap battery's cells (inside its case) / 電池セル */}
      <RoundedBox args={[0.13, 0.04, 0.118]} radius={0.006} position={[0, 0.229, 0.088]}
        material={matFor('battery', 'cells')} userData={{ part: 'battery' }}>{outline('battery')}</RoundedBox>
      <Box part="battery" paint="M_white" p={[0, 0.2495, 0.088]} s={[0.07, 0.0012, 0.05]} {...o} />
      {/* ---- wiring / 配線 */}
      {wires.map((w) => (
        <mesh key={`${w.n}-${w.k}`} geometry={w.geo} material={matFor('pdb', w.k ? 'wireRed' : 'wire')} userData={{ part: 'pdb' }} />
      ))}

      {/* ===== outside / 外部 ===== */}
      {/* front / rear stereo pairs on the shell / 前後ステレオ */}
      {[1, -1].map((s) => {
        const part = s > 0 ? 'cam_front' : 'cam_rear'
        return (
          <group key={s} position={[s * 0.176, 0.282, 0]}>
            <RoundedBox args={[0.018, 0.016, 0.092]} radius={0.004} material={matFor(part, 'M_black')} userData={{ part }}>{outline(part)}</RoundedBox>
            {[-0.032, 0.032].map((z, i) => (
              <Lens key={i} part={part} p={[s * 0.0092, 0, z]} rad={0.0062} r={[0, 0, -s * Math.PI / 2]} matFor={matFor} />
            ))}
          </group>
        )
      })}
      {/* delivery camera and optical flow under the nose / 配達カメラ・フロー */}
      {([['cam_down', 0.042], ['flow', -0.042]] as const).map(([part, z]) => (
        <group key={part} position={[0.128, 0.18, z]}>
          <Cyl part={part} paint="M_black" p={[0, 0, 0]} rad={0.012} h={0.014} {...o} />
          <Lens part={part} p={[0, -0.007, 0]} rad={0.0075} r={[Math.PI, 0, 0]} matFor={matFor} />
        </group>
      ))}
      {/* lidar window and FPV front lens / LiDAR の窓・FPV のレンズ */}
      <Lens part="rangefinder" p={[0.097, 0.1715, 0]} rad={0.009} r={[Math.PI, 0, 0]} matFor={matFor} />
      <Lens part="cam_fpv" p={[0.2045, 0.189, 0]} rad={0.0085} r={[0, 0, -Math.PI / 2]} matFor={matFor} />
      {/* Remote ID module on the shell / リモート ID */}
      <RoundedBox args={[0.034, 0.008, 0.022]} radius={0.002} position={[0, 0.315, 0.062]}
        material={matFor('remote_id', 'M_aluDark')} userData={{ part: 'remote_id' }}>{outline('remote_id')}</RoundedBox>
      <Box part="remote_id" paint="M_ledG" p={[0.012, 0.3195, 0.062]} s={[0.003, 0.0015, 0.003]} {...o} />
    </group>
  )
}
