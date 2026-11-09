// The LastMile Mini Delivery Pod in 3D: the real model (lmh_minipod.glb,
// served from web/public/models; the same model the CARLA prop is built
// from) with physical materials, its moving parts driven by the live state
// (six wheels rolling at the pod's speed, the top lid on its front hinge, the
// rear roller shutter, the telescopic flag raised while out), the sensors the
// GLB has no separate geometry for (lidar window, six cameras, GNSS, LTE,
// ultrasonic ring), the inside for the X-ray cutaway (Internals.tsx) and the
// damage the sim reports: a broken wheel is gone (shards where it was), a torn
// lid hangs askew, a cracked lidar shows its cracks, broken parts pulse red
// with a label, damaged / warning parts glow amber. Every part is pickable.
// 実モデルの 3D ミニポッド：物理マテリアル・車輪/蓋/シャッター/旗の動き・
// センサー・内部・損傷表示
import { useEffect, useMemo, useRef, type ReactNode, type RefObject } from 'react'
import { createPortal, useFrame, type ThreeEvent } from '@react-three/fiber'
import { Html, Line, useGLTF } from '@react-three/drei'
import * as THREE from 'three'
import { C } from '@/lib/colors'
import { crackPoints, shardGeometry } from '@/views/drones/three/geometry'
import { WHEEL_R, partCentre, wheelPos, type PartId, type PartState } from '../parts'
import { XRAY_MATS, fromGlb, paint } from './materials'
import Internals from './Internals'

/** web/public/models/lmh_minipod.glb (dashboard.py serves /models/). */
export const MODEL_URL = '/models/lmh_minipod.glb'

export interface ModelProps {
  states: Record<PartId, PartState>
  /** m/s: wheels roll at this speed / 車輪の回転 */
  speed: number
  /** 0..1 open / raised / 開閉量 */
  lid: number
  shutter: number
  flag: number
  /** headlights on (out on the street) / 前照灯 */
  lights: boolean
  /** lidar spinning / LiDAR 回転 */
  scanning: boolean
  xray: boolean
  animate: boolean
  selected?: PartId | null
  hovered?: PartId | null
  onHover?: (p: PartId | null) => void
  onPick?: (p: PartId) => void
  labels?: (p: PartId) => ReactNode
  interactive?: boolean
}

export type MatFor = (part: PartId, paintName: string) => THREE.MeshPhysicalMaterial

/** body meshes by GLB material -> our part / 本体メッシュの部品 */
const BODY_PART: Record<string, PartId> = {
  M_pod_headlight: 'headlights', M_pod_tail: 'taillights', M_hv: 'charge_port', M_pod_contact: 'charge_port',
  M_pod_belt: 'payload', M_ledB: 'lidar', M_ledG: 'charge_port', M_alu: 'charge_port',
}

const noRaycast = () => null
const LID_OPEN = -66 * Math.PI / 180
const easeTo = (cur: number, to: number, k: number) => cur + (to - cur) * k

interface Rig {
  root: THREE.Object3D
  wheels: (THREE.Object3D | null)[]
  lid: THREE.Object3D | null
  shutter: THREE.Object3D | null
  mast: THREE.Object3D | null
  flag: THREE.Object3D | null
  mastY: number
  flagY: number
  meshes: { mesh: THREE.Mesh; part: PartId; mat: string }[]
}

let SHARDS: THREE.BufferGeometry[] | null = null
const shards = () => (SHARDS ??= [1, 2, 3, 4, 5, 6].map(shardGeometry))

export default function PodModel(p: ModelProps) {
  const gltf = useGLTF(MODEL_URL)
  // per-instance materials, one per (part, material) so each part glows on its own
  // 部品ごとのマテリアル
  const mats = useMemo(() => new Map<string, { part: PartId; m: THREE.Material; base: THREE.Color | null; baseI: number; glb: string }>(), [])
  const rig = useMemo<Rig>(() => {
    const root = gltf.scene.clone(true)
    const find = (re: RegExp) => { let o: THREE.Object3D | null = null; root.traverse((x) => { if (!o && re.test(x.name)) o = x }); return o as THREE.Object3D | null }
    const wheels = [1, 2, 3, 4, 5, 6].map((n) => find(new RegExp(`_Wheel${n}$`)))
    const lid = find(/_Lid$/), shutter = find(/_Shutter$/), mast = find(/_FlagMast$/), flag = find(/_Flag$/)
    const meshes: Rig['meshes'] = []
    root.traverse((o) => {
      const mesh = o as THREE.Mesh
      if (!mesh.isMesh) return
      const parent = mesh.parent?.name ?? ''
      const mat = (Array.isArray(mesh.material) ? mesh.material[0] : mesh.material) as THREE.Material
      const w = /_Wheel(\d)$/.exec(parent)
      const part: PartId = w ? `wheel${w[1]}` : /_Lid$/.test(parent) ? 'lid' : /_Shutter$/.test(parent) ? 'shutter'
        : /_Flag(Mast)?$/.test(parent) ? 'flag' : BODY_PART[mat.name] ?? 'shell'
      const k = `${part}|${mat.name}`
      let e = mats.get(k)
      if (!e) {
        const m = fromGlb(mat)
        const std = m instanceof THREE.MeshStandardMaterial
        e = { part, m, base: std ? m.emissive.clone() : null, baseI: std ? m.emissiveIntensity : 0, glb: mat.name }
        mats.set(k, e)
      }
      mesh.material = e.m
      mesh.userData = { part }
      mesh.castShadow = false
      meshes.push({ mesh, part, mat: mat.name })
    })
    return { root, wheels, lid, shutter, mast, flag, mastY: mast?.position.y ?? 0.96, flagY: flag?.position.y ?? 1.235, meshes }
  }, [gltf, mats])

  // our own paints for the extra parts / 追加部品の材質
  const extra = useMemo(() => new Map<string, { part: PartId; m: THREE.MeshPhysicalMaterial; base: THREE.Color; baseI: number }>(), [])
  const matFor: MatFor = useMemo(() => (part, name) => {
    const k = `${part}|${name}`
    let e = extra.get(k)
    if (!e) {
      const m = paint(name)
      e = { part, m, base: m.emissive.clone(), baseI: m.emissiveIntensity }
      extra.set(k, e)
    }
    return e.m
  }, [extra])
  useEffect(() => () => {
    mats.forEach((e) => e.m.dispose()); mats.clear()
    extra.forEach((e) => e.m.dispose()); extra.clear()
  }, [mats, extra])

  const lvl = (k: string) => p.states[k]?.dmg

  // X-ray: the shell, lid and shutter go see-through; their meshes stop
  // catching the pointer so the inside can be picked / X 線表示
  useEffect(() => {
    mats.forEach((e) => {
      const o = XRAY_MATS[e.glb]
      if (o === undefined || /^wheel|^flag/.test(e.part)) return
      e.m.transparent = p.xray
      e.m.opacity = p.xray ? o : 1
      e.m.depthWrite = !p.xray
      e.m.side = p.xray ? THREE.DoubleSide : THREE.FrontSide
      e.m.needsUpdate = true
    })
    for (const { mesh, part, mat } of rig.meshes) {
      const see = p.xray && XRAY_MATS[mat] !== undefined && !/^wheel|^flag/.test(part)
      mesh.raycast = see ? noRaycast : THREE.Mesh.prototype.raycast
      mesh.renderOrder = see ? 2 : 0
    }
  }, [p.xray, mats, rig])

  // a broken wheel is gone / 破損した車輪は外れる
  const brokenWheels = [1, 2, 3, 4, 5, 6].filter((n) => lvl(`wheel${n}`) === 'broken').join(',')
  useEffect(() => {
    rig.wheels.forEach((w, i) => { if (w) w.visible = !brokenWheels.split(',').includes(String(i + 1)) })
  }, [rig, brokenWheels])

  // moving parts and glows, every frame / 可動部と発光（毎フレーム）
  const live = useRef(p)
  live.current = p
  const anim = useRef({ lid: p.lid, shutter: p.shutter, flag: p.flag, beam: 0 })
  const beam = useRef<THREE.Group>(null)
  const red = useMemo(() => new THREE.Color(C.danger), [])
  const amber = useMemo(() => new THREE.Color(C.warn), [])
  const green = useMemo(() => new THREE.Color(C.accent), [])
  useFrame(({ clock }, dt0) => {
    const q = live.current, a = anim.current, dt = Math.min(dt0, 0.05)
    const k = q.animate ? 1 - Math.pow(0.02, dt) : 1
    const st = (id: string) => q.states[id]?.dmg
    // wheels roll forward (about local z) / 車輪の回転
    const w = q.animate ? -(Math.min(Math.abs(q.speed), 4) * Math.sign(q.speed)) / WHEEL_R : 0
    rig.wheels.forEach((wh, i) => {
      if (!wh) return
      wh.rotation.z += w * dt
      wh.rotation.x = st(`wheel${i + 1}`) === 'damaged' ? 0.2 : 0
    })
    // lid on its front hinge; torn = askew / 蓋（破損時は傾く）
    a.lid = easeTo(a.lid, q.lid, k)
    if (rig.lid) {
      const torn = st('lid') === 'broken'
      rig.lid.rotation.z = torn ? -0.62 : LID_OPEN * a.lid + (st('lid') === 'damaged' ? -0.08 : 0)
      rig.lid.rotation.x = torn ? 0.22 : 0
    }
    // rear roller shutter rolls up (scale y) / シャッター
    a.shutter = easeTo(a.shutter, q.shutter, k)
    if (rig.shutter) {
      const bent = st('shutter') === 'broken'
      rig.shutter.scale.y = bent ? 0.55 : 1 - 0.96 * a.shutter
      rig.shutter.rotation.x = bent ? 0.22 : 0
    }
    // telescopic flag / 伸縮式の旗
    a.flag = easeTo(a.flag, q.flag, k)
    const r = a.flag, bentMast = st('flag') === 'broken'
    if (rig.mast) {
      rig.mast.position.y = rig.mastY - 0.259 * (1 - r)
      rig.mast.scale.y = 0.06 + 0.94 * r
      rig.mast.rotation.x = bentMast ? 0.6 * r : 0
    }
    if (rig.flag) {
      // a broken mast is bent over and its flag torn off / 折れた旗竿は旗なし
      rig.flag.position.y = rig.flagY - 0.517 * (1 - r)
      rig.flag.scale.setScalar(0.001 + 0.999 * r)
      rig.flag.visible = !bentMast
    }
    // lidar beam spins while scanning / LiDAR のビーム
    if (beam.current) {
      if (q.scanning && q.animate && st('lidar') !== 'broken') a.beam += dt * 9
      beam.current.rotation.y = a.beam
      beam.current.visible = q.scanning && st('lidar') !== 'broken'
    }
    // glows / 発光
    const t = clock.elapsedTime
    const pulse = q.animate ? 0.5 + 0.5 * Math.sin(t * 4.2) : 0.7
    const glow = (part: PartId, m: THREE.MeshStandardMaterial, base: THREE.Color, baseI: number) => {
      const lv = q.states[part]?.level
      // big panels glow softer, so the pod stays readable / 大きな面は控えめに
      const big = part === 'shell' || part === 'lid' ? 0.4 : 1
      if (lv === 'fail') { m.emissive.copy(red); m.emissiveIntensity = (0.35 + 0.75 * pulse) * big }
      else if (lv === 'warn') { m.emissive.copy(amber); m.emissiveIntensity = (0.32 + (q.animate ? 0.08 * Math.sin(t * 2) : 0)) * big }
      else if (part === q.selected) { m.emissive.copy(green); m.emissiveIntensity = 0.35 }
      else if (part === q.hovered) { m.emissive.copy(green); m.emissiveIntensity = 0.22 }
      else {
        m.emissive.copy(base)
        m.emissiveIntensity = part === 'headlights' ? (q.lights ? 2.4 : baseI) : part === 'taillights' ? (q.lights ? 1.4 : baseI) : baseI
      }
    }
    mats.forEach((e) => { if (e.base && e.m instanceof THREE.MeshStandardMaterial) glow(e.part, e.m, e.base, e.baseI) })
    extra.forEach((e) => glow(e.part, e.m, e.base, e.baseI))
  })

  const ev = p.interactive ? {
    onPointerMove: (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation()
      const part = (e.object.userData as { part?: string }).part ?? null
      if (part !== p.hovered) p.onHover?.(part)
    },
    onPointerOut: () => p.onHover?.(null),
    onClick: (e: ThreeEvent<MouseEvent>) => {
      e.stopPropagation()
      const part = (e.object.userData as { part?: string }).part
      if (part) p.onPick?.(part)
    },
  } : {}

  const lidarDmg = lvl('lidar'), shellDmg = lvl('shell'), lidDmg = lvl('lid')
  return (
    <group dispose={null} {...ev}>
      <primitive object={rig.root} />
      <Externals matFor={matFor} beam={beam} lidarDmg={lidarDmg} />
      <Internals matFor={matFor} />
      {/* the lid camera and the lid's cracks ride on the lid / 蓋に付く部品 */}
      {rig.lid && createPortal(<LidParts matFor={matFor} dmg={lidDmg} />, rig.lid)}
      {(shellDmg === 'damaged' || shellDmg === 'broken') && <ShellCracks broken={shellDmg === 'broken'} />}
      {/* shards where a broken wheel was / 外れた車輪の破片 */}
      {[1, 2, 3, 4, 5, 6].filter((n) => lvl(`wheel${n}`) === 'broken').map((n) => {
        const [x, , z] = wheelPos(n)
        return (
          <group key={n} position={[x, 0.012, z]}>
            {shards().map((g, i) => (
              <mesh key={i} geometry={g} material={matFor(`wheel${n}`, 'black')} raycast={noRaycast}
                position={[Math.cos(i * 1.9) * 0.07, 0, Math.sin(i * 1.9) * 0.05]} rotation={[i, i * 1.3, 0]} scale={1.6} />
            ))}
            {/* the bare axle stub / 車軸 */}
            <mesh position={[0, 0.073, (n % 2 ? 1 : -1) * 0.03]} rotation={[Math.PI / 2, 0, 0]} material={matFor(`wheel${n}`, 'steel')}
              userData={{ part: `wheel${n}` }}>
              <cylinderGeometry args={[0.012, 0.012, 0.05, 12]} />
            </mesh>
          </group>
        )
      })}
      {p.labels && Object.entries(p.states).filter(([, s]) => s.level === 'fail').map(([part]) => (
        <Html key={part} position={partCentre(part)} center zIndexRange={[20, 0]} style={{ pointerEvents: 'none' }}>
          {p.labels?.(part)}
        </Html>
      ))}
    </group>
  )
}

/** A lens: dark coated glass facing ``r``. / レンズ */
function Lens({ part, p, rad, r, matFor }: { part: PartId; p: [number, number, number]; rad: number; r: [number, number, number]; matFor: MatFor }) {
  return (
    <mesh position={p} rotation={r} material={matFor(part, 'lens')} userData={{ part }}>
      <sphereGeometry args={[rad, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2.4]} />
    </mesh>
  )
}

/** A camera: a small black housing and its lens along ``dir``. / カメラ */
function Camera({ part, p, dir, matFor }: { part: PartId; p: [number, number, number]; dir: 'x' | '-x' | 'z' | '-z' | '-y'; matFor: MatFor }) {
  const rot: Record<typeof dir, [number, number, number]> = {
    x: [0, 0, -Math.PI / 2], '-x': [0, 0, Math.PI / 2], z: [Math.PI / 2, 0, 0], '-z': [-Math.PI / 2, 0, 0], '-y': [Math.PI, 0, 0],
  }
  const off: Record<typeof dir, [number, number, number]> = {
    x: [0.007, 0, 0], '-x': [-0.007, 0, 0], z: [0, 0, 0.007], '-z': [0, 0, -0.007], '-y': [0, -0.007, 0],
  }
  const box: [number, number, number] = dir === 'x' || dir === '-x' ? [0.014, 0.03, 0.04] : dir === '-y' ? [0.04, 0.014, 0.03] : [0.04, 0.03, 0.014]
  return (
    <group position={p}>
      <mesh material={matFor(part, 'black')} userData={{ part }}><boxGeometry args={box} /></mesh>
      <Lens part={part} p={off[dir]} rad={0.0085} r={rot[dir]} matFor={matFor} />
    </group>
  )
}

/** Sensors outside the shell that the GLB has no separate mesh for. / 外部センサー */
function Externals({ matFor, beam, lidarDmg }: { matFor: MatFor; beam: RefObject<THREE.Group | null>; lidarDmg?: string }) {
  const cracks = useMemo(() => [
    crackPoints(2, 0.0, 0.0, 0.07, 0.6, 0),
    crackPoints(9, 0.0, 0.0, 0.06, 2.8, 0),
    crackPoints(4, 0.0, 0.0, 0.05, -1.4, 0),
  ], [])
  return (
    <group>
      {/* lidar: dark glass window with a blue LED band, a spinning beam inside
          LiDAR：暗いガラス窓・青い LED 帯・回転するビーム */}
      <group position={[0.27, 0.665, 0]}>
        <mesh position={[0, 0.026, 0]} material={matFor('lidar', 'glass')} userData={{ part: 'lidar' }}>
          <cylinderGeometry args={[0.048, 0.05, 0.052, 32]} />
        </mesh>
        <mesh position={[0, 0.053, 0]} material={matFor('lidar', 'black')} userData={{ part: 'lidar' }}>
          <cylinderGeometry args={[0.044, 0.048, 0.006, 32]} />
        </mesh>
        <mesh position={[0, 0.03, 0]} material={matFor('lidar', 'ledB')} userData={{ part: 'lidar' }}>
          <cylinderGeometry args={[0.0505, 0.0505, 0.004, 32, 1, true]} />
        </mesh>
        <group ref={beam} position={[0, 0.026, 0]}>
          <mesh position={[0.046, 0, 0]} raycast={noRaycast}>
            <boxGeometry args={[0.004, 0.03, 0.01]} />
            <meshBasicMaterial color="#7fd0ff" toneMapped={false} transparent opacity={0.85} />
          </mesh>
        </group>
        {lidarDmg && (
          <group position={[0, 0.0565, 0]} rotation={[0, 0.4, 0]} raycast={noRaycast}>
            {cracks.slice(0, lidarDmg === 'broken' ? 3 : 1).map((pts, i) => (
              <Line key={i} points={pts} color="#e8eef6" lineWidth={1.4} raycast={noRaycast} />
            ))}
          </group>
        )}
        {lidarDmg === 'broken' && shards().slice(0, 3).map((g, i) => (
          <mesh key={i} geometry={g} material={matFor('lidar', 'glass')} raycast={noRaycast}
            position={[0.06 + i * 0.01, 0.003, (i - 1) * 0.03]} rotation={[i, i * 2, 0]} />
        ))}
      </group>
      {/* the cameras, where delivery/assets/lmh_parts.json mounts them
          (sensors_mounts; CARLA x, y, z = model x, z, y) / カメラ */}
      <Camera part="cam_front" p={[0.358, 0.515, 0]} dir="x" matFor={matFor} />
      <Camera part="cam_rear" p={[-0.358, 0.64, 0]} dir="-x" matFor={matFor} />
      <Camera part="cam_left" p={[0.08, 0.58, -0.276]} dir="-z" matFor={matFor} />
      <Camera part="cam_right" p={[0.08, 0.58, 0.276]} dir="z" matFor={matFor} />
      <group position={[0.357, 0.6, 0]} rotation={[0, 0, -50 * Math.PI / 180]}>
        <Camera part="cam_down" p={[0, 0, 0]} dir="x" matFor={matFor} />
      </group>
      {/* GNSS puck and the LTE / V2X antenna on the fixed front deck / GNSS・LTE */}
      <mesh position={[0.21, 0.674, 0.12]} material={matFor('gnss', 'aluDark')} userData={{ part: 'gnss' }}>
        <cylinderGeometry args={[0.03, 0.033, 0.014, 28]} />
      </mesh>
      <mesh position={[0.21, 0.682, 0.12]} material={matFor('gnss', 'black')} userData={{ part: 'gnss' }}>
        <cylinderGeometry args={[0.022, 0.03, 0.004, 28]} />
      </mesh>
      <group position={[0.325, 0.666, 0.205]}>
        <mesh position={[0, 0.03, 0]} material={matFor('link', 'black')} userData={{ part: 'link' }}>
          <cylinderGeometry args={[0.005, 0.008, 0.06, 12]} />
        </mesh>
        <mesh position={[0, 0.062, 0]} material={matFor('link', 'safety')} userData={{ part: 'link' }}>
          <sphereGeometry args={[0.006, 12, 8]} />
        </mesh>
      </group>
      {/* ultrasonic sensors: 4 on the front bumper band, 2 at the rear (lmh_parts.json) / 超音波 */}
      {([[0.357, -0.19, 0.245], [0.357, -0.06, 0.245], [0.357, 0.06, 0.245], [0.357, 0.19, 0.245],
        [-0.358, -0.2, 0.215], [-0.358, 0.2, 0.215]] as const).map(([x, z, y], i) => (
        <mesh key={i} position={[x, y, z]} rotation={[0, 0, x > 0 ? -Math.PI / 2 : Math.PI / 2]}
          material={matFor('bumper', 'aluDark')} userData={{ part: 'bumper' }}>
          <cylinderGeometry args={[0.009, 0.009, 0.006, 16]} />
        </mesh>
      ))}
    </group>
  )
}

/** On the lid (moves with it): the payload camera under it, the lock bolt,
 *  and cracks when it is damaged / torn. Lid frame: hinge at the origin, the
 *  lid reaches back to x -0.53. / 蓋の上の部品（蓋と一緒に動く） */
function LidParts({ matFor, dmg }: { matFor: MatFor; dmg?: string }) {
  const cracks = useMemo(() => [
    crackPoints(5, -0.12, -0.05, 0.16, 2.6, 0.0025),
    crackPoints(11, -0.3, 0.1, 0.14, -0.4, 0.0025),
    crackPoints(7, -0.42, -0.12, 0.12, 1.2, 0.0025),
  ], [])
  return (
    <group>
      <group position={[-0.27, -0.03, 0]}>
        <Camera part="cam_lid" p={[0, 0, 0]} dir="-y" matFor={matFor} />
      </group>
      <mesh position={[-0.012, -0.03, 0]} material={matFor('lid_lock', 'steel')} userData={{ part: 'lid_lock' }}>
        <boxGeometry args={[0.02, 0.02, 0.06]} />
      </mesh>
      {dmg && cracks.slice(0, dmg === 'broken' ? 3 : 1).map((pts, i) => (
        <Line key={i} points={pts} color="#1d1f22" lineWidth={1.6} raycast={noRaycast} />
      ))}
    </group>
  )
}

/** Cracks on the left side panel and the nose. / 側面・前面のひび */
function ShellCracks({ broken }: { broken: boolean }) {
  const lines = useMemo(() => {
    const side = (seed: number, x: number, y: number, len: number, dir: number) =>
      crackPoints(seed, x, y, len, dir, 0).map((v) => new THREE.Vector3(v.x, v.z, -0.2735))
    const nose = (seed: number, z: number, y: number, len: number, dir: number) =>
      crackPoints(seed, z, y, len, dir, 0).map((v) => new THREE.Vector3(0.3535, v.z, v.x))
    return [side(3, 0.05, 0.55, 0.2, -2.2), side(8, -0.15, 0.36, 0.16, 0.5), nose(5, 0.08, 0.6, 0.12, -1.8),
      side(12, 0.2, 0.3, 0.12, 1.9)]
  }, [])
  return (
    <group raycast={noRaycast}>
      {lines.slice(0, broken ? 4 : 2).map((p, i) => <Line key={i} points={p} color="#1d1f22" lineWidth={1.6} raycast={noRaycast} />)}
    </group>
  )
}

useGLTF.preload(MODEL_URL)
