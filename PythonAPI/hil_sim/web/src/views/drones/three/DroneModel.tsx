// The LastMile drone in 3D: the real model (lmp_drone_parts.glb, split into
// named parts by delivery/assets/make_drone_web.py) with physical materials,
// procedural twisted rotors spinning at the live motor RPM, the inside of the
// airframe (Internals.tsx) for the X-ray cutaway, and the damage the sim
// reports drawn on the parts: a broken arm droops with its motor and rotor, a
// broken rotor loses a blade and bends the other, a cracked shell shows its
// cracks, broken parts pulse red with a label, damaged / warning parts glow
// amber. Every part is pickable (hover / click), materials are per part so a
// part can glow on its own.
// 実モデルの 3D ドローン：物理マテリアル・回転するローター・内部・損傷表示
import { useEffect, useMemo, useRef, type ReactNode } from 'react'
import { useFrame, type ThreeEvent } from '@react-three/fiber'
import { Html, Line, useGLTF } from '@react-three/drei'
import * as THREE from 'three'
import { C } from '@/lib/colors'
import { CORNER, HUB_R, HUB_Y, partCentre, type PartId, type PartState } from '../parts'
import { makeMaterial, discAlpha, XRAY_OPACITY } from './materials'
import { bladeGeometry, crackPoints, shardGeometry, PROP_R } from './geometry'
import Internals from './Internals'
import glbUrl from './lmp_drone_parts.glb?url'

export const MODEL_URL = glbUrl

export interface ModelProps {
  states: Record<PartId, PartState>
  rpm?: number[]
  spinning: boolean
  xray: boolean
  animate: boolean
  selected?: PartId | null
  hovered?: PartId | null
  onHover?: (p: PartId | null) => void
  onPick?: (p: PartId) => void
  /** labels over broken parts (main view) / 破損ラベル */
  labels?: (p: PartId) => ReactNode
  interactive?: boolean
}

/** A per-instance material cache: one material per (part, paint), so each
 *  part glows on its own. / 部品ごとのマテリアル */
export type MatFor = (part: PartId, paint: string) => THREE.MeshPhysicalMaterial

const GLB_PAINT = (part: string, mat: string) =>
  part.startsWith('arm') && mat === 'M_black' ? 'carbon'
    : part === 'cam_fpv' && mat === 'M_ledB' ? 'lens'
      : part === 'shell' ? 'shell' : mat

// shared geometry (every view uses the same) / 共有ジオメトリ
let SHARED: { blade: THREE.BufferGeometry; chip: THREE.BufferGeometry; hub: THREE.BufferGeometry;
  cap: THREE.BufferGeometry; disc: THREE.BufferGeometry; shard: THREE.BufferGeometry[] } | null = null
function shared() {
  if (!SHARED) {
    SHARED = {
      blade: bladeGeometry(1), chip: bladeGeometry(0.62),
      hub: new THREE.CylinderGeometry(0.011, 0.012, 0.009, 24),
      cap: new THREE.SphereGeometry(0.0085, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2),
      disc: new THREE.CircleGeometry(PROP_R, 48).rotateX(-Math.PI / 2),
      shard: [1, 2, 3, 4, 5].map(shardGeometry),
    }
  }
  return SHARED
}

const noRaycast = () => null

function Rotor({ n, st, rpm, spinning, animate, matFor }: {
  n: number; st: PartState; rpm: number; spinning: boolean; animate: boolean; matFor: MatFor
}) {
  const g = shared()
  const spin = useRef<THREE.Group>(null)
  const [fx, fz] = CORNER[n]
  const dir = n === 1 || n === 4 ? 1 : -1            // diagonal pairs opposite / 対角で逆回転
  const broken = st.dmg?.level === 'broken'
  const chipped = st.dmg?.level === 'damaged'
  const turning = spinning && !broken && rpm > 50
  const disc = useMemo(() => new THREE.MeshBasicMaterial({
    color: '#aeb6c0', alphaMap: discAlpha(), transparent: true, opacity: 0, depthWrite: false,
  }), [])
  useEffect(() => () => disc.dispose(), [disc])
  useFrame((_, dt) => {
    if (!spin.current) return
    // visual speed, not the real 100 rev/s (a strobe) / 見た目の回転速度
    const w = turning && animate ? dir * Math.min((rpm / 60) * 0.06, 6) * Math.PI * 2 : 0
    spin.current.rotation.y += w * Math.min(dt, 0.05)
    disc.opacity = turning ? Math.min(rpm / 6000, 1) * (animate ? 0.14 : 0.08) : 0
  })
  const carbon = matFor(`rotor${n}`, 'carbon')
  const alu = matFor(`rotor${n}`, 'M_aluDark')
  return (
    <group position={[fx * HUB_R, HUB_Y + 0.0015, fz * HUB_R]}
      rotation={broken ? [fz * 0.22, 0, -fx * 0.18] : [0, 0, 0]}>
      <group ref={spin} rotation={[0, n * 0.7, 0]}>
        <mesh geometry={g.hub} material={alu} userData={{ part: `rotor${n}` }} />
        <mesh geometry={g.cap} material={alu} position={[0, 0.0045, 0]} userData={{ part: `rotor${n}` }} />
        {/* blade A: bent and drooping when the rotor broke / 破損時は曲がる */}
        <group rotation={broken ? [0.55, 0, -0.3] : [0, 0, 0]}>
          <mesh geometry={g.blade} material={carbon} userData={{ part: `rotor${n}` }} />
        </group>
        {/* blade B: chipped, or gone / 欠け・脱落 */}
        {!broken && (
          <mesh geometry={chipped ? g.chip : g.blade} material={carbon} rotation={[0, Math.PI, 0]}
            userData={{ part: `rotor${n}` }} />
        )}
      </group>
      <mesh geometry={g.disc} material={disc} position={[0, 0.003, 0]} raycast={noRaycast} />
      {broken && g.shard.slice(0, 3).map((s, i) => (
        <mesh key={i} geometry={s} material={carbon} raycast={noRaycast}
          position={[Math.cos(i * 2.1) * 0.02, -0.006, Math.sin(i * 2.1) * 0.02]} rotation={[i, i * 2, 0]} />
      ))}
    </group>
  )
}

/** A corner (arm, motor, ESC, rotor): pivots at the fuselage, droops when the arm broke. */
function Corner({ n, broken, damaged, children }: { n: number; broken: boolean; damaged: boolean; children: ReactNode }) {
  const [fx, fz] = CORNER[n]
  const P: [number, number, number] = [fx * 0.118, 0.25, fz * 0.118]
  const q = useMemo(() => {
    const ang = broken ? -0.3 : damaged ? -0.07 : 0
    return new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(-fz, 0, fx).normalize(), ang)
  }, [fx, fz, broken, damaged])
  return (
    <group position={P} quaternion={q}>
      <group position={[-P[0], -P[1], -P[2]]}>{children}</group>
    </group>
  )
}

function Cracks({ broken }: { broken: boolean }) {
  const lines = useMemo(() => [
    crackPoints(3, 0.02, -0.01, 0.13, 0.4, 0.3135),
    crackPoints(8, 0.02, -0.01, 0.09, 2.6, 0.3135),
    crackPoints(5, -0.04, 0.03, 0.08, -1.2, 0.3135),
  ], [])
  const hole = useMemo(() => {
    const s = new THREE.Shape()
    const pts = [[0, 0], [0.018, -0.006], [0.03, 0.004], [0.026, 0.017], [0.012, 0.022], [0.002, 0.014], [-0.009, 0.011]]
    s.moveTo(pts[0][0], pts[0][1])
    pts.slice(1).forEach(([x, y]) => s.lineTo(x, y))
    return new THREE.ShapeGeometry(s).rotateX(-Math.PI / 2)
  }, [])
  return (
    <group raycast={noRaycast}>
      {lines.map((p, i) => <Line key={i} points={p} color="#1d1f22" lineWidth={1.6} raycast={noRaycast} />)}
      {broken && (
        <mesh geometry={hole} position={[0.008, 0.3138, 0.0]} raycast={noRaycast}>
          <meshBasicMaterial color="#050607" side={THREE.DoubleSide} />
        </mesh>
      )}
    </group>
  )
}

export default function DroneModel(p: ModelProps) {
  const gltf = useGLTF(MODEL_URL, false)
  // the model's parts: (part, material name, geometry) / 部品・材質・形状
  const items = useMemo(() => {
    const out: { key: string; part: string; mat: string; geo: THREE.BufferGeometry }[] = []
    gltf.scene.traverse((o) => {
      const m = o as THREE.Mesh
      if (!m.isMesh) return
      const [part, mat] = m.name.split('__')
      if (part.startsWith('rotor')) return          // replaced by Rotor / 自前のローター
      out.push({ key: m.name, part, mat: mat ?? '', geo: m.geometry })
    })
    return out
  }, [gltf])

  // per-instance materials, one per (part, paint) / インスタンスごとのマテリアル
  const mats = useMemo(() => new Map<string, { part: string; m: THREE.MeshPhysicalMaterial; base: THREE.Color; baseI: number }>(), [])
  const matFor: MatFor = useMemo(() => (part, paint) => {
    const k = `${part}|${paint}`
    let e = mats.get(k)
    if (!e) {
      const m = makeMaterial(paint)
      e = { part, m, base: m.emissive.clone(), baseI: m.emissiveIntensity }
      mats.set(k, e)
    }
    return e.m
  }, [mats])
  useEffect(() => () => { mats.forEach((e) => e.m.dispose()); mats.clear() }, [mats])

  // X-ray: the shell, the core and the battery case go see-through / X 線表示
  useEffect(() => {
    mats.forEach((e, k) => {
      const o = XRAY_OPACITY[e.part]
      if (o === undefined || k.endsWith('|cells')) return
      e.m.transparent = p.xray
      e.m.opacity = p.xray ? o : 1
      e.m.depthWrite = !p.xray
      e.m.side = p.xray && e.part === 'shell' ? THREE.DoubleSide : THREE.FrontSide
      e.m.needsUpdate = true
    })
  }, [p.xray, mats, items])

  // glows: fault / broken pulse red, warning / damaged amber, hover and
  // selection accent; LEDs breathe / 状態の発光
  const stRef = useRef(p.states)
  stRef.current = p.states
  const hl = useRef({ sel: p.selected, hov: p.hovered, anim: p.animate })
  hl.current = { sel: p.selected, hov: p.hovered, anim: p.animate }
  const red = useMemo(() => new THREE.Color(C.danger), [])
  const amber = useMemo(() => new THREE.Color(C.warn), [])
  const green = useMemo(() => new THREE.Color(C.accent), [])
  useFrame(({ clock }) => {
    const t = clock.elapsedTime
    const { sel, hov, anim } = hl.current
    const pulse = anim ? 0.5 + 0.5 * Math.sin(t * 4.2) : 0.7
    mats.forEach((e) => {
      const st = stRef.current[e.part]
      const lv = st?.level
      const m = e.m
      if (lv === 'fail') { m.emissive.copy(red); m.emissiveIntensity = 0.35 + 0.75 * pulse }
      else if (lv === 'warn') { m.emissive.copy(amber); m.emissiveIntensity = 0.32 + (anim ? 0.08 * Math.sin(t * 2) : 0) }
      else if (e.part === sel) { m.emissive.copy(green); m.emissiveIntensity = 0.35 }
      else if (e.part === hov) { m.emissive.copy(green); m.emissiveIntensity = 0.22 }
      else {
        m.emissive.copy(e.base)
        m.emissiveIntensity = e.part === 'lights' && anim ? e.baseI * (0.7 + 0.3 * Math.sin(t * 2.3)) : e.baseI
      }
    })
  })

  const st = (k: string) => p.states[k] ?? {}
  const lvl = (k: string) => st(k).dmg?.level
  // selection shows as an accent glow (useFrame above); no outline pass
  // 選択はアクセント色の発光で示す
  const outline = (_part: string): ReactNode => null

  const glbMesh = (it: (typeof items)[number]) => {
    const hidden = it.part === 'hook' && it.mat === 'M_safety' && lvl('hook') === 'broken'
    if (hidden) return null
    const shellX = p.xray && it.part === 'shell'
    return (
      <mesh key={it.key} geometry={it.geo} material={matFor(it.part, GLB_PAINT(it.part, it.mat))}
        userData={{ part: it.part }} raycast={shellX ? noRaycast : undefined}
        renderOrder={XRAY_OPACITY[it.part] !== undefined && p.xray ? 2 : 0}>
        {outline(it.part)}
      </mesh>
    )
  }
  const cornerOf = (part: string) => { const m = /^(arm|motor|esc)(\d)$/.exec(part); return m ? Number(m[2]) : 0 }
  const gearBroken = lvl('gear') === 'broken'

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

  return (
    <group dispose={null} {...ev}>
      {items.filter((it) => !cornerOf(it.part) && it.part !== 'gear').map(glbMesh)}
      <group position={[0, 0.19, 0]} rotation={gearBroken ? [0.16, 0, 0.05] : [0, 0, 0]}>
        <group position={[0, -0.19, 0]}>{items.filter((it) => it.part === 'gear').map(glbMesh)}</group>
      </group>
      {[1, 2, 3, 4].map((n) => (
        <Corner key={n} n={n} broken={lvl(`arm${n}`) === 'broken'} damaged={lvl(`arm${n}`) === 'damaged'}>
          {items.filter((it) => cornerOf(it.part) === n).map(glbMesh)}
          <Rotor n={n} st={st(`rotor${n}`)} rpm={p.rpm?.[n - 1] ?? 0} spinning={p.spinning}
            animate={p.animate} matFor={matFor} />
          {lvl(`arm${n}`) === 'broken' && shared().shard.map((s, i) => (
            <mesh key={i} geometry={s} material={matFor(`arm${n}`, 'carbon')} raycast={noRaycast}
              position={[CORNER[n][0] * (0.122 + i * 0.004), 0.244 - i * 0.003, CORNER[n][1] * (0.118 - i * 0.003)]}
              rotation={[i, i * 1.7, i * 0.5]} />
          ))}
        </Corner>
      ))}
      <Internals matFor={matFor} outline={outline} />
      {(lvl('shell') === 'damaged' || lvl('shell') === 'broken') && <Cracks broken={lvl('shell') === 'broken'} />}
      {p.labels && Object.entries(p.states).filter(([, s]) => s.level === 'fail').map(([part]) => (
        <Html key={part} position={partCentre(part)} center zIndexRange={[20, 0]} style={{ pointerEvents: 'none' }}>
          {p.labels?.(part)}
        </Html>
      ))}
    </group>
  )
}

useGLTF.preload(MODEL_URL, false)
