// The big real-time 3D drone of the Drones page (lazy-loaded with three.js):
// studio lighting from local light formers (no HDR download), a soft contact
// shadow, slow auto-rotation that stops while you drag (damped orbit, zoom
// limits), rotors at the live motor RPM, the X-ray cutaway, hover tooltips
// (part, health, a reading), click = select, red labels on broken parts.
// The drone hovers while it flies, lies tilted when it crashed.
// ページ中央の 3D ドローン（three.js、遅延読み込み）
import { Suspense, useEffect, useRef, useState, type ComponentRef, type ReactNode } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { ContactShadows, Environment, Html, Lightformer, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import { C } from '@/lib/colors'
import { partCentre, type PartId, type PartState } from '../parts'
import DroneModel from './DroneModel'
import Thumb from './Thumb'

export { Thumb }

export type Pose = 'ground' | 'flying' | 'falling' | 'crashed'

export interface Drone3DProps {
  states: Record<PartId, PartState>
  rpm?: number[]
  spinning: boolean
  pose: Pose
  xray: boolean
  selected: PartId | null
  onPick: (p: PartId) => void
  tooltip: (p: PartId) => ReactNode
  label: (p: PartId) => ReactNode
  /** bump to reset the camera / カメラを戻す */
  resetKey: number
  ariaLabel: string
}

const reducedMotion = () => window.matchMedia?.('(prefers-reduced-motion: reduce)').matches ?? false

/** Hover bob in flight, a tilt when down. / 飛行中の上下動・墜落時の傾き */
function Posed({ pose, animate, children }: { pose: Pose; animate: boolean; children: ReactNode }) {
  const g = useRef<THREE.Group>(null)
  useFrame(({ clock }) => {
    const o = g.current
    if (!o) return
    const t = clock.elapsedTime
    const fly = pose === 'flying'
    const y = fly ? 0.07 + (animate ? 0.008 * Math.sin(t * 1.6) : 0) : pose === 'falling' ? 0.05 : 0
    o.position.y += (y - o.position.y) * 0.08
    // lies on its right side when crashed (as in CARLA, less steep) / 右側を下に
    const rx = pose === 'crashed' ? 0.2 : pose === 'falling' ? 0.12 + (animate ? 0.04 * Math.sin(t * 5) : 0)
      : fly && animate ? 0.015 * Math.sin(t * 1.1) : 0
    const rz = pose === 'crashed' ? 0.07 : fly && animate ? 0.012 * Math.sin(t * 0.9 + 1) : 0
    o.rotation.x += (rx - o.rotation.x) * 0.08
    o.rotation.z += (rz - o.rotation.z) * 0.08
  })
  return <group ref={g}>{children}</group>
}

/** Studio light: a soft box above, two strip lights, a faint accent rim. */
export function Studio({ accentRim = true }: { accentRim?: boolean }) {
  return (
    <>
      <ambientLight intensity={0.25} />
      <directionalLight position={[1.5, 3, 1.2]} intensity={1.5} />
      <directionalLight position={[-2, 1.2, -1.5]} intensity={0.5} />
      <Environment resolution={256} frames={1}>
        <Lightformer form="rect" intensity={2.6} position={[0, 4, 0]} rotation-x={Math.PI / 2} scale={[6, 6, 1]} />
        <Lightformer form="rect" intensity={1.4} position={[-4, 1.2, 1]} rotation-y={Math.PI / 2} scale={[8, 1.2, 1]} />
        <Lightformer form="rect" intensity={1.4} position={[4, 1.2, -1]} rotation-y={-Math.PI / 2} scale={[8, 1.2, 1]} />
        <Lightformer form="rect" intensity={0.8} position={[0, 0.6, 5]} scale={[10, 0.6, 1]} />
        {accentRim && <Lightformer form="ring" color={C.accent} intensity={0.9} position={[0, 2, -5]} scale={2.5} />}
      </Environment>
    </>
  )
}

/** Faint rings on the floor round the drone (our green). / 床のリング */
function FloorRings() {
  return (
    <group rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
      {[0.42, 0.62].map((r, i) => (
        <mesh key={r} raycast={() => null}>
          <ringGeometry args={[r, r + 0.0025, 96]} />
          <meshBasicMaterial color={C.accent} transparent opacity={i ? 0.12 : 0.22} depthWrite={false} />
        </mesh>
      ))}
    </group>
  )
}

export default function Drone3D(p: Drone3DProps) {
  const [hover, setHover] = useState<PartId | null>(null)
  const [auto, setAuto] = useState(!reducedMotion())
  const [visible, setVisible] = useState(true)
  const box = useRef<HTMLDivElement>(null)
  const ctl = useRef<ComponentRef<typeof OrbitControls>>(null)
  const idle = useRef<number>(0)
  const animate = !reducedMotion()

  // render every frame only while on screen / 画面内のときだけ毎フレーム描画
  useEffect(() => {
    const el = box.current
    if (!el) return
    const io = new IntersectionObserver(([e]) => setVisible(e.isIntersecting), { threshold: 0.01 })
    io.observe(el)
    return () => io.disconnect()
  }, [])
  useEffect(() => { if (p.resetKey) ctl.current?.reset() }, [p.resetKey])
  useEffect(() => () => window.clearTimeout(idle.current), [])

  return (
    <div ref={box} className="absolute inset-0" role="img" aria-label={p.ariaLabel}
      style={{ cursor: hover ? 'pointer' : 'grab' }}>
      <Canvas frameloop={visible ? 'always' : 'demand'} dpr={[1, 1.75]}
        camera={{ position: [1.0, 0.47, 1.06], fov: 30, near: 0.02, far: 20 }}
        gl={{ antialias: true, alpha: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.2 }}
        onPointerMissed={() => setHover(null)}>
        <Studio />
        <Suspense fallback={null}>
          <Posed pose={p.pose} animate={animate}>
            <DroneModel states={p.states} rpm={p.rpm} spinning={p.spinning} xray={p.xray} animate={animate}
              selected={p.selected} hovered={hover} onHover={setHover} onPick={p.onPick}
              labels={p.label} interactive />
            {hover && (
              <Html position={partCentre(hover)} zIndexRange={[25, 0]} style={{ pointerEvents: 'none' }}>
                <div className="translate-x-3 -translate-y-1/2">{p.tooltip(hover)}</div>
              </Html>
            )}
          </Posed>
          <ContactShadows position={[0, 0.0005, 0]} opacity={0.65} scale={1.5} blur={2.4} far={0.55}
            resolution={512} frames={animate ? Infinity : 1} />
        </Suspense>
        <FloorRings />
        <OrbitControls ref={ctl} makeDefault target={[0, 0.24, 0]} enableDamping dampingFactor={0.08}
          enablePan={false} minDistance={0.45} maxDistance={3.6} minPolarAngle={0.15} maxPolarAngle={1.85}
          autoRotate={auto && animate} autoRotateSpeed={0.7}
          onStart={() => { setAuto(false); window.clearTimeout(idle.current) }}
          onEnd={() => { window.clearTimeout(idle.current); idle.current = window.setTimeout(() => setAuto(true), 12000) }} />
      </Canvas>
    </div>
  )
}
