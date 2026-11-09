// The big real-time 3D mini pod of the Mini Pods page (lazy-loaded with
// three.js, its own chunk): studio lighting from local light formers (no HDR
// download), a soft contact shadow, slow auto-rotation that stops while you
// drag (damped orbit, zoom limits), the moving parts driven by the live
// state, the X-ray cutaway, hover tooltips, click = select, red labels on
// broken parts. A crashed pod lies tipped over; a stuck one leans.
// ページ中央の 3D ミニポッド（three.js、遅延読み込み）
import { Suspense, useEffect, useRef, useState, type ComponentRef, type ReactNode } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { ContactShadows, Environment, Html, Lightformer, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import { C } from '@/lib/colors'
import { partCentre, type PartId, type PartState } from '../parts'
import PodModel from './PodModel'
import Thumb from './Thumb'

export { Thumb }

export type Pose = 'ground' | 'crashed' | 'stuck'

export interface Pod3DProps {
  states: Record<PartId, PartState>
  speed: number
  lid: number
  shutter: number
  flag: number
  lights: boolean
  scanning: boolean
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

/** tipped over when crashed, leaning when stuck / 転倒・傾き */
function Posed({ pose, children }: { pose: Pose; children: ReactNode }) {
  const g = useRef<THREE.Group>(null)
  useFrame(() => {
    const o = g.current
    if (!o) return
    const rx = pose === 'crashed' ? 0.42 : pose === 'stuck' ? 0.05 : 0
    const rz = pose === 'crashed' ? 0.06 : pose === 'stuck' ? -0.06 : 0
    o.rotation.x += (rx - o.rotation.x) * 0.08
    o.rotation.z += (rz - o.rotation.z) * 0.08
    const y = pose === 'crashed' ? 0.07 : 0
    o.position.y += (y - o.position.y) * 0.08
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

function FloorRings() {
  return (
    <group rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
      {[0.62, 0.9].map((r, i) => (
        <mesh key={r} raycast={() => null}>
          <ringGeometry args={[r, r + 0.003, 96]} />
          <meshBasicMaterial color={C.accent} transparent opacity={i ? 0.12 : 0.22} depthWrite={false} />
        </mesh>
      ))}
    </group>
  )
}

export default function Pod3D(p: Pod3DProps) {
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
        camera={{ position: [1.75, 1.05, 1.8], fov: 30, near: 0.02, far: 30 }}
        gl={{ antialias: true, alpha: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.15 }}
        onPointerMissed={() => setHover(null)}>
        <Studio />
        <Suspense fallback={null}>
          <Posed pose={p.pose}>
            <PodModel states={p.states} speed={p.speed} lid={p.lid} shutter={p.shutter} flag={p.flag}
              lights={p.lights} scanning={p.scanning} xray={p.xray} animate={animate}
              selected={p.selected} hovered={hover} onHover={setHover} onPick={p.onPick}
              labels={p.label} interactive />
            {hover && (
              <Html position={partCentre(hover)} zIndexRange={[25, 0]} style={{ pointerEvents: 'none' }}>
                <div className="translate-x-3 -translate-y-1/2">{p.tooltip(hover)}</div>
              </Html>
            )}
          </Posed>
          <ContactShadows position={[0, 0.0005, 0]} opacity={0.7} scale={2.2} blur={2.4} far={0.8}
            resolution={512} frames={animate ? Infinity : 1} />
        </Suspense>
        <FloorRings />
        <OrbitControls ref={ctl} makeDefault target={[0, 0.48, 0]} enableDamping dampingFactor={0.08}
          enablePan={false} minDistance={0.7} maxDistance={5} minPolarAngle={0.15} maxPolarAngle={1.75}
          autoRotate={auto && animate} autoRotateSpeed={0.7}
          onStart={() => { setAuto(false); window.clearTimeout(idle.current) }}
          onEnd={() => { window.clearTimeout(idle.current); idle.current = window.setTimeout(() => setAuto(true), 12000) }} />
      </Canvas>
    </div>
  )
}
