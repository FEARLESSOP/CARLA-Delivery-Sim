// Physical materials of the 3D mini pod, a studio product shot like the drone:
// the GLB's own PBR values (colours, printed textures) lifted into
// MeshPhysicalMaterial: glossy clear-coated white shell, matte rubber tyres,
// brushed aluminium hubs (the drone's streak roughness map), emissive LEDs,
// headlights and tail lights, dark glass for lenses and the lidar window.
// Textures come from the GLB or are generated (no downloads). These are the
// model's paint colours, not UI tokens.
// 3D ミニポッドの物理マテリアル（GLB の値を MeshPhysicalMaterial に）
import * as THREE from 'three'
import { brushedRoughness } from '@/views/drones/three/materials'

const PAINT = {
  lens: '#0b1830', glass: '#0d141c', cells: '#1f6fe0', pcb: '#0f3b2b', copper: '#c47a42', gold: '#d8b25a',
  black: '#16181b', aluDark: '#7d8590', alu: '#cfd4db', wire: '#1a1a1a', wireRed: '#c4231c', ledB: '#2d7bff',
  steel: '#9aa3ad', foam: '#e8e8e2', safety: '#ff6a10', tail: '#ff2a1f', head: '#fff6e0',
} as const

let treadN: THREE.Texture | null = null
/** Shallow tyre tread as a normal map (grooves across the tread). / タイヤの溝 */
export function treadNormal(): THREE.Texture {
  if (treadN) return treadN
  const W = 64, H = 16
  const c = document.createElement('canvas')
  c.width = W; c.height = H
  const ctx = c.getContext('2d')
  if (ctx) {
    const img = ctx.createImageData(W, H)
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const u = (x % 8) / 8
      const dx = u < 0.2 ? -0.6 : u > 0.8 ? 0.6 : 0
      const v = new THREE.Vector3(dx, 0, 1).normalize()
      const i = (y * W + x) * 4
      img.data[i] = (v.x * 0.5 + 0.5) * 255; img.data[i + 1] = (v.y * 0.5 + 0.5) * 255
      img.data[i + 2] = (v.z * 0.5 + 0.5) * 255; img.data[i + 3] = 255
    }
    ctx.putImageData(img, 0, 0)
  }
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.NoColorSpace
  treadN = t
  return t
}

/** A physical material from one of the GLB's materials, tuned per name. */
export function fromGlb(src: THREE.Material): THREE.Material {
  // unlit (printed labels exported as KHR_materials_unlit): keep as they are
  if (!(src instanceof THREE.MeshStandardMaterial)) return src.clone()
  const m = new THREE.MeshPhysicalMaterial({
    color: src.color.clone(), map: src.map, roughness: src.roughness, metalness: src.metalness,
    emissive: src.emissive.clone(), emissiveMap: src.emissiveMap, emissiveIntensity: src.emissiveIntensity,
    normalMap: src.normalMap, transparent: src.transparent, opacity: src.opacity, side: src.side,
    alphaTest: src.alphaTest,
  })
  m.name = src.name
  switch (src.name) {
    case 'M_pod_body':
      m.roughness = 0.28; m.metalness = 0; m.clearcoat = 1; m.clearcoatRoughness = 0.06
      m.sheen = 0.25; m.sheenColor.set('#ffffff'); break
    case 'M_pod_trim': case 'M_pod_accent':
      m.clearcoat = 0.6; m.clearcoatRoughness = 0.15; break
    case 'M_tire':
      m.roughness = 0.9; m.metalness = 0; m.normalMap = treadNormal(); m.normalScale.set(0.6, 0.6); break
    case 'M_alu':
      m.roughness = 0.3; m.metalness = 1; m.roughnessMap = brushedRoughness(); m.anisotropy = 0.7; break
    case 'M_aluDark':
      m.roughness = 0.36; m.metalness = 1; m.roughnessMap = brushedRoughness(); m.anisotropy = 0.6; break
    case 'M_steel':
      m.roughness = 0.24; m.metalness = 1; break
    case 'M_black':
      m.roughness = 0.6; m.metalness = 0.05; break
    case 'M_pod_shutter':
      m.roughness = 0.4; m.metalness = 0.6; m.roughnessMap = brushedRoughness(); break
    case 'M_ledG': case 'M_ledB':
      m.color.set('#050505'); m.emissive.set(src.name === 'M_ledG' ? '#21ff6a' : PAINT.ledB)
      m.emissiveIntensity = 2.2; m.toneMapped = false; break
    case 'M_pod_headlight':
      m.color.set('#101010'); m.emissive.set(PAINT.head); m.emissiveIntensity = 0.25; m.toneMapped = false
      m.roughness = 0.1; m.clearcoat = 1; break
    case 'M_pod_tail':
      m.color.set('#1a0505'); m.emissive.set(PAINT.tail); m.emissiveIntensity = 0.5; m.toneMapped = false; break
    case 'M_hv':
      m.roughness = 0.45; m.metalness = 0; break
    case 'M_pod_flag':
      m.roughness = 0.8; m.side = THREE.DoubleSide; m.emissive.set(PAINT.safety); m.emissiveIntensity = 0.15; break
    case 'M_pod_contact':
      m.color.set(PAINT.gold); m.roughness = 0.25; m.metalness = 1; break
    default:
      break
  }
  return m
}

/** Our own paints for what the GLB has no geometry for. / 追加部品の材質 */
export function paint(name: string): THREE.MeshPhysicalMaterial {
  const m = new THREE.MeshPhysicalMaterial({ color: PAINT.black, roughness: 0.6, metalness: 0.05 })
  m.name = name
  switch (name) {
    case 'lens':
      m.color.set(PAINT.lens); m.roughness = 0.04; m.metalness = 0.2; m.clearcoat = 1; m.clearcoatRoughness = 0.02
      m.iridescence = 0.8; m.iridescenceIOR = 1.6; m.ior = 1.7; break
    case 'glass':
      m.color.set(PAINT.glass); m.roughness = 0.08; m.metalness = 0.3; m.clearcoat = 1; m.clearcoatRoughness = 0.03; break
    case 'ledB':
      m.color.set('#050505'); m.emissive.set(PAINT.ledB); m.emissiveIntensity = 2.2; m.toneMapped = false; break
    case 'cells':
      m.color.set(PAINT.cells); m.roughness = 0.22; m.metalness = 0.1; m.clearcoat = 1; m.clearcoatRoughness = 0.05; break
    case 'pcb':
      m.color.set(PAINT.pcb); m.roughness = 0.45; m.metalness = 0.1; m.clearcoat = 0.6; break
    case 'copper':
      m.color.set(PAINT.copper); m.roughness = 0.3; m.metalness = 1; break
    case 'gold':
      m.color.set(PAINT.gold); m.roughness = 0.25; m.metalness = 1; break
    case 'alu':
      m.color.set(PAINT.alu); m.roughness = 0.3; m.metalness = 1; m.roughnessMap = brushedRoughness(); m.anisotropy = 0.7; break
    case 'aluDark':
      m.color.set(PAINT.aluDark); m.roughness = 0.34; m.metalness = 1; m.roughnessMap = brushedRoughness(); break
    case 'steel':
      m.color.set(PAINT.steel); m.roughness = 0.24; m.metalness = 1; break
    case 'wire':
      m.color.set(PAINT.wire); m.roughness = 0.5; break
    case 'wireRed':
      m.color.set(PAINT.wireRed); m.roughness = 0.45; break
    case 'foam':
      m.color.set(PAINT.foam); m.roughness = 0.95; break
    case 'safety':
      m.color.set(PAINT.safety); m.roughness = 0.42; m.clearcoat = 0.5; break
    case 'black': default:
      break
  }
  return m
}

/** GLB materials that go see-through in X-ray mode, and how far. / X 線で透ける材質 */
export const XRAY_MATS: Record<string, number> = {
  M_pod_body: 0.12, M_pod_trim: 0.16, M_pod_accent: 0.2, M_black: 0.22, M_pod_shutter: 0.18,
  M_Printed_Pod_1: 0.2, M_Printed_Pod_2: 0.2, M_Printed_Pod_3: 0.2, M_Printed_Pod_4: 0.25,
  M_aluDark: 0.35, M_steel: 0.5, M_pod_belt: 0.35,
}
