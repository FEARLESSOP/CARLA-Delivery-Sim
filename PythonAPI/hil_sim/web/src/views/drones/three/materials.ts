// Physical materials of the 3D drone, made to look like a studio product shot:
// glossy white clear-coated shell, carbon rotors and arms (a procedural twill
// normal map), brushed aluminium motor bells (anisotropic, a streak roughness
// map), matte black frame, steel, emissive navigation LEDs, glass lenses.
// Everything is generated here (small canvas textures): nothing is downloaded.
// These are the model's own paint colours, not UI tokens.
// 3D ドローンの物理マテリアル（テクスチャはすべてキャンバスで生成）
import * as THREE from 'three'

const PAINT = {
  shell: '#eef1f4', white: '#f4f6f8', core: '#2a2f36', black: '#16181b', carbon: '#1b1d21',
  steel: '#9aa3ad', alu: '#cfd4db', aluDark: '#7d8590', safety: '#ff6a10', hazard: '#ffa21a',
  hv: '#ff4a12', ledR: '#ff2a1f', ledG: '#21ff6a', ledB: '#2d7bff', lens: '#0b1830', pcb: '#0f3b2b',
  copper: '#c47a42', gold: '#d8b25a', cellBlue: '#1f6fe0', foam: '#e8e8e2', wire: '#1a1a1a', wireRed: '#c4231c',
} as const
export type Paint = keyof typeof PAINT

let carbonN: THREE.Texture | null = null
let brushedR: THREE.Texture | null = null

/** Twill-weave carbon fibre as a normal map (tangent space). / カーボン綾織の法線マップ */
export function carbonNormal(): THREE.Texture {
  if (carbonN) return carbonN
  const N = 128
  const h = new Float32Array(N * N)
  const cell = 16
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    // 2/2 twill: tows alternate direction on a diagonal staircase / 2/2 綾織
    const cx = Math.floor(x / cell), cy = Math.floor(y / cell)
    const warp = ((cx + cy) >> 1) % 2 === 0
    const u = ((warp ? x : y) % cell) / cell
    const bump = Math.sin(Math.PI * u)                     // a tow's round profile / 繊維束の断面
    const fibre = 0.12 * Math.sin((warp ? y : x) * 2.1)    // fine fibre lines / 細かい繊維
    h[y * N + x] = (warp ? 1 : 0.7) * bump + fibre
  }
  const c = document.createElement('canvas')
  c.width = c.height = N
  const ctx = c.getContext('2d')
  if (ctx) {
    const img = ctx.createImageData(N, N)
    for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
      const dx = h[y * N + ((x + 1) % N)] - h[y * N + ((x + N - 1) % N)]
      const dy = h[((y + 1) % N) * N + x] - h[((y + N - 1) % N) * N + x]
      const v = new THREE.Vector3(-dx * 1.4, -dy * 1.4, 1).normalize()
      const i = (y * N + x) * 4
      img.data[i] = (v.x * 0.5 + 0.5) * 255
      img.data[i + 1] = (v.y * 0.5 + 0.5) * 255
      img.data[i + 2] = (v.z * 0.5 + 0.5) * 255
      img.data[i + 3] = 255
    }
    ctx.putImageData(img, 0, 0)
  }
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.NoColorSpace
  t.anisotropy = 4
  carbonN = t
  return t
}

/** Fine brushing streaks as a roughness map. / ヘアライン加工の粗さマップ */
export function brushedRoughness(): THREE.Texture {
  if (brushedR) return brushedR
  const W = 256, H = 64
  const c = document.createElement('canvas')
  c.width = W
  c.height = H
  const ctx = c.getContext('2d')
  if (ctx) {
    ctx.fillStyle = 'rgb(90,90,90)'
    ctx.fillRect(0, 0, W, H)
    // deterministic streaks (no Math.random: same look every load) / 決定的な筋
    let seed = 7
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647)
    for (let i = 0; i < 700; i++) {
      const y = rnd() * H, g = 60 + rnd() * 70
      ctx.strokeStyle = `rgba(${g},${g},${g},0.55)`
      ctx.lineWidth = 0.6 + rnd()
      ctx.beginPath()
      ctx.moveTo(0, y)
      ctx.lineTo(W, y + (rnd() - 0.5) * 1.5)
      ctx.stroke()
    }
  }
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.NoColorSpace
  brushedR = t
  return t
}

/** A soft radial alpha for the blur disc of a spinning rotor. / 回転円盤のぼかし */
let discA: THREE.Texture | null = null
export function discAlpha(): THREE.Texture {
  if (discA) return discA
  const S = 128
  const c = document.createElement('canvas')
  c.width = c.height = S
  const ctx = c.getContext('2d')
  if (ctx) {
    const g = ctx.createRadialGradient(S / 2, S / 2, S * 0.08, S / 2, S / 2, S / 2)
    g.addColorStop(0, 'rgba(255,255,255,0)')
    g.addColorStop(0.25, 'rgba(255,255,255,0.55)')
    g.addColorStop(0.85, 'rgba(255,255,255,0.35)')
    g.addColorStop(1, 'rgba(255,255,255,0)')
    ctx.fillStyle = g
    ctx.fillRect(0, 0, S, S)
  }
  discA = new THREE.CanvasTexture(c)
  return discA
}

export interface MatOpts { xray?: boolean }

/** The material for one of the model's material names (or our own paints). / 材質 */
export function makeMaterial(name: string): THREE.MeshPhysicalMaterial {
  const m = new THREE.MeshPhysicalMaterial({ color: PAINT.black, roughness: 0.6, metalness: 0.05 })
  m.name = name
  switch (name) {
    case 'M_droneShell':
    case 'shell':
      m.color.set(PAINT.shell); m.roughness = 0.3; m.metalness = 0; m.clearcoat = 1; m.clearcoatRoughness = 0.06
      m.sheen = 0.2; m.sheenColor.set('#ffffff'); break
    case 'M_white':
      m.color.set(PAINT.white); m.roughness = 0.35; m.metalness = 0; m.clearcoat = 0.8; m.clearcoatRoughness = 0.1; break
    case 'M_droneBody':
      m.color.set(PAINT.core); m.roughness = 0.55; m.metalness = 0.15; break
    case 'M_black':
      m.color.set(PAINT.black); m.roughness = 0.62; m.metalness = 0.04; break
    case 'carbon':
      m.color.set(PAINT.carbon); m.roughness = 0.34; m.metalness = 0.1; m.clearcoat = 0.7; m.clearcoatRoughness = 0.18
      m.normalMap = carbonNormal(); m.normalScale.set(0.45, 0.45); break
    case 'M_steel':
      m.color.set(PAINT.steel); m.roughness = 0.22; m.metalness = 1; break
    case 'M_alu':
      m.color.set(PAINT.alu); m.roughness = 0.3; m.metalness = 1
      m.roughnessMap = brushedRoughness(); m.anisotropy = 0.7; break
    case 'M_aluDark':
      m.color.set(PAINT.aluDark); m.roughness = 0.34; m.metalness = 1
      m.roughnessMap = brushedRoughness(); m.anisotropy = 0.7; break
    case 'M_safety':
      m.color.set(PAINT.safety); m.roughness = 0.42; m.metalness = 0; m.clearcoat = 0.5
      m.emissive.set(PAINT.safety); m.emissiveIntensity = 0.12; break
    case 'M_hazard':
      m.color.set(PAINT.hazard); m.roughness = 0.42; m.metalness = 0; m.clearcoat = 0.5
      m.emissive.set(PAINT.hazard); m.emissiveIntensity = 0.12; break
    case 'M_hv':
      m.color.set(PAINT.hv); m.roughness = 0.45; m.metalness = 0; break
    case 'M_ledR': case 'M_ledG': case 'M_ledB': {
      const c = name === 'M_ledR' ? PAINT.ledR : name === 'M_ledG' ? PAINT.ledG : PAINT.ledB
      m.color.set('#050505'); m.roughness = 0.25; m.emissive.set(c); m.emissiveIntensity = 2.4
      m.toneMapped = false; break
    }
    case 'lens':
      m.color.set(PAINT.lens); m.roughness = 0.04; m.metalness = 0.2; m.clearcoat = 1; m.clearcoatRoughness = 0.02
      m.iridescence = 0.8; m.iridescenceIOR = 1.6; m.ior = 1.7; m.specularIntensity = 1; break
    case 'pcb':
      m.color.set(PAINT.pcb); m.roughness = 0.45; m.metalness = 0.1; m.clearcoat = 0.6; break
    case 'copper':
      m.color.set(PAINT.copper); m.roughness = 0.3; m.metalness = 1; break
    case 'gold':
      m.color.set(PAINT.gold); m.roughness = 0.25; m.metalness = 1; break
    case 'cells':
      m.color.set(PAINT.cellBlue); m.roughness = 0.22; m.metalness = 0.1; m.clearcoat = 1; m.clearcoatRoughness = 0.05; break
    case 'foam':
      m.color.set(PAINT.foam); m.roughness = 0.95; break
    case 'wire':
      m.color.set(PAINT.wire); m.roughness = 0.5; break
    case 'wireRed':
      m.color.set(PAINT.wireRed); m.roughness = 0.45; break
    default:
      break
  }
  return m
}

/** Materials whose part goes see-through in X-ray mode, and how far. / X 線で透ける材質 */
export const XRAY_OPACITY: Record<string, number> = {
  shell: 0.18, body: 0.2, battery: 0.3,
}

/** The model's LED colours (health glows use the UI's own C.* tokens). / LED 色 */
export const LED = { red: PAINT.ledR, green: PAINT.ledG, blue: PAINT.ledB }
