// Procedural geometry the GLB does not have in a nice enough form: a twisted,
// tapered propeller blade (the model's rotors are flat bars), and the jagged
// crack lines / shards of a damaged airframe.
// GLB に無い形状：ねじれたプロペラ翼、損傷のひび・破片
import * as THREE from 'three'

export const PROP_R = 0.15          // rotor radius, 0.17 x 0.88 (lmp_parts.json) / ローター半径

/** One blade along +x from the hub, chord along z, thickness along y,
 *  twisted from 20 deg at the root to 7 deg at the tip. ``tip`` < 1 = a
 *  chipped blade. / 1 枚の翼（根元 20°→先端 7° のねじれ） */
export function bladeGeometry(tip = 1): THREE.BufferGeometry {
  const r0 = 0.014, r1 = PROP_R * tip
  const chord = (u: number) => 0.012 + 0.016 * Math.sin(Math.PI * Math.min(1, u * 1.25)) * (1 - 0.55 * u)
  const s = new THREE.Shape()
  const n = 18
  s.moveTo(r0, -0.006)
  for (let i = 0; i <= n; i++) {
    const u = i / n
    s.lineTo(r0 + (r1 - r0) * u, -chord(u) * 0.38)
  }
  // rounded tip / 丸い翼端
  s.quadraticCurveTo(r1 + 0.006, 0, r1, chord(1) * 0.62)
  for (let i = n; i >= 0; i--) {
    const u = i / n
    s.lineTo(r0 + (r1 - r0) * u, chord(u) * 0.62)
  }
  s.lineTo(r0, 0.006)
  const g = new THREE.ExtrudeGeometry(s, { depth: 0.0018, bevelEnabled: true, bevelThickness: 0.0007,
    bevelSize: 0.0007, bevelSegments: 2, curveSegments: 6 })
  g.translate(0, 0, -0.0009)
  g.rotateX(-Math.PI / 2)                // shape plane -> horizontal, thickness up / 水平に
  const p = g.attributes.position as THREE.BufferAttribute
  const uv = g.attributes.uv as THREE.BufferAttribute
  const v = new THREE.Vector3()
  for (let i = 0; i < p.count; i++) {
    v.fromBufferAttribute(p, i)
    const u = Math.max(0, (v.x - r0) / (PROP_R - r0))
    const a = THREE.MathUtils.degToRad(20 - 13 * u)          // twist about the blade axis / ねじれ
    const y = v.y * Math.cos(a) - v.z * Math.sin(a)
    const z = v.y * Math.sin(a) + v.z * Math.cos(a)
    p.setXYZ(i, v.x, y + 0.004 * u * u, z)                     // slight upward cone / 上反角
    uv.setXY(i, v.x * 38, v.z * 38)
  }
  g.computeVertexNormals()
  return g
}

/** A jagged crack across a surface at height y: a polyline that wanders. */
export function crackPoints(seed: number, x0: number, z0: number, len: number, dir: number, y: number): THREE.Vector3[] {
  let s = seed * 9301 + 49297
  const rnd = () => ((s = (s * 9301 + 49297) % 233280) / 233280)
  const pts: THREE.Vector3[] = []
  let x = x0, z = z0, a = dir
  const steps = 9
  for (let i = 0; i <= steps; i++) {
    pts.push(new THREE.Vector3(x, y, z))
    a += (rnd() - 0.5) * 1.1
    const d = len / steps
    x += Math.cos(a) * d
    z += Math.sin(a) * d
  }
  return pts
}

/** Small irregular shards around a fracture (a broken arm root). / 破断部の破片 */
export function shardGeometry(seed: number): THREE.BufferGeometry {
  const g = new THREE.TetrahedronGeometry(0.008, 0)
  let s = seed * 7 + 3
  const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647)
  const p = g.attributes.position as THREE.BufferAttribute
  for (let i = 0; i < p.count; i++) p.setXYZ(i, p.getX(i) * (0.6 + rnd()), p.getY(i) * (0.4 + rnd()), p.getZ(i) * (0.6 + rnd()))
  g.computeVertexNormals()
  return g
}
