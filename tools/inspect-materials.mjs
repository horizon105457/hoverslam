// Material + normal-orientation audit of the car GLB: per material, how many triangles have vertex normals
// that disagree with the winding, and how many face "inward" (towards the car centre).
import fs from 'fs';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { bakeCarMeshes } from '../src/car/carPrep.js';
const buf = fs.readFileSync(new URL('../public/assets/car-model.glb', import.meta.url));
const gltf = await new GLTFLoader().parseAsync(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '');
const meshes = bakeCarMeshes(gltf.scene);
const mats = new Map();
for (const m of meshes) {
  const k = m.material;
  if (!mats.has(k)) mats.set(k, { tris: 0, windFlip: 0, area: 0, meshes: new Set() });
  const s = mats.get(k);
  s.meshes.add(m.name.replace(/_\d+$/, ''));
  const P = m.pos, N = m.nrm, I = m.index;
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const ar = Math.hypot(nx, ny, nz) / 2; if (ar < 1e-12) continue;
    const vn = [N[a] + N[b] + N[c], N[a + 1] + N[b + 1] + N[c + 1], N[a + 2] + N[b + 2] + N[c + 2]];
    s.tris++; s.area += ar;
    if (nx * vn[0] + ny * vn[1] + nz * vn[2] < 0) s.windFlip += ar;
  }
}
for (const [mat, s] of mats) {
  const tex = ['map', 'normalMap', 'roughnessMap', 'metalnessMap', 'aoMap', 'emissiveMap'].filter(t => mat[t]).join(',');
  console.log(`${mat.name.padEnd(42)} ${mat.type.padEnd(22)} col=#${mat.color.getHexString()} met=${mat.metalness?.toFixed(2)} rough=${mat.roughness?.toFixed(2)} cc=${mat.clearcoat ?? '-'} tr=${mat.transmission ?? '-'} side=${mat.side} em=#${mat.emissive?.getHexString()}*${mat.emissiveIntensity} tex=[${tex}] tris=${s.tris} area=${s.area.toFixed(2)} windFlip=${(100 * s.windFlip / s.area).toFixed(1)}%`);
  console.log('    ', [...s.meshes].slice(0, 12).join(' '));
}
