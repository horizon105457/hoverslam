// Node helper: load the GLB without a browser and build the destructible lattice.
import fs from 'fs';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { bakeCarMeshes, buildCar } from '../../src/car/carPrep.js';
import { repo } from './paths.mjs';
export async function loadCarMeshes() {
  const buf = fs.readFileSync(repo('public/assets/car-model.glb'));
  const gltf = await new GLTFLoader().parseAsync(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '');
  return bakeCarMeshes(gltf.scene);
}
export async function loadCarNode(opts) {
  return buildCar(await loadCarMeshes(), opts);
}
