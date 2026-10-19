// Node helper: load the GLB without a browser and build the destructible lattice.
import fs from 'fs';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { bakeCarMeshes, buildCar } from '../src/car/carPrep.js';
export async function loadCarNode(opts) {
  const buf = fs.readFileSync(new URL('../public/assets/car-model.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '');
  const meshes = bakeCarMeshes(gltf.scene);
  return buildCar(meshes, opts);
}
