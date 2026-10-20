// App context handed to every program: URL params, the Stage (renderer, scene, camera, post chain), DOM
// helpers and lazily loaded, cached shared assets. Programs take only what they need (a studio scene never
// pays for the viaduct or the soft-body lattice).
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Stage } from '../render/stage.js';
import { buildWorld } from '../scene/world.js';
import { bakeCarMeshes, buildCar } from '../car/carPrep.js';
import { unpackRecording } from '../softbody/recording.js';

export function createUI() {
  const $ = id => document.getElementById(id);
  return {
    $,
    /** container for program UI (header/panel/footer/...) */
    root: $('hud'),
    loadText(t) { $('load-text').textContent = t; },
    /** caption of the current shot: number + name, or free text; null clears it */
    setShot(html) { if (html !== this._shot) { $('shot').innerHTML = html ?? ''; this._shot = html; } },
    /** insert HTML into the program UI root and return the root */
    mount(html) { this.root.insertAdjacentHTML('beforeend', html); return this.root; },
    setTitle(t) { document.title = t; },
  };
}

export function createContext(params) {
  const ui = createUI();
  const stage = new Stage(ui.$('viewport'), { preserve: params.has('capture'), pixelRatio: params.has('pr') ? +params.get('pr') : Math.min(devicePixelRatio, 2) });
  if (params.has('w')) { stage.renderer.setPixelRatio(1); stage.resize(+params.get('w'), +params.get('h')); }
  const cache = new Map();
  const once = (key, fn) => { if (!cache.has(key)) cache.set(key, fn()); return cache.get(key); };
  const assets = {
    /** HDR sky + PMREM lighting environment (sets scene.background/environment); resolves to the env texture */
    sky: (url = '/assets/hdri/sky_4k.hdr') => once(`sky:${url}`, () => stage.loadSky(url)),
    /** the viaduct set (adds itself to the scene); `.ready` resolves when its textures are in */
    world: () => once('world', () => buildWorld(stage.scene, { renderer: stage.renderer })),
    gltf: (url = '/assets/car-model.glb') => once(`gltf:${url}`, () => new GLTFLoader().loadAsync(url)),
    /** car meshes baked into car-local space (no lattice) */
    carMeshes: () => once('carMeshes', async () => bakeCarMeshes((await assets.gltf()).scene)),
    /** soft-body lattice + render embedding (~1-2 s) */
    car: () => once('car', async () => { const m = await assets.carMeshes(); await new Promise(r => setTimeout(r, 20)); return buildCar(m); }),
    /** settled pose for CrashSim.setup */
    settled: (url = '/assets/settled.bin') => once(`settled:${url}`, async () => new Float32Array(await fetch(url).then(r => r.arrayBuffer()), 8)),
    /** a precomputed recording from public/sims, or null if there is none */
    sim: async name => {
      const res = await fetch(`/sims/${name}.sim`);
      // the dev server answers missing files with index.html (200, text/html)
      if (!res.ok || res.headers.get('content-type')?.includes('text/html')) return null;
      const buf = await new Response(res.body.pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
      return unpackRecording(buf);
    },
  };
  return { params, stage, ui, assets };
}
