// Converts the render-only car GLB into a destructible soft body:
//  * every physical piece (body shell, doors, hood, wheels, seats, glass ...) is voxelised on a shared grid,
//  * voxels become jittered Kuhn tetrahedra whose 19 edges are plastic beams,
//  * pieces are tied together by breakable joints (hinges, latches, bolts, suspension, bonding),
//  * every render vertex is embedded (barycentric) in one tetrahedron so the GPU can follow the lattice.
// Pure data in / typed arrays out, so it runs in the browser and in Node test tools alike.
import { Matrix4, Vector3 } from 'three';

// STRUCT/SPOKE: rigid-plastic lattice beams; `fy` holds the yield strain, `tear` the rupture plastic strain.
// WELD/JOINT/SUSP: compliant connectors that snap once stretched by more than `tear` metres.
// SPRING: suspension coil-over, soft within +-tear metres of travel, rigid bump/rebound stop beyond (snaps beyond
// `fy` newtons); with a levelled suspension (CrashSim `level`) preloaded and with the SPRING.bump / rebound stops.
export const BEAM = { STRUCT: 0, WELD: 1, JOINT: 2, SPOKE: 3, SUSP: 4, SPRING: 5 };
// Coil-overs, two per wheel (one per axle node); travel and stops measured along the spring from its modelled
// length. Unloaded springs let the car sag 5-11 cm, but the GLB is modelled at ride height with only 29 mm (rear)
// / 34 mm (front) between tyres and arch lips: at rest the tyres sit inside the arches. The levelled suspension
// (opt-in, CrashSim `level`) preloads the springs to hold that height under the static load and stops the bump
// about where a tyre meets its arch (springs lean 30-40 deg: wheel travel ~ spring / 0.82), see levelSprings.
export const SPRING = { k: 14e3, travel: 0.11, bump: 0.03, rebound: 0.02, breakF: 20e3 };
const JOINT_GIVE = 0.007; // m of (low-passed) stretch at which a bolt / hinge / clip lets go

// Piece table. mass in kg, ey = yield strain of the lattice beams, tear = plastic strain at rupture,
// brittle pieces split into `chunks` fragments glued by welds that snap at `weld` N. Joints: `box` = region of the
// piece's bounding box (u0 u1 v0 v1 w0 w1 over x y z), `k` anchors sharing `F` N, `parents` overrides the piece's.
export const PIECES = {
  body:       { mass: 330, ey: 0.0045, tear: 0.55, color: 'paint' },
  door_L:     { mass: 18, ey: 0.0035, tear: 0.5, parents: ['body'], color: 'paint',
                joints: [{ box: [0, 1, .55, .8, .8, 1], F: 11e3, k: 1 }, { box: [0, 1, .1, .35, .8, 1], F: 11e3, k: 1 }, { box: [0, 1, .3, .6, 0, .15], F: 4.5e3, k: 1, latch: 0.02 }] },
  door_R:     { mass: 18, ey: 0.0035, tear: 0.5, parents: ['body'], color: 'paint',
                joints: [{ box: [0, 1, .55, .8, .8, 1], F: 11e3, k: 1 }, { box: [0, 1, .1, .35, .8, 1], F: 11e3, k: 1 }, { box: [0, 1, .3, .6, 0, .15], F: 4.5e3, k: 1, latch: 0.02 }] },
  // the front latch needs bumper_front: no body node lies under the hood's front edge, so it used to anchor at the
  // cowl and the front 40 cm of the hood flapped +-2 cm at ~5 Hz from the launch kick on (flicker in every film)
  hood:       { mass: 7, ey: 0.003, tear: 0.45, parents: ['body', 'windshield', 'bumper_front'], color: 'paint',
                joints: [{ box: [.1, .3, 0, 1, 0, .25], F: 2.6e3, k: 1 }, { box: [.7, .9, 0, 1, 0, .25], F: 2.6e3, k: 1 }, { box: [.4, .6, 0, 1, .7, 1], F: 1.8e3, k: 1, latch: 0.012 }] },
  hatch:      { mass: 13, ey: 0.0035, tear: 0.45, parents: ['body'], color: 'paint',
                joints: [{ box: [0, .3, .5, 1, 0, 1], F: 9e3, k: 1 }, { box: [.7, 1, .5, 1, 0, 1], F: 9e3, k: 1 }, { box: [.4, .6, 0, .2, 0, 1], F: 3.5e3, k: 1, latch: 0.018 }] },
  // the headlamp belongs to the fender in the GLB: its 35 cm block hung 40-70 cm off the nearest fender anchors and
  // swung +-7 mm sideways against the bumper skin that wraps it, pushing its black housing (flush behind the skin)
  // through the paint every other frame after the landing. The lamp joint bolts the block to that bumper.
  fender_front_L: { mass: 4, ey: 0.003, tear: 0.4, parents: ['body'], color: 'paint', joints: [{ F: 8e3, k: 6 }, { box: [0, 1, .6, .9, .7, 1], F: 3e3, k: 3, parents: ['bumper_front'] }] },
  fender_front_R: { mass: 4, ey: 0.003, tear: 0.4, parents: ['body'], color: 'paint', joints: [{ F: 8e3, k: 6 }, { box: [0, 1, .6, .9, .7, 1], F: 3e3, k: 3, parents: ['bumper_front'] }] },
  bumper_front: { mass: 6, ey: 0.006, tear: 0.18, chunks: 7, weld: 1500, parents: ['body'], color: 'plastic', joints: [{ F: 6e3, k: 10 }] },
  bumper_rear:  { mass: 5, ey: 0.006, tear: 0.18, chunks: 6, weld: 1500, parents: ['body'], color: 'plastic', joints: [{ F: 5e3, k: 8 }] },
  mirror_L:   { mass: 1.2, ey: 0.004, tear: 0.4, parents: ['door_L', 'body'], color: 'plastic', joints: [{ box: [0, 1, 0, .6, 0, 1], F: 1.6e3, k: 2 }] },
  mirror_R:   { mass: 1.2, ey: 0.004, tear: 0.4, parents: ['door_R', 'body'], color: 'plastic', joints: [{ box: [0, 1, 0, .6, 0, 1], F: 1.6e3, k: 2 }] },
  seat_D:     { mass: 15, ey: 0.006, tear: 0.6, parents: ['body'], color: 'fabric', joints: [{ box: [0, 1, 0, .12, 0, 1], F: 7e3, k: 4 }] },
  seat_P:     { mass: 15, ey: 0.006, tear: 0.6, parents: ['body'], color: 'fabric', joints: [{ box: [0, 1, 0, .12, 0, 1], F: 7e3, k: 4 }] },
  windshield: { mass: 11, ey: 0.004, tear: 0.3, glass: 'laminated', parents: ['body'], joints: [{ F: 25e3, k: 16 }] },
  glass_dL:   { mass: 3, ey: 1, tear: 9, glass: 'tempered', parents: ['door_L'], joints: [{ F: 8e3, k: 8 }] },
  glass_dR:   { mass: 3, ey: 1, tear: 9, glass: 'tempered', parents: ['door_R'], joints: [{ F: 8e3, k: 8 }] },
  glass_qL:   { mass: 2, ey: 1, tear: 9, glass: 'tempered', parents: ['body'], joints: [{ F: 10e3, k: 8 }] },
  glass_qR:   { mass: 2, ey: 1, tear: 9, glass: 'tempered', parents: ['body'], joints: [{ F: 10e3, k: 8 }] },
  glass_rear: { mass: 4, ey: 1, tear: 9, glass: 'tempered', parents: ['hatch'], joints: [{ F: 12e3, k: 10 }] },
  wheel_FL:   { mass: 14, wheel: true }, wheel_FR: { mass: 14, wheel: true },
  wheel_RL:   { mass: 14, wheel: true }, wheel_RR: { mass: 14, wheel: true },
};

// Small brittle optics hosted by a piece: they vanish and burst into shards when the host distorts or hits hard.
export const LENSES = [
  { name: 'lamp_L', re: /^Headlamp_flush_clear_cover_L|^Headlamp_small_projector_lens_L/, shard: 0, count: 140 },
  { name: 'lamp_R', re: /^Headlamp_flush_clear_cover_R|^Headlamp_small_projector_lens_R/, shard: 0, count: 140 },
  { name: 'tail_L', re: /^Sculpted_red_lens_L|^Tail_lamp_perimeter_optic_L|^Rear_horizontal_clear_optic_L/, shard: 1, count: 110 },
  { name: 'tail_R', re: /^Sculpted_red_lens_R|^Tail_lamp_perimeter_optic_R|^Rear_horizontal_clear_optic_R/, shard: 1, count: 110 },
  { name: 'stop', re: /^High_level_stop_light/, shard: 1, count: 30 },
  { name: 'mirror_gL', re: /^Mirror_glass_inset_L/, shard: 5, count: 40 },
  { name: 'mirror_gR', re: /^Mirror_glass_inset_R/, shard: 5, count: 40 },
  { name: 'repeater_L', re: /^Fender_side_repeater_L/, shard: 2, count: 14 },
  { name: 'repeater_R', re: /^Fender_side_repeater_R/, shard: 2, count: 14 },
  { name: 'fog', re: /^Front_lower_lamp_reflector|^Front_lower_optical_flute/, shard: 0, count: 50 },
];
// Shard looks: 0 clear glass, 1 red lens, 2 amber, 3 paint chip, 4 black plastic, 5 mirror, 6 tempered glass
export const SHARD_KINDS = 7;

export function partIdOf(obj) {
  for (let n = obj; n; n = n.parent) if (n.userData?.part_id) return n.userData.part_id;
  return null;
}

/** Physics piece a render mesh belongs to. */
export function pieceOf(mesh) {
  const pid = partIdOf(mesh), n = mesh.name;
  switch (pid) {
    case 'body_structure': case 'roof': return 'body';
    case 'quarter_rear_L': case 'quarter_rear_R': return /^Quarter_glass/.test(n) ? 'glass_q' + pid.slice(-1) : 'body';
    case 'interior': {
      const m = n.match(/^(driver|passenger)_/);
      return m ? (m[1] === 'driver' ? 'seat_D' : 'seat_P') : 'body';
    }
    case 'windshield': return /^Windshield_(glass|ceramic)/.test(n) ? 'windshield' : 'body';
    case 'door_L': case 'door_R': return /^Door_glass/.test(n) ? 'glass_d' + pid.slice(-1) : pid;
    case 'hatch': return /^Rear_glass|^Rear_window_ceramic/.test(n) ? 'glass_rear' : 'hatch';
    default: return pid;
  }
}

// ---------------------------------------------------------------------------------------------
// Small helpers
const hash3 = (i, j, k) => {
  let h = (i * 73856093) ^ (j * 19349663) ^ (k * 83492791);
  h = Math.imul(h ^ (h >>> 13), 0x5bd1e995);
  return ((h ^ (h >>> 15)) >>> 0) / 4294967296;
};
function mulberry(seed) {
  return () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}
// Kuhn decomposition: 6 tets per cube sharing the 0-7 diagonal; corners encoded as bit x|y<<1|z<<2.
const KUHN = [[0, 1, 3, 7], [0, 1, 5, 7], [0, 2, 3, 7], [0, 2, 6, 7], [0, 4, 5, 7], [0, 4, 6, 7]];
const KUHN_EDGES = (() => {
  const s = new Set(), out = [];
  for (const t of KUHN) for (let a = 0; a < 4; a++) for (let b = a + 1; b < 4; b++) {
    const k = Math.min(t[a], t[b]) * 8 + Math.max(t[a], t[b]);
    if (!s.has(k)) { s.add(k); out.push([Math.min(t[a], t[b]), Math.max(t[a], t[b])]); }
  }
  return out;
})();

function bary(p, a, b, c, d, out) {
  // Solve p = a + u(b-a) + v(c-a) + w(d-a)
  const e1x = b[0] - a[0], e1y = b[1] - a[1], e1z = b[2] - a[2];
  const e2x = c[0] - a[0], e2y = c[1] - a[1], e2z = c[2] - a[2];
  const e3x = d[0] - a[0], e3y = d[1] - a[1], e3z = d[2] - a[2];
  const px = p[0] - a[0], py = p[1] - a[1], pz = p[2] - a[2];
  const det = e1x * (e2y * e3z - e2z * e3y) - e2x * (e1y * e3z - e1z * e3y) + e3x * (e1y * e2z - e1z * e2y);
  const inv = 1 / det;
  const u = (px * (e2y * e3z - e2z * e3y) - e2x * (py * e3z - pz * e3y) + e3x * (py * e2z - pz * e2y)) * inv;
  const v = (e1x * (py * e3z - pz * e3y) - px * (e1y * e3z - e1z * e3y) + e3x * (e1y * pz - e1z * py)) * inv;
  const w = (e1x * (e2y * pz - e2z * py) - e2x * (e1y * pz - e1z * py) + px * (e1y * e2z - e1z * e2y)) * inv;
  out[0] = 1 - u - v - w; out[1] = u; out[2] = v; out[3] = w;
  return Math.min(out[0], u, v, w);
}

// ---------------------------------------------------------------------------------------------
/**
 * Bake every mesh of the loaded car into car-local space.
 * @returns {Array<{name, piece, partId, material, pos: Float32Array, nrm: Float32Array, index: Uint32Array, surfaceRole}>}
 */
export function bakeCarMeshes(root) {
  root.updateMatrixWorld(true);
  const inv = new Matrix4().copy(root.matrixWorld).invert();
  const out = [];
  const m = new Matrix4(), nm = new Matrix4(), v = new Vector3();
  root.traverse(o => {
    if (!o.isMesh) return;
    m.multiplyMatrices(inv, o.matrixWorld);
    nm.copy(m).invert().transpose();
    const g = o.geometry, P = g.attributes.position, N = g.attributes.normal;
    const n = P.count, pos = new Float32Array(n * 3), nrm = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      v.fromBufferAttribute(P, i).applyMatrix4(m); pos[i * 3] = v.x; pos[i * 3 + 1] = v.y; pos[i * 3 + 2] = v.z;
      if (N) { v.fromBufferAttribute(N, i).applyMatrix4(nm).normalize(); nrm[i * 3] = v.x; nrm[i * 3 + 1] = v.y; nrm[i * 3 + 2] = v.z; }
    }
    let index;
    if (g.index) index = Uint32Array.from(g.index.array);
    else { index = new Uint32Array(n); for (let i = 0; i < n; i++) index[i] = i; }
    if (m.determinant() < 0) for (let i = 0; i < index.length; i += 3) { const t = index[i + 1]; index[i + 1] = index[i + 2]; index[i + 2] = t; }
    out.push({ name: o.name, piece: pieceOf(o), partId: partIdOf(o), material: o.material, pos, nrm, index, surfaceRole: o.userData.surface_role });
  });
  return out;
}

// ---------------------------------------------------------------------------------------------
/**
 * Build lattice + embedding.
 * @param {ReturnType<typeof bakeCarMeshes>} meshes
 * @param {{h?: number, seed?: number}} [opts]
 */
export function buildCar(meshes, { h = 0.065, seed = 7 } = {}) {
  const t0 = Date.now();
  // --- grid -------------------------------------------------------------------------------
  const bmin = [Infinity, Infinity, Infinity], bmax = [-Infinity, -Infinity, -Infinity];
  for (const me of meshes) for (let i = 0; i < me.pos.length; i += 3) for (let a = 0; a < 3; a++) {
    bmin[a] = Math.min(bmin[a], me.pos[i + a]); bmax[a] = Math.max(bmax[a], me.pos[i + a]);
  }
  const org = bmin.map(v => v - 2 * h);
  const NX = Math.ceil((bmax[0] - org[0]) / h) + 3, NY = Math.ceil((bmax[1] - org[1]) / h) + 3, NZ = Math.ceil((bmax[2] - org[2]) / h) + 3;
  const vkey = (i, j, k) => i + NX * (j + NY * k);
  const ckey = (i, j, k) => i + (NX + 1) * (j + (NY + 1) * k); // corner key
  const cornerPos = (i, j, k, out) => {
    const J = 0.13 * h;
    out[0] = org[0] + i * h + (hash3(i, j, k) - 0.5) * 2 * J;
    out[1] = org[1] + j * h + (hash3(j + 17, k, i) - 0.5) * 2 * J;
    out[2] = org[2] + k * h + (hash3(k + 31, i, j) - 0.5) * 2 * J;
    return out;
  };

  // --- group meshes per piece ----------------------------------------------------------------
  const pieceMeshes = new Map();
  for (const me of meshes) {
    if (!pieceMeshes.has(me.piece)) pieceMeshes.set(me.piece, []);
    pieceMeshes.get(me.piece).push(me);
  }
  const pieceNames = [...pieceMeshes.keys()].filter(p => PIECES[p]);
  // Make sure every piece name used by joints resolves (quarter_rear -> body).
  const alias = name => (name === 'quarter_rear' ? 'body' : name);

  // --- sample surfaces & voxelise ------------------------------------------------------------
  function sampleMeshes(list, step, cb) {
    for (const me of list) {
      const P = me.pos, I = me.index;
      for (let t = 0; t < I.length; t += 3) {
        const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
        const ab = Math.hypot(P[b] - P[a], P[b + 1] - P[a + 1], P[b + 2] - P[a + 2]);
        const ac = Math.hypot(P[c] - P[a], P[c + 1] - P[a + 1], P[c + 2] - P[a + 2]);
        const bc = Math.hypot(P[c] - P[b], P[c + 1] - P[b + 1], P[c + 2] - P[b + 2]);
        const n = Math.max(1, Math.ceil(Math.max(ab, ac, bc) / step));
        for (let i = 0; i <= n; i++) for (let j = 0; j <= n - i; j++) {
          const u = i / n, v = j / n, w = 1 - u - v;
          cb(P[a] * w + P[b] * u + P[c] * v, P[a + 1] * w + P[b + 1] * u + P[c + 1] * v, P[a + 2] * w + P[b + 2] * u + P[c + 2] * v, me, t);
        }
      }
    }
  }
  const pieceVox = new Map(); // piece -> Set(vkey)
  const pieceSamples = new Map(); // piece -> Float32Array samples (for contact allowance)
  for (const p of pieceNames) {
    if (PIECES[p].wheel) continue;
    const set = new Set(), samples = [];
    const margin = 0.22 * h;
    sampleMeshes(pieceMeshes.get(p), h / 3, (x, y, z) => {
      const fx = (x - org[0]) / h, fy = (y - org[1]) / h, fz = (z - org[2]) / h;
      const i = Math.floor(fx), j = Math.floor(fy), k = Math.floor(fz);
      set.add(vkey(i, j, k));
      // thicken so thin skins stay face-connected
      const rx = fx - i, ry = fy - j, rz = fz - k, m = margin / h;
      if (rx < m) set.add(vkey(i - 1, j, k)); else if (rx > 1 - m) set.add(vkey(i + 1, j, k));
      if (ry < m) set.add(vkey(i, j - 1, k)); else if (ry > 1 - m) set.add(vkey(i, j + 1, k));
      if (rz < m) set.add(vkey(i, j, k - 1)); else if (rz > 1 - m) set.add(vkey(i, j, k + 1));
      samples.push(x, y, z);
    });
    pieceVox.set(p, set);
    pieceSamples.set(p, Float32Array.from(samples));
  }

  // --- chunks ----------------------------------------------------------------------------------
  const rand = mulberry(seed);
  const chunks = []; // {name, piece, voxels: number[], nodeStart, nodeCount, cornerToNode: Map}
  const voxChunk = new Map(); // piece -> Map(vkey -> chunk index)
  const vc = (key) => { const i = key % NX, j = Math.floor(key / NX) % NY, k = Math.floor(key / (NX * NY)); return [i, j, k]; };
  for (const p of pieceNames) {
    const def = PIECES[p];
    if (def.wheel) continue;
    const vox = [...pieceVox.get(p)];
    const map = new Map();
    const K = def.chunks || 1;
    if (K === 1) {
      const ci = chunks.length;
      chunks.push({ name: p, piece: p, voxels: vox });
      for (const v of vox) map.set(v, ci);
    } else {
      // k-means on voxel centres (deterministic seeds) -> brittle fragments
      const C = vox.map(vc);
      const seeds = [C[Math.floor(rand() * C.length)]];
      while (seeds.length < K) {
        let best = null, bd = -1;
        for (const c of C) {
          let d = Infinity; for (const s of seeds) d = Math.min(d, (c[0] - s[0]) ** 2 + (c[1] - s[1]) ** 2 * 1.5 + (c[2] - s[2]) ** 2);
          d *= 0.6 + rand() * 0.8; if (d > bd) { bd = d; best = c; }
        }
        seeds.push(best.slice());
      }
      let assign = new Int32Array(C.length);
      for (let it = 0; it < 8; it++) {
        for (let n = 0; n < C.length; n++) {
          let bi = 0, bd = Infinity;
          for (let s = 0; s < K; s++) { const d = (C[n][0] - seeds[s][0]) ** 2 + (C[n][1] - seeds[s][1]) ** 2 + (C[n][2] - seeds[s][2]) ** 2; if (d < bd) { bd = d; bi = s; } }
          assign[n] = bi;
        }
        const acc = Array.from({ length: K }, () => [0, 0, 0, 0]);
        for (let n = 0; n < C.length; n++) { const a = acc[assign[n]]; a[0] += C[n][0]; a[1] += C[n][1]; a[2] += C[n][2]; a[3]++; }
        for (let s = 0; s < K; s++) if (acc[s][3]) seeds[s] = [acc[s][0] / acc[s][3], acc[s][1] / acc[s][3], acc[s][2] / acc[s][3]];
      }
      const base = chunks.length;
      for (let s = 0; s < K; s++) chunks.push({ name: `${p}#${s}`, piece: p, voxels: [] });
      for (let n = 0; n < vox.length; n++) { chunks[base + assign[n]].voxels.push(vox[n]); map.set(vox[n], base + assign[n]); }
      for (let s = K - 1; s >= 0; s--) if (!chunks[base + s].voxels.length) chunks[base + s].voxels = null;
    }
    voxChunk.set(p, map);
  }

  // --- nodes / beams / tets for voxel chunks --------------------------------------------------
  const rest = [], mass = [], nodeChunk = [];
  const beams = { i: [], j: [], L0: [], fy: [], tear: [], kind: [], comp: [] };
  const tets = []; // flat 4 node ids
  const tetOfVoxel = []; // per chunk: Map(vkey -> first tet index)
  const tmp = [0, 0, 0];
  function addBeam(a, b, fy, tear, kind, comp = 0, L0 = null) {
    const L = L0 ?? Math.hypot(rest[a * 3] - rest[b * 3], rest[a * 3 + 1] - rest[b * 3 + 1], rest[a * 3 + 2] - rest[b * 3 + 2]);
    beams.i.push(a); beams.j.push(b); beams.L0.push(L); beams.fy.push(fy); beams.tear.push(tear); beams.kind.push(kind); beams.comp.push(comp);
  }
  for (let ci = 0; ci < chunks.length; ci++) {
    const ch = chunks[ci];
    if (!ch.voxels) { tetOfVoxel.push(null); continue; }
    const def = PIECES[ch.piece];
    ch.nodeStart = nodeChunk.length;
    const cmap = new Map();
    ch.cornerToNode = cmap;
    const node = (i, j, k) => {
      const key = ckey(i, j, k);
      let n = cmap.get(key);
      if (n === undefined) {
        n = nodeChunk.length; cmap.set(key, n);
        cornerPos(i, j, k, tmp); rest.push(tmp[0], tmp[1], tmp[2]); mass.push(0); nodeChunk.push(ci);
      }
      return n;
    };
    const edgeSet = new Set();
    const tv = new Map();
    for (const v of ch.voxels) {
      const [i, j, k] = vc(v);
      const c = [];
      for (let b = 0; b < 8; b++) c.push(node(i + (b & 1), j + ((b >> 1) & 1), k + ((b >> 2) & 1)));
      for (const [a, b] of KUHN_EDGES) {
        const lo = Math.min(c[a], c[b]), hi = Math.max(c[a], c[b]), key = lo * 4194304 + hi;
        if (edgeSet.has(key)) continue;
        edgeSet.add(key);
        addBeam(lo, hi, def.ey, def.tear, BEAM.STRUCT);
      }
      tv.set(v, tets.length / 4);
      for (const t of KUHN) tets.push(c[t[0]], c[t[1]], c[t[2]], c[t[3]]);
    }
    tetOfVoxel.push(tv);
    ch.nodeCount = nodeChunk.length - ch.nodeStart;
  }

  // --- wheels: structured cylinder lattice (rolls smoothly, spins about two axle nodes) ---------
  const wheelInfo = [];
  const SEG = 24;
  for (const p of pieceNames) {
    if (!PIECES[p].wheel) continue;
    const list = pieceMeshes.get(p);
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const me of list) for (let i = 0; i < me.pos.length; i += 3) for (let a = 0; a < 3; a++) { lo[a] = Math.min(lo[a], me.pos[i + a]); hi[a] = Math.max(hi[a], me.pos[i + a]); }
    const cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2, cz = (lo[2] + hi[2]) / 2;
    const R = (hi[1] - lo[1]) / 2 / Math.cos(Math.PI / SEG) + 0.002, Rr = R * 0.62, hw = (hi[0] - lo[0]) / 2 + 0.004;
    const ci = chunks.length;
    const ch = { name: p, piece: p, voxels: null, wheel: true, nodeStart: nodeChunk.length };
    chunks.push(ch);
    const addNode = (x, y, z) => { rest.push(x, y, z); mass.push(0); nodeChunk.push(ci); return nodeChunk.length - 1; };
    const inner = Math.sign(-cx); // +1 if inner side is toward +x
    const axle = [addNode(cx - hw, cy, cz), addNode(cx + hw, cy, cz)];
    const rim = [[], []], tyre = [[], []];
    for (let s = 0; s < SEG; s++) {
      const a = (s / SEG) * Math.PI * 2, ca = Math.cos(a), sa = Math.sin(a);
      for (let l = 0; l < 2; l++) {
        const x = cx + (l ? hw : -hw);
        rim[l].push(addNode(x, cy + Rr * sa, cz + Rr * ca));
        tyre[l].push(addNode(x, cy + R * sa, cz + R * ca));
      }
    }
    ch.nodeCount = nodeChunk.length - ch.nodeStart;
    const tfirst = tets.length / 4;
    const prism = (a0, b0, c0, a1, b1, c1) => { tets.push(a0, b0, c0, a1, b0, c0, a1, b1, c0, a1, b1, c1); };
    for (let s = 0; s < SEG; s++) {
      const s1 = (s + 1) % SEG;
      prism(axle[0], rim[0][s], rim[0][s1], axle[1], rim[1][s], rim[1][s1]);
      prism(rim[0][s], tyre[0][s], tyre[0][s1], rim[1][s], tyre[1][s], tyre[1][s1]);
      prism(rim[0][s], tyre[0][s1], rim[0][s1], rim[1][s], tyre[1][s1], rim[1][s1]);
      for (let l = 0; l < 2; l++) {
        addBeam(rim[l][s], rim[l][s1], 0.02, 2, BEAM.SPOKE);
        addBeam(tyre[l][s], tyre[l][s1], 0.12, 2, BEAM.SPOKE, 2e-6);
        addBeam(rim[l][s], tyre[l][s], 0.12, 2, BEAM.SPOKE, 4e-6);
        addBeam(rim[l][s], tyre[l][s1], 0.12, 2, BEAM.SPOKE, 4e-6);
        addBeam(tyre[l][s], rim[l][s1], 0.12, 2, BEAM.SPOKE, 4e-6);
        addBeam(axle[0], rim[l][s], 0.02, 2, BEAM.SPOKE);
        addBeam(axle[1], rim[l][s], 0.02, 2, BEAM.SPOKE);
        addBeam(axle[l], tyre[l][s], 0.12, 2, BEAM.SPOKE, 3e-6);
      }
      addBeam(rim[0][s], rim[1][s], 0.02, 2, BEAM.SPOKE);
      addBeam(tyre[0][s], tyre[1][s], 0.12, 2, BEAM.SPOKE, 2e-6);
      addBeam(rim[0][s], rim[1][s1], 0.02, 2, BEAM.SPOKE);
      addBeam(tyre[0][s], tyre[1][s1], 0.12, 2, BEAM.SPOKE, 2e-6);
    }
    addBeam(axle[0], axle[1], 0.02, 2, BEAM.SPOKE);
    ch.tetRange = [tfirst, tets.length / 4];
    wheelInfo.push({ piece: p, chunk: ci, axle, centre: [cx, cy, cz], radius: R, halfWidth: hw, inner });
  }

  const NN = nodeChunk.length;
  const restArr = Float64Array.from(rest);

  // --- spatial index over rest nodes ----------------------------------------------------------
  const cell = h * 1.01;
  const grid = new Map();
  const gk = (x, y, z) => `${Math.floor(x / cell)},${Math.floor(y / cell)},${Math.floor(z / cell)}`;
  for (let n = 0; n < NN; n++) {
    const k = gk(restArr[n * 3], restArr[n * 3 + 1], restArr[n * 3 + 2]);
    if (!grid.has(k)) grid.set(k, []);
    grid.get(k).push(n);
  }
  function nearby(x, y, z, r, filter) {
    const out = [], R = Math.ceil(r / cell);
    const ix = Math.floor(x / cell), iy = Math.floor(y / cell), iz = Math.floor(z / cell);
    for (let a = -R; a <= R; a++) for (let b = -R; b <= R; b++) for (let c = -R; c <= R; c++) {
      const l = grid.get(`${ix + a},${iy + b},${iz + c}`); if (!l) continue;
      for (const n of l) {
        const d = Math.hypot(restArr[n * 3] - x, restArr[n * 3 + 1] - y, restArr[n * 3 + 2] - z);
        if (d <= r && (!filter || filter(n))) out.push([n, d]);
      }
    }
    return out.sort((p, q) => p[1] - q[1]);
  }
  const chunkPiece = chunks.map(c => c.piece);
  const pieceOfNode = n => chunkPiece[nodeChunk[n]];

  // --- connect stray islands inside each chunk (e.g. wiper arms) ------------------------------
  {
    const parent = new Int32Array(NN); for (let i = 0; i < NN; i++) parent[i] = i;
    const find = a => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
    for (let b = 0; b < beams.i.length; b++) { const a = find(beams.i[b]), c = find(beams.j[b]); if (a !== c) parent[a] = c; }
    for (const ch of chunks) {
      if (!ch.nodeCount || ch.wheel) continue;
      const comps = new Map();
      for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const r = find(n); if (!comps.has(r)) comps.set(r, []); comps.get(r).push(n); }
      if (comps.size < 2) continue;
      const sorted = [...comps.values()].sort((a, b) => b.length - a.length);
      const mainRoot = find(sorted[0][0]);
      const def = PIECES[ch.piece];
      for (const comp of sorted.slice(1)) {
        // weld each island node set to the nearest main-component nodes
        let made = 0;
        const pairs = [];
        for (const n of comp) {
          const nb = nearby(restArr[n * 3], restArr[n * 3 + 1], restArr[n * 3 + 2], h * 4, m => nodeChunk[m] === nodeChunk[n] && find(m) === mainRoot);
          if (nb.length) pairs.push([n, nb[0][0], nb[0][1]]);
        }
        pairs.sort((a, b) => a[2] - b[2]);
        for (const [a, b] of pairs.slice(0, 6)) { addBeam(a, b, 8000, 0.01, BEAM.WELD, 0.01 / 8000); made++; }
        if (!made) { // island far from its piece: hang it onto anything nearby
          for (const n of comp.slice(0, 4)) {
            const nb = nearby(restArr[n * 3], restArr[n * 3 + 1], restArr[n * 3 + 2], h * 8, m => find(m) === mainRoot);
            if (nb.length) addBeam(n, nb[0][0], 8000, 0.01, BEAM.WELD, 0.01 / 8000);
          }
        }
        for (const n of comp) parent[find(n)] = mainRoot;
      }
    }
  }

  // --- welds between fragments of brittle pieces -----------------------------------------------
  for (const p of pieceNames) {
    const def = PIECES[p];
    if (!def.chunks) continue;
    const ids = chunks.map((c, i) => (c.piece === p && c.voxels ? i : -1)).filter(i => i >= 0);
    for (let a = 0; a < ids.length; a++) for (let b = a + 1; b < ids.length; b++) {
      const A = chunks[ids[a]].cornerToNode, B = chunks[ids[b]].cornerToNode;
      for (const [key, na] of A) { const nb = B.get(key); if (nb !== undefined) addBeam(na, nb, def.weld, 0.008, BEAM.WELD, 0.008 / def.weld); }
    }
  }

  // --- masses ------------------------------------------------------------------------------------
  const massArr = new Float64Array(NN);
  for (const ch of chunks) {
    if (!ch.nodeCount) continue;
    const def = PIECES[ch.piece];
    const pieceChunks = chunks.filter(c => c.piece === ch.piece && c.nodeCount);
    const total = pieceChunks.reduce((s, c) => s + c.nodeCount, 0);
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) massArr[n] = def.mass / total;
    if (ch.wheel) { massArr[ch.nodeStart] = massArr[ch.nodeStart + 1] = 1.5; }
  }
  { // battery pack under the floor and rear motor -> low, rear-biased centre of mass
    const body = chunks.find(c => c.name === 'body');
    const floor = [], motor = [];
    for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n++) {
      const y = restArr[n * 3 + 1], z = restArr[n * 3 + 2];
      if (y < 0.42 && z > -0.95 && z < 0.75) floor.push(n);
      if (y < 0.75 && z < -0.95) motor.push(n);
    }
    for (const n of floor) massArr[n] += 170 / floor.length;
    for (const n of motor) massArr[n] += 55 / motor.length;
  }

  // --- joints between pieces ---------------------------------------------------------------------
  const pieceBounds = new Map();
  for (const ch of chunks) {
    if (!ch.nodeCount) continue;
    let b = pieceBounds.get(ch.piece);
    if (!b) { b = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity]; pieceBounds.set(ch.piece, b); }
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) for (let a = 0; a < 3; a++) { b[a] = Math.min(b[a], restArr[n * 3 + a]); b[a + 3] = Math.max(b[a + 3], restArr[n * 3 + a]); }
  }
  const jointLog = [];
  const anchorsOf = new Map();
  for (const p of pieceNames) {
    const def = PIECES[p];
    if (!def.joints) continue;
    const parents = new Set(def.parents.map(alias).filter(q => pieceBounds.has(q)));
    const bb = pieceBounds.get(p);
    const own = [];
    for (const ch of chunks) if (ch.piece === p && ch.nodeCount) for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) own.push(n);
    for (const J of def.joints) {
      const jp = J.parents ? new Set(J.parents.map(alias).filter(q => pieceBounds.has(q))) : parents;
      let grow = 0;
      const inBox = n => {
        if (!J.box) return true;
        const u = (restArr[n * 3] - bb[0]) / (bb[3] - bb[0] || 1), v = (restArr[n * 3 + 1] - bb[1]) / (bb[4] - bb[1] || 1), w = (restArr[n * 3 + 2] - bb[2]) / (bb[5] - bb[2] || 1);
        return u >= J.box[0] - grow && u <= J.box[1] + grow && v >= J.box[2] - grow && v <= J.box[3] + grow && w >= J.box[4] - grow && w <= J.box[5] + grow;
      };
      let cand = [];
      for (; grow < 1 && cand.length < J.k; grow += 0.12) {
        let reach = 1.5 * h;
        for (let tries = 0; tries < 3 && cand.length < J.k; tries++, reach *= 1.5) {
          cand = own.filter(n => inBox(n)).map(n => [n, nearby(restArr[n * 3], restArr[n * 3 + 1], restArr[n * 3 + 2], reach, m => jp.has(pieceOfNode(m)))]).filter(c => c[1].length);
        }
      }
      if (!cand.length) { jointLog.push(`${p}: joint found no parent nodes`); continue; }
      // farthest point sampling for K anchor points
      const picks = [cand[Math.floor(cand.length / 2)]];
      while (picks.length < Math.min(J.k, cand.length)) {
        let best = null, bd = -1;
        for (const c of cand) {
          let d = Infinity; for (const q of picks) d = Math.min(d, Math.hypot(restArr[c[0] * 3] - restArr[q[0] * 3], restArr[c[0] * 3 + 1] - restArr[q[0] * 3 + 1], restArr[c[0] * 3 + 2] - restArr[q[0] * 3 + 2]));
          if (d > bd) { bd = d; best = c; }
        }
        picks.push(best);
      }
      // each anchor ties its node and two lattice neighbours to up to 3 parent nodes -> moment-carrying bolt
      const perPoint = J.F / picks.length;
      const firstBeam = beams.i.length;
      anchorsOf.get(p)?.push({ latch: J.latch || 0, parent: picks[0][1][0][0], beams: null }) ?? anchorsOf.set(p, [{ latch: J.latch || 0, parent: picks[0][1][0][0], beams: null }]);
      for (const [n, nb] of picks) {
        const own2 = nearby(restArr[n * 3], restArr[n * 3 + 1], restArr[n * 3 + 2], h * 1.2, m => pieceOfNode(m) === p).slice(0, 3).map(x => x[0]);
        const targets = nb.slice(0, 3).map(x => x[0]);
        const list = [];
        for (const a of own2) for (const b of targets) list.push([a, b]);
        for (const [a, b] of list) { const Fb = (perPoint / list.length) * 2; addBeam(a, b, Fb, JOINT_GIVE, BEAM.JOINT, JOINT_GIVE / Fb); }
      }
      { const l = anchorsOf.get(p); l[l.length - 1].beams = [firstBeam, beams.i.length]; }
      jointLog.push(`${p}: ${picks.length} anchors`);
    }
  }
  // latches: when the opening around a door / hood / tailgate distorts, the latch lets go
  const latches = [];
  for (const [p, list] of anchorsOf) {
    for (const L of list) {
      if (!L.latch || !L.beams) continue;
      const pairs = [];
      for (const o of list) {
        if (o === L) continue;
        const a = L.parent, b = o.parent;
        pairs.push(a, b, Math.hypot(restArr[a * 3] - restArr[b * 3], restArr[a * 3 + 1] - restArr[b * 3 + 1], restArr[a * 3 + 2] - restArr[b * 3 + 2]));
      }
      if (pairs.length) latches.push({ piece: p, strain: L.latch, beams: L.beams, pairs: Float32Array.from(pairs) });
    }
  }

  // wheels -> body: two near-horizontal control arms per axle node (stiff sideways, swing vertically)
  // plus a soft coil-over spring to the arch top with bump / rebound stops.
  const mounts = new Set();
  for (const w of wheelInfo) {
    const [cx, cy, cz] = w.centre, side = Math.sign(cx);
    const isBody = m => pieceOfNode(m) === 'body';
    for (let l = 0; l < 2; l++) {
      const a = w.axle[l];
      const ax = restArr[a * 3];
      for (const dz of [-0.28, 0.28]) {
        const target = [ax - side * 0.42, cy - 0.02, cz + dz];
        const nb = nearby(target[0], target[1], target[2], 0.3, isBody);
        if (nb.length) { addBeam(a, nb[0][0], 14e3, 0.009, BEAM.SUSP, 0.009 / 14e3); mounts.add(nb[0][0]); }
      }
      const top = nearby(ax - side * 0.22, cy + 0.36, cz, 0.3, isBody);
      if (top.length) { addBeam(a, top[0][0], SPRING.breakF, SPRING.travel, BEAM.SPRING, 1 / SPRING.k); mounts.add(top[0][0]); }
    }
  }

  // reinforced suspension mounts: heavy nodes let the spring/damper act on real inertia
  for (const m of mounts) massArr[m] += 2.5;

  // --- contact allowance: how far a node may sink before the real skin touches --------------------
  const allow = new Float32Array(NN);
  {
    const sgrid = new Map();
    for (const [p, S] of pieceSamples) {
      for (let s = 0; s < S.length; s += 3) {
        const k = `${p}|${Math.floor(S[s] / h)},${Math.floor(S[s + 1] / h)},${Math.floor(S[s + 2] / h)}`;
        let l = sgrid.get(k); if (!l) sgrid.set(k, l = []); l.push(s);
      }
    }
    for (let n = 0; n < NN; n++) {
      const p = pieceOfNode(n);
      if (PIECES[p].wheel) { allow[n] = 0; continue; }
      const S = pieceSamples.get(p), x = restArr[n * 3], y = restArr[n * 3 + 1], z = restArr[n * 3 + 2];
      let best = Infinity;
      const ix = Math.floor(x / h), iy = Math.floor(y / h), iz = Math.floor(z / h);
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
        const l = sgrid.get(`${p}|${ix + a},${iy + b},${iz + c}`); if (!l) continue;
        for (const s of l) best = Math.min(best, (S[s] - x) ** 2 + (S[s + 1] - y) ** 2 + (S[s + 2] - z) ** 2);
      }
      allow[n] = Math.min(Math.sqrt(best), h * 0.8);
    }
  }

  // --- embedding of render vertices ---------------------------------------------------------------
  const usedTets = new Map(); // sim tet -> render tet id
  const usedList = [];
  const useTet = t => { let r = usedTets.get(t); if (r === undefined) { r = usedList.length; usedTets.set(t, r); usedList.push(t); } return r; };
  const A = [0, 0, 0], B = [0, 0, 0], C = [0, 0, 0], D = [0, 0, 0], W = [0, 0, 0, 0], P = [0, 0, 0];
  const restOf = (n, out) => { out[0] = restArr[n * 3]; out[1] = restArr[n * 3 + 1]; out[2] = restArr[n * 3 + 2]; return out; };
  const bestW = [0, 0, 0, 0];
  function embedInTets(tlist, p) {
    let best = -1, bm = -Infinity;
    for (const t of tlist) {
      const m = bary(p, restOf(tets[t * 4], A), restOf(tets[t * 4 + 1], B), restOf(tets[t * 4 + 2], C), restOf(tets[t * 4 + 3], D), W);
      if (m > bm) { bm = m; best = t; bestW[0] = W[0]; bestW[1] = W[1]; bestW[2] = W[2]; bestW[3] = W[3]; if (m >= 0) break; }
    }
    return best;
  }
  function voxelTetsOfChunk(ci, i, j, k, R) {
    const tv = tetOfVoxel[ci], out = [];
    for (let a = -R; a <= R; a++) for (let b = -R; b <= R; b++) for (let c = -R; c <= R; c++) {
      if (R && Math.max(Math.abs(a), Math.abs(b), Math.abs(c)) < R) continue;
      const t0 = tv.get(vkey(i + a, j + b, k + c)); if (t0 === undefined) continue;
      for (let s = 0; s < 6; s++) out.push(t0 + s);
    }
    return out;
  }
  // lens / glass visibility groups
  const groups = [{ name: 'none' }];
  const groupOfMesh = me => {
    const def = PIECES[me.piece];
    if (def?.glass === 'tempered') return me.piece;
    if (def?.glass === 'laminated') return 'windshield';
    const L = LENSES.find(l => l.re.test(me.name));
    return L ? L.name : null;
  };
  const groupIndex = new Map();
  const renderMeshes = [];
  let worstOut = 0;
  for (const me of meshes) {
    const def = PIECES[me.piece];
    if (!def) continue;
    const gname = groupOfMesh(me);
    let gid = 0;
    if (gname) {
      if (!groupIndex.has(gname)) {
        groupIndex.set(gname, groups.length);
        const L = LENSES.find(l => l.name === gname);
        groups.push({ name: gname, piece: me.piece, kind: def.glass || 'lens', shard: L ? L.shard : (def.glass === 'tempered' ? 6 : 0), count: L ? L.count : (def.glass === 'tempered' ? 520 : 160), tris: [] });
      }
      gid = groupIndex.get(gname);
    }
    const nv = me.pos.length / 3;
    // Fragmented pieces: split vertices so every triangle lives in exactly one fragment.
    let pos = me.pos, nrm = me.nrm, index = me.index, vChunk;
    const pieceChunks = chunks.map((c, i) => (c.piece === me.piece && c.voxels ? i : -1)).filter(i => i >= 0);
    if (def.wheel) {
      vChunk = new Int32Array(nv).fill(chunks.findIndex(c => c.piece === me.piece));
    } else if (pieceChunks.length === 1) {
      vChunk = new Int32Array(nv).fill(pieceChunks[0]);
    } else {
      const map = voxChunk.get(me.piece);
      const remap = new Map(); const np = [], nn = [], vc2 = [];
      index = new Uint32Array(me.index.length);
      for (let t = 0; t < me.index.length; t += 3) {
        let cx = 0, cy = 0, cz = 0;
        for (let q = 0; q < 3; q++) { const v = me.index[t + q]; cx += me.pos[v * 3]; cy += me.pos[v * 3 + 1]; cz += me.pos[v * 3 + 2]; }
        const key = vkey(Math.floor((cx / 3 - org[0]) / h), Math.floor((cy / 3 - org[1]) / h), Math.floor((cz / 3 - org[2]) / h));
        let ci = map.get(key);
        if (ci === undefined) { // centroid fell into an empty voxel: take the closest occupied one
          const i0 = Math.floor((cx / 3 - org[0]) / h), j0 = Math.floor((cy / 3 - org[1]) / h), k0 = Math.floor((cz / 3 - org[2]) / h);
          for (let R = 1; R <= 4 && ci === undefined; R++)
            for (let a = -R; a <= R && ci === undefined; a++) for (let b = -R; b <= R && ci === undefined; b++) for (let c = -R; c <= R && ci === undefined; c++) ci = map.get(vkey(i0 + a, j0 + b, k0 + c));
          ci ??= pieceChunks[0];
        }
        for (let q = 0; q < 3; q++) {
          const v = me.index[t + q], k = v * 64 + (ci - pieceChunks[0]);
          let nvI = remap.get(k);
          if (nvI === undefined) { nvI = vc2.length; remap.set(k, nvI); np.push(me.pos[v * 3], me.pos[v * 3 + 1], me.pos[v * 3 + 2]); nn.push(me.nrm[v * 3], me.nrm[v * 3 + 1], me.nrm[v * 3 + 2]); vc2.push(ci); }
          index[t + q] = nvI;
        }
      }
      pos = Float32Array.from(np); nrm = Float32Array.from(nn); vChunk = Int32Array.from(vc2);
    }
    const n2 = pos.length / 3;
    const aTet = new Float32Array(n2), aBary = new Float32Array(n2 * 3);
    for (let v = 0; v < n2; v++) {
      P[0] = pos[v * 3]; P[1] = pos[v * 3 + 1]; P[2] = pos[v * 3 + 2];
      const ci = vChunk[v];
      let t;
      if (chunks[ci].wheel) {
        const [t0, t1] = chunks[ci].tetRange; const list = []; for (let q = t0; q < t1; q++) list.push(q);
        t = embedInTets(list, P);
      } else {
        const i = Math.floor((P[0] - org[0]) / h), j = Math.floor((P[1] - org[1]) / h), k = Math.floor((P[2] - org[2]) / h);
        for (let R = 0; R <= 3; R++) {
          const list = voxelTetsOfChunk(ci, i, j, k, R);
          if (!list.length) continue;
          t = embedInTets(list, P);
          if (bestW[0] > -0.6 && bestW[1] > -0.6 && bestW[2] > -0.6 && bestW[3] > -0.6) break;
        }
      }
      if (t === undefined || t < 0) throw new Error(`embedding failed for ${me.name}`);
      worstOut = Math.min(worstOut, bestW[0], bestW[1], bestW[2], bestW[3]);
      aTet[v] = useTet(t);
      aBary[v * 3] = bestW[1]; aBary[v * 3 + 1] = bestW[2]; aBary[v * 3 + 2] = bestW[3];
    }
    if (gid) {
      const g = groups[gid];
      for (let tI = 0; tI < index.length; tI += 3) g.tris.push([index[tI], index[tI + 1], index[tI + 2]].map(v => [aTet[v], aBary[v * 3], aBary[v * 3 + 1], aBary[v * 3 + 2], pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]]));
    }
    renderMeshes.push({ name: me.name, piece: me.piece, material: me.material, pos, nrm, index, aTet, aBary, group: gid, surfaceRole: me.surfaceRole });
  }

  // --- render tet table: node ids + inverse rest edge matrix (for normals) --------------------------
  const RT = usedList.length;
  const rtNodes = new Int32Array(RT * 4), rtDmInv = new Float32Array(RT * 9), rtSim = Int32Array.from(usedList);
  for (let r = 0; r < RT; r++) {
    const t = usedList[r];
    for (let q = 0; q < 4; q++) rtNodes[r * 4 + q] = tets[t * 4 + q];
    const a = tets[t * 4] * 3, b = tets[t * 4 + 1] * 3, c = tets[t * 4 + 2] * 3, d = tets[t * 4 + 3] * 3;
    const m = [restArr[b] - restArr[a], restArr[c] - restArr[a], restArr[d] - restArr[a], restArr[b + 1] - restArr[a + 1], restArr[c + 1] - restArr[a + 1], restArr[d + 1] - restArr[a + 1], restArr[b + 2] - restArr[a + 2], restArr[c + 2] - restArr[a + 2], restArr[d + 2] - restArr[a + 2]];
    const M = new Matrix4().set(m[0], m[1], m[2], 0, m[3], m[4], m[5], 0, m[6], m[7], m[8], 0, 0, 0, 0, 1).invert();
    const e = M.elements; // column-major
    // store rows of Dm^-1
    rtDmInv.set([e[0], e[4], e[8], e[1], e[5], e[9], e[2], e[6], e[10]], r * 9);
  }

  // --- groups: shard spawn samples + sentinel node pairs ------------------------------------------------
  const grand = mulberry(seed * 13 + 5);
  for (let g = 1; g < groups.length; g++) {
    const G = groups[g];
    const tris = G.tris; delete G.tris;
    const areas = tris.map(([a, b, c]) => {
      const ux = b[4] - a[4], uy = b[5] - a[5], uz = b[6] - a[6], vx = c[4] - a[4], vy = c[5] - a[5], vz = c[6] - a[6];
      return 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    });
    const total = areas.reduce((s, a) => s + a, 0);
    G.area = total;
    const S = [];
    for (let s = 0; s < G.count; s++) {
      let r = grand() * total, ti = 0; while (ti < tris.length - 1 && r > areas[ti]) { r -= areas[ti]; ti++; }
      let u = grand(), v = grand(); if (u + v > 1) { u = 1 - u; v = 1 - v; }
      const [a, b, c] = tris[ti];
      const pick = u < 0.5 ? (v < 0.5 ? a : c) : b; // tet of nearest corner
      const x = a[4] + (b[4] - a[4]) * u + (c[4] - a[4]) * v, y = a[5] + (b[5] - a[5]) * u + (c[5] - a[5]) * v, z = a[6] + (b[6] - a[6]) * u + (c[6] - a[6]) * v;
      const rt = pick[0], t = usedList[rt];
      bary([x, y, z], restOf(tets[t * 4], A), restOf(tets[t * 4 + 1], B), restOf(tets[t * 4 + 2], C), restOf(tets[t * 4 + 3], D), W);
      S.push(tets[t * 4], tets[t * 4 + 1], tets[t * 4 + 2], tets[t * 4 + 3], W[1], W[2], W[3]);
    }
    G.samples = Float32Array.from(S);
    // nodes hosting this group: their distortion triggers breakage
    const host = new Set();
    for (let s = 0; s < S.length; s += 7) for (let q = 0; q < 4; q++) host.add(S[s + q]);
    const hl = [...host];
    const pairs = [];
    for (let tries = 0; tries < 400 && pairs.length < 40; tries++) {
      const a = hl[Math.floor(grand() * hl.length)], b = hl[Math.floor(grand() * hl.length)];
      const d = Math.hypot(restArr[a * 3] - restArr[b * 3], restArr[a * 3 + 1] - restArr[b * 3 + 1], restArr[a * 3 + 2] - restArr[b * 3 + 2]);
      if (d > h * 1.5) pairs.push(a, b, d);
    }
    G.hostNodes = Int32Array.from(hl);
    G.pairs = Float32Array.from(pairs);
    // centre in rest space
    let cx = 0, cy = 0, cz = 0; for (const n of hl) { cx += restArr[n * 3]; cy += restArr[n * 3 + 1]; cz += restArr[n * 3 + 2]; }
    G.centre = [cx / hl.length, cy / hl.length, cz / hl.length];
  }

  const NB = beams.i.length;
  const out = {
    h, origin: org, dims: [NX, NY, NZ],
    nodes: { count: NN, rest: restArr, mass: massArr, chunk: Uint16Array.from(nodeChunk), allow },
    chunks: chunks.map(c => ({ name: c.name, piece: c.piece, nodeStart: c.nodeStart ?? 0, nodeCount: c.nodeCount ?? 0, wheel: !!c.wheel, glass: PIECES[c.piece].glass || null })),
    pieces: [...new Set(chunks.map(c => c.piece))],
    beams: {
      count: NB, i: Int32Array.from(beams.i), j: Int32Array.from(beams.j), L0: Float64Array.from(beams.L0),
      fy: Float32Array.from(beams.fy), tear: Float32Array.from(beams.tear), kind: Uint8Array.from(beams.kind), comp: Float32Array.from(beams.comp),
    },
    tets: Int32Array.from(tets),
    render: { tetCount: RT, tetNodes: rtNodes, tetDmInv: rtDmInv, tetSim: rtSim, meshes: renderMeshes },
    groups, wheels: wheelInfo, latches,
    stats: { nodes: NN, beams: NB, tets: tets.length / 4, renderTets: RT, worstOutside: worstOut, ms: Date.now() - t0, joints: jointLog },
  };
  return out;
}
