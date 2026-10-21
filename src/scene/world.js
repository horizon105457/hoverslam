// Procedural urban set modelled on the reference footage: a concrete box-girder viaduct on vase piers
// with anti-collision striping, the ground-level arterial road and intersection underneath, sidewalks
// with tactile paving and railings, hedges, camphor-like trees, double-arm street lamps, signals, signs,
// a construction gap in the deck parapet, and a distant skyline in the haze.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { DECK, DECK_LINES, PIERS, GROUND } from './layout.js';
import * as T from './textures.js';

const TEX = '/assets/tex/';

function loadPBR(loader, name, repeat, { diff = true, ao = false } = {}) {
  const set = (t, srgb) => { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat, repeat); t.anisotropy = 16; if (srgb) t.colorSpace = THREE.SRGBColorSpace; return t; };
  const m = {
    normalMap: set(loader.load(`${TEX}${name}_nor.jpg`), false),
    roughnessMap: set(loader.load(`${TEX}${name}_rough.jpg`), false),
  };
  if (diff) m.map = set(loader.load(`${TEX}${name}_diff.jpg`), true);
  if (ao) m.aoMap = set(loader.load(`${TEX}${name}_ao.jpg`), false);
  return m;
}

/**
 * Sweep a 2D profile [(z,y)...] along X into a mesh with metric UVs. The profile may wind either way:
 * faces and normals always point out of the (implicitly closed) profile polygon; `flip` turns them in.
 */
function sweepX(profile, x0, x1, { uvScale = 2, closed = false, flip = false } = {}) {
  const pos = [], nrm = [], uv = [], idx = [];
  const n = profile.length;
  const segs = closed ? n : n - 1;
  // signed area in the (z, y) plane: > 0 counter-clockwise
  let area = 0;
  for (let s = 0; s < n; s++) { const [za, ya] = profile[s], [zb, yb] = profile[(s + 1) % n]; area += za * yb - zb * ya; }
  const out = (area > 0) !== flip; // true: outward normal of a segment is (dz, dy) rotated clockwise
  let v = 0;
  for (let s = 0; s < segs; s++) {
    const [z0, y0] = profile[s], [z1, y1] = profile[(s + 1) % n];
    const len = Math.hypot(z1 - z0, y1 - y0);
    let nz = y1 - y0, ny = -(z1 - z0);
    const l = Math.hypot(nz, ny) || 1; nz /= l; ny /= l;
    if (!out) { nz = -nz; ny = -ny; }
    const b = pos.length / 3;
    for (const [x, z, y, vv] of [[x0, z0, y0, v], [x1, z0, y0, v], [x1, z1, y1, v + len], [x0, z1, y1, v + len]]) {
      pos.push(x, y, z); nrm.push(0, ny, nz); uv.push(x / uvScale, vv / uvScale);
    }
    // winding must agree with the normal: (b, b+2, b+1) faces (0, dz, -dy), i.e. the opposite side
    if (out) idx.push(b, b + 1, b + 2, b, b + 2, b + 3); else idx.push(b, b + 2, b + 1, b, b + 3, b + 2);
    v += len;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(nrm, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setAttribute('uv1', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  return g;
}
/** End cap polygon of a profile at x (fan triangulation, profile must be convex-ish / star shaped). */
function capX(profile, x, dir) {
  const shape = new THREE.Shape(profile.map(([z, y]) => new THREE.Vector2(z, y)));
  const s2 = new THREE.ShapeGeometry(shape);
  const P = s2.attributes.position;
  const out = new Float32Array(P.count * 3), uv = new Float32Array(P.count * 2);
  for (let i = 0; i < P.count; i++) {
    out[i * 3] = x; out[i * 3 + 1] = P.getY(i); out[i * 3 + 2] = P.getX(i);
    uv[i * 2] = P.getX(i) / 2; uv[i * 2 + 1] = P.getY(i) / 2;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(out, 3));
  g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  g.setAttribute('uv1', new THREE.BufferAttribute(uv.slice(), 2));
  const I = Array.from(s2.index.array);
  if (dir < 0) for (let i = 0; i < I.length; i += 3) [I[i + 1], I[i + 2]] = [I[i + 2], I[i + 1]];
  g.setIndex(I);
  g.computeVertexNormals();
  s2.dispose();
  return g;
}

function box(w, h, d, x, y, z) { const g = new THREE.BoxGeometry(w, h, d); g.translate(x, y, z); return g; }
function boxUV(w, h, d, x, y, z, s = 1) {
  const g = box(w, h, d, x, y, z);
  // metric UVs per face
  const P = g.attributes.position, N = g.attributes.normal, U = g.attributes.uv;
  for (let i = 0; i < P.count; i++) {
    const nx = Math.abs(N.getX(i)), ny = Math.abs(N.getY(i));
    const px = P.getX(i), py = P.getY(i), pz = P.getZ(i);
    if (ny > 0.5) U.setXY(i, px / s, pz / s); else if (nx > 0.5) U.setXY(i, pz / s, py / s); else U.setXY(i, px / s, py / s);
  }
  g.setAttribute('uv1', U.clone());
  return g;
}

// ---------------------------------------------------------------------------------------------
export function buildWorld(scene, { renderer }) {
  const loader = new THREE.TextureLoader();
  // track the PBR downloads so reflection probes are only captured once the set is fully textured
  const pending = [], load = loader.load.bind(loader);
  loader.load = url => { let done; pending.push(new Promise(r => (done = r))); return load(url, done, undefined, done); };
  const world = new THREE.Group();
  world.name = 'world';
  scene.add(world);
  const add = (geo, mat, { cast = true, receive = true, name } = {}) => {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = cast; m.receiveShadow = receive; if (name) m.name = name;
    world.add(m);
    return m;
  };

  // ---------------- materials
  const asphalt = new THREE.MeshStandardMaterial({ ...loadPBR(loader, 'asphalt_02', 1), color: 0x9a9a98, roughness: 1, normalScale: new THREE.Vector2(0.8, 0.8) });
  const deckAsphalt = asphalt.clone();
  const concrete = new THREE.MeshStandardMaterial({ ...loadPBR(loader, 'concrete_wall_008', 1, { ao: true }), color: 0xc9c4bb, roughness: 1, aoMapIntensity: 0.6 });
  const concreteRough = new THREE.MeshStandardMaterial({ ...loadPBR(loader, 'rough_concrete', 1), color: 0xbab5ad, roughness: 1 });
  const pavers = new THREE.MeshStandardMaterial({ ...loadPBR(loader, 'concrete_pavement', 1, { diff: false }), color: 0x8e8b86, roughness: 1 });
  const grass = new THREE.MeshStandardMaterial({ ...loadPBR(loader, 'sparse_grass', 1), color: 0xa9b496, roughness: 1 });
  const curbMat = new THREE.MeshStandardMaterial({ color: 0x9d9c98, roughness: 0.85 });
  const tactile = new THREE.MeshStandardMaterial({ color: 0xc9a227, roughness: 0.8 });
  const paintWear = T.paintWear();
  paintWear.repeat.set(0.25, 0.25);
  const roadPaint = new THREE.MeshStandardMaterial({ color: 0xe8e8e2, roughness: 0.75, alphaMap: paintWear, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
  const yellowPaint = roadPaint.clone(); yellowPaint.color.set(0xe0b02a);
  const steelDark = new THREE.MeshStandardMaterial({ color: 0x3c4147, metalness: 0.6, roughness: 0.5 });
  const steelGrey = new THREE.MeshStandardMaterial({ color: 0x8b9197, metalness: 0.7, roughness: 0.4 });
  const railPaint = new THREE.MeshStandardMaterial({ color: 0xdfe3e6, metalness: 0.3, roughness: 0.45 });
  const hazard = new THREE.MeshStandardMaterial({ map: T.hazardStripes(), roughness: 0.7 });
  hazard.map.repeat.set(2, 1);

  // world-space UV scaling helpers (textures repeat every `s` metres)
  const scaleUV = (g, s) => { const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, U.getX(i) / s, U.getY(i) / s); if (g.attributes.uv1) g.attributes.uv1.copy(U); return g; };

  // ---------------- ground plane (asphalt everywhere; raised kerbs/sidewalks/greens sit on top)
  {
    const g = new THREE.PlaneGeometry(1400, 1400, 1, 1);
    g.rotateX(-Math.PI / 2);
    const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, U.getX(i) * 1400 / 5, U.getY(i) * 1400 / 5);
    const m = add(g, asphalt, { cast: false, name: 'ground' });
    world.userData.ground = m;
  }

  // ---------------- sidewalks, kerbs, tactile paving, green belts (both sides, interrupted by the crossing road)
  const G = GROUND, [cx0, cx1] = G.cross;
  const xRanges = [[-420, cx0], [cx1, 420]];
  const sideSpecs = [
    { s: 1, walk: G.sidewalk, green: G.greenBelt },
    { s: -1, walk: G.sidewalkN, green: G.greenBeltN },
  ];
  const walkGeos = [], curbGeos = [], tactGeos = [], greenGeos = [], greenCurb = [];
  for (const [a, b] of xRanges) {
    const L = b - a, xc = (a + b) / 2;
    for (const sp of sideSpecs) {
      const [w0, w1] = sp.walk, [g0, g1] = sp.green;
      walkGeos.push(scaleUV(boxUV(L, G.curbH, w1 - w0, xc, G.curbH / 2, (w0 + w1) / 2), 1.2));
      const kerbZ = sp.s > 0 ? w0 + 0.12 : w1 - 0.12;
      curbGeos.push(boxUV(L, G.curbH + 0.03, 0.24, xc, (G.curbH + 0.03) / 2, kerbZ));
      const tz = sp.s > 0 ? w0 + 1.6 : w1 - 1.6;
      tactGeos.push(boxUV(L, 0.012, 0.3, xc, G.curbH + 0.006, tz));
      greenGeos.push(scaleUV(boxUV(L, G.curbH + 0.08, g1 - g0, xc, (G.curbH + 0.08) / 2, (g0 + g1) / 2), 3));
      greenCurb.push(boxUV(L, G.curbH + 0.18, 0.15, xc, (G.curbH + 0.18) / 2, sp.s > 0 ? g0 : g1));
    }
    // median under the viaduct
    greenGeos.push(scaleUV(boxUV(L, 0.2, G.medianHalf * 2 - 0.3, xc, 0.1, 0), 3));
    curbGeos.push(boxUV(L, 0.22, 0.15, xc, 0.11, G.medianHalf - 0.075), boxUV(L, 0.22, 0.15, xc, 0.11, -G.medianHalf + 0.075));
  }
  add(mergeGeometries(walkGeos), pavers, { cast: false });
  add(mergeGeometries(curbGeos), curbMat);
  add(mergeGeometries(tactGeos), tactile, { cast: false });
  add(mergeGeometries(greenGeos), grass, { cast: false });
  add(mergeGeometries(greenCurb), curbMat);

  // ---------------- road markings
  {
    const geos = [], ygeos = [];
    const strip = (x0, x1, z0, z1, list = geos) => { const g = new THREE.PlaneGeometry(x1 - x0, z1 - z0); g.rotateX(-Math.PI / 2); g.translate((x0 + x1) / 2, 0.006, (z0 + z1) / 2); const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, ((i & 1) ? x1 : x0) / 3, (i < 2 ? z0 : z1) / 3); list.push(g); };
    const dashed = (zLine, xa, xb) => { for (let x = xa; x < xb - 6; x += 15) strip(x, x + 6, zLine - 0.075, zLine + 0.075); };
    const solid = (zLine, xa, xb, w = 0.15, list) => strip(xa, xb, zLine - w / 2, zLine + w / 2, list);
    for (const [a, b] of [[-420, cx0 - 4.5], [cx1 + 4.5, 420]]) {
      // +z carriageway: 5 lanes
      solid(G.medianHalf + 0.35, a, b);
      for (let k = 1; k < 6; k++) dashed(G.medianHalf + 0.25 + k * 3.85, a, b);
      solid(G.roadHalf[1] - 0.45, a, b);
      // -z carriageway: 4 lanes
      solid(-G.medianHalf - 0.35, a, b);
      for (let k = 1; k < 4; k++) dashed(-G.medianHalf - 0.25 - k * 3.8, a, b);
      solid(G.roadHalf[0] + 0.45, a, b);
    }
    // zebra crossings across the arterial on both sides of the intersection + stop lines
    for (const xc of [cx0 - 2.5, cx1 + 2.5]) {
      for (let z = G.roadHalf[0] + 1; z < G.roadHalf[1] - 0.8; z += 1.0) if (Math.abs(z) > G.medianHalf + 0.4) strip(xc - 2, xc + 2, z, z + 0.45);
    }
    strip(cx1 + 5.0, cx1 + 5.4, G.medianHalf + 0.2, G.roadHalf[1] - 0.3);
    strip(cx0 - 5.4, cx0 - 5.0, G.roadHalf[0] + 0.3, -G.medianHalf - 0.2);
    // zebra across the crossing road, along the sidewalks
    for (const [z0, z1] of [[G.sidewalk[0] + 0.3, G.sidewalk[1] - 0.3], [G.sidewalkN[0] + 0.3, G.sidewalkN[1] - 0.3]]) {
      for (let x = cx0 + 1; x < cx1 - 0.8; x += 1.0) strip(x, x + 0.45, z0, z1);
    }
    // crossing road centre double yellow + lane lines beyond the arterial
    for (const [z0, z1] of [[G.greenBelt[1], 130], [-130, G.greenBeltN[0]]]) {
      const xm = (cx0 + cx1) / 2;
      strip(xm - 0.2, xm - 0.08, z0, z1, ygeos); strip(xm + 0.08, xm + 0.2, z0, z1, ygeos);
      for (const xl of [xm - 3.7, xm + 3.7]) for (let z = z0; z < z1 - 4; z += 12) strip(xl - 0.075, xl + 0.075, z, z + 4);
    }
    const valid = geos.filter(g => g.attributes.position.count);
    add(mergeGeometries(valid), roadPaint, { cast: false });
    add(mergeGeometries(ygeos), yellowPaint, { cast: false });
  }

  // ---------------- viaduct: box girder, deck asphalt, parapets, median barrier, joints
  const D = DECK;
  {
    const hw = D.halfWidth, top = D.top, bot = top - D.girderDepth;
    const half = [[hw, top - 0.02], [hw, top - 0.28], [hw - 0.15, top - 0.33], [5.3, top - 0.62], [4.5, top - 0.98], [D.boxHalf, bot], [0, bot]];
    const prof = [...half.map(([z, y]) => [z, y]), ...half.slice(0, -1).reverse().map(([z, y]) => [-z, y])];
    const girder = sweepX(prof, D.x0, D.x1, { uvScale: 3 });
    add(girder, concrete, { name: 'girder' });
    // deck road surface
    const deck = new THREE.PlaneGeometry(D.x1 - D.x0, hw * 2 - 0.2); deck.rotateX(-Math.PI / 2); deck.translate(0, top, 0);
    scaleUV(deck, 1 / ((D.x1 - D.x0) / 5));
    const U = deck.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, U.getX(i), U.getY(i) * (hw * 2) / (D.x1 - D.x0));
    add(deck, deckAsphalt, { cast: false, name: 'deck' });
    // deck markings
    const geos = [];
    const strip = (x0, x1, z0, z1) => { const g = new THREE.PlaneGeometry(x1 - x0, z1 - z0); g.rotateX(-Math.PI / 2); g.translate((x0 + x1) / 2, top + 0.006, (z0 + z1) / 2); geos.push(g); };
    for (const s of [1, -1]) {
      for (const zl of [DECK_LINES.inner, DECK_LINES.outer]) strip(D.x0, D.x1, s * zl - 0.075, s * zl + 0.075);
      for (let x = D.x0; x < D.x1 - 6; x += 15) strip(x, x + 6, s * DECK_LINES.divider - 0.075, s * DECK_LINES.divider + 0.075);
    }
    add(mergeGeometries(geos), roadPaint, { cast: false });
    // parapets (F-shape) with the construction gap on +z
    const pw = D.parapetW, ph = D.parapetH;
    const para = [[0, top], [0, top + 0.08], [0.07, top + 0.33], [0.2, top + ph], [pw, top + ph], [pw, top]];
    const mkPara = (s, x0, x1) => {
      const p = para.map(([d, y]) => [s * (hw - pw + (pw - d)) , y]);
      const g = sweepX(s > 0 ? p : p.slice().reverse(), x0, x1, { uvScale: 2 });
      return g;
    };
    const pg = [mkPara(-1, D.x0, D.x1), mkPara(1, D.x0, D.gap[0]), mkPara(1, D.gap[1], D.x1)];
    // broken ends: caps
    pg.push(capX(para.map(([d, y]) => [hw - d, y]), D.gap[0], 1), capX(para.map(([d, y]) => [hw - d, y]), D.gap[1], -1));
    add(mergeGeometries(pg.map(g => { if (!g.attributes.uv1) g.setAttribute('uv1', g.attributes.uv.clone()); return g; })), concreteRough, { name: 'parapet' });
    // median barrier on the deck
    const mb = [[-0.3, top], [-0.3, top + 0.08], [-0.1, top + 0.82], [0.1, top + 0.82], [0.3, top + 0.08], [0.3, top]];
    add(sweepX(mb, D.x0, D.x1, { uvScale: 2 }), concreteRough);
    // expansion joints (dark steel strips) above each pier
    const jg = [];
    for (let x = D.x0 + PIERS.offset + 6; x < D.x1; x += PIERS.spacing) jg.push(boxUV(0.12, 0.012, hw * 2 - 0.2, x, top + 0.004, 0));
    add(mergeGeometries(jg), steelDark, { cast: false });
    // remains of the removed parapet: rebar stubs on the edge of the gap
    const rebar = [];
    for (let x = D.gap[0] + 0.15; x < D.gap[1]; x += 0.35) {
      const h = 0.12 + Math.abs(Math.sin(x * 7.1)) * 0.22;
      const g = new THREE.CylinderGeometry(0.012, 0.012, h, 5); g.rotateZ((Math.sin(x * 3.3)) * 0.4); g.translate(x, top + h / 2, hw - 0.18 - (Math.sin(x * 5) > 0 ? 0.18 : 0));
      rebar.push(g);
    }
    add(mergeGeometries(rebar), new THREE.MeshStandardMaterial({ color: 0x5a3a28, roughness: 0.8, metalness: 0.4 }));
    // rough broken concrete strip where the parapet stood
    add(boxUV(D.gap[1] - D.gap[0], 0.07, pw, (D.gap[0] + D.gap[1]) / 2, top + 0.035, hw - pw / 2), concreteRough);
  }

  // ---------------- piers: rounded rectangle column flaring into a vase head
  {
    const bot = D.top - D.girderDepth;
    const geo = pierGeometry(PIERS.w / 2, PIERS.d / 2, bot);
    const pierGeos = [], bandGeos = [];
    for (let x = D.x0 + PIERS.offset; x < D.x1; x += PIERS.spacing) {
      pierGeos.push(geo.clone().translate(x, 0, 0));
      bandGeos.push(pierBand(PIERS.w / 2 + 0.03, PIERS.d / 2 + 0.03, 1.6).translate(x, 0.2, 0));
    }
    add(mergeGeometries(pierGeos), concrete, { name: 'piers' });
    add(mergeGeometries(bandGeos), hazard);
    // drain pipes
    const pipes = [];
    for (let x = D.x0 + PIERS.offset; x < D.x1; x += PIERS.spacing) {
      const g = new THREE.CylinderGeometry(0.08, 0.08, bot - 0.3, 10); g.translate(x + PIERS.w / 2 + 0.1, (bot - 0.3) / 2 + 0.2, 0.7); pipes.push(g);
    }
    add(mergeGeometries(pipes), new THREE.MeshStandardMaterial({ color: 0x2b2e31, roughness: 0.6 }));
    // height-limit signs on the piers next to the intersection
    const hs = new THREE.MeshStandardMaterial({ map: T.heightSign(), roughness: 0.5 });
    for (const x of [D.x0 + PIERS.offset + PIERS.spacing * 7, D.x0 + PIERS.offset + PIERS.spacing * 8]) {
      for (const s of [1, -1]) { const m = add(new THREE.CircleGeometry(0.35, 32), hs); m.position.set(x, 3.2, s * (PIERS.d / 2 + 0.04)); if (s < 0) m.rotation.y = Math.PI; }
    }
  }

  // ---------------- deck lamps and ground street lamps (double arm), signals, signs
  {
    const poleGeos = [], headGeos = [];
    const lampAt = (x, y0, z, height, armDirs) => {
      const pole = new THREE.CylinderGeometry(0.07, 0.12, height, 12); pole.translate(x, y0 + height / 2, z); poleGeos.push(pole);
      for (const d of armDirs) {
        const curve = new THREE.CubicBezierCurve3(new THREE.Vector3(x, y0 + height - 1.6, z), new THREE.Vector3(x, y0 + height + 0.4, z), new THREE.Vector3(x + d[0] * 0.8, y0 + height + 0.6, z + d[1] * 0.8), new THREE.Vector3(x + d[0] * 2.4, y0 + height + 0.25, z + d[1] * 2.4));
        poleGeos.push(new THREE.TubeGeometry(curve, 20, 0.045, 8));
        const head = new THREE.BoxGeometry(0.7, 0.12, 0.32); head.rotateY(Math.atan2(-d[1], d[0])); head.translate(x + d[0] * 2.65, y0 + height + 0.2, z + d[1] * 2.65);
        headGeos.push(head);
      }
    };
    for (let x = D.x0 + 22; x < D.x1; x += PIERS.spacing) lampAt(x, D.top + 0.82, 0, 9, [[0, 1], [0, -1]]);
    for (const [a, b] of xRanges) {
      for (let x = Math.ceil(a / 34) * 34 + 9; x < b - 3; x += 34) {
        lampAt(x, G.curbH, G.sidewalk[0] + 0.7, 10.5, [[0, -1], [0.45, 0.89]]);
        lampAt(x + 17, G.curbH, G.sidewalkN[1] - 0.7, 10.5, [[0, 1], [0.45, -0.89]]);
      }
    }
    add(mergeGeometries(poleGeos), steelGrey);
    add(mergeGeometries(headGeos), new THREE.MeshStandardMaterial({ color: 0x6d7378, metalness: 0.5, roughness: 0.4, emissive: 0x111111 }));

    // traffic signal gantry at the near corner of the intersection
    const sx = cx0 - 1.2, sz = G.sidewalk[0] + 0.6;
    const gant = [];
    gant.push(new THREE.CylinderGeometry(0.16, 0.2, 7.2, 14).translate(sx, 3.6 + G.curbH, sz));
    gant.push(new THREE.CylinderGeometry(0.09, 0.12, 14, 10).rotateX(Math.PI / 2).translate(sx, 6.8, sz - 7));
    add(mergeGeometries(gant), steelDark);
    const sigBox = new THREE.MeshStandardMaterial({ color: 0x15171a, roughness: 0.5 });
    const red = new THREE.MeshStandardMaterial({ color: 0x330000, emissive: 0xff2010, emissiveIntensity: 3 });
    const off = new THREE.MeshStandardMaterial({ color: 0x1b1d1f, roughness: 0.3 });
    for (const dz of [-4, -8, -12]) {
      const b = add(new THREE.BoxGeometry(0.35, 1.1, 0.35), sigBox); b.position.set(sx + 0.1, 6.15, sz + dz);
      [0.33, 0, -0.33].forEach((dy, i) => { const l = add(new THREE.CircleGeometry(0.12, 20), i === 0 ? red : off, { cast: false }); l.position.set(sx + 0.29, 6.15 + dy, sz + dz); l.rotation.y = Math.PI / 2; });
    }
    const guide = add(new THREE.PlaneGeometry(4.2, 2.1), new THREE.MeshStandardMaterial({ map: T.guideSign(), roughness: 0.45, side: THREE.DoubleSide }));
    guide.position.set(sx - 0.15, 6.6, sz - 15.5); guide.rotation.y = -Math.PI / 2;
    // stop sign + convex mirror on the far corner
    const stop = add(new THREE.CircleGeometry(0.45, 8), new THREE.MeshStandardMaterial({ map: T.stopSign(), roughness: 0.4 }));
    stop.position.set(cx0 - 1.1, 2.6, G.sidewalk[0] + 0.5); stop.rotation.y = -Math.PI / 2;
    add(new THREE.CylinderGeometry(0.04, 0.04, 2.6, 8).translate(cx0 - 1.15, 1.3 + G.curbH, G.sidewalk[0] + 0.5), steelGrey);
    const mirrorPole = add(new THREE.CylinderGeometry(0.045, 0.045, 3.2, 8).translate(cx0 - 1.2, 1.6 + G.curbH, G.sidewalk[1] - 0.5), new THREE.MeshStandardMaterial({ map: T.redWhite(), roughness: 0.5 }));
    mirrorPole.material.map.repeat.set(1, 4);
    const mir = add(new THREE.SphereGeometry(0.42, 24, 12, 0, Math.PI * 2, 0, 0.5), new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 1, roughness: 0.05 }));
    mir.position.set(cx0 - 1.0, 3.1, G.sidewalk[1] - 0.5); mir.rotation.z = Math.PI / 2;
  }

  // ---------------- pedestrian railings along the kerbs
  {
    const posts = [], rails = [];
    for (const [a, b] of xRanges) {
      for (const z of [G.sidewalk[0] + 0.32, G.sidewalkN[1] - 0.32]) {
        for (let x = a + 1; x < b - 1; x += 2.0) posts.push(new THREE.CylinderGeometry(0.025, 0.025, 1.0, 6).translate(x, G.curbH + 0.5, z));
        for (const y of [0.25, 0.95]) rails.push(new THREE.CylinderGeometry(0.03, 0.03, b - a - 2, 8).rotateZ(Math.PI / 2).translate((a + b) / 2, G.curbH + y, z));
        for (let x = a + 1; x < b - 1; x += 0.16) posts.push(new THREE.CylinderGeometry(0.009, 0.009, 0.7, 4).translate(x, G.curbH + 0.6, z));
      }
    }
    add(mergeGeometries(posts), railPaint, { cast: true });
    add(mergeGeometries(rails), railPaint, { cast: true });
  }

  // ---------------- hedges (box shrubs) along the green belts
  {
    const hedgeMat = new THREE.MeshStandardMaterial({ map: T.hedgeTexture(), roughness: 0.95, color: 0xbad0a0 });
    hedgeMat.map.repeat.set(1, 1);
    const hg = [];
    for (const [a, b] of xRanges) {
      for (const [z, s] of [[G.greenBelt[0] + 0.9, 1], [G.greenBeltN[1] - 0.9, -1]]) {
        for (let x = a + 1.5; x < b - 3; x += 6.3) {
          const g = new THREE.BoxGeometry(6, 1.15, 1.25, 12, 4, 3);
          const P = g.attributes.position;
          for (let i = 0; i < P.count; i++) {
            const px = P.getX(i), py = P.getY(i), pz = P.getZ(i);
            const k = 1 + 0.05 * Math.sin(px * 3.1 + pz * 5) * Math.cos(py * 4 + px);
            P.setXYZ(i, px, py * k + 0.02 * Math.sin(px * 7), pz * k);
          }
          g.computeVertexNormals();
          scaleUV(g, 1);
          const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, U.getX(i) * 6, U.getY(i) * 1.2);
          g.translate(x + 3, G.curbH + 0.08 + 0.575, z);
          hg.push(g);
        }
      }
      // median shrubs under the viaduct
      for (let x = a + 2; x < b - 2; x += 3.2) {
        if (Math.abs(((x - D.x0 - PIERS.offset) % PIERS.spacing + PIERS.spacing) % PIERS.spacing) < 3) continue;
        const g = new THREE.SphereGeometry(0.7, 12, 8); g.scale(1.6, 0.7, 0.9); g.translate(x, 0.55, 0); hg.push(g);
      }
    }
    for (const g of hg) if (!g.attributes.uv1) g.setAttribute('uv1', g.attributes.uv.clone());
    add(mergeGeometries(hg.map(g => g.index ? g.toNonIndexed() : g)), hedgeMat);
  }

  // ---------------- trees
  const trees = buildTrees(world, { xRanges, G, D });

  // ---------------- skyline in the haze
  {
    const bgeos = [[], [], []];
    let s = 7;
    const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    for (const side of [1, -1]) {
      for (let x = -420; x < 420; x += 18 + r() * 26) {
        const w = 14 + r() * 26, d = 14 + r() * 18, h = 12 + Math.pow(r(), 1.6) * 75;
        const z = side * (92 + r() * 90);
        if (Math.abs(x - (cx0 + cx1) / 2) < 22 && Math.abs(z) < 75) continue; // keep the crossing road clear
        const g = boxUV(w, h, d, x + w / 2, h / 2, z, 1);
        const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, U.getX(i) / 16, U.getY(i) / 36);
        bgeos[Math.floor(r() * 3)].push(g);
      }
    }
    const tones = [[214, 208, 196], [178, 182, 186], [222, 214, 200]];
    bgeos.forEach((list, i) => { if (list.length) add(mergeGeometries(list), new THREE.MeshStandardMaterial({ map: T.facadeTexture(i, tones[i]), roughness: 0.85 }), { cast: false }); });
  }

  // ---------------- construction props around the parapet gap (barriers, cones, warning board)
  buildConstruction(world, D);

  return { group: world, ground: world.userData.ground, materials: { asphalt, deckAsphalt }, trees, ready: Promise.all(pending) };
}

// -------------------------------------------------------------------------------------------------
function roundedRectRing(ax, bz, r, n) {
  // CCW ring around +Y for a rounded rectangle with half extents ax (x) and bz (z)
  const pts = [];
  const per = Math.max(2, Math.floor(n / 4));
  const corners = [[ax - r, bz - r, 0], [-(ax - r), bz - r, Math.PI / 2], [-(ax - r), -(bz - r), Math.PI], [ax - r, -(bz - r), Math.PI * 1.5]];
  for (const [cx, cz, a0] of corners) for (let i = 0; i <= per; i++) { const a = a0 + (i / per) * Math.PI / 2; pts.push([cx + Math.cos(a) * r, cz + Math.sin(a) * r]); }
  return pts;
}
function pierGeometry(ax, bz, topY) {
  const levels = [];
  const flare0 = topY - 3.8;
  for (let i = 0; i <= 28; i++) {
    const y = (i / 28) * topY;
    const t = Math.max(0, (y - flare0) / (topY - flare0));
    const e = Math.pow(t, 2.3);
    levels.push([y, ax + e * 0.25, bz + e * 2.4]);
  }
  const ringN = 32;
  const pos = [], uv = [], idx = [];
  for (let l = 0; l < levels.length; l++) {
    const [y, a, b] = levels[l];
    const ring = roundedRectRing(a, b, Math.min(0.42, a * 0.45), ringN);
    let acc = 0;
    for (let k = 0; k < ring.length; k++) {
      if (k) acc += Math.hypot(ring[k][0] - ring[k - 1][0], ring[k][1] - ring[k - 1][1]);
      pos.push(ring[k][0], y, ring[k][1]); uv.push(acc / 2.5, y / 2.5);
    }
  }
  const R = roundedRectRing(1, 1, 0.4, ringN).length;
  for (let l = 0; l < levels.length - 1; l++) for (let k = 0; k < R - 1; k++) {
    const a = l * R + k, b = a + 1, c = a + R, d = c + 1;
    idx.push(a, c, b, b, c, d);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setAttribute('uv1', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}
function pierBand(ax, bz, h) {
  const ring = roundedRectRing(ax, bz, Math.min(0.42, ax * 0.45), 32);
  const pos = [], uv = [], idx = [];
  let acc = 0;
  for (let k = 0; k < ring.length; k++) {
    if (k) acc += Math.hypot(ring[k][0] - ring[k - 1][0], ring[k][1] - ring[k - 1][1]);
    pos.push(ring[k][0], 0, ring[k][1], ring[k][0], h, ring[k][1]); uv.push(acc / 1.6, 0, acc / 1.6, 1);
  }
  for (let k = 0; k < ring.length - 1; k++) { const a = k * 2; idx.push(a, a + 1, a + 2, a + 2, a + 1, a + 3); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// -------------------------------------------------------------------------------------------------
function buildTrees(world, { xRanges, G, D }) {
  const leafTex = T.leafAtlas();
  const leafMat = new THREE.MeshStandardMaterial({ map: leafTex, alphaTest: 0.45, side: THREE.DoubleSide, roughness: 0.85, color: 0xd8e6c8 });
  leafMat.onBeforeCompile = sh => {
    sh.uniforms.uWind = world.userData.wind = { value: 0 };
    sh.vertexShader = 'uniform float uWind;\n' + sh.vertexShader.replace('#include <begin_vertex>', `#include <begin_vertex>
      #ifdef USE_INSTANCING
        vec3 ip = instanceMatrix[3].xyz;
      #else
        vec3 ip = vec3(0.0);
      #endif
      float sway = sin(uWind * 1.7 + ip.x * 0.3 + position.y * 0.5) * 0.04 + sin(uWind * 4.1 + position.x * 2.0 + ip.z) * 0.015;
      transformed.x += sway * max(position.y - 2.0, 0.0) * 0.25;
      transformed.z += sway * 0.6 * max(position.y - 2.0, 0.0) * 0.25;`);
  };
  const barkMat = new THREE.MeshStandardMaterial({ color: 0x4a3f35, roughness: 0.95 });
  let seed = 3;
  const r = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  const protos = [];
  for (let p = 0; p < 3; p++) {
    const H = 7 + p * 1.6;
    const wood = [], leaves = [];
    const trunkTop = H * 0.42;
    const bend = new THREE.Vector3((r() - 0.5) * 0.4, 0, (r() - 0.5) * 0.4);
    const trunk = new THREE.CatmullRomCurve3([new THREE.Vector3(0, 0, 0), new THREE.Vector3(bend.x * 0.5, trunkTop * 0.5, bend.z * 0.5), new THREE.Vector3(bend.x, trunkTop, bend.z)]);
    wood.push(new THREE.TubeGeometry(trunk, 8, 0.2, 8));
    const crownC = new THREE.Vector3(bend.x, H * 0.66, bend.z);
    const nb = 6 + p;
    for (let b = 0; b < nb; b++) {
      const a = (b / nb) * Math.PI * 2 + r() * 0.5, el = 0.5 + r() * 0.7, len = H * (0.28 + r() * 0.12);
      const s = new THREE.Vector3(bend.x, trunkTop * (0.85 + r() * 0.2), bend.z);
      const e = s.clone().add(new THREE.Vector3(Math.cos(a) * Math.cos(el) * len, Math.sin(el) * len, Math.sin(a) * Math.cos(el) * len));
      const m = s.clone().lerp(e, 0.5).add(new THREE.Vector3(0, len * 0.12, 0));
      wood.push(new THREE.TubeGeometry(new THREE.QuadraticBezierCurve3(s, m, e), 6, 0.09, 6));
    }
    // leaf clusters on a squashed sphere crown
    const clusters = 46 + p * 8;
    for (let c = 0; c < clusters; c++) {
      const u = r(), v = r(), th = 2 * Math.PI * u, ph = Math.acos(2 * v - 1) * 0.85;
      const R = H * 0.33 * (0.75 + r() * 0.3);
      const cc = new THREE.Vector3(Math.sin(ph) * Math.cos(th) * R * 1.1, Math.cos(ph) * R * 0.75, Math.sin(ph) * Math.sin(th) * R * 1.1).add(crownC);
      const cards = 26;
      for (let k = 0; k < cards; k++) {
        const g = new THREE.PlaneGeometry(0.9, 0.9);
        const tile = Math.floor(r() * 16), tu = (tile % 4) / 4, tv = Math.floor(tile / 4) / 4;
        const U = g.attributes.uv; for (let i = 0; i < U.count; i++) U.setXY(i, tu + U.getX(i) / 4, tv + U.getY(i) / 4);
        g.rotateX(r() * Math.PI); g.rotateY(r() * Math.PI * 2); g.rotateZ(r() * Math.PI);
        const off = new THREE.Vector3((r() - 0.5), (r() - 0.5) * 0.8, (r() - 0.5)).multiplyScalar(1.3);
        g.translate(cc.x + off.x, cc.y + off.y, cc.z + off.z);
        // soft crown normals: point away from the crown centre
        const P = g.attributes.position, N = g.attributes.normal;
        for (let i = 0; i < P.count; i++) {
          const n = new THREE.Vector3(P.getX(i) - crownC.x, (P.getY(i) - crownC.y) * 1.3, P.getZ(i) - crownC.z).normalize();
          N.setXYZ(i, n.x, n.y, n.z);
        }
        leaves.push(g);
      }
    }
    protos.push({ wood: mergeGeometries(wood), leaves: mergeGeometries(leaves) });
  }
  // placements
  const spots = [];
  for (const [a, b] of xRanges) {
    for (const [z0, s] of [[G.greenBelt[0] + 3.4, 1], [G.greenBeltN[1] - 3.4, -1]]) {
      for (let x = a + 3; x < b - 3; x += 7.5 + r() * 2) spots.push([x + (r() - 0.5), 0.24, z0 + (r() - 0.5) * 1.2]);
    }
  }
  // a second row deeper in the belts and distant tree lines; none on the crossing road (cars landing in it roll on
  // across z ~ 40: these trees stood in its lanes). Skipped spots (4th entry true) still draw their random numbers, so
  // every other tree keeps its place and species.
  const onCross = x => x > G.cross[0] - 2 && x < G.cross[1] + 2;
  for (let x = -400; x < 400; x += 9 + r() * 6) for (const z of [G.greenBelt[1] + 3 + r() * 6, G.greenBeltN[0] - 3 - r() * 6]) spots.push([x, 0.1, z, onCross(x)]);
  // park belts behind the sidewalks (keeps the skyline soft and far)
  for (let x = -400; x < 400; x += 6 + r() * 8) for (let k = 0; k < 4; k++) {
    if (x > G.cross[0] - 6 && x < G.cross[1] + 6) continue;
    for (const s of [1, -1]) spots.push([x + (r() - 0.5) * 4, 0.05, s > 0 ? G.greenBelt[1] + 12 + k * 9 + r() * 6 : G.greenBeltN[0] - 12 - k * 9 - r() * 6]);
  }
  const perProto = protos.map(() => []);
  for (const s of spots) perProto[Math.floor(r() * protos.length)].push(s);
  const meshes = [];
  protos.forEach((p, i) => {
    const list = perProto[i], n = list.filter(s => !s[3]).length;
    const wood = new THREE.InstancedMesh(p.wood, barkMat, n);
    const leaves = new THREE.InstancedMesh(p.leaves, leafMat, n);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), sc = new THREE.Vector3();
    let k = 0;
    for (const s of list) {
      q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), r() * Math.PI * 2);
      const k2 = 0.85 + r() * 0.35; sc.set(k2, k2 * (0.9 + r() * 0.2), k2);
      if (s[3]) continue; // on the crossing road: draws consumed, no tree
      m.compose(new THREE.Vector3(s[0], s[1], s[2]), q, sc);
      wood.setMatrixAt(k, m); leaves.setMatrixAt(k, m); k++;
    }
    for (const im of [wood, leaves]) { im.castShadow = true; im.receiveShadow = true; im.computeBoundingSphere(); world.add(im); meshes.push(im); }
  });
  return meshes;
}

function buildConstruction(world, D) {
  const orange = new THREE.MeshStandardMaterial({ color: 0xe8561c, roughness: 0.55 });
  const white = new THREE.MeshStandardMaterial({ color: 0xeeeeea, roughness: 0.55 });
  const hw = D.halfWidth, top = D.top;
  // water-filled barriers (水马) pushed aside at both ends of the gap
  const barrier = () => {
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.75, 0.42, 4, 3, 2), orange);
    body.position.y = 0.375;
    const P = body.geometry.attributes.position; for (let i = 0; i < P.count; i++) { const y = P.getY(i); P.setZ(i, P.getZ(i) * (1 - (y + 0.375) * 0.35)); }
    body.geometry.computeVertexNormals();
    const band = new THREE.Mesh(new THREE.BoxGeometry(1.42, 0.12, 0.36), white); band.position.y = 0.55;
    g.add(body, band);
    g.traverse(o => { o.castShadow = o.receiveShadow = true; });
    return g;
  };
  const place = (x, z, yaw, tilt = 0) => { const b = barrier(); b.position.set(x, top, z); b.rotation.set(0, yaw, tilt); world.add(b); };
  place(D.gap[1] + 0.9, hw - 0.6, 0.15); place(D.gap[1] + 2.4, hw - 0.7, -0.08);
  place(D.gap[0] - 0.9, hw - 0.7, 0.4); place(D.gap[0] + 0.6, hw - 1.9, 1.2, 0);
  // cones taper upstream (car comes from +x)
  const coneGeo = new THREE.ConeGeometry(0.17, 0.7, 20); coneGeo.translate(0, 0.35, 0);
  const coneMat = new THREE.MeshStandardMaterial({ color: 0xff5a1f, roughness: 0.5 });
  const cones = new THREE.InstancedMesh(coneGeo, coneMat, 9);
  const m = new THREE.Matrix4();
  for (let i = 0; i < 9; i++) {
    const x = D.gap[1] + 4 + i * 4.5, z = hw - 0.9 - Math.min(i, 6) * 0.45;
    m.makeTranslation(x, top, z); cones.setMatrixAt(i, m);
  }
  cones.castShadow = true; world.add(cones);
  const sign = new THREE.Mesh(new THREE.PlaneGeometry(0.9, 1.35), new THREE.MeshStandardMaterial({ map: T.constructionSign(), roughness: 0.5, side: THREE.DoubleSide }));
  sign.position.set(D.gap[1] + 30, top + 1.1, hw - 1.3); sign.rotation.y = Math.PI / 2 - 0.2; sign.castShadow = true;
  world.add(sign);
  const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.6, 6), white); leg.position.set(D.gap[1] + 30, top + 0.3, hw - 1.3); world.add(leg);
}
