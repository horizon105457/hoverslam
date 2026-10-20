// Procedural cold-gas thruster module, sized from one spec/propulsion.json nozzle entry.
//
// Packaging: an L-shaped module. The nozzle is coaxial with `exhaust`; the proportional solenoid valve
// stands on the side of the plenum, perpendicular to it. A coaxial valve would be ~170 mm deep behind the
// flange and the rear-upper nozzles only have ~120 mm before the COPV bank (spec bottle C2), so the valve
// lies along the skin instead ("every usable void").
//
// Module frame (meshes of the nozzle part): +Y = exhaust (gas leaves along +Y), origin = `mount` (flange seat
// on the skin), +X = valve axis. Valve frame: +Y = valve axis pointing away from the nozzle axis, origin on the
// nozzle axis at the plenum centre, +X = inlet port. Stack, from the exit inwards:
//
//   exit (y = Le) ── 80 % Rao bell (re = rt·√ε, θn 26° → θe 9°) ── throat (yT) ── 30° convergent ── plenum
//   (rc = 2.2 rt) ── closed bulkhead. Side opening (rs = 1.25 rt) ── seat ring: at full lift the poppet
//   curtain area 2π·rs·lift ≈ 1.1 A_t, i.e. the valve really is the "variable effective throat" of the spec
//   model ── valve block with the feed gallery and the inlet port ── solenoid can: coil on a bobbin around a
//   stainless guide tube, armature on the poppet stem, return spring, pole piece ── end cap + connector.
//
// Cutaway: every lathe profile runs counter-clockwise in (r, y) (outward normals) and all part materials are
// double-sided with back faces drawn as a flat hatched section colour, so through a clipping wedge one sees
// solid section faces instead of hollow shells.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

const V2 = (r, y) => new THREE.Vector2(r, y);
const DEG = Math.PI / 180;

/** Lathe solid from a closed CCW profile split into smooth runs (hard edges between runs). */
function latheSolid(runs, seg = 64) {
  const gs = runs.filter(r => r.length > 1).map(r => new THREE.LatheGeometry(r.map(([a, b]) => V2(a, b)), seg));
  const g = mergeGeometries(gs, false);
  gs.forEach(x => x.dispose());
  return g;
}
/** Closed CCW polygon (every edge a hard edge). */
const latheBox = (pts, seg) => latheSolid(pts.map((p, i) => [p, pts[(i + 1) % pts.length]]), seg);

// ------------------------------------------------------------------------------------------- dimensions
/** All module dimensions (m) for one nozzle spec entry. Nozzle-part values in the module frame, valve values
 *  (prefix v) along the valve axis measured from the nozzle axis. */
export function nozzleDims(z) {
  const rt = z.throat_diameter / 2, eps = z.expansion_ratio, re = rt * Math.sqrt(eps), k = rt;
  const Le = Math.hypot(z.exit[0] - z.mount[0], z.exit[1] - z.mount[1], z.exit[2] - z.mount[2]);
  const Ld = 0.8 * (re - rt) / Math.tan(15 * DEG); // 80 % bell of the equivalent 15° cone
  const yT = Le - Ld, yC = yT - 2.48 * k, Lp = 3.6 * k, yN0 = yC - Lp;
  const vA = 2.35 * k, seatH = 0.45 * k, vB = vA + 3.2 * k, vC = vB + 3.4 * k;
  return {
    rt, re, eps, Le, Ld, yT, yC, yN0, yPc: (yN0 + yC) / 2, yBot: yN0 - 1.3 * k,
    rc: 2.2 * k, rh: 3.0 * k, // hub = valve radius: the junction reads as a cross-drilled body
    rs: 1.25 * k, rg: 2.0 * k, rv: 3.0 * k, tf: 0.3 * k,
    // valve axis stations: block [vA, vB] (seat ring [vA, vA + seatH]), can [vB, vC], pole piece [vP, vC]
    vA, seatH, vB, vC, vP: vC - 1.5 * k, lift: 0.45 * k,
    vG: vA + seatH + 1.1 * k,   // inlet port axis (middle of the gallery)
    rPort: 1.1 * k,             // port bore: feed area 1.2 A_t, the line runs at M ≈ 0.57 (must not choke before the throat)
    rHose: 1.3 * k,             // braided hose outer radius
    portEnd: 3.0 * k + 1.7 * k, // radial distance of the hose start (ferrule end)
  };
}

// ------------------------------------------------------------------------------------------- contours
/** Gas-side contour of the nozzle part from the bulkhead to the exit lip: [[r, y], ...], increasing y. */
export function innerContour(d, n = 28) {
  const { rt, re, yT, rc, yN0, Le } = d;
  const R1 = 1.5 * rt, th = 30 * DEG, R2 = 0.382 * rt, tn = 26 * DEG, te = 9 * DEG;
  // start of the upstream throat arc, and where the 30° cone meets the plenum wall (= d.yC)
  const A = [rt + R1 * (1 - Math.cos(th)), yT - R1 * Math.sin(th)];
  const yCorner = A[1] - (rc - A[0]) / Math.tan(th);
  const pts = [[rc, yN0 + 0.3 * rt]];
  // rounded plenum corner (quadratic Bezier wall -> corner -> cone) so the convergent catches a highlight
  const b = 0.35 * rt, P0 = [rc, yCorner - b], C = [rc, yCorner], P1 = [rc - b * Math.sin(th), yCorner + b * Math.cos(th)];
  for (let i = 0; i <= 6; i++) { const s = i / 6, u = 1 - s; pts.push([u * u * P0[0] + 2 * u * s * C[0] + s * s * P1[0], u * u * P0[1] + 2 * u * s * C[1] + s * s * P1[1]]); }
  pts.push(A);
  for (let i = 1; i <= 8; i++) { const p = -th + th * (i / 8); pts.push([rt + R1 * (1 - Math.cos(p)), yT + R1 * Math.sin(p)]); }
  // downstream throat arc (0.382 rt) to θn, then the Rao parabola (quadratic Bezier) to the exit at θe
  for (let i = 1; i <= 6; i++) { const p = tn * (i / 6); pts.push([rt + R2 * (1 - Math.cos(p)), yT + R2 * Math.sin(p)]); }
  const N = pts[pts.length - 1];
  // control point = intersection of the tangents r = N.r + (y - N.y) tan θn and r = re + (y - Le) tan θe
  const yq = (re - N[0] + N[1] * Math.tan(tn) - Le * Math.tan(te)) / (Math.tan(tn) - Math.tan(te));
  const Q = [N[0] + (yq - N[1]) * Math.tan(tn), yq];
  for (let i = 1; i <= n; i++) {
    const s = i / n, a = (1 - s) * (1 - s), b2 = 2 * s * (1 - s), c = s * s;
    pts.push([a * N[0] + b2 * Q[0] + c * re, a * N[1] + b2 * Q[1] + c * Le]);
  }
  return pts;
}

/** Bore radius at axial y (module frame); beyond the exit: free plume boundary (13° half angle). */
export function boreRadius(d, contour, y) {
  if (y <= contour[0][1]) return d.rc;
  for (let i = 1; i < contour.length; i++) {
    const [r1, y1] = contour[i];
    if (y <= y1) { const [r0, y0] = contour[i - 1]; return r0 + (r1 - r0) * (y - y0) / Math.max(1e-9, y1 - y0); }
  }
  return d.re + (y - d.Le) * Math.tan(13 * DEG);
}

// ------------------------------------------------------------------------------------------- materials
const SECTION_FRAG = /* glsl */`
  if (!gl_FrontFacing) {
    // Back face seen through a cut: the visible "cap" is where the view ray leaves the removed wedge
    // (both cut-plane distances < 0). Shade that planar point flat, with 45° hatching in the plane.
    vec3 P = vSecW, Nc = normalize(cameraPosition - vSecW);
  #ifdef SEC_CAP
    vec3 C = cameraPosition, D = P - C;
    float best = -1.0;
    for (int i = 0; i < 2; i++) {
      vec4 pl = i == 0 ? uCut0 : uCut1, ot = i == 0 ? uCut1 : uCut0;
      float dc = dot(pl.xyz, C) + pl.w, dp = dot(pl.xyz, P) + pl.w;
      if (abs(dc - dp) > 1e-7) {
        float t = dc / (dc - dp);
        vec3 X = C + D * t;
        if (t > 0.0 && t <= 1.0 && t > best && dot(ot.xyz, X) + ot.w <= 1e-5) { best = t; Nc = -pl.xyz; }
      }
    }
    if (best > 0.0) P = C + D * best;
  #endif
    vec3 hd = normalize(vec3(0.577) - Nc * dot(vec3(0.577), Nc) + 1e-4);
    float h = abs(fract(dot(P, hd) * uHatch) - 0.5);
    float line = 1.0 - smoothstep(0.10, 0.18, h);
    float lit = 0.62 + 0.38 * max(0.0, dot(Nc, normalize(vec3(0.25, 1.0, 0.35))));
    gl_FragColor = vec4(uSection * lit * (1.0 - 0.14 * line), 1.0);
  }
`;
/** Make a material double-sided with hatched section back faces. cap = {uCut0, uCut1} uniforms of the cut
 *  planes (world normal.xyz, constant) for planar caps; without it the back face itself is hatched. */
export function sectioned(mat, section, hatch = 900, cap = null) {
  mat.side = THREE.DoubleSide;
  mat.userData.section = { uSection: { value: new THREE.Color(section) }, uHatch: { value: hatch }, ...(cap || {}) };
  if (cap) mat.defines = { ...(mat.defines || {}), SEC_CAP: '' };
  const prev = mat.onBeforeCompile;
  mat.onBeforeCompile = (sh, r) => {
    prev?.call(mat, sh, r);
    Object.assign(sh.uniforms, mat.userData.section);
    sh.vertexShader = 'varying vec3 vSecW;\n' + sh.vertexShader.replace('#include <project_vertex>', `#include <project_vertex>
  vSecW = (modelMatrix * vec4(transformed, 1.0)).xyz;`);
    sh.fragmentShader = 'uniform vec3 uSection; uniform float uHatch; uniform vec4 uCut0, uCut1; varying vec3 vSecW;\n' + sh.fragmentShader.replace('#include <dithering_fragment>', '#include <dithering_fragment>\n' + SECTION_FRAG);
  };
  const key = mat.customProgramCacheKey.bind(mat);
  mat.customProgramCacheKey = () => key() + (cap ? '|cap' : '|sec');
  return mat;
}

/** Side hole through a lathe wall along +X at height y (discard inside that cylinder, outside r > xMin). */
function withSideHole(mat, y, r, xMin) {
  const prev = mat.onBeforeCompile;
  mat.onBeforeCompile = (sh, rr) => {
    prev?.call(mat, sh, rr);
    sh.uniforms.uHole = { value: new THREE.Vector3(y, r, xMin) };
    sh.vertexShader = 'varying vec3 vObj;\n' + sh.vertexShader.replace('#include <begin_vertex>', '#include <begin_vertex>\n  vObj = position;');
    sh.fragmentShader = 'uniform vec3 uHole; varying vec3 vObj;\n' + sh.fragmentShader.replace('#include <clipping_planes_fragment>',
      '#include <clipping_planes_fragment>\n  if (vObj.x > uHole.z && length(vObj.yz - vec2(uHole.x, 0.0)) < uHole.y) discard;');
  };
  const key = mat.customProgramCacheKey.bind(mat);
  mat.customProgramCacheKey = () => key() + '|hole';
  return mat;
}

/** Braided stainless: diagonal over-under weave from the tube uv (u along, v around). */
export function braided(mat, along = 260, around = 10) {
  mat.userData.braid = { uBraid: { value: new THREE.Vector2(along, around) } };
  mat.onBeforeCompile = sh => {
    Object.assign(sh.uniforms, mat.userData.braid);
    sh.vertexShader = 'varying vec2 vBr;\n' + sh.vertexShader.replace('#include <uv_vertex>', '#include <uv_vertex>\n  vBr = uv;');
    sh.fragmentShader = 'uniform vec2 uBraid; varying vec2 vBr;\n' + sh.fragmentShader
      .replace('#include <normal_fragment_maps>', /* glsl */`#include <normal_fragment_maps>
      {
        float a = vBr.x * uBraid.x + vBr.y * uBraid.y, b = vBr.x * uBraid.x - vBr.y * uBraid.y;
        float ca = fract(a), cb = fract(b);
        float over = step(0.5, fract(floor(a) * 0.5 + floor(b) * 0.5));
        float strand = mix(sin(ca * 3.1416), sin(cb * 3.1416), over);
        vec3 t = normalize(cross(normal, vec3(0.0, 1.0, 0.0)) + vec3(1e-3));
        normal = normalize(normal + (strand - 0.6) * 0.3 * t);
      }`)
      .replace('#include <roughnessmap_fragment>', '#include <roughnessmap_fragment>\n  roughnessFactor = clamp(roughnessFactor + 0.18 * (fract(vBr.x * uBraid.x * 3.7) - 0.5), 0.05, 1.0);');
  };
  mat.customProgramCacheKey = () => 'braid';
  return mat;
}

// section colours: mint-tinted greys (cut faces read as one family, darker than the polished metal around them)
const SEC = { metal: 0x7d9a8f, dark: 0x4a5a55, moving: 0xc9a27a }; // moving parts (poppet, armature) in warm sand

/** Turned-metal finish: fine concentric tool marks along the lathe axis (local y), so a polished bore shows
 *  its contour instead of reflecting a featureless black studio. */
function turned(mat, pitch) {
  const prev = mat.onBeforeCompile;
  mat.onBeforeCompile = (sh, r) => {
    prev?.call(mat, sh, r);
    sh.uniforms.uTurn = { value: 1 / pitch };
    sh.vertexShader = 'varying float vTurnY;\n' + sh.vertexShader.replace('#include <begin_vertex>', '#include <begin_vertex>\n  vTurnY = position.y;');
    sh.fragmentShader = 'uniform float uTurn; varying float vTurnY;\n' + sh.fragmentShader.replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
  { float ring = fract(vTurnY * uTurn); float aa = clamp(1.0 - fwidth(vTurnY * uTurn) * 1.5, 0.0, 1.0);
    roughnessFactor = clamp(roughnessFactor + aa * 0.12 * (smoothstep(0.0, 0.5, ring) - smoothstep(0.5, 1.0, ring) - 0.5), 0.04, 1.0);
    diffuseColor.rgb *= 1.0 - aa * 0.08 * step(0.85, ring); }`);
  };
  const key = mat.customProgramCacheKey.bind(mat);
  mat.customProgramCacheKey = () => key() + '|turn';
  return mat;
}

/** One material set (per nozzle group, plus clipped copies for the hero). */
export function makeMaterials(d, clip = null, cap = null) {
  const P = o => new THREE.MeshPhysicalMaterial(o);
  const sec = (m, c, h) => {
    if (clip) { m.clippingPlanes = clip; m.clipIntersection = true; m.clipShadows = true; }
    return sectioned(m, c, h, cap);
  };
  const H = 1 / (d.rt * 0.22); // hatch pitch ~0.22 rt
  const M = {
    polished: sec(P({ color: 0xdfe3e6, metalness: 1, roughness: 0.13 }), SEC.metal, H),
    satin: sec(P({ color: 0xc0c6cb, metalness: 1, roughness: 0.3 }), SEC.metal, H),
    anod: sec(P({ color: 0x424a52, metalness: 0.7, roughness: 0.28, clearcoat: 0.5, clearcoatRoughness: 0.2 }), SEC.dark, H), // gunmetal anodising
    iron: sec(P({ color: 0x858c92, metalness: 1, roughness: 0.4 }), SEC.metal, H),
    copper: sec(P({ color: 0xe0915c, metalness: 1, roughness: 0.26 }), 0xd8834f, H * 3),
    spring: sec(P({ color: 0xb8bfc5, metalness: 1, roughness: 0.2 }), SEC.metal, H),
    ptfe: sec(P({ color: 0x3a3c40, metalness: 0, roughness: 0.45 }), SEC.dark, H), // FKM seal (dark: a white disc would blank out the macro shots)
    bobbin: sec(P({ color: 0x383b40, metalness: 0, roughness: 0.5 }), SEC.dark, H),
    bolt: sec(P({ color: 0x60666c, metalness: 1, roughness: 0.28 }), SEC.metal, H),
    blue: sec(P({ color: 0x2a5fc4, metalness: 0.6, roughness: 0.3 }), SEC.dark, H),
  };
  M.poppet = sec(P({ color: 0xdfe3e6, metalness: 1, roughness: 0.13 }), SEC.moving, H);
  M.armature = sec(P({ color: 0x858c92, metalness: 1, roughness: 0.4 }), SEC.moving, H);
  M.bore = withSideHole(sec(P({ color: 0xe8eaec, metalness: 1, roughness: 0.12 }), SEC.metal, H), d.yPc, d.rs * 0.99, d.rc * 0.6);
  M.block = withSideHole(sec(P({ color: 0xc6ccd1, metalness: 1, roughness: 0.26 }), SEC.metal, H), d.vG, d.rPort * 0.98, d.rg * 0.85);
  return M;
}

// ------------------------------------------------------------------------------------------- geometry
/** Geometry set for one nozzle type (shared by all nozzles of a group). Valve parts in the valve frame. */
export function buildGeometry(d, { hero = false } = {}) {
  const { rt: k, re, Le, yT, rc, yN0, yBot, rh, rv, rs, rg, seatH, vA, vB, vC, vP, lift, vG, rPort } = d;
  const G = {};
  const inner = innerContour(d);
  // ---- nozzle part: closed bulkhead + hub + bell; the bell wall thins from 0.45 rt to a 0.07 rt lip
  const wall = y => { const s = Math.min(1, Math.max(0, (y - yT) / (Le - yT))); return k * (0.45 - 0.38 * Math.pow(s, 0.7)); };
  const outerBell = [];
  for (const [r, y] of inner) if (y >= yT + 0.6 * k) outerBell.push([r + wall(y), y]);
  const lip = 0.07 * k;
  outerBell[outerBell.length - 1] = [re + lip, Le];
  const hubTop = yT - 0.4 * k;
  G.nozzle = latheSolid([
    [[0, yBot], [rh - 0.15 * k, yBot]], [[rh - 0.15 * k, yBot], [rh, yBot + 0.15 * k]],
    [[rh, yBot + 0.15 * k], [rh, hubTop]],
    [[rh, hubTop], outerBell[0]],
    outerBell,
    [[re + lip, Le], [re, Le]],
    inner.slice().reverse(),
    [[rc, yN0 + 0.3 * k], [rc - 0.3 * k, yN0]], [[rc - 0.3 * k, yN0], [0, yN0]],
  ], 96);
  const rOut = y => {
    if (y <= hubTop) return rh;
    const b0 = outerBell[0];
    if (y <= b0[1]) return rh + (b0[0] - rh) * (y - hubTop) / (b0[1] - hubTop);
    for (let i = 1; i < outerBell.length; i++) if (outerBell[i][1] >= y) { const a = outerBell[i - 1], b = outerBell[i]; return a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]); }
    return re + lip;
  };
  // ---- mounting flange around the bell neck + 6 bolts
  {
    const fi = Math.max(rOut(0), rOut(d.tf * 0.5), rOut(d.tf)) + 0.03 * k, fo = fi + 1.35 * k, tf = d.tf;
    G.flange = latheSolid([[[fi, 0], [fo - 0.08 * k, 0]], [[fo - 0.08 * k, 0], [fo, 0.08 * k]], [[fo, 0.08 * k], [fo, tf]], [[fo, tf], [fi + 0.1 * k, tf]], [[fi + 0.1 * k, tf], [fi, tf - 0.1 * k]], [[fi, tf - 0.1 * k], [fi, 0]]], 96);
    G.flangeR = [fi, fo]; G.boltR = (fi + fo) / 2;
    const br = 0.17 * k;
    G.bolt = mergeGeometries([
      new THREE.CylinderGeometry(br * 2.1, br * 2.1, 0.06 * k, 24).translate(0, 0.03 * k, 0),
      new THREE.CylinderGeometry(br * 1.9, br * 1.9, 0.28 * k, 6).translate(0, 0.2 * k, 0),
      new THREE.CylinderGeometry(br, br, 1.2 * k, 12).translate(0, -0.6 * k, 0),
    ].map(g => { g.deleteAttribute('uv'); return g.toNonIndexed(); }), false);
  }
  // ---- pressure transducer on the hub, radial along -X (measures the chamber pressure p_c)
  {
    const ty = d.yPc;
    G.transducer = mergeGeometries([
      latheBox([[0, rh - 0.3 * k], [0.62 * k, rh - 0.3 * k], [0.62 * k, rh + 0.35 * k], [0, rh + 0.35 * k]], 6),
      latheBox([[0, rh + 0.35 * k], [0.42 * k, rh + 0.35 * k], [0.42 * k, rh + 2.0 * k], [0, rh + 2.0 * k]], 40),
    ], false).rotateZ(Math.PI / 2).translate(0, ty, 0);
    G.transducerCap = latheBox([[0, rh + 2.0 * k], [0.46 * k, rh + 2.0 * k], [0.46 * k, rh + 2.45 * k], [0, rh + 2.45 * k]], 40).rotateZ(Math.PI / 2).translate(0, ty, 0);
  }
  // ================= valve frame (lathe axis = valve axis, y measured outwards from the nozzle axis)
  // seat ring (hard insert) with a 45° lead-in on the gallery side
  G.seat = latheBox([[rs, vA], [1.85 * k, vA], [1.85 * k, vA + seatH], [rs + 0.14 * k, vA + seatH], [rs, vA + seatH - 0.14 * k]], 64);
  // valve block: seat counterbore, feed gallery, guide bore; its inner face sits on the hub
  G.block = latheBox([
    [1.85 * k, vA], [rv, vA], [rv, vB], [1.0 * k, vB], [1.0 * k, vB - 0.55 * k], [rg, vB - 0.55 * k], [rg, vA + seatH], [1.85 * k, vA + seatH],
  ], 96);
  // inlet port: boss + hex nut + ferrule along +X (built about Y, then turned onto X)
  {
    const boss = latheBox([[rPort, rg - 0.1 * k], [1.4 * k, rg + 0.3 * k], [1.4 * k, rv + 0.5 * k], [rPort, rv + 0.5 * k]], 48);
    const nut = new THREE.CylinderGeometry(1.62 * k, 1.62 * k, 0.45 * k, 6).translate(0, rv + 0.72 * k, 0);
    const fer = latheBox([[rPort, rv + 0.95 * k], [1.4 * k, rv + 0.95 * k], [1.4 * k, rv + 1.7 * k], [rPort, rv + 1.7 * k]], 48);
    G.fitting = mergeGeometries([boss, nut, fer], false).rotateZ(-Math.PI / 2).translate(0, vG, 0);
  }
  // solenoid can + status band
  G.can = latheBox([[rv - 0.22 * k, vB], [rv, vB], [rv, vC], [rv - 0.22 * k, vC]], 96);
  G.band = new THREE.TorusGeometry(rv + 0.02 * k, 0.07 * k, 8, 96).rotateX(Math.PI / 2).translate(0, vB + 0.35 * k, 0);
  // guide tube, pole piece (spring pocket on its inner face), armature, stem + poppet head, PTFE seal
  G.guide = latheBox([[0.9 * k, vB - 0.55 * k], [1.0 * k, vB - 0.55 * k], [1.0 * k, vC], [0.9 * k, vC]], 48);
  G.pole = latheBox([[0, vP + 0.8 * k], [0.55 * k, vP + 0.8 * k], [0.55 * k, vP], [0.86 * k, vP], [0.86 * k, vC], [0, vC]], 48);
  const aOut = vP - lift, aIn = aOut - 2.0 * k; // armature, closed
  G.armature = latheBox([[0, aIn], [0.74 * k, aIn], [0.86 * k, aIn + 0.12 * k], [0.86 * k, aOut], [0.55 * k, aOut], [0.55 * k, aOut - 0.5 * k], [0, aOut - 0.5 * k]], 48);
  const hIn = vA + seatH, hOut = hIn + 0.5 * k; // poppet head, closed (seal face on the seat)
  G.seal = latheBox([[0, hIn], [1.36 * k, hIn], [1.36 * k, hIn + 0.08 * k], [0, hIn + 0.08 * k]], 64);
  G.poppet = latheBox([[0, hIn + 0.08 * k], [1.45 * k, hIn + 0.08 * k], [1.45 * k, hOut - 0.1 * k], [0.22 * k, hOut + 0.15 * k], [0.22 * k, aIn + 0.2 * k], [0, aIn + 0.2 * k]], 64);
  // return spring between the pockets; built at its closed length and scaled with the stroke
  {
    const y0 = aOut - 0.5 * k, y1 = vP + 0.8 * k, L = y1 - y0, R = 0.42 * k, turns = 6.5, pts = [];
    for (let i = 0; i <= 260; i++) { const s = i / 260, a = s * turns * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * R, -s * L, Math.sin(a) * R)); }
    G.spring = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 260, 0.065 * k, 8, false);
    G.springTop = y1; G.springL = L; // the outer end sits on the pole piece; the inner end rides the armature
  }
  // coil: bobbin + windings (hero: real turns layer by layer so the section shows wire cross-sections)
  const cy0 = vB + 0.12 * k, cy1 = vC - 0.45 * k, ci = 1.06 * k, co = rv - 0.26 * k, w = 0.105 * k;
  G.bobbin = latheBox([[ci - 0.04 * k, cy0 - 0.12 * k], [co, cy0 - 0.12 * k], [co, cy0], [ci, cy0], [ci, cy1], [co, cy1], [co, cy1 + 0.12 * k], [ci - 0.04 * k, cy1 + 0.12 * k]], 64);
  if (hero) {
    const nz = Math.floor((cy1 - cy0) / (2 * w)), nr = Math.floor((co - ci) / (2 * w * 0.9)), turns = [];
    for (let j = 0; j < nr; j++) for (let i = 0; i < nz - (j % 2); i++)
      turns.push(new THREE.TorusGeometry(ci + w + j * 2 * w * 0.9, w * 0.96, 10, 48).rotateX(Math.PI / 2).translate(0, cy0 + w + i * 2 * w + (j % 2) * w, 0));
    G.coil = mergeGeometries(turns, false);
    turns.forEach(t => t.dispose());
  } else {
    // rippled sleeve: reads as windings from outside at a fraction of the triangles
    const n = Math.round((cy1 - cy0) / (2 * w)), prof = [];
    for (let i = 0; i <= n * 6; i++) { const s = i / (n * 6); prof.push([co - w * 0.6 * (1 - Math.abs(Math.sin(s * n * Math.PI))), cy0 + (cy1 - cy0) * s]); }
    G.coil = latheSolid([[[ci, cy0], [co, cy0]], prof, [[co, cy1], [ci, cy1]], [[ci, cy1], [ci, cy0]]], 72);
  }
  // end cap + connector
  G.cap = latheSolid([
    [[0.86 * k, vC], [rv, vC]], [[rv, vC], [rv, vC + 0.33 * k]], [[rv, vC + 0.33 * k], [rv - 0.12 * k, vC + 0.45 * k]],
    [[rv - 0.12 * k, vC + 0.45 * k], [0.75 * k, vC + 0.45 * k]], [[0.75 * k, vC + 0.45 * k], [0.86 * k, vC]],
  ], 96);
  G.connector = latheBox([[0.35 * k, vC + 0.45 * k], [0.8 * k, vC + 0.45 * k], [0.8 * k, vC + 1.55 * k], [0.35 * k, vC + 1.55 * k]], 48);
  return G;
}

// ------------------------------------------------------------------------------------------- module
const NOZZLE_PARTS = ['nozzle', 'flange', 'transducer', 'transducerCap'];
const VALVE_PARTS = ['seat', 'block', 'fitting', 'can', 'band', 'guide', 'pole', 'bobbin', 'coil', 'armature', 'poppet', 'seal', 'spring', 'cap', 'connector'];

/**
 * One installed thruster in the car body frame. Every part is a Group (animatable offset) holding one mesh;
 * nozzle parts live in the module frame, valve parts in `this.valve` (valve frame).
 */
export class NozzleModule {
  /**
   * @param z       spec nozzle entry
   * @param d, G, M nozzleDims(z), buildGeometry(d), makeMaterials(d[, clip])
   * @param valveDir THREE.Vector3 (body frame, ⟂ exhaust): valve axis
   * @param portAim  THREE.Vector3 (body frame): point the inlet port should face (the manifold)
   */
  constructor(z, d, G, M, valveDir, portAim) {
    this.spec = z; this.d = d; this.G = G; this.M = M;
    const g = (this.group = new THREE.Group());
    g.name = `nozzle-${z.id}`;
    const Y = new THREE.Vector3(...z.exhaust).normalize();
    const X = valveDir.clone().addScaledVector(Y, -valveDir.dot(Y)).normalize();
    const Z = new THREE.Vector3().crossVectors(X, Y);
    this.basis = new THREE.Matrix4().makeBasis(X, Y, Z);
    this.mount = new THREE.Vector3(...z.mount);
    g.quaternion.setFromRotationMatrix(this.basis);
    g.position.copy(this.mount);
    this.axis = Y; this.X = X; this.Z = Z;
    // valve frame: Y_v = module +X, X_v = port direction (⟂ valve axis, towards the manifold)
    const vc = this.mount.clone().addScaledVector(Y, d.yPc).addScaledVector(X, d.vA + 1.5 * d.rt);
    const aimM = portAim.clone().sub(vc).applyMatrix4(new THREE.Matrix4().copy(this.basis).invert()); // module frame
    const Pm = new THREE.Vector3(0, aimM.y, aimM.z);
    if (Pm.lengthSq() < 1e-8) Pm.set(0, -1, 0);
    Pm.normalize();
    const Vm = new THREE.Vector3(1, 0, 0);
    this.valve = new THREE.Group(); this.valve.name = 'valve';
    this.valve.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(Pm, Vm, new THREE.Vector3().crossVectors(Pm, Vm)));
    this.valve.position.set(0, d.yPc, 0);
    this.portM = Pm; // module frame
    g.add(this.valve);
    // the status band gets its own emissive material (lights up per nozzle)
    this.bandMat = new THREE.MeshStandardMaterial({ color: 0x101413, emissive: new THREE.Color(0x93d3b8), emissiveIntensity: 0, roughness: 0.4, metalness: 0 });
    if (M.polished.clippingPlanes) Object.assign(this.bandMat, { clippingPlanes: M.polished.clippingPlanes, clipIntersection: true });
    this.parts = {};
    const part = (name, geo, mat, parent) => {
      const m = new THREE.Mesh(geo, mat); m.name = name; m.castShadow = true; m.receiveShadow = true;
      const p = new THREE.Group(); p.name = name; p.add(m); parent.add(p);
      return (this.parts[name] = p);
    };
    const mats = { nozzle: M.bore, flange: M.satin, transducer: M.satin, transducerCap: M.blue, seat: M.polished, block: M.block, fitting: M.satin,
      can: M.anod, band: this.bandMat, guide: M.polished, pole: M.iron, bobbin: M.bobbin, coil: M.copper, armature: M.armature, poppet: M.poppet,
      seal: M.ptfe, spring: M.spring, cap: M.anod, connector: M.bobbin };
    for (const n of NOZZLE_PARTS) part(n, G[n], mats[n], g);
    for (const n of VALVE_PARTS) part(n, G[n], mats[n], this.valve);
    this.bolts = [];
    for (let i = 0; i < 6; i++) {
      const a = (i / 6) * Math.PI * 2 + Math.PI / 6;
      const b = part(`bolt${i}`, G.bolt, M.bolt, g);
      b.userData.home = new THREE.Vector3(Math.cos(a) * G.boltR, d.tf, Math.sin(a) * G.boltR);
      b.position.copy(b.userData.home);
      this.bolts.push(b);
    }
    this.parts.spring.children[0].position.y = G.springTop;
    this.setValve(0);
  }

  /** Poppet lift fraction 0..1: armature, stem, poppet and seal move outwards; the spring compresses. */
  setValve(s) {
    const L = this.d.lift * s;
    for (const n of ['armature', 'poppet', 'seal']) this.parts[n].children[0].position.y = L;
    this.parts.spring.children[0].scale.y = (this.G.springL - L) / this.G.springL;
    this.valveS = s;
  }

  /** Module-frame point -> body frame. */
  toBody(v) { return v.clone().applyMatrix4(this.basis).add(this.mount); }
  /** Valve-frame point -> module frame. */
  valveToModule(v) { return v.clone().applyQuaternion(this.valve.quaternion).add(this.valve.position); }
  /** Body-frame hose start (ferrule end) and its outward direction. */
  portEnd() {
    const pv = new THREE.Vector3(this.d.portEnd, this.d.vG, 0);
    return { p: this.toBody(this.valveToModule(pv)), dir: this.portM.clone().applyMatrix4(this.basis).normalize() };
  }
}
