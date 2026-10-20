// GPU-deformed car: every render vertex follows its embedding tetrahedron in the simulated lattice.
// Node positions stream into a float texture each frame; vertex shaders rebuild positions and rotate
// normals with the cofactor of the deformation gradient (averaged per node, so shading stays smooth
// across tetrahedron boundaries instead of breaking into facets). Fragment shaders add damage: crumple noise,
// scraped paint with bare metal, laminated-glass cracks, and an optional strain heat-map.
import * as THREE from 'three';

const TEXW = 1024;
const idx2 = n => [n % TEXW, Math.floor(n / TEXW)];

function makeFloatTex(count, texelsPer = 1) {
  const n = count * texelsPer, h = Math.max(1, Math.ceil(n / TEXW));
  const data = new Float32Array(TEXW * h * 4);
  const t = new THREE.DataTexture(data, TEXW, h, THREE.RGBAFormat, THREE.FloatType);
  t.minFilter = t.magFilter = THREE.NearestFilter;
  t.generateMipmaps = false;
  t.needsUpdate = true;
  return t;
}

// Computes the deformed position, the normal cofactor matrix and the damage/visibility varyings.
const GLSL_LATTICE_FN = /* glsl */`
void lattice(out vec3 defPos, out mat3 cofF, out float hideGroup, out float hideTear) {
  int ti = int(aTet + 0.5);
  vec4 tn = texelFetch(uTetNodes, tix(ti), 0);
  vec3 p0 = nodeP(tn.x), p1 = nodeP(tn.y), p2 = nodeP(tn.z), p3 = nodeP(tn.w);
  float b0 = 1.0 - aBary.x - aBary.y - aBary.z;
  defPos = p0 * b0 + p1 * aBary.x + p2 * aBary.y + p3 * aBary.z;
  // lattice nodes may sink by their contact allowance (node-to-skin distance), which leaves up to ~3 cm
  // of skin under the asphalt where crushed panels rest; flatten it onto the road surface instead
  defPos.y = max(defPos.y, 0.002);
  vec4 r0 = texelFetch(uTetDm, tix(ti * 3), 0);
  cofF = nodeCof(tn.x) * b0 + nodeCof(tn.y) * aBary.x + nodeCof(tn.z) * aBary.y + nodeCof(tn.w) * aBary.z;
  float gT = texelFetch(uGroupTime, tix(int(aGroup + 0.5)), 0).x;
  hideGroup = uTime >= gT ? 1.0 : 0.0;
  hideTear = uTime >= r0.w ? 1.0 : 0.0;
  vDmg = nodeA(tn.x) * b0 + nodeA(tn.y) * aBary.x + nodeA(tn.z) * aBary.y + nodeA(tn.w) * aBary.z;
  vRest = position;
  vGroup = aGroup;
}
`;
const GLSL_DEFORM = /* glsl */`
  vec3 defPos; mat3 cofF; float hideG, hideT;
  lattice(defPos, cofF, hideG, hideT);
  vHide = max(hideG, hideT);
`;

const GLSL_COMMON = /* glsl */`
uniform highp sampler2D uNodePos;
uniform highp sampler2D uTetNodes;
uniform highp sampler2D uTetDm;
uniform highp sampler2D uGroupTime;
uniform highp sampler2D uNodeAttr;
uniform highp sampler2D uNodeCof;
uniform float uTime;
attribute float aTet;
attribute vec3 aBary;
attribute float aGroup;
varying vec3 vRest;
varying vec2 vDmg;
varying float vHide;
varying float vGroup;
ivec2 tix(int i) { return ivec2(i % ${TEXW}, i / ${TEXW}); }
vec3 nodeP(float n) { return texelFetch(uNodePos, tix(int(n + 0.5)), 0).xyz; }
vec2 nodeA(float n) { return texelFetch(uNodeAttr, tix(int(n + 0.5)), 0).xy; }
mat3 nodeCof(float n) { int i = int(n + 0.5) * 3; return mat3(texelFetch(uNodeCof, tix(i), 0).xyz, texelFetch(uNodeCof, tix(i + 1), 0).xyz, texelFetch(uNodeCof, tix(i + 2), 0).xyz); }
` + GLSL_LATTICE_FN;

const GLSL_FRAG_COMMON = /* glsl */`
varying vec3 vRest;
varying vec2 vDmg;
varying float vHide;
varying float vGroup;
uniform float uHeat;
uniform float uTime;
uniform vec4 uCrack;      // xyz crack origin (car rest space), w crack start time
uniform float uWindshield;
float h31(vec3 p) { p = fract(p * 0.3183099 + 0.1); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
float vnoise(vec3 x) {
  vec3 i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(h31(i), h31(i + vec3(1,0,0)), f.x), mix(h31(i + vec3(0,1,0)), h31(i + vec3(1,1,0)), f.x), f.y),
             mix(mix(h31(i + vec3(0,0,1)), h31(i + vec3(1,0,1)), f.x), mix(h31(i + vec3(0,1,1)), h31(i + vec3(1,1,1)), f.x), f.y), f.z);
}
float fbm(vec3 p) { float a = 0.5, s = 0.0; for (int i = 0; i < 4; i++) { s += a * vnoise(p); p *= 2.07; a *= 0.5; } return s; }
vec3 heat(float t) { t = clamp(t, 0.0, 1.0); return clamp(vec3(1.5 * t + 0.1, 1.6 * t * t * (1.0 - t) * 3.0 + 0.15 * t, 0.9 * (1.0 - t) * (1.0 - t) + 0.05), 0.0, 1.0); }
// laminated windshield: radial + concentric crack lines around the impact point
float crackMask(vec3 p, vec3 c, float age) {
  vec3 d = p - c;
  float r = length(d);
  float grow = clamp(age * 18.0, 0.0, 1.0);
  float reach = grow * (0.35 + 1.2 * grow);
  if (r > reach) return 0.0;
  float ang = atan(d.y + d.z * 0.3, d.x);
  float spokes = abs(fract(ang * 3.0 + fbm(p * 7.0) * 0.6) - 0.5);
  float radial = smoothstep(0.035, 0.0, spokes * max(r, 0.02) * 9.0);
  float ringN = r * 22.0 + fbm(p * 12.0) * 2.2;
  float ring = smoothstep(0.08, 0.0, abs(fract(ringN) - 0.5) - 0.42) * step(r, reach * 0.8);
  float fine = smoothstep(0.62, 0.66, fbm(p * 60.0)) * 0.6;
  float core = smoothstep(0.09, 0.0, r);
  return clamp(max(max(radial, ring * 0.7), max(fine * smoothstep(reach, 0.0, r), core)), 0.0, 1.0);
}
`;

/**
 * Patch a mesh material (and matching depth materials) for lattice deformation.
 * kind: 'paint' | 'plastic' | 'glass' | 'other'
 */
function patchMaterial(mat, uniforms, kind) {
  mat.onBeforeCompile = shader => {
    Object.assign(shader.uniforms, uniforms);
    shader.vertexShader = GLSL_COMMON + shader.vertexShader
      .replace('#include <beginnormal_vertex>', `${GLSL_DEFORM}\n  vec3 objectNormal = normalize(cofF * normal);\n#ifdef USE_TANGENT\n  vec3 objectTangent = vec3( tangent.xyz );\n#endif`)
      .replace('#include <begin_vertex>', 'vec3 transformed = defPos;')
      .replace('#include <shadowmap_vertex>', SHADOW_VERTEX);
    let fs = GLSL_FRAG_COMMON + shader.fragmentShader;
    fs = fs.replace('#include <clipping_planes_fragment>', '#include <clipping_planes_fragment>\n  if (vHide > 0.02) discard;');
    // damage: crumple + scraped finish
    fs = fs.replace('#include <normal_fragment_maps>', /* glsl */`#include <normal_fragment_maps>
      {
        float dmg = clamp(vDmg.x * 1.6, 0.0, 1.0);
        if (dmg > 0.01) {
          vec3 q = vRest * 34.0;
          float e = 0.02;
          float n0 = fbm(q);
          vec3 g = vec3(fbm(q + vec3(e,0,0)) - n0, fbm(q + vec3(0,e,0)) - n0, fbm(q + vec3(0,0,e)) - n0) / e;
          vec3 gv = (viewMatrix * vec4(g, 0.0)).xyz;
          normal = normalize(normal - gv * 0.05 * dmg);
        }
      }`);
    // crushed regions can tilt the interpolated normal away from the eye; shading such a normal gives
    // grazing-angle mirror reflections (chrome patches). Mirror it back into the visible hemisphere
    // (base and clear-coat normal alike).
    fs = fs.replace('#include <normal_fragment_begin>', /* glsl */`#include <normal_fragment_begin>
      {
        vec3 eyeV = normalize(vViewPosition);
        float nv = dot(normal, eyeV);
        if (nv < 0.0) { normal = normalize(normal - 2.0 * nv * eyeV); nonPerturbedNormal = normal; }
      }`);
    if (kind === 'paint' || kind === 'plastic') {
      fs = fs.replace('#include <roughnessmap_fragment>', /* glsl */`#include <roughnessmap_fragment>
      float scr = clamp(vDmg.y * 1.2, 0.0, 1.0);
      float streak = fbm(vec3(vRest.x * 9.0, vRest.y * 90.0, vRest.z * 9.0) + vRest.yzx * 40.0);
      float scrM = smoothstep(0.45, 0.9, scr * (0.45 + streak));
      ${kind === 'paint'
        ? 'diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.42, 0.42, 0.43) * (0.7 + 0.5 * streak), scrM);\n      roughnessFactor = mix(roughnessFactor, 0.55 + 0.3 * streak, scrM);'
        : 'diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.38, 0.38, 0.39) * (0.8 + 0.4 * streak), scrM * 0.85);\n      roughnessFactor = mix(roughnessFactor, 0.85, scrM);'}
      float dirt = clamp(vDmg.x * 0.8, 0.0, 0.6) * fbm(vRest * 18.0);
      diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.26, 0.24, 0.21), dirt * 0.7);`);
      if (kind === 'paint') fs = fs.replace('#include <metalnessmap_fragment>', '#include <metalnessmap_fragment>\n      metalnessFactor = mix(metalnessFactor, 0.7, scrM);');
      // the clear coat is ground off with the paint: scraped areas are dull bare metal / plastic, not chrome
      fs = fs.replace('#include <lights_physical_fragment>', '#include <lights_physical_fragment>\n#ifdef USE_CLEARCOAT\n      material.clearcoat *= 1.0 - scrM;\n#endif');
    }
    if (kind === 'glass') {
      fs = fs.replace('#include <roughnessmap_fragment>', /* glsl */`#include <roughnessmap_fragment>
      if (abs(vGroup - uWindshield) < 0.5 && uTime >= uCrack.w) {
        float cm = crackMask(vRest, uCrack.xyz, uTime - uCrack.w);
        diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.92, 0.95, 0.97), cm);
        diffuseColor.a = max(diffuseColor.a, cm * 0.9);
        roughnessFactor = mix(roughnessFactor, 0.55, cm);
      }`);
      if (fs.includes('material.transmission = transmission;'))
        fs = fs.replace('material.transmission = transmission;', `material.transmission = transmission;
        if (abs(vGroup - uWindshield) < 0.5 && uTime >= uCrack.w) material.transmission *= 1.0 - 0.85 * crackMask(vRest, uCrack.xyz, uTime - uCrack.w);`);
    }
    fs = fs.replace('#include <tonemapping_fragment>', `#include <tonemapping_fragment>
      if (uHeat > 0.5) gl_FragColor.rgb = mix(gl_FragColor.rgb * 0.25, heat(vDmg.x * 1.2), 0.85);`);
    shader.fragmentShader = fs;
  };
  mat.customProgramCacheKey = () => 'lattice-' + kind;
}

function makeDepthMaterials(uniforms) {
  const make = (Base, opts) => {
    const m = new Base(opts);
    m.onBeforeCompile = shader => {
      Object.assign(shader.uniforms, uniforms);
      shader.vertexShader = GLSL_COMMON + shader.vertexShader
        .replace('#include <begin_vertex>', `${GLSL_DEFORM}\nvec3 transformed = defPos;`);
      shader.fragmentShader = 'varying float vHide;\n' + shader.fragmentShader.replace('void main() {', 'void main() {\n  if (vHide > 0.02) discard;');
    };
    m.customProgramCacheKey = () => 'lattice-depth-' + Base.name;
    return m;
  };
  return {
    depth: make(THREE.MeshDepthMaterial, { depthPacking: THREE.RGBADepthPacking }),
    distance: make(THREE.MeshDistanceMaterial, {}),
  };
}

// Underbody pan: the source model reuses the near-black satin trim, which renders as a flat black hole
// whenever the floor faces the camera. Real floors are dark grey, rough underseal.
const UNDERBODY = /^Underfloor/;
function underbodyMaterial(src) {
  const m = src.clone();
  m.name = 'Underbody • underseal';
  m.color.set(0x45484a); m.roughness = 0.9; m.metalness = 0.0; m.clearcoat = 0; // dusty underseal, ~70 sRGB
  return m;
}

// PBR plausibility for the source trims: base colours like #060808 (albedo 0.2%) and "half metal" blacks
// are darker than any real surface (charcoal/black plastic/rubber sit at ~30-50 sRGB) and render as holes.
// Dielectrics get metalness 0 and an albedo floor; true metals (chrome, aluminium, mirror) are left alone.
const ALBEDO_FLOOR = 0.022; // linear luminance, ~42 sRGB
function physicalAlbedo(mat) {
  if (mat.metalness >= 0.6) return;
  mat.metalness = 0;
  const c = mat.color, l = 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;
  if (l < ALBEDO_FLOOR) {
    if (l < 1e-4) c.setRGB(ALBEDO_FLOOR, ALBEDO_FLOOR, ALBEDO_FLOOR);
    else c.multiplyScalar(ALBEDO_FLOOR / l);
  }
}

// The source model's triangle winding disagrees with its vertex normals on ~25% of the paint (up to 50% of
// some trims). Every material is double-sided, so three.js flips the normal of each back-facing triangle:
// those triangles were shaded with an inward normal and went black from some view angles. The vertex
// normals carry the authored shading, so make each triangle's winding agree with them.
function alignWinding(P, N, index, i0, i1, vo) {
  for (let i = i0; i < i1; i += 3) {
    const a = (index[i] - vo) * 3, b = (index[i + 1] - vo) * 3, c = (index[i + 2] - vo) * 3;
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    const gx = uy * vz - uz * vy, gy = uz * vx - ux * vz, gz = ux * vy - uy * vx;
    if (gx * (N[a] + N[b] + N[c]) + gy * (N[a + 1] + N[b + 1] + N[c + 1]) + gz * (N[a + 2] + N[b + 2] + N[c + 2]) < 0) {
      const t = index[i + 1]; index[i + 1] = index[i + 2]; index[i + 2] = t;
    }
  }
}

// The source normals are also broken along most panel edges (door, bumper, hood and lamp borders): the last
// row of outer-skin vertices carries the normal of the hem behind it, pointing into the car, so a triangle
// interpolates between opposite normals and alignWinding flips it or not depending on which corners won. The
// edge strips then shaded alternately light and dark: sawtooth / dotted lines along every seam. A corner whose
// normal is more than 60 deg off its own (orientation-corrected) face gets a split vertex with the average of
// those faces' normals instead (per vertex and dominant face direction, so the strip stays smooth).
const BAD_CORNER = 0.5; // cos 60 deg
function repairNormals(m) {
  const P = m.pos, N = m.nrm, I = m.index, nv = P.length / 3;
  const fixes = [], acc = new Map();
  for (let i = 0; i < I.length; i += 3) {
    const a = I[i] * 3, b = I[i + 1] * 3, c = I[i + 2] * 3;
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    let gx = uy * vz - uz * vy, gy = uz * vx - ux * vz, gz = ux * vy - uy * vx;
    const l = Math.hypot(gx, gy, gz);
    if (l < 1e-14) continue;
    gx /= l; gy /= l; gz /= l;
    const d0 = gx * N[a] + gy * N[a + 1] + gz * N[a + 2], d1 = gx * N[b] + gy * N[b + 1] + gz * N[b + 2], d2 = gx * N[c] + gy * N[c + 1] + gz * N[c + 2];
    // face orientation from the corners that do lie along the face normal (all three if none does)
    let s = 0, any = false;
    for (const d of [d0, d1, d2]) if (d >= BAD_CORNER || d <= -BAD_CORNER) { s += d; any = true; }
    if (!any) s = d0 + d1 + d2;
    const o = s < 0 ? -1 : 1;
    if (d0 * o >= BAD_CORNER && d1 * o >= BAD_CORNER && d2 * o >= BAD_CORNER) continue;
    gx *= o; gy *= o; gz *= o;
    const ax = Math.abs(gx), ay = Math.abs(gy), az = Math.abs(gz);
    const dir = ax >= ay && ax >= az ? (gx > 0 ? 0 : 1) : ay >= az ? (gy > 0 ? 2 : 3) : (gz > 0 ? 4 : 5);
    [d0, d1, d2].forEach((d, k) => {
      if (d * o >= BAD_CORNER) return;
      const key = I[i + k] * 6 + dir;
      let e = acc.get(key);
      if (!e) acc.set(key, (e = [I[i + k], 0, 0, 0, nv + acc.size]));
      e[1] += gx * l; e[2] += gy * l; e[3] += gz * l; // area weighted
      fixes.push(i + k, key);
    });
  }
  if (!fixes.length) return m;
  const n2 = nv + acc.size;
  const pos = new Float32Array(n2 * 3), nrm = new Float32Array(n2 * 3), aTet = new Float32Array(n2), aBary = new Float32Array(n2 * 3);
  pos.set(P); nrm.set(N); aTet.set(m.aTet); aBary.set(m.aBary);
  for (const [src, x, y, z, v] of acc.values()) {
    const l = Math.hypot(x, y, z) || 1;
    for (let k = 0; k < 3; k++) { pos[v * 3 + k] = P[src * 3 + k]; aBary[v * 3 + k] = m.aBary[src * 3 + k]; }
    nrm[v * 3] = x / l; nrm[v * 3 + 1] = y / l; nrm[v * 3 + 2] = z / l;
    aTet[v] = m.aTet[src];
  }
  const index = Uint32Array.from(I);
  for (let f = 0; f < fixes.length; f += 2) index[fixes[f]] = acc.get(fixes[f + 1])[4];
  return { pos, nrm, aTet, aBary, index };
}

// Shadow lookups are offset along the vertex normal (normalBias) to avoid acne. Many panels' normals point
// into the car (their shading is fine: every material is double-sided), which pushed the lookup inside the
// body: sunlit hatch, bumpers, hood and roof shadowed themselves in fine diagonal stripes. Offset towards the
// sun instead (minus the depth gradient of the shadow matrix): a sunlit side is always the one facing it.
const SHADOW_VERTEX = THREE.ShaderChunk.shadowmap_vertex.replace(
  'shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias',
  'shadowWorldNormal * ( dot( shadowWorldNormal, vec3( directionalShadowMatrix[ i ][ 0 ][ 2 ], directionalShadowMatrix[ i ][ 1 ][ 2 ], directionalShadowMatrix[ i ][ 2 ][ 2 ] ) ) > 0.0 ? - 1.0 : 1.0 ) * directionalLightShadows[ i ].shadowNormalBias');
if (SHADOW_VERTEX === THREE.ShaderChunk.shadowmap_vertex) console.warn('carRender: shadowmap_vertex changed, sun-facing normal bias not applied');

function materialKind(mat) {
  const n = mat.name || '';
  if (/^Glass|clear cover|optical clear|Optics/.test(n) || mat.transmission > 0.3) return 'glass';
  if (/^Paint/.test(n)) return 'paint';
  if (/Fascia|Trim|Rubber|Headlamp • black|Wheel/.test(n)) return 'plastic';
  return 'other';
}

export class CarRenderer {
  /** @param car output of buildCar (with render meshes) */
  constructor(car, { envMap } = {}) {
    this.car = car;
    const N = car.nodes.count, RT = car.render.tetCount, G = car.groups.length;
    this.nodeTex = makeFloatTex(N);
    this.tetTex = makeFloatTex(RT);
    this.dmTex = makeFloatTex(RT, 3);
    this.groupTex = makeFloatTex(G);
    this.attrTex = makeFloatTex(N);
    this.cofTex = makeFloatTex(N, 3);
    this._cofCount = new Float32Array(N);
    this._tearTime = null;
    // static tet data
    const tn = car.render.tetNodes, dm = car.render.tetDmInv;
    const T = this.tetTex.image.data, D = this.dmTex.image.data;
    for (let r = 0; r < RT; r++) {
      T.set([tn[r * 4], tn[r * 4 + 1], tn[r * 4 + 2], tn[r * 4 + 3]], r * 4);
      for (let k = 0; k < 3; k++) D.set([dm[r * 9 + k * 3], dm[r * 9 + k * 3 + 1], dm[r * 9 + k * 3 + 2], k === 0 ? 1e9 : 0], (r * 3 + k) * 4);
    }
    this.groupTex.image.data.fill(1e9);
    this.windshieldGroup = car.groups.findIndex(g => g.name === 'windshield');
    this.uniforms = {
      uNodePos: { value: this.nodeTex }, uTetNodes: { value: this.tetTex }, uTetDm: { value: this.dmTex },
      uGroupTime: { value: this.groupTex }, uNodeAttr: { value: this.attrTex }, uNodeCof: { value: this.cofTex },
      uTime: { value: 0 }, uHeat: { value: 0 }, uCrack: { value: new THREE.Vector4(0, 0, 0, 1e9) },
      uWindshield: { value: this.windshieldGroup },
    };
    this.group = new THREE.Group();
    this.group.name = 'deformable-car';
    const depth = makeDepthMaterials(this.uniforms);
    // merge render meshes by material
    const byMat = new Map();
    let under = null;
    for (const m of car.render.meshes) {
      let mat = m.material;
      if (UNDERBODY.test(m.name)) mat = under ||= underbodyMaterial(mat);
      if (!byMat.has(mat)) byMat.set(mat, []);
      byMat.get(mat).push(m);
    }
    this.materials = [];
    for (const [srcMat, raw] of byMat) {
      const list = raw.map(m => ({ ...repairNormals(m), group: m.group }));
      let nv = 0, ni = 0;
      for (const m of list) { nv += m.pos.length / 3; ni += m.index.length; }
      const pos = new Float32Array(nv * 3), nrm = new Float32Array(nv * 3), tet = new Float32Array(nv), bar = new Float32Array(nv * 3), grp = new Float32Array(nv);
      const index = new Uint32Array(ni);
      let vo = 0, io = 0;
      for (const m of list) {
        const n = m.pos.length / 3;
        pos.set(m.pos, vo * 3); nrm.set(m.nrm, vo * 3); tet.set(m.aTet, vo); bar.set(m.aBary, vo * 3); grp.fill(m.group, vo, vo + n);
        for (let i = 0; i < m.index.length; i++) index[io + i] = m.index[i] + vo;
        alignWinding(m.pos, m.nrm, index, io, io + m.index.length, vo);
        vo += n; io += m.index.length;
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      g.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
      g.setAttribute('aTet', new THREE.BufferAttribute(tet, 1));
      g.setAttribute('aBary', new THREE.BufferAttribute(bar, 3));
      g.setAttribute('aGroup', new THREE.BufferAttribute(grp, 1));
      g.setIndex(new THREE.BufferAttribute(index, 1));
      const mat = srcMat.clone();
      const kind = materialKind(srcMat);
      if (envMap) mat.envMap = envMap;
      // solid (non-metallic) paint under a glossy clear coat: the source 0.22 metalness and sharp base
      // specular made the panels mirror the sky and read as tinted glass
      if (kind === 'paint') { mat.metalness = 0.0; mat.roughness = 0.38; mat.clearcoat = 1.0; mat.clearcoatRoughness = 0.06; }
      else if (kind !== 'glass') physicalAlbedo(mat);
      mat.side = kind === 'glass' ? THREE.DoubleSide : mat.side;
      patchMaterial(mat, this.uniforms, kind);
      this.materials.push(mat);
      const mesh = new THREE.Mesh(g, mat);
      mesh.frustumCulled = false;
      mesh.castShadow = kind !== 'glass';
      mesh.receiveShadow = true;
      mesh.customDepthMaterial = depth.depth;
      mesh.customDistanceMaterial = depth.distance;
      mesh.name = srcMat.name;
      if (kind === 'glass') mesh.userData.noAO = true;
      this.group.add(mesh);
    }
  }

  /**
   * Put the deformed car into the GTAO normal/depth G-buffer. The pass renders the scene with one
   * override material, and three.js does not re-upload a non-ShaderMaterial's uniforms between draws,
   * so a per-mesh "deform" toggle never takes effect: the car stayed at its rest pose in the G-buffer
   * and the AO of whatever was behind it (tree foliage) got printed onto the body. Instead the set is
   * drawn without the car, then the car alone with its own always-deforming normal material into the
   * same target (sharing its depth).
   */
  attachGTAO(gtao) {
    const mat = gtao.normalMaterial.clone();
    // double-sided: some panels (hatch, rear bumper) are wound inward after alignWinding; a front-sided normal
    // pass dropped them from the AO buffer and the AO of whatever lies behind showed through the panel
    mat.side = THREE.DoubleSide;
    mat.onBeforeCompile = shader => {
      Object.assign(shader.uniforms, this.uniforms);
      shader.vertexShader = GLSL_COMMON + shader.vertexShader
        .replace('#include <beginnormal_vertex>', `${GLSL_DEFORM}\n  vec3 objectNormal = normalize(cofF * normal);`)
        .replace('#include <begin_vertex>', 'vec3 transformed = defPos;')
        .replace('#include <project_vertex>', '#include <project_vertex>\n  vEyeDir = -mvPosition.xyz;');
      shader.vertexShader = 'varying vec3 vEyeDir;\n' + shader.vertexShader;
      // same back-facing normal mirror as the shaded materials: crushed (inverted) panels would otherwise
      // face into the body and GTAO darkens them as if fully enclosed
      shader.fragmentShader = 'varying float vHide;\nvarying vec3 vEyeDir;\n' + shader.fragmentShader
        .replace('void main() {', 'void main() {\n  if (vHide > 0.02) discard;')
        .replace('#include <normal_fragment_begin>', `#include <normal_fragment_begin>
  { vec3 eyeV = normalize(vEyeDir); float nv = dot(normal, eyeV); if (nv < 0.0) normal = normalize(normal - 2.0 * nv * eyeV); }`);
    };
    mat.customProgramCacheKey = () => 'lattice-gtao-normal';
    const group = this.group, base = gtao._renderOverride.bind(gtao);
    gtao._renderOverride = (renderer, overrideMaterial, target, clearColor, clearAlpha) => {
      if (overrideMaterial !== gtao.normalMaterial) return base(renderer, overrideMaterial, target, clearColor, clearAlpha);
      const vis = group.visible;
      group.visible = false;
      base(renderer, overrideMaterial, target, clearColor, clearAlpha);
      group.visible = vis;
      if (!vis) return;
      const saved = group.children.map(m => m.material), autoClear = renderer.autoClear;
      group.children.forEach(m => (m.material = mat));
      renderer.setRenderTarget(target);
      renderer.autoClear = false;
      renderer.render(group, gtao.camera);
      renderer.autoClear = autoClear;
      group.children.forEach((m, i) => (m.material = saved[i]));
    };
  }

  /** Reflection environment (PMREM) for every car material. */
  setEnvMap(env) {
    if (this._env === env) return;
    this._env = env;
    for (const m of this.materials) m.envMap = env;
  }

  /** Static per-run data: tear times, glass/lens break times, windshield crack. */
  setRunInfo(meta) {
    const D = this.dmTex.image.data, RT = this.car.render.tetCount;
    const tt = meta?.tearTime;
    this._tearTime = tt || null;
    for (let r = 0; r < RT; r++) D[(r * 3) * 4 + 3] = tt ? tt[r] : 1e9;
    this.dmTex.needsUpdate = true;
    const G = this.groupTex.image.data;
    G.fill(1e9);
    if (meta?.groupTime) meta.groupTime.forEach((t, i) => { G[i * 4] = i === 0 ? 1e9 : t; });
    this.groupTex.needsUpdate = true;
    const ws = meta?.groups?.find(g => g.name === 'windshield');
    if (ws && ws.crackAt) this.uniforms.uCrack.value.set(ws.crackAt[0], ws.crackAt[1], ws.crackAt[2], ws.crackT);
    else this.uniforms.uCrack.value.set(0, 0, 0, 1e9);
  }

  /** Upload node positions (Float32Array N*3) and optional damage frame. */
  update(pos, time, damageFrame) {
    const d = this.nodeTex.image.data, N = this.car.nodes.count;
    for (let n = 0; n < N; n++) { d[n * 4] = pos[n * 3]; d[n * 4 + 1] = pos[n * 3 + 1]; d[n * 4 + 2] = pos[n * 3 + 2]; }
    this.nodeTex.needsUpdate = true;
    this.updateNormals(pos, time);
    this.uniforms.uTime.value = time;
    if (damageFrame !== this._lastDamage) {
      this._lastDamage = damageFrame;
      const a = this.attrTex.image.data;
      if (damageFrame) for (let n = 0; n < N; n++) { a[n * 4] = damageFrame.damage[n] / 255; a[n * 4 + 1] = damageFrame.scrape[n] / 255; }
      else a.fill(0);
      this.attrTex.needsUpdate = true;
    }
  }

  /**
   * Per-node normal transform: average of the deformation-gradient cofactors of the intact render tets
   * around each node. Interpolated with the embedding barycentrics in the vertex shader. Inverted
   * (crushed inside-out) tets have a flipped cofactor; they only count where a node has nothing else.
   */
  updateNormals(pos, time) {
    const RT = this.car.render.tetCount, tn = this.car.render.tetNodes, dm = this.car.render.tetDmInv, tt = this._tearTime;
    const C = this.cofTex.image.data, cnt = this._cofCount;
    const Ci = (this._cofInv ||= new Float32Array(C.length)), cntI = (this._cofInvCount ||= new Float32Array(cnt.length));
    C.fill(0); cnt.fill(0); Ci.fill(0); cntI.fill(0);
    for (let r = 0; r < RT; r++) {
      if (tt && tt[r] <= time) continue;
      const n0 = tn[r * 4] * 3, n1 = tn[r * 4 + 1] * 3, n2 = tn[r * 4 + 2] * 3, n3 = tn[r * 4 + 3] * 3;
      const ax = pos[n1] - pos[n0], ay = pos[n1 + 1] - pos[n0 + 1], az = pos[n1 + 2] - pos[n0 + 2];
      const bx = pos[n2] - pos[n0], by = pos[n2 + 1] - pos[n0 + 1], bz = pos[n2 + 2] - pos[n0 + 2];
      const cx = pos[n3] - pos[n0], cy = pos[n3 + 1] - pos[n0 + 1], cz = pos[n3 + 2] - pos[n0 + 2];
      // F = Ds * Dm^-1 (dm holds the rows of Dm^-1); f_j = column j of F
      const o = r * 9;
      const f0x = ax * dm[o] + bx * dm[o + 3] + cx * dm[o + 6], f0y = ay * dm[o] + by * dm[o + 3] + cy * dm[o + 6], f0z = az * dm[o] + bz * dm[o + 3] + cz * dm[o + 6];
      const f1x = ax * dm[o + 1] + bx * dm[o + 4] + cx * dm[o + 7], f1y = ay * dm[o + 1] + by * dm[o + 4] + cy * dm[o + 7], f1z = az * dm[o + 1] + bz * dm[o + 4] + cz * dm[o + 7];
      const f2x = ax * dm[o + 2] + bx * dm[o + 5] + cx * dm[o + 8], f2y = ay * dm[o + 2] + by * dm[o + 5] + cy * dm[o + 8], f2z = az * dm[o + 2] + bz * dm[o + 5] + cz * dm[o + 8];
      // cofactor columns: f1 x f2, f2 x f0, f0 x f1
      const k0x = f1y * f2z - f1z * f2y, k0y = f1z * f2x - f1x * f2z, k0z = f1x * f2y - f1y * f2x;
      const k1x = f2y * f0z - f2z * f0y, k1y = f2z * f0x - f2x * f0z, k1z = f2x * f0y - f2y * f0x;
      const k2x = f0y * f1z - f0z * f1y, k2y = f0z * f1x - f0x * f1z, k2z = f0x * f1y - f0y * f1x;
      const inv = f0x * k0x + f0y * k0y + f0z * k0z <= 0; // det F
      const A = inv ? Ci : C, An = inv ? cntI : cnt;
      for (let q = 0; q < 4; q++) {
        const n = tn[r * 4 + q], b = n * 12;
        A[b] += k0x; A[b + 1] += k0y; A[b + 2] += k0z;
        A[b + 4] += k1x; A[b + 5] += k1y; A[b + 6] += k1z;
        A[b + 8] += k2x; A[b + 9] += k2y; A[b + 10] += k2z;
        An[n]++;
      }
    }
    for (let n = 0, N = cnt.length; n < N; n++) {
      const b = n * 12;
      if (cnt[n] === 0) {
        if (cntI[n] === 0) { C[b] = C[b + 5] = C[b + 10] = 1; continue; }
        for (let k = 0; k < 11; k++) C[b + k] = Ci[b + k];
        cnt[n] = cntI[n];
      }
      const s = 1 / cnt[n];
      for (let k = 0; k < 11; k++) C[b + k] *= s;
    }
    this.cofTex.needsUpdate = true;
  }

  /** Rest pose (for previews before a run exists). */
  showRest(offset = [0, 0, 0]) {
    const R = this.car.nodes.rest, N = this.car.nodes.count, p = new Float32Array(N * 3);
    for (let n = 0; n < N; n++) { p[n * 3] = R[n * 3] + offset[0]; p[n * 3 + 1] = R[n * 3 + 1] + offset[1]; p[n * 3 + 2] = R[n * 3 + 2] + offset[2]; }
    this.update(p, -1e9, null);
  }
}
