// The cold-gas kit in the car body frame, built only from spec/propulsion.json: ten thruster modules at their
// mounts, the six-bottle COPV bank, the manifold/regulator block with its gauge, high-pressure lines from the
// bottle necks and one braided feed line per nozzle, routed around the bottle bank. Feed lines can be drawn
// on (grow) and carry light pulses; bottles carry a pressurisation front. The program's choreography poses
// all of it per film time; this file only builds and exposes the handles.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { PROPULSION } from '../../core/spec.js';
import { NozzleModule, nozzleDims, buildGeometry, makeMaterials, braided } from './nozzle.js';

const v3 = a => new THREE.Vector3(a[0], a[1], a[2]);

/** Axis-aligned box around the bottle bank (spec bottles: centre, axis, length, diameter). */
export function bankBox(bottles = PROPULSION.tank.bottles) {
  const b = new THREE.Box3();
  for (const c of bottles) {
    // cylinder extent per axis: h |a_k| + r sqrt(1 - a_k^2)
    const ax = v3(c.axis).normalize(), C = v3(c.center), r = c.diameter / 2, h = c.length / 2;
    const e = new THREE.Vector3(...['x', 'y', 'z'].map(k => h * Math.abs(ax[k]) + r * Math.sqrt(Math.max(0, 1 - ax[k] * ax[k]))));
    b.expandByPoint(C.clone().add(e)); b.expandByPoint(C.clone().sub(e));
  }
  return b;
}

/** Segment-vs-box test (slab method). */
function segHitsBox(a, b, box) {
  let t0 = 0, t1 = 1;
  for (const k of ['x', 'y', 'z']) {
    const d = b[k] - a[k];
    if (Math.abs(d) < 1e-9) { if (a[k] < box.min[k] || a[k] > box.max[k]) return false; continue; }
    let u0 = (box.min[k] - a[k]) / d, u1 = (box.max[k] - a[k]) / d;
    if (u0 > u1) [u0, u1] = [u1, u0];
    t0 = Math.max(t0, u0); t1 = Math.min(t1, u1);
    if (t0 > t1) return false;
  }
  return true;
}

// ------------------------------------------------------------------------------------------- materials
function carbonMaterial() {
  // carbon-fibre overwrap: 2x2 twill from the lathe uv, glossy resin clear coat; uFill/uGlow = pressurisation
  const m = new THREE.MeshPhysicalMaterial({ color: 0x2e3338, roughness: 0.36, metalness: 0.25, clearcoat: 1.0, clearcoatRoughness: 0.14 });
  m.userData.u = { uFill: { value: 0 }, uGlow: { value: 0 }, uPulse: { value: -1 } };
  m.onBeforeCompile = sh => {
    Object.assign(sh.uniforms, m.userData.u);
    sh.vertexShader = 'varying vec2 vCf;\n' + sh.vertexShader.replace('#include <uv_vertex>', '#include <uv_vertex>\n  vCf = uv;');
    sh.fragmentShader = 'uniform float uFill, uGlow, uPulse; varying vec2 vCf;\n' + sh.fragmentShader
      .replace('#include <color_fragment>', /* glsl */`#include <color_fragment>
      vec2 tw = vec2(vCf.x * 72.0, vCf.y * 150.0);
      float tile = mod(floor(tw.x) + floor(tw.y), 4.0);
      float dir = step(2.0, tile);
      float f = dir > 0.5 ? fract(tw.x) : fract(tw.y);
      float strand = 0.75 + 0.5 * sin(f * 3.1416);
      float aa = clamp(1.0 - max(fwidth(tw.x), fwidth(tw.y)) * 0.8, 0.0, 1.0);
      diffuseColor.rgb *= mix(1.0, mix(0.7, 1.25, dir) * strand, aa);`)
      .replace('#include <emissivemap_fragment>', /* glsl */`#include <emissivemap_fragment>
      {
        // filled part: faint cool glow; the front itself: a bright band moving along the bottle
        float front = exp(-pow((vCf.y - uFill) / 0.012, 2.0));
        float filled = step(vCf.y, uFill);
        float pulse = uPulse >= 0.0 ? exp(-pow((vCf.y - uPulse) / 0.04, 2.0)) : 0.0;
        totalEmissiveRadiance += vec3(0.35, 0.75, 1.0) * uGlow * (filled * 0.06 + front * 0.55) + vec3(0.58, 0.83, 0.72) * pulse * 0.6;
      }`);
  };
  m.customProgramCacheKey = () => 'copv';
  return m;
}

/** Additive light-pulse sleeve over a tube: a Gaussian band at uP (0..1 along the tube) plus a growth tip. */
function pulseMaterial(color = 0x9fe8d0) {
  return new THREE.ShaderMaterial({
    uniforms: { uColor: { value: new THREE.Color(color) }, uP: { value: -1 }, uW: { value: 0.05 }, uAmt: { value: 0 }, uGrow: { value: 1 } },
    vertexShader: 'varying vec2 vUv; varying vec3 vN; varying vec3 vV; void main() { vUv = uv; vec4 w = modelMatrix * vec4(position, 1.0); vN = normalize(mat3(modelMatrix) * normal); vV = cameraPosition - w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }',
    fragmentShader: /* glsl */`uniform vec3 uColor; uniform float uP, uW, uAmt, uGrow; varying vec2 vUv; varying vec3 vN; varying vec3 vV;
      void main() {
        if (vUv.x > uGrow) discard;
        float band = exp(-pow((vUv.x - uP) / uW, 2.0));
        float tip = uGrow < 1.0 ? exp(-pow((vUv.x - uGrow) / 0.006, 2.0)) * 0.8 : 0.0;
        float rim = 0.35 + 0.65 * pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 1.5);
        gl_FragColor = vec4(uColor * (band + tip) * uAmt * rim * 3.0, 1.0);
      }`,
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false,
  });
}

// ------------------------------------------------------------------------------------------- kit
export class Kit {
  /**
   * @param {{heroId: string}} opts nozzle id that gets the hero treatment (clipped cutaway materials, real coil turns)
   */
  constructor({ heroId }) {
    const P = PROPULSION;
    this.group = new THREE.Group(); this.group.name = 'coldgas-kit';
    this.manifold = v3(P.regulator.manifold);
    this.bank = bankBox();
    this.heroIndex = P.nozzles.findIndex(n => n.id === heroId);
    // ---- hero cutaway: a wedge between the module's +X (valve axis) and +Z, opened by angle alpha
    this.cutPlanes = [new THREE.Plane(), new THREE.Plane()];
    this.cutU = { uCut0: { value: new THREE.Vector4() }, uCut1: { value: new THREE.Vector4() } }; // same planes, for planar caps
    // ---- per-group shared geometry + materials, the hero gets its own
    const byGroup = {};
    this.nozzles = P.nozzles.map((z, i) => {
      const hero = i === this.heroIndex;
      const d = nozzleDims(z);
      const key = hero ? 'hero' : z.group;
      byGroup[key] ||= { G: buildGeometry(d, { hero }), M: makeMaterials(d, hero ? this.cutPlanes : null, hero ? this.cutU : null) };
      const { G, M } = byGroup[key];
      const m = new NozzleModule(z, d, G, M, this.valveDir(z), this.portAim(z, d));
      m.index = i; m.hero = hero;
      this.group.add(m.group);
      return m;
    });
    this.hero = this.nozzles[this.heroIndex];
    this.heroMaterials = byGroup.hero.M;
    this.buildBank();
    this.buildManifold();
    this.buildHoses();
    this.setCut(0);
  }

  /** Valve axis: outboard for rear/bottom nozzles (along the skin, away from the centre line); rearwards along
   *  the sill for side nozzles (their exhaust already points outboard). */
  valveDir(z) {
    const ex = v3(z.exhaust);
    if (Math.abs(ex.x) > 0.7) return new THREE.Vector3(0, 0, -1);
    return new THREE.Vector3(Math.sign(z.mount[0]) || 1, 0, 0);
  }

  /** Point the inlet port at: the manifold, unless the straight line crosses the bottle bank (then down). */
  portAim(z, d) {
    const m = v3(z.mount), here = m.clone().addScaledVector(v3(z.exhaust), d.yPc);
    const box = this.bank.clone().expandByScalar(0.02);
    // stop short of the manifold: it sits right against the bank's front face
    const end = this.manifold.clone().addScaledVector(here.clone().sub(this.manifold).normalize(), 0.06);
    return segHitsBox(here, end, box) ? here.clone().add(new THREE.Vector3(0, -1, 0)) : this.manifold.clone();
  }

  // ------------------------------------------------------------------------------------- COPV bank
  buildBank() {
    const T = PROPULSION.tank;
    this.bottles = [];
    const satin = new THREE.MeshPhysicalMaterial({ color: 0xc4cacf, metalness: 1, roughness: 0.3 });
    const white = new THREE.MeshPhysicalMaterial({ color: 0xc9cecb, metalness: 0, roughness: 0.45 }); // printed label bands
    for (const b of T.bottles) {
      const R = b.diameter / 2, L = b.length, dome = 0.62 * R, cyl = L / 2 - dome;
      // profile along the bottle axis (lathe Y): neck boss, isotensoid-ish elliptic domes, cylinder
      const prof = [];
      const neck = 0.028, boss = 0.045;
      prof.push(new THREE.Vector2(0.0001, -L / 2 - 0.035), new THREE.Vector2(neck, -L / 2 - 0.035), new THREE.Vector2(neck, -L / 2 - 0.005), new THREE.Vector2(boss, -L / 2 + 0.004));
      for (let i = 0; i <= 16; i++) { const a = (i / 16) * Math.PI / 2; const r = Math.max(boss, R * Math.sin(a)); prof.push(new THREE.Vector2(r, -cyl - dome * Math.cos(a))); }
      for (let i = 0; i <= 16; i++) { const a = (i / 16) * Math.PI / 2; const r = Math.max(boss, R * Math.cos(a)); prof.push(new THREE.Vector2(r, cyl + dome * Math.sin(a))); }
      prof.push(new THREE.Vector2(boss, L / 2 - 0.004), new THREE.Vector2(neck, L / 2 + 0.005), new THREE.Vector2(neck, L / 2 + 0.035), new THREE.Vector2(0.0001, L / 2 + 0.035));
      const geo = new THREE.LatheGeometry(prof, 72);
      // LatheGeometry sets uv.y by profile point index (the finely sampled domes would take 80 % of it):
      // re-parameterise by arc length so the weave and the fill front run evenly along the bottle
      const acc = [0];
      for (let j = 1; j < prof.length; j++) acc.push(acc[j - 1] + prof[j].distanceTo(prof[j - 1]));
      const uvA = geo.attributes.uv, np = prof.length;
      for (let v = 0; v < uvA.count; v++) uvA.setY(v, acc[v % np] / acc[np - 1]);
      const mat = carbonMaterial();
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = true; mesh.receiveShadow = true;
      const grp = new THREE.Group();
      grp.add(mesh);
      // label bands near both ends
      for (const s of [-1, 1]) {
        const band = new THREE.Mesh(new THREE.CylinderGeometry(R * 1.004, R * 1.004, 0.03, 72, 1, true), white);
        band.position.y = s * (cyl - 0.05); grp.add(band);
      }
      // align lathe Y with the bottle axis
      grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v3(b.axis).normalize());
      grp.position.copy(v3(b.center));
      this.group.add(grp);
      this.bottles.push({ spec: b, group: grp, mat, R, L, neckEnd: v3(b.center).addScaledVector(v3(b.axis).normalize(), L / 2 + 0.035) });
    }
    // straps and rails
    const box = this.bank;
    const strapR = T.bottles[0].diameter / 2 + 0.006;
    for (const sx of [-0.3, 0.3]) {
      for (const b of T.bottles) {
        const s = new THREE.Mesh(new THREE.TorusGeometry(strapR, 0.006, 8, 64), satin);
        s.rotation.y = Math.PI / 2; s.position.set(sx, b.center[1], b.center[2]); this.group.add(s);
      }
      for (const y of [box.min.y - 0.012, box.max.y + 0.012]) {
        const r = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.02, box.max.z - box.min.z + 0.04), satin);
        r.position.set(sx, y, (box.min.z + box.max.z) / 2); r.castShadow = true; this.group.add(r);
      }
    }
  }

  // ------------------------------------------------------------------------------------- manifold + regulator
  buildManifold() {
    const M = this.manifold;
    const alu = new THREE.MeshPhysicalMaterial({ color: 0xc9cfd4, metalness: 1, roughness: 0.24 });
    const dark = new THREE.MeshPhysicalMaterial({ color: 0x424a52, metalness: 0.7, roughness: 0.28, clearcoat: 0.5 });
    // keep the block clear of the bank face behind it
    const depth = Math.min(0.06, 2 * (M.z - this.bank.max.z) - 0.004);
    this.mSize = new THREE.Vector3(0.30, 0.06, Math.max(0.03, depth));
    const blk = new THREE.Mesh(new RoundedBoxGeometry(this.mSize.x, this.mSize.y, this.mSize.z, 3, 0.006), alu);
    blk.position.copy(M); blk.castShadow = true; blk.receiveShadow = true;
    // regulator: body + spring dome + adjuster on top of the block
    const reg = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.034, 0.04, 48), alu); body.position.y = 0.02;
    const domeM = new THREE.Mesh(new THREE.SphereGeometry(0.034, 48, 16, 0, Math.PI * 2, 0, Math.PI / 2), dark); domeM.position.y = 0.04;
    const knob = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.02, 16), alu); knob.position.y = 0.083;
    reg.add(body, domeM, knob);
    reg.position.set(M.x + 0.07, M.y + this.mSize.y / 2, M.z);
    // gauge: dial face (canvas), needle driven by tank pressure
    const gauge = new THREE.Group();
    const face = new THREE.Mesh(new THREE.CircleGeometry(0.026, 48), new THREE.MeshBasicMaterial({ map: gaugeTexture() }));
    const rim = new THREE.Mesh(new THREE.TorusGeometry(0.027, 0.003, 8, 48), alu);
    const needle = new THREE.Mesh(new THREE.BoxGeometry(0.0018, 0.021, 0.001).translate(0, 0.0085, 0.001), new THREE.MeshBasicMaterial({ color: 0xff7a35 }));
    gauge.add(face, rim, needle);
    gauge.position.set(M.x - 0.07, M.y + this.mSize.y / 2 + 0.032, M.z + 0.002);
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.03, 12), alu); stem.position.set(M.x - 0.07, M.y + this.mSize.y / 2 + 0.006, M.z);
    this.gaugeNeedle = needle;
    this.group.add(blk, reg, gauge, stem);
    this.regulatorIn = new THREE.Vector3(M.x + 0.07 + 0.034, M.y + this.mSize.y / 2 + 0.02, M.z);
    // high-pressure lines: each bottle neck -> header along the bank's side -> regulator inlet
    const hp = new THREE.MeshPhysicalMaterial({ color: 0xd2d7db, metalness: 1, roughness: 0.18 });
    const xH = Math.max(this.bank.max.x, ...this.bottles.map(b => b.neckEnd.x)) + 0.06, yH = (this.bank.min.y + this.bank.max.y) / 2;
    this.hpLines = [];
    const zs = this.bottles.map(b => b.spec.center[2]), zMin = Math.min(...zs), zMax = Math.max(...zs);
    const header = [new THREE.Vector3(xH, yH, zMin - 0.03), new THREE.Vector3(xH, yH, zMax + 0.02), new THREE.Vector3(xH, yH - 0.06, this.bank.max.z + 0.03),
      new THREE.Vector3(xH - 0.05, this.regulatorIn.y, M.z + 0.01), this.regulatorIn.clone().add(new THREE.Vector3(0.03, 0, 0)), this.regulatorIn.clone()];
    const hc = new THREE.CatmullRomCurve3(header, false, 'centripetal');
    this.header = { curve: hc, mesh: new THREE.Mesh(new THREE.TubeGeometry(hc, 160, 0.009, 12, false), hp) };
    this.header.pulse = new THREE.Mesh(new THREE.TubeGeometry(hc, 160, 0.0125, 12, false), pulseMaterial(0xa8f0ff));
    this.group.add(this.header.mesh, this.header.pulse);
    for (const b of this.bottles) {
      if (b.neckEnd.x < 0) continue; // necks on the far side are capped (relief valves)
      const a = b.neckEnd, c = new THREE.CatmullRomCurve3([a, a.clone().add(new THREE.Vector3(0.02, 0, 0)), new THREE.Vector3(xH, b.spec.center[1] > yH ? yH + 0.03 : yH - 0.03, a.z), new THREE.Vector3(xH, yH, a.z + 0.01)], false, 'centripetal');
      const mesh = new THREE.Mesh(new THREE.TubeGeometry(c, 40, 0.0065, 10, false), hp);
      const pulse = new THREE.Mesh(new THREE.TubeGeometry(c, 40, 0.009, 10, false), pulseMaterial(0xa8f0ff));
      this.group.add(mesh, pulse);
      this.hpLines.push({ curve: c, mesh, pulse, bottle: b });
    }
    for (const o of [this.header.pulse, ...this.hpLines.map(l => l.pulse)]) { o.userData.noAO = true; o.renderOrder = 2; }
  }

  // ------------------------------------------------------------------------------------- feed lines
  /** Ports on the manifold faces: bottom face for lines arriving from under the bank, front/side faces else. */
  buildHoses() {
    const M = this.manifold, S = this.mSize, box = this.bank.clone().expandByScalar(0.03);
    const yUnder = this.bank.min.y - 0.055;
    const plans = this.nozzles.map(n => {
      const { p, dir } = n.portEnd();
      const A = p.clone().addScaledVector(dir, 0.05);
      const via = [];
      const end = M.clone().addScaledVector(A.clone().sub(M).normalize(), 0.06);
      const under = segHitsBox(A, end, box) || A.z < this.bank.min.z;
      if (under) {
        if (A.y > yUnder) via.push(new THREE.Vector3(A.x, yUnder, A.z));
        via.push(new THREE.Vector3(A.x * 0.5, yUnder, (this.bank.max.z + M.z) / 2 - 0.02));
      }
      return { n, p, dir, A, via, under };
    });
    // assign manifold ports
    const bottom = plans.filter(q => q.under).sort((a, b) => a.A.x - b.A.x);
    bottom.forEach((q, i) => { q.Q = new THREE.Vector3(M.x + (i - (bottom.length - 1) / 2) * (S.x / bottom.length), M.y - S.y / 2, M.z); q.N = new THREE.Vector3(0, -1, 0); });
    const rest = plans.filter(q => !q.under);
    for (const q of rest) {
      const d = q.A.clone().sub(M);
      if (Math.abs(d.x) > Math.abs(d.z) * 1.2) { q.N = new THREE.Vector3(Math.sign(d.x), 0, 0); q.Q = new THREE.Vector3(M.x + Math.sign(d.x) * S.x / 2, M.y, M.z); }
      else { q.N = new THREE.Vector3(0, 0, 1); q.Q = new THREE.Vector3(M.x + Math.sign(d.x) * 0.06, M.y, M.z + S.z / 2); }
    }
    this.hoses = plans.map(q => {
      const r = q.n.d.rHose;
      const pts = [q.p, q.A, ...q.via, q.Q.clone().addScaledVector(q.N, 0.07), q.Q.clone().addScaledVector(q.N, 0.004)];
      const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
      const len = curve.getLength();
      const mat = braided(new THREE.MeshPhysicalMaterial({ color: 0xd5dade, metalness: 1, roughness: 0.48 }), len / (r * 0.9), 9);
      mat.userData.braid.uGrow = { value: 1 };
      const ob = mat.onBeforeCompile;
      mat.onBeforeCompile = sh => { ob(sh); sh.uniforms.uGrow = mat.userData.braid.uGrow; sh.fragmentShader = 'uniform float uGrow;\n' + sh.fragmentShader.replace('#include <clipping_planes_fragment>', '#include <clipping_planes_fragment>\n  if (vBr.x > uGrow) discard;'); };
      const seg = Math.max(60, Math.round(len * 220));
      const mesh = new THREE.Mesh(new THREE.TubeGeometry(curve, seg, r, 20, false), mat);
      mesh.castShadow = true; mesh.receiveShadow = true;
      const pulse = new THREE.Mesh(new THREE.TubeGeometry(curve, seg, r * 1.18, 16, false), pulseMaterial());
      pulse.userData.noAO = true; pulse.renderOrder = 2;
      // fitting at the manifold end
      const fit = new THREE.Mesh(new THREE.CylinderGeometry(r * 1.25, r * 1.25, 0.018, 6), new THREE.MeshPhysicalMaterial({ color: 0xc0c6cb, metalness: 1, roughness: 0.3 }));
      fit.position.copy(q.Q).addScaledVector(q.N, 0.009); fit.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), q.N);
      this.group.add(mesh, pulse, fit);
      return { nozzle: q.n, curve, len, mesh, mat, pulse, fit };
    });
  }

  // ------------------------------------------------------------------------------------- handles
  /** Hose i: drawn fraction (from the nozzle port towards the manifold) and pulse position/strength. */
  setHose(i, grow, pulse = -1, amt = 0, w = 0.05) {
    const h = this.hoses[i];
    h.mesh.visible = grow > 0.001; h.fit.visible = grow > 0.98;
    h.mat.userData.braid.uGrow.value = grow;
    const u = h.pulse.material.uniforms;
    u.uGrow.value = grow; u.uP.value = pulse; u.uAmt.value = amt; u.uW.value = w;
    h.pulse.visible = amt > 0.001 && grow > 0.001;
  }
  /** High-pressure header + bottle lines pulse (manifold -> bottles), position 0..1 along the header. */
  setHP(pulse, amt) {
    const set = (m, p) => { const u = m.material.uniforms; u.uP.value = p; u.uAmt.value = amt; m.visible = amt > 0.001; };
    set(this.header.pulse, 1 - pulse);
    for (const l of this.hpLines) set(l.pulse, 1 - (pulse - 0.8) / 0.2);
  }
  /** Pressurisation: fill 0..1 (front along the bottle), glow 0..1, optional pulse along the bottles. */
  setTank(fill, glow, pulse = -1) {
    for (const b of this.bottles) { const u = b.mat.userData.u; u.uFill.value = fill; u.uGlow.value = glow; u.uPulse.value = pulse; }
  }
  /** Gauge needle for a pressure p (Pa): 270° sweep over 0..40 MPa. */
  setGauge(p) { this.gaugeNeedle.rotation.z = 2.356 - (p / 40e6) * 4.712; }

  /** Open the hero cutaway wedge by alpha (rad, 0 = closed). Planes in world space from the hero's world matrix. */
  setCut(alpha, world = null) {
    const h = this.hero;
    const m = world || h.group.matrixWorld;
    const o = new THREE.Vector3().setFromMatrixPosition(m);
    const X = new THREE.Vector3().setFromMatrixColumn(m, 0).normalize(), Z = new THREE.Vector3().setFromMatrixColumn(m, 2).normalize();
    // plane 1 keeps z <= 0, plane 2 keeps angles beyond alpha: removed wedge = theta in (0, alpha) from +X towards +Z
    const n1 = Z.clone().negate();
    const n2 = X.clone().multiplyScalar(-Math.sin(alpha)).addScaledVector(Z, Math.cos(alpha));
    this.cutPlanes[0].setFromNormalAndCoplanarPoint(n1, o);
    this.cutPlanes[1].setFromNormalAndCoplanarPoint(n2, o);
    this.cutPlanes.forEach((p, i) => this.cutU[`uCut${i}`].value.set(p.normal.x, p.normal.y, p.normal.z, p.constant));
    this.cutAlpha = alpha;
  }
}

/** Dial face for the regulator gauge: 0..40 MPa over 270°, red arc above the fill pressure (spec tank p0). */
function gaugeTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#0c0f11'; g.beginPath(); g.arc(128, 128, 128, 0, Math.PI * 2); g.fill();
  g.translate(128, 128);
  const a0 = Math.PI * 0.75, span = Math.PI * 1.5;
  g.strokeStyle = '#ff6a2b'; g.lineWidth = 10; g.beginPath(); g.arc(0, 0, 100, a0 + span * Math.min(1, PROPULSION.tank.p0 / 40e6), a0 + span); g.stroke();
  for (let i = 0; i <= 40; i++) {
    const a = a0 + span * (i / 40), L = i % 5 === 0 ? 22 : 10;
    g.strokeStyle = '#e8eef0'; g.lineWidth = i % 5 === 0 ? 4 : 2;
    g.beginPath(); g.moveTo(Math.cos(a) * 108, Math.sin(a) * 108); g.lineTo(Math.cos(a) * (108 - L), Math.sin(a) * (108 - L)); g.stroke();
    if (i % 10 === 0) { g.fillStyle = '#e8eef0'; g.font = '600 26px -apple-system, Helvetica, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(String(i), Math.cos(a) * 70, Math.sin(a) * 70); }
  }
  g.fillStyle = '#93d3b8'; g.font = '600 22px -apple-system, Helvetica, sans-serif'; g.textAlign = 'center'; g.fillText('MPa', 0, 52);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8;
  return t;
}
