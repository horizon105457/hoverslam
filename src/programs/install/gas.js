// Gas, as pure functions of film time:
//  - HeroFlow: tracer streaks along the hero module's gas path (feed port -> gallery -> poppet curtain ->
//    seat -> plenum -> convergent -> throat -> Rao bell -> plume). Speeds follow quasi-1D isentropic flow:
//    the local Mach number comes from the passage area ratio A/A* (subsonic branch upstream of the throat,
//    supersonic downstream), v = M a0 / sqrt(1 + (γ-1)/2 M²), slowed by a fixed film factor. Tracers are
//    emitted at a rate proportional to the poppet lift at their emission time, so the flow starts and stops
//    with the valve and the front travels down the nozzle.
//  - Jets: the test-fire burst of every nozzle as condensation fog (the exit flow is ~100 K: moist studio air
//    entrained into the jet fogs), with a faint transparent core showing weak shock diamonds (at 6 MPa feed the
//    exit pressure is ~61 kPa: over-expanded, so the jet necks in through weak shocks). Bottom jets spread over the floor.
import * as THREE from 'three';
import { PROPULSION } from '../../core/spec.js';
import { innerContour, boreRadius } from './nozzle.js';

const G = PROPULSION.gas.gamma, Rg = PROPULSION.gas.R;
const A0 = Math.sqrt(G * Rg * PROPULSION.tank.T0); // stagnation speed of sound (isenthalpic regulator)

/** Mach number for area ratio ar = A/A* (>= 1) on the sub- or supersonic branch (bisection, deterministic). */
export function machFromArea(ar, supersonic) {
  ar = Math.max(1, ar);
  const k = (G + 1) / (2 * (G - 1));
  const f = M => (1 / M) * ((2 / (G + 1)) * (1 + 0.5 * (G - 1) * M * M)) ** k;
  let lo = supersonic ? 1 : 1e-4, hi = supersonic ? 50 : 1;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (supersonic ? f(mid) < ar : f(mid) > ar) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}
export const speedOf = M => (M * A0) / Math.sqrt(1 + 0.5 * (G - 1) * M * M);
export const tempOf = M => PROPULSION.tank.T0 / (1 + 0.5 * (G - 1) * M * M);

/** mulberry32 (spec/README.md §7), for reproducible particle seeds */
export function rng(seed) {
  let a = seed >>> 0;
  return () => { a |= 0; a = (a + 0x6d2b79f5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

// streak billboards: quad stretched along the instance velocity, facing the camera
const STREAK_VS = /* glsl */`
attribute vec3 aP; attribute vec3 aD; attribute vec2 aA; // aA = (alpha, colour 0..1)
uniform float uW, uNear;
varying vec2 vQ; varying float vA; varying float vC;
void main() {
  vec3 dir = aD; float L = length(dir);
  vec3 t = L > 1e-7 ? dir / L : vec3(0.0, 1.0, 0.0);
  vec3 toCam = normalize(cameraPosition - aP);
  vec3 side = normalize(cross(t, toCam) + vec3(1e-6));
  float dist = length(cameraPosition - aP);
  vec3 p = aP + dir * position.x + side * position.y * uW;
  vQ = position.xy * 2.0; vC = aA.y;
  // fade tracers right in front of the lens (the camera flies through the stream)
  vA = aA.x * smoothstep(uNear, uNear * 6.0, dist);
  gl_Position = projectionMatrix * viewMatrix * vec4(p, 1.0);
}`;
const STREAK_FS = /* glsl */`
uniform vec3 uCold, uHot, uSlow; varying vec2 vQ; varying float vA; varying float vC;
void main() {
  float a = (1.0 - vQ.y * vQ.y) * (1.0 - smoothstep(0.1, 2.0, -vQ.x));
  vec3 c = vC < 0.5 ? mix(uSlow, uHot, vC * 2.0) : mix(uHot, uCold, (vC - 0.5) * 2.0);
  gl_FragColor = vec4(c * a * vA, 1.0);
}`;

export class HeroFlow {
  /** @param module NozzleModule (hero), count tracers, slow film slow-down factor (1000 = 1 ms of flow per s) */
  constructor(module, { count = 3600, slow = 1000, seed = 11 } = {}) {
    this.m = module; this.count = count; this.slow = slow;
    const d = module.d, k = d.rt;
    this.contour = innerContour(d);
    // ---- stations: [segment, s] -> position(theta, f, psi) in the module frame, plus passage area
    const st = [];
    const push = (n, fn) => { for (let i = 0; i < n; i++) st.push(fn(i / n)); };
    const cur = 2 * Math.PI * d.rs * d.lift; // poppet curtain area at full lift
    // A: port bore, valve frame x_v from the ferrule end to the gallery
    push(10, s => ({ seg: 0, s, area: Math.PI * d.rPort ** 2 }));
    // B: gallery, swirl around the poppet towards the seat
    push(10, s => ({ seg: 1, s, area: Math.PI * (d.rg ** 2 - (1.45 * k) ** 2) }));
    // C: curtain between seal and seat, then the seat orifice
    push(6, s => ({ seg: 2, s, area: s < 0.5 ? cur : Math.PI * d.rs ** 2 }));
    // D: plenum, turning onto the nozzle axis
    push(12, s => ({ seg: 3, s, area: Math.PI * d.rc ** 2 }));
    // E: convergent -> throat -> bell
    const yA = d.yC, yB = d.Le;
    push(60, s => { const y = yA + (yB - yA) * s; const r = boreRadius(d, this.contour, y); return { seg: 4, s, y, area: Math.PI * r * r }; });
    // F: plume, 9 exit radii
    push(18, s => ({ seg: 5, s, y: d.Le + s * 9 * d.re, area: Math.PI * d.re ** 2 * (1 + s * 2) }));
    st.push({ seg: 5, s: 1, y: d.Le + 9 * d.re, area: Math.PI * d.re ** 2 * 3 });
    const At = Math.PI * k * k;
    let tau = 0;
    const ref = new THREE.Vector3(), prev = new THREE.Vector3();
    st.forEach((q, i) => {
      const sup = q.seg === 5 || (q.seg === 4 && q.y > d.yT);
      q.M = q.seg === 5 ? Math.max(1.2, machFromArea(d.eps, true) * (1 - 0.35 * q.s)) : machFromArea(q.area / At, sup);
      q.v = speedOf(q.M) / slow;
      this.pos(q, 0.3, 0.5, 0.3, ref);
      if (i > 0) tau += ref.distanceTo(prev) / (0.5 * (q.v + st[i - 1].v));
      q.tau = tau;
      prev.copy(ref);
    });
    this.st = st; this.Tc = tau;
    // ---- per-tracer seeds
    const r = rng(seed);
    this.seed = Array.from({ length: count }, () => ({ th: r() * Math.PI * 2, f: Math.sqrt(r()) * 0.92, psi: r() * Math.PI * 2, ph: r() * tau, thr: r() * 0.98 + 0.02 }));
    // ---- instanced streaks
    const quad = new THREE.PlaneGeometry(1, 1).translate(-0.5, 0, 0); // x in [-1, 0]: tail behind the head
    const geo = new THREE.InstancedBufferGeometry();
    geo.index = quad.index; geo.attributes.position = quad.attributes.position;
    this.aP = new THREE.InstancedBufferAttribute(new Float32Array(count * 3), 3).setUsage(THREE.DynamicDrawUsage);
    this.aD = new THREE.InstancedBufferAttribute(new Float32Array(count * 3), 3).setUsage(THREE.DynamicDrawUsage);
    this.aA = new THREE.InstancedBufferAttribute(new Float32Array(count * 2), 2).setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('aP', this.aP); geo.setAttribute('aD', this.aD); geo.setAttribute('aA', this.aA);
    geo.instanceCount = count;
    this.mesh = new THREE.Mesh(geo, new THREE.ShaderMaterial({
      uniforms: { uW: { value: 0.035 * k }, uNear: { value: 0.001 }, uCold: { value: new THREE.Color(0.7, 1.5, 2.4) }, uHot: { value: new THREE.Color(1.9, 2.0, 2.2) }, uSlow: { value: new THREE.Color(0.35, 0.6, 1.0) } },
      vertexShader: STREAK_VS, fragmentShader: STREAK_FS, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false,
      side: THREE.DoubleSide, // the billboard's winding depends on the streak direction
    }));
    this.mesh.frustumCulled = false; this.mesh.userData.noAO = true; this.mesh.renderOrder = 3;
    this.mesh.visible = false;
    this._v = new THREE.Vector3(); this._w = new THREE.Vector3(); this._m = new THREE.Matrix4();
  }

  /** Station q at tracer seeds (th, f, psi) -> module-frame position (into out). */
  pos(q, th, f, psi, out) {
    const d = this.m.d, k = d.rt, s = q.s;
    const V = this.m;
    switch (q.seg) {
      case 0: { // port bore: valve frame (x_v, y_v, z_v)
        const x = d.portEnd + 0.3 * k - (d.portEnd + 0.3 * k - d.rg + 0.2 * k) * s;
        const rr = f * d.rPort * 0.8;
        return out.copy(V.valveToModule(this._tmp(x, d.vG + Math.cos(psi) * rr, Math.sin(psi) * rr)));
      }
      case 1: { // gallery: from the port mouth around the poppet to above its rim
        const a = th * s * s * (1 - 0.2 * s) + 0;
        const R = d.rg - 0.15 * k - 0.4 * k * s, y = d.vG + (d.vA + d.seatH + d.lift * 0.5 + 0.15 * k - d.vG) * s;
        return out.copy(V.valveToModule(this._tmp(Math.cos(a) * R, y, Math.sin(a) * R)));
      }
      case 2: { // curtain gap, then down through the seat orifice
        const a = th;
        const R = s < 0.5 ? 1.45 * k - (1.45 * k - d.rs * 0.95) * (s * 2) : d.rs * (0.95 - 0.6 * (s - 0.5) * 2 * (1 - f));
        const y = s < 0.5 ? d.vA + d.seatH + d.lift * 0.5 : d.vA + d.seatH - (d.seatH + 0.5 * k) * (s - 0.5) * 2;
        return out.copy(V.valveToModule(this._tmp(Math.cos(a) * R, y, Math.sin(a) * R)));
      }
      case 3: { // plenum: from the side opening, turn onto the axis and spread to the tracer's (theta, f)
        const e = s * s * (3 - 2 * s);
        const x0 = d.rc * 0.9 * (1 - s) + Math.cos(th) * f * d.rc * 0.85 * e, z0 = Math.sin(th) * f * d.rc * 0.85 * e;
        const y = d.yPc + (Math.cos(psi) * 0.5 * d.rs) * (1 - e) + (d.yC - d.yPc) * e;
        return out.set(x0, y, z0);
      }
      case 4: case 5: {
        const r = boreRadius(this.m.d, this.contour, q.y) * f * (q.seg === 5 ? 1 + q.s * 0.6 : 0.97);
        return out.set(Math.cos(th) * r, q.y, Math.sin(th) * r);
      }
    }
    return out;
  }
  _tmp(x, y, z) { return (this.__t ||= new THREE.Vector3()).set(x, y, z); }

  /**
   * Pose all tracers at film time T. valveAt(t) = poppet lift fraction at film time t. world: module -> world.
   * @returns visible tracer count
   */
  update(T, valveAt, world, { amt = 1, near = 0.001, streak = 0.035 } = {}) {
    const st = this.st, n = st.length, Tc = this.Tc;
    const P = this.aP.array, D = this.aD.array, A = this.aA.array;
    const a = this._v, b = this._w;
    let vis = 0;
    for (let i = 0; i < this.count; i++) {
      const sd = this.seed[i];
      const age = (((T + sd.ph) % Tc) + Tc) % Tc, te = T - age;
      let alpha = 0;
      if (amt > 0 && valveAt(te) > sd.thr) {
        // station by binary search on tau
        let lo = 0, hi = n - 1;
        while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (st[mid].tau <= age) lo = mid; else hi = mid; }
        const q0 = st[lo], q1 = st[hi], u = Math.min(1, Math.max(0, (age - q0.tau) / Math.max(1e-9, q1.tau - q0.tau)));
        this.pos(q0, sd.th, sd.f, sd.psi, a); this.pos(q1, sd.th, sd.f, sd.psi, b);
        const v = q0.v + (q1.v - q0.v) * u, M = q0.M + (q1.M - q0.M) * u;
        b.sub(a); const seg = b.length();
        a.addScaledVector(b, u);
        if (seg > 1e-9) b.multiplyScalar((v * streak) / seg); // streak vector ~ distance covered in `streak` s
        const L = b.length(), Lmax = this.m.d.rt * 1.6;
        if (L > Lmax) b.multiplyScalar(Lmax / L);
        a.applyMatrix4(world); b.transformDirection(world).multiplyScalar(Math.min(L, Lmax));
        // fade in at the port, out along the plume
        const fadeIn = Math.min(1, age / (Tc * 0.03));
        const fadeOut = q0.seg === 5 ? 1 - q0.s : 1;
        alpha = amt * fadeIn * fadeOut * (0.06 + 0.5 * Math.min(1, M * M * 0.5));
        P[i * 3] = a.x; P[i * 3 + 1] = a.y; P[i * 3 + 2] = a.z;
        D[i * 3] = b.x; D[i * 3 + 1] = b.y; D[i * 3 + 2] = b.z;
        A[i * 2 + 1] = Math.min(1, M / 3.2);
        vis++;
      }
      A[i * 2] = alpha;
    }
    this.aP.needsUpdate = this.aD.needsUpdate = this.aA.needsUpdate = true;
    // fade distance: a few throat radii, never less than a few near planes (tracers in front of the lens are huge)
    this.mesh.material.uniforms.uNear.value = Math.max(near * 3, this.m.d.rt * 0.35);
    this.mesh.visible = vis > 0;
    return vis;
  }
}

// ------------------------------------------------------------------------------------------- jets
function puffTexture() {
  // soft fog puff: radial falloff modulated by value noise (deterministic)
  const N = 128, c = document.createElement('canvas'); c.width = c.height = N;
  const g = c.getContext('2d'), img = g.createImageData(N, N), r = rng(5);
  const L = 16, grid = Array.from({ length: L * L }, () => r());
  const vn = (x, y) => { const xi = Math.floor(x), yi = Math.floor(y), fx = x - xi, fy = y - yi, s = t => t * t * (3 - 2 * t);
    const at = (i, j) => grid[((j % L + L) % L) * L + ((i % L + L) % L)];
    return at(xi, yi) * (1 - s(fx)) * (1 - s(fy)) + at(xi + 1, yi) * s(fx) * (1 - s(fy)) + at(xi, yi + 1) * (1 - s(fx)) * s(fy) + at(xi + 1, yi + 1) * s(fx) * s(fy); };
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const u = x / N * 2 - 1, v = y / N * 2 - 1, d = Math.sqrt(u * u + v * v);
    const n = vn(x / 10, y / 10) * 0.6 + vn(x / 5, y / 5) * 0.3 + vn(x / 2.5, y / 2.5) * 0.1;
    const a = Math.max(0, 1 - d) ** 1.6 * (0.55 + 0.9 * n);
    const o = (y * N + x) * 4; img.data[o] = img.data[o + 1] = img.data[o + 2] = 255; img.data[o + 3] = Math.min(255, a * 255);
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.NoColorSpace;
  return t;
}

const PUFF_VS = /* glsl */`
attribute vec4 aP; attribute vec2 aR; // aP = (pos, size), aR = (alpha, rotation)
varying vec2 vUv; varying float vA; varying float vH;
void main() {
  vec3 right = vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
  vec3 up = vec3(viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1]);
  float c = cos(aR.y), s = sin(aR.y);
  vec2 q = vec2(c * position.x - s * position.y, s * position.x + c * position.y) * aP.w;
  vec3 p = aP.xyz + right * q.x + up * q.y;
  vUv = uv; vA = aR.x; vH = position.y;
  gl_Position = projectionMatrix * viewMatrix * vec4(p, 1.0);
}`;
const PUFF_FS = /* glsl */`
uniform sampler2D uTex; uniform vec3 uLit, uShade; varying vec2 vUv; varying float vA; varying float vH;
void main() {
  float a = texture2D(uTex, vUv).a * vA;
  vec3 c = mix(uShade, uLit, clamp(vH + 0.6, 0.0, 1.0));
  gl_FragColor = vec4(c * a, a); // premultiplied
}`;

export class Jets {
  /** @param kit Kit; per = puffs per nozzle */
  constructor(kit, { per = 90, seed = 23 } = {}) {
    this.kit = kit; this.per = per;
    const N = kit.nozzles.length * per;
    const r = rng(seed);
    this.puffs = [];
    for (let i = 0; i < kit.nozzles.length; i++) for (let j = 0; j < per; j++)
      this.puffs.push({ i, te: r(), sp: 0.7 + 0.6 * r(), a: r() * Math.PI * 2, c: Math.sqrt(r()), rot: r() * 6.28, life: 0.45 + 0.75 * r() * r(), n1: r() * 10, n2: r() * 10 });
    const quad = new THREE.PlaneGeometry(1, 1);
    const geo = new THREE.InstancedBufferGeometry();
    geo.index = quad.index; geo.attributes.position = quad.attributes.position; geo.attributes.uv = quad.attributes.uv;
    this.aP = new THREE.InstancedBufferAttribute(new Float32Array(N * 4), 4).setUsage(THREE.DynamicDrawUsage);
    this.aR = new THREE.InstancedBufferAttribute(new Float32Array(N * 2), 2).setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('aP', this.aP); geo.setAttribute('aR', this.aR); geo.instanceCount = N;
    this.mesh = new THREE.Mesh(geo, new THREE.ShaderMaterial({
      uniforms: { uTex: { value: puffTexture() }, uLit: { value: new THREE.Color(0.92, 0.95, 1.0) }, uShade: { value: new THREE.Color(0.42, 0.47, 0.53) } },
      vertexShader: PUFF_VS, fragmentShader: PUFF_FS, transparent: true, depthWrite: false,
      blending: THREE.CustomBlending, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor, fog: false,
    }));
    this.mesh.frustumCulled = false; this.mesh.userData.noAO = true; this.mesh.renderOrder = 4; this.mesh.visible = false;
    // transparent cores with faint shock diamonds
    this.cores = kit.nozzles.map(n => {
      const d = n.d, L = 14 * d.re;
      const g = new THREE.CylinderGeometry(d.re * 1.6, d.re * 0.95, L, 32, 24, true).translate(0, L / 2, 0);
      const m = new THREE.ShaderMaterial({
        uniforms: { uAmt: { value: 0 }, uL: { value: L }, uLam: { value: 2.6 * d.re }, uT: { value: 0 } },
        vertexShader: 'varying float vY; varying vec3 vN; varying vec3 vV; void main() { vY = position.y; vec4 w = modelMatrix * vec4(position, 1.0); vN = normalize(mat3(modelMatrix) * normal); vV = cameraPosition - w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }',
        fragmentShader: /* glsl */`uniform float uAmt, uL, uLam, uT; varying float vY; varying vec3 vN; varying vec3 vV;
          void main() {
            float s = vY / uL;
            float body = (1.0 - s) * (1.0 - s) * smoothstep(0.0, 0.03, s);
            float diamonds = 0.55 + 0.45 * pow(0.5 + 0.5 * cos(6.2832 * vY / uLam), 3.0) * (1.0 - s);
            float face = pow(abs(dot(normalize(vN), normalize(vV))), 1.5);
            gl_FragColor = vec4(vec3(0.55, 0.75, 1.0) * body * diamonds * face * uAmt * 0.45, 1.0);
          }`,
        transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false,
      });
      const mesh = new THREE.Mesh(g, m);
      mesh.userData.noAO = true; mesh.renderOrder = 3; mesh.visible = false;
      mesh.position.set(...n.spec.exit); mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), n.axis);
      kit.group.add(mesh);
      return mesh;
    });
    this._p = new THREE.Vector3(); this._e = new THREE.Vector3(); this._t = new THREE.Vector3(); this._b = new THREE.Vector3();
  }

  /**
   * Pose the burst at film time T: valve open over [t0, t1] (film s) for every nozzle. world = body -> world.
   * floorY = floor height in the body frame (y of the studio floor relative to the car).
   */
  update(T, t0, t1, world, floorY) {
    const P = this.aP.array, R = this.aR.array, ps = this.puffs, dur = t1 - t0;
    const on = T > t0 && T < t1 + 3.2;
    this.mesh.visible = on;
    const open = t => (t < t0 ? 0 : t < t0 + 0.05 ? (t - t0) / 0.05 : t < t1 ? 1 : Math.max(0, 1 - (t - t1) / 0.04));
    this.cores.forEach(c => { const a = open(T); c.visible = a > 0.01; c.material.uniforms.uAmt.value = a; });
    if (!on) return;
    const p = this._p, e = this._e, tan = this._t, bin = this._b;
    for (let k = 0; k < ps.length; k++) {
      const q = ps[k], n = this.kit.nozzles[q.i];
      const te = t0 + q.te * dur, age = T - te;
      let alpha = 0, size = 0;
      if (age > 0 && age < q.life) {
        const d = n.d, ax = n.axis;
        // drag-limited travel along the axis (fog decelerates fast), cone spread, slow sinking of cold fog
        const U = 14 * q.sp, kd = 5.0;
        const s = (U / kd) * (1 - Math.exp(-kd * age));
        e.set(...n.spec.exit).addScaledVector(ax, s);
        tan.set(1, 0, 0); if (Math.abs(ax.x) > 0.9) tan.set(0, 1, 0);
        tan.addScaledVector(ax, -tan.dot(ax)).normalize(); bin.crossVectors(ax, tan);
        const spread = d.re * 0.6 + s * 0.2 * q.c;
        e.addScaledVector(tan, Math.cos(q.a) * spread).addScaledVector(bin, Math.sin(q.a) * spread);
        e.y -= 0.12 * age * age;
        e.x += Math.sin(q.n1 + age * 2.1) * 0.04 * age; e.z += Math.cos(q.n2 + age * 1.7) * 0.04 * age;
        // the floor: bottom jets fan out radially over it
        if (e.y < floorY + 0.04) {
          const over = floorY + 0.04 - e.y;
          const rx = e.x - n.spec.exit[0], rz = e.z - n.spec.exit[2], rl = Math.hypot(rx, rz) || 1;
          e.x += (rx / rl) * over * 1.4; e.z += (rz / rl) * over * 1.4; e.y = floorY + 0.04 + over * 0.08;
        }
        p.copy(e).applyMatrix4(world);
        size = (d.re * 1.6 + s * 0.26) * (0.8 + 0.4 * q.c);
        const fin = Math.min(1, age / 0.04), fout = Math.pow(1 - age / q.life, 2.0);
        alpha = 0.2 * fin * fout;
      }
      P[k * 4] = p.x; P[k * 4 + 1] = p.y; P[k * 4 + 2] = p.z; P[k * 4 + 3] = size;
      R[k * 2] = alpha; R[k * 2 + 1] = q.rot + age * 0.4;
    }
    this.aP.needsUpdate = this.aR.needsUpdate = true;
  }
}
