// Cold-gas jets of the thruster kit, time-scrubbable and deterministic in sim time t (same pattern as effects.js:
// everything is laid out once from the logs, shaders animate it from uTime).
//
// What a 6 MPa air jet at ~96 K exit temperature looks like: the supersonic core is nearly invisible (a faint
// refractive shimmer with weak shock diamonds where the slightly under-expanded jet re-compresses); humid ambient
// air entrained into the freezing shear layer condenses into a white sheath that thickens downstream, billows
// and re-evaporates within a few metres as the mixture warms up; a jet that reaches the road spreads into a
// radial wall jet of fog and dust.
//
//  * JetVolumes: per nozzle a bounding cylinder (back faces) raymarched in its own frame: core shimmer +
//    diamonds (premultiplied additive, kept well under the stage's bloom threshold) and the condensation sheath
//    (lit white, alpha). Length and density follow the thrust; the volume ends where the jet meets the ground.
//  * PlumePuffs: fog parcels emitted at fixed time steps along the jet with turbulent-jet kinematics
//    (centre-line u ∝ 1/x: x(age) = sqrt(x0² + 2 C age), C ∝ sqrt(F/ρ)), lingering residual fog, and the ground
//    sheet + dust where a jet axis hits the road within a few metres. Same lit-sprite shading as the crash dust.
//
// Nozzle poses come from a PlumeTrack: sampled from a soft-body recording (rigid fit of each nozzle's mount
// cluster, meta.propulsion.mounts) or from a rigid trajectory (spec/README §1). Thrust from the logs.
import * as THREE from 'three';
import { puffTexture } from '../scene/textures.js';
import { PROPULSION, WORLD } from '../core/spec.js';
import { Propulsion } from '../flight/propulsion.js';
import { quatToMat } from '../core/quat.js';
import { fitFrame } from '../softbody/actuators.js';

const NZ = PROPULSION.nozzles.length;
const RHO = WORLD.air.density, DECK = WORLD.deck;
const GROUND_REACH = 4.0;   // jets closer than this to a surface raise the ground sheet (full strength at ~1 m)
const EMIT_DT = 1 / 240;    // emission step of the fog parcels (s)

/** Nominal full-open thrust per nozzle at a full tank (N) and exit radius (m). */
export function nozzleNominal(spec = PROPULSION) {
  const P = new Propulsion(spec), pc = P.feedPressure(), T = P.T;
  const F = Float64Array.from({ length: NZ }, (_, i) => P.At[i] * pc * P.choke / Math.sqrt(T) * P.Me[i] * Math.sqrt(P.g * P.R * T * P.TeRatio[i]) + (pc * P.peRatio[i] - P.pa) * P.Ae[i]);
  const re = Float64Array.from(spec.nozzles, z => (z.throat_diameter / 2) * Math.sqrt(z.expansion_ratio));
  return { F, re };
}

// ------------------------------------------------------------------------------------------------ track
/**
 * Nozzle poses + thrust over time. t[k] sample times; exit/dir (Float32, k*NZ*3) world exit point and exhaust
 * direction; thrust (k*NZ); vel (k*3) vehicle velocity (fog inherits it).
 */
export class PlumeTrack {
  constructor(n) {
    this.n = n; this.t = new Float64Array(n);
    this.exit = new Float32Array(n * NZ * 3); this.dir = new Float32Array(n * NZ * 3);
    this.thrust = new Float32Array(n * NZ); this.vel = new Float32Array(n * 3);
  }
  /** Index + fraction for time t (clamped). */
  locate(t) {
    const T = this.t, n = this.n;
    if (n < 2 || t <= T[0]) return [0, 0];
    if (t >= T[n - 1]) return [n - 2, 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; }
    return [lo, (t - T[lo]) / (T[hi] - T[lo])];
  }
  thrustAt(t, i) { const [k, a] = this.locate(t), T = this.thrust; return T[k * NZ + i] * (1 - a) + T[(k + 1) * NZ + i] * a; }
  /** Interpolated pose of nozzle i at t into pos/dir (arrays of 3); returns vehicle velocity into vel if given. */
  poseAt(t, i, pos, dir, vel) {
    const [k, a] = this.locate(t), o0 = (k * NZ + i) * 3, o1 = o0 + NZ * 3, E = this.exit, D = this.dir;
    for (let c = 0; c < 3; c++) { pos[c] = E[o0 + c] * (1 - a) + E[o1 + c] * a; dir[c] = D[o0 + c] * (1 - a) + D[o1 + c] * a; }
    const l = Math.hypot(dir[0], dir[1], dir[2]) || 1; dir[0] /= l; dir[1] /= l; dir[2] /= l;
    if (vel) for (let c = 0; c < 3; c++) vel[c] = this.vel[k * 3 + c] * (1 - a) + this.vel[(k + 1) * 3 + c] * a;
  }
}

/** Mount-cluster rigid frames -> nozzle exit/exhaust in world, for node positions X (flat). */
export class MountFitter {
  constructor(rest, log) {
    this.rest = rest; this.mounts = log.mounts;
    this.nodes = this.mounts.map(m => Int32Array.from(m.nodes)); this.share = this.mounts.map(m => Float64Array.from(m.share));
    this.restC = this.mounts.map((m, i) => { const c = [0, 0, 0]; this.nodes[i].forEach((n, k) => { for (let a = 0; a < 3; a++) c[a] += this.share[i][k] * rest[n * 3 + a]; }); return c; });
    this.q = this.mounts.map(() => [0, 0, 0, 1]); this.c = this.mounts.map(() => [0, 0, 0]);
    this.R = new Float64Array(9); this.A = new Float64Array(9);
  }
  /** Seed every frame with a body orientation (e.g. the logged state q) before large jumps in time. */
  seed(q) { for (const qq of this.q) qq.splice(0, 4, q[0], q[1], q[2], q[3]); }
  /** Pose of nozzle i: writes exit point and exhaust direction (world). */
  pose(X, i, pos, dir, iters = 4) {
    const m = this.mounts[i], R = this.R, rc = this.restC[i], c = this.c[i];
    fitFrame(X, this.rest, this.nodes[i], this.share[i], rc, this.q[i], c, iters, this.A);
    quatToMat(this.q[i], R);
    const e = [m.exit[0] - rc[0], m.exit[1] - rc[1], m.exit[2] - rc[2]], x = m.exhaust;
    for (let a = 0; a < 3; a++) {
      pos[a] = c[a] + R[a * 3] * e[0] + R[a * 3 + 1] * e[1] + R[a * 3 + 2] * e[2];
      dir[a] = R[a * 3] * x[0] + R[a * 3 + 1] * x[1] + R[a * 3 + 2] * x[2];
    }
  }
}

/** Track from a soft-body recording (RecordingPlayer + meta.propulsion), sampled at the log rate. */
export function trackFromRecording(player, log) {
  const n = log.t.length, tr = new PlumeTrack(n), fit = new MountFitter(player.car.nodes.rest, log);
  const pos = [0, 0, 0], dir = [0, 0, 0];
  for (let k = 0; k < n; k++) {
    const t = log.t[k], X = player.sample(t), S = log.state[k];
    if (k === 0) fit.seed(S.slice(3, 7));
    tr.t[k] = t;
    for (let i = 0; i < NZ; i++) {
      fit.pose(X, i, pos, dir, k === 0 ? 30 : 4);
      tr.exit.set(pos, (k * NZ + i) * 3); tr.dir.set(dir, (k * NZ + i) * 3);
      tr.thrust[k * NZ + i] = log.thrust[k][i];
    }
    tr.vel.set([S[7], S[8], S[9]], k * 3);
  }
  tr.fitter = fit;
  return tr;
}

/** Track from a coldgas-traj@1 rigid trajectory: x_W = p + R(q)(x_B − comB) (spec/README §1). */
export function trackFromTrajectory(traj, spec = PROPULSION) {
  const n = traj.t.length, tr = new PlumeTrack(n), R = new Float64Array(9);
  for (let k = 0; k < n; k++) {
    const S = traj.state[k], c = traj.comB?.[k] || [0, 0, 0];
    quatToMat([S[3], S[4], S[5], S[6]], R);
    tr.t[k] = traj.t[k];
    spec.nozzles.forEach((z, i) => {
      const e = [z.exit[0] - c[0], z.exit[1] - c[1], z.exit[2] - c[2]], x = z.exhaust, o = (k * NZ + i) * 3;
      for (let a = 0; a < 3; a++) {
        tr.exit[o + a] = S[a] + R[a * 3] * e[0] + R[a * 3 + 1] * e[1] + R[a * 3 + 2] * e[2];
        tr.dir[o + a] = R[a * 3] * x[0] + R[a * 3 + 1] * x[1] + R[a * 3 + 2] * x[2];
      }
      tr.thrust[k * NZ + i] = traj.thrust?.[k]?.[i] ?? 0;
    });
    tr.vel.set([S[7], S[8], S[9]], k * 3);
  }
  return tr;
}

/** Distance along a ray (from p, direction d) to the road surface below it (street, or deck top over the deck). */
export function groundHit(p, d) {
  if (d[1] > -0.2) return Infinity;
  const overDeck = Math.abs(p[2]) < DECK.halfWidth && p[1] > DECK.top - 0.3;
  const y = overDeck ? DECK.top : 0;
  const lam = (p[1] - y) / -d[1];
  if (overDeck) { const z = p[2] + d[2] * lam; if (Math.abs(z) > DECK.halfWidth) return (p[1]) / -d[1]; }
  return lam;
}

// ------------------------------------------------------------------------------------------------ jet volumes
const JET_VS = /* glsl */`
  attribute vec4 iA;   // intensity (F/Fnom), length L, exit radius re, ground distance (m along the axis)
  uniform float uSpread;
  uniform vec3 uSunL;
  varying vec3 vLocal; varying vec3 vCam; varying vec4 vA; varying float vR; varying vec3 vSun;
  void main() {
    // bounding cylinder: stops at the road; wider where the jet flares out on it
    float L = max(min(iA.y, iA.w + 0.05), 0.05), hit = iA.w < iA.y ? 1.0 : 0.0;
    float R = iA.z * 2.2 + L * uSpread * (1.0 + 2.4 * hit);
    vec3 p = vec3(position.x * R, position.y * L, position.z * R);
    vLocal = p; vA = iA; vR = R;
    mat4 m = modelMatrix * instanceMatrix;
    // camera in the instance frame (rigid instance matrix: inverse = transpose of the rotation)
    mat3 rot = mat3(m);
    vCam = transpose(rot) * (cameraPosition - m[3].xyz);
    vSun = normalize(transpose(rot) * uSunL);
    gl_Position = projectionMatrix * viewMatrix * m * vec4(p, 1.0);
  }`;
const JET_FS = /* glsl */`
  uniform float uTime; uniform vec3 uSunL; uniform vec3 uSunCol; uniform vec3 uAmb; uniform float uSpread;
  varying vec3 vLocal; varying vec3 vCam; varying vec4 vA; varying float vR; varying vec3 vSun;
  float h31(vec3 p) { p = fract(p * 0.3183099 + 0.1); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
  float vnoise(vec3 x) {
    vec3 i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(h31(i), h31(i + vec3(1,0,0)), f.x), mix(h31(i + vec3(0,1,0)), h31(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(h31(i + vec3(0,0,1)), h31(i + vec3(1,0,1)), f.x), mix(h31(i + vec3(0,1,1)), h31(i + vec3(1,1,1)), f.x), f.y), f.z);
  }
  void main() {
    float I = vA.x, L = max(vA.y, 0.05), re = vA.z, gd = vA.w;
    if (I < 0.003) discard;
    // ray from the camera to this back-face point, clipped to the bounding cylinder r <= vR, 0 <= y <= L
    vec3 ro = vCam, rd = normalize(vLocal - vCam);
    float tExit = length(vLocal - vCam), tEnter = 0.0;
    float a = dot(rd.xz, rd.xz), b = dot(ro.xz, rd.xz), c = dot(ro.xz, ro.xz) - vR * vR;
    if (a > 1e-6) { float disc = b * b - a * c; if (disc > 0.0) tEnter = max(tEnter, (-b - sqrt(disc)) / a); }
    if (abs(rd.y) > 1e-6) { float t0 = (0.0 - ro.y) / rd.y, t1 = (L - ro.y) / rd.y; tEnter = max(tEnter, min(t0, t1)); }
    tEnter = clamp(tEnter, 0.0, tExit);
    const int STEPS = 22;
    float dt = (tExit - tEnter) / float(STEPS);
    float trans = 1.0; vec3 glow = vec3(0.0); float fogA = 0.0; vec3 fogC = vec3(0.0);
    vec3 fogCol = vec3(0.92, 0.94, 0.98) * (uAmb + uSunCol * 0.85);
    float yEnd = min(L, gd);
    for (int k = 0; k < STEPS; k++) {
      vec3 p = ro + rd * (tEnter + (float(k) + 0.5) * dt);
      float y = p.y; if (y < 0.0 || y > yEnd) continue;
      float r = length(p.xz);
      // supersonic core: potential core ~6 exit diameters with weak shock cells, then gone
      float rc = re * (1.0 + 0.6 * y / (12.0 * re + 1e-3));
      float core = exp(-r * r / (rc * rc)) * exp(-y / (14.0 * re));
      float cell = 1.4 * re * 3.0;                                     // shock-cell spacing ~ 1.3 D sqrt(M^2-1)
      float diamonds = pow(0.5 + 0.5 * cos(6.2832 * y / cell), 6.0) * smoothstep(0.0, 0.6 * cell, y);
      glow += vec3(0.55, 0.72, 1.0) * core * (0.18 + 1.6 * diamonds) * dt / re;
      // condensation sheath: forms in the freezing mixing layer (annulus), fills the jet by ~1 m, eddies scale
      // with the local jet width (self-similar turbulence) and stream downstream; re-evaporates near the end
      float flare = gd < L ? smoothstep(gd - 0.55, gd, y) : 0.0;  // splash on the road
      float w = (re * 1.2 + y * uSpread * 0.9) * (1.0 + 1.4 * flare);
      vec3 wq = vec3(p.x * 3.0, y * 1.6 - uTime * 9.0, p.z * 3.0);
      float rn = (r + (vnoise(wq) - 0.5) * w * 0.9) / w;
      float fill = smoothstep(0.15, 1.3, y);
      float prof = exp(-rn * rn * 2.2) * mix(smoothstep(0.25, 0.95, rn), 1.0, fill);
      vec3 q = vec3(p.x / w * 1.6, y / (w + 0.05) * 0.55 - uTime * 26.0 * sqrt(I), p.z / w * 1.6);
      float n = 0.55 * vnoise(q) + 0.3 * vnoise(q * 2.3 + 7.1) + 0.15 * vnoise(q * 5.1 + 3.7);
      float turb = smoothstep(0.34, 0.72, n) * 2.2;
      float grow = smoothstep(0.08, 0.75, y) * (1.0 - smoothstep(0.55 * L, L, y));
      float dens = prof * max(grow, flare) * turb * (1.5 / (1.0 + y * 0.7)) * pow(I, 0.6)
                 * smoothstep(vR, 0.5 * vR, r) * (1.0 - 0.85 * smoothstep(gd - 0.3, gd, y));  // never touch the bounds
      float aStep = 1.0 - exp(-dens * dt * 8.0);
      // lit side of the column brighter, eddy interiors a touch darker (cheap self-shadowing)
      vec2 rad = r > 1e-4 ? p.xz / r : vec2(0.0);
      float lit = 0.62 + 0.38 * clamp(dot(vec3(rad.x, 0.0, rad.y), vSun) * 0.5 + 0.5 + 0.25 * vSun.y, 0.0, 1.0);
      lit *= 0.8 + 0.2 * n;
      fogC += trans * aStep * fogCol * lit;
      fogA += trans * aStep;
      trans *= 1.0 - aStep;
    }
    float g = sqrt(I) * 0.75;
    vec3 col = fogC + glow * g * 0.08 * trans;
    gl_FragColor = vec4(col, fogA);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;

class JetVolumes {
  constructor(scene, stage) {
    const geo = new THREE.CylinderGeometry(1, 1, 1, 20, 1, true).translate(0, 0.5, 0);
    // y = 0 lies at the exit; cap the far end so rays entering through it still hit a back face
    const cap = new THREE.CircleGeometry(1, 20).rotateX(-Math.PI / 2).translate(0, 1, 0);
    const cap0 = new THREE.CircleGeometry(1, 20).rotateX(Math.PI / 2);
    const g = mergeGeoms([geo, cap, cap0]);
    this.iA = new THREE.InstancedBufferAttribute(new Float32Array(NZ * 4), 4);
    g.setAttribute('iA', this.iA);
    this.mat = new THREE.ShaderMaterial({
      vertexShader: JET_VS, fragmentShader: JET_FS, transparent: true, depthWrite: false, side: THREE.BackSide,
      blending: THREE.CustomBlending, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
      uniforms: { uTime: { value: 0 }, uSpread: { value: 0.2 }, uSunL: { value: stage.sunDir }, uSunCol: { value: new THREE.Color(0xfff1df).multiplyScalar(1.15) }, uAmb: { value: new THREE.Color(0.52, 0.57, 0.64) } },
    });
    this.mesh = new THREE.InstancedMesh(g, this.mat, NZ);
    this.mesh.frustumCulled = false; this.mesh.userData.noAO = true; this.mesh.renderOrder = 6;
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    scene.add(this.mesh);
    this._m = new THREE.Matrix4(); this._q = new THREE.Quaternion(); this._p = new THREE.Vector3(); this._d = new THREE.Vector3(); this._one = new THREE.Vector3(1, 1, 1); this._y = new THREE.Vector3(0, 1, 0);
  }
  set(i, pos, dir, I, L, re, gd) {
    this._p.set(pos[0], pos[1], pos[2]); this._d.set(dir[0], dir[1], dir[2]);
    this._q.setFromUnitVectors(this._y, this._d);
    this._m.compose(this._p, this._q, this._one);
    this.mesh.setMatrixAt(i, this._m);
    this.iA.setXYZW(i, I, L, re, gd);
  }
  commit() { this.mesh.instanceMatrix.needsUpdate = true; this.iA.needsUpdate = true; }
}
function mergeGeoms(list) {
  const pos = [], idx = []; let off = 0;
  for (const g of list) {
    const gi = g.index ? g : g.toNonIndexed(), P = gi.attributes.position;
    for (let k = 0; k < P.count; k++) pos.push(P.getX(k), P.getY(k), P.getZ(k));
    if (gi.index) for (let k = 0; k < gi.index.count; k++) idx.push(gi.index.getX(k) + off); else for (let k = 0; k < P.count; k++) idx.push(k + off);
    off += P.count;
  }
  const g = new THREE.InstancedBufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setIndex(idx);
  return g;
}

// ------------------------------------------------------------------------------------------------ puffs
const PUFF_VS = /* glsl */`
  attribute vec4 a0, a1, a2, a3;  // pos+birth | dir+C | carry vel+x0 | life, seed, kind, intensity
  uniform float uTime; uniform vec3 uSun; uniform vec3 uWind;   // uWind: ambient drift (m/s), zero unless a program sets it
  varying vec2 vUv; varying float vAlpha; varying float vShade; varying float vSeed; varying float vKind; varying float vWy; varying float vErode;
  void main() {
    float age = uTime - a0.w, life = a3.x, kind = a3.z;
    vSeed = a3.y; vKind = kind;
    if (age < 0.0 || age > life) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
    float C = a1.w, x0 = a2.w;
    float x = sqrt(x0 * x0 + 2.0 * C * age);         // turbulent jet: u ~ 1/x
    float kc = 2.2;                                    // carried vehicle velocity relaxes to the still air
    vec3 p = a0.xyz + a1.xyz * x + a2.xyz * (1.0 - exp(-kc * age)) / kc;
    float size, alpha = a3.w;
    float u = age / life;
    if (kind < 0.5) {            // jet fog parcel: grows with the jet, dilutes, sinks a little (cold, dense)
      // parcels pile up ∝ x² along a u ~ 1/x jet: opacity per parcel ∝ (x0/x)² keeps the column even, fading
      // in where the near-field volume fades out and thinning out towards the evaporation distance
      size = 0.14 + 0.5 * x;
      p.y -= 0.35 * age * age;
      float xr = x0 / max(x, x0), xe = sqrt(x0 * x0 + 2.0 * C * life);
      alpha *= smoothstep(x0, x0 * 1.25, x) * xr * xr * (1.0 - smoothstep(0.35 * xe, xe, x));
    } else if (kind < 1.5) {     // residual fog: slow, large, lingering
      size = (0.6 + 1.8 * (1.0 - exp(-1.5 * age))) * (0.7 + 0.6 * vSeed);
      p.y += -0.25 * age + 0.06 * age * age;
      p += uWind * age;
      alpha *= smoothstep(0.0, 0.25, age) * pow(1.0 - u, 1.8);
    } else {                     // ground sheet (2) / dust (3): radial wall jet hugging the road
      size = (0.3 + 0.34 * x) * (kind > 2.5 ? 0.6 : 1.0);
      p.y = a0.y + size * 0.3 + 0.05 * x * (0.4 + vSeed) + 0.12 * age;
      float xr = (x0 + 0.6) / (x + 0.6);
      alpha *= smoothstep(0.0, 0.02, age) * min(1.0, xr * 1.4) * pow(1.0 - u, 1.4);
    }
    vAlpha = alpha; vErode = kind < 0.5 ? 0.6 + 0.4 * u : kind < 1.5 ? 0.8 + 0.2 * u : 0.35 + 0.6 * u;
    vec4 mv = modelViewMatrix * vec4(p, 1.0);
    float rot = vSeed * 6.283 + age * (vSeed - 0.5) * 1.6;
    vec2 q = mat2(cos(rot), -sin(rot), sin(rot), cos(rot)) * position.xy;
    mv.xy += q * size;
    vWy = (inverse(viewMatrix) * mv).y;
    vUv = position.xy + 0.5;
    vShade = 0.6 + 0.4 * clamp(dot(normalize(vec3(q, 0.6)), normalize((viewMatrix * vec4(uSun, 0.0)).xyz)) * 0.5 + 0.5, 0.0, 1.0);
    gl_Position = projectionMatrix * mv;
  }`;
const PUFF_FS = /* glsl */`
  uniform sampler2D uTex; uniform vec3 uSunCol; uniform vec3 uAmb;
  varying vec2 vUv; varying float vAlpha; varying float vShade; varying float vSeed; varying float vKind; varying float vWy; varying float vErode;
  void main() {
    vec2 uv = vUv; if (vSeed > 0.5) uv.x = 1.0 - uv.x;
    // wispy break-up that grows as the parcel dissipates (value noise over the sprite, seeded per parcel)
    vec2 nq = uv * 3.3 + vec2(vSeed * 17.0, vSeed * 31.0);
    vec2 i = floor(nq), f = fract(nq); f = f * f * (3.0 - 2.0 * f);
    float h00 = fract(sin(dot(i, vec2(127.1, 311.7))) * 43758.5), h10 = fract(sin(dot(i + vec2(1, 0), vec2(127.1, 311.7))) * 43758.5);
    float h01 = fract(sin(dot(i + vec2(0, 1), vec2(127.1, 311.7))) * 43758.5), h11 = fract(sin(dot(i + vec2(1, 1), vec2(127.1, 311.7))) * 43758.5);
    float nz = mix(mix(h00, h10, f.x), mix(h01, h11, f.x), f.y);
    float rr = length(vUv - 0.5) * 2.0;
    float a = texture2D(uTex, uv).r * exp(-rr * rr * 3.2) * (1.0 - smoothstep(0.85, 1.0, rr)) * 1.6 * vAlpha * smoothstep(-0.05, 0.3, vWy) * mix(1.0, smoothstep(0.15, 0.85, nz) * 1.6, vErode);
    if (a < 0.002) discard;
    vec3 base = vKind > 2.5 ? vec3(0.50, 0.47, 0.42) : vKind > 1.5 ? mix(vec3(0.86, 0.87, 0.88), vec3(0.62, 0.59, 0.54), step(0.72, vSeed)) : vec3(0.92, 0.94, 0.98);
    vec3 c = base * (uAmb + uSunCol * vShade);
    gl_FragColor = vec4(c, a);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;

class PuffField {
  constructor(scene, stage, max) {
    const g = new THREE.InstancedBufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0], 3));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    this.attr = ['a0', 'a1', 'a2', 'a3'].map(k => { const a = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4); g.setAttribute(k, a); return a; });
    g.instanceCount = 0;
    this.mat = new THREE.ShaderMaterial({
      vertexShader: PUFF_VS, fragmentShader: PUFF_FS, transparent: true, depthWrite: false,
      uniforms: { uTime: { value: 0 }, uTex: { value: puffTexture() }, uSun: { value: stage.sunDir }, uSunCol: { value: new THREE.Color(0xfff1df).multiplyScalar(1.1) }, uAmb: { value: new THREE.Color(0.50, 0.55, 0.62) }, uWind: { value: new THREE.Vector3() } },
    });
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.frustumCulled = false; this.mesh.userData.noAO = true; this.mesh.renderOrder = 7;
    scene.add(this.mesh);
    this.max = max; this.n = 0;
  }
  clear() { this.n = 0; this.mesh.geometry.instanceCount = 0; }
  push(a0, a1, a2, a3) {
    if (this.n >= this.max) return;
    const i = this.n++;
    this.attr[0].setXYZW(i, ...a0); this.attr[1].setXYZW(i, ...a1); this.attr[2].setXYZW(i, ...a2); this.attr[3].setXYZW(i, ...a3);
  }
  commit() { for (const a of this.attr) a.needsUpdate = true; this.mesh.geometry.instanceCount = this.n; }
}

// ------------------------------------------------------------------------------------------------ plumes
export class Plumes {
  /**
   * style 'default': the look every program had so far (length, density and fog all ∝ sqrt(thrust): a jet at 10 % of
   * full thrust already reads as a column). style 'thrust' (opt-in, 直到用计算机模拟 04 / 05): each nozzle's jet length,
   * core density, fog emission, reach and opacity scale ~linearly with its own thrust, so a trimming valve at 0.1-0.3
   * shows a short faint wisp and only 0.8+ makes a column down to the road; the fog keeps less of the vehicle's velocity
   * and the lingering residual drifts with `wind` (m/s, world) instead of hanging as a vertical column.
   */
  constructor(scene, stage, { maxPuffs = 60000, style = 'default', wind = null } = {}) {
    this.jets = new JetVolumes(scene, stage);
    this.puffs = new PuffField(scene, stage, maxPuffs);
    this.style = style;
    if (wind) this.puffs.mat.uniforms.uWind.value.set(wind[0], wind[1], wind[2]);
    this.nom = nozzleNominal();
    this.groups = PROPULSION.nozzles.map(z => z.group);
    this.track = null;
    this._p = [0, 0, 0]; this._d = [0, 0, 0]; this._v = [0, 0, 0];
  }
  /** Objects to keep out of reflection probes / AO. */
  get objects() { return [this.jets.mesh, this.puffs.mesh]; }
  set visible(v) { this.jets.mesh.visible = this.puffs.mesh.visible = v; }

  /** Lay out every fog parcel of a run (deterministic: fixed emission steps, seeded per nozzle). */
  setTrack(track) {
    this.track = track;
    const P = this.puffs; P.clear();
    if (!track || track.n < 2) { P.commit(); return; }
    const t0 = track.t[0], t1 = track.t[track.n - 1], pos = this._p, dir = this._d, vel = this._v;
    for (let i = 0; i < NZ; i++) {
      let s = 1 + i * 7919;
      const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
      let accJ = 0, accR = 0, accG = 0;
      for (let t = t0; t <= t1; t += EMIT_DT) {
        const F = track.thrustAt(t, i), f = F / this.nom.F[i];
        if (f < 0.01) { accJ = accR = accG = 0; continue; }
        track.poseAt(t, i, pos, dir, vel);
        const C = 4.4 * Math.sqrt(F / RHO), L = jetLength(f, this.style), x0 = 0.8 * L, lin = this.style === 'thrust';
        // jet parcels: continuous enough at the jet speed (more when strong), residual fog at the end
        const xm = lin ? 0.6 + 5.0 * f : 2.6 + 3.6 * Math.sqrt(f), carry = lin ? 0.35 : 1;
        accJ += EMIT_DT * 900 * (lin ? f : Math.sqrt(f));
        while (accJ >= 1) {
          accJ -= 1;
          const tb = t + r() * EMIT_DT, life = (xm * xm - x0 * x0) / (2 * C) * (0.7 + 0.6 * r());
          const j = jitterDir(dir, 0.07, r);
          P.push([pos[0], pos[1], pos[2], tb], [j[0], j[1], j[2], C * (0.8 + 0.4 * r())], [vel[0] * carry, vel[1] * carry, vel[2] * carry, x0], [Math.max(0.05, life), r(), 0, lin ? 0.13 + 0.1 * f : 0.14 + 0.1 * Math.sqrt(f)]);
        }
        accR += EMIT_DT * 34 * f;
        while (accR >= 1) {
          accR -= 1;
          const j = jitterDir(dir, 0.12, r), cr = lin ? 0.2 : 0.5;
          P.push([pos[0] + j[0] * xm, pos[1] + j[1] * xm, pos[2] + j[2] * xm, t + r() * EMIT_DT], [j[0], j[1], j[2], 6 * Math.sqrt(f)], [vel[0] * cr, vel[1] * cr, vel[2] * cr, 0.2], [1.6 + 1.6 * r(), r(), 1, lin ? 0.012 + 0.05 * f : 0.03 + 0.035 * f]);
        }
        // ground sheet where the jet axis reaches the road ('thrust': only where the jet's own reach gets there)
        const lam = groundHit(pos, dir), reach = lin ? Math.min(GROUND_REACH, 0.9 * xm + 0.3) : GROUND_REACH;
        if (lam < reach) {
          const k = smooth01((reach - lam) / Math.max(0.3, reach - 1.0)) * (lin ? f : Math.sqrt(f));
          accG += EMIT_DT * 110 * k;
          const hx = pos[0] + dir[0] * lam, hy = pos[1] + dir[1] * lam, hz = pos[2] + dir[2] * lam;
          const bias = [dir[0], dir[2]], bl = Math.hypot(bias[0], bias[1]);
          while (accG >= 1) {
            accG -= 1;
            let a = r() * 6.2832, ux = Math.cos(a), uz = Math.sin(a);
            if (bl > 0.05) { ux += bias[0] / bl * 0.6 * bl; uz += bias[1] / bl * 0.6 * bl; const l = Math.hypot(ux, uz); ux /= l; uz /= l; }
            const dust = r() < 0.22;
            P.push([hx, hy, hz, t + r() * EMIT_DT], [ux, 0, uz, C * 0.07 * k * (0.6 + 0.8 * r())], [vel[0] * 0.15, 0, vel[2] * 0.15, 0.25 + 0.15 * lam], [(dust ? 1.2 : 0.7) + 1.2 * r(), r(), dust ? 3 : 2, (dust ? 0.07 : 0.09) * (0.4 + 0.6 * k)]);
          }
        } else accG = 0;
      }
    }
    P.commit();
  }

  /**
   * Pose the jets at sim time t. `nodePos` = current soft-body node positions (exact attachment to the mount
   * clusters through track.fitter, `iters` rotation-fit iterations); without it the track is interpolated.
   */
  update(t, nodePos = null, iters = 3) {
    this.puffs.mat.uniforms.uTime.value = t;
    this.jets.mat.uniforms.uTime.value = t;
    const tr = this.track;
    if (!tr) { for (let i = 0; i < NZ; i++) this.jets.iA.setXYZW(i, 0, 0, 0, 0); this.jets.commit(); return; }
    const pos = this._p, dir = this._d;
    for (let i = 0; i < NZ; i++) {
      const F = tr.thrustAt(t, i), f = F / this.nom.F[i];
      if (nodePos && tr.fitter) tr.fitter.pose(nodePos, i, pos, dir, iters); else tr.poseAt(t, i, pos, dir);
      const L = jetLength(f, this.style), I = this.style === 'thrust' ? Math.max(0, f) ** 1.6 : f;   // the shader's density goes as I^0.6
      this.jets.set(i, pos, dir, f > 0.005 ? I : 0, L, this.nom.re[i], groundHit(pos, dir));
    }
    this.jets.commit();
  }
}

const jetLength = (f, style) => (style === 'thrust' ? 0.25 + 3.3 * Math.max(0, f) : 0.5 + 2.6 * Math.sqrt(Math.max(0, f)));
const smooth01 = u => { u = u < 0 ? 0 : u > 1 ? 1 : u; return u * u * (3 - 2 * u); };
function jitterDir(d, s, r) {
  const j = [d[0] + (r() - 0.5) * 2 * s, d[1] + (r() - 0.5) * 2 * s, d[2] + (r() - 0.5) * 2 * s], l = Math.hypot(j[0], j[1], j[2]);
  return [j[0] / l, j[1] / l, j[2] / l];
}
