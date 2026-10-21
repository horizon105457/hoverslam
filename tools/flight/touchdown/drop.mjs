// One soft-body touchdown experiment for the grey-box identification of the rigid twin's touchdown model: the kit car
// (ThrusterRig without controller: kit masses + kit rear springs, gas fill sets the mass, no thrust) flies
// ballistically onto open street and is recorded through the first tyre contact and `post` seconds beyond.
// usage: node tools/flight/touchdown/drop.mjs [vs=1.25 u=15 beta=0 theta=0 phi=0 yaw_rate=0 pitch_rate=0 roll_rate=0
//          gas=0.4 spin=0.57] [id=drop outDir=experiments/runs/touchdown-scratch] [post=2 airtime=0.3 cmin=0.004
//          hz=200 denseHz=1000 denseWin=0.4 site=-40,22 pose=rest|settled joints=1 force=0 quiet=0]
// joints: CrashSim connector give factor (the launch presets fly with 0.55, spec/launch-presets.json; the DOE config
// sets it since the 2026-10-21 arm round; the first DOE, 20261021-020006, ran at 1 = stronger arms than any flight)
// Factors = the state at the instant the first tyre touches (conventions in lib.mjs / README.md: theta nose up +,
// phi right side down +, beta + = moving toward the right side, pitch/roll rates = d theta/dt, d phi/dt, yaw_rate =
// w_B.y, spin = rim speed / u). Writes <outDir>/<id>.json.gz (format touchdown-drop@1, README.md) and prints a summary.
//
// Pose. The release pose is the lattice REST pose rigidly placed (pose=rest), not settled-coldgas.bin: the rest pose
// is the free-flight equilibrium (every beam at its rest length, springs at free length) that the settled car relaxes
// to in the air, verified by a 1 s free fall of the settled pose (body within 0.3 mm, springs within 1 mm). Releasing
// the settled pose would need ~1 s of air for the overdamped coil-overs (tau = c/k ~ 0.23 s front, 0.11 s rear: 17 mm
// front sag left after 0.3 s), but a ballistic arc ending at sink speed vs has spent at most 2 vs/g in the air above
// the street (0.06 s at 0.3 m/s, 0.25 s at 1.25 m/s). pose=settled places the settled pose instead (springs sagged).
//
// Release. The target state is flown back as a torque-free gyrostat (lattice inertia + the wheels' spin momentum) to
// the latest time <= airtime at which no node was below the street since and the lowest node clears it by >= cmin
// (an earlier release would start inside the street). The wheels get omega = spin u / r about their axles on top of
// the rigid motion, raised for the solver's PBD spin bleed over the fall and pre-rotated so they reach the touchdown at
// a tyre node at the world bottom (like the twin's disc). The achieved touchdown differs a little from the
// target (lattice flex, polygonal tyres, bleed model): it is measured and recorded.
// Speed clamp. CrashSim.substep caps every node at 40 m/s; a tyre's top node moves at ~|v_h| + omega r (2u once rolling),
// so too much speed + spin is clipped within milliseconds and jerks the body (warning at release; the DOE excludes it),
// and landings above ~20 m/s are clipped every substep after the spin-up (flag speed_clamp, samples vmax / nclamp).
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import crypto from 'crypto';
import { loadCarNode } from '../../lib/car.mjs';
import { parseArgs } from '../../lib/args.mjs';
import { readSettled } from '../../lib/sims.mjs';
import { repo } from '../../lib/paths.mjs';
import { CrashSim } from '../../../src/softbody/solver.js';
import { ThrusterRig, fitFrame } from '../../../src/softbody/actuators.js';
import { BEAM } from '../../../src/car/carPrep.js';
import { NU } from '../../../src/flight/contract.js';
import { MassModel } from '../../../src/flight/massprops.js';
import { quatToMat } from '../../../src/core/quat.js';
import { FACTORS, NOMINAL, SITE, SUBSTEP_HZ, VCLAMP, deg, targetState, stateParams, propagate, spinBleed, rotate, rotateT, cross, specHash } from './lib.mjs';

const args = parseArgs();
const f = Object.fromEntries(FACTORS.map(F => [F.key, args[F.key] !== undefined ? +args[F.key] : NOMINAL[F.key]]));
const num = (k, d) => (args[k] !== undefined ? +args[k] : d);
const opt = {
  id: args.id || 'drop', outDir: args.outDir || 'experiments/runs/touchdown-scratch', post: num('post', 2), airtime: num('airtime', 0.3),
  cmin: num('cmin', 0.004), hz: num('hz', 200), denseHz: num('denseHz', 1000), denseWin: num('denseWin', 0.4), pose: args.pose || 'rest',
  force: !!num('force', 0), quiet: !!num('quiet', 0), precontact: num('precontact', 0.05), joints: num('joints', 1),
};
const site = args.site ? (([x, z]) => ({ x: +x, z: +z }))(String(args.site).split(',')) : SITE;
for (const F of FACTORS) if (!Number.isFinite(f[F.key])) throw new Error(`bad factor ${F.key}=${args[F.key]}`);
const T0 = Date.now();

// ------------------------------------------------------------------------------------------------ car + kit
const car = await loadCarNode({ h: 0.065 });
const SETTLED = 'public/assets/settled-coldgas.bin', settled = readSettled(SETTLED, car);
const sim = new CrashSim(car, { seed: 1, joints: opt.joints });
const rig = new ThrusterRig(car, { controller: null, logHz: 100 });
rig.attach(sim);
rig.prop.m = rig.prop.m0 * f.gas; rig.prop.step(new Float64Array(NU), 0); rig.applyMass(sim, f.gas); // partly filled tank (as tools/sim/flight.mjs gas=)
sim.setup(settled); // bookkeeping (springs, events, ...); the deck placement is overwritten below
const N = sim.N, Rest = car.nodes.rest, allow = car.nodes.allow, mass = rig.mass, isWheel = sim.isWheel;
const W = car.wheels, WHEEL_IDS = ['wheel_FL', 'wheel_FR', 'wheel_RL', 'wheel_RR'];
if (W.map(w => w.piece).join() !== WHEEL_IDS.join()) throw new Error('wheel order differs from vehicle-massprops');
const wchunk = W.map(w => car.chunks[w.chunk]);

// body-frame node coordinates of the release shape
let X = Rest;
if (opt.pose === 'settled') { // rigid fit of the settled body chunk to its rest pose -> settled shape in body coordinates
  const body = car.chunks.find(c => c.piece === 'body'), nodes = Int32Array.from({ length: body.nodeCount }, (_, i) => body.nodeStart + i);
  const wts = Float64Array.from(nodes, n => car.nodes.mass[n]), rc = [0, 0, 0]; let ws = 0;
  for (let i = 0; i < nodes.length; i++) { ws += wts[i]; for (let a = 0; a < 3; a++) rc[a] += wts[i] * Rest[nodes[i] * 3 + a]; }
  rc.forEach((v, a) => (rc[a] = v / ws));
  const q = [0, 0, 0, 1], c = [0, 0, 0]; fitFrame(settled, Rest, nodes, wts, rc, q, c, 40);
  const m = quatToMat(q, new Float64Array(9)); X = new Float64Array(N * 3);
  for (let n = 0; n < N; n++) { const b = rotateT(m, [settled[n * 3] - c[0], settled[n * 3 + 1] - c[1], settled[n * 3 + 2] - c[2]]); for (let a = 0; a < 3; a++) X[n * 3 + a] = b[a] + rc[a]; }
} else if (opt.pose !== 'rest') throw new Error(`pose=${opt.pose}?`);

// mass properties of the lattice + kit (the rig's masses): COM, inertia about it, wheel axle inertias
let M = 0; const comB = [0, 0, 0];
for (let n = 0; n < N; n++) { M += mass[n]; for (let a = 0; a < 3; a++) comB[a] += mass[n] * Rest[n * 3 + a]; }
comB.forEach((v, a) => (comB[a] = v / M)); // the rig's comB (rest positions)
let Mx = [0, 0, 0]; for (let n = 0; n < N; n++) for (let a = 0; a < 3; a++) Mx[a] += mass[n] * X[n * 3 + a];
const comX = Mx.map(v => v / M); // COM of the release shape (= comB for pose=rest)
const I = new Array(9).fill(0);
for (let n = 0; n < N; n++) {
  const r = [X[n * 3] - comX[0], X[n * 3 + 1] - comX[1], X[n * 3 + 2] - comX[2]], m = mass[n], r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) I[a * 3 + b] += m * ((a === b ? r2 : 0) - r[a] * r[b]);
}
const wheelC = W.map(w => w.centre), wheelR = W.map(w => w.radius);
const Iax = wchunk.map((ch, k) => { let s = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) s += mass[n] * ((X[n * 3 + 1] - wheelC[k][1]) ** 2 + (X[n * 3 + 2] - wheelC[k][2]) ** 2); return s; });
const omegaTd = W.map((_, k) => f.spin * f.u / wheelR[k]);
const hW = [Iax.reduce((s, v, k) => s + v * omegaTd[k], 0), 0, 0]; // spinning wheels: internal angular momentum (body)

// ------------------------------------------------------------------------------------------------ target + release
const S_td = targetState(f, site);
const wheelOf = new Int8Array(N).fill(-1); W.forEach((w, k) => { for (let n = wchunk[k].nodeStart; n < wchunk[k].nodeStart + wchunk[k].nodeCount; n++) wheelOf[n] = k; });
// candidate nodes for the clearance scan: tyres + every non-wheel node that a <= 20 deg tilt could bring to the street
const scan = []; for (let n = 0; n < N; n++) if (isWheel[n] || X[n * 3 + 1] - allow[n] < 0.8) scan.push(n);
const rotX = (k, n, ang) => { // body coords of node n with its wheel k turned by ang about the axle (+X)
  const c = wheelC[k], y = X[n * 3 + 1] - c[1], z = X[n * 3 + 2] - c[2], ca = Math.cos(ang), sa = Math.sin(ang);
  return [X[n * 3], c[1] + y * ca - z * sa, c[2] + y * sa + z * ca];
};
const bodyOf = (n, ang) => (wheelOf[n] >= 0 && ang ? rotX(wheelOf[n], n, ang[wheelOf[n]]) : [X[n * 3], X[n * 3 + 1], X[n * 3 + 2]]);
// wheel phase at the touchdown: a tyre node at the WORLD bottom of every ring (the 24-gon's rest phase puts one at the
// body bottom; pitched, that node sits theta off the bottom and rises with the spin, i.e. it was below the street just
// before). Rotation a about +X takes the rest bottom (0, -R, 0) to -R (cos a, sin a) = the YZ direction of world down.
const aTd = (() => { const m = quatToMat(S_td.q, new Float64Array(9)), d = rotateT(m, [0, -1, 0]); return Math.atan2(-d[2], -d[1]); })();
/** min clearance (y + allow) over the scan nodes for COM height py, attitude q, wheel angles ang -> [c, node] */
function clearance(py, q, ang) {
  const m = quatToMat(q, new Float64Array(9));
  let lo = Infinity, at = -1;
  for (const n of scan) { const b = bodyOf(n, ang), y = py + m[3] * (b[0] - comX[0]) + m[4] * (b[1] - comX[1]) + m[5] * (b[2] - comX[2]) + allow[n]; if (y < lo) { lo = y; at = n; } }
  return [lo, at];
}
{ const [c0, n0] = clearance(0, S_td.q, W.map(() => aTd)); S_td.p[1] = -c0; S_td.firstNode = n0; }
const hullAtTd = (() => { const m = quatToMat(S_td.q, new Float64Array(9)); let lo = Infinity; for (const n of scan) if (!isWheel[n]) lo = Math.min(lo, S_td.p[1] + m[3] * (X[n * 3] - comX[0]) + m[4] * (X[n * 3 + 1] - comX[1]) + m[5] * (X[n * 3 + 2] - comX[2]) + allow[n]); return lo; })();
// fly back 1 ms at a time: c[k] = clearance k ms before the contact (wheels turned back by omega t)
const KMAX = Math.round(opt.airtime * 1000), cs = [0];
let Sb = { p: [...S_td.p], q: [...S_td.q], v: [...S_td.v], w: [...S_td.w] };
for (let k = 1; k <= KMAX; k++) { Sb = propagate(Sb, I, hW, -1e-3); cs.push(clearance(Sb.p[1], Sb.q, omegaTd.map(o => aTd - o * k * 1e-3))[0]); }
let valid = 0; while (valid < KMAX && cs[valid + 1] > 0) valid++;
let kRel = 0; for (let k = 1; k <= valid; k++) if (cs[k] >= opt.cmin) kRel = k;
if (!kRel) for (let k = 1; k <= valid; k++) if (cs[k] > (cs[kRel] || 0)) kRel = k; // apex below cmin: release at the apex
const feasibleTarget = valid >= Math.min(KMAX, Math.round(opt.precontact * 1000) + 5);
if (!opt.force && (kRel < 5 || !feasibleTarget)) {
  console.error(`${opt.id}: target is not a first contact (a node was below the street ${valid + 1} ms before it; clearance scan ${cs.slice(1, 8).map(c => (c * 1000).toFixed(1)).join(' ')} mm). force=1 runs it anyway`);
  process.exit(2);
}
const tauRel = kRel * 1e-3;
const S_rel = propagate(S_td, I, hW, -tauRel);
const bleed = omegaTd.map(o => spinBleed(o, tauRel)), angRel = bleed.map(b => aTd - b.angle), omegaRel = bleed.map(b => b.omega0);

// ------------------------------------------------------------------------------------------------ placement
let releaseVmax = 0;
{
  const m = quatToMat(S_rel.q, new Float64Array(9)), wW = rotate(m, S_rel.w), aW = [m[0], m[3], m[6]];
  const x = sim.x, v = sim.v, hub = wheelC.map(c => rotate(m, [c[0] - comX[0], c[1] - comX[1], c[2] - comX[2]]).map((r, a) => r + S_rel.p[a]));
  for (let n = 0; n < N; n++) {
    const b = bodyOf(n, angRel), r = rotate(m, [b[0] - comX[0], b[1] - comX[1], b[2] - comX[2]]), i = n * 3;
    for (let a = 0; a < 3; a++) x[i + a] = S_rel.p[a] + r[a];
    const u = cross(wW, r);
    for (let a = 0; a < 3; a++) v[i + a] = S_rel.v[a] + u[a];
    const k = wheelOf[n];
    if (k >= 0) { const s = cross(aW, [x[i] - hub[k][0], x[i + 1] - hub[k][1], x[i + 2] - hub[k][2]]); for (let a = 0; a < 3; a++) v[i + a] += omegaRel[k] * s[a]; }
  }
  // exact COM position / velocity (turned wheels and the spin may shift them by a hair)
  const P = [0, 0, 0], V = [0, 0, 0];
  for (let n = 0; n < N; n++) for (let a = 0; a < 3; a++) { P[a] += mass[n] * x[n * 3 + a]; V[a] += mass[n] * v[n * 3 + a]; }
  for (let n = 0; n < N; n++) for (let a = 0; a < 3; a++) { x[n * 3 + a] -= P[a] / M - S_rel.p[a]; v[n * 3 + a] -= V[a] / M - S_rel.v[a]; }
  sim.p.set(x);
  for (let n = 0; n < N; n++) releaseVmax = Math.max(releaseVmax, Math.hypot(v[n * 3], v[n * 3 + 1], v[n * 3 + 2]));
  if (releaseVmax > VCLAMP - 0.5) console.error(`${opt.id}: warning: release node speed ${releaseVmax.toFixed(1)} m/s at the solver's ${VCLAMP} m/s clamp (tyre top: speed + spin), the spin will be clipped`);
  sim.kick = null; // no kerb kick off the deck
  const psi = S_td.psi;
  sim.launch = { velocity: [...S_rel.v], forward: [Math.sin(psi), 0, Math.cos(psi)], right: [Math.cos(psi), 0, -Math.sin(psi)], speed: Math.hypot(S_rel.v[0], S_rel.v[2]) }; // rig warm start
  sim.wheelSpin = W.map((w, k) => ({ chunk: w.chunk, axle: w.axle, omega: omegaRel[k] }));
  sim.contact.fill(0); sim.impactV.fill(0);
  sim.updateActiveBoxes();
  sim.fprobe = new Float64Array(N * 3);
}

// ------------------------------------------------------------------------------------------------ bookkeeping
const nodePiece = sim.nodePiece, pieceDet = sim.pieceDetached, alive = sim.alive, x = sim.x, v = sim.v, fp = sim.fprobe;
const wheelPiece = W.map(w => sim.pieceIndex.get(w.piece));
const beamsOf = kind => W.map((_, k) => { const o = []; for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === kind && (wheelOf[sim.bi[b]] === k) !== (wheelOf[sim.bj[b]] === k)) o.push(b); return o; });
const springB = beamsOf(BEAM.SPRING), suspB = beamsOf(BEAM.SUSP); // (no JOINT beams reach the wheels)
const wRest = wchunk.map(ch => { const c = [0, 0, 0]; let s = 0; for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { s += mass[n]; for (let a = 0; a < 3; a++) c[a] += mass[n] * Rest[n * 3 + a]; } return c.map(v => v / s); });
const floor = new Uint8Array(N); for (let n = 0; n < N; n++) floor[n] = !isWheel[n] && Rest[n * 3 + 1] < 0.35 ? 1 : 0;
const vehicle = n => alive[n] && !pieceDet[nodePiece[n]];
const boxes = sim.boxes.map(B => { const cy = Math.abs(Math.cos(B.yaw)), sy = Math.abs(Math.sin(B.yaw)); return { name: B.name, lo: [B.c[0] - (cy * B.e[0] + sy * B.e[2]), B.c[1] - B.e[1], B.c[2] - (sy * B.e[0] + cy * B.e[2])], hi: [B.c[0] + (cy * B.e[0] + sy * B.e[2]), B.c[1] + B.e[1], B.c[2] + (sy * B.e[0] + cy * B.e[2])] }; });
const region = r => (r[1] >= 0.35 ? 'upper' : r[2] > 1.25 ? 'front' : r[2] < -1.3 ? 'rear' : Math.abs(r[0]) > 0.6 ? 'sill' : 'floor');
const R9 = new Float64Array(9);
const rd = (v, k = 1e5) => Math.round(v * k) / k, rdA = (a, k) => Array.from(a, v => rd(v, k));

function wheelSpinNow(k, wW) { // spin about the axle relative to the body (rad/s, + rolling forward), as CrashSim.keepWheelSpin measures it
  const wh = W[k], a0 = wh.axle[0] * 3, a1 = wh.axle[1] * 3, ch = wchunk[k], w = sim.w;
  if (!w[wh.axle[0]] || !w[wh.axle[1]]) return NaN;
  let ax = x[a1] - x[a0], ay = x[a1 + 1] - x[a0 + 1], az = x[a1 + 2] - x[a0 + 2]; const al = Math.hypot(ax, ay, az); ax /= al; ay /= al; az /= al;
  const cx = (x[a0] + x[a1]) / 2, cy = (x[a0 + 1] + x[a1 + 1]) / 2, cz = (x[a0 + 2] + x[a1 + 2]) / 2, ux = (v[a0] + v[a1]) / 2, uy = (v[a0 + 1] + v[a1 + 1]) / 2, uz = (v[a0 + 2] + v[a1 + 2]) / 2;
  let L = 0, J = 0;
  for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) {
    if (!w[n]) continue;
    const i = n * 3, m = 1 / w[n]; let rx = x[i] - cx, ry = x[i + 1] - cy, rz = x[i + 2] - cz;
    const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
    const vx = v[i] - ux, vy = v[i + 1] - uy, vz = v[i + 2] - uz;
    L += m * (ax * (ry * vz - rz * vy) + ay * (rz * vx - rx * vz) + az * (rx * vy - ry * vx)); J += m * (rx * rx + ry * ry + rz * rz);
  }
  return L / J - (wW[0] * ax + wW[1] * ay + wW[2] * az);
}
const springComp = k => { let s = 0, c = 0; for (const b of springB[k]) { if (sim.broken[b]) continue; const i = sim.bi[b] * 3, j = sim.bj[b] * 3; s += sim.L0o[b] - Math.hypot(x[i] - x[j], x[i + 1] - x[j + 1], x[i + 2] - x[j + 2]); c++; } return c ? s / c : NaN; };
const brokenOf = list => list.reduce((s, b) => s + sim.broken[b], 0);
const snapState = S => ({ t: S.t, p: [...S.p], q: [...S.q], v: [...S.v], w: [...S.w], comB: [...S.comB], M: rig.massNow });

// ------------------------------------------------------------------------------------------------ sampling
const C = { t: [], p: [], q: [], v: [], w: [], comB: [], M: [], tilt: [], boxDist: [], vmax: [], nclamp: [],
  wheel: { disp: [], comp: [], spin: [], hubY: [], clear: [], F: [], Flonglat: [], susp: [], spring: [], detached: [] },
  hull: { min: [], minRest: [], minPiece: [], floorMin: [], floorRest: [], F: [], Fc: [], n: [] } };
let nSince = 0, hullCand = new Int32Array(0);
function sample() {
  const S = rig.estimate(sim), t = sim.step / SUBSTEP_HZ;
  const m = quatToMat(S.q, R9), wW = rotate(m, S.w), inv = nSince ? 1 / nSince : 0;
  // one pass over the nodes: clearances, hull contact force, AABB of everything alive, next substeps' hull candidates
  let hmin = Infinity, hn = -1, fmin = Infinity, fn = -1, Fh = [0, 0, 0], Fc = [0, 0, 0], nh = 0;
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity], cand = [];
  let vmax = 0, ncl = 0; // vehicle node speeds: the solver clamps at VCLAMP (rolling tyre tops reach 2u)
  for (let n = 0; n < N; n++) {
    if (!alive[n]) continue;
    const i = n * 3;
    for (let a = 0; a < 3; a++) { const q = x[i + a]; if (q < lo[a]) lo[a] = q; if (q > hi[a]) hi[a] = q; }
    if (pieceDet[nodePiece[n]]) continue;
    const sp = v[i] * v[i] + v[i + 1] * v[i + 1] + v[i + 2] * v[i + 2];
    if (sp > vmax) vmax = sp;
    if (sp > (VCLAMP - 0.05) ** 2) ncl++;
    if (isWheel[n]) continue;
    const c = x[i + 1] + allow[n];
    if (c < hmin) { hmin = c; hn = n; }
    if (floor[n] && c < fmin) { fmin = c; fn = n; }
    if (c < 0.06) cand.push(n);
    if (fp[i] !== 0 || fp[i + 1] !== 0 || fp[i + 2] !== 0) {
      nh++; for (let a = 0; a < 3; a++) { Fh[a] += fp[i + a] * inv; Fc[a] += fp[i + 1] * inv * Rest[i + a]; }
    }
  }
  hullCand = Int32Array.from(cand);
  let bd = Infinity;
  for (const B of boxes) { let d2 = 0; for (let a = 0; a < 3; a++) { const g = Math.max(B.lo[a] - hi[a], lo[a] - B.hi[a], 0); d2 += g * g; } bd = Math.min(bd, Math.sqrt(d2)); }
  // wheels
  const disp = [], comp = [], spin = [], hubY = [], clear = [], F = [], FL = [], susp = [], spring = [], det = [];
  for (let k = 0; k < 4; k++) {
    const ch = wchunk[k], c = [0, 0, 0], Fw = [0, 0, 0]; let s = 0, cl = Infinity;
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) {
      const i = n * 3; for (let a = 0; a < 3; a++) Fw[a] += fp[i + a] * inv;
      if (!alive[n]) continue;
      s += mass[n]; for (let a = 0; a < 3; a++) c[a] += mass[n] * x[i + a]; cl = Math.min(cl, x[i + 1]);
    }
    const b = rotateT(m, [c[0] / s - S.p[0], c[1] / s - S.p[1], c[2] / s - S.p[2]]);
    disp.push([b[0] + S.comB[0] - wRest[k][0], b[1] + S.comB[1] - wRest[k][1], b[2] + S.comB[2] - wRest[k][2]].map(q => rd(q, 1e5)));
    comp.push(rd(springComp(k), 1e5)); spin.push(rd(wheelSpinNow(k, wW), 1e3));
    const a0 = W[k].axle[0] * 3, a1 = W[k].axle[1] * 3; hubY.push(rd((x[a0 + 1] + x[a1 + 1]) / 2, 1e5)); clear.push(rd(cl, 1e5));
    F.push(Fw.map(q => rd(q, 10)));
    // twin wheel axes: axle a, rolling direction (a_z, 0, -a_x)/h, lateral (a_x, 0, a_z)/h
    let ax = x[a1] - x[a0], az = x[a1 + 2] - x[a0 + 2]; const h = Math.hypot(ax, az) || 1; ax /= h; az /= h;
    FL.push([rd(Fw[0] * az - Fw[2] * ax, 10), rd(Fw[0] * ax + Fw[2] * az, 10)]);
    susp.push(brokenOf(suspB[k])); spring.push(brokenOf(springB[k])); det.push(pieceDet[wheelPiece[k]]);
  }
  fp.fill(0); nSince = 0;
  C.t.push(rd(t, 1e6)); C.p.push(rdA(S.p, 1e5)); C.q.push(rdA(S.q, 1e7)); C.v.push(rdA(S.v, 1e5)); C.w.push(rdA(S.w, 1e5)); C.comB.push(rdA(S.comB, 1e5));
  C.M.push(rd(rig.massNow, 1e3)); C.tilt.push(rd(Math.acos(Math.max(-1, Math.min(1, m[4]))) * deg, 1e3)); C.boxDist.push(rd(bd, 1e3));
  C.vmax.push(rd(Math.sqrt(vmax), 1e3)); C.nclamp.push(ncl);
  const Wc = C.wheel; Wc.disp.push(disp); Wc.comp.push(comp); Wc.spin.push(spin); Wc.hubY.push(hubY); Wc.clear.push(clear); Wc.F.push(F); Wc.Flonglat.push(FL); Wc.susp.push(susp); Wc.spring.push(spring); Wc.detached.push(det);
  const Hc = C.hull; Hc.min.push(rd(hmin, 1e5)); Hc.minRest.push(hn >= 0 ? rdA(Rest.subarray(hn * 3, hn * 3 + 3), 1e3) : null); Hc.minPiece.push(hn >= 0 ? car.pieces[nodePiece[hn]] : null);
  Hc.floorMin.push(rd(fmin, 1e5)); Hc.floorRest.push(fn >= 0 ? rdA(Rest.subarray(fn * 3, fn * 3 + 3), 1e3) : null);
  Hc.F.push(Fh.map(q => rd(q, 10))); Hc.Fc.push(Fh[1] > 1 ? Fc.map(q => rd(q / Fh[1], 1e3)) : null); Hc.n.push(nh);
  return S;
}

// ------------------------------------------------------------------------------------------------ run
rig.initFrames(sim); // warm-started rotation fits + the t = 0 estimate (else done on the first substep)
const releaseS = snapState(sample());
const perDense = Math.round(SUBSTEP_HZ / opt.denseHz), perSparse = Math.round(SUBSTEP_HZ / opt.hz), denseSteps = Math.round(opt.denseWin * SUBSTEP_HZ);
let tc = null, first = null, tdS = null, tdSpin = null, tdComp = null, hullFirst = null;
const wheelTd = [null, null, null, null];
const maxSteps = Math.round((tauRel + 1.0) * SUBSTEP_HZ);
let lastLog = Date.now();
while (sim.step < (tc === null ? maxSteps : tc + Math.round(opt.post * SUBSTEP_HZ))) {
  sim.substep();
  if (sim.step % 8 === 0) sim.stepParticles(8 * sim.dt);
  nSince++;
  const contact = sim.contact;
  // first contacts, every substep: tyres (4 x 98 nodes) and the hull candidates of the last sample (< 6 cm off)
  for (let k = 0; k < 4; k++) {
    if (wheelTd[k] !== null || pieceDet[wheelPiece[k]]) continue;
    for (let n = wchunk[k].nodeStart, e = n + wchunk[k].nodeCount; n < e; n++) if (contact[n] && alive[n]) { wheelTd[k] = sim.step; if (tc === null) { tc = sim.step; first = { kind: 'wheel', wheel: WHEEL_IDS[k], node: n }; } break; }
  }
  if (hullFirst === null) for (const n of hullCand) if (contact[n] && vehicle(n)) {
    const r = Array.from(Rest.subarray(n * 3, n * 3 + 3), q => rd(q, 1e3));
    hullFirst = { step: sim.step, node: n, piece: car.pieces[nodePiece[n]], rest: r, region: region(r) };
    if (tc === null) { tc = sim.step; first = { kind: 'hull', piece: hullFirst.piece, node: n }; }
    break;
  }
  if (tc === sim.step) { // touchdown snapshot at the end of the first-contact substep
    tdS = snapState(rig.estimate(sim)); const wW = rotate(quatToMat(tdS.q, R9), tdS.w);
    tdSpin = [0, 1, 2, 3].map(k => wheelSpinNow(k, wW)); tdComp = [0, 1, 2, 3].map(springComp);
  }
  const dense = tc === null || sim.step <= tc + denseSteps;
  if (sim.step % (dense ? perDense : perSparse) === 0) sample();
  if (!opt.quiet && Date.now() - lastLog > 10000) { lastLog = Date.now(); process.stdout.write(`\r${opt.id} t=${(sim.step / SUBSTEP_HZ).toFixed(2)} ${tc === null ? 'air' : 'ground'} ${((Date.now() - T0) / 1000).toFixed(0)}s   `); }
}
if (nSince) sample();
const simS = sim.step / SUBSTEP_HZ, computeS = (Date.now() - T0) / 1000;

// ------------------------------------------------------------------------------------------------ outcomes
const mm = new MassModel(), mp = mm.at(rig.prop.m), comBT = Array.from(mp.com);
const tOf = s => (s === null ? null : rd(s / SUBSTEP_HZ, 1e6));
const tcT = tOf(tc);
/** twin-ready initial state (spec/README §5.7 'state'): soft-body COM state + the same body origin with the twin's comB */
function twinState(S, extra = {}) {
  const m = quatToMat(S.q, new Float64Array(9)), origin = rotate(m, S.comB).map((r, a) => S.p[a] - r);
  const pT = rotate(m, comBT).map((r, a) => origin[a] + r), wW = rotate(m, S.w), dv = cross(wW, pT.map((q, a) => q - S.p[a]));
  return { type: 'state', t: rd(S.t, 1e6), ...extra, p: rdA(S.p, 1e7), q: rdA(S.q, 1e9), v: rdA(S.v, 1e7), w: rdA(S.w, 1e7), gas: rd(rig.prop.m, 1e4), leftDeck: true,
    comB: rdA(S.comB, 1e6), mass: rd(S.M, 1e3), origin: rdA(origin, 1e7), comB_twin: rdA(comBT, 1e6), p_twin: rdA(pT, 1e7), v_twin: rdA(S.v.map((q, a) => q + dv[a]), 1e7) };
}
const iPre = tcT === null ? 0 : C.t.reduce((b, t, i) => (Math.abs(t - (tcT - opt.precontact)) < Math.abs(C.t[b] - (tcT - opt.precontact)) && t <= tcT ? i : b), 0);
const stateAt = i => ({ t: C.t[i], p: C.p[i], q: C.q[i], v: C.v[i], w: C.w[i], comB: C.comB[i], M: C.M[i] });
const preS = stateAt(iPre);
const breaks = sim.events.breaks.map(e => ({ t: rd(e.t, 1e5), type: e.type, piece: e.piece || e.group }));
const detaches = breaks.filter(e => e.type === 'detach');
const wheelDetach = detaches.filter(e => WHEEL_IDS.includes(e.piece));
const firstRise = col => { const out = [null, null, null, null]; col.forEach((row, i) => row.forEach((c, k) => { if (c && out[k] === null) out[k] = C.t[i]; })); return out; };
const suspT = firstRise(C.wheel.susp), springT = firstRise(C.wheel.spring);
const last = C.t.length - 1, tilts = C.tilt;
let maxTilt = 0, maxTiltT = null; tilts.forEach((a, i) => { if (tcT !== null && C.t[i] >= tcT && a > maxTilt) { maxTilt = a; maxTiltT = C.t[i]; } });
// hull loads: impulse and peak of the vertical hull force, samples weighted by their interval
let hullImp = 0, hullPeak = 0; C.hull.F.forEach((F, i) => { const dt = i ? C.t[i] - C.t[i - 1] : 0; hullImp += F[1] * dt; hullPeak = Math.max(hullPeak, F[1]); });
// resting check: wheel normal forces / (M g) over the last 0.3 s
const tail = C.t.map((t, i) => i).filter(i => C.t[i] >= C.t[last] - 0.3 + 1e-9);
const wheelFy = i => C.wheel.F[i].reduce((s, F) => s + F[1], 0);
const restRatio = tail.length ? tail.reduce((s, i) => s + wheelFy(i) + C.hull.F[i][1], 0) / tail.length / (C.M[last] * 9.81) : null;
const wheelRatio = tail.length ? tail.reduce((s, i) => s + wheelFy(i), 0) / tail.length / (C.M[last] * 9.81) : null;
const achieved = tdS ? Object.fromEntries(Object.entries({ ...stateParams(tdS), gas: rig.prop.m / rig.prop.m0, spin: f.u > 0.5 ? tdSpin.reduce((s, o, k) => s + o * wheelR[k], 0) / 4 / stateParams(tdS).u : null }).map(([k, q]) => [k, q === null ? null : rd(q, 1e4)])) : null;
const finalP = stateParams(stateAt(last));
const flags = [];
if (tc === null) flags.push('no_touchdown');
if (first?.kind === 'hull') flags.push('hull_first');
if (maxTilt > 90) flags.push('rollover');
if (wheelDetach.length) flags.push('wheel_detach');
if (suspT.some(t => t !== null)) flags.push('arm_break');
if (springT.some(t => t !== null)) flags.push('spring_snap');
if (hullFirst) flags.push('hull_strike');
// solver speed clamp (informational: rolling tyre tops move at ~2u, so landings above ~20 m/s are clipped every substep)
const iClamp = C.nclamp.findIndex(c => c > 0), afterTd = C.t.map((t, i) => i).filter(i => tcT !== null && C.t[i] >= tcT);
if (iClamp >= 0) flags.push('speed_clamp');
const rigSum = rig.summary(sim);
// bent mount: summed plastic strain > MOUNT_PLASTIC or >= MOUNT_TORN torn beams in the region. Every landing yields the
// front spring mounts a little (nominal 2.4-2.8, clean landings of the first DOE <= 9.3, often 1 torn beam without a
// lasting wheel offset); from ~20 the wheel stays 3-10 cm off its place (a slipping 2 m/s touchdown: 35, 12 cm)
const MOUNT_PLASTIC = 20, MOUNT_TORN = 5;
// suspension mount damage: plastic strain / torn STRUCT beams of the body within 0.3 m (rest) of each wheel's arm and
// spring mounts (a bent mount moves the wheel for good without snapping an arm)
const mountDamage = W.map((_, k) => {
  const mounts = [...suspB[k], ...springB[k]].map(b => (wheelOf[sim.bi[b]] === k ? sim.bj[b] : sim.bi[b]));
  const near = n => mounts.some(m => Math.hypot(Rest[n * 3] - Rest[m * 3], Rest[n * 3 + 1] - Rest[m * 3 + 1], Rest[n * 3 + 2] - Rest[m * 3 + 2]) < 0.3);
  let mx = 0, sum = 0, torn = 0, nb = 0;
  for (let b = 0; b < sim.NB; b++) {
    if (sim.kind[b] !== BEAM.STRUCT || isWheel[sim.bi[b]] || !near(sim.bi[b]) || !near(sim.bj[b])) continue;
    nb++; mx = Math.max(mx, sim.plastic[b]); sum += sim.plastic[b]; torn += sim.broken[b];
  }
  return { max_plastic: rd(mx, 1e4), sum_plastic: rd(sum, 1e3), torn, beams: nb };
});
const finalDisp = C.wheel.disp[last], maxLat = [0, 1, 2, 3].map(k => rd(Math.max(...C.wheel.disp.slice(tcT === null ? 0 : C.t.findIndex(t => t >= tcT)).map(d => Math.abs(d[k][0]))), 1e4));
if (mountDamage.some(d => d.sum_plastic > MOUNT_PLASTIC || d.torn >= MOUNT_TORN)) flags.push('mount_damage');
const outcome = ['no_touchdown', 'rollover', 'wheel_detach', 'arm_break', 'spring_snap', 'mount_damage', 'hull_strike'].find(k => flags.includes(k)) || 'clean';
const summary = {
  id: opt.id, outcome, flags, target: f, achieved, t_release: rd(tauRel, 1e4), first_contact: first ? { t: tcT, ...first } : null,
  wheel_td: wheelTd.map(s => (s === null ? null : rd(s / SUBSTEP_HZ - tcT, 1e5))), springs_at_td_mm: tdComp ? tdComp.map(c => rd(c * 1000, 100)) : null,
  hull_strike: hullFirst ? { t: tOf(hullFirst.step), after_td: tcT === null ? null : rd(hullFirst.step / SUBSTEP_HZ - tcT, 1e5), piece: hullFirst.piece, rest: hullFirst.rest, region: hullFirst.region } : null,
  hull_impulse_Ns: rd(hullImp, 10), hull_peak_N: rd(hullPeak, 1), min_hull_clearance: rd(Math.min(...C.hull.min.slice(tcT === null ? 0 : C.t.findIndex(t => t >= tcT))), 1e5),
  min_floor_clearance: rd(Math.min(...C.hull.floorMin.slice(tcT === null ? 0 : C.t.findIndex(t => t >= tcT))), 1e5),
  max_spring_comp_mm: C.wheel.comp.reduce((a, row) => row.map((c, k) => Math.max(a[k], Number.isFinite(c) ? c * 1000 : -Infinity)), [-Infinity, -Infinity, -Infinity, -Infinity]).map(c => rd(c, 10)),
  wheel_detach: wheelDetach, detached: detaches.map(e => e.piece), arm_break_t: suspT, spring_snap_t: springT,
  susp_broken: C.wheel.susp[last], spring_broken: C.wheel.spring[last], mount_damage: mountDamage, final_wheel_disp_mm: finalDisp.map(d => d.map(q => rd(q * 1000, 10))), max_wheel_lat_mm: maxLat.map(q => rd(q * 1000, 10)),
  damage: { detached: rigSum.damage.detached, glass: rigSum.damage.glass, torn_tets: rigSum.damage.torn_tets, max_plastic: rigSum.damage.max_plastic, damage_sum: rigSum.damage.damage_sum },
  max_tilt_deg: rd(maxTilt, 100), max_tilt_t: maxTiltT, rollover: maxTilt > 90,
  final: { t: C.t[last], tilt: rd(finalP.tilt, 100), hs: rd(finalP.hs, 1e3), vs: rd(finalP.vs, 1e3), yaw_rate: rd(finalP.yaw_rate, 1e3), heading_change: rd(finalP.heading - S_td.psi * deg, 100), support_ratio: restRatio && rd(restRatio, 1e3), wheel_support_ratio: wheelRatio && rd(wheelRatio, 1e3) },
  speed_clamp: iClamp < 0 ? null : { first_t: C.t[iClamp], after_td: tcT === null ? null : rd(C.t[iClamp] - tcT, 1e5), share_after_td: afterTd.length ? rd(afterTd.filter(i => C.nclamp[i] > 0).length / afterTd.length, 1e3) : null, max_nodes: Math.max(...C.nclamp) },
  max_node_speed: rd(Math.max(...C.vmax), 1e3), min_box_dist: rd(Math.min(...C.boxDist), 1e3), rig: { touchdown_t: rigSum.metrics.touchdown_t ?? null, hull_strike_t: rig.strikeT === null ? null : rd(rig.strikeT, 1e5) },
  compute_s: rd(computeS, 10), sim_s: rd(simS, 1e4),
};
const fileHash = p => crypto.createHash('sha1').update(fs.readFileSync(repo(p))).digest('hex').slice(0, 12);
const out = {
  format: 'touchdown-drop@1', id: opt.id, created: new Date().toISOString(), spec_hash: specHash(),
  inputs: { settled: SETTLED, settled_sha1: fileHash(SETTLED), propulsion_sha1: fileHash('spec/propulsion.json'), solver_sha1: fileHash('src/softbody/solver.js'), actuators_sha1: fileHash('src/softbody/actuators.js') },
  target: f, opts: opt, site, wheel_ids: WHEEL_IDS,
  setup: {
    pose: opt.pose, mass: rd(M, 1e3), gas_kg: rd(rig.prop.m, 1e4), comB: rdA(comB, 1e6), inertia: rdA(I, 1e3), wheel_axle_inertia: rdA(Iax, 1e4), h_wheels: rdA(hW, 1e3),
    target_state: { p: rdA(S_td.p, 1e6), q: rdA(S_td.q, 1e9), v: rdA(S_td.v, 1e6), w: rdA(S_td.w, 1e6), heading_deg: rd(S_td.psi * deg, 1e4) },
    target_first_node: { node: S_td.firstNode, piece: car.pieces[nodePiece[S_td.firstNode]], rest: rdA(Rest.subarray(S_td.firstNode * 3, S_td.firstNode * 3 + 3), 1e3) },
    hull_clearance_at_target: rd(hullAtTd, 1e5), release_max_node_speed: rd(releaseVmax, 1e3), t_release: rd(tauRel, 1e4), release_clearance: rd(cs[kRel], 1e6), feasible: feasibleTarget, scan_valid_ms: valid,
    clearance_scan_mm: cs.filter((_, k) => k % 5 === 0).map(c => rd(c * 1000, 100)), wheel_phase_td: rd(aTd, 1e5), omega_td: rdA(omegaTd, 1e3), omega_release: rdA(omegaRel, 1e3), wheel_angle_release: rdA(angRel, 1e4),
  },
  release: twinState(releaseS, { t_before_contact: tcT === null ? null : rd(tcT, 1e6), spin: rdA(omegaRel, 1e3) }),
  precontact: twinState(preS, { t_before_contact: tcT === null ? null : rd(tcT - preS.t, 1e6) }),
  touchdown: tdS ? { ...twinState(tdS), spin: tdSpin.map(o => rd(o, 1e3)), springs_mm: tdComp.map(c => rd(c * 1000, 100)), params: achieved } : null,
  summary, events: breaks, rig: rigSum,
  samples: { n: C.t.length, note: 'F = mean contact force over (t[i-1], t[i]] (N, world); disp/comp in m; spin rad/s; see README', ...C },
};
fs.mkdirSync(opt.outDir, { recursive: true });
const file = path.join(opt.outDir, `${opt.id}.json.gz`);
fs.writeFileSync(file + '.tmp', zlib.gzipSync(Buffer.from(JSON.stringify(out)), { level: 6 }));
fs.renameSync(file + '.tmp', file); // atomic: the DOE runner treats an existing file as done
const a = achieved || {};
if (!opt.quiet) process.stdout.write('\r');
console.log(`${opt.id}: ${outcome}${flags.length > 1 ? ` [${flags.join(',')}]` : ''} · td ${tcT === null ? '-' : tcT.toFixed(3)} s (release ${tauRel.toFixed(3)}) vs ${a.vs?.toFixed(2)}/${f.vs} u ${a.u?.toFixed(1)}/${f.u} beta ${a.beta?.toFixed(1)}/${f.beta} theta ${a.theta?.toFixed(1)}/${f.theta} phi ${a.phi?.toFixed(1)}/${f.phi} spin ${a.spin?.toFixed(2) ?? '-'}/${f.spin} · springs@td ${summary.springs_at_td_mm?.map(c => c.toFixed(1)).join('/')} mm · hull ${hullFirst ? `${hullFirst.piece}/${hullFirst.region} +${summary.hull_strike.after_td.toFixed(3)} s` : '-'} · detach ${detaches.map(e => `${e.piece}@${e.t.toFixed(2)}`).join(' ') || '-'} · tilt max ${maxTilt.toFixed(1)} · box ${summary.min_box_dist.toFixed(1)} m · ${computeS.toFixed(0)} s for ${simS.toFixed(2)} s`);
console.log('SUMMARY ' + JSON.stringify(summary));
