// Soft-body drive test: the kit car (ThrusterRig, no thrust) rolling on open street, then a scripted brake / steer
// schedule through its brakes and steering (src/softbody/drive.js). The reference data for the rigid twins' drive
// model (spec/README.md 5.13): straight braking, steer steps, combined, coasting.
// usage: node tools/flight/drive/test.mjs u=20 script='[[0.5,1,0]]' [gas=0.65 dur=5 x0=60 z0=13 heading=0 hz=100
//          id=drive outDir=experiments/runs/drive-scratch joints=0.55 settle=0.3]
//   script: [[t, brake, steer], ...] piecewise constant from t on (sim time; before the first row 0, 0)
//   heading: deg about +Y of the car's forward from -X (0 = along the viaduct towards -X, the launch direction)
// Placement: the settled kit pose (settled-coldgas.bin) put on the street at (x0, z0), velocity u along the heading,
// wheels rolling (CrashSim.setup's launch placement moved off the deck; no kerb kick). Records 100 Hz: rig estimate
// (p q v w, phase), drive state, wheel spins, speed / heading / yaw rate / lateral speed, and `twin_init` at t = settle
// (an explicit-state twin init with gas and measured wheel spins) for the twin replay (tools/flight/drive/compare.mjs).
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { loadCarNode } from '../../lib/car.mjs';
import { parseArgs } from '../../lib/args.mjs';
import { readSettled } from '../../lib/sims.mjs';
import { CrashSim } from '../../../src/softbody/solver.js';
import { ThrusterRig } from '../../../src/softbody/actuators.js';
import { DECK } from '../../../src/scene/layout.js';
import { NU } from '../../../src/flight/contract.js';
import { quatToMat } from '../../../src/core/quat.js';

const a = parseArgs();
const num = (k, d) => (a[k] !== undefined ? +a[k] : d);
const u0 = num('u', 20), gas = num('gas', 0.65), dur = num('dur', 5), x0 = num('x0', 60), z0 = num('z0', 13), hd = num('heading', 0), settleT = num('settle', 0.3);
const script = a.script ? JSON.parse(a.script) : [];
const id = a.id || 'drive', outDir = a.outDir || 'experiments/runs/drive-scratch';
const car = await loadCarNode({ h: 0.065 });
const settled = readSettled('public/assets/settled-coldgas.bin', car);
const sim = new CrashSim(car, { speed: u0 * 3.6, angle: hd, spin: [0, 0, 0], seed: 1, joints: num('joints', 0.55), duration: dur, recordHz: num('hz', 100) });
const ctl = { name: 'drive-script', reset() {}, act(s) { const o = new Float64Array(NU + 2); let r = null; for (const row of script) if (s.t >= row[0] - 1e-9) r = row; if (r) { o[NU] = r[1]; o[NU + 1] = r[2]; } return o; } };
const task = { id: 'drive-test', arm: 'start', success: { upright_hold: 0.2, stop_speed: 0.3 } };
const rig = new ThrusterRig(car, { controller: ctl, controllerName: 'drive-script', task });
rig.attach(sim);
rig.prop.m = rig.prop.m0 * gas; rig.prop.step(new Float64Array(NU), 0); rig.applyMass(sim, gas);
sim.setup(settled);
// move the launch placement off the deck onto the street, no kick
{
  const N = sim.N, x = sim.x, w = sim.w; let mx = 0, mz = 0, M = 0;
  for (let n = 0; n < N; n++) { if (!w[n]) continue; const m = 1 / w[n]; mx += m * x[n * 3]; mz += m * x[n * 3 + 2]; M += m; }
  const dx = x0 - mx / M, dz = z0 - mz / M;
  for (let n = 0; n < N; n++) { x[n * 3] += dx; x[n * 3 + 1] -= DECK.top; x[n * 3 + 2] += dz; }
  sim.kick = null; sim.updateActiveBoxes();
}
const rows = [], t0 = Date.now(), suspBeams = [];
for (let b = 0; b < sim.NB; b++) if (sim.kind[b] === 4 || sim.kind[b] === 5) suspBeams.push(b); // BEAM.SUSP, BEAM.SPRING
let twinInit = null, firstBreak = null;
const sample = s => {
  const S = rig.estimate(s), m = quatToMat(S.q, new Float64Array(9));
  const fwd = [m[2], m[5], m[8]], psi = Math.atan2(fwd[0], fwd[2]), h = [Math.sin(psi), Math.cos(psi)], rh = [Math.cos(psi), -Math.sin(psi)];
  const D = rig.drive, spin = D.wheelSpins(s, rig.wW);
  rows.push({ t: +s.time.toFixed(4), p: [...S.p], q: [...S.q], v: [...S.v], w: [...S.w], u: S.v[0] * h[0] + S.v[2] * h[1], vlat: S.v[0] * rh[0] + S.v[2] * rh[1], psi, yaw_rate: S.w[1], tilt: S.tilt,
    brake: D.brake, steer: D.steer, cmd: [D.cmd.brake, D.cmd.steer], spin, M: rig.massNow });
  if (firstBreak === null && suspBeams.some(b => s.broken[b])) firstBreak = s.time;
  if (!twinInit && s.time >= settleT - 1e-9) twinInit = { type: 'state', t: s.time, p: [...S.p], q: [...S.q], v: [...S.v], w: [...S.w], gas: rig.prop.m, leftDeck: true, comB: [...S.comB], wheel_spin: spin };
};
sim.run(dur, sample);
const flight = rig.summary(sim), last = rows[rows.length - 1];
const out = { format: 'drive-test@1', id, opts: { u: u0, gas, dur, x0, z0, heading: hd, script, settle: settleT }, twin_init: twinInit, rows, events: flight.events, damage: flight.damage, broken: brokenSusp(sim), first_break: firstBreak, compute_s: (Date.now() - t0) / 1000 };
fs.mkdirSync(outDir, { recursive: true });
fs.writeFileSync(path.join(outDir, `${id}.json.gz`), zlib.gzipSync(Buffer.from(JSON.stringify(out))));
console.log(`${id}: u ${u0} -> ${last.u.toFixed(2)} m/s, heading ${((last.psi - rows[0].psi) * 57.3).toFixed(1)} deg, yaw ${last.yaw_rate.toFixed(3)} rad/s, path ${Math.hypot(last.p[0] - rows[0].p[0], last.p[2] - rows[0].p[2]).toFixed(1)} m, broken ${JSON.stringify(out.broken)}, events ${flight.events.map(e => `${e.type}@${e.t.toFixed(2)}`).join(' ')} (${out.compute_s.toFixed(0)} s)`);

function brokenSusp(sim) {
  const W = sim.car.wheels, wheelOf = new Map(W.map((w, i) => [w.chunk, i])), o = W.map(w => ({ wheel: w.piece, arms: 0, springs: 0 }));
  for (let b = 0; b < sim.NB; b++) {
    if (!sim.broken[b]) continue;
    const k = sim.kind[b]; if (k !== 4 && k !== 5) continue; // BEAM.SUSP, BEAM.SPRING
    const i = sim.bi[b], j = sim.bj[b], wi = wheelOf.get(sim.isWheel[i] ? sim.chunk[i] : sim.chunk[j]);
    if (wi !== undefined) o[wi][k === 4 ? 'arms' : 'springs']++;
  }
  return o.filter(x => x.arms || x.springs);
}
