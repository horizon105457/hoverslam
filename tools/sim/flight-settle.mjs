// Settled pose of the car WITH the cold-gas kit (bottles full, nozzles, plumbing: ~312 kg on top of the
// lattice), resting on its springs. Every flight run (tools/sim/flight.mjs) starts from it, exactly like the
// crash presets start from public/assets/settled.bin.
// usage: node tools/sim/flight-settle.mjs [out=public/assets/settled-coldgas.bin]
import { loadCarNode } from '../lib/car.mjs';
import { parseArgs } from '../lib/args.mjs';
import { writeSettled } from '../lib/sims.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
import { ThrusterRig } from '../../src/softbody/actuators.js';
import { BEAM, SPRING } from '../../src/car/carPrep.js';
const args = parseArgs();
const car = await loadCarNode({ h: 0.065 });
const sim = new CrashSim(car, {});
const rig = new ThrusterRig(car).applyMass(sim, 1);
const t = Date.now();
const pos = sim.settle();
writeSettled(args.out || 'public/assets/settled-coldgas.bin', car, pos);
// report: spring sag and anything that broke under the extra weight
// (rear = the kit's coil-overs, see spec/propulsion.json suspension)
let sag = 0, ns = 0, broken = 0;
const ax = { front: [0, 0], rear: [0, 0] }, rear = new Set(rig.rearSprings);
for (let b = 0; b < sim.NB; b++) {
  broken += sim.broken[b];
  if (sim.kind[b] !== BEAM.SPRING) continue;
  const i = sim.bi[b] * 3, j = sim.bj[b] * 3, d = Math.hypot(pos[i] - pos[j], pos[i + 1] - pos[j + 1], pos[i + 2] - pos[j + 2]) - sim.L0o[b];
  sag += d; ns++;
  const a = ax[rear.has(b) ? 'rear' : 'front']; a[0] += d; a[1]++;
}
let M = 0, cy = 0; for (let n = 0; n < sim.N; n++) { M += rig.mass[n]; cy += rig.mass[n] * pos[n * 3 + 1]; }
console.log(`settled ${car.nodes.count} nodes in ${Date.now() - t} ms · mass ${M.toFixed(1)} kg (kit ${(rig.kitDry + rig.prop.m0).toFixed(1)}) · COM y ${(cy / M).toFixed(3)} · spring change ${(sag / ns * 1000).toFixed(1)} mm (front ${(ax.front[0] / ax.front[1] * 1000).toFixed(1)}, rear ${(ax.rear[0] / ax.rear[1] * 1000).toFixed(1)}, travel ${SPRING.travel * 1000} mm) · broken beams ${broken}`);
