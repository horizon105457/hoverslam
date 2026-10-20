// Web Worker: runs a crash scenario and streams encoded frames back to the page.
import { CrashSim } from './solver.js';
import { RecordingEncoder, finalMeta } from './recording.js';

let cancelled = false;

self.onmessage = async e => {
  const msg = e.data;
  if (msg.type === 'cancel') { cancelled = true; return; }
  if (msg.type !== 'run') return;
  cancelled = false;
  const { car, settled, scenario } = msg;
  const sim = new CrashSim(car, scenario);
  sim.setup(settled);
  const enc = new RecordingEncoder(car);
  let batch = [], k = 0;
  const transfer = f => [f.xf.buffer, f.local.buffer, f.pp.buffer, f.pq.buffer, ...(f.damage ? [f.damage.buffer, f.scrape.buffer] : [])];
  const flush = () => {
    if (!batch.length) return;
    self.postMessage({ type: 'frames', frames: batch, progress: sim.time / scenario.duration, events: liveEvents(sim) }, batch.flatMap(transfer));
    batch = [];
  };
  batch.push(enc.encode(sim, true));
  const slice = 0.05;
  const t0 = performance.now();
  while (sim.time < scenario.duration - 1e-6 && !cancelled) {
    sim.run(Math.min(slice, scenario.duration - sim.time), s => { k++; batch.push(enc.encode(s, k % 4 === 0)); });
    flush();
    // let the cancel message through
    await new Promise(r => setTimeout(r, 0));
  }
  if (cancelled) { self.postMessage({ type: 'cancelled' }); return; }
  self.postMessage({ type: 'done', meta: finalMeta(sim, enc, { label: scenario.label || '自定义', computeSeconds: (performance.now() - t0) / 1000 }) });
};

// break/tear information needed to render partially computed runs
function liveEvents(sim) {
  return { groupTime: Array.from(sim.groupTime), tearTime: sim.tearTime, groups: sim.groups.map(g => ({ name: g.name, kind: g.kind, breakT: g.breakT, crackT: g.crackT, crackAt: g.crackAt })), particles: { count: sim.np, kind: Array.from(sim.pk.subarray(0, sim.np)), spawn: Array.from(sim.pt.subarray(0, sim.np)), size: Array.from(sim.pSize.subarray(0, sim.np * 3)) }, dust: sim.events.dust.slice(), sparks: sim.events.sparks.slice(), scrape: sim.events.scrape.slice(), breaks: sim.events.breaks.slice() };
}
