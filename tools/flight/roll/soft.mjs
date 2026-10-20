// Barrel-roll report metrics of soft-body recordings (tools/sim/flight.mjs output, public/sims/*.sim or outDir runs).
// usage: node tools/flight/roll/soft.mjs <file.sim> [...] [h_jump=2.0] [timeline=0.1]
import fs from 'fs';
import zlib from 'zlib';
import { parseArgs } from '../../lib/args.mjs';
import { unpackRecording } from '../../../src/softbody/recording.js';
import { GROUPS } from '../../../src/flight/contract.js';
import { rollMetrics, fromSoft, rollSeries, attitude } from './metrics.mjs';

const a = parseArgs();
for (const f of Object.keys(a).filter(k => a[k] === true && k.endsWith('.sim'))) {
  const raw = zlib.gunzipSync(fs.readFileSync(f)), { meta } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
  const src = fromSoft(meta), m = rollMetrics(src, a.h_jump != null ? { h_jump: +a.h_jump } : {}), F = meta.flight;
  if (a.timeline) {
    const L = meta.propulsion, every = Math.max(1, Math.round(+a.timeline * L.hz)), { phi } = rollSeries(src);
    const grp = (k, g) => GROUPS[g].reduce((s, i) => s + L.thrust[k][i], 0) / 1000;
    console.log('   t     y      z     vy     vz    roll   tilt  pitch   wz    gas  bottom  rear  side  phase');
    for (let k = 0; k < L.n; k += every) {
      const s = L.state[k], at = attitude(s.slice(3, 7));
      console.log([L.t[k].toFixed(2), s[1].toFixed(2), s[2].toFixed(2), s[8].toFixed(2), s[9].toFixed(2), Number.isNaN(phi[k]) ? '-' : (phi[k] * 57.2958).toFixed(0), at.tilt.toFixed(0), at.pitch.toFixed(1), s[12].toFixed(2),
        (src.m0 - L.tank[k][0]).toFixed(1), grp(k, 'bottom').toFixed(1), grp(k, 'rear').toFixed(1), grp(k, 'side').toFixed(1), L.phase[k]].map(x => String(x).padStart(6)).join(' '));
    }
  }
  console.log(JSON.stringify({ file: f, ...m, fails: F.metrics.fails, max_tilt_hold_deg: F.metrics.max_tilt_hold_deg, hull_strike_any: F.metrics.hull_strike_any, strike_t: F.events.find(e => e.type === 'hull_strike')?.t ?? null,
    detached: F.damage.detached.length, glass: F.damage.glass.length, max_plastic: F.damage.max_plastic, plans: F.controllerStats?.plans, plan_s: F.controllerStats?.plan_s, compute_s: F.computeSeconds,
    events: F.events.map(e => `${e.type}@${e.t.toFixed(2)}`).join(' ') }));
}
