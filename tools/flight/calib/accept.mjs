// Closed-loop acceptance on the soft body: the pure MPC (src/flight/mpc, no pd) flies untuned cases (launch preset x
// seed x manoeuvre) through tools/sim/flight.mjs, at most `par` processes at a time, then success rate with a
// Wilson 95 % interval per set and overall. `root` runs another source tree (e.g. a snapshot with the old twin) with
// the same runner, so before / after compare the same cases. Finished cases are skipped (resumable).
// usage: node tools/flight/calib/accept.mjs [sets=land,roll] [seeds=1,2,3] [par=8] [dur=7] [root=<tree>]
//        [outDir=experiments/runs/accept/<tag>] [tag=now] [cfg=<json merged into every MPC config>] [summary=1]
// Sets stop / rollstop (brakes + steering, spec/drive.json): task escape-land-stop with the drive MPC of
// experiments/configs/drive-stop.json (`mpc-drive`), run for their own duration (11 s: touchdown + stop + margin).
import { spawn } from 'child_process';
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { parseArgs } from '../../lib/args.mjs';
import { repo } from '../../lib/paths.mjs';

// the acceptance sets: escape-soft-landing with the default pure MPC on every launch preset, and the barrel roll of
// experiments/reports/barrel-roll (experiments/configs/roll-final.json `mpc-roll`) from the flat preset
export const ROLL = { horizon: 4.0, first_passes: 30, cem: { sigma: 0.2 }, channel_scale: { heave: 2, pitch: 0.5, roll: 0.8, pitch_r: 0.2, yaw: 0.2, surge: 0.2, sway: 0.2 }, optimizer: 'hybrid', hybrid: { stages: [{ type: 'cem', sigma: 0.2, iters: 1 }, { type: 'cem', sigma: 0.07, iters: 2 }] }, objective: { name: 'roll', jump_first: true, w_climb: 3, w_deck: 2, w_gas: 0.6, w_inv: 2, w_jump: 200, w_roll_run: 0, w_wrong: 20 } };
const DRIVE_CFG = (() => { const c = JSON.parse(fs.readFileSync(repo('experiments/configs/drive-stop.json'), 'utf8')).controllers.find(x => x.label === 'mpc-drive'); const { name, label, ...cfg } = c; return cfg; })();
export const SETS = {
  land: { presets: ['flat', 'nose', 'rear', 'side', 'roof', 'corner'], seeds: [1, 2, 3], cfg: {} },
  roll: { presets: ['flat'], seeds: [1, 2, 3, 4, 5, 6, 7, 8], cfg: ROLL },
  stop: { presets: ['flat', 'nose', 'rear', 'side', 'roof', 'corner'], seeds: [1, 2, 3], cfg: DRIVE_CFG, task: 'escape-land-stop', dur: 11 },
  rollstop: { presets: ['flat'], seeds: [1, 2, 3, 4, 5, 6, 7, 8], cfg: { ...ROLL, drive: true, objective: { ...ROLL.objective, ...DRIVE_CFG.objective, name: 'roll' } }, task: 'escape-land-stop', dur: 11 },
};

/** Wilson score interval (95 %) for k successes out of n. */
export function wilson(k, n, z = 1.96) {
  if (!n) return [0, 1];
  const p = k / n, d = 1 + z * z / n, c = (p + z * z / (2 * n)) / d, h = (z * Math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d;
  return [Math.max(0, c - h), Math.min(1, c + h)];
}

const merge = (a, b) => { const o = { ...a }; for (const [k, v] of Object.entries(b || {})) o[k] = v && typeof v === 'object' && !Array.isArray(v) && typeof a[k] === 'object' ? merge(a[k], v) : v; return o; };

const a = parseArgs();
const root = a.root ? path.resolve(a.root) : repo('.'), dur = a.dur || '7', par = +(a.par || 8);
const outDir = path.resolve(a.outDir || repo(`experiments/runs/accept/${a.tag || new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '')}`));
const extra = a.cfg ? JSON.parse(a.cfg) : {};
const jobs = (a.sets || 'land,roll').split(',').flatMap(set => {
  const S = SETS[set], seeds = a.seeds ? a.seeds.split(',').map(Number) : S.seeds;
  return S.presets.flatMap(p => seeds.map(seed => ({ set, preset: p, seed, name: `${set}-${p}-s${seed}`, ctl: `mpc:${JSON.stringify(merge(S.cfg, extra))}`, task: S.task || null, dur: a.dur || S.dur || dur })));
});
fs.mkdirSync(outDir, { recursive: true });
const done = j => fs.existsSync(path.join(outDir, `flight-${j.name}.json`));
if (!a.summary) {
  const todo = jobs.filter(j => !done(j)), t0 = Date.now();
  let next = 0, live = 0;
  await new Promise(fin => {
    if (!todo.length) return fin();
    const go = () => {
      while (live < par && next < todo.length) {
        const j = todo[next++], args = [path.join(root, 'tools/sim/flight.mjs'), `preset=${j.preset}`, `seed=${j.seed}`, `controller=${j.ctl}`, `out=${j.name}`, `dur=${j.dur}`, `outDir=${outDir}`, ...(j.task ? [`task=${j.task}`] : [])];
        const log = fs.openSync(path.join(outDir, `${j.name}.log`), 'w');
        fs.writeSync(log, `cd ${root} && node ${args.map(x => (x.includes('{') ? `'${x}'` : x)).join(' ')}\n`);
        const ch = spawn('node', args, { cwd: root, stdio: ['ignore', log, log] });
        live++;
        ch.on('exit', code => { live--; fs.closeSync(log); console.log(`${j.name}: exit ${code} (${((Date.now() - t0) / 1000).toFixed(0)} s)`); if (next >= todo.length && live === 0) fin(); else go(); });
      }
    };
    go();
  });
}
// summary: one row per case, then success with Wilson intervals per set and overall. Two verdicts:
//   task      the task's own success (spec/tasks: soft touchdown, upright hold, no hull strike in the hold)
//   touchdown no touchdown-caused failure within TD_WIN s of the touchdown: a hull strike or a wheel coming off
//             counts unless the first failure happened at a kerb (COM within KERB m of a street box's footprint: rolling
//             into the sidewalk at ~18 m/s without brakes is a different problem from the twin/soft touchdown gap)
const TD_WIN = 2.0, KERB = 2.5;
const STREET = JSON.parse(fs.readFileSync(path.join(root, 'spec/generated/colliders.json'), 'utf8')).boxes.filter(b => b.c[1] + b.e[1] < 1.0);
const kerbDist = (x, z) => Math.min(...STREET.map(b => { const cy = Math.cos(b.yaw || 0), sy = Math.sin(b.yaw || 0), dx = x - b.c[0], dz = z - b.c[2];
  return Math.max(Math.abs(cy * dx - sy * dz) - b.e[0], Math.abs(sy * dx + cy * dz) - b.e[2]); }));
const { unpackRecording } = await import(path.join(root, 'src/softbody/recording.js'));
const rows = jobs.filter(done).map(j => {
  const F = JSON.parse(fs.readFileSync(path.join(outDir, `flight-${j.name}.json`), 'utf8')), m = F.metrics, ev = F.events || [];
  const strike = ev.find(e => e.type === 'hull_strike'), td = m.touchdown_t;
  // break events and the COM track from the recording
  const raw = zlib.gunzipSync(fs.readFileSync(path.join(outDir, `flight-${j.name}.sim`))), { meta } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
  const L = meta.propulsion, at = t => { let k = 0; while (k < L.n - 1 && L.t[k] < t) k++; return L.state[k]; };
  const wheelOff = (meta.events.breaks || []).filter(b => b.type === 'detach' && /^wheel/.test(b.piece)).sort((a, b) => a.t - b.t)[0] || null;
  const fails = [strike && { what: 'hull_strike', t: strike.t }, wheelOff && { what: `detach ${wheelOff.piece}`, t: wheelOff.t }].filter(Boolean)
    .map(f => { const s = at(f.t); return { ...f, after_td: td != null ? +(f.t - td).toFixed(2) : null, kerb: kerbDist(s[0], s[2]) < KERB }; });
  // the first failure decides: a wheel lost at the kerb and the hull strike that follows are the kerb's
  const first = fails.slice().sort((a, b) => a.t - b.t)[0];
  const tdFail = td == null ? { what: 'no touchdown' } : first && first.after_td <= TD_WIN && !first.kerb ? first : null;
  // stop verdict (stop sets): landed softly, stopped, no kerb and no failure at all after the touchdown (kerb or not)
  const stopOk = j.task ? !!m.success && !!m.stopped && !m.kerb && !fails.length : null;
  return { ...j, ctl: undefined, success: !!m.success, td_ok: !!m.success && !tdFail, td_fail: tdFail, stop_ok: stopOk, stopped: m.stopped ?? null, stop_time: m.stop_time ?? null, stop_distance: m.stop_distance ?? null, kerb: m.kerb ?? null, fails: m.fails || [], late: fails, td_t: td, vy: m.touchdown_vertical_speed, vh: m.touchdown_horizontal_speed,
    tilt: m.touchdown_tilt_deg, strike_after_td: strike && td != null ? +(strike.t - td).toFixed(2) : null, detached: F.damage?.detached || [], gas: m.gas_used_kg, tilt_hold: m.max_tilt_hold_deg };
});
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify({ root, dur, extra, rows }, null, 1));
const pct = v => (100 * v).toFixed(0);
const lateS = r => r.late.map(f => `${f.what}+${f.after_td}${f.kerb ? '(kerb)' : ''}`).join(' ') || '-';
for (const r of rows) console.log(`${r.name.padEnd(18)} task ${r.success ? 'OK  ' : 'FAIL'} td ${r.td_ok ? 'OK  ' : 'FAIL'}${r.stop_ok !== null ? ` stop ${r.stop_ok ? 'OK  ' : 'FAIL'}` : ''} ${(r.fails.join(',') || '-').padEnd(12)} td ${r.td_t?.toFixed(2) ?? '-'} vy ${r.vy?.toFixed(2) ?? '-'} vh ${r.vh?.toFixed(1) ?? '-'} tilt ${r.tilt?.toFixed(1) ?? '-'}${r.stopped ? ` stop ${r.stop_time?.toFixed(1)} s ${r.stop_distance?.toFixed(0)} m` : ''}${r.kerb ? ' KERB' : ''} | ${lateS(r)} | gas ${r.gas?.toFixed(1) ?? '-'}`);
for (const set of [...new Set(rows.map(r => r.set)), 'all']) {
  const R = rows.filter(r => set === 'all' || r.set === set);
  for (const [lab, key] of [['task', 'success'], ['touchdown', 'td_ok'], ...(R.some(r => r.stop_ok !== null) ? [['stop', 'stop_ok']] : [])]) {
    const k = R.filter(r => r[key]).length, [lo, hi] = wilson(k, R.length);
    console.log(`${set} ${lab}: ${k}/${R.length} = ${pct(k / (R.length || 1))} % (95 % CI ${pct(lo)}-${pct(hi)} %)`);
  }
}
