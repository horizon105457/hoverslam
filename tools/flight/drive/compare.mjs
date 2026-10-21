// Twin vs soft body on the drive tests (tools/flight/drive/doe.mjs): every case replayed on the rigid twin from the
// soft body's state at `settle` (twin COM put on the soft body frame, measured wheel spins), with the same drive
// commands per control step, valves closed. Compared per 100 Hz sample until the first soft-body control-arm / spring
// break: speed along the heading u, lateral speed vlat (heading frame), yaw rate, heading change; RMS per case and
// overall, final heading / speed / stopping distance side by side.
// usage: node tools/flight/drive/compare.mjs run=experiments/runs/drive-doe/<tag> [vehicle=<json override | fit.json>] [only=id,..] [quiet=1]
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { parseArgs } from '../../lib/args.mjs';
import { RigidTwin, quatToMat } from '../../../src/flight/twin.js';
import { MassModel } from '../../../src/flight/massprops.js';
import { NA, NU, CONTROL_DT, DRIVE_IDX } from '../../../src/flight/contract.js';

export function loadDrive(run, only = null) {
  const man = JSON.parse(fs.readFileSync(path.join(run, 'manifest.json'), 'utf8'));
  return man.cases.filter(c => (!only || only.includes(c.id)) && fs.existsSync(path.join(run, `${c.id}.json.gz`))).map(c => prepDrive(JSON.parse(zlib.gunzipSync(fs.readFileSync(path.join(run, `${c.id}.json.gz`))))));
}

const mm = new MassModel();
/** Twin-ready init, rows from the start state on (t relative), end of the valid window (first arm / spring break). */
export function prepDrive(d) {
  const I = d.twin_init, R = quatToMat(I.q, new Float64Array(9)), cs = I.comB, ct = mm.at(I.gas).com;
  const dc = [ct[0] - cs[0], ct[1] - cs[1], ct[2] - cs[2]], rd = [R[0] * dc[0] + R[1] * dc[1] + R[2] * dc[2], R[3] * dc[0] + R[4] * dc[1] + R[5] * dc[2], R[6] * dc[0] + R[7] * dc[1] + R[8] * dc[2]];
  const w = I.w, wW = [R[0] * w[0] + R[1] * w[1] + R[2] * w[2], R[3] * w[0] + R[4] * w[1] + R[5] * w[2], R[6] * w[0] + R[7] * w[1] + R[8] * w[2]];
  const p = I.p.map((x, k) => x + rd[k]), v = [I.v[0] + wW[1] * rd[2] - wW[2] * rd[1], I.v[1] + wW[2] * rd[0] - wW[0] * rd[2], I.v[2] + wW[0] * rd[1] - wW[1] * rd[0]];
  const init = { type: 'state', p, q: I.q, v, w: I.w, gas: I.gas, leftDeck: true, wheel_spin: I.wheel_spin };
  const t0 = I.t, rows = d.rows.filter(r => r.t >= t0 - 1e-9);
  // valid window: until the first broken arm / spring (first_break; older runs without it: until a hull strike / kerb)
  const tEnd = Math.min(rows[rows.length - 1].t, d.first_break ?? Infinity, ...(d.events || []).filter(e => e.type === 'hull_strike' || e.type === 'kerb').map(e => e.t)) - t0;
  return { id: d.id, opts: d.opts, init, t0, rows: rows.map(r => ({ ...r, t: r.t - t0 })), tEnd, broken: d.broken, script: d.opts.script, firstBreak: d.first_break ?? null };
}

const headingOf = q => { const R = quatToMat(q, new Float64Array(9)); return Math.atan2(R[2], R[8]); };
export function replayDrive(tw, cs, { tEnd = cs.tEnd } = {}) {
  tw.reset(cs.init);
  const u = new Float64Array(NA), out = [], n = Math.round(tEnd / CONTROL_DT), ema = [0, 0, 0, 0];
  for (let k = 0; k <= n; k++) {
    if (k) { const L = tw.wheelLoad.lat; for (let j = 0; j < 4; j++) ema[j] = 0.5 * ema[j] + 0.5 * Math.abs(L[j]); } // land.js gnd_lat EMA
    const t = k * CONTROL_DT, R = quatToMat(tw.q, new Float64Array(9)), psi = Math.atan2(R[2], R[8]), h = [Math.sin(psi), Math.cos(psi)], rh = [Math.cos(psi), -Math.sin(psi)];
    out.push({ t, u: tw.v[0] * h[0] + tw.v[2] * h[1], vlat: tw.v[0] * rh[0] + tw.v[2] * rh[1], yaw_rate: tw.w[1], psi, p: [tw.p[0], tw.p[2]], spin: Array.from(tw.wheelSpin), ema: ema.slice() });
    if (k === n) break;
    let r = null; for (const row of cs.script) if (cs.t0 + t >= row[0] - 1e-9) r = row;
    u[DRIVE_IDX.brake] = r ? r[1] : 0; u[DRIVE_IDX.steer] = r ? r[2] : 0;
    tw.step(u);
  }
  return out;
}

/** Per-case errors: RMS of u, vlat, yaw rate, heading change over the window (soft samples at the twin's control grid). */
export function caseError(cs, tr) {
  const e = { u: 0, vlat: 0, yaw: 0, psi: 0 }, psi0s = cs.rows[0].psi, psi0t = tr[0].psi;
  let n = 0;
  for (const r of cs.rows) {
    const k = Math.round(r.t / CONTROL_DT); if (k >= tr.length || Math.abs(k * CONTROL_DT - r.t) > 0.004) continue;
    const T = tr[k], dps = (r.psi - psi0s) - (T.psi - psi0t);
    e.u += (r.u - T.u) ** 2; e.vlat += (r.vlat - T.vlat) ** 2; e.yaw += (r.yaw_rate - T.yaw_rate) ** 2; e.psi += dps * dps; n++;
  }
  for (const k in e) e[k] = Math.sqrt(e[k] / Math.max(1, n));
  return { ...e, n };
}

// drive part of the joint twin calibration (tools/flight/calib/fit.mjs drive=): scales of the compared quantities
export const DSCALE = { u: 0.3, vlat: 0.2, yaw: 0.05, psi: 0.035 };
/** Weighted squared error of one drive replay (time integral over the soft samples on the twin's control grid). */
export function driveLoss(cs, tr) {
  const parts = { u: 0, vlat: 0, yaw: 0, psi: 0 }, psi0s = cs.rows[0].psi, psi0t = tr[0].psi;
  let T = 0, last = 0;
  for (const r of cs.rows) {
    const k = Math.round(r.t / CONTROL_DT); if (k >= tr.length || Math.abs(k * CONTROL_DT - r.t) > 0.004) continue;
    const w = r.t - last; last = r.t; if (!(w > 0)) continue;
    const X = tr[k];
    if (!Number.isFinite(X.u) || !Number.isFinite(X.yaw_rate)) return { total: Infinity, T: 1, parts };
    let dps = (r.psi - psi0s) - (X.psi - psi0t); dps -= 2 * Math.PI * Math.round(dps / (2 * Math.PI));
    parts.u += w * ((r.u - X.u) / DSCALE.u) ** 2; parts.vlat += w * ((r.vlat - X.vlat) / DSCALE.vlat) ** 2;
    parts.yaw += w * ((r.yaw_rate - X.yaw_rate) / DSCALE.yaw) ** 2; parts.psi += w * (dps / DSCALE.psi) ** 2; T += w;
  }
  return { total: parts.u + parts.vlat + parts.yaw + parts.psi, T, parts };
}

if (process.argv[1] && process.argv[1].endsWith('compare.mjs')) {
  const a = parseArgs(), run = path.resolve(a.run);
  let vehicle = null;
  if (a.vehicle) { const v = a.vehicle.trim().startsWith('{') ? JSON.parse(a.vehicle) : JSON.parse(fs.readFileSync(a.vehicle, 'utf8')); vehicle = v.vehicle || v; }
  const tw = new RigidTwin({ vehicle }), cases = loadDrive(run, a.only ? a.only.split(',') : null), deg = 180 / Math.PI;
  const tot = { u: 0, vlat: 0, yaw: 0, psi: 0 };
  for (const cs of cases) {
    const tr = replayDrive(tw, cs), e = caseError(cs, tr), S = cs.rows[cs.rows.length - 1], T = tr[tr.length - 1];
    for (const k in tot) tot[k] += e[k] ** 2 / cases.length;
    const dS = Math.hypot(S.p[0] - cs.rows[0].p[0], S.p[2] - cs.rows[0].p[2]), dT = Math.hypot(T.p[0] - tr[0].p[0], T.p[1] - tr[0].p[1]);
    // late failure criterion (land.js gnd_lat_*): the planning twin's peak wheel lateral-load EMA up to the soft break
    const em = Math.max(...tr.flatMap(r => r.ema));
    console.log(`${cs.id.padEnd(12)} ema ${(em / 1000).toFixed(2)} kN${cs.firstBreak != null ? ` | break @${(cs.firstBreak - cs.t0).toFixed(2)} s` : ''} | rms u ${e.u.toFixed(2)} vlat ${e.vlat.toFixed(2)} yaw ${e.yaw.toFixed(3)} psi ${(e.psi * deg).toFixed(1)}° | end (soft/twin) u ${S.u.toFixed(1)}/${T.u.toFixed(1)} psi ${((S.psi - cs.rows[0].psi) * deg).toFixed(1)}/${((T.psi - tr[0].psi) * deg).toFixed(1)}° yaw ${S.yaw_rate.toFixed(2)}/${T.yaw_rate.toFixed(2)} path ${dS.toFixed(1)}/${dT.toFixed(1)} m${cs.broken?.length ? ` (soft broke ${cs.broken.map(b => b.wheel.slice(6) + ':' + b.arms + 'a' + b.springs + 's').join(' ')})` : ''}`);
  }
  console.log(`overall rms u ${Math.sqrt(tot.u).toFixed(2)} vlat ${Math.sqrt(tot.vlat).toFixed(2)} yaw ${Math.sqrt(tot.yaw).toFixed(3)} psi ${(Math.sqrt(tot.psi) * deg).toFixed(1)}°`);
}
