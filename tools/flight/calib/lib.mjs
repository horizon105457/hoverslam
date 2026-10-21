// Twin touchdown calibration: soft-body landing experiments (tools/flight/touchdown, `touchdown-drop@1`) replayed on
// the rigid twin (src/flight/twin.js with a `vehicle` override, spec/README.md §5.12) and compared sample by sample.
// The twin starts from the soft body's own state 0.05 s before the first tyre contact (`precontact`, twin COM on the
// soft body frame, measured wheel spins), runs with closed valves, and is read every integrator step; soft samples
// are interval means (forces) or instants (pose), so twin values are averaged / picked to match.
// Compared (each over the case's valid window, weighted by sample interval, scaled by SCALE):
//   hub   height above the street of the four body-fixed wheel-centre points (pose: heave, pitch, roll at the axles)
//   floor lowest underbody clearance (soft: attached non-wheel nodes with rest y < 0.35; twin: hull + floor points)
//   N / lat / long   per-wheel ground contact force: normal, lateral, rolling direction (twin's own axes, §5.5)
//   vlat / yaw       lateral velocity in the heading frame, yaw rate (the rolling-direction dynamics after touchdown)
// The window ends at the first soft-body failure that the rigid twin cannot represent (control arm / spring broken,
// wheel detached) and 0.15 s after a hull strike (the soft body then crushes / scrapes plastically).
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { RigidTwin, quatToMat } from '../../../src/flight/twin.js';
import { GENERATED } from '../../../src/core/spec.js';
import { NU } from '../../../src/flight/contract.js';

export const SCALE = { hub: 0.01, floor: 0.01, N: 1500, lat: 1000, long: 1000, vlat: 0.2, yaw: 0.05 };
export const WEIGHT = { hub: 1, floor: 2, N: 0.5, lat: 1, long: 0.5, vlat: 1, yaw: 1 };
// experiment override (JSON, merged), e.g. CALIB_WEIGHT='{"lat":4}' (worker threads inherit the environment)
if (process.env.CALIB_WEIGHT) Object.assign(WEIGHT, JSON.parse(process.env.CALIB_WEIGHT));
const WHEELS = GENERATED.massprops.wheels;

/** Cases of a touchdown-doe run (or a list of case files): compact arrays for the replay + the summary. */
export function loadCases(run, { splits = null } = {}) {
  const man = JSON.parse(fs.readFileSync(path.join(run, 'manifest.json'), 'utf8'));
  const out = [];
  for (const c of man.cases) {
    if (splits && !splits.includes(c.split)) continue;
    const f = path.join(run, 'cases', `${c.id}.json.gz`);
    if (!fs.existsSync(f)) continue;
    out.push(prepCase(JSON.parse(zlib.gunzipSync(fs.readFileSync(f))), c.split));
  }
  return out;
}

const qmat = (q, R = new Float64Array(9)) => quatToMat(q, R);

export function prepCase(c, split) {
  const S = c.samples, n = S.n, pc = c.precontact, t0 = pc.t, sm = c.summary;
  let k0 = 0; while (k0 < n - 1 && S.t[k0] < t0 - 1e-9) k0++;
  // end of the valid window: first arm / spring break or detach; hull strike + 0.15 s
  let tEnd = S.t[n - 1];
  const brk = [...(sm.arm_break_t || []), ...(sm.spring_snap_t || [])].filter(v => v != null);
  if (brk.length) tEnd = Math.min(tEnd, Math.min(...brk));
  for (const e of c.events || []) if (e.type === 'detach' && /wheel/.test(e.piece || '')) tEnd = Math.min(tEnd, e.t);
  if (sm.hull_strike) tEnd = Math.min(tEnd, sm.hull_strike.t + 0.15);
  const R = new Float64Array(9), rows = [];
  for (let k = k0 + 1; k < n && S.t[k] <= tEnd + 1e-9; k++) {
    const q = S.q[k], p = S.p[k], cb = S.comB[k];
    qmat(q, R);
    // body origin = COM - R comB; hub height of the body-fixed rest wheel centre
    const ox = p[0] - (R[0] * cb[0] + R[1] * cb[1] + R[2] * cb[2]), oy = p[1] - (R[3] * cb[0] + R[4] * cb[1] + R[5] * cb[2]), oz = p[2] - (R[6] * cb[0] + R[7] * cb[1] + R[8] * cb[2]);
    const hub = WHEELS.map(w => oy + R[3] * w.centre[0] + R[4] * w.centre[1] + R[5] * w.centre[2]);
    const fwd = [R[2], R[5], R[8]], hl = Math.hypot(fwd[0], fwd[2]) || 1, fx = fwd[0] / hl, fz = fwd[2] / hl;
    const v = S.v[k];
    rows.push({ t: S.t[k] - t0, dt: S.t[k] - S.t[k - 1], hub, floor: S.hull.floorMin[k], N: S.wheel.F[k].map(F => F[1]), lat: S.wheel.Flonglat[k].map(F => F[1]), long: S.wheel.Flonglat[k].map(F => F[0]),
      vlat: v[0] * fz - v[2] * fx, yaw: S.w[k][1], o: [ox, oy, oz] });
  }
  const init = { type: 'state', p: pc.p_twin, q: pc.q, v: pc.v_twin, w: pc.w, gas: pc.gas, leftDeck: true, wheel_spin: S.wheel.spin[k0].slice() };
  return { id: c.id, split, init, t0, rows, tEnd: tEnd - t0, summary: sm, target: c.target, achieved: sm.achieved, wheelIds: c.wheel_ids };
}

/**
 * Replay one case on a twin (built with the parameter set); returns per soft sample the twin's comparable values
 * plus per-wheel load diagnostics. Integrator steps are run one by one with the control-step bookkeeping per step.
 */
export function replay(tw, cs, { tMax = null } = {}) {
  tw.reset(cs.init);
  const u = new Float64Array(NU), R = new Float64Array(9), dt = tw.dt, out = [], under = tw.hull.filter(H => H[1] < 0.35); // underbody points, as the soft floor
  const T = tMax ?? cs.rows[cs.rows.length - 1]?.t ?? 0;
  let ri = 0, acc = null, nacc = 0;
  const zero = () => ({ N: [0, 0, 0, 0], lat: [0, 0, 0, 0], long: [0, 0, 0, 0] });
  acc = zero();
  const steps = Math.ceil(T / dt + 1e-9);
  for (let s = 0; s < steps; s++) {
    const F = tw.stepFlags; F.any = F.street = F.deck = F.other = F.hull = false;
    tw.substep(u); tw.boundary();
    const L = tw.wheelLoad;
    for (let j = 0; j < 4; j++) { acc.N[j] += L.N[j]; acc.lat[j] += L.lat[j]; acc.long[j] += L.long[j]; }
    nacc++;
    const t = (s + 1) * dt;
    while (ri < cs.rows.length && cs.rows[ri].t <= t + 1e-9) {
      quatToMat(tw.q, R);
      const c = tw.mp.com, p = tw.p;
      const oy = p[1] - (R[3] * c[0] + R[4] * c[1] + R[5] * c[2]);
      const hub = WHEELS.map(w => oy + R[3] * w.centre[0] + R[4] * w.centre[1] + R[5] * w.centre[2]);
      let floor = Infinity;
      for (const H of under) floor = Math.min(floor, oy + R[3] * H[0] + R[4] * H[1] + R[5] * H[2]);
      const hl = Math.hypot(R[2], R[8]) || 1, fx = R[2] / hl, fz = R[8] / hl;
      const k = 1 / nacc;
      out.push({ hub, floor, N: acc.N.map(x => x * k), lat: acc.lat.map(x => x * k), long: acc.long.map(x => x * k), vlat: tw.v[0] * fz - tw.v[2] * fx, yaw: tw.w[1] });
      acc = zero(); nacc = 0; ri++;
    }
  }
  return { rows: out, peak: tw.wheelPeak, events: tw.events, hullStrike: tw.hullStrike };
}

/** Weighted squared error of one replay against its case: { total, parts: {name: sum}, T } (time integral). */
export function caseLoss(cs, rp) {
  const parts = { hub: 0, floor: 0, N: 0, lat: 0, long: 0, vlat: 0, yaw: 0 };
  let T = 0;
  const n = Math.min(cs.rows.length, rp.rows.length);
  for (let i = 0; i < n; i++) {
    const a = cs.rows[i], b = rp.rows[i], w = a.dt;
    T += w;
    for (let j = 0; j < 4; j++) {
      parts.hub += w * ((a.hub[j] - b.hub[j]) / SCALE.hub) ** 2 / 4;
      parts.N += w * ((a.N[j] - b.N[j]) / SCALE.N) ** 2 / 4;
      parts.lat += w * ((a.lat[j] - b.lat[j]) / SCALE.lat) ** 2 / 4;
      parts.long += w * ((a.long[j] - b.long[j]) / SCALE.long) ** 2 / 4;
    }
    // floor: only where it matters (either model within 6 cm of the street)
    const fa = a.floor, fb = Number.isFinite(b.floor) ? b.floor : fa;
    if (fa < 0.06 || fb < 0.06) parts.floor += w * ((Math.max(fa, 0) - Math.max(fb, 0)) / SCALE.floor) ** 2;
    parts.vlat += w * ((a.vlat - b.vlat) / SCALE.vlat) ** 2;
    parts.yaw += w * ((a.yaw - b.yaw) / SCALE.yaw) ** 2;
  }
  let total = 0;
  for (const k in parts) total += WEIGHT[k] * parts[k];
  return { total, parts, T };
}

/** RMS errors per quantity over a set of cases (validation report). */
export function rmsReport(cases, replays) {
  const acc = {}, add = (k, e, w) => { (acc[k] ||= [0, 0]); acc[k][0] += w * e * e; acc[k][1] += w; };
  cases.forEach((cs, ci) => {
    const rp = replays[ci], n = Math.min(cs.rows.length, rp.rows.length);
    for (let i = 0; i < n; i++) {
      const a = cs.rows[i], b = rp.rows[i], w = a.dt;
      for (let j = 0; j < 4; j++) { add('hub_m', a.hub[j] - b.hub[j], w); add('N_N', a.N[j] - b.N[j], w); add('lat_N', a.lat[j] - b.lat[j], w); add('long_N', a.long[j] - b.long[j], w); }
      if (Number.isFinite(b.floor)) add('floor_m', Math.max(a.floor, 0) - Math.max(b.floor, 0), w);
      add('vlat_ms', a.vlat - b.vlat, w); add('yaw_rads', a.yaw - b.yaw, w);
    }
  });
  return Object.fromEntries(Object.entries(acc).map(([k, [s, w]]) => [k, Math.sqrt(s / w)]));
}
