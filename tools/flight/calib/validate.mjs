// Validation of a twin touchdown fit (fit.mjs) against the soft-body landing experiments, and the failure thresholds:
//  1. RMS errors per quantity (lib.mjs rmsReport) of the spec twin (v1) and of the fit, per split (fit / anchor /
//     holdout; the holdout cases took no part in the fit).
//  2. Failure thresholds in load space (what a rigid twin cannot model: control arms / springs breaking). Every
//     case is replayed for its full length (not cut at the soft failure); per wheel the twin's peak normal force and
//     peak lateral force up to the soft wheel's break time (or the end) are compared with the soft outcome of that
//     wheel (arm break / detach -> lateral; spring snap -> normal). Thresholds are chosen on FIT + ANCHOR cases only
//     (largest balanced accuracy, ties -> the lower = conservative value), then scored on the holdout.
//  3. Failure prediction per case: soft "fails" = hull strike, arm break, spring snap or wheel detach; twin "fails" =
//     hull strike (underbody points) or wheel overload (thresholds); confusion per split, v1 vs fit.
// usage: node tools/flight/calib/validate.mjs run=<touchdown-doe run> fit=<fit.json> [out=<dir of fit.json>]
import fs from 'fs';
import path from 'path';
import { RigidTwin } from '../../../src/flight/twin.js';
import { parseArgs } from '../../lib/args.mjs';
import { loadCases, replay, rmsReport } from './lib.mjs';

const a = parseArgs(), run = path.resolve(a.run), fitDoc = JSON.parse(fs.readFileSync(a.fit, 'utf8')), out = a.out || path.dirname(a.fit);
const cases = loadCases(run), splits = ['fit', 'anchor', 'holdout'];
const twV1 = new RigidTwin(), twFit = new RigidTwin({ vehicle: fitDoc.vehicle });

// 1. RMS per split
const rms = {};
for (const [name, tw] of [['v1', twV1], ['fit', twFit]]) for (const s of splits) {
  const C = cases.filter(c => c.split === s);
  rms[`${name}/${s}`] = rmsReport(C, C.map(c => replay(tw, c)));
}

// 2. per-wheel peaks from full-length replays (2.3 s from the precontact state, valves closed)
// peaks up to a time need a trace: re-run with a per-step recorder of |lat| and N per wheel
function wheelTrace(tw, cs, T) {
  tw.reset(cs.init);
  const u = new Float64Array(10), n = Math.ceil(T / tw.dt), N = [], L = [];
  for (let s = 0; s < n; s++) {
    const F = tw.stepFlags; F.any = F.street = F.deck = F.other = F.hull = false;
    tw.substep(u); tw.boundary();
    const w = tw.wheelLoad; N.push(Array.from(w.N)); L.push(Array.from(w.lat, Math.abs));
  }
  return { N, L, hull: tw.hullStrike, dt: tw.dt };
}
function samples(tw) {
  const S = [];
  for (const cs of cases) {
    // soft times are absolute; twin time 0 is the soft precontact time cs.t0
    const T = 2.3, tr = wheelTrace(tw, cs, T), sm = cs.summary, tpc = cs.t0;
    for (let j = 0; j < 4; j++) {
      const arm = sm.arm_break_t?.[j] ?? null, spr = sm.spring_snap_t?.[j] ?? null, det = (sm.wheel_detach || []).includes(cs.wheelIds?.[j]);
      const tb = Math.min(arm ?? 1e9, spr ?? 1e9);
      const kEnd = Math.min(tr.N.length, Math.ceil(((tb < 1e9 ? tb - tpc : T)) / tr.dt));
      let pN = 0, pL = 0;
      for (let k = 0; k < kEnd; k++) { if (tr.N[k][j] > pN) pN = tr.N[k][j]; if (tr.L[k][j] > pL) pL = tr.L[k][j]; }
      S.push({ id: cs.id, split: cs.split, j, pN, pL, arm: arm != null || det, spring: spr != null });
    }
  }
  return S;
}
function pickThreshold(S, key, label) {
  const vals = [...new Set(S.map(s => s[key]))].sort((x, y) => x - y);
  let best = { thr: Infinity, bacc: 0 };
  for (const thr of vals) {
    const tp = S.filter(s => s[label] && s[key] >= thr).length, fn = S.filter(s => s[label] && s[key] < thr).length;
    const tn = S.filter(s => !s[label] && s[key] < thr).length, fp = S.filter(s => !s[label] && s[key] >= thr).length;
    const bacc = 0.5 * (tp / Math.max(1, tp + fn) + tn / Math.max(1, tn + fp));
    if (bacc > best.bacc + 1e-12) best = { thr, bacc, tp, fn, tn, fp };
  }
  return best;
}
function score(S, key, label, thr) {
  const tp = S.filter(s => s[label] && s[key] >= thr).length, fn = S.filter(s => s[label] && s[key] < thr).length;
  const tn = S.filter(s => !s[label] && s[key] < thr).length, fp = S.filter(s => !s[label] && s[key] >= thr).length;
  return { tp, fn, tn, fp, bacc: +(0.5 * (tp / Math.max(1, tp + fn) + tn / Math.max(1, tn + fp))).toFixed(3) };
}
const thresholds = {}, thrScore = {};
for (const [name, tw] of [['v1', twV1], ['fit', twFit]]) {
  const S = samples(tw), train = S.filter(s => s.split !== 'holdout'), hold = S.filter(s => s.split === 'holdout');
  const lat = pickThreshold(train, 'pL', 'arm'), nor = pickThreshold(train, 'pN', 'spring');
  thresholds[name] = { wheel_lat_max: lat.thr, wheel_normal_max: nor.thr };
  thrScore[name] = { lat_train: score(train, 'pL', 'arm', lat.thr), lat_holdout: score(hold, 'pL', 'arm', lat.thr), normal_train: score(train, 'pN', 'spring', nor.thr), normal_holdout: score(hold, 'pN', 'spring', nor.thr),
    lat_broken: S.filter(s => s.arm).map(s => Math.round(s.pL)).sort((x, y) => x - y), lat_intact_max: Math.round(Math.max(...S.filter(s => !s.arm).map(s => s.pL))) };
}

// 3. case-level failure prediction with the thresholds of the same model
const softFail = c => !!(c.summary.hull_strike || (c.summary.arm_break_t || []).some(v => v != null) || (c.summary.spring_snap_t || []).some(v => v != null) || (c.summary.wheel_detach || []).length);
const confusion = {};
for (const [name, vehicle] of [['v1', null], ['fit', fitDoc.vehicle]]) {
  const tw = new RigidTwin({ vehicle: { ...(vehicle || {}), failure: { wheel_lat_max: thresholds[name].wheel_lat_max, wheel_normal_max: thresholds[name].wheel_normal_max } } });
  for (const s of splits) {
    const C = cases.filter(c => c.split === s), m = { tp: 0, fn: 0, tn: 0, fp: 0 };
    for (const cs of C) {
      tw.reset(cs.init);
      const u = new Float64Array(10);
      for (let k = 0; k < 2300; k++) { const F = tw.stepFlags; F.any = F.street = F.deck = F.other = F.hull = false; tw.substep(u); tw.boundary(); }
      const tf = !!(tw.hullStrike || tw.wheelOverload), sf = softFail(cs);
      m[sf ? (tf ? 'tp' : 'fn') : (tf ? 'fp' : 'tn')]++;
    }
    confusion[`${name}/${s}`] = { ...m, acc: +((m.tp + m.tn) / C.length).toFixed(3) };
  }
}
const doc = { run, fit: a.fit, rms, thresholds, thrScore, confusion };
fs.writeFileSync(path.join(out, 'validate.json'), JSON.stringify(doc, null, 1));
const f4 = o => Object.fromEntries(Object.entries(o).map(([k, v]) => [k, +v.toFixed(4)]));
console.log('RMS (time-weighted, valid windows):');
for (const k of Object.keys(rms)) console.log(`  ${k.padEnd(12)}`, JSON.stringify(f4(rms[k])));
console.log('thresholds', JSON.stringify(thresholds));
console.log('threshold scores', JSON.stringify(thrScore, null, 1));
console.log('case failure prediction', JSON.stringify(confusion, null, 1));
