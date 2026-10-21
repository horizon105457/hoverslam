// Failure criteria of the MPC land objective (src/flight/mpc/objectives/land.js td_lat_* / gnd_lat_*) from a
// touchdown-doe run, as the planning twin sees them: the fitted twin (fit.json vehicle, dt 2 ms) stepped per control
// step (20 ms) from each case's precontact state with closed valves.
//   impact: largest contact-patch lateral speed over the wheels at the twin's touchdown (land.js contactLateralSpeed);
//           soft label `early` = first arm break / spring snap < 0.5 s after the first contact
//   late:   per wheel EMA (½ per control step) of |F_lat| (last substep), max over the wheels from 0.3 s to 2 s after
//           the touchdown; soft label `late` = first break in [0.5, 2) s
// Per criterion: the best balanced-accuracy split on fit + anchor cases (scored on the holdout) and the conservative
// value with 90 % recall on fit + anchor; then the OR rule (imp >= a | late >= b) against any break within 2 s.
// usage: node tools/flight/calib/criteria.mjs run=<touchdown-doe run> fit=<fit.json> [imp=4 late=2000] [out=<dir>]
import fs from 'fs';
import path from 'path';
import { RigidTwin } from '../../../src/flight/twin.js';
import { parseArgs } from '../../lib/args.mjs';
import { loadCases } from './lib.mjs';
import { contactLateralSpeed } from '../../../src/flight/mpc/objectives/land.js';

const a = parseArgs(), run = path.resolve(a.run), fit = JSON.parse(fs.readFileSync(a.fit, 'utf8'));
const cases = loadCases(run), tw = new RigidTwin({ vehicle: fit.vehicle, dt: 0.002 }), S = [];
for (const cs of cases) {
  tw.reset(cs.init);
  const u = new Float64Array(10), ema = [0, 0, 0, 0];
  let tdK = -1, imp = 0, late = 0;
  for (let k = 0; k < 120; k++) {
    tw.step(u);
    if (tdK < 0 && tw.touchdown) { tdK = k; const t = tw.touchdown; imp = contactLateralSpeed(t.q, t.v, t.w, tw.mp.com); }
    const L = tw.wheelLoad.lat;
    for (let j = 0; j < 4; j++) ema[j] = 0.5 * ema[j] + 0.5 * Math.abs(L[j]);
    if (tdK >= 0 && (k - tdK) * 0.02 >= 0.3 && (k - tdK) * 0.02 <= 2) late = Math.max(late, ...ema);
  }
  const sm = cs.summary, tc = sm.first_contact.t, brk = [...(sm.arm_break_t || []), ...(sm.spring_snap_t || [])].filter(x => x != null).map(x => x - tc).sort((p, q) => p - q);
  S.push({ id: cs.id, split: cs.split, early: brk.length > 0 && brk[0] < 0.5, late: brk.length > 0 && brk[0] >= 0.5 && brk[0] < 2, imp, lateF: late, anyF: brk.length > 0 && brk[0] < 2 });
}
const sc = (S, k, lab, t) => { const tp = S.filter(s => s[lab] && s[k] >= t).length, fn = S.filter(s => s[lab] && s[k] < t).length, tn = S.filter(s => !s[lab] && s[k] < t).length, fp = S.filter(s => !s[lab] && s[k] >= t).length; return { t: +(+t).toPrecision(4), tp, fn, tn, fp, bacc: +(0.5 * (tp / Math.max(1, tp + fn) + tn / Math.max(1, tn + fp))).toFixed(3), recall: +(tp / Math.max(1, tp + fn)).toFixed(2) }; };
const train = S.filter(s => s.split !== 'holdout'), hold = S.filter(s => s.split === 'holdout'), doc = { run, fit: a.fit, n: S.length, criteria: {} };
for (const [k, lab] of [['imp', 'early'], ['lateF', 'late']]) {
  let b = null; for (const t of [...new Set(train.map(s => s[k]))].sort((x, y) => x - y)) { const r = sc(train, k, lab, t); if (!b || r.bacc > b.bacc) b = r; }
  const vals = train.filter(s => s[lab]).map(s => s[k]).sort((x, y) => x - y), t90 = vals[Math.floor(0.1 * (vals.length - 1))];
  doc.criteria[k] = { label: lab, positives: train.filter(s => s[lab]).length, best: b, best_holdout: sc(hold, k, lab, b.t), recall90: sc(train, k, lab, t90), recall90_holdout: sc(hold, k, lab, t90) };
  console.log(`${k} -> ${lab} (${doc.criteria[k].positives} of ${train.length} train): best ${JSON.stringify(b)} holdout ${JSON.stringify(doc.criteria[k].best_holdout)}`);
  console.log(`   90 % recall: ${JSON.stringify(doc.criteria[k].recall90)} holdout ${JSON.stringify(doc.criteria[k].recall90_holdout)}`);
}
const ti = +(a.imp ?? doc.criteria.imp.best.t), tl = +(a.late ?? doc.criteria.lateF.best.t), f = s => s.imp >= ti || s.lateF >= tl;
doc.or = { imp: ti, late: tl };
for (const [nm, set] of [['train', train], ['holdout', hold]]) {
  const tp = set.filter(s => s.anyF && f(s)).length, fn = set.filter(s => s.anyF && !f(s)).length, tn = set.filter(s => !s.anyF && !f(s)).length, fp = set.filter(s => !s.anyF && f(s)).length;
  doc.or[nm] = { tp, fn, tn, fp, bacc: +(0.5 * (tp / Math.max(1, tp + fn) + tn / Math.max(1, tn + fp))).toFixed(3) };
  console.log(`OR rule imp >= ${ti} | late >= ${tl} ${nm}: ${JSON.stringify(doc.or[nm])}`);
}
doc.samples = S;
fs.writeFileSync(path.join(a.out || path.dirname(a.fit), 'criteria.json'), JSON.stringify(doc, null, 1));
