// Sim-time access to a flight log: the soft body's meta.propulsion (coldgas-log@1) or a rigid-twin trajectory
// (coldgas-traj@1) share the fields used here (t, state[13], comB, tank, u_cmd, valve, thrust). Before the first
// sample the car is extrapolated along its launch velocity (the drive-in pre-roll, like RecordingPlayer).
// Also finds the key moments of a flight for the edit and the HUD.
import { slerp, quatToMat } from '../../core/quat.js';
import { DECK } from '../../scene/layout.js';
import { NU } from '../../flight/contract.js';

export class FlightLog {
  constructor(log, { flight = null, duration = null } = {}) {
    this.log = log; this.flight = flight;
    this.t = log.t; this.n = log.t.length;
    this.duration = duration ?? log.t[this.n - 1];
    this._q0 = [0, 0, 0, 1]; this._q1 = [0, 0, 0, 1]; this._R = new Float64Array(9);
  }
  locate(t) {
    const T = this.t, n = this.n;
    if (t <= T[0]) return [0, 0];
    if (t >= T[n - 1]) return [n - 2, 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; }
    return [lo, (t - T[lo]) / (T[hi] - T[lo])];
  }
  /** Linear interpolation of row field `f` (array of arrays) component c. */
  lerp(f, t, c) { const [k, a] = this.locate(t), A = this.log[f]; return A[k][c] * (1 - a) + A[k + 1][c] * a; }
  p(t, out = [0, 0, 0]) {
    const S = this.log.state, pre = t < this.t[0] ? t - this.t[0] : 0;
    for (let c = 0; c < 3; c++) out[c] = this.lerp('state', t, c) + (pre ? S[0][7 + c] * pre : 0);
    return out;
  }
  v(t, out = [0, 0, 0]) { for (let c = 0; c < 3; c++) out[c] = this.lerp('state', t, 7 + c); return out; }
  w(t, out = [0, 0, 0]) { for (let c = 0; c < 3; c++) out[c] = this.lerp('state', t, 10 + c); return out; }
  q(t, out = [0, 0, 0, 1]) {
    const [k, a] = this.locate(t), S = this.log.state;
    for (let c = 0; c < 4; c++) { this._q0[c] = S[k][3 + c]; this._q1[c] = S[k + 1][3 + c]; }
    return slerp(this._q0, this._q1, a, out);
  }
  comB(t, out = [0, 0, 0]) { for (let c = 0; c < 3; c++) out[c] = this.log.comB ? this.lerp('comB', t, c) : 0; return out; }
  thrust(t, i) { return this.log.thrust ? Math.max(0, this.lerp('thrust', t, i)) : 0; }
  valve(t, i) { return this.log.valve ? this.lerp('valve', t, i) : 0; }
  tank(t) { return { m: this.lerp('tank', t, 0), T: this.lerp('tank', t, 1), p: this.lerp('tank', t, 2) }; }
  /** World position of body point xB (spec/README §1: p + R(q)(xB − comB)). */
  bodyPoint(t, xB, out = [0, 0, 0]) {
    const p = this.p(t), R = quatToMat(this.q(t), this._R), c = this.comB(t), d = [xB[0] - c[0], xB[1] - c[1], xB[2] - c[2]];
    for (let a = 0; a < 3; a++) out[a] = p[a] + R[a * 3] * d[0] + R[a * 3 + 1] * d[1] + R[a * 3 + 2] * d[2];
    return out;
  }
  /** Body axis (body-frame vector) in world. */
  bodyDir(t, dB, out = [0, 0, 0]) {
    const R = quatToMat(this.q(t), this._R);
    for (let a = 0; a < 3; a++) out[a] = R[a * 3] * dB[0] + R[a * 3 + 1] * dB[1] + R[a * 3 + 2] * dB[2];
    return out;
  }
  /** Tilt (deg) of body up from world up. */
  tilt(t) { const R = quatToMat(this.q(t), this._R); return Math.acos(Math.max(-1, Math.min(1, R[4]))) * 180 / Math.PI; }
  totalThrust(t) { let s = 0; for (let i = 0; i < NU; i++) s += this.thrust(t, i); return s; }

  /**
   * Key moments: tEdge (COM over the deck edge), tIgn (first thrust), tBrake (peak descent rate under thrust:
   * the jets bite), tTouch (first street contact), tRest (car settled on its wheels / wreck still) and end.
   */
  moments() {
    const ev = this.flight?.events || this.log.events || [];
    const find = type => ev.find(e => e.type === type)?.t ?? null;
    const T = this.t, n = this.n, S = this.log.state, end = this.duration;
    let tEdge = find('edge');
    if (tEdge === null) { const k = S.findIndex(s => s[2] > DECK.halfWidth); tEdge = k >= 0 ? T[k] : 0.7; }
    let tIgn = find('ignition');
    if (tIgn === null && this.log.thrust) { const k = this.log.thrust.findIndex(r => r.some(f => f > 200)); tIgn = k >= 0 ? T[k] : null; }
    let tTouch = find('touchdown');
    if (tTouch === null) { // lowest point of the COM
      let best = 1e9; for (let k = 0; k < n; k++) if (T[k] > tEdge && S[k][1] < best) { best = S[k][1]; tTouch = T[k]; }
    }
    let tBrake = tIgn ?? tEdge, vmin = 0;
    for (let k = 0; k < n; k++) if (T[k] > (tIgn ?? tEdge) && T[k] < tTouch && S[k][8] < vmin) { vmin = S[k][8]; tBrake = T[k]; }
    // rest: after touchdown, vertical motion and body rates calm for 0.3 s
    let tRest = Math.min(end, tTouch + 1.5), calm = 0;
    for (let k = 0; k < n; k++) {
      if (T[k] <= tTouch + 0.2) continue;
      const s = S[k], q = Math.abs(s[8]) < 0.25 && Math.hypot(s[10], s[11], s[12]) < 0.35;
      calm = q ? calm + (T[k] - T[k - 1]) : 0;
      if (calm > 0.3) { tRest = T[k] - 0.3; break; }
    }
    const p = k => [S[k][0], 0, S[k][2]];
    const L = p(this.locate(tTouch)[0]), R = p(n - 1);
    return { tEdge, tIgn, tBrake, tTouch, tRest, end, L, R };
  }
}
