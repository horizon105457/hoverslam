// Keyframed camera for long designed moves (fly-throughs). Unlike cinema/shots.js `fly()` (arc-length along a
// Catmull-Rom, one ease for the whole path) every key carries its own film time, so speed is authored per
// segment: crawl through a throat, rush down a bell. Positions and targets are interpolated with
// time-parameterised cubic Hermite splines (Catmull-Rom tangents from the neighbouring keys, divided by their
// time spans), which keeps velocity continuous across unevenly spaced keys. Scalars (fov, roll, near,
// aperture) use the same spline. Candidate for promotion to src/cinema/shots.js.
//
// key = { t, pos: Vector3, tgt: Vector3, fov?, roll?, near?, aperture?, focus?: Vector3 }
import * as THREE from 'three';

function hermite(p0, m0, p1, m1, s, h) {
  const s2 = s * s, s3 = s2 * s;
  return (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * h * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * h * m1;
}

/** Scalar/vector track over keyed times. `hold` = zero velocity at the first/last key (ease in/out). */
class Track {
  constructor(ts, vals, { hold = true } = {}) {
    this.ts = ts; this.v = vals; this.n = ts.length;
    this.m = vals.map((_, i) => {
      if (this.n < 2) return 0;
      if (i === 0 || i === this.n - 1) return hold ? 0 : (vals[Math.min(i + 1, this.n - 1)] - vals[Math.max(i - 1, 0)]) / (ts[Math.min(i + 1, this.n - 1)] - ts[Math.max(i - 1, 0)]);
      return (vals[i + 1] - vals[i - 1]) / (ts[i + 1] - ts[i - 1]);
    });
  }
  at(t) {
    const { ts, v, m, n } = this;
    if (n === 1 || t <= ts[0]) return v[0];
    if (t >= ts[n - 1]) return v[n - 1];
    let i = 0; while (i < n - 2 && t > ts[i + 1]) i++;
    const h = ts[i + 1] - ts[i], s = (t - ts[i]) / h;
    return hermite(v[i], m[i], v[i + 1], m[i + 1], s, h);
  }
}

/**
 * @param keys  keyframes with absolute film times (the Sequence passes film time as `t` when t0..t1 = film range)
 * @returns cam(u, t) for a Sequence shot
 */
export function keyed(keys, { hold = true, fov = 40 } = {}) {
  const ts = keys.map(k => k.t);
  const vtrack = get => ['x', 'y', 'z'].map(c => new Track(ts, keys.map(k => get(k)[c]), { hold }));
  const P = vtrack(k => k.pos), Tg = vtrack(k => k.tgt), F = vtrack(k => k.focus || k.tgt);
  const scal = (name, def) => new Track(ts, keys.map(k => k[name] ?? def), { hold });
  const Fov = scal('fov', fov), Roll = scal('roll', 0), Ap = scal('aperture', 0.006);
  // near is spread over decades: interpolate its log
  const NearL = new Track(ts, keys.map(k => Math.log(k.near ?? 0.05)), { hold });
  return (u, t) => ({
    pos: new THREE.Vector3(P[0].at(t), P[1].at(t), P[2].at(t)),
    tgt: new THREE.Vector3(Tg[0].at(t), Tg[1].at(t), Tg[2].at(t)),
    focus: new THREE.Vector3(F[0].at(t), F[1].at(t), F[2].at(t)),
    fov: Fov.at(t), roll: Roll.at(t), near: Math.exp(NearL.at(t)), aperture: Math.max(0, Ap.at(t)),
  });
}
