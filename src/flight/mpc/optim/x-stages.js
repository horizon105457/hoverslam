// Schedule stages for hybrid optimizers (gas-study work stream): they shape θ or its box and leave the search to the
// stages that follow, e.g. 'optimizer': [{type:'xinit', ...}, {type:'xmask', ...}, {type:'cem'}].
//   xinit  time-varying cold start: on the episode's first solve only, θ = a piecewise-linear schedule per channel,
//          {schedule: {heave: [[τ0, v0], [τ1, v1], ...], roll: ...}} in s since the first plan (= arming); channels
//          not listed keep the controller's init. (The MPC's own `init` is one constant per channel.)
//   xmask  heave-only coast: the heave channel is held at 0 (box [0, 0]) for every knot whose segment reaches back
//          before arming + heave_until, i.e. the collective lift starts ramping at the first knot at or after that
//          time; the torque channels stay free (with bottom nozzles only, a torque still lifts: one side fires).
//          Unlike the MPC's `coast`, which closes every valve, this separates "when to brake" from "when to fix the
//          attitude". It narrows P.lo / P.hi in place for the stages after it (they are the schedule's arrays) and
//          restores them first on every plan, so the mask follows the receding horizon. Optimizers must tolerate
//          empty boxes: cem, mppi, xlbfgs do; grad's finite differences divide by zero there (0/0 -> NaN, no step).
export function createInit(o = {}) {
  let done = false;
  return {
    name: 'xinit', config: o,
    solve(theta, P) {
      if (done) return { cost: NaN };
      done = true;
      const par = P.param, nv = par.nv, tk = par.tk;
      for (const [name, pts] of Object.entries(o.schedule || {})) {
        const k = par.names.indexOf(name);
        if (k < 0) throw new Error(`xinit: unknown channel ${name} (have ${par.names.join(', ')})`);
        for (let j = 0; j < par.nk; j++) theta[j * nv + k] = interp(pts, tk[j]);
      }
      par.project(theta);
      return { cost: P.cost(theta) };
    },
  };
}

function interp(pts, t) {
  if (t <= pts[0][0]) return pts[0][1];
  for (let i = 1; i < pts.length; i++) {
    const [t1, v1] = pts[i], [t0, v0] = pts[i - 1];
    if (t <= t1) return t1 > t0 ? v0 + (v1 - v0) * (t - t0) / (t1 - t0) : v1;
  }
  return pts[pts.length - 1][1];
}

export function createMask(o = {}) {
  const until = o.heave_until ?? 0, ch = o.channels || ['heave'];
  let tArm = null, lo0 = null, hi0 = null;
  return {
    name: 'xmask', config: o,
    solve(theta, P) {
      const par = P.param, nv = par.nv, tk = par.tk, t0 = P.s.t;
      if (tArm === null) tArm = t0; // the first plan of an episode happens at arming
      if (!lo0) { lo0 = Float64Array.from(P.lo); hi0 = Float64Array.from(P.hi); }
      P.lo.set(lo0); P.hi.set(hi0);
      const ks = ch.map(nm => par.names.indexOf(nm));
      for (let j = 0; j < par.nk; j++) {
        const prev = j > 0 ? tk[j - 1] : tk[j];
        if (t0 + prev >= tArm + until - 1e-9) break;
        for (const k of ks) { const e = j * nv + k; P.lo[e] = 0; P.hi[e] = 0; }
      }
      par.project(theta);
      return { cost: NaN };
    },
  };
}
