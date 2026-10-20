// Projected L-BFGS (box-constrained quasi-Newton, L-BFGS-B in spirit) on the rollout cost with finite-difference
// gradients (gas-study work stream). Why: the normalised steepest descent of grad.js takes steps that ignore the
// curvature, and the open-loop attitude problem is badly conditioned (a torque knot early in the horizon moves the
// touchdown tilt orders of magnitude more than a heave knot late in it), so a quasi-Newton metric should need far
// fewer rollouts per unit of progress. Per iteration:
//   g     forward (n+1 rollouts, one-sided away from an active bound) or central (2n) differences, step h·(hi-lo)
//         per element (elements with an empty box, e.g. masked by xmask, are frozen). h must step over the valves'
//         dead band (2 % opening: smaller commands close the valve), or the cost is exactly flat around a closed
//         schedule (1 % steps from the all-closed cold start see 218 identical costs);
//   free  elements at a bound with the gradient pointing out of the box are fixed for this iteration (projected
//         gradient); the two-loop recursion runs on the free ones with H0 = γ D², D = (hi-lo)·scale (the channel's
//         exploration scale: torque channels small), γ from the newest pair (sᵀy / yᵀ D² y) or, cold, such that
//         the largest element moves by `step0` box widths;
//   line  projected backtracking x(α) = clip(θ + α d), Armijo on the actual step, α ← shrink·α up to `ls` times;
//         no descent -> the memory is dropped and the iteration falls back to the scaled steepest direction once.
// Memory: pairs (s, y) with sᵀy > 0 only, `mem` of them; dropped at every new plan (the warm start shifts θ by a
// fraction of a knot, so old pairs no longer describe the same coordinates) unless keep = true. The gradient is not
// re-estimated after the last iteration of a plan (it would not be used).
const DEF = { iters: 2, fd: 'forward', h: 0.04, mem: 8, ls: 5, c1: 1e-4, shrink: 0.4, step0: 0.2, keep: false };

export function createLBFGS(over = {}) {
  const o = { ...DEF, ...over };
  let n = 0, g = null, gn = null, x = null, d = null, D2 = null, free = null, q = null, al = null;
  let S = [], Y = [], R = [], lastS = null;
  const alloc = m => { n = m; g = new Float64Array(n); gn = new Float64Array(n); x = new Float64Array(n); d = new Float64Array(n); D2 = new Float64Array(n); free = new Uint8Array(n); q = new Float64Array(n); al = new Float64Array(o.mem); };

  const grad = (P, th, J, out) => {
    const lo = P.lo, hi = P.hi;
    x.set(th);
    for (let e = 0; e < n; e++) {
      const w = hi[e] - lo[e];
      if (!(w > 0)) { out[e] = 0; continue; }
      const h = o.h * w, v = th[e];
      if (o.fd === 'central') {
        const a = v + h > hi[e] ? hi[e] : v + h, b = v - h < lo[e] ? lo[e] : v - h;
        x[e] = a; const Ja = P.cost(x); x[e] = b; const Jb = P.cost(x);
        out[e] = (Ja - Jb) / (a - b);
      } else {
        const a = v + h <= hi[e] ? v + h : v - h;
        x[e] = a; out[e] = (P.cost(x) - J) / (a - v);
      }
      x[e] = v;
    }
  };

  /** d = -H g on the free set (two-loop recursion). */
  const direction = () => {
    for (let e = 0; e < n; e++) q[e] = free[e] ? g[e] : 0;
    const m = S.length;
    for (let i = m - 1; i >= 0; i--) {
      let a = 0; const s = S[i], y = Y[i];
      for (let e = 0; e < n; e++) if (free[e]) a += s[e] * q[e];
      a *= R[i]; al[i] = a;
      for (let e = 0; e < n; e++) if (free[e]) q[e] -= a * y[e];
    }
    let gam;
    if (m) { // γ = sᵀy / yᵀ D² y of the newest pair
      const s = S[m - 1], y = Y[m - 1]; let sy = 0, yy = 0;
      for (let e = 0; e < n; e++) if (free[e]) { sy += s[e] * y[e]; yy += y[e] * D2[e] * y[e]; }
      gam = yy > 0 ? sy / yy : 1;
    } else { // cold: the largest element moves by step0 box widths
      let mx = 0;
      for (let e = 0; e < n; e++) if (free[e]) { const v = Math.abs(D2[e] * q[e]) / Math.sqrt(D2[e] || 1); if (v > mx) mx = v; }
      gam = mx > 0 ? o.step0 / mx : 0;
    }
    for (let e = 0; e < n; e++) q[e] *= gam * D2[e];
    for (let i = 0; i < m; i++) {
      let b = 0; const s = S[i], y = Y[i];
      for (let e = 0; e < n; e++) if (free[e]) b += y[e] * q[e];
      b *= R[i];
      for (let e = 0; e < n; e++) if (free[e]) q[e] += s[e] * (al[i] - b);
    }
    for (let e = 0; e < n; e++) d[e] = free[e] ? -q[e] : 0;
  };

  return {
    name: 'xlbfgs', config: o,
    solve(theta, P) {
      if (P.dim !== n) alloc(P.dim);
      const lo = P.lo, hi = P.hi, sc = P.scale;
      if (P.s !== lastS && !o.keep) { S = []; Y = []; R = []; }
      lastS = P.s;
      for (let e = 0; e < n; e++) { const w = (hi[e] - lo[e]) * (sc ? sc[e] : 1); D2[e] = w * w; }
      let J = P.cost(theta);
      grad(P, theta, J, g);
      for (let it = 0; it < o.iters; it++) {
        let anyFree = false;
        for (let e = 0; e < n; e++) {
          free[e] = hi[e] > lo[e] && !((theta[e] <= lo[e] && g[e] > 0) || (theta[e] >= hi[e] && g[e] < 0)) ? 1 : 0;
          if (free[e] && g[e] !== 0) anyFree = true;
        }
        if (!anyFree) break;
        direction();
        let gd = 0;
        for (let e = 0; e < n; e++) gd += g[e] * d[e];
        if (!(gd < 0)) { S = []; Y = []; R = []; direction(); gd = 0; for (let e = 0; e < n; e++) gd += g[e] * d[e]; if (!(gd < 0)) break; }
        let a = 1, ok = false, Jx = J;
        for (let l = 0; l <= o.ls; l++) {
          let lin = 0;
          for (let e = 0; e < n; e++) { const v = theta[e] + a * d[e]; x[e] = v < lo[e] ? lo[e] : v > hi[e] ? hi[e] : v; lin += g[e] * (x[e] - theta[e]); }
          Jx = P.cost(x);
          if (Jx <= J + o.c1 * lin && Jx < J) { ok = true; break; }
          a *= o.shrink;
        }
        if (!ok) { S = []; Y = []; R = []; break; }
        const last = it === o.iters - 1;
        if (!last) {
          const xs = Float64Array.from(x);
          grad(P, xs, Jx, gn);
          const s = new Float64Array(n), y = new Float64Array(n);
          let sy = 0, ss = 0, yy = 0;
          for (let e = 0; e < n; e++) { s[e] = xs[e] - theta[e]; y[e] = gn[e] - g[e]; sy += s[e] * y[e]; ss += s[e] * s[e]; yy += y[e] * y[e]; }
          if (sy > 1e-10 * Math.sqrt(ss * yy)) { S.push(s); Y.push(y); R.push(1 / sy); if (S.length > o.mem) { S.shift(); Y.shift(); R.shift(); } }
          g.set(gn);
          theta.set(xs);
        } else theta.set(x);
        J = Jx;
      }
      return { cost: J };
    },
  };
}
