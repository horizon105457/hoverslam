// Gradient-based shooting: projected descent on the box with finite-difference gradients of the rollout cost
// (the twin is not differentiable: penalty contacts, events; finite differences see exactly the planning model).
//   fd 'forward'  n+1 rollouts per gradient (one-sided step away from an active bound),
//      'central'  2n rollouts, 'spsa' 2·spsa_samples rollouts (simultaneous ±h perturbations, averaged).
// Note: h is relative to the box width; with modes / raw channels in [0, 1] the default 0.02 equals the valve dead
// band (spec/README.md §3), so around a closed schedule forward differences see an exactly flat cost (gas study).
// Step: x = clip(θ - step · g / |g|∞) (the steepest element moves by `step`, in θ units), components that push
// out of the box at an active bound are dropped (projected gradient); accepted if the cost decreases, otherwise
// the step shrinks (up to `ls` tries per iteration). The step grows after a success and carries over to the next
// plan of the episode (a trust region that adapts to the cost's scale).
export function createGrad(o, { rng }) {
  let g = null, x = null, xm = null, step = o.step;
  const fd = (theta, J, P) => {
    const n = P.dim, lo = P.lo, hi = P.hi;
    g.fill(0);
    if (o.fd === 'spsa') {
      for (let r = 0; r < o.spsa_samples; r++) {
        for (let e = 0; e < n; e++) {
          const h = o.h * (hi[e] - lo[e]), d = rng() < 0.5 ? -h : h;
          const a = theta[e] + d, b = theta[e] - d;
          x[e] = a < lo[e] ? lo[e] : a > hi[e] ? hi[e] : a; xm[e] = b < lo[e] ? lo[e] : b > hi[e] ? hi[e] : b;
        }
        const dJ = P.cost(x) - P.cost(xm);
        for (let e = 0; e < n; e++) { const dx = x[e] - xm[e]; if (dx) g[e] += dJ / dx / o.spsa_samples; }
      }
      return;
    }
    x.set(theta);
    for (let e = 0; e < n; e++) {
      const h = o.h * (hi[e] - lo[e]), v = theta[e];
      if (!(h > 0)) continue; // zero-width box (e.g. umax 0): no direction to differentiate, g = 0 (was 0/0)
      if (o.fd === 'central') {
        const a = v + h > hi[e] ? hi[e] : v + h, b = v - h < lo[e] ? lo[e] : v - h;
        x[e] = a; const Ja = P.cost(x); x[e] = b; const Jb = P.cost(x);
        g[e] = (Ja - Jb) / (a - b);
      } else {
        const a = v + h <= hi[e] ? v + h : v - h;
        x[e] = a; g[e] = (P.cost(x) - J) / (a - v);
      }
      x[e] = v;
    }
  };
  return {
    name: 'grad', config: o,
    solve(theta, P) {
      const n = P.dim, lo = P.lo, hi = P.hi;
      if (!g || g.length !== n) { g = new Float64Array(n); x = new Float64Array(n); xm = new Float64Array(n); }
      let J = P.cost(theta);
      for (let it = 0; it < o.iters; it++) {
        fd(theta, J, P);
        let gn = 0;
        for (let e = 0; e < n; e++) {
          if ((theta[e] <= lo[e] && g[e] > 0) || (theta[e] >= hi[e] && g[e] < 0)) g[e] = 0;
          const a = Math.abs(g[e]); if (a > gn) gn = a;
        }
        if (!(gn > 0)) break;
        let ok = false, Jx = J;
        for (let l = 0; l <= o.ls; l++) {
          for (let e = 0; e < n; e++) { const v = theta[e] - (step * g[e]) / gn; x[e] = v < lo[e] ? lo[e] : v > hi[e] ? hi[e] : v; }
          Jx = P.cost(x);
          if (Jx < J) { ok = true; break; }
          step = Math.max(step * o.shrink, o.step_min);
        }
        if (!ok) break;
        theta.set(x); J = Jx;
        step = Math.min(step * o.grow, o.step_max);
      }
      return { cost: J };
    },
  };
}
