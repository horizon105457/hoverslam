// Cross-entropy method (sampling): Gaussian around the warm start, K samples per iteration (lane 0 = the current
// mean, lane 1 = the best schedule so far), refit mean / std to the `elites` cheapest with smoothing `alpha`,
// std floored at sigma_min (refit in the unclipped sample space, noise.js); returns the final (clipped) mean if it
// is at least as good as the best sample, else that sample. The std is re-initialised every plan (sigma), the mean
// is the shifted previous plan, so consecutive plans refine one solution.
import { Noise, boxSigma, clip as clipInto } from './noise.js';

export function createCEM(o, { rng, nv }) {
  let noise = null, mu = null, sd = null, sdMin = null, X = null, Xr = null, cost = null, idx = null, best = null, mc = null;
  return {
    name: 'cem', config: o,
    solve(theta, P) {
      const K = o.samples, n = P.dim, ne = Math.min(o.elites, K);
      if (!X || X.length !== K * n) {
        noise = new Noise(n); mu = new Float64Array(n); sd = new Float64Array(n); sdMin = new Float64Array(n);
        X = new Float64Array(K * n); Xr = new Float64Array(K * n); cost = new Float64Array(K); idx = new Int32Array(K); best = new Float64Array(n);
        mc = new Float64Array(n);
      }
      mu.set(theta); boxSigma(P, o.sigma, sd); boxSigma(P, o.sigma_min, sdMin);
      best.set(theta);
      let bc = P.cost(theta);
      for (let it = 0; it < o.iters; it++) {
        for (let k = 0; k < K; k++) {
          const x = X.subarray(k * n, (k + 1) * n), xr = Xr.subarray(k * n, (k + 1) * n);
          if (k === 0) { xr.set(mu); x.set(mu); clipInto(P, x); } else if (k === 1) { x.set(best); xr.set(best); } else noise.around(mu, sd, P, nv, rng, o.rho, x, xr);
          cost[k] = k === 1 ? bc : P.cost(x);
          idx[k] = k;
        }
        idx.sort((a, b) => cost[a] - cost[b] || a - b);
        if (cost[idx[0]] < bc) { bc = cost[idx[0]]; best.set(X.subarray(idx[0] * n, idx[0] * n + n)); }
        for (let e = 0; e < n; e++) {
          let m = 0; // refit on the unclipped elites (noise.js)
          for (let r = 0; r < ne; r++) m += Xr[idx[r] * n + e];
          m /= ne;
          let v = 0;
          for (let r = 0; r < ne; r++) { const d = Xr[idx[r] * n + e] - m; v += d * d; }
          const s = Math.sqrt(v / ne);
          mu[e] = o.alpha * m + (1 - o.alpha) * mu[e];
          const s2 = o.alpha * s + (1 - o.alpha) * sd[e];
          sd[e] = s2 > sdMin[e] ? s2 : sdMin[e];
        }
      }
      // the refitted mean is the smooth schedule (the best sample carries its noise): keep it unless it is worse
      mc.set(mu); clipInto(P, mc);
      const cm = P.cost(mc);
      if (cm <= bc) { theta.set(mc); return { cost: cm }; }
      theta.set(best);
      return { cost: bc };
    },
  };
}
