// MPPI (sampling): K schedules around the nominal (lane 0 = the nominal itself), softmax-weighted mean with a
// temperature relative to the cost spread, `iters` times. Pure: no base controller (cf. controllers/mppi.js,
// which perturbs residuals on pd). The mean is updated in the unclipped sample space and clipped afterwards
// (noise.js). With check_mean the new mean is rolled out once more and the best sample of the plan replaces it if
// the mean is worse (a weighted mean is not always better than its best member).
import { Noise, boxSigma } from './noise.js';
import { exp } from '../../fmath.js';

export function createMPPIOpt(o, { rng, nv }) {
  let noise = null, sd = null, X = null, Xr = null, mu = null, cost = null, w = null, best = null;
  return {
    name: 'mppi', config: o,
    solve(theta, P) {
      const K = o.samples, n = P.dim;
      if (!X || X.length !== K * n) {
        noise = new Noise(n); sd = new Float64Array(n); X = new Float64Array(K * n); Xr = new Float64Array(K * n); mu = new Float64Array(n);
        cost = new Float64Array(K); w = new Float64Array(K); best = new Float64Array(n);
      }
      boxSigma(P, o.sigma, sd);
      mu.set(theta);
      let bc = Infinity;
      for (let it = 0; it < o.iters; it++) {
        for (let k = 0; k < K; k++) {
          const x = X.subarray(k * n, (k + 1) * n), xr = Xr.subarray(k * n, (k + 1) * n);
          if (k === 0) { for (let e = 0; e < n; e++) { const v = mu[e]; xr[e] = v; x[e] = v < P.lo[e] ? P.lo[e] : v > P.hi[e] ? P.hi[e] : v; } }
          else noise.around(mu, sd, P, nv, rng, o.rho, x, xr);
          cost[k] = P.cost(x);
          if (cost[k] < bc) { bc = cost[k]; best.set(x); }
        }
        let cmin = Infinity, sum = 0;
        for (let k = 0; k < K; k++) { if (cost[k] < cmin) cmin = cost[k]; sum += cost[k]; }
        const d = sum / K - cmin, lam = o.temperature * (d > 1e-6 ? d : 1e-6);
        let ws = 0;
        for (let k = 0; k < K; k++) { const a = -(cost[k] - cmin) / lam; w[k] = exp(a > -700 ? a : -700); ws += w[k]; }
        for (let e = 0; e < n; e++) { let a = 0; for (let k = 0; k < K; k++) a += w[k] * Xr[k * n + e]; mu[e] = a / ws; }
      }
      for (let e = 0; e < n; e++) { const v = mu[e]; theta[e] = v < P.lo[e] ? P.lo[e] : v > P.hi[e] ? P.hi[e] : v; }
      let c = Infinity;
      if (o.check_mean) { c = P.cost(theta); if (bc < c) { theta.set(best); c = bc; } }
      return { cost: Number.isFinite(c) ? c : bc };
    },
  };
}
