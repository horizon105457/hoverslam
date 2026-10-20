// Shared helpers of the optimizers: exploration noise and box clipping. Noise comes from the controller's
// mulberry32 stream (rng.gauss: Irwin-Hall normal), optionally correlated along the knots (AR(1) with coefficient
// rho and unit marginal variance), so a sample is a smooth schedule instead of per-knot chatter.

// Samples live unclipped (raw) and are clipped only to be evaluated: averaging clipped samples would bias every
// channel that sits on a bound (a closed valve at 0 drifts open by the mean of the clipped half-normal).
export class Noise {
  constructor(dim) { this.z = new Float64Array(dim); }
  /** raw_e = mu_e + sd_e · z_e, z ~ N(0, 1) per element, AR(1) over knots (layout knot-major, nv per knot);
   *  out = clip(raw) into P's box. */
  around(mu, sd, P, nv, rng, rho, out, raw) {
    const z = this.z, s = Math.sqrt(1 - rho * rho);
    for (let e = 0; e < P.dim; e++) {
      const g = rng.gauss();
      z[e] = rho && e >= nv ? rho * z[e - nv] + s * g : g;
      const x = mu[e] + sd[e] * z[e];
      raw[e] = x;
      out[e] = x < P.lo[e] ? P.lo[e] : x > P.hi[e] ? P.hi[e] : x;
    }
    return out;
  }
}

/** Per-element standard deviation sigma · (hi - lo) · scale (the channel's exploration scale, P.scale). */
export function boxSigma(P, sigma, out = new Float64Array(P.dim)) {
  for (let e = 0; e < P.dim; e++) out[e] = sigma * (P.hi[e] - P.lo[e]) * (P.scale ? P.scale[e] : 1);
  return out;
}

/** Clamp x into P's box (in place). */
export function clip(P, x) {
  for (let e = 0; e < P.dim; e++) { const v = x[e]; x[e] = v < P.lo[e] ? P.lo[e] : v > P.hi[e] ? P.hi[e] : v; }
  return x;
}
