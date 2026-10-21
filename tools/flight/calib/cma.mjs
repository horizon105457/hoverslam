// (mu/mu_w, lambda)-CMA-ES (Hansen 2016, "The CMA Evolution Strategy: A Tutorial") for the twin calibration: a
// dozen or two physical parameters, a noisy-ish loss (contact events switch on and off), no gradients. Parameters
// live in a unit box [0, 1]^n mapped to their physical ranges by the caller; the box is enforced by mirroring
// (keeps the search distribution intact near a bound, unlike clipping). Deterministic for a seed (mulberry32).
// usage: const r = await cmaes(f, x0, { sigma: 0.2, seed: 1, maxEvals: 3000, batch: xs => Promise.all(...) })
//   f(x) -> loss (or batch(xs) -> losses, for parallel evaluation); returns { x, f, evals, history }.
import { mulberry32 } from '../../../src/flight/rng.js';

const mirror = v => { let y = v % 2; if (y < 0) y += 2; return y > 1 ? 2 - y : y; };

/** Symmetric eigen-decomposition (cyclic Jacobi): C = B diag(d) B^T, B columns = eigenvectors. */
function eigSym(C, n) {
  const A = C.map(r => r.slice()), V = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => +(i === j)));
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk; }
      for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
    }
  }
  return { d: A.map((r, i) => Math.max(r[i], 1e-20)), B: V };
}

export async function cmaes(f, x0, { sigma = 0.2, seed = 1, lambda = null, maxEvals = 2000, tolFun = 1e-9, batch = null, log = null } = {}) {
  const n = x0.length, rnd = mulberry32(seed);
  const gauss = () => { let u = 0, v = 0; while (u === 0) u = rnd(); v = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
  const lam = lambda || 4 + Math.floor(3 * Math.log(n)), mu = Math.floor(lam / 2);
  const wRaw = Array.from({ length: mu }, (_, i) => Math.log(mu + 0.5) - Math.log(i + 1)), wS = wRaw.reduce((a, b) => a + b, 0);
  const w = wRaw.map(v => v / wS), mueff = 1 / w.reduce((a, b) => a + b * b, 0);
  const cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n), cs = (mueff + 2) / (n + mueff + 5);
  const c1 = 2 / ((n + 1.3) ** 2 + mueff), cmu = Math.min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff));
  const damps = 1 + 2 * Math.max(0, Math.sqrt((mueff - 1) / (n + 1)) - 1) + cs, chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));
  let m = x0.map(mirror), sig = sigma, pc = new Array(n).fill(0), ps = new Array(n).fill(0);
  let C = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => +(i === j))), B = C.map(r => r.slice()), D = new Array(n).fill(1);
  let evals = 0, gen = 0, best = { x: m.slice(), f: Infinity };
  const history = [];
  const evalAll = async xs => batch ? batch(xs) : xs.map(x => f(x));
  while (evals < maxEvals) {
    // sample: y = B D z, x = mirror(m + sigma y)
    const Z = [], Y = [], X = [];
    for (let k = 0; k < lam; k++) {
      const z = Array.from({ length: n }, gauss), y = new Array(n).fill(0);
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) y[i] += B[i][j] * D[j] * z[j];
      Z.push(z); Y.push(y); X.push(m.map((mi, i) => mirror(mi + sig * y[i])));
    }
    const F = await evalAll(X);
    evals += lam; gen++;
    const idx = F.map((v, i) => [Number.isFinite(v) ? v : 1e300, i]).sort((a, b) => a[0] - b[0]).map(e => e[1]);
    if (F[idx[0]] < best.f) best = { x: X[idx[0]].slice(), f: F[idx[0]] };
    // recombination in the mirrored space (the step is taken from the actually evaluated points)
    const mOld = m.slice();
    m = new Array(n).fill(0);
    for (let k = 0; k < mu; k++) for (let i = 0; i < n; i++) m[i] += w[k] * X[idx[k]][i];
    const yw = m.map((v, i) => (v - mOld[i]) / sig);
    // C^-1/2 yw = B D^-1 B^T yw
    const t = new Array(n).fill(0), ciy = new Array(n).fill(0);
    for (let j = 0; j < n; j++) { let s = 0; for (let i = 0; i < n; i++) s += B[i][j] * yw[i]; t[j] = s / D[j]; }
    for (let i = 0; i < n; i++) { let s = 0; for (let j = 0; j < n; j++) s += B[i][j] * t[j]; ciy[i] = s; }
    for (let i = 0; i < n; i++) ps[i] = (1 - cs) * ps[i] + Math.sqrt(cs * (2 - cs) * mueff) * ciy[i];
    const psN = Math.hypot(...ps), hsig = psN / Math.sqrt(1 - (1 - cs) ** (2 * gen)) / chiN < 1.4 + 2 / (n + 1) ? 1 : 0;
    for (let i = 0; i < n; i++) pc[i] = (1 - cc) * pc[i] + hsig * Math.sqrt(cc * (2 - cc) * mueff) * yw[i];
    const Ys = idx.slice(0, mu).map(k => X[k].map((v, i) => (v - mOld[i]) / sig));
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
      let r = 0; for (let k = 0; k < mu; k++) r += w[k] * Ys[k][i] * Ys[k][j];
      const v = (1 - c1 - cmu) * C[i][j] + c1 * (pc[i] * pc[j] + (1 - hsig) * cc * (2 - cc) * C[i][j]) + cmu * r;
      C[i][j] = v; C[j][i] = v;
    }
    sig *= Math.exp((cs / damps) * (psN / chiN - 1));
    sig = Math.min(sig, 0.5);
    const e = eigSym(C, n); B = e.B; D = e.d.map(Math.sqrt);
    const spread = F[idx[lam - 1]] - F[idx[0]];
    history.push({ gen, evals, best: best.f, gen_best: F[idx[0]], sigma: sig });
    log?.({ gen, evals, best: best.f, gen_best: F[idx[0]], median: F[idx[mu]], sigma: sig, x: best.x });
    if (spread < tolFun && sig * Math.max(...D) < 1e-6) break;
  }
  return { x: best.x, f: best.f, evals, history, mean: m, sigma: sig };
}
