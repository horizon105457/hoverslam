// mulberry32 exactly as rng() in src/softbody/solver.js (spec/README.md section 7); mirrored by
// python/coldgas/rng.py. Task randomisation and parity scenarios draw from it in a documented order.
export function mulberry32(seed) {
  let s = seed >>> 0 || 1;
  const r = () => { s |= 0; s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  r.uniform = (a, b) => a + (b - a) * r();
  // Standard normal (Irwin-Hall: sum of 12 draws - 6, mean 0, variance 1, tails cut at +-6). Every draw is a
  // multiple of 2^-32 below 1, so the sum is exact in float64 in any order: Python (rng.py gauss_array, numpy)
  // gets the same bits without a portable log/cos (MPPI exploration noise).
  r.gauss = () => { let z = 0; for (let i = 0; i < 12; i++) z += r(); return z - 6; };
  return r;
}
