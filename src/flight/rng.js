// mulberry32 exactly as rng() in src/softbody/solver.js (spec/README.md section 7); mirrored by
// python/coldgas/rng.py. Task randomisation and parity scenarios draw from it in a documented order.
export function mulberry32(seed) {
  let s = seed >>> 0 || 1;
  const r = () => { s |= 0; s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  r.uniform = (a, b) => a + (b - a) * r();
  return r;
}
