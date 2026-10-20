// Quaternion helpers on plain arrays [x, y, z, w] (three.js order). Rotation matrices are row-major 3x3.
// Shared by the soft-body recording, the flight twin and renderers; keep them allocation-free.

export function quatToMat(q, m) {
  const [x, y, z, w] = q;
  m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y - z * w); m[2] = 2 * (x * z + y * w);
  m[3] = 2 * (x * y + z * w); m[4] = 1 - 2 * (x * x + z * z); m[5] = 2 * (y * z - x * w);
  m[6] = 2 * (x * z - y * w); m[7] = 2 * (y * z + x * w); m[8] = 1 - 2 * (x * x + y * y);
  return m;
}
export function quatMul(a, b, out) {
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  out[0] = aw * bx + ax * bw + ay * bz - az * by;
  out[1] = aw * by - ax * bz + ay * bw + az * bx;
  out[2] = aw * bz + ax * by - ay * bx + az * bw;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
  return out;
}
export function quatNorm(q) { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; q[0] /= l; q[1] /= l; q[2] /= l; q[3] /= l; return q; }
export function slerp(a, b, t, out) {
  let bx = b[0], by = b[1], bz = b[2], bw = b[3];
  let cos = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
  if (cos < 0) { cos = -cos; bx = -bx; by = -by; bz = -bz; bw = -bw; }
  let k0, k1;
  if (cos > 0.9995) { k0 = 1 - t; k1 = t; }
  else { const th = Math.acos(cos), s = Math.sin(th); k0 = Math.sin((1 - t) * th) / s; k1 = Math.sin(t * th) / s; }
  out[0] = a[0] * k0 + bx * k1; out[1] = a[1] * k0 + by * k1; out[2] = a[2] * k0 + bz * k1; out[3] = a[3] * k0 + bw * k1;
  return quatNorm(out);
}
