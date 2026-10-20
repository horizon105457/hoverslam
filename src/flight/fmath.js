// Portable transcendental functions for the flight twins: fdlibm 5.3 algorithms (e_exp.c, k_sin.c, k_cos.c,
// e_rem_pio2.c, e_acos.c) written out with plain IEEE double arithmetic, mirrored line by line in
// python/coldgas/fmath.py. Why: Math.sin/cos/acos/tanh differ between engines (V8 uses glibc-derived trig) and
// from CPython/numpy (platform libm) in the last bit for a few percent of inputs; the JS and Python twins must
// agree to rounding, so everything the twins feed into dynamics, rewards or policies goes through these.
// (Math.pow and Math.sqrt agree bit for bit with CPython and are used directly.)

const F = new Float64Array(1), U = new Uint32Array(F.buffer); // little endian: U[1] = high word
const hiOf = x => { F[0] = x; return U[1] | 0; };
const loOf = x => { F[0] = x; return U[0]; };
const withLo0 = x => { F[0] = x; U[0] = 0; return F[0]; };
const fromHi = h => { U[1] = h; U[0] = 0; return F[0]; };
const ldexp = (y, k) => { // exact y * 2^k for |k| <= 2000 (y normal, result normal)
  while (k > 1000) { y *= 2 ** 1000; k -= 1000; }
  while (k < -1000) { y *= 2 ** -1000; k += 1000; }
  return y * 2 ** k;
};

const ln2HI = [6.93147180369123816490e-01, -6.93147180369123816490e-01];
const ln2LO = [1.90821492927058770002e-10, -1.90821492927058770002e-10];
const invln2 = 1.44269504088896338700e+00;
const P1 = 1.66666666666666019037e-01, P2 = -2.77777777770155933842e-03, P3 = 6.61375632143793436117e-05;
const P4 = -1.65339022054652515390e-06, P5 = 4.13813679705723846039e-08;

export function exp(x) {
  if (x !== x) return x;
  let hx = hiOf(x);
  const xsb = (hx >>> 31) & 1;
  hx &= 0x7fffffff;
  if (hx >= 0x40862E42) {
    if (hx >= 0x7ff00000) return xsb === 0 ? x : 0;
    if (x > 7.09782712893383973096e+02) return Infinity;
    if (x < -7.45133219101941108420e+02) return 0;
  }
  let k = 0, hi = 0, lo = 0;
  if (hx > 0x3fd62e42) {
    if (hx < 0x3FF0A2B2) { hi = x - ln2HI[xsb]; lo = ln2LO[xsb]; k = 1 - xsb - xsb; }
    else { k = Math.trunc(invln2 * x + (xsb ? -0.5 : 0.5)); const t = k; hi = x - t * ln2HI[0]; lo = t * ln2LO[0]; }
    x = hi - lo;
  } else if (hx < 0x3e300000) return 1 + x;
  const t = x * x;
  const c = x - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5))));
  if (k === 0) return 1 - ((x * c) / (c - 2.0) - x);
  return ldexp(1 - ((lo - (x * c) / (2.0 - c)) - hi), k);
}

const S1 = -1.66666666666666324348e-01, S2 = 8.33333333332248946124e-03, S3 = -1.98412698298579493134e-04;
const S4 = 2.75573137070700676789e-06, S5 = -2.50507602534068634195e-08, S6 = 1.58969099521155010221e-10;
const C1 = 4.16666666666666019037e-02, C2 = -1.38888888888741095749e-03, C3 = 2.48015872894767294178e-05;
const C4 = -2.75573143513906633035e-07, C5 = 2.08757232129817482790e-09, C6 = -1.13596475577881948265e-11;

function ksin(x, y, iy) {
  const ix = hiOf(x) & 0x7fffffff;
  if (ix < 0x3e400000 && Math.trunc(x) === 0) return x;
  const z = x * x, v = z * x, r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
  if (iy === 0) return x + v * (S1 + z * r);
  return x - ((z * (0.5 * y - v * r) - y) - v * S1);
}
function kcos(x, y) {
  const ix = hiOf(x) & 0x7fffffff;
  if (ix < 0x3e400000 && Math.trunc(x) === 0) return 1;
  const z = x * x, r = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
  if (ix < 0x3FD33333) return 1 - (0.5 * z - (z * r - x * y));
  const qx = ix > 0x3fe90000 ? 0.28125 : fromHi(ix - 0x00200000);
  const hz = 0.5 * z - qx, a = 1 - qx;
  return a - (hz - (z * r - x * y));
}

const invpio2 = 6.36619772367581382433e-01, pio2_1 = 1.57079632673412561417e+00, pio2_1t = 6.07710050650619224932e-11;
const pio2_2 = 6.07710050630396597660e-11, pio2_2t = 2.02226624879595063154e-21, pio2_3 = 2.02226624871116645580e-21;
const pio2_3t = 8.47842766036889956997e-32;
const RY = [0, 0];
/** fdlibm __ieee754_rem_pio2 restricted to |x| <= 2^19 pi/2; returns n, writes the remainder to RY. */
function remPio2(x) {
  const hx = hiOf(x), ix = hx & 0x7fffffff;
  if (ix < 0x4002D97C) {
    let z;
    if (hx > 0) {
      z = x - pio2_1;
      if (ix !== 0x3ff921fb) { RY[0] = z - pio2_1t; RY[1] = (z - RY[0]) - pio2_1t; }
      else { z -= pio2_2; RY[0] = z - pio2_2t; RY[1] = (z - RY[0]) - pio2_2t; }
      return 1;
    }
    z = x + pio2_1;
    if (ix !== 0x3ff921fb) { RY[0] = z + pio2_1t; RY[1] = (z - RY[0]) + pio2_1t; }
    else { z += pio2_2; RY[0] = z + pio2_2t; RY[1] = (z - RY[0]) + pio2_2t; }
    return -1;
  }
  if (ix > 0x413921fb) throw new Error('fmath: argument too large for the medium-range reduction');
  let t = Math.abs(x);
  const n = Math.trunc(t * invpio2 + 0.5), fn = n;
  let r = t - fn * pio2_1, w = fn * pio2_1t;
  const j = ix >> 20;
  let y0 = r - w, i = j - ((hiOf(y0) >> 20) & 0x7ff);
  if (i > 16) {
    t = r; w = fn * pio2_2; r = t - w; w = fn * pio2_2t - ((t - r) - w); y0 = r - w;
    i = j - ((hiOf(y0) >> 20) & 0x7ff);
    if (i > 49) { t = r; w = fn * pio2_3; r = t - w; w = fn * pio2_3t - ((t - r) - w); y0 = r - w; }
  }
  const y1 = (r - y0) - w;
  if (hx < 0) { RY[0] = -y0; RY[1] = -y1; return -n; }
  RY[0] = y0; RY[1] = y1;
  return n;
}

export function sin(x) {
  const ix = hiOf(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return ksin(x, 0, 0);
  if (ix >= 0x7ff00000) return x - x;
  const n = remPio2(x) & 3, y0 = RY[0], y1 = RY[1];
  return n === 0 ? ksin(y0, y1, 1) : n === 1 ? kcos(y0, y1) : n === 2 ? -ksin(y0, y1, 1) : -kcos(y0, y1);
}
export function cos(x) {
  const ix = hiOf(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return kcos(x, 0);
  if (ix >= 0x7ff00000) return x - x;
  const n = remPio2(x) & 3, y0 = RY[0], y1 = RY[1];
  return n === 0 ? kcos(y0, y1) : n === 1 ? -ksin(y0, y1, 1) : n === 2 ? -kcos(y0, y1) : ksin(y0, y1, 1);
}

const pi = 3.14159265358979311600e+00, pio2_hi = 1.57079632679489655800e+00, pio2_lo = 6.12323399573676603587e-17;
const pS0 = 1.66666666666666657415e-01, pS1 = -3.25565818622400915405e-01, pS2 = 2.01212532134862925881e-01;
const pS3 = -4.00555345006794114027e-02, pS4 = 7.91534994289814532176e-04, pS5 = 3.47933107596021167570e-05;
const qS1 = -2.40339491173441421878e+00, qS2 = 2.02094576023350569471e+00, qS3 = -6.88283971605453293030e-01;
const qS4 = 7.70381505559019352791e-02;

export function acos(x) {
  const hx = hiOf(x), ix = hx & 0x7fffffff;
  if (ix >= 0x3ff00000) {
    if (((ix - 0x3ff00000) | loOf(x)) === 0) return hx > 0 ? 0 : pi + 2.0 * pio2_lo;
    return NaN;
  }
  let z, p, q;
  if (ix < 0x3fe00000) {
    if (ix <= 0x3c600000) return pio2_hi + pio2_lo;
    z = x * x;
    p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
    q = 1 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
    return pio2_hi - (x - (pio2_lo - x * (p / q)));
  }
  if (hx < 0) {
    z = (1 + x) * 0.5;
    p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
    q = 1 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
    const s = Math.sqrt(z), w = (p / q) * s - pio2_lo;
    return pi - 2.0 * (s + w);
  }
  z = (1 - x) * 0.5;
  const s = Math.sqrt(z), df = withLo0(s), c = (z - df * df) / (s + df);
  p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
  q = 1 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
  return 2.0 * (df + ((p / q) * s + c));
}

/** MLP activation of coldgas-policy@1 (not fdlibm tanh): sign(x) (1 - 2 / (exp(2|x|) + 1)), saturating at 20. */
export function tanh(x) {
  const a = Math.abs(x);
  if (a > 20) return x < 0 ? -1 : 1;
  const t = 1 - 2 / (exp(2 * a) + 1);
  return x >= 0 ? t : -t;
}
