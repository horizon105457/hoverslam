// Vehicle mass properties (spec/README.md section 4): bare lattice + COPV bottles + nozzles + plumbing + gas.
// Everything except the gas is constant, and the gas enters linearly (split over the bottles by volume), so the
// kit is folded once into first/second moments about the body origin; at(m_g) is then a few dozen flops per
// integrator step:  M = m_dry + m_g,  c = (s_dry + m_g s_gas) / M,  I_c = J_dry + m_g J_gas - M (|c|^2 E - c c^T).
// Mirrored operation for operation by python/coldgas/massprops.py.
import { PROPULSION, GENERATED } from '../core/spec.js';

// symmetric 3x3 as [xx, yy, zz, xy, xz, yz]
function addPoint(J, m, x) { // m (|x|^2 E - x x^T)
  const xx = x[0] * x[0], yy = x[1] * x[1], zz = x[2] * x[2];
  J[0] += m * (yy + zz); J[1] += m * (xx + zz); J[2] += m * (xx + yy);
  J[3] -= m * (x[0] * x[1]); J[4] -= m * (x[0] * x[2]); J[5] -= m * (x[1] * x[2]);
}
function addCylinder(J, m, b) { // solid cylinder about its own centre: I_t E + (I_a - I_t) a a^T
  const al = Math.sqrt(b.axis[0] * b.axis[0] + b.axis[1] * b.axis[1] + b.axis[2] * b.axis[2]);
  const a = [b.axis[0] / al, b.axis[1] / al, b.axis[2] / al], r = 0.5 * b.diameter;
  const Ia = m * (0.5 * r * r), It = m * ((3 * r * r + b.length * b.length) / 12), d = Ia - It;
  J[0] += It + d * a[0] * a[0]; J[1] += It + d * a[1] * a[1]; J[2] += It + d * a[2] * a[2];
  J[3] += d * a[0] * a[1]; J[4] += d * a[0] * a[2]; J[5] += d * a[1] * a[2];
}

export class MassModel {
  constructor(prop = PROPULSION, mp = GENERATED.massprops) {
    const L = mp.lattice, Jd = new Float64Array(6), sd = new Float64Array(3);
    let md = 0;
    const point = (m, x) => { md += m; sd[0] += m * x[0]; sd[1] += m * x[1]; sd[2] += m * x[2]; addPoint(Jd, m, x); };
    // order fixed by spec/README.md section 4: lattice, bottles (dry), nozzles, hardware
    point(L.mass, L.com);
    const I = L.inertia;
    Jd[0] += I[0][0]; Jd[1] += I[1][1]; Jd[2] += I[2][2]; Jd[3] += I[0][1]; Jd[4] += I[0][2]; Jd[5] += I[1][2];
    for (const b of prop.tank.bottles) { point(b.dry_mass, b.center); addCylinder(Jd, b.dry_mass, b); }
    for (const z of prop.nozzles) point(z.mass, z.mount);
    for (const h of prop.hardware) point(h.mass, h.center);
    // gas per kg, split by bottle volume
    let V = 0;
    for (const b of prop.tank.bottles) V += b.volume;
    const Jg = new Float64Array(6), sg = new Float64Array(3);
    for (const b of prop.tank.bottles) {
      const w = b.volume / V;
      sg[0] += w * b.center[0]; sg[1] += w * b.center[1]; sg[2] += w * b.center[2];
      addPoint(Jg, w, b.center); addCylinder(Jg, w, b);
    }
    Object.assign(this, { mDry: md, sDry: sd, JDry: Jd, sGas: sg, JGas: Jg });
  }

  /**
   * Mass properties with m_g kg of gas aboard, written into `out` ({M, com: Float64Array(3), I: Float64Array(6),
   * Iinv: Float64Array(6)}, symmetric 3x3 as [xx, yy, zz, xy, xz, yz], body axes about the COM).
   */
  at(mg, out = { M: 0, com: new Float64Array(3), I: new Float64Array(6), Iinv: new Float64Array(6) }) {
    const M = this.mDry + mg, s = this.sDry, sg = this.sGas, J = this.JDry, Jg = this.JGas;
    const cx = (s[0] + mg * sg[0]) / M, cy = (s[1] + mg * sg[1]) / M, cz = (s[2] + mg * sg[2]) / M;
    const I = out.I, c = out.com;
    c[0] = cx; c[1] = cy; c[2] = cz;
    I[0] = (J[0] + mg * Jg[0]) - M * (cy * cy + cz * cz);
    I[1] = (J[1] + mg * Jg[1]) - M * (cx * cx + cz * cz);
    I[2] = (J[2] + mg * Jg[2]) - M * (cx * cx + cy * cy);
    I[3] = (J[3] + mg * Jg[3]) + M * (cx * cy);
    I[4] = (J[4] + mg * Jg[4]) + M * (cx * cz);
    I[5] = (J[5] + mg * Jg[5]) + M * (cy * cz);
    out.M = M;
    invSym(I, out.Iinv);
    return out;
  }
}

/** Inverse of a symmetric 3x3 [xx, yy, zz, xy, xz, yz] by cofactors. */
export function invSym(I, out) {
  const a = I[0], b = I[1], c = I[2], d = I[3], e = I[4], f = I[5];
  const A = b * c - f * f, B = e * f - d * c, C = d * f - e * b;
  const det = a * A + d * B + e * C;
  out[0] = A / det; out[1] = (a * c - e * e) / det; out[2] = (a * b - d * d) / det;
  out[3] = B / det; out[4] = C / det; out[5] = (d * e - a * f) / det;
  return out;
}
