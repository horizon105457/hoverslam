// Twin structure constants (spec/vehicle.json `twin`, shared with python/coldgas/model.py) and the contact
// parameters of one twin instance: spec VEHICLE, optionally deep-merged with a partial override (fitting and
// experiments only), resolved per wheel / per hull point, and the brake / steer constants of spec/drive.json (the
// car's own hardware, not overridable). Semantics: spec/README.md section 5 (5.11, 5.12, 5.13).
import { VEHICLE, GENERATED, DRIVE } from '../core/spec.js';
import { cos, sin } from './fmath.js';
export const MODEL = VEHICLE.twin;
export const MODEL_ID = MODEL.model;

const isObj = x => x !== null && typeof x === 'object' && !Array.isArray(x);
/**
 * Deep merge for parameter overrides: objects key by key, arrays / scalars / null replace (same as python). Keys
 * the spec does not have are an error (a misspelt parameter would otherwise be silently ignored by a fit).
 */
export function mergeParams(a, b, path = 'vehicle') {
  const out = { ...a };
  for (const [k, v] of Object.entries(b || {})) {
    if (!(k in a)) throw new Error(`${path}.${k}: no such parameter in spec/vehicle.json`);
    out[k] = isObj(v) && isObj(a[k]) ? mergeParams(a[k], v, `${path}.${k}`) : v;
  }
  return out;
}

/** A number (both axles) or [front, rear] -> [front, rear]. */
function perAxle(x, name) {
  if (typeof x === 'number') return [x, x];
  if (Array.isArray(x) && x.length === 2 && x.every(y => typeof y === 'number')) return [x[0], x[1]];
  throw new Error(`vehicle ${name}: expected a number or [front, rear], got ${JSON.stringify(x)}`);
}
const thresh = (x, name) => {
  if (x === null || x === undefined) return Infinity; // off: `>` never fires
  if (typeof x !== 'number' || !(x > 0)) throw new Error(`vehicle failure.${name}: expected null or a positive number`);
  return x;
};

/**
 * Brakes and steering (spec/drive.json, spec/README.md 5.13): brake torque per wheel [front, rear] (N m), pressure lag
 * tau (s), ABS slip bound, steer lock and rate in rad, rad/s (same operand order as python: deg * pi / 180).
 */
export function driveParams() {
  const B = DRIVE.brake, S = DRIVE.steer;
  const torque = perAxle(B.torque_max, 'drive brake.torque_max');
  if (!(torque.every(x => x >= 0) && B.tau > 0 && B.abs_slip >= 0 && B.abs_slip < 1 && S.max_deg >= 0 && S.rate_deg_s >= 0)) {
    throw new Error('spec/drive.json: need torque_max >= 0, tau > 0, 0 <= abs_slip < 1, max_deg >= 0, rate_deg_s >= 0');
  }
  return { torque, tau: B.tau, absSlip: B.abs_slip, steerMax: S.max_deg * Math.PI / 180, steerRate: S.rate_deg_s * Math.PI / 180 };
}

/**
 * Contact parameters of one twin: `override` (partial vehicle.json object, or null) merged over VEHICLE, then
 * resolved. Wheels j follow generated massprops order; axle 0 (front) = centre z > 0, 1 (rear) otherwise. Hull
 * points = hull.points followed by hull.floor_points (each with its own k / c / mu; null = the hull's).
 */
export function contactParams(override = null) {
  if (override && override.integrator !== undefined) throw new Error('vehicle.integrator: use the twin\'s dt option');
  const V = override ? mergeParams(VEHICLE, override) : VEHICLE;
  const H = V.hull, TY = V.tyre, SP = V.wheel_spin, FA = V.failure, C = V.twin.contact;
  if (TY.model !== 'coulomb' && TY.model !== 'slip') throw new Error(`vehicle tyre.model: 'coulomb' or 'slip', got ${TY.model}`);
  const get = name => perAxle(name.split('.').reduce((o, k) => o[k], V), name);
  const ax = Object.fromEntries(['wheels.spring_k', 'wheels.damper_c', 'wheels.travel', 'wheels.bump_k', 'wheels.bump_c', 'wheels.bump_k2',
    'wheels.tyre_mu', 'wheels.crr', 'wheels.toe_compliance', 'tyre.c_alpha', 'tyre.v_slip_min', 'wheel_spin.inertia', 'wheel_spin.v_long_sat',
    'wheel_spin.slip_ratio_sat', 'tyre.slide'].map(n => [n, get(n)]));
  const toe = V.wheels.toe, nw = GENERATED.massprops.wheels.length, Tb = driveParams().torque;
  if (!(Array.isArray(toe) && toe.length === nw && toe.every(x => typeof x === 'number'))) throw new Error(`vehicle wheels.toe: expected ${nw} numbers (FL FR RL RR), got ${JSON.stringify(toe)}`);
  // one object per wheel: geometry + axle-resolved parameters (the twin adds its per-wheel state to it); the static
  // toe enters as its cos / sin (portable fmath, so both languages rotate the tyre axes identically)
  const wheels = GENERATED.massprops.wheels.map((w, j) => {
    const a = w.centre[2] > 0 ? 0 : 1, A = n => ax[n][a];
    return { c: Float64Array.from(w.centre), r: w.radius, axle: a, ks: A('wheels.spring_k'), cs: A('wheels.damper_c'), tr: A('wheels.travel'),
      kb: A('wheels.bump_k'), bc: A('wheels.bump_c'), kb2: A('wheels.bump_k2'), mu: A('wheels.tyre_mu'), crr: A('wheels.crr'),
      ca: A('tyre.c_alpha'), vsm: A('tyre.v_slip_min'), I: A('wheel_spin.inertia'), vls: A('wheel_spin.v_long_sat'), kss: A('wheel_spin.slip_ratio_sat'), sld: A('tyre.slide'),
      toe: toe[j], tc: cos(toe[j]), ts: sin(toe[j]), kap: A('wheels.toe_compliance'), Tb: Tb[a] };
  });
  const floor = H.floor_points || [];
  const pts = [...H.points, ...floor];
  const pick = (x, d) => (x === null || x === undefined ? d : x);
  const P = {
    V, wheels, ksAxle: ax['wheels.spring_k'],
    slip: TY.model === 'slip', spinOn: !!SP.enabled, spinAir: SP.spin_ratio_air, spinDecay: SP.decay,
    failN: thresh(FA.wheel_normal_max, 'wheel_normal_max'), failL: thresh(FA.wheel_lat_max, 'wheel_lat_max'),
    failI: thresh(FA.wheel_lat_impulse_max, 'wheel_lat_impulse_max'),
    hull: pts.map(x => Float64Array.from(x)),
    kh: Float64Array.from(pts, (_, j) => (j < H.points.length ? H.k : pick(H.floor_k, H.k))),
    ch: Float64Array.from(pts, (_, j) => (j < H.points.length ? H.c : pick(H.floor_c, H.c))),
    muh: Float64Array.from(pts, (_, j) => (j < H.points.length ? H.mu : pick(H.floor_mu, H.mu))),
    vreg: C.v_reg, hmin: C.wheel_min_h, placement: V.twin.placement, CdA: V.aero.CdA,
  };
  // the v2 bump-stop terms are skipped entirely when zero (v1 expressions, v1 speed)
  P.bumpX = wheels.some(w => w.bc !== 0 || w.kb2 !== 0);
  P.complyOn = wheels.some(w => w.kap !== 0);
  P.toeOn = P.complyOn || wheels.some(w => w.toe !== 0); // tyre axes rotated (else v1 axes, no rotation evaluated)
  P.failOn = P.failN !== Infinity || P.failL !== Infinity || P.failI !== Infinity;
  if (P.spinOn && !wheels.every(w => w.I > 0 && w.vls > 0)) throw new Error('vehicle wheel_spin: inertia and v_long_sat must be > 0');
  if (!wheels.every(w => w.kss >= 0)) throw new Error('vehicle wheel_spin.slip_ratio_sat must be >= 0');
  if (!wheels.every(w => w.sld >= 0 && w.sld <= 1)) throw new Error('vehicle tyre.slide must be in [0, 1]');
  if (P.slip && !wheels.every(w => w.vsm > 0)) throw new Error('vehicle tyre.v_slip_min must be > 0');
  return P;
}
