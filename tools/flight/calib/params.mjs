// Parameter vector of the twin touchdown calibration: unit box [0, 1]^n <-> physical values (log scale where the
// plausible range spans decades) <-> spec/vehicle.json override (spec/README.md §5.11-5.12). FIXED holds what is
// measured, not fitted: the wheel's axle inertia comes from the soft-body lattice (tools/flight/touchdown setup
// `wheel_axle_inertia`), the underbody points from the lattice geometry (UNDERBODY).
import { VEHICLE } from '../../../src/core/spec.js';

// lowest skin points (rest y + allow, as the drop rig's clearance) of every non-wheel node that is the lowest one for
// some pitch / roll within ±14° (scratch analysis on the h = 0.065 lattice, merged within 5 cm); they replace the
// spec's five y = 0.2 hull points (2.5-5.5 cm below the soft body's real underbody) and cover every hull strike
// location seen in the drop experiments (rear bumper corners, floor by the wheels, sills, front bumper / apron)
export const UNDERBODY = [[-0.583, 0.246, -1.569], [0.721, 0.255, -1.562], [0.649, 0.239, -1.562], [-0.52, 0.244, -1.557], [-0.708, 0.249, -1.556],
  [-0.782, 0.243, -1.369], [0.78, 0.242, -1.364], [0.786, 0.24, -1.299], [0.066, 0.225, -1.246], [0.586, 0.225, -1.246], [-0.581, 0.226, -1.232],
  [0.596, 0.225, -0.722], [0.597, 0.225, -0.524], [-0.321, 0.225, -0.39], [0.785, 0.239, -0.267], [-0.783, 0.24, -0.265], [0.789, 0.24, -0.199],
  [-0.586, 0.225, 0.058], [0.597, 0.225, 1.171], [0.584, 0.225, 1.225], [-0.578, 0.225, 1.226], [-0.13, 0.225, 1.227], [0.782, 0.241, 1.297],
  [-0.774, 0.24, 1.357], [0.717, 0.241, 1.491], [-0.78, 0.246, 1.492], [-0.707, 0.244, 1.548], [0.715, 0.255, 1.554], [-0.641, 0.241, 1.555], [0.525, 0.239, 1.563]];
const UPPER = VEHICLE.hull.points.filter(p => p[1] > 0.3); // roof, belt line, nose / tail: rollover and walls

// name, lo, hi, log scale
export const PARAMS = [
  ['spring_k_f', 12e3, 150e3, 1], ['spring_k_r', 12e3, 250e3, 1],
  ['damper_c_f', 300, 25e3, 1], ['damper_c_r', 300, 25e3, 1],
  ['travel_f', 0.04, 0.25, 0], ['travel_r', 0.04, 0.25, 0],
  ['bump_k_f', 1e4, 3e6, 1], ['bump_k_r', 1e4, 3e6, 1],
  ['bump_c', 10, 6e4, 1], ['bump_k2', 1e3, 1e8, 1],
  ['c_alpha_f', 0.3, 40, 1], ['c_alpha_r', 0.3, 40, 1], ['v_slip_min', 0.1, 10, 1],
  ['mu_f', 0.4, 2.0, 0], ['mu_r', 0.4, 2.0, 0], ['crr', 0.002, 0.1, 1],
  ['v_long_sat', 0.02, 4, 1], ['floor_dy', -0.05, 0.03, 0],
  // alignment: static toe per wheel (rad, + = front edge to the car's right; settled lattice FL .0042 FR .0007
  // RL .0003 RR .0035) and compliance steer per axle (rad/N, + = steers to reduce the lateral force)
  ['toe_fl', -0.02, 0.02, 0], ['toe_fr', -0.02, 0.02, 0], ['toe_rl', -0.02, 0.02, 0], ['toe_rr', -0.02, 0.02, 0],
  ['toe_comp_f', -1e-4, 1e-4, 0], ['toe_comp_r', -1e-4, 1e-4, 0],
  // combined slip (2026-10-21 drive round, spec/README.md 5.11): longitudinal slip measured against κ |v_t| above
  // v_long_sat, and the blend towards isotropic Coulomb sliding with the longitudinal slip (braked / spinning-up tyre)
  ['slip_ratio_sat', 0, 0.3, 0], ['slide', 0, 1, 0],
];
export const FIXED = { inertia: 0.8009, spin_ratio_air: 0.57 };

export function toPhys(x) {
  const o = {};
  PARAMS.forEach(([k, lo, hi, lg], i) => { const u = Math.min(1, Math.max(0, x[i])); o[k] = lg ? lo * (hi / lo) ** u : lo + (hi - lo) * u; });
  return o;
}
export function toUnit(o) {
  return PARAMS.map(([k, lo, hi, lg]) => (lg ? Math.log(o[k] / lo) / Math.log(hi / lo) : (o[k] - lo) / (hi - lo)));
}

/** Physical values -> partial vehicle override (twin `vehicle` option). */
export function toVehicle(p) {
  const r = v => +v.toPrecision(5);
  return {
    wheels: { spring_k: [r(p.spring_k_f), r(p.spring_k_r)], damper_c: [r(p.damper_c_f), r(p.damper_c_r)], travel: [r(p.travel_f), r(p.travel_r)],
      bump_k: [r(p.bump_k_f), r(p.bump_k_r)], bump_c: r(p.bump_c), bump_k2: r(p.bump_k2), tyre_mu: [r(p.mu_f), r(p.mu_r)], crr: r(p.crr),
      toe: [r(p.toe_fl), r(p.toe_fr), r(p.toe_rl), r(p.toe_rr)], toe_compliance: [r(p.toe_comp_f), r(p.toe_comp_r)] },
    tyre: { model: 'slip', c_alpha: [r(p.c_alpha_f), r(p.c_alpha_r)], v_slip_min: r(p.v_slip_min), ...(p.slide !== undefined ? { slide: r(p.slide) } : {}) },
    wheel_spin: { enabled: true, inertia: FIXED.inertia, v_long_sat: r(p.v_long_sat), spin_ratio_air: FIXED.spin_ratio_air, ...(p.slip_ratio_sat !== undefined ? { slip_ratio_sat: r(p.slip_ratio_sat) } : {}) },
    hull: { points: UPPER, floor_points: UNDERBODY.map(([x, y, z]) => [x, +(y + p.floor_dy).toFixed(4), z]) },
  };
}

// a start inside the box near the spec values (v1 springs, a moderately damped stop, a mid-range slip tyre)
export const START = { spring_k_f: 30e3, spring_k_r: 60e3, damper_c_f: 2600, damper_c_r: 2600, travel_f: 0.11, travel_r: 0.11, bump_k_f: 4e5, bump_k_r: 4e5,
  bump_c: 2000, bump_k2: 1e5, c_alpha_f: 10, c_alpha_r: 10, v_slip_min: 1, mu_f: 1, mu_r: 1, crr: 0.015, v_long_sat: 0.5, floor_dy: 0,
  toe_fl: 0.0042, toe_fr: 0.0007, toe_rl: 0.0003, toe_rr: 0.0035, toe_comp_f: 0, toe_comp_r: 0, slip_ratio_sat: 0, slide: 0 };
