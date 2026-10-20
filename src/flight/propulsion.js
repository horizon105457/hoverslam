// Cold-gas propulsion reference model (spec/README.md "Propulsion model v1"): COPV bank with polytropic
// blowdown, regulator with drop-out, proportional valves with first-order lag, choked isentropic nozzles.
// Pure numbers, no three.js; float64, fixed operation order, so the Python mirror (python/coldgas/propulsion.py)
// reproduces it to rounding. Used by the rigid twin and by the soft-body thruster actuator.
import { PROPULSION, WORLD } from '../core/spec.js';

/** Supersonic exit Mach number for area ratio eps (bisection, fixed 80 iterations: deterministic). */
export function exitMach(eps, g) {
  const k = (g + 1) / (2 * (g - 1));
  const ratio = M => (1 / M) * ((2 / (g + 1)) * (1 + 0.5 * (g - 1) * M * M)) ** k;
  let lo = 1, hi = 50;
  for (let i = 0; i < 80; i++) { const mid = 0.5 * (lo + hi); if (ratio(mid) < eps) lo = mid; else hi = mid; }
  return 0.5 * (lo + hi);
}

export class Propulsion {
  /** @param spec spec/propulsion.json content; @param ambient Pa */
  constructor(spec = PROPULSION, ambient = WORLD.air.pressure) {
    this.spec = spec;
    const g = (this.g = spec.gas.gamma), R = (this.R = spec.gas.R);
    this.pa = ambient;
    this.n = spec.nozzles.length;
    this.V = spec.tank.bottles.reduce((s, b) => s + b.volume, 0);
    this.rho0 = spec.tank.p0 / (R * spec.tank.T0);
    this.m0 = this.rho0 * this.V;
    this.K = spec.tank.p0 / this.rho0 ** spec.tank.polytropic_n; // p = K rho^n
    // per nozzle constants: throat/exit area, exit Mach and the isentropic exit ratios
    this.At = new Float64Array(this.n); this.Ae = new Float64Array(this.n);
    this.peRatio = new Float64Array(this.n); this.TeRatio = new Float64Array(this.n); this.Me = new Float64Array(this.n);
    spec.nozzles.forEach((z, i) => {
      const At = Math.PI * 0.25 * z.throat_diameter * z.throat_diameter, M = exitMach(z.expansion_ratio, g);
      const f = 1 + 0.5 * (g - 1) * M * M;
      this.At[i] = At; this.Ae[i] = At * z.expansion_ratio; this.Me[i] = M;
      this.peRatio[i] = f ** (-g / (g - 1)); this.TeRatio[i] = 1 / f;
    });
    this.choke = Math.sqrt(g / R) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1)));
    this.valve = new Float64Array(this.n);
    this.thrust = new Float64Array(this.n);
    this.mdot = new Float64Array(this.n);
    this.reset();
  }

  reset() {
    this.m = this.m0; this.T = this.spec.tank.T0; this.p = this.spec.tank.p0;
    this.pc = 0;
    this.valve.fill(0); this.thrust.fill(0); this.mdot.fill(0);
  }

  /** Feed pressure behind the valves for the current tank state (0 = too low to choke: tank exhausted). */
  feedPressure() {
    const { p_set, dp_min } = this.spec.regulator;
    const pc = Math.min(p_set, this.p - dp_min);
    return pc > 2 * this.pa ? pc : 0;
  }

  /**
   * Advance by dt with commanded openings u (array-like, NU, held over dt). Order: valves -> flows/thrust
   * (from the tank state at the start of the step) -> tank blowdown. Fills this.valve/thrust/mdot.
   * @returns {Float64Array} thrust per nozzle (N, along -exhaust)
   */
  step(u, dt) {
    const { tau_open, tau_close, deadband } = this.spec.valve;
    const n = this.n, s = this.valve;
    for (let i = 0; i < n; i++) {
      let c = u[i];
      c = c > 1 ? 1 : c < deadband ? 0 : c;
      const tau = c > s[i] ? tau_open : tau_close;
      s[i] += (c - s[i]) * (1 - Math.exp(-dt / tau));
    }
    const pc = (this.pc = this.feedPressure()), Tc = this.T;
    let mtot = 0;
    for (let i = 0; i < n; i++) {
      if (pc <= 0 || s[i] <= 0) { this.mdot[i] = 0; this.thrust[i] = 0; continue; }
      const md = s[i] * this.At[i] * pc * this.choke / Math.sqrt(Tc);
      const ve = this.Me[i] * Math.sqrt(this.g * this.R * Tc * this.TeRatio[i]);
      const F = md * ve + s[i] * (pc * this.peRatio[i] - this.pa) * this.Ae[i];
      this.mdot[i] = md; this.thrust[i] = F > 0 ? F : 0;
      mtot += md;
    }
    // polytropic blowdown of the bottle bank
    let m = this.m - mtot * dt;
    if (m < 0) m = 0;
    this.m = m;
    const rho = m / this.V;
    this.p = this.K * rho ** this.spec.tank.polytropic_n;
    this.T = rho > 0 ? this.p / (rho * this.R) : this.T;
    return this.thrust;
  }

  /** Snapshot for logs/trajectories: {m, T, p}. */
  get tank() { return { m: this.m, T: this.T, p: this.p }; }
}
