// Rigid 6-DOF flight twin of the MINI EV with the cold-gas kit (spec/README.md section 5 is the contract).
// Fixed step vehicle.integrator.dt, SUBSTEPS integrator steps per control step. Per integrator step:
//   kick -> propulsion (reference model) -> drive (brake pressure lag, rate-limited steer: 5.13) ->
//   contact / thrust / aero forces at the state of the step start ->
//   semi-implicit Euler for v, p; midpoint for the Euler equations; first-order quaternion update ->
//   mass properties from the remaining gas with the kinematic COM shift.
// Float64 throughout, no allocation in the step, and every expression is written in the same order as the numpy
// mirror (python/coldgas/twin.py) so both agree bit for bit (tests/parity/). Change both or neither.
// Contact parameters come from the twin's vehicle object (spec, or spec + override for fitting: model.js). The
// touchdown extensions (per-axle suspension, bump damping, slip tyre, toe, wheel spin, floor points, wheel loads and
// the overload criterion; spec/README.md 5.11) run the v1 expressions unchanged with the spec values. Brakes and
// steering (5.13) are skipped entirely while command and state are 0, so zero drive is the old physics bit for bit.
import { WORLD, LAUNCH, PROPULSION, GENERATED } from '../core/spec.js';
import { Propulsion } from './propulsion.js';
import { MassModel } from './massprops.js';
import { contactParams, driveParams } from './model.js';
import { BOXES, GROUND, KIND, supportLevel } from './world.js';
import { cos, sin, exp } from './fmath.js';
import { NU, SUBSTEPS, DT, DRIVE_IDX } from './contract.js';

/** Row-major rotation matrix of q = [x, y, z, w] (same expressions as src/core/quat.js quatToMat). */
export function quatToMat(q, m) {
  const x = q[0], y = q[1], z = q[2], w = q[3];
  m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y - z * w); m[2] = 2 * (x * z + y * w);
  m[3] = 2 * (x * y + z * w); m[4] = 1 - 2 * (x * x + z * z); m[5] = 2 * (y * z - x * w);
  m[6] = 2 * (x * z - y * w); m[7] = 2 * (y * z + x * w); m[8] = 1 - 2 * (x * x + y * y);
  return m;
}
/** a ⊗ b, [x, y, z, w] (same expressions as src/core/quat.js quatMul). */
export function quatMul(a, b, out) {
  const ax = a[0], ay = a[1], az = a[2], aw = a[3], bx = b[0], by = b[1], bz = b[2], bw = b[3];
  out[0] = aw * bx + ax * bw + ay * bz - az * by;
  out[1] = aw * by - ax * bz + ay * bw + az * bx;
  out[2] = aw * bz + ax * by - ay * bx + az * bw;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
  return out;
}
/** Unit quaternion for a yaw about +Y with cos c, sin s (no trig: half angle from c, s). */
export function quatYaw(c, s) {
  let y, w;
  if (c > -0.5) { w = Math.sqrt((1 + c) / 2); y = s / (2 * w); }
  else { y = Math.sqrt((1 - c) / 2); if (s < 0) y = -y; w = s / (2 * y); }
  return [0, y, 0, w];
}

const DECK = WORLD.deck, KICK = LAUNCH.kick;
const NB = BOXES.length;
const OVERLOAD = ['normal', 'lat', 'lat_impulse']; // wheel_overload event quantities (spec/README.md 5.11)
const IB = DRIVE_IDX.brake, IS = DRIVE_IDX.steer; // action indices of the drive commands (5.13)
// brake pressure snaps onto its command once within this (model constant, 5.13): without it the lag would leave a
// subnormal tail after a release (beta * k_beta rounds to 0) and the brake arithmetic could never be skipped again
const BETA_SNAP = 1e-9;

/**
 * Initial conditions (spec/README.md section 6 / 8):
 *  {type:'launch', speed (km/h), angle (deg), spin [pitch, yaw, roll] (rad/s), lead (s)}
 *  {type:'rest', x, z, heading_deg}       COM above (x, z), on its wheels on whatever surface is there
 *  {type:'state', p, q, v, w, gas?, valve?, leftDeck?, kick?: {spin_w: [3]}, wheel_spin?: [4]}  explicit (tests,
 *                                       soft-body hand-off); wheel_spin (rad/s) only matters with wheel_spin.enabled
 * Drive (5.13): brake pressure `beta` (0..1) and front road-wheel angle `delta` (rad), 0 after reset(); `drive`
 * reads both, `ud` holds the applied [brake, steer] commands of the last control step, `kerb` the first kerb hit.
 */
export class RigidTwin {
  /**
   * dt: integrator step override for planning models only (MPPI, like coldgas.twin.Twin(dt=)); never for parity.
   * vehicle: partial spec/vehicle.json object deep-merged over the spec (wheels, tyre, wheel_spin, failure, hull,
   * aero, twin.contact / placement) for fitting and experiments (spec/README.md 5.12); integrator.dt is not taken
   * from it (use dt). Every contact parameter is read from it here, never from module constants.
   */
  constructor({ arm = 'start', dt = null, vehicle = null } = {}) {
    this.arm = arm;
    this.dt = dt ?? DT; this.nsub = dt == null ? SUBSTEPS : pyRound(PROPULSION.control.dt / dt); this.g = WORLD.gravity;
    this.prop = new Propulsion(PROPULSION, WORLD.air.pressure);
    this.mass = new MassModel(PROPULSION, GENERATED.massprops);
    const P = contactParams(vehicle);
    this.vehicle = P.V; this.params = P;
    // wheels: one object each with geometry and axle-resolved parameters (model.js), plus the twin's wheel state:
    // spin (rad/s, + = rolling forward), last-substep normal / lateral / longitudinal force N, lat, long, and the
    // per-episode peaks pN, pL (from the deck edge on) and lateral impulse pI since touchdown (spec/README.md 5.11).
    // Plain fields on objects the loop already holds: as cheap as the v1 scalars (typed arrays per parameter were not).
    // Every field is created as a double first: a field born from a JSON integer (30000) is a Smi field in V8 and
    // each later double store into it would box (allocate) in the step.
    const D = 0.5;
    // Drive (5.13): Tb = brake torque limit, c0 / s0 = cos / sin of the wheel's heading before compliance steer (static
    // toe, front wheels turned by the steer angle delta; = tc / ts exactly while delta = 0).
    this.wheels = P.wheels.map(w => Object.assign({ c: w.c, r: D, axle: 0, ks: D, cs: D, tr: D, kb: D, bc: D, kb2: D, mu: D, crr: D, ca: D,
      vsm: D, I: D, vls: D, kss: D, sld: D, toe: D, tc: D, ts: D, kap: D, A: D, spin: D, N: D, lat: D, long: D, pN: D, pL: D, pI: D, Tb: D, c0: D, s0: D },
    w, { A: w.I / this.dt, spin: 0, N: 0, lat: 0, long: 0, pN: 0, pL: 0, pI: 0, c0: w.tc, s0: w.ts }));
    Object.assign(this, { bumpX: P.bumpX, slip: P.slip, spinOn: P.spinOn, spinAir: P.spinAir, spinKeep: 1 - this.dt * P.spinDecay });
    Object.assign(this, { toeOn: P.toeOn, complyOn: P.complyOn });
    Object.assign(this, { hull: P.hull, kh: P.kh, ch: P.ch, hullMu: P.muh, vreg: P.vreg, hmin: P.hmin, placement: P.placement });
    Object.assign(this, { failOn: P.failOn, failN: P.failN, failL: P.failL, failI: P.failI });
    this.wheelOverload = null; this.fLat = 0;
    // drive constants (spec/drive.json): pressure lag per integrator step through the portable exp (both languages
    // agree), ABS target factor, steer lock and per-step rate limit (operand order as python)
    const DP = driveParams();
    this.kBrake = 1 - exp(-this.dt / DP.tau); this.absKeep = 1 - DP.absSlip;
    this.steerMax = DP.steerMax; this.steerStep = DP.steerRate * this.dt;
    this.beta = D; this.delta = D; this.steerOn = false; this.kerb = null; // doubles first (see above), zeroed by reset()
    this.ud = new Float64Array(2);
    // thrust direction (-exhaust, unit) and exit point per nozzle, body frame
    this.ndir = new Float64Array(NU * 3); this.nexit = new Float64Array(NU * 3);
    PROPULSION.nozzles.forEach((z, i) => {
      const e = z.exhaust, l = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
      this.ndir[i * 3] = -e[0] / l; this.ndir[i * 3 + 1] = -e[1] / l; this.ndir[i * 3 + 2] = -e[2] / l;
      this.nexit.set(z.exit, i * 3);
    });
    const rho = WORLD.air.density;
    this.ka = Float64Array.from(P.CdA, A => 0.5 * rho * A);
    this.nKick = Math.round(KICK.dur / this.dt);
    // broadphase radius: farthest contact point from the COM for a full and an empty tank, plus margin
    let rb = 0;
    for (const mg of [this.prop.m0, 0]) {
      const c = this.mass.at(mg).com;
      for (const x of this.hull) rb = Math.max(rb, Math.hypot(x[0] - c[0], x[1] - c[1], x[2] - c[2]));
      for (const w of this.wheels) rb = Math.max(rb, Math.hypot(w.c[0] - c[0], w.c[1] - c[1], w.c[2] - c[2]) + w.r);
    }
    this.rBroad = rb + 0.5;
    // state
    this.p = new Float64Array(3); this.q = new Float64Array([0, 0, 0, 1]); this.v = new Float64Array(3); this.w = new Float64Array(3);
    this.mp = { M: 0, com: new Float64Array(3), I: new Float64Array(6), Iinv: new Float64Array(6) };
    this.mp2 = { M: 0, com: new Float64Array(3), I: new Float64Array(6), Iinv: new Float64Array(6) };
    this.R = new Float64Array(9); this.R2 = new Float64Array(9);
    this.u = new Float64Array(NU);
    this.active = new Int32Array(NB);
    this.kickDw = new Float64Array(3);
    this.events = [];
  }

  // ------------------------------------------------------------------------------------------------ reset
  reset(init) {
    this.prop.reset();
    this.mass.at(this.prop.m, this.mp);
    this.k = 0; this.events = [];
    this.kickState = 2; this.kickLeft = 0; this.kickDw.fill(0);
    this.launch = null; this.leftDeck = true; this.armed = false; this.airborne = false; this.touchdown = null;
    this.hullStrike = null; this.peakSf = 0; this.wheelMask = 0; this.dropout = false;
    this.stepFlags = { any: false, street: false, deck: false, other: false, hull: false, kerb: false };
    this.v.fill(0); this.w.fill(0);
    if (init.type === 'launch') {
      const th = (init.angle * Math.PI) / 180, s = init.speed / 3.6, lead = init.lead ?? LAUNCH.defaults.lead;
      const ct = cos(th), st = sin(th);
      const f = [-ct, 0, st], r = [st, 0, ct];
      const vx = s * f[0], vz = s * f[2];
      const xc = (DECK.gap[0] + DECK.gap[1]) / 2 + 1.5 - vx * lead, zc = DECK.halfWidth - vz * lead - 0.9;
      // body (X, Y, Z) -> (r, up, f): yaw phi with cos phi = sin th, sin phi = -cos th
      this.settle(xc, zc, supportLevel(xc, zc), quatYaw(st, -ct));
      this.v[0] = vx; this.v[1] = 0; this.v[2] = vz;
      const sp = init.spin[0], sy = init.spin[1], sr = init.spin[2], nk = this.nKick;
      this.kickDw[0] = (sp * r[0] + sr * f[0]) / nk; this.kickDw[1] = sy / nk; this.kickDw[2] = (sp * r[2] + sr * f[2]) / nk;
      this.kickState = 0; this.leftDeck = false;
      this.launch = { speed: s, angle: init.angle, f, r, x: xc, z: zc };
    } else if (init.type === 'rest') {
      const ps = (init.heading_deg * Math.PI) / 180;
      this.settle(init.x, init.z, supportLevel(init.x, init.z), quatYaw(cos(ps), sin(ps)));
      this.launch = { x: init.x, z: init.z };
    } else if (init.type === 'state') {
      const P = this.prop;
      if (init.gas !== undefined) { // tank state on the blowdown polytrope for that much gas (propulsion model step 4)
        P.m = init.gas;
        const rho = P.m / P.V;
        P.p = P.K * rho ** P.spec.tank.polytropic_n;
        P.T = rho > 0 ? P.p / (rho * P.R) : P.spec.tank.T0;
      }
      if (init.valve) P.valve.set(init.valve);
      this.mass.at(this.prop.m, this.mp);
      this.p.set(init.p); this.q.set(init.q); this.v.set(init.v); this.w.set(init.w);
      if (init.kick) { this.kickDw.set(init.kick.spin_w.map(x => x / this.nKick)); this.kickState = 0; }
      this.leftDeck = init.leftDeck ?? true;
      this.launch = { x: init.p[0], z: init.p[2] };
    } else throw new Error(`unknown init type ${init.type}`);
    this.mgas = this.prop.m;
    this.init = init;
    this.resetWheels();
    this.setDrive(0, 0);
    // unknown spin: launch / rest roll with the ground; an explicit state is taken as after a flight
    if (this.spinOn) {
      if (init.type === 'state' && init.wheel_spin) this.wheels.forEach((wh, j) => { wh.spin = +init.wheel_spin[j]; });
      else this.matchSpin(init.type === 'state' ? this.spinAir : 1);
    }
    this.boundary();
    return this;
  }

  /** Clear the per-episode wheel state and diagnostics (spin is set by the caller). */
  resetWheels() {
    for (const wh of this.wheels) { wh.spin = 0; wh.N = 0; wh.lat = 0; wh.long = 0; wh.pN = 0; wh.pL = 0; wh.pI = 0; }
    this.wheelOverload = null; this.kerb = null;
  }

  /** Drive state (5.13): brake pressure beta, steer angle delta (rad); applied commands cleared. */
  setDrive(beta, delta) {
    this.ud[0] = 0; this.ud[1] = 0; this.beta = beta;
    this.steerTo(delta);
  }

  /**
   * Front road-wheel angle d (rad): the front wheels' static toe heading turned by d (cos / sin through the portable
   * fmath, both languages identical); exactly the static toe when d = 0 (no composition evaluated).
   */
  steerTo(d) {
    this.delta = d; this.steerOn = d !== 0;
    const W = this.wheels;
    if (d === 0) { for (let j = 0; j < 4; j++) { const wh = W[j]; wh.c0 = wh.tc; wh.s0 = wh.ts; } return; }
    const cd = cos(d), sd = sin(d);
    for (let j = 0; j < 4; j++) {
      const wh = W[j];
      if (wh.axle === 0) { wh.c0 = wh.tc * cd - wh.ts * sd; wh.s0 = wh.ts * cd + wh.tc * sd; }
    }
  }

  /** Wheel spin = ratio x rolling-matched (omega r = forward speed of the tyre point) at the current state. */
  matchSpin(ratio) {
    const R = quatToMat(this.q, this.R), v = this.v, w = this.w, c = this.mp.com;
    const ox = R[0] * w[0] + R[1] * w[1] + R[2] * w[2], oy = R[3] * w[0] + R[4] * w[1] + R[5] * w[2], oz = R[6] * w[0] + R[7] * w[1] + R[8] * w[2];
    const ax = R[0], ay = R[3], az = R[6], hh = Math.sqrt(ax * ax + az * az);
    if (!(hh > this.hmin)) { for (const wh of this.wheels) wh.spin = 0; return; }
    const ih = 1 / hh, dlx = ay * ax * ih, dly = -hh, dlz = ay * az * ih;
    for (let j = 0; j < 4; j++) {
      const wh = this.wheels[j];
      const bx = wh.c[0] - c[0], by = wh.c[1] - c[1], bz = wh.c[2] - c[2];
      const rx = (R[0] * bx + R[1] * by + R[2] * bz) + wh.r * dlx;
      const ry = (R[3] * bx + R[4] * by + R[5] * bz) + wh.r * dly;
      const rz = (R[6] * bx + R[7] * by + R[8] * bz) + wh.r * dlz;
      const vx = v[0] + (oy * rz - oz * ry), vz = v[2] + (ox * ry - oy * rx);
      let vt = (vx * az - vz * ax) * ih;
      if (this.toeOn || this.steerOn) { const vl = (vx * ax + vz * az) * ih; vt = wh.c0 * vt - wh.s0 * vl; } // along the toed / steered heading
      wh.spin = ratio * (-vt / wh.r);
    }
  }

  /** Static suspension equilibrium on the horizontal support y = ys (Newton, spec/README.md section 5.7). */
  settle(xc, zc, ys, qh) {
    const { iterations, fd_step: h } = this.placement, M = this.mp.M, c = this.mp.com, Wg = M * this.g;
    const bumpX = this.bumpX;
    const q = new Float64Array(4), qt = new Float64Array(4), R = new Float64Array(9);
    const res = (u, out) => {
      qt[0] = u[1]; qt[1] = 0; qt[2] = u[2]; qt[3] = 1;
      quatMul(qt, qh, q);
      const n = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
      quatToMat(q, R);
      const ax = R[0], ay = R[3], az = R[6], hh = Math.sqrt(ax * ax + az * az), ih = 1 / hh;
      const dx = ay * ax * ih, dy = -hh, dz = ay * az * ih;
      let F = 0, Tx = 0, Tz = 0;
      for (let j = 0; j < 4; j++) {
        const wh = this.wheels[j];
        const bx = wh.c[0] - c[0], by = wh.c[1] - c[1], bz = wh.c[2] - c[2];
        const rx = (R[0] * bx + R[1] * by + R[2] * bz) + wh.r * dx;
        const ry = (R[3] * bx + R[4] * by + R[5] * bz) + wh.r * dy;
        const rz = (R[6] * bx + R[7] * by + R[8] * bz) + wh.r * dz;
        const d = ys - (u[0] + ry);
        if (d > 0) { // the static part of the step's wheel force (5.5) at rest: spring + bump stop
          let N = wh.ks * d;
          if (d > wh.tr) { const e = d - wh.tr; N += wh.kb * e; if (bumpX) N += wh.kb2 * e * e; }
          F += N; Tx -= rz * N; Tz += rx * N;
        }
      }
      out[0] = F - Wg; out[1] = Tx; out[2] = Tz;
      return out;
    };
    // first guess on the springs alone (2 (k_f + k_r) = 4 k_s exactly for equal axles)
    const u = [ys + c[1] - Wg / (2 * (this.params.ksAxle[0] + this.params.ksAxle[1])), 0, 0], r0 = [0, 0, 0], r1 = [0, 0, 0], J = new Float64Array(9), uk = [0, 0, 0];
    for (let it = 0; it < iterations; it++) {
      res(u, r0);
      for (let k = 0; k < 3; k++) {
        uk[0] = u[0]; uk[1] = u[1]; uk[2] = u[2]; uk[k] += h;
        res(uk, r1);
        J[k] = (r1[0] - r0[0]) / h; J[3 + k] = (r1[1] - r0[1]) / h; J[6 + k] = (r1[2] - r0[2]) / h;
      }
      const d = solve3(J, r0);
      u[0] -= d[0]; u[1] -= d[1]; u[2] -= d[2];
    }
    res(u, r0);
    this.settleResidual = [r0[0], r0[1], r0[2]];
    this.p[0] = xc; this.p[1] = u[0]; this.p[2] = zc;
    this.q.set(q);
  }

  // --------------------------------------------------------------------------------------- control step
  /** Bookkeeping at a control-step boundary (after reset and after every step): edge, airborne, arming, phase. */
  boundary() {
    const t = this.k * this.dt, F = this.stepFlags;
    if (!this.leftDeck && this.p[2] > DECK.halfWidth) {
      this.leftDeck = true; this.events.push({ t, type: 'edge' });
      for (const wh of this.wheels) { wh.pN = 0; wh.pL = 0; } // wheel peaks count from the edge (the deck run is the launcher)
    }
    if (this.k > 0 && this.leftDeck && !this.airborne && !F.any) { this.airborne = true; this.events.push({ t, type: 'airborne' }); }
    if (!this.armed && (this.arm === 'start' || this.leftDeck)) { this.armed = true; this.events.push({ t, type: 'armed' }); }
    // at k = 0 no step has run yet: a rest start stands on the street, an explicit state is taken as airborne
    this.phase = !this.leftDeck ? 'deck' : this.k === 0 ? (this.init.type === 'rest' ? 'ground' : 'air') : !F.any ? 'air' : F.deck ? 'deck' : 'ground';
  }

  /**
   * One control step: u_cmd (NU valve commands, optionally + brake, steer: NA), held for SUBSTEPS integrator steps;
   * forced to 0 while not armed. Drive commands (5.13): brake clamped to [0, 1], steer to [-1, 1], NaN / missing -> 0.
   */
  step(uCmd) {
    const u = this.u, ud = this.ud, n = uCmd.length;
    for (let i = 0; i < NU; i++) { const c = this.armed ? +uCmd[i] : 0; u[i] = c === c ? c : 0; } // NaN -> 0
    let b = 0, st = 0;
    if (this.armed && n > IB) {
      const x = +uCmd[IB]; b = x > 0 ? (x < 1 ? x : 1) : 0; // NaN -> 0
      if (n > IS) { const y = +uCmd[IS]; st = y > -1 ? (y < 1 ? y : 1) : y <= -1 ? -1 : 0; }
    }
    ud[0] = b; ud[1] = st;
    const F = this.stepFlags;
    F.any = F.street = F.deck = F.other = F.hull = F.kerb = false;
    for (let s = 0; s < this.nsub; s++) this.substep(u);
    this.boundary();
    return this;
  }

  // ------------------------------------------------------------------------------------- integrator step
  substep(u) {
    const p = this.p, q = this.q, v = this.v, w = this.w, R = this.R, mp = this.mp, dt = this.dt;
    const t = this.k * dt;
    quatToMat(q, R);
    // 1. kerb kick: world angular velocity grows linearly for nKick steps once the COM passes the trigger line
    if (this.kickState === 0 && p[2] > DECK.halfWidth - KICK.trigger) {
      this.kickState = 1; this.kickLeft = this.nKick; this.events.push({ t, type: 'kick' });
    }
    if (this.kickLeft > 0) {
      const dx = this.kickDw[0], dy = this.kickDw[1], dz = this.kickDw[2];
      w[0] += R[0] * dx + R[3] * dy + R[6] * dz;
      w[1] += R[1] * dx + R[4] * dy + R[7] * dz;
      w[2] += R[2] * dx + R[5] * dy + R[8] * dz;
      if (--this.kickLeft === 0) this.kickState = 2;
    }
    // 2. propulsion: valves, then flows/thrust from the tank state at the step start, then blowdown
    const pcPrev = this.prop.pc;
    const T = this.prop.step(u, dt);
    if (this.prop.pc < PROPULSION.regulator.p_set && pcPrev >= PROPULSION.regulator.p_set && !this.dropout) {
      this.dropout = true; this.events.push({ t, type: 'regulator_dropout' });
    }
    // 2b. drive (5.13): brake pressure lag towards the command, steer angle rate-limited towards s * lock; nothing is
    // evaluated while command and state are 0 (zero drive = the old physics, and the planner's rollouts stay cheap)
    {
      const bc = this.ud[0], sc = this.ud[1];
      if (bc !== 0 || this.beta !== 0) { const e = bc - this.beta; this.beta = e < BETA_SNAP && e > -BETA_SNAP ? bc : this.beta + e * this.kBrake; }
      if (sc !== 0 || this.delta !== 0) {
        const e = sc * this.steerMax - this.delta, r = this.steerStep;
        const d = this.delta + (e > r ? r : e < -r ? -r : e);
        if (d !== this.delta) this.steerTo(d);
      }
    }
    // 3. forces at the state of the step start
    const c = mp.com, M = mp.M;
    const ox = R[0] * w[0] + R[1] * w[1] + R[2] * w[2], oy = R[3] * w[0] + R[4] * w[1] + R[5] * w[2], oz = R[6] * w[0] + R[7] * w[1] + R[8] * w[2];
    let Fx = 0, Fy = 0, Fz = 0, Tx = 0, Ty = 0, Tz = 0;
    const flags = this.stepFlags;
    let street = false, any = false;
    // broadphase (never changes a result: culled boxes cannot contain any contact point)
    let na = 0;
    const rbB = this.rBroad;
    for (let b = 0; b < NB; b++) {
      const B = BOXES[b];
      if (Math.abs(p[0] - B.c[0]) < B.rx + rbB && Math.abs(p[1] - B.c[1]) < B.e[1] + rbB && Math.abs(p[2] - B.c[2]) < B.rz + rbB) this.active[na++] = b;
    }
    // wheels: massless, suspension along the support normal (+Y), tyre point = lowest point of the wheel disc.
    // Per-wheel (axle-resolved) parameters; the v2 force laws (slip tyre, spin, toe / steer, brake) live in tyre(),
    // off the v1 path. Diagnostics only touch loaded wheels (an unloaded wheel adds exact zeros to peaks and impulse).
    const ax = R[0], ay = R[3], az = R[6], hh = Math.sqrt(ax * ax + az * az);
    const wok = hh > this.hmin, ih = 1 / hh, dlx = ay * ax * ih, dly = -hh, dlz = ay * az * ih, vreg = this.vreg;
    const bumpX = this.bumpX, v2 = this.slip || this.spinOn || this.toeOn || this.steerOn, td0 = this.touchdown !== null;
    const brake = this.spinOn && this.beta !== 0; // the brake acts on the wheel spin (no-op without wheel_spin)
    let wmask = 0;
    for (let j = 0; j < 4; j++) {
      const wh = this.wheels[j];
      if (wok) {
        const bx = wh.c[0] - c[0], by = wh.c[1] - c[1], bz = wh.c[2] - c[2];
        const rx = (R[0] * bx + R[1] * by + R[2] * bz) + wh.r * dlx;
        const ry = (R[3] * bx + R[4] * by + R[5] * bz) + wh.r * dly;
        const rz = (R[6] * bx + R[7] * by + R[8] * bz) + wh.r * dlz;
        const xx = p[0] + rx, xy = p[1] + ry, xz = p[2] + rz;
        // support: deepest of the ground plane and the box tops containing the point (at most one radius deep);
        // kb: the support is a street box (sidewalk / median top: a kerb, 5.13), not the ground plane
        let d = GROUND.y - xy, mu = GROUND.mu, kind = KIND.street, kb = false;
        for (let a = 0; a < na; a++) {
          const B = BOXES[this.active[a]];
          const dx = xx - B.c[0], dy = xy - B.c[1], dz = xz - B.c[2];
          const lx = B.cy * dx - B.sy * dz, lz = B.sy * dx + B.cy * dz;
          if (Math.abs(lx) < B.e[0] && Math.abs(dy) < B.e[1] && Math.abs(lz) < B.e[2]) {
            const db = B.e[1] - dy;
            if (db <= wh.r && db > d) { d = db; mu = B.mu; kind = B.kind; kb = kind === KIND.street; }
          }
        }
        if (d > 0) {
          wmask |= 1 << j; any = true;
          if (kind === KIND.street) street = true; else if (kind === KIND.deck) flags.deck = true; else flags.other = true;
          const vx = v[0] + (oy * rz - oz * ry), vy = v[1] + (oz * rx - ox * rz), vz = v[2] + (ox * ry - oy * rx);
          let N = wh.ks * d - wh.cs * vy;
          if (d > wh.tr) { const e = d - wh.tr; N += wh.kb * e; if (bumpX) N += wh.kb2 * e * e - wh.bc * vy; }
          if (N > 0) {
            if (kb) { // a loaded wheel on a kerb: step flag, event once per episode off the deck (before the overload)
              flags.kerb = true;
              if (this.leftDeck && !this.kerb) { this.kerb = { t, wheel: j }; this.events.push({ t, type: 'kerb', wheel: j }); }
            }
            const vt = (vx * az - vz * ax) * ih, vl = (vx * ax + vz * az) * ih;
            const me = wh.mu * mu; // tyre x surface friction
            let ft, fl;
            if (v2) { ft = this.tyre(wh, N, vt, vl, me); fl = this.fLat; } // also writes wh.lat / wh.long (tyre axes)
            else {
              ft = -(wh.crr * N) * vt / Math.max(Math.abs(vt), vreg);
              fl = -(me * N) * vl / Math.max(Math.abs(vl), vreg);
              wh.lat = fl; wh.long = ft;
            }
            const fx = (ft * az + fl * ax) * ih, fy = N, fz = (fl * az - ft * ax) * ih;
            Fx += fx; Fy += fy; Fz += fz;
            Tx += ry * fz - rz * fy; Ty += rz * fx - rx * fz; Tz += rx * fy - ry * fx;
            // load diagnostics (5.11, tyre axes): last substep, peaks, lateral impulse from the step after the
            // touchdown, and the overload at the first exceedance off the deck (wheel order, then normal / lateral / impulse)
            const al = Math.abs(wh.lat);
            wh.N = N;
            if (N > wh.pN) wh.pN = N;
            if (al > wh.pL) wh.pL = al;
            if (td0) wh.pI += al * dt;
            if (this.failOn && this.leftDeck && !this.wheelOverload) {
              const qi = N > this.failN ? 0 : al > this.failL ? 1 : wh.pI > this.failI ? 2 : -1;
              if (qi >= 0) { this.wheelOverload = { t, wheel: j, quantity: OVERLOAD[qi] }; this.events.push({ t, type: 'wheel_overload', wheel: j, quantity: OVERLOAD[qi] }); }
            }
            continue;
          }
        }
      }
      wh.N = 0; wh.lat = 0; wh.long = 0;
      if (this.spinOn) {
        wh.spin *= this.spinKeep; // unloaded wheel keeps its spin (bearing decay)
        if (brake) { // ... and is braked towards 0 by at most T dt / I (Coulomb: never reversed)
          const T = this.beta * wh.Tb;
          if (T > 0) { const dw = T * dt / wh.I, om = wh.spin; wh.spin = om > dw ? om - dw : om < -dw ? om + dw : 0; }
        }
      }
    }
    this.wheelMask = wmask;
    // hull points: penalty + regularised Coulomb against the ground and every box they are inside of (per-point k/c/mu)
    const hull = this.hull, kh = this.kh, ch = this.ch, muh = this.hullMu;
    for (let j = 0; j < hull.length; j++) {
      const x = hull[j];
      const bx = x[0] - c[0], by = x[1] - c[1], bz = x[2] - c[2];
      const rx = R[0] * bx + R[1] * by + R[2] * bz, ry = R[3] * bx + R[4] * by + R[5] * bz, rz = R[6] * bx + R[7] * by + R[8] * bz;
      const xx = p[0] + rx, xy = p[1] + ry, xz = p[2] + rz;
      const vx = v[0] + (oy * rz - oz * ry), vy = v[1] + (oz * rx - ox * rz), vz = v[2] + (ox * ry - oy * rx);
      let fx = 0, fy = 0, fz = 0, hit = false;
      const d0 = GROUND.y - xy;
      if (d0 > 0) {
        hit = true; any = true; street = true;
        const N = kh[j] * d0 - ch[j] * vy;
        if (N > 0) {
          const vt = Math.sqrt(vx * vx + vz * vz), fr = (muh[j] * GROUND.mu * N) / Math.max(vt, this.vreg);
          fx += -(fr * vx); fy += N; fz += -(fr * vz);
        }
      }
      for (let a = 0; a < na; a++) {
        const B = BOXES[this.active[a]];
        const dx = xx - B.c[0], dy = xy - B.c[1], dz = xz - B.c[2];
        const lx = B.cy * dx - B.sy * dz, lz = B.sy * dx + B.cy * dz;
        const alx = Math.abs(lx), aly = Math.abs(dy), alz = Math.abs(lz);
        if (!(alx < B.e[0] && aly < B.e[1] && alz < B.e[2])) continue;
        const px = B.e[0] - alx, py = B.e[1] - aly, pz = B.e[2] - alz;
        let d, nx, ny, nz;
        if (py <= px && py <= pz) { d = py; nx = 0; ny = dy >= 0 ? 1 : -1; nz = 0; }
        else if (px <= pz) { const sx = lx >= 0 ? 1 : -1; d = px; nx = B.cy * sx; ny = 0; nz = -B.sy * sx; }
        else { const sz = lz >= 0 ? 1 : -1; d = pz; nx = B.sy * sz; ny = 0; nz = B.cy * sz; }
        any = true;
        if (B.kind === KIND.deck) flags.deck = true;
        else { hit = true; if (B.kind === KIND.street) street = true; else flags.other = true; }
        const vn = vx * nx + vy * ny + vz * nz, N = kh[j] * d - ch[j] * vn;
        if (!(N > 0)) continue;
        const tx = vx - vn * nx, ty = vy - vn * ny, tz = vz - vn * nz;
        const fr = (muh[j] * B.mu * N) / Math.max(Math.sqrt(tx * tx + ty * ty + tz * tz), this.vreg);
        fx += N * nx - fr * tx; fy += N * ny - fr * ty; fz += N * nz - fr * tz;
      }
      if (hit) {
        flags.hull = true;
        if (!this.hullStrike) { this.hullStrike = { t, point: j }; this.events.push({ t, type: 'hull_strike', point: j }); }
      }
      Fx += fx; Fy += fy; Fz += fz;
      Tx += ry * fz - rz * fy; Ty += rz * fx - rx * fz; Tz += rx * fy - ry * fx;
    }
    if (any) flags.any = true;
    if (street) {
      flags.street = true;
      if (this.airborne && !this.touchdown) {
        this.touchdown = { t, p: Array.from(p), q: Array.from(q), v: Array.from(v), w: Array.from(w), wheels: wmask };
        this.events.push({ t, type: 'touchdown', wheels: wmask });
      }
    }
    // thrust (body frame) at the nozzle exits about the current COM, then quadratic body-axis drag at the COM
    let Bx = 0, By = 0, Bz = 0, Ux = 0, Uy = 0, Uz = 0;
    const nd = this.ndir, ne = this.nexit;
    for (let i = 0; i < NU; i++) {
      const f = T[i], gx = f * nd[i * 3], gy = f * nd[i * 3 + 1], gz = f * nd[i * 3 + 2];
      const rx = ne[i * 3] - c[0], ry = ne[i * 3 + 1] - c[1], rz = ne[i * 3 + 2] - c[2];
      Bx += gx; By += gy; Bz += gz;
      Ux += ry * gz - rz * gy; Uy += rz * gx - rx * gz; Uz += rx * gy - ry * gx;
    }
    const vbx = R[0] * v[0] + R[3] * v[1] + R[6] * v[2], vby = R[1] * v[0] + R[4] * v[1] + R[7] * v[2], vbz = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
    const ka = this.ka;
    Bx -= ka[0] * Math.abs(vbx) * vbx; By -= ka[1] * Math.abs(vby) * vby; Bz -= ka[2] * Math.abs(vbz) * vbz;
    Fx += R[0] * Bx + R[1] * By + R[2] * Bz; Fy += R[3] * Bx + R[4] * By + R[5] * Bz; Fz += R[6] * Bx + R[7] * By + R[8] * Bz;
    const tx = Ux + (R[0] * Tx + R[3] * Ty + R[6] * Tz), ty = Uy + (R[1] * Tx + R[4] * Ty + R[7] * Tz), tz = Uz + (R[2] * Tx + R[5] * Ty + R[8] * Tz);
    const sf = Math.sqrt(Fx * Fx + Fy * Fy + Fz * Fz) / M;
    if (sf > this.peakSf) this.peakSf = sf;
    // 4. integrate: semi-implicit Euler (v then p), midpoint rule for the Euler equations, quaternion update
    v[0] += dt * (Fx / M); v[1] += dt * (Fy / M - this.g); v[2] += dt * (Fz / M);
    const I = mp.I, Ii = mp.Iinv;
    let wx = w[0], wy = w[1], wz = w[2];
    for (let pass = 0; pass < 2; pass++) {
      const Lx = I[0] * wx + I[3] * wy + I[4] * wz, Ly = I[3] * wx + I[1] * wy + I[5] * wz, Lz = I[4] * wx + I[5] * wy + I[2] * wz;
      const gx = tx - (wy * Lz - wz * Ly), gy = ty - (wz * Lx - wx * Lz), gz = tz - (wx * Ly - wy * Lx);
      const ex = Ii[0] * gx + Ii[3] * gy + Ii[4] * gz, ey = Ii[3] * gx + Ii[1] * gy + Ii[5] * gz, ez = Ii[4] * gx + Ii[5] * gy + Ii[2] * gz;
      const hdt = pass === 0 ? 0.5 * dt : dt;
      wx = w[0] + hdt * ex; wy = w[1] + hdt * ey; wz = w[2] + hdt * ez;
    }
    w[0] = wx; w[1] = wy; w[2] = wz;
    p[0] += dt * v[0]; p[1] += dt * v[1]; p[2] += dt * v[2];
    {
      const qx = q[0], qy = q[1], qz = q[2], qw = q[3], h = 0.5 * dt;
      const nx = qx + h * (qw * wx + qy * wz - qz * wy);
      const ny = qy + h * (qw * wy + qz * wx - qx * wz);
      const nz = qz + h * (qw * wz + qx * wy - qy * wx);
      const nw = qw + h * (-qx * wx - qy * wy - qz * wz);
      const n = Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
      q[0] = nx / n; q[1] = ny / n; q[2] = nz / n; q[3] = nw / n;
    }
    // 5. mass properties of the remaining gas; the COM moves inside the body (body frame stays put)
    if (this.prop.m !== this.mgas) {
      const m2 = this.mass.at(this.prop.m, this.mp2), R2 = this.R2;
      quatToMat(q, R2);
      const dcx = m2.com[0] - c[0], dcy = m2.com[1] - c[1], dcz = m2.com[2] - c[2];
      p[0] += R2[0] * dcx + R2[1] * dcy + R2[2] * dcz; p[1] += R2[3] * dcx + R2[4] * dcy + R2[5] * dcz; p[2] += R2[6] * dcx + R2[7] * dcy + R2[8] * dcz;
      const ex = wy * dcz - wz * dcy, ey = wz * dcx - wx * dcz, ez = wx * dcy - wy * dcx;
      v[0] += R2[0] * ex + R2[1] * ey + R2[2] * ez; v[1] += R2[3] * ex + R2[4] * ey + R2[5] * ez; v[2] += R2[6] * ex + R2[7] * ey + R2[8] * ez;
      this.mp2 = mp; this.mp = m2;
      this.mgas = this.prop.m;
    }
    this.k++;
  }

  /**
   * v2 tyre forces of a loaded wheel (spec/README.md 5.11, 5.13): v_t, v_l (body axes) are turned into the tyre axes by
   * the toe (+ steer on the front wheels), the force laws run there, and the force goes back to body axes: returns
   * f_t, leaves f_l in this.fLat. Writes the tyre-axis forces to wh.lat / wh.long (diagnostics; wh.lat is also the
   * compliance input of the next step) and updates the wheel spin (brake included). Only + - * / sqrt.
   */
  tyre(wh, N, vt, vl, me) {
    let c = 1, s = 0;
    const rot = this.toeOn || this.steerOn;
    if (rot) {
      // heading about the vertical (+ = front edge to the car's right): static toe (+ steer angle, front) cos / sin,
      // composed with the compliance steer tan(delta_c) = -kappa F_lat(previous step) as an exact rotation (c^2 + s^2 = 1)
      c = wh.c0; s = wh.s0;
      if (this.complyOn) {
        const dc = -wh.kap * wh.lat, cc = 1 / Math.sqrt(1 + dc * dc), sc = dc * cc, c1 = c * cc - s * sc;
        s = s * cc + c * sc; c = c1;
      }
      const vt0 = vt;
      vt = c * vt - s * vl; vl = c * vl + s * vt0;
    }
    let sy = 0, D2 = 1, ft, fl;
    // slip tyre: sigma_y = c_alpha s / mu_eff, |F| = mu_eff N sigma / sqrt(1 + sigma^2) (one friction circle with sigma_x)
    if (this.slip) { sy = wh.ca * (vl / Math.max(Math.abs(vt), wh.vsm)) / me; D2 = 1 + sy * sy; }
    if (this.spinOn) {
      // longitudinal slip v_t + omega r; secant stiffness k frozen at the step start, then omega and the force
      // implicitly together (the slip contracts, never overshoots, however stiff N r^2 / I gets). Slip-ratio
      // saturation kappa: saturation velocity max(v_long_sat, kappa |v_t|) (skipped for kappa = 0, 5.11)
      const r = wh.r, om = wh.spin;
      let vls = wh.vls;
      if (wh.kss !== 0) { const a = wh.kss * (vt < 0 ? -vt : vt); vls = a > vls ? a : vls; }
      const sx = (vt + om * r) / vls;
      D2 += sx * sx;
      const D = Math.sqrt(D2), k = (me * N) / (vls * D), A = wh.A;
      const wr = om * r, tau = -(wh.crr * N) * r * wr / Math.max(Math.abs(wr), this.vreg); // rolling resistance torque
      const den = A + k * r * r;
      let om2 = (A * om - k * r * vt + tau) / den;
      if (this.beta !== 0) {
        // brake (5.13): implicit Coulomb torque of at most T on the wheel (spin change <= T / den), never reversing
        // the spin and never pulling it below the ABS target (1 - abs_slip) x rolling-matched; the tyre force below
        // is then exactly what decelerated the wheel
        const T = this.beta * wh.Tb;
        if (T > 0) {
          const dmax = T / den;
          let tgt = this.absKeep * (-vt / r);
          if (om2 > 0) { tgt = tgt > 0 ? tgt : 0; if (om2 > tgt) om2 = om2 - dmax > tgt ? om2 - dmax : tgt; }
          else if (om2 < 0) { tgt = tgt < 0 ? tgt : 0; if (om2 < tgt) om2 = om2 + dmax < tgt ? om2 + dmax : tgt; }
        }
      }
      ft = -k * (vt + r * om2);
      wh.spin = om2;
      fl = this.slip ? -(me * N) * sy / D : -(me * N) * vl / Math.max(Math.abs(vl), this.vreg);
      if (wh.sld !== 0) {
        // sliding blend (tyre.slide, 5.11): towards isotropic Coulomb along the step-start slip velocity (v_s0, v_l),
        // weight xi = slide sigma_x^2 / (1 + sigma_x^2) (skipped for slide = 0)
        const vs0 = vt + om * r, ss = sx * sx, xi = wh.sld * ss / (1 + ss);
        const fs = -(me * N) * vl / Math.max(Math.sqrt(vs0 * vs0 + vl * vl), this.vreg);
        fl = (1 - xi) * fl + xi * fs;
      }
    } else {
      ft = -(wh.crr * N) * vt / Math.max(Math.abs(vt), this.vreg);
      fl = this.slip ? -(me * N) * sy / Math.sqrt(D2) : -(me * N) * vl / Math.max(Math.abs(vl), this.vreg);
    }
    wh.lat = fl; wh.long = ft;
    if (rot) { this.fLat = c * fl - s * ft; return c * ft + s * fl; }
    this.fLat = fl;
    return ft;
  }

  /**
   * Plan from a state estimate (mirror of coldgas.twin.Twin.load_state): restores what a FlightState carries; tank
   * p/T follow the blowdown polytrope from m, mass properties come from the model, the kick is over, a vehicle in
   * phase 'ground' counts as touched down (snapshot = this state), no hull strike yet. Wheel spin (if modelled):
   * s.wheel_spin if the state carries it (a twin's own state does), else spin_ratio_air x rolling-matched in the
   * air and rolling-matched on the ground / deck (along the current, possibly steered heading); wheel peaks, the
   * overload and the kerb hit start afresh. Drive (5.13): s.drive {brake, steer} if present, else 0.
   */
  loadState(s) {
    const P = this.prop, reg = PROPULSION.regulator;
    this.p.set(s.p); this.q.set(s.q); this.v.set(s.v); this.w.set(s.w);
    const m = +s.tank.m, rho = m / P.V;
    P.m = m;
    P.p = P.K * rho ** P.spec.tank.polytropic_n;
    P.T = rho > 0 ? P.p / (rho * P.R) : +s.tank.T;
    P.valve.set(s.valve); P.thrust.fill(0); P.mdot.fill(0);
    const pc = Math.min(reg.p_set, P.p - reg.dp_min);
    P.pc = pc > 2 * P.pa ? pc : 0;
    this.mass.at(m, this.mp);
    this.k = pyRound(s.t / this.dt);
    this.kickState = 2; this.kickLeft = 0;
    const ph = s.phase;
    this.leftDeck = true; this.armed = !!s.armed; this.airborne = ph !== 'deck';
    this.touchdown = ph === 'ground' ? { t: this.k * this.dt, p: Array.from(this.p), q: Array.from(this.q), v: Array.from(this.v), w: Array.from(this.w), wheels: 0 } : null;
    this.hullStrike = null; this.peakSf = 0; this.dropout = P.pc < reg.p_set;
    this.stepFlags = { any: false, street: false, deck: false, other: false, hull: false, kerb: false };
    this.init ||= { type: 'state' }; // a fresh twin (never reset) behaves like Python's default lane at k = 0
    this.phase = ph; this.mgas = P.m; this.events = [];
    this.resetWheels();
    const dv = s.drive; // brake pressure / steer angle of the state (5.13), 0 if it does not carry them
    this.setDrive(dv ? +dv.brake : 0, dv ? +dv.steer : 0);
    if (this.spinOn) {
      if (s.wheel_spin) this.wheels.forEach((wh, j) => { wh.spin = +s.wheel_spin[j]; });
      else this.matchSpin(ph === 'air' ? this.spinAir : 1);
    }
    return this;
  }

  // ---------------------------------------------------------------------------------------------- views
  get t() { return this.k * this.dt; }

  /**
   * FlightState snapshot (copies; spec/README.md section 7) incl. drive {brake, steer} (5.13); twin extras pc, thrust,
   * step (+ wheel_spin if modelled).
   */
  get state() {
    const P = this.prop;
    const s = {
      t: this.k * this.dt, p: Float64Array.from(this.p), q: Float64Array.from(this.q), v: Float64Array.from(this.v), w: Float64Array.from(this.w),
      comB: Float64Array.from(this.mp.com), tank: { m: P.m, T: P.T, p: P.p }, valve: Float64Array.from(P.valve),
      armed: this.armed, phase: this.phase, drive: this.drive, pc: P.pc, thrust: Float64Array.from(P.thrust), step: this.k,
    };
    if (this.spinOn) s.wheel_spin = this.wheelSpin;
    return s;
  }

  /** Brake pressure (0..1) and front road-wheel steer angle (rad, + = front edge to the right) now (5.13). */
  get drive() { return { brake: this.beta, steer: this.delta }; }
  /** Wheel spin (rad/s, + = rolling forward; zeros unless wheel_spin is enabled), copy. */
  get wheelSpin() { return Float64Array.from(this.wheels, wh => wh.spin); }
  /** Normal, lateral and longitudinal (along -forward) tyre force of each wheel in the last integrator step, copies. */
  get wheelLoad() { const W = this.wheels; return { N: Float64Array.from(W, w => w.N), lat: Float64Array.from(W, w => w.lat), long: Float64Array.from(W, w => w.long) }; }
  /** Per-episode peaks (from the deck edge on) of N and |F_lat|, lateral impulse since touchdown (N s), copies. */
  get wheelPeak() { const W = this.wheels; return { N: Float64Array.from(W, w => w.pN), lat: Float64Array.from(W, w => w.pL), latImpulse: Float64Array.from(W, w => w.pI) }; }

  /** [p, q, v, w] (STATE layout). */
  flat(out = new Float64Array(13)) {
    out.set(this.p, 0); out.set(this.q, 3); out.set(this.v, 7); out.set(this.w, 10);
    return out;
  }
}

/** Python's round() (half to even), for integers derived from settings that must match coldgas exactly. */
export function pyRound(x) {
  const r = Math.round(x);
  return r - x === 0.5 && r % 2 !== 0 ? r - 1 : r;
}

/** Solve the general 3x3 system J d = r (row-major J) by cofactors. */
export function solve3(J, r) {
  const a = J[0], b = J[1], c = J[2], d = J[3], e = J[4], f = J[5], g = J[6], h = J[7], i = J[8];
  const A = e * i - f * h, B = f * g - d * i, C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const x0 = (A * r[0] + (c * h - b * i) * r[1] + (b * f - c * e) * r[2]) / det;
  const x1 = (B * r[0] + (a * i - c * g) * r[1] + (c * d - a * f) * r[2]) / det;
  const x2 = (C * r[0] + (b * g - a * h) * r[1] + (a * e - b * d) * r[2]) / det;
  return [x0, x1, x2];
}
