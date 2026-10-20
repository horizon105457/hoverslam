// Rigid 6-DOF flight twin of the MINI EV with the cold-gas kit (spec/README.md section 5 is the contract).
// Fixed step vehicle.integrator.dt, SUBSTEPS integrator steps per control step. Per integrator step:
//   kick -> propulsion (reference model) -> contact / thrust / aero forces at the state of the step start ->
//   semi-implicit Euler for v, p; midpoint for the Euler equations; first-order quaternion update ->
//   mass properties from the remaining gas with the kinematic COM shift.
// Float64 throughout, no allocation in the step, and every expression is written in the same order as the numpy
// mirror (python/coldgas/twin.py) so both agree bit for bit (tests/parity/). Change both or neither.
import { WORLD, LAUNCH, PROPULSION, VEHICLE, GENERATED } from '../core/spec.js';
import { Propulsion } from './propulsion.js';
import { MassModel } from './massprops.js';
import { MODEL } from './model.js';
import { BOXES, GROUND, KIND, supportLevel } from './world.js';
import { cos, sin } from './fmath.js';
import { NU, SUBSTEPS, DT } from './contract.js';

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

/**
 * Initial conditions (spec/README.md section 6 / 8):
 *  {type:'launch', speed (km/h), angle (deg), spin [pitch, yaw, roll] (rad/s), lead (s)}
 *  {type:'rest', x, z, heading_deg}       COM above (x, z), on its wheels on whatever surface is there
 *  {type:'state', p, q, v, w, gas?, valve?, leftDeck?, kick?: {spin_w: [3]}}  explicit (tests, soft-body hand-off)
 */
export class RigidTwin {
  constructor({ arm = 'start' } = {}) {
    this.arm = arm;
    this.dt = DT; this.nsub = SUBSTEPS; this.g = WORLD.gravity;
    this.prop = new Propulsion(PROPULSION, WORLD.air.pressure);
    this.mass = new MassModel(PROPULSION, GENERATED.massprops);
    const W = VEHICLE.wheels, H = VEHICLE.hull, C = MODEL.contact;
    Object.assign(this, { ks: W.spring_k, cs: W.damper_c, travel: W.travel, kb: W.bump_k, tyreMu: W.tyre_mu, crr: W.crr });
    Object.assign(this, { kh: H.k, ch: H.c, hullMu: H.mu, vreg: C.v_reg, hmin: C.wheel_min_h });
    this.wheels = GENERATED.massprops.wheels.map(w => ({ c: Float64Array.from(w.centre), r: w.radius }));
    this.hull = H.points.map(x => Float64Array.from(x));
    // thrust direction (-exhaust, unit) and exit point per nozzle, body frame
    this.ndir = new Float64Array(NU * 3); this.nexit = new Float64Array(NU * 3);
    PROPULSION.nozzles.forEach((z, i) => {
      const e = z.exhaust, l = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
      this.ndir[i * 3] = -e[0] / l; this.ndir[i * 3 + 1] = -e[1] / l; this.ndir[i * 3 + 2] = -e[2] / l;
      this.nexit.set(z.exit, i * 3);
    });
    const rho = WORLD.air.density;
    this.ka = Float64Array.from(VEHICLE.aero.CdA, A => 0.5 * rho * A);
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
    this.stepFlags = { any: false, street: false, deck: false, other: false, hull: false };
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
    this.boundary();
    return this;
  }

  /** Static suspension equilibrium on the horizontal support y = ys (Newton, spec/README.md section 5.7). */
  settle(xc, zc, ys, qh) {
    const { iterations, fd_step: h } = MODEL.placement, M = this.mp.M, c = this.mp.com, Wg = M * this.g;
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
      for (const wh of this.wheels) {
        const bx = wh.c[0] - c[0], by = wh.c[1] - c[1], bz = wh.c[2] - c[2];
        const rx = (R[0] * bx + R[1] * by + R[2] * bz) + wh.r * dx;
        const ry = (R[3] * bx + R[4] * by + R[5] * bz) + wh.r * dy;
        const rz = (R[6] * bx + R[7] * by + R[8] * bz) + wh.r * dz;
        const d = ys - (u[0] + ry);
        if (d > 0) {
          let N = this.ks * d;
          if (d > this.travel) N += this.kb * (d - this.travel);
          F += N; Tx -= rz * N; Tz += rx * N;
        }
      }
      out[0] = F - Wg; out[1] = Tx; out[2] = Tz;
      return out;
    };
    const u = [ys + c[1] - Wg / (4 * this.ks), 0, 0], r0 = [0, 0, 0], r1 = [0, 0, 0], J = new Float64Array(9), uk = [0, 0, 0];
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
    if (!this.leftDeck && this.p[2] > DECK.halfWidth) { this.leftDeck = true; this.events.push({ t, type: 'edge' }); }
    if (this.k > 0 && this.leftDeck && !this.airborne && !F.any) { this.airborne = true; this.events.push({ t, type: 'airborne' }); }
    if (!this.armed && (this.arm === 'start' || this.leftDeck)) { this.armed = true; this.events.push({ t, type: 'armed' }); }
    // at k = 0 no step has run yet: a rest start stands on the street, an explicit state is taken as airborne
    this.phase = !this.leftDeck ? 'deck' : this.k === 0 ? (this.init.type === 'rest' ? 'ground' : 'air') : !F.any ? 'air' : F.deck ? 'deck' : 'ground';
  }

  /** One control step: u_cmd (NU, held for SUBSTEPS integrator steps; forced to 0 while not armed). */
  step(uCmd) {
    const u = this.u;
    for (let i = 0; i < NU; i++) { const c = this.armed ? +uCmd[i] : 0; u[i] = c === c ? c : 0; } // NaN -> 0
    const F = this.stepFlags;
    F.any = F.street = F.deck = F.other = F.hull = false;
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
    // wheels: massless, suspension along the support normal (+Y), tyre point = lowest point of the wheel disc
    const ax = R[0], ay = R[3], az = R[6], hh = Math.sqrt(ax * ax + az * az);
    let wmask = 0;
    if (hh > this.hmin) {
      const ih = 1 / hh, dlx = ay * ax * ih, dly = -hh, dlz = ay * az * ih;
      for (let j = 0; j < 4; j++) {
        const wh = this.wheels[j];
        const bx = wh.c[0] - c[0], by = wh.c[1] - c[1], bz = wh.c[2] - c[2];
        const rx = (R[0] * bx + R[1] * by + R[2] * bz) + wh.r * dlx;
        const ry = (R[3] * bx + R[4] * by + R[5] * bz) + wh.r * dly;
        const rz = (R[6] * bx + R[7] * by + R[8] * bz) + wh.r * dlz;
        const xx = p[0] + rx, xy = p[1] + ry, xz = p[2] + rz;
        // support: deepest of the ground plane and the box tops containing the point (at most one radius deep)
        let d = GROUND.y - xy, mu = GROUND.mu, kind = KIND.street;
        for (let a = 0; a < na; a++) {
          const B = BOXES[this.active[a]];
          const dx = xx - B.c[0], dy = xy - B.c[1], dz = xz - B.c[2];
          const lx = B.cy * dx - B.sy * dz, lz = B.sy * dx + B.cy * dz;
          if (Math.abs(lx) < B.e[0] && Math.abs(dy) < B.e[1] && Math.abs(lz) < B.e[2]) {
            const db = B.e[1] - dy;
            if (db <= wh.r && db > d) { d = db; mu = B.mu; kind = B.kind; }
          }
        }
        if (!(d > 0)) continue;
        wmask |= 1 << j; any = true;
        if (kind === KIND.street) street = true; else if (kind === KIND.deck) flags.deck = true; else flags.other = true;
        const vx = v[0] + (oy * rz - oz * ry), vy = v[1] + (oz * rx - ox * rz), vz = v[2] + (ox * ry - oy * rx);
        let N = this.ks * d - this.cs * vy;
        if (d > this.travel) N += this.kb * (d - this.travel);
        if (!(N > 0)) continue;
        const vt = (vx * az - vz * ax) * ih, vl = (vx * ax + vz * az) * ih;
        const ft = -(this.crr * N) * vt / Math.max(Math.abs(vt), this.vreg);
        const fl = -(this.tyreMu * mu * N) * vl / Math.max(Math.abs(vl), this.vreg);
        const fx = (ft * az + fl * ax) * ih, fy = N, fz = (fl * az - ft * ax) * ih;
        Fx += fx; Fy += fy; Fz += fz;
        Tx += ry * fz - rz * fy; Ty += rz * fx - rx * fz; Tz += rx * fy - ry * fx;
      }
    }
    this.wheelMask = wmask;
    // hull points: penalty + regularised Coulomb against the ground and every box they are inside of
    const hull = this.hull;
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
        const N = this.kh * d0 - this.ch * vy;
        if (N > 0) {
          const vt = Math.sqrt(vx * vx + vz * vz), fr = (this.hullMu * GROUND.mu * N) / Math.max(vt, this.vreg);
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
        const vn = vx * nx + vy * ny + vz * nz, N = this.kh * d - this.ch * vn;
        if (!(N > 0)) continue;
        const tx = vx - vn * nx, ty = vy - vn * ny, tz = vz - vn * nz;
        const fr = (this.hullMu * B.mu * N) / Math.max(Math.sqrt(tx * tx + ty * ty + tz * tz), this.vreg);
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

  // ---------------------------------------------------------------------------------------------- views
  get t() { return this.k * this.dt; }

  /** FlightState snapshot (copies; spec/README.md section 7). */
  get state() {
    const P = this.prop;
    return {
      t: this.k * this.dt, p: Float64Array.from(this.p), q: Float64Array.from(this.q), v: Float64Array.from(this.v), w: Float64Array.from(this.w),
      comB: Float64Array.from(this.mp.com), tank: { m: P.m, T: P.T, p: P.p }, valve: Float64Array.from(P.valve),
      armed: this.armed, phase: this.phase, pc: P.pc, thrust: Float64Array.from(P.thrust), step: this.k,
    };
  }

  /** [p, q, v, w] (STATE layout). */
  flat(out = new Float64Array(13)) {
    out.set(this.p, 0); out.set(this.q, 3); out.set(this.v, 7); out.set(this.w, 10);
    return out;
  }
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
