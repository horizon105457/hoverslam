// Brakes and steering on the soft body: the car's own hardware of spec/drive.json (the twins' model of the same
// hardware is spec/README.md 5.13), driven by ThrusterRig from the two drive channels after the nozzles in a
// Controller's output (b = brake 0..1, s = steer -1..1). Inert while both commands and states are 0: no arithmetic
// touches the lattice, so every run without drive commands is bit-identical to the solver without this module.
//
//  * Steering. The lattice has no knuckle: each wheel's two axle nodes hang on four control arms (SUSP beams, fore /
//    aft, inboard) and two coil-overs (SPRING) to the body. Turning the wheel by delta about the vertical through its
//    centre is a new set of rest lengths: every arm / spring of a front wheel gets L0 + |A(delta) - M| - |A - M|, with
//    A the rest position of its axle node, M of its body node and A(delta) = C + R_y(delta)(A - C), C the wheel centre.
//    The constraints then hold the wheel at the steered pose (3 distances per axle node fix it). + = front edge to
//    the car's right = R_y(+delta), as the toe sign. delta follows the command at rate_deg_s (per substep).
//  * Brakes. Per wheel, the spin about the axle relative to the hub carrier (wheel angular momentum about the axle /
//    its inertia, minus the body's angular velocity about it) is braked by at most T = beta torque_max per second,
//    Coulomb (never reversed): the wheel nodes get the angular impulse -J about the axle line (pure torque: radial
//    lever arms, zero net force), the body gets +J as a pure couple about the same axis over BRAKE_CLUSTER body nodes
//    around the wheel's suspension mounts (where the caliper reaction goes in). The tyre then has to stop the wheel
//    against the street: the deceleration comes from the solver's tyre-node friction, as in a real car.
//  * ABS. A wheel with a tyre node in contact is never braked below (1 - abs_slip) x its rolling-matched spin (the
//    axle's speed along the wheel heading / radius); a wheel off the ground is braked towards zero spin.
// State is read by ThrusterRig into FlightState.drive {brake, steer} and the log (drive_cmd, drive, wheel_spin).
import { DRIVE } from '../core/spec.js';
import { BEAM } from '../car/carPrep.js';

const BRAKE_CLUSTER = 64; // body nodes taking the caliper reaction of one wheel (~6 kg of lattice: no local dent)
const deg = Math.PI / 180;

export class SoftDrive {
  constructor(car, spec = DRIVE) {
    this.car = car; this.spec = spec;
    const R = car.nodes.rest, B = car.beams, chunk = car.nodes.chunk, body = car.chunks.find(c => c.piece === 'body');
    const tq = spec.brake.torque_max;
    this.dmax = spec.steer.max_deg * deg; this.rate = spec.steer.rate_deg_s * deg; this.tau = spec.brake.tau; this.lam = spec.brake.abs_slip;
    const wheelOf = new Map(car.wheels.map((w, i) => [w.chunk, i]));
    this.wheels = car.wheels.map(wh => {
      const ch = car.chunks[wh.chunk], front = wh.centre[2] > 0;
      return { chunk: wh.chunk, n0: ch.nodeStart, n1: ch.nodeStart + ch.nodeCount, axle: wh.axle, radius: wh.radius, front, Tmax: front ? tq[0] : tq[1], centre: wh.centre, beams: [], mounts: [], cluster: null, spin: 0 };
    });
    // suspension beams of each wheel: arms + coil-overs, axle end first
    for (let b = 0; b < B.count; b++) {
      const k = B.kind[b]; if (k !== BEAM.SUSP && k !== BEAM.SPRING) continue;
      const i = B.i[b], j = B.j[b], ai = car.nodes.chunk[i], wi = wheelOf.get(ai) ?? wheelOf.get(chunk[j]);
      if (wi === undefined) continue;
      const W = this.wheels[wi], ax = wheelOf.has(ai) ? i : j, bn = ax === i ? j : i;
      W.mounts.push(bn);
      const A = [R[ax * 3], R[ax * 3 + 1], R[ax * 3 + 2]], M = [R[bn * 3], R[bn * 3 + 1], R[bn * 3 + 2]];
      W.beams.push({ b, A, M, L0: B.L0[b], d0: Math.hypot(A[0] - M[0], A[1] - M[1], A[2] - M[2]) });
    }
    // caliper reaction cluster: the body nodes nearest (rest) to the centroid of the wheel's mount nodes
    for (const W of this.wheels) {
      const c = [0, 0, 0]; for (const m of W.mounts) for (let a = 0; a < 3; a++) c[a] += R[m * 3 + a] / W.mounts.length;
      const near = [];
      for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n++) near.push([Math.hypot(R[n * 3] - c[0], R[n * 3 + 1] - c[1], R[n * 3 + 2] - c[2]), n]);
      near.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
      W.cluster = Int32Array.from(near.slice(0, BRAKE_CLUSTER), e => e[1]);
    }
    this.cmd = { brake: 0, steer: 0 }; this.brake = 0; this.steer = 0;
    this._steerSet = 0; // steer angle the beam rest lengths currently hold
  }

  /** Commands from the controller (already forced to 0 while unarmed): brake 0..1, steer -1..1 (clamped, NaN -> 0). */
  set(b, s) {
    b = +b || 0; s = +s || 0;
    this.cmd.brake = b < 0 ? 0 : b > 1 ? 1 : b; this.cmd.steer = s < -1 ? -1 : s > 1 ? 1 : s;
  }

  /** Once per substep (ThrusterRig.apply, before the solve). wW: body angular velocity, world (rig estimate). */
  apply(sim, dt, wW) {
    const c = this.cmd;
    // actuator states: brake pressure lag, rate-limited steer
    if (c.brake !== this.brake) { this.brake += (c.brake - this.brake) * (1 - Math.exp(-dt / this.tau)); if (Math.abs(c.brake - this.brake) < 1e-7) this.brake = c.brake; }
    const tgt = c.steer * this.dmax, e = tgt - this.steer, r = this.rate * dt;
    if (e !== 0) this.steer += e > r ? r : e < -r ? -r : e;
    if (this.steer !== this._steerSet) this.steerBeams(sim);
    if (this.brake > 0) for (const W of this.wheels) this.brakeWheel(sim, W, dt, wW);
  }

  /** Rest lengths of the front wheels' arms / coil-overs for the current steer angle. */
  steerBeams(sim) {
    const d = this.steer, cd = Math.cos(d), sd = Math.sin(d), L0 = sim.L0;
    for (const W of this.wheels) {
      if (!W.front) continue;
      const C = W.centre;
      for (const s of W.beams) {
        const dx = s.A[0] - C[0], dz = s.A[2] - C[2];
        const x = C[0] + cd * dx + sd * dz, z = C[2] - sd * dx + cd * dz;
        L0[s.b] = s.L0 + Math.hypot(x - s.M[0], s.A[1] - s.M[1], z - s.M[2]) - s.d0;
      }
    }
    this._steerSet = d;
  }

  /** Axle frame of a wheel from its axle nodes: unit axle a, hub c, hub velocity u (null if an axle node is gone). */
  hub(sim, W) {
    const x = sim.x, v = sim.v, w = sim.w, a0 = W.axle[0], a1 = W.axle[1];
    if (!w[a0] || !w[a1] || sim.pieceDetached[sim.nodePiece[a0]]) return null;
    const i = a0 * 3, j = a1 * 3;
    let ax = x[j] - x[i], ay = x[j + 1] - x[i + 1], az = x[j + 2] - x[i + 2];
    const l = Math.hypot(ax, ay, az); ax /= l; ay /= l; az /= l;
    return { a: [ax, ay, az], c: [(x[i] + x[j]) / 2, (x[i + 1] + x[j + 1]) / 2, (x[i + 2] + x[j + 2]) / 2], u: [(v[i] + v[j]) / 2, (v[i + 1] + v[j + 1]) / 2, (v[i + 2] + v[j + 2]) / 2] };
  }

  /** Spin of the wheel about its axle (rad/s, + = rolling forward), inertia about the axle, contact flag. */
  spinOf(sim, W, H) {
    const x = sim.x, v = sim.v, w = sim.w, [ax, ay, az] = H.a, [cx, cy, cz] = H.c, [ux, uy, uz] = H.u;
    let L = 0, I = 0, touch = 0;
    for (let n = W.n0; n < W.n1; n++) {
      if (!w[n]) continue;
      const i = n * 3, m = 1 / w[n];
      let rx = x[i] - cx, ry = x[i + 1] - cy, rz = x[i + 2] - cz;
      const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
      const vx = v[i] - ux, vy = v[i + 1] - uy, vz = v[i + 2] - uz;
      L += m * (ax * (ry * vz - rz * vy) + ay * (rz * vx - rx * vz) + az * (rx * vy - ry * vx));
      I += m * (rx * rx + ry * ry + rz * rz);
      touch |= sim.contact[n];
    }
    return { om: I > 0 ? L / I : 0, I, touch };
  }

  brakeWheel(sim, W, dt, wW) {
    const H = this.hub(sim, W);
    if (!H) return;
    const { om, I, touch } = this.spinOf(sim, W, H), [ax, ay, az] = H.a;
    if (!(I > 0)) return;
    const ob = wW ? wW[0] * ax + wW[1] * ay + wW[2] * az : 0, rel = om - ob; // spin relative to the hub carrier
    // ABS target: rolling-matched spin from the hub speed along the wheel heading f = a x up (= (-a_z, 0, a_x)/h)
    let tgt = 0;
    if (touch) {
      const h = Math.hypot(ax, az);
      if (h > 0.5) tgt = (1 - this.lam) * ((-az * H.u[0] + ax * H.u[2]) / h / W.radius - ob);
    }
    if (rel > 0) { if (!(tgt > 0)) tgt = 0; } else if (!(tgt < 0)) tgt = 0;
    const need = (rel - tgt) * I, J = this.brake * W.Tmax * dt;
    const Japp = need > 0 ? Math.min(J, need) : need < 0 ? -Math.min(J, -need) : 0; // angular impulse taken off the wheel
    if (!Japp) return;
    // wheel: -Japp about the axle line through the hub (radial arms: no net force)
    const x = sim.x, v = sim.v, w = sim.w, [cx, cy, cz] = H.c, dw = -Japp / I;
    for (let n = W.n0; n < W.n1; n++) {
      if (!w[n]) continue;
      const i = n * 3;
      let rx = x[i] - cx, ry = x[i + 1] - cy, rz = x[i + 2] - cz;
      const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
      v[i] += dw * (ay * rz - az * ry); v[i + 1] += dw * (az * rx - ax * rz); v[i + 2] += dw * (ax * ry - ay * rx);
    }
    // body: +Japp as a couple about the axle direction through the cluster centroid
    const cl = W.cluster;
    let M = 0, gx = 0, gy = 0, gz = 0;
    for (let k = 0; k < cl.length; k++) { const n = cl[k]; if (!w[n]) continue; const m = 1 / w[n], i = n * 3; M += m; gx += m * x[i]; gy += m * x[i + 1]; gz += m * x[i + 2]; }
    if (!(M > 0)) return;
    gx /= M; gy /= M; gz /= M;
    let Ib = 0;
    for (let k = 0; k < cl.length; k++) {
      const n = cl[k]; if (!w[n]) continue; const i = n * 3, m = 1 / w[n];
      let rx = x[i] - gx, ry = x[i + 1] - gy, rz = x[i + 2] - gz;
      const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
      Ib += m * (rx * rx + ry * ry + rz * rz);
    }
    if (!(Ib > 0)) return;
    const db = Japp / Ib;
    for (let k = 0; k < cl.length; k++) {
      const n = cl[k]; if (!w[n]) continue; const i = n * 3;
      let rx = x[i] - gx, ry = x[i + 1] - gy, rz = x[i + 2] - gz;
      const d = rx * ax + ry * ay + rz * az; rx -= d * ax; ry -= d * ay; rz -= d * az;
      v[i] += db * (ay * rz - az * ry); v[i + 1] += db * (az * rx - ax * rz); v[i + 2] += db * (ax * ry - ay * rx);
    }
  }

  /** Spin of every wheel relative to the body (rad/s, + = rolling forward; 0 if gone): logs and analysis. */
  wheelSpins(sim, wW) {
    return this.wheels.map(W => {
      const H = this.hub(sim, W); if (!H) return 0;
      const { om } = this.spinOf(sim, W, H), [ax, ay, az] = H.a;
      return om - (wW ? wW[0] * ax + wW[1] * ay + wW[2] * az : 0);
    });
  }
}
