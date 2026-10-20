// The flight edit: shot list built around the key moments of a thrust-assisted escape (deck edge, ignition,
// the jets biting, touchdown, rest). Every shot runs in real time (1 s of film = 1 s of simulation, the project's
// no-slow-motion rule); replays re-show the landing from other angles, also in real time. Two cuts: a landing
// (success) and a crash (the controller did not make it). Pure functions of (u, t): realtime = offline film.
import * as THREE from 'three';
import { DECK } from '../../scene/layout.js';
import { Sequence } from '../../cinema/sequence.js';
import { orbit, withShake } from '../../cinema/shots.js';
import { smooth, inOutCubic } from '../../cinema/ease.js';
import { PROPULSION } from '../../core/spec.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const V = a => v3(a[0], a[1], a[2]);

/** Dutch angle that makes a lookAt camera's up follow U (rigidly attached cameras), see cinema/film.js. */
export function attachedRoll(pos, tgt, U) {
  const f = tgt.clone().sub(pos).normalize();
  const r0 = v3(0, 0, 0).crossVectors(f, v3(0, 1, 0)).normalize(), u0 = v3(0, 0, 0).crossVectors(r0, f);
  const up = U.clone().sub(f.clone().multiplyScalar(f.dot(U))).normalize();
  return -Math.atan2(v3(0, 0, 0).crossVectors(u0, up).dot(f), u0.dot(up));
}

export class FlightEdit extends Sequence {
  /**
   * @param log      FlightLog (timeline.js)
   * @param success  landing verdict (picks the cut)
   */
  constructor(log, { success = true } = {}) {
    const m = log.moments();
    super(success ? landingShots(log, m) : crashShots(log, m));
    Object.assign(this, m);
    this.log = log; this.success = success;
  }
  describe() { const { tEdge, tIgn, tBrake, tTouch, tRest, end } = this; return { ...super.describe(), tEdge, tIgn, tBrake, tTouch, tRest, end, success: this.success }; }
}

function common(log, m) {
  const body = t => V(log.p(t));
  const E = body(m.tEdge), L = V(m.L);
  const dir = L.clone().sub(E).setY(0);
  if (dir.length() < 2) { const v = log.v(m.tEdge); dir.set(v[0], 0, v[2]); }
  dir.normalize();
  const side = v3(-dir.z, 0, dir.x); if (side.z < 0) side.negate(); // away from the viaduct
  const at = (o, a, b, y) => o.clone().addScaledVector(dir, a).addScaledVector(side, b).setY(y);
  /** Camera rigidly attached to the car: body-frame position and look point. */
  const attached = (posB, lookB, fov) => (u, t) => {
    const pos = V(log.bodyPoint(t, posB)), tgt = V(log.bodyPoint(t, lookB));
    return { pos, tgt, fov, roll: attachedRoll(pos, tgt, V(log.bodyDir(t, [0, 1, 0]))), aperture: 0.004 };
  };
  return { body, E, L, dir, side, at, attached };
}

// nozzle geometry for the attached cameras
const NOZ = Object.fromEntries(PROPULSION.nozzles.map(z => [z.id, z]));

function landingShots(log, m) {
  const { body, E, L, dir, side, at, attached } = common(log, m);
  const { tEdge, tIgn, tTouch, tRest, end } = m;
  const t1 = tIgn ?? tEdge;
  const shots = [];
  const add = (name, t0, tEnd, cam) => { if (tEnd - t0 > 0.15) shots.push({ name, t0, t1: tEnd, cam }); };
  const tA = tEdge - 0.45, tB = t1 + 0.4, tC = tB + 0.95, tD = Math.max(tC + 0.4, tTouch - 0.75), tF = tTouch + 1.4, tEnd = Math.min(end, tF + 1.7);
  // 1 establishing: drone beside the viaduct, the car comes down the elevated road (pre-roll)
  add('高架远景', -2.6, tA, (u, t) => {
    const c = body(t);
    return { pos: v3(c.x - 9 - u * 2, DECK.top + 4.5 - u * 1.4, DECK.halfWidth + 9.5 - u * 2), tgt: c.clone().setY(DECK.top + 0.6), fov: 32 };
  });
  // 2 through the parapet gap and over the edge: the valves crack open as the deck runs out
  add('冲出 · 点火', tA, tB, (u, t) => {
    const pos = v3(E.x - 15 + u * 1.5, DECK.top + 1.45, DECK.halfWidth - 5.2);
    return { pos, tgt: body(t).add(v3(0, -0.5, 0)), fov: 36 };
  });
  // 3 nozzle cam: rigidly on the car, outside and below the front-left sill: the underbody, the jets and the
  // street far below, the horizon tilting with the car
  const B = NOZ.B_FL.exit;
  add('喷口视角', tB, tC, attached([B[0] - 1.0, B[1] - 0.47, B[2] + 1.2], [B[0] + 0.1, B[1] - 1.05, B[2] - 0.65], 72));
  // 4 from the street, low, looking up at the descending car on its jets
  add('仰望 · 下降', tC, tD, (u, t) => {
    const pos = at(L, 6.5 - u * 0.8, 2.6, 0.3);
    return { pos, tgt: body(t).add(v3(0, -0.5, 0)), fov: 27, aperture: 0.02 };
  });
  // 5 touchdown: slow low orbit around the landing point while the jets flatten into the ground sheet
  const a0 = Math.atan2(side.z, side.x) + 0.9;
  add('触地', tD, tF, orbit({ center: t => body(t).setY(0), radius: u => 7.2 - u * 0.8, a0, a1: a0 - 0.75, height: u => 1.15 + 0.5 * u, tgt: t => body(t).setY(0.75), fov: 40, ease: smooth }));
  // 6 rolling away down the street, jets trimming the heading (chase from behind, high-ish)
  add('驶离', tF, tEnd, (u, t) => {
    const c = body(t), v = log.v(t), d = v3(v[0], 0, v[2]); if (d.length() < 0.5) d.copy(dir); d.normalize();
    const s = v3(-d.z, 0, d.x);
    return { pos: c.clone().addScaledVector(d, -7.5 - 3 * u).addScaledVector(s, 2.6).setY(2.4 + u), tgt: c.clone().setY(0.7), fov: 42 };
  });
  // 7 replay: long lens from the viaduct side, the whole descent in one move (real time)
  const rr0 = Math.max(t1 + 0.2, tTouch - 2.2);
  add('回放 · 长焦', rr0, tTouch + 0.6, (u, t) => {
    const mid = E.clone().lerp(L, 0.62);
    const pos = at(mid, -6, -24, 3.2);
    return { pos, tgt: body(t).add(v3(0, -0.5, 0)), fov: 22, aperture: 0.06 };
  });
  // 8 replay: from the asphalt at the landing spot, wide and low, the jets flattening into the ground sheet
  add('回放 · 低角', tTouch - 1.4, tTouch + 0.7, (u, t) => {
    const pos = at(L, 2.2 - 1.2 * u, -4.4, 0.24);
    return { pos, tgt: body(t).add(v3(0, -0.35, 0)), fov: 50, aperture: 0.01 };
  });
  // 9 settle: wide from across the street as it rolls on (until the recording ends)
  add('落定', Math.max(tTouch + 0.9, tRest - 0.4), Math.min(end, Math.max(tTouch + 0.9, tRest - 0.4) + 2.0), (u, t) => {
    const c = body(t);
    return { pos: at(c, 9 - u * 2.5, 9.5, 1.6), tgt: c.clone().setY(0.6), fov: 34 };
  });
  return shots;
}

function crashShots(log, m) {
  const { body, E, L, at, side } = common(log, m);
  const { tEdge, tIgn, tTouch, end } = m;
  const shots = [];
  const add = (name, t0, tEnd, cam) => { if (tEnd - t0 > 0.15) shots.push({ name, t0, t1: tEnd, cam }); };
  const tA = tEdge - 0.45, tB = Math.max(tEdge + 0.35, (tIgn ?? tEdge) - 0.15), tC = tTouch - 0.25, tD = tTouch + 1.4, tE = Math.min(end, tTouch + 3.6), tEnd = end + 1.5;
  add('高架远景', -2.6, tA, (u, t) => {
    const c = body(t);
    return { pos: v3(c.x - 9 - u * 2, DECK.top + 4.5 - u * 1.4, DECK.halfWidth + 9.5 - u * 2), tgt: c.clone().setY(DECK.top + 0.6), fov: 32 };
  });
  add('冲出护栏', tA, tB, (u, t) => ({ pos: v3(E.x - 10 + u * 1.8, DECK.top + 1.1, DECK.halfWidth - 1.4), tgt: body(t), fov: 44 }));
  // late ignition: chase from the viaduct side while the jets fight the roll
  add('迟到的点火', tB, tC, (u, t) => {
    const c = body(t);
    return { pos: c.clone().addScaledVector(side, -10.5).add(v3(4.5 - 2 * u, 1.6 - 0.8 * u, 0)), tgt: c, fov: 38 };
  });
  add('撞击', tC, tD, withShake((u, t) => ({ pos: at(L, 5.5, 10, 0.95 + 0.2 * u), tgt: body(t).lerp(L.clone().setY(1.0), 0.3), fov: 46 }), { t0: tTouch, amp: 0.9, decay: 7 }));
  add('翻滚', tD, tE, (u, t) => { const c = body(t); return { pos: at(c, -2.5, 8.5, 2.2), tgt: c.clone().setY(0.6), fov: 40 }; });
  add('残骸', tE, tEnd, (u, t) => {
    const c = body(t), a = Math.atan2(side.z, side.x) - 0.5 + u * 0.9, r = 6 + u * 2;
    return { pos: v3(c.x + Math.cos(a) * r, 1.4 + u * 2.0, c.z + Math.sin(a) * r), tgt: c.clone().setY(0.4), fov: 38 };
  });
  add('回放 · 侧面', tTouch - 0.9, tTouch + 1.8, (u, t) => ({ pos: at(L, 2.5 - u * 3, -8, 0.45), tgt: body(t).lerp(L.clone().setY(0.6), 0.25), fov: 36 }));
  add('回放 · 俯视', tTouch - 0.6, tTouch + 2.0, (u, t) => { const c = L.clone(); return { pos: at(c, -2 + 3 * inOutCubic(u), 3.5, 10 - u * 1.5), tgt: body(t).lerp(c.setY(0.5), 0.5), fov: 44 }; });
  return shots;
}
