// Reusable camera moves for creative shots. Each factory returns a `cam(u, t)` function for a Sequence shot
// (see ./sequence.js): pure functions of film progress u and simulation time t, no hidden state, so they
// render identically in realtime and offline. Targets may be fixed Vector3s or functions t -> Vector3
// (e.g. t => body(t), the car's centre at sim time t). Programs keep bespoke shots in their own folder and
// promote general-purpose ones here.
import * as THREE from 'three';
import { linear } from './ease.js';

const at = (p, u, t) => (typeof p === 'function' ? p(t, u) : p);
const num = (x, u, t) => (typeof x === 'function' ? x(u, t) : x);

/** Locked-off camera. */
export function lockOff({ pos, tgt, fov = 40, roll = 0 }) {
  return (u, t) => ({ pos: at(pos, u, t).clone(), tgt: at(tgt, u, t).clone(), fov: num(fov, u, t), roll: num(roll, u, t) });
}

/** Straight dolly/truck from `from` to `to`. */
export function dolly({ from, to, tgt, fov = 40, ease = linear, roll = 0 }) {
  return (u, t) => ({ pos: at(from, u, t).clone().lerp(at(to, u, t), ease(u)), tgt: at(tgt, u, t).clone(), fov: num(fov, u, t), roll: num(roll, u, t) });
}

/** Orbit around `center` (angles in radians about +Y, measured from +X towards +Z). */
export function orbit({ center, radius, a0, a1, height, tgt = center, fov = 40, ease = linear, roll = 0 }) {
  return (u, t) => {
    const c = at(center, u, t), e = ease(u), a = a0 + (a1 - a0) * e, r = num(radius, e, t);
    return { pos: new THREE.Vector3(c.x + Math.cos(a) * r, c.y + num(height, e, t), c.z + Math.sin(a) * r), tgt: at(tgt, u, t).clone(), fov: num(fov, u, t), roll: num(roll, u, t) };
  };
}

/** Rigidly attached to a moving target: pos = target(t) + offset(u), looking at target(t) + lookOffset. */
export function track({ target, offset, lookOffset = new THREE.Vector3(), fov = 40, roll = 0 }) {
  return (u, t) => {
    const c = at(target, u, t);
    return { pos: c.clone().add(at(offset, u, t)), tgt: c.clone().add(lookOffset), fov: num(fov, u, t), roll: num(roll, u, t) };
  };
}

/**
 * Fly-through along a smooth path (Catmull-Rom through `points`), looking ahead along the path or at an
 * explicit target. For "into the device and out again" moves: put points through the interior.
 * roll / fov may be functions of u (e.g. a barrel roll while threading a nozzle).
 */
export function fly({ points, lookAhead = 0.03, tgt = null, fov = 50, roll = 0, ease = linear, tension = 0.5, curve = 'catmullrom' }) {
  const path = new THREE.CatmullRomCurve3(points, false, curve, tension);
  return (u, t) => {
    const e = ease(u), pos = path.getPointAt(Math.min(1, e));
    const look = tgt ? at(tgt, u, t).clone() : path.getPointAt(Math.min(1, e + lookAhead));
    if (!tgt && e + lookAhead > 1) look.copy(pos).add(path.getTangentAt(1));
    return { pos, tgt: look, fov: num(fov, u, t), roll: num(roll, u, t) };
  };
}

/** Blend two camera functions over the shot (e.g. a slow push from a wide into a close-up). */
export function blend(camA, camB, ease = linear) {
  return (u, t) => {
    const a = camA(u, t), b = camB(u, t), e = ease(u);
    return { pos: a.pos.clone().lerp(b.pos, e), tgt: a.tgt.clone().lerp(b.tgt, e), fov: a.fov + (b.fov - a.fov) * e, roll: (a.roll || 0) + ((b.roll || 0) - (a.roll || 0)) * e };
  };
}

/** Add an impact shake envelope to any camera: shake = amp * exp(-(t - t0) * decay) after t0. */
export function withShake(cam, { t0, amp = 0.8, decay = 8 }) {
  return (u, t) => ({ ...cam(u, t), shake: t > t0 ? Math.exp(-(t - t0) * decay) * amp : 0 });
}
