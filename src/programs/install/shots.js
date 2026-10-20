// Shot list of the install film. Each shot spans a film-time range [t0, t1] and its `cam` receives film time
// as `t` (1 s film = 1 s animation clock). Wide shots are authored in body coordinates (+ lift); the hero
// shots and the fly-through in the hero module / valve frames, so they follow the spec mount and exhaust.
import * as THREE from 'three';
import { keyed } from './keyed.js';
import { LIFT, liftAt, TL } from './choreo.js';

const V = (x, y, z) => new THREE.Vector3(x, y, z);

export function buildShots(kit) {
  const h = kit.hero, d = h.d, k = d.rt;
  // body -> world at the lift height of film time t
  const B = (x, y, z, t = 99) => V(x, y + liftAt(t), z);
  // hero module frame (installed) -> world; valve frame -> world
  const M = (x, y, z) => h.toBody(V(x, y, z)).add(V(0, LIFT, 0));
  const Vv = (x, y, z) => h.toBody(h.valveToModule(V(x, y, z))).add(V(0, LIFT, 0));
  // bench: the assembled module waits this far behind its mount
  const bench = 1.0;
  const Mb = (x, y, z) => M(x, y + bench, z);
  const hub = Mb(0, d.yPc, 0), lay = Mb(0.2, d.yPc, 0); // exploded layout centre (valve parts fan out along +X)
  const mount = M(0, 0, 0);
  const shots = [];
  const add = (name, en, t0, t1, cam) => shots.push({ name, en, t0, t1, cam });

  add('亮灯', 'LIGHTS ON', 0, 4.7, keyed([
    { t: 0, pos: B(-4.4, 0.40, 4.3, 0), tgt: B(0, 0.55, 0.25, 0), fov: 30, aperture: 0.012 },
    { t: 4.7, pos: B(-3.55, 0.58, 3.35, 0), tgt: B(0.05, 0.6, 0.05, 0), fov: 29, aperture: 0.012 },
  ]));
  add('X 光透视', 'X-RAY SCAN', 4.7, 8.0, keyed([
    { t: 4.7, pos: B(-3.3, 1.45, 3.0, 4.7), tgt: B(0, 0.7, 0.15, 4.7), fov: 32, aperture: 0.008 },
    { t: 6.3, pos: B(-4.05, 2.1, -0.5, 6.3), tgt: B(0, 0.75, -0.25, 6.3), fov: 32, aperture: 0.008 },
    { t: 8.0, pos: B(-1.9, 2.25, -3.9, 8.0), tgt: B(0.25, 0.8, -1.0, 8.0), fov: 32, aperture: 0.008 },
  ]));
  // exploded view and assembly from above: the L-shaped layout lies flat at the bench and the open cut wedge
  // (upper outboard quadrant) faces the lens, so every part shows its section
  add('爆炸视图', 'EXPLODED VIEW', 8.0, 10.0, keyed([
    { t: 8.0, pos: lay.clone().add(V(-0.04, 1.02, -0.34)), tgt: lay.clone().add(V(0.0, 0, 0.02)), fov: 34, aperture: 0.004 },
    { t: 10.0, pos: lay.clone().add(V(0.0, 0.9, -0.30)), tgt: lay.clone().add(V(-0.01, 0, 0.02)), fov: 33, aperture: 0.004 },
  ]));
  add('组装', 'ASSEMBLY', 10.0, TL.insert[0], keyed([
    { t: 10.0, pos: lay.clone().add(V(0.0, 0.9, -0.30)), tgt: lay.clone().add(V(-0.01, 0, 0.02)), fov: 33, aperture: 0.003 },
    { t: 12.0, pos: lay.clone().add(V(-0.04, 0.74, -0.32)), tgt: lay.clone().add(V(-0.08, 0, 0.02)), fov: 31, aperture: 0.003 },
    { t: TL.insert[0], pos: hub.clone().add(V(0.30, 0.42, -0.42)), tgt: hub.clone().add(V(0.05, 0, 0.03)), fov: 28, aperture: 0.003 },
  ]));
  add('就位 · 紧固', 'SEAT & TORQUE', TL.insert[0], TL.fly[0], keyed([
    { t: TL.insert[0], pos: hub.clone().add(V(0.30, 0.42, -0.42)), tgt: hub.clone().add(V(0.05, 0, 0.03)), fov: 28, aperture: 0.003 },
    { t: TL.insert[1], pos: mount.clone().add(V(0.46, 0.06, -0.62)), tgt: mount.clone().add(V(0.0, 0.0, -0.08)), fov: 28, aperture: 0.002 },
    // low, behind and outboard: the flange face against the dark floor, the bank out of focus above
    { t: 15.0, pos: mount.clone().add(V(0.21, -0.10, -0.25)), tgt: mount.clone().add(V(0.0, 0.0, 0.0)), fov: 26, aperture: 0.0012 },
    { t: TL.fly[0], pos: mount.clone().add(V(0.62, 0.12, -0.92)), tgt: mount.clone().add(V(-0.22, -0.28, 0.42)), fov: 32, aperture: 0.003 },
  ]));
  // fly-through: in through the exit cone, throat, plenum, seat orifice, up out of the cutaway past the poppet,
  // along the solenoid section, then out and back to reveal the docked module
  const mm = 0.001;
  const Mm = (x, y, z) => M(x * mm, y * mm, z * mm);
  const Le = d.Le / mm, yT = d.yT / mm, yC = d.yC / mm, yPc = d.yPc / mm, vA = d.vA / mm, vS = (d.vA + d.seatH) / mm, vB = d.vB / mm, vC = d.vC / mm;
  add('穿越 · 喷管内部', 'FLY-THROUGH', TL.fly[0], TL.fly[1], keyed([
    { t: TL.fly[0], pos: mount.clone().add(V(0.62, 0.12, -0.92)), tgt: mount.clone().add(V(-0.22, -0.28, 0.42)), fov: 32, near: 0.02, aperture: 0.003 },
    { t: 17.2, pos: Mm(95, Le + 210, 120), tgt: Mm(15, Le - 30, 10), fov: 34, near: 0.008, aperture: 0.0015 },
    { t: 17.9, pos: Mm(6, Le + 70, 14), tgt: Mm(0, yT, 0), fov: 46, near: 0.002, aperture: 0.0001 },
    { t: 18.5, pos: Mm(0, Le - 8, 2), tgt: Mm(0, yT - 20, 0), fov: 58, near: 0.0007, aperture: 0.00003 },
    { t: 19.15, pos: Mm(0, yT + 14, 0), tgt: Mm(0, yC - 30, 0), fov: 62, near: 0.0005, aperture: 0.00002, roll: 0.15 },
    { t: 19.6, pos: Mm(0, yT - 8, 0), tgt: Mm(4, yPc, 0), fov: 64, near: 0.0005, aperture: 0.00002, roll: 0.25 },
    { t: 20.3, pos: Mm(-2, yC - 4, 10), tgt: Mm(vA, yPc, -2), fov: 62, near: 0.0005, aperture: 0.00002, roll: 0.1 },
    // up out of the plenum through the open quadrant: look down onto the seat, poppet and gallery in section
    { t: 21.1, pos: Mm(vA - 6, yPc - 22, 92), tgt: Mm(vS + 4, yPc, 0), fov: 44, near: 0.001, aperture: 0.00005 },
    { t: 21.9, pos: Mm(vS + 22, yPc - 28, 100), tgt: Mm(vS + 22, yPc, 0), fov: 42, near: 0.001, aperture: 0.00006 },
    { t: 22.75, pos: Mm(vB + 14, yPc - 34, 96), tgt: Mm(vB + 12, yPc, 0), fov: 44, near: 0.0012, aperture: 0.0001 },
    { t: 23.5, pos: Mm(vC + 40, yPc + 60, 150), tgt: Mm(vB - 10, yPc, 0), fov: 42, near: 0.003, aperture: 0.0004 },
    { t: 24.45, pos: Mm(320, 230, 300), tgt: Mm(40, -40, -30), fov: 36, near: 0.01, aperture: 0.002 },
    { t: TL.fly[1], pos: Mm(950, 760, 520), tgt: Mm(-150, -500, -260), fov: 34, near: 0.03, aperture: 0.004 },
  ]));
  add('九口齐装', 'NINE AT ONCE', TL.fly[1], 31.0, keyed([
    { t: TL.fly[1], pos: Mm(950, 760, 520), tgt: Mm(-150, -500, -260), fov: 34, aperture: 0.004 },
    { t: 26.6, pos: B(2.7, 0.08, -3.2), tgt: B(0, 0.3, -0.3), fov: 34, aperture: 0.006 },
    { t: 28.4, pos: B(4.15, 0.25, -0.2), tgt: B(0, 0.38, -0.3), fov: 32, aperture: 0.006 },
    { t: 31.0, pos: B(3.1, 0.95, 3.2), tgt: B(0, 0.45, -0.3), fov: 32, aperture: 0.006 },
  ]));
  add('满压 · 试喷', 'PRESSURISE · TEST FIRE', 31.0, TL.title, keyed([
    { t: 31.0, pos: B(3.0, 0.85, -3.9), tgt: B(0, 0.5, -0.5), fov: 32, aperture: 0.006 },
    { t: 34.5, pos: B(2.7, 0.55, -3.45), tgt: B(0, 0.42, -0.5), fov: 33, aperture: 0.006 },
    { t: TL.title, pos: B(2.5, 0.45, -3.2), tgt: B(0, 0.4, -0.5), fov: 34, aperture: 0.006 },
  ]));
  add('冷气推进 · 10 喷口', 'COLD GAS · 10 THRUSTERS', TL.title, TL.end, keyed([
    { t: TL.title, pos: B(2.5, 0.45, -3.2), tgt: B(0, 0.4, -0.5), fov: 34, aperture: 0.006 },
    { t: TL.end, pos: B(3.9, 1.2, -5.6), tgt: B(0, 0.9, -0.3), fov: 30, aperture: 0.006 },
  ]));
  return shots;
}
