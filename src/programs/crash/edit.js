// The crash edit: maps film time to simulation time (always 1:1, no slow motion) and drives the camera
// through a shot list built around the run's key moments (leaving the deck, first impact).
import * as THREE from 'three';
import { DECK } from '../../scene/layout.js';
import { Sequence } from '../../cinema/sequence.js';
import { launchMoments } from '../../cinema/moments.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);

/**
 * Every shot runs in real time (1 s of film = 1 s of simulation, no warps): the edit is continuous
 * action with cuts, followed by two real-time replays of the impact from other angles. The wreck shot
 * runs on past the end of the recording (the car is at rest, dust keeps drifting).
 */
export class CrashEdit extends Sequence {
  constructor(player) {
    const m = launchMoments(player);
    super(crashShots(player, m));
    this.player = player;
    Object.assign(this, m); // tEdge, tImp, end, L (landing point), R (rest point)
  }
  describe() { return { ...super.describe(), tImp: this.tImp, tEdge: this.tEdge }; }
}

function crashShots(P, { tImp, tEdge, end, L, R }) {
  const body = t => v3(...P.bodyCentre(t));
  // travel direction after the impact and the street-side normal to it (the deck is on the other side)
  const dir = R.clone().sub(L).setY(0);
  if (dir.length() < 2) { const v = P.meta.launch?.velocity || [-1, 0, 0.3]; dir.set(v[0], 0, v[2]); }
  dir.normalize();
  const side = v3(-dir.z, 0, dir.x);
  if (side.z < 0) side.negate();
  const at = (o, a, b, y) => o.clone().addScaledVector(dir, a).addScaledVector(side, b).setY(y);
  const shots = [];
  const add = (name, t0, t1, cam) => shots.push({ name, t0, t1, cam });
  const tA = -0.9, tB = tEdge + 0.3, tC = tImp + 0.7, tD = tImp + 3.0, tEnd = end + 2.5;
  // 1 establishing: drone view beside the viaduct, the car comes down the elevated road
  add('高架远景', -3.6, tA, (u, t) => {
    const c = body(t);
    return { pos: v3(c.x - 9 - u * 2, DECK.top + 4.5 - u * 1.5, DECK.halfWidth + 9 - u * 2), tgt: c.clone().setY(DECK.top + 0.6), fov: 32 };
  });
  // 2 on the deck: the car swerves through the gap and over the edge
  add('冲出护栏', tA, tB, (u, t) => {
    const e = body(tEdge);
    return { pos: v3(e.x - 11 + u * 1.5, DECK.top + 1.25, DECK.halfWidth - 1.6), tgt: body(t), fov: 42 };
  });
  // 3 the fall and the impact, seen from the street (real time)
  add('坠落 · 撞击', tB, tC, (u, t) => {
    const pos = at(L, 4.5, 10.5, 0.9 + u * 0.2);
    const tgt = body(t).lerp(L.clone().setY(1.2), 0.3);
    return { pos, tgt, fov: 46, shake: t > tImp ? Math.exp(-(t - tImp) * 8) * 0.8 : 0 };
  });
  // 4 tumbling and parts flying, tracked from the street side
  add('翻滚解体', tC, tD, (u, t) => {
    const c = body(t);
    return { pos: at(c, -2.5, 8.5, 2.2), tgt: c.clone().setY(0.6), fov: 40 };
  });
  // 5 wreck: slow crane and orbit over the debris field
  add('残骸', tD, tEnd, (u, t) => {
    const c = body(t).lerp(R, 0.5);
    const a = Math.atan2(side.z, side.x) - 0.5 + u * 0.9, r = 5.5 + u * 2.0;
    return { pos: v3(c.x + Math.cos(a) * r, 1.3 + u * 2.2, c.z + Math.sin(a) * r), tgt: c.clone().setY(0.4), fov: 38 };
  });
  // 6 replay from above, real time
  add('回放 · 俯视', tImp - 0.2, tImp + 2.4, (u, t) => {
    const m = L.clone().lerp(R, 0.35);
    return { pos: at(m, -2 + u * 3, 3.5, 9.5 - u * 1.5), tgt: body(t).lerp(m.setY(0.5), 0.5), fov: 44 };
  });
  // 7 replay from the deck side, low, real time
  add('回放 · 侧面', tImp - 0.6, tImp + 2.6, (u, t) => {
    return { pos: at(L, 2.5 - u * 3, -7.5, 0.4), tgt: body(t).lerp(L.clone().setY(0.6), 0.25), fov: 36 };
  });
  return shots;
}
