// Static world of the flight twins: the ground plane and the oriented boxes of spec/generated/colliders.json,
// classified for the tasks (spec/README.md section 5): 'deck' (the box named deck), 'street' (the ground plane and
// boxes whose top is below model.contact.street_top_max: sidewalks, median) and 'other' (parapets, piers).
// Mirrored by python/coldgas/world.py.
import { GENERATED } from '../core/spec.js';
import { MODEL } from './model.js';
import { cos, sin } from './fmath.js';

export const KIND = { street: 0, deck: 1, other: 2 };
export const GROUND = { y: GENERATED.colliders.ground.y, mu: GENERATED.colliders.ground.mu, kind: KIND.street };
export const BOXES = GENERATED.colliders.boxes.map(b => {
  const cy = cos(b.yaw), sy = sin(b.yaw), top = b.c[1] + b.e[1];
  const kind = b.name === 'deck' ? KIND.deck : top < MODEL.contact.street_top_max ? KIND.street : KIND.other;
  // world AABB half extents (broadphase only; never changes a result)
  const rx = Math.abs(cy) * b.e[0] + Math.abs(sy) * b.e[2], rz = Math.abs(sy) * b.e[0] + Math.abs(cy) * b.e[2];
  return { name: b.name, c: b.c, e: b.e, mu: b.mu, cy, sy, top, kind, rx, rz };
});

/** Height of the street surface under (x, z): the ground plane or the top of a street box containing the point. */
export function streetLevel(x, z) {
  let y = GROUND.y;
  for (const b of BOXES) {
    if (b.kind !== KIND.street) continue;
    const dx = x - b.c[0], dz = z - b.c[2];
    const lx = b.cy * dx - b.sy * dz, lz = b.sy * dx + b.cy * dz;
    if (Math.abs(lx) < b.e[0] && Math.abs(lz) < b.e[2] && b.top > y) y = b.top;
  }
  return y;
}

/** Height of the highest support surface (any box top or the ground) under (x, z) below height yMax. */
export function supportLevel(x, z, yMax = Infinity) {
  let y = GROUND.y;
  for (const b of BOXES) {
    const dx = x - b.c[0], dz = z - b.c[2];
    const lx = b.cy * dx - b.sy * dz, lz = b.sy * dx + b.cy * dz;
    if (Math.abs(lx) < b.e[0] && Math.abs(lz) < b.e[2] && b.top > y && b.top <= yMax) y = b.top;
  }
  return y;
}
