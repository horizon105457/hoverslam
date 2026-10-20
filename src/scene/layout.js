// Shared world layout: the renderer builds visuals from it, the solver builds colliders from it.
// World: metres, +Y up. The viaduct runs along X. The car drives towards -X on the +Z carriageway
// (right-hand traffic) and leaves through a construction gap in the +Z parapet, on its right.
// Numbers live in spec/world.json and spec/launch-presets.json (shared with the Python twin).
import { WORLD, LAUNCH } from '../core/spec.js';

/** deck: top = road surface height, halfWidth = edge |z|, gap = missing +z parapet segment (construction zone) */
export const DECK = WORLD.deck;
/** w along X, d across (Z) at the base */
export const PIERS = WORLD.piers;
/** roadHalf: asphalt across z; cross: x-range of the crossing road (intersection below the launch point) */
export const GROUND = WORLD.ground;

/**
 * Static colliders: oriented boxes {c:[x,y,z], e:[hx,hy,hz], yaw, mu} plus the infinite ground plane y=0.
 * Written to spec/generated/colliders.json by `npm run spec` for the flight twins.
 */
export function staticColliders() {
  const boxes = [];
  const D = DECK;
  // deck slab (only the top matters for driving)
  boxes.push({ c: [0, D.top - 0.4, 0], e: [(D.x1 - D.x0) / 2, 0.4, D.halfWidth], yaw: 0, mu: 0.8, name: 'deck' });
  // parapets: -z full length; +z with the construction gap
  const pz = D.halfWidth - D.parapetW / 2;
  boxes.push({ c: [0, D.top + D.parapetH / 2, -pz], e: [(D.x1 - D.x0) / 2, D.parapetH / 2, D.parapetW / 2], yaw: 0, mu: 0.5, name: 'parapet' });
  const [g0, g1] = D.gap;
  boxes.push({ c: [(D.x0 + g0) / 2, D.top + D.parapetH / 2, pz], e: [(g0 - D.x0) / 2, D.parapetH / 2, D.parapetW / 2], yaw: 0, mu: 0.5, name: 'parapet' });
  boxes.push({ c: [(g1 + D.x1) / 2, D.top + D.parapetH / 2, pz], e: [(D.x1 - g1) / 2, D.parapetH / 2, D.parapetW / 2], yaw: 0, mu: 0.5, name: 'parapet' });
  // ground level: sidewalk slab and median kerb
  const G = GROUND;
  // sidewalks / green belt and the median kerb, interrupted by the crossing road
  const [cx0, cx1] = G.cross;
  for (const [a, b] of [[-400, cx0], [cx1, 400]]) {
    for (const [z0, z1] of [[G.sidewalk[0], G.greenBelt[1]], [G.greenBeltN[0], G.sidewalkN[1]]]) {
      boxes.push({ c: [(a + b) / 2, G.curbH / 2, (z0 + z1) / 2], e: [(b - a) / 2, G.curbH / 2, (z1 - z0) / 2], yaw: 0, mu: 0.7, name: 'sidewalk' });
    }
    boxes.push({ c: [(a + b) / 2, 0.1, 0], e: [(b - a) / 2, 0.1, G.medianHalf], yaw: 0, mu: 0.7, name: 'median' });
  }
  // piers
  for (let x = -256 + PIERS.offset; x < 260; x += PIERS.spacing) {
    boxes.push({ c: [x, (D.top - D.girderDepth) / 2, 0], e: [PIERS.w / 2, (D.top - D.girderDepth) / 2, PIERS.d / 2], yaw: 0, mu: 0.5, name: 'pier' });
  }
  return boxes;
}

/** Launch presets (spec/launch-presets.json). speed km/h, angle = heading toward the edge (deg),
 *  spin = angular kick from the broken kerb in car axes [pitch(+nose down), yaw, roll(+right side up)] rad/s. */
export const PRESETS = LAUNCH.presets;
/** Kerb kick timing shared by the soft body and the flight twins. */
export const KICK = LAUNCH.kick;
