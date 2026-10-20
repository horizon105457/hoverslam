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

/** Painted lines of each deck carriageway, |z| of the line centre: solid by the median barrier, dashed lane
 *  divider, solid edge line by the parapet (inner lane 0.75–4.3, outer lane 4.3–7.4). Render only. */
export const DECK_LINES = { inner: 0.75, divider: 4.3, outer: 7.4 };

/** Centre of mass [x, z] at t = 0 for launch velocity v: `lead` s before it is 0.9 m short of the deck edge,
 *  in the middle of the gap (spec/launch-presets.json; same expression in src/flight/twin.js). */
export function launchOrigin(v, lead) {
  return [(DECK.gap[0] + DECK.gap[1]) / 2 + 1.5 - v[0] * lead, DECK.halfWidth - v[2] * lead - 0.9];
}

const SWERVE = { aMax: 0.8 * 9.81, zMin: 1.3 }; // peak lateral acceleration; closest start to the median (car centre)
const smoothstep = u => u * u * (3 - 2 * u);
/**
 * The drive-in before a run starts (t < 0, render only: every run begins at t = 0 already on its launch heading).
 * Extrapolating that heading backwards put the car on the far carriageway, through the far parapet and the median
 * barrier. Instead it comes down the middle of the inner +z lane parallel to the deck and swerves onto the launch
 * heading. The heading follows a smoothstep, so the yaw rate starts and ends at zero (runs start without any).
 * It cannot come from the outer lane: at t = 0 the nose run is only 0.7 m into it, while turning onto 14° at 72 km/h
 * takes 1.5 m of sideways run even on a constant 0.8 g arc (2.3 m with the smooth swerve); that lane is also coned
 * off upstream of the gap. Launches that would need more than 0.8 g start further left, clear of the median barrier.
 * @param p0 [x, z] of the car's centre of mass at t = 0 (`launchOrigin`), v0 launch velocity [vx, vy, vz]
 * @returns pose(t) → {c, s, dx, dz, q, yaw, rate}: the car's state at t = 0 moved rigidly, x' = c·x + s·z + dx,
 *          z' = −s·x + c·z + dz, orientation q ⊗ q0 (a yaw about +Y); rate = yaw rate (rad/s)
 */
export function driveIn(p0, v0) {
  const v = Math.hypot(v0[0], v0[2]), th = Math.atan2(v0[2], -v0[0]);
  const pose = (yaw, x, z, rate) => { // rotate about p0 by yaw, then p0 → (x, z)
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return { c, s, dx: x - c * p0[0] - s * p0[1], dz: z + s * p0[0] - c * p0[1], q: [0, Math.sin(yaw / 2), 0, Math.cos(yaw / 2)], yaw, rate };
  };
  // not heading for the +z edge: straight back along the launch velocity
  if (!(v > 1 && th > 0.01)) return t => pose(0, p0[0] + v0[0] * Math.min(t, 0), p0[1] + v0[2] * Math.min(t, 0), 0);
  // sideways run per second of swerve, I = ∫0..1 sin(th·smoothstep(u)) du; the peak yaw rate is 1.5·th/T
  let I = 0; for (let k = 0; k < 64; k++) I += Math.sin(th * smoothstep((k + 0.5) / 64)) / 64;
  const lane = (DECK_LINES.inner + DECK_LINES.divider) / 2;
  const zs = Math.max(SWERVE.zMin, Math.min(lane, p0[1] - v * I * 1.5 * v * th / SWERVE.aMax));
  const T = Math.max(p0[1] - zs, 0.05) / (v * I);
  // path of the centre of mass through the swerve, integrated backwards from t = 0 (midpoint rule)
  const n = 256, h = T / n, X = new Float64Array(n + 1), Z = new Float64Array(n + 1);
  X[n] = p0[0]; Z[n] = p0[1];
  for (let i = n; i > 0; i--) { const psi = th * smoothstep((i - 0.5) / n); X[i - 1] = X[i] + v * Math.cos(psi) * h; Z[i - 1] = Z[i] - v * Math.sin(psi) * h; }
  return t => {
    if (t >= 0) return pose(0, p0[0], p0[1], 0);
    if (t <= -T) return pose(-th, X[0] - v * (t + T), Z[0], 0);
    const u = (t + T) / T, f = u * n, i = Math.min(n - 1, Math.floor(f)), a = f - i;
    return pose(th * (smoothstep(u) - 1), X[i] + (X[i + 1] - X[i]) * a, Z[i] + (Z[i + 1] - Z[i]) * a, th * 6 * u * (1 - u) / T);
  };
}
