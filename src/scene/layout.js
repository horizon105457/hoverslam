// Shared world layout: the renderer builds visuals from it, the solver builds colliders from it.
// World: metres, +Y up. The viaduct runs along X. The car drives towards -X on the +Z carriageway
// (right-hand traffic) and leaves through a construction gap in the +Z parapet, on its right.

export const DECK = {
  top: 9.2,          // road surface height of the elevated deck
  halfWidth: 8.6,    // deck edge |z|
  slab: 0.32,        // top slab thickness
  girderDepth: 2.1,  // depth of the box girder under the road surface
  boxHalf: 3.6,      // half width of the girder box bottom
  x0: -260, x1: 260,
  parapetH: 1.05, parapetW: 0.5,
  gap: [-7, 9],      // missing parapet segment on the +z side (construction zone)
};
export const PIERS = { spacing: 32, offset: 6, w: 1.8, d: 2.4, z: 0 }; // w along X, d across (Z) at the base
export const GROUND = {
  roadHalf: [-17, 26],      // asphalt carriageway across z (the -z side mirrors +z shifted by 4 m)
  curbH: 0.16,
  sidewalk: [26, 30.5],
  greenBelt: [30.5, 37],
  sidewalkN: [-21.5, -17],  // far (-z) side
  greenBeltN: [-28, -21.5],
  cross: [-64, -16],        // crossing road x-range (intersection below the launch point)
  medianHalf: 1.1,          // planted median under the piers
};

/**
 * Static colliders: oriented boxes {c:[x,y,z], e:[hx,hy,hz], yaw, mu} plus the infinite ground plane y=0.
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

/** Scenario presets. speed km/h, angle = heading toward the edge (deg),
 *  spin = angular kick from the broken kerb in car axes [pitch(+nose down), yaw, roll(+right side up)] rad/s. */
export const PRESETS = {
  nose:   { label: '车头俯冲', speed: 72, angle: 14, spin: [1.6, -0.15, -0.25], seed: 11, joints: 0.55 },
  roof:   { label: '倒扣车顶', speed: 70, angle: 15, spin: [0.25, -0.25, -2.45], seed: 12, joints: 0.55 },
  side:   { label: '侧面拍击', speed: 66, angle: 16, spin: [0.1, -0.2, -1.2], seed: 13, joints: 0.55 },
  rear:   { label: '车尾坠地', speed: 62, angle: 13, spin: [-1.35, -0.1, -0.25], seed: 14, joints: 0.55 },
  corner: { label: '角部翻滚', speed: 78, angle: 17, spin: [1.05, -0.9, -1.55], seed: 15, joints: 0.55 },
  flat:   { label: '四轮平拍', speed: 64, angle: 14, spin: [0.05, 0, -0.05], seed: 16, joints: 0.55 },
};
