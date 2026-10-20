// Key moments of a recorded launch (soft-body recording player): when the car leaves the deck, when it
// first hits the street, where it lands and where it comes to rest. Used to build edits around the action.
import * as THREE from 'three';
import { DECK } from '../scene/layout.js';

/**
 * @param player RecordingPlayer (bodyCentre(t), duration, meta.events)
 * @returns {{tEdge:number, tImp:number, end:number, L:THREE.Vector3, R:THREE.Vector3}} L/R on the ground (y = 0)
 */
export function launchMoments(player) {
  const P = player, meta = P.meta, end = P.duration;
  // first ground impact: earliest dust event near ground level
  let tImp = null;
  const D = meta.events?.dust || [];
  for (let i = 0; i < D.length; i += 7) if (D[i + 2] < 1.2) { tImp = D[i]; break; }
  if (tImp === null) { // fall back: lowest point of the body
    let best = 1e9;
    for (let t = 0; t < end; t += 0.02) { const c = P.bodyCentre(t); if (c[1] < best) { best = c[1]; tImp = t; } }
  }
  // leaving the deck: body centre passes the deck edge
  let tEdge = 0.5;
  for (let t = 0; t < Math.min(end, tImp); t += 0.01) { if (P.bodyCentre(t)[2] > DECK.halfWidth) { tEdge = t; break; } }
  const L = P.bodyCentre(tImp), R = P.bodyCentre(end);
  return { tEdge, tImp, end, L: new THREE.Vector3(L[0], 0, L[2]), R: new THREE.Vector3(R[0], 0, R[2]) };
}
