import { loadCarNode } from '../lib/car.mjs';
const car = await loadCarNode({ h: 0.065 });
const fc = car.chunks.findIndex(c => c.name === 'fender_front_L');
const ch = car.chunks[fc];
const N = car.nodes.count, R = car.nodes.rest;
const beams = []; for (let b = 0; b < car.beams.count; b++) if (car.nodes.chunk[car.beams.i[b]] === fc && car.nodes.chunk[car.beams.j[b]] === fc) beams.push([car.beams.i[b], car.beams.j[b], car.beams.L0[b]]);
function run(omega, rev, pinFirst) {
  const x = Float64Array.from(R), p = new Float64Array(N*3), v = new Float64Array(N*3), dt = 1/8000;
  const w = new Float64Array(N); for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) w[n] = 136;
  if (pinFirst) for (let n = ch.nodeStart; n < ch.nodeStart + 20; n++) w[n] = 0;
  let maxv = 0;
  for (let s = 0; s < 200; s++) {
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const i = n*3; p[i]=x[i];p[i+1]=x[i+1];p[i+2]=x[i+2]; if (!w[n]) continue; v[i+1] -= 9.81*dt; x[i]+=v[i]*dt;x[i+1]+=v[i+1]*dt;x[i+2]+=v[i+2]*dt; }
    const order = (rev && s % 2) ? [...beams].reverse() : beams;
    for (const [a, c, L0] of order) {
      const ws = w[a] + w[c]; if (!ws) continue;
      const dx = x[c*3]-x[a*3], dy = x[c*3+1]-x[a*3+1], dz = x[c*3+2]-x[a*3+2]; const L = Math.hypot(dx,dy,dz);
      const dl = -(L - L0) / ws * omega / L;
      x[a*3] -= w[a]*dl*dx; x[a*3+1] -= w[a]*dl*dy; x[a*3+2] -= w[a]*dl*dz;
      x[c*3] += w[c]*dl*dx; x[c*3+1] += w[c]*dl*dy; x[c*3+2] += w[c]*dl*dz;
    }
    for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) { const i = n*3; v[i]=(x[i]-p[i])/dt; v[i+1]=(x[i+1]-p[i+1])/dt; v[i+2]=(x[i+2]-p[i+2])/dt; }
  }
  for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) maxv = Math.max(maxv, Math.hypot(v[n*3], v[n*3+1], v[n*3+2]));
  return maxv.toFixed(4);
}
console.log('free, fwd', run(1, false, false), ' free, sym', run(1, true, false), ' pinned fwd', run(1, false, true), ' pinned sym', run(1, true, true), ' omega .5', run(0.5, true, true));
