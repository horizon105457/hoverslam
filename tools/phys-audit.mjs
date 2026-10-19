// Physics sanity audit of a recorded run: ground / deck penetration, per-piece lowest point, peak speeds.
import fs from 'fs'; import zlib from 'zlib';
import { loadCarNode } from './load-car-node.mjs';
import { unpackRecording, RecordingPlayer } from '../src/sim/recording.js';
import { staticColliders } from '../src/scene/layout.js';
const name = process.argv[2] || 'roof';
const car = await loadCarNode({ h: 0.065 });
const raw = zlib.gunzipSync(fs.readFileSync(`public/sims/${name}.sim`));
const { meta, frames } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
const P = new RecordingPlayer(car, meta, frames);
const boxes = staticColliders();
const N = car.nodes.count;
const pieceOfNode = new Int16Array(N);
car.chunks.forEach((c, ci) => { for (let i = 0; i < c.nodeCount; i++) pieceOfNode[c.nodeStart + i] = ci; });
let worstGround = 0, worstBox = 0, worstT = 0;
const rows = [];
let prev = null;
for (let t = 0; t <= P.duration; t += 0.01) {
  const pos = P.sample(t);
  let minY = 1e9, pen = 0, boxPen = 0, boxName = '', vmax = 0;
  for (let n = 0; n < N; n++) {
    const x = pos[n * 3], y = pos[n * 3 + 1], z = pos[n * 3 + 2];
    if (y < minY) minY = y;
    if (-y > pen) pen = -y;
    for (const b of boxes) {
      const dx = Math.abs(x - b.c[0]) - b.e[0], dy = Math.abs(y - b.c[1]) - b.e[1], dz = Math.abs(z - b.c[2]) - b.e[2];
      if (dx < 0 && dy < 0 && dz < 0) { const d = -Math.max(dx, dy, dz); if (d > boxPen) { boxPen = d; boxName = b.name; } }
    }
    if (prev) { const v = Math.hypot(x - prev[n * 3], y - prev[n * 3 + 1], z - prev[n * 3 + 2]) / 0.01; if (v > vmax) vmax = v; }
  }
  prev = Float32Array.from(pos);
  if (pen > worstGround) { worstGround = pen; worstT = t; }
  worstBox = Math.max(worstBox, boxPen);
  const c = P.bodyCentre(t);
  if (Math.round(t * 100) % 10 === 0) rows.push(`t=${t.toFixed(2)} body=(${c.map(v => v.toFixed(2)).join(',')}) minY=${minY.toFixed(3)} groundPen=${pen.toFixed(3)} boxPen=${boxPen.toFixed(3)}${boxName ? '(' + boxName + ')' : ''} vmaxNode=${vmax.toFixed(1)}`);
}
console.log(rows.join('\n'));
console.log(`worst ground penetration ${worstGround.toFixed(3)} m at t=${worstT.toFixed(2)}; worst box penetration ${worstBox.toFixed(3)} m`);
console.log('launch', JSON.stringify(meta.launch), 'duration', P.duration);
