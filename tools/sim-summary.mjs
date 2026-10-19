import fs from 'fs'; import zlib from 'zlib';
import { unpackRecording } from '../src/sim/recording.js';
for (const name of process.argv.slice(2)) {
  const raw = zlib.gunzipSync(fs.readFileSync(`public/sims/${name}.sim`));
  const { meta, frames } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
  const b = meta.events.breaks;
  const by = {}; for (const e of b) by[e.type] = (by[e.type] || 0) + 1;
  console.log(name, JSON.stringify(by), 'latches:', b.filter(e => e.type === 'latch').map(e => e.piece + '@' + e.t.toFixed(2)).join(' '), 'particles', meta.particles.count, 'tears', meta.tearTime.filter(t => t < 1e8).length, 'end body', Array.from(frames.at(-1).body).map(v => v.toFixed(1)).join(','));
}
