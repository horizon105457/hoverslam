// Read/write the soft-body artefacts: settled pose (public/assets/settled*.bin) and recordings (public/sims/*.sim).
import fs from 'fs';
import zlib from 'zlib';
import { repo } from './paths.mjs';
import { packRecording, unpackRecording } from '../../src/softbody/recording.js';

/** Settled pose: [u32 nodes][u32 beams][f32 xyz * nodes]. Pass `car` to verify it matches the lattice. */
export function readSettled(file = 'public/assets/settled.bin', car = null) {
  const buf = fs.readFileSync(repo(file));
  const head = new Uint32Array(buf.buffer.slice(buf.byteOffset, buf.byteOffset + 8));
  if (car && (head[0] !== car.nodes.count || head[1] !== car.beams.count)) throw new Error(`${file} is stale: run npm run settle`);
  return new Float32Array(buf.buffer.slice(buf.byteOffset + 8, buf.byteOffset + buf.length));
}
export function writeSettled(file, car, pos) {
  const head = new Uint32Array([car.nodes.count, car.beams.count]);
  fs.writeFileSync(repo(file), Buffer.concat([Buffer.from(head.buffer), Buffer.from(pos.buffer, pos.byteOffset, pos.byteLength)]));
}
export function simPath(name) { return repo('public/sims', `${name}.sim`); }
export function readSim(name) {
  const raw = zlib.gunzipSync(fs.readFileSync(simPath(name)));
  return unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
}
/** @returns {{raw:number, gz:number}} sizes in bytes */
export function writeSim(name, meta, frames) {
  const raw = packRecording(meta, frames);
  const gz = zlib.gzipSync(Buffer.from(raw), { level: 6 });
  fs.mkdirSync(repo('public/sims'), { recursive: true });
  fs.writeFileSync(simPath(name), gz);
  return { raw: raw.byteLength, gz: gz.length };
}
