// quick check of the packing format on an existing recording
import fs from 'fs'; import zlib from 'zlib';
import { packRecording, unpackRecording } from '../src/sim/recording.js';
const f = process.argv[2];
const raw = zlib.gunzipSync(fs.readFileSync(f));
console.log('unpack old not supported; rerun precompute');
