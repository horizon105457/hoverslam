import fs from 'fs';
import { HDRLoader } from 'three/addons/loaders/HDRLoader.js';
const buf = fs.readFileSync('public/assets/hdri/sky_4k.hdr');
const d = new HDRLoader().setDataType(1015).parse(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.length));
const { width: W, height: H, data } = d;
let best = 0, bi = 0, sum = [0,0,0];
for (let i = 0; i < W * H; i++) { const l = data[i*4] + data[i*4+1] + data[i*4+2]; if (l > best) { best = l; bi = i; } }
const x = bi % W, y = Math.floor(bi / W);
const u = x / W, v = y / H;
const phi = u * 2 * Math.PI, theta = v * Math.PI; // equirect: u -> longitude, v -> polar from top
// three.js equirect mapping: dir = (-cos(phi)*sin(theta)?) -- print raw for reference
console.log('W', W, 'H', H, 'brightest', best.toFixed(0), 'px', x, y, 'u', u.toFixed(4), 'v', v.toFixed(4), 'elev deg', (90 - v * 180).toFixed(1));
// average sky colour of upper hemisphere & horizon
let up = [0,0,0], n = 0; for (let yy = 0; yy < H/2; yy += 8) for (let xx = 0; xx < W; xx += 8) { const i = (yy*W+xx)*4; up[0]+=data[i]; up[1]+=data[i+1]; up[2]+=data[i+2]; n++; } console.log('upper avg', up.map(q=>(q/n).toFixed(3)));
