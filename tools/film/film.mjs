// Offline film export of any program: drives `window.app.film.render(T)` frame by frame with accumulation
// (motion blur, DOF, AA) and encodes an MP4 with ffmpeg. Needs a running dev server.
// usage: node tools/film/film.mjs [program=crash preset=roof] out=renders/x.mp4 [w=1920 h=1080 fps=30 samples=16 from=0 to=end port=5173 q=extra&query]
//        out is required; relative paths are taken from the shell's cwd ($PWD, also through a symlinked checkout)
//        marks=1: also write <out>.marks.json = {describe: film.describe(), frames: [film.marks(T) per frame]} (key
//        times and screen points for a compositor)
import fs from 'fs';
import path from 'path';
import { execFileSync } from 'child_process';
import { parseArgs } from '../lib/args.mjs';
import { launch, openApp, programQuery } from '../lib/browser.mjs';
import { cwdPath } from '../lib/paths.mjs';
const a = parseArgs();
if (!a.out) throw new Error('out= is required, e.g. out=renders/x.mp4 (the directory is created)');
const W = +(a.w || 1920), H = +(a.h || 1080), fps = +(a.fps || 30), samples = +(a.samples || 16);
const query = programQuery(a);
const out = cwdPath(a.out);
fs.mkdirSync(path.dirname(out), { recursive: true });
const dir = fs.mkdtempSync(path.join(process.env.TMPDIR || '/tmp', 'film-'));
const { browser, page } = await launch({ width: W, height: H });
await openApp(page, a, query, { w: W, h: H });
const total = await page.evaluate(() => window.app.film.total);
const from = +(a.from || 0), to = a.to ? +a.to : total;
const n = Math.round((to - from) * fps);
console.log(`rendering ${n} frames ${W}x${H} @${fps} with ${samples} samples -> ${out}`);
const t0 = Date.now(), marks = [];
for (let i = 0; i < n; i++) {
  const T = from + i / fps;
  const png = await page.evaluate(({ T, samples, fps }) => { window.app.film.render(T, { samples, shutter: 0.5 / fps, fps }); return window.app.stage.renderer.domElement.toDataURL('image/png'); }, { T, samples, fps });
  if (a.marks) marks.push(await page.evaluate(T => window.app.film.marks(T), T));
  fs.writeFileSync(path.join(dir, `f${String(i).padStart(5, '0')}.png`), Buffer.from(png.split(',')[1], 'base64'));
  if (i % 10 === 0) process.stdout.write(`\r${i}/${n} ${((Date.now() - t0) / 1000 / (i + 1)).toFixed(2)} s/frame`);
}
const desc = a.marks ? await page.evaluate(() => window.app.film.describe?.() ?? null) : null;
await browser.close();
execFileSync('ffmpeg', ['-y', '-v', 'error', '-framerate', String(fps), '-i', path.join(dir, 'f%05d.png'), '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]);
if (a.marks) fs.writeFileSync(out.replace(/\.mp4$/, '') + '.marks.json', JSON.stringify({ describe: desc, frames: marks }));
console.log(`\nwrote ${out}`);
fs.rmSync(dir, { recursive: true, force: true });
