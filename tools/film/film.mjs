// Offline film export of any program: drives `window.app.film.render(T)` frame by frame with accumulation
// (motion blur, DOF, AA) and encodes an MP4 with ffmpeg. Needs a running dev server.
// usage: node tools/film/film.mjs [program=crash preset=roof] out=film.mp4 [w=1920 h=1080 fps=30 samples=16 from=0 to=end port=5173 q=extra&query]
import fs from 'fs';
import path from 'path';
import { execFileSync } from 'child_process';
import { parseArgs } from '../lib/args.mjs';
import { launch, openApp, programQuery } from '../lib/browser.mjs';
const a = parseArgs();
const W = +(a.w || 1920), H = +(a.h || 1080), fps = +(a.fps || 30), samples = +(a.samples || 16);
const query = programQuery(a);
const out = path.resolve(a.out || `film_${a.program || 'crash'}_${a.preset || 'default'}.mp4`);
const dir = fs.mkdtempSync(path.join(process.env.TMPDIR || '/tmp', 'film-'));
const { browser, page } = await launch({ width: W, height: H });
await openApp(page, a, query, { w: W, h: H });
const total = await page.evaluate(() => window.app.film.total);
const from = +(a.from || 0), to = a.to ? +a.to : total;
const n = Math.round((to - from) * fps);
console.log(`rendering ${n} frames ${W}x${H} @${fps} with ${samples} samples -> ${out}`);
const t0 = Date.now();
for (let i = 0; i < n; i++) {
  const T = from + i / fps;
  const png = await page.evaluate(({ T, samples, fps }) => { window.app.film.render(T, { samples, shutter: 0.5 / fps, fps }); return window.app.stage.renderer.domElement.toDataURL('image/png'); }, { T, samples, fps });
  fs.writeFileSync(path.join(dir, `f${String(i).padStart(5, '0')}.png`), Buffer.from(png.split(',')[1], 'base64'));
  if (i % 10 === 0) process.stdout.write(`\r${i}/${n} ${((Date.now() - t0) / 1000 / (i + 1)).toFixed(2)} s/frame`);
}
await browser.close();
execFileSync('ffmpeg', ['-y', '-v', 'error', '-framerate', String(fps), '-i', path.join(dir, 'f%05d.png'), '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]);
console.log(`\nwrote ${out}`);
fs.rmSync(dir, { recursive: true, force: true });
