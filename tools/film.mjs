// Offline film export: drives the app's director frame by frame with accumulation (motion blur, DOF, AA)
// and encodes an MP4 with ffmpeg.
// usage: node tools/film.mjs preset=roof out=film_roof.mp4 [w=1920 h=1080 fps=30 samples=16 from=0 to=end]
import { chromium } from 'playwright-core';
import fs from 'fs';
import path from 'path';
import { execFileSync } from 'child_process';
const a = Object.fromEntries(process.argv.slice(2).map(s => s.split('=')));
const W = +(a.w || 1920), H = +(a.h || 1080), fps = +(a.fps || 30), samples = +(a.samples || 16);
const preset = a.preset || 'roof';
const out = path.resolve(a.out || `film_${preset}.mp4`);
const dir = fs.mkdtempSync(path.join(process.env.TMPDIR || '/tmp', 'film-'));
const browser = await chromium.launch({ channel: 'chrome', headless: true, args: ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
page.on('pageerror', e => console.log('[pageerror]', e.message));
await page.goto(`http://127.0.0.1:5173/?capture&paused&preset=${preset}&w=${W}&h=${H}&pr=1&film`);
await page.waitForFunction(() => window.app?.ready, null, { timeout: 300000 });
await page.evaluate(() => document.body.classList.add('clean'));
const total = await page.evaluate(() => window.app.director.total);
const from = +(a.from || 0), to = a.to ? +a.to : total;
const n = Math.round((to - from) * fps);
console.log(`rendering ${n} frames ${W}x${H} @${fps} with ${samples} samples -> ${out}`);
const t0 = Date.now();
for (let i = 0; i < n; i++) {
  const T = from + i / fps;
  const png = await page.evaluate(({ T, samples, fps }) => { window.app.renderFilm(T, { samples, shutter: 0.5 / fps }); return window.app.stage.renderer.domElement.toDataURL('image/png'); }, { T, samples, fps });
  fs.writeFileSync(path.join(dir, `f${String(i).padStart(5, '0')}.png`), Buffer.from(png.split(',')[1], 'base64'));
  if (i % 10 === 0) process.stdout.write(`\r${i}/${n} ${((Date.now() - t0) / 1000 / (i + 1)).toFixed(2)} s/frame`);
}
await browser.close();
execFileSync('ffmpeg', ['-y', '-v', 'error', '-framerate', String(fps), '-i', path.join(dir, 'f%05d.png'), '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]);
console.log(`\nwrote ${out}`);
fs.rmSync(dir, { recursive: true, force: true });
