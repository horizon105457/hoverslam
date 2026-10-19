// Render individual director frames (film path) to PNG for quick look-dev checks.
// usage: node tools/film-frame.mjs preset=roof T=7,10.5,19 out=dir [w=1280 h=720 samples=4]
import { chromium } from 'playwright-core';
import fs from 'fs';
import path from 'path';
const a = Object.fromEntries(process.argv.slice(2).map(s => s.split('=')));
const W = +(a.w || 1280), H = +(a.h || 720), samples = +(a.samples || 4), preset = a.preset || 'roof';
const out = path.resolve(a.out || 'frames'); fs.mkdirSync(out, { recursive: true });
const browser = await chromium.launch({ channel: 'chrome', headless: true, args: ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
page.on('pageerror', e => console.log('[pageerror]', e.message));
page.on('console', m => { if (m.type() === 'error' || m.type() === 'warning') console.log(`[${m.type()}]`, m.text()); });
await page.goto(`http://127.0.0.1:5173/?capture&paused&preset=${preset}&w=${W}&h=${H}&pr=1&film`);
await page.waitForFunction(() => window.app?.ready, null, { timeout: 300000 });
await page.evaluate(() => document.body.classList.add('clean'));
const info = await page.evaluate(() => ({ total: window.app.director.total, shots: window.app.director.shots.map(s => [s.name, +s.start.toFixed(2), +s.dur.toFixed(2), +s.t0.toFixed(2), +s.t1.toFixed(2)]), tImp: window.app.director.tImp, tEdge: window.app.director.tEdge }));
console.log(JSON.stringify(info));
for (const T of (a.T || '0').split(',').map(Number)) {
  const png = await page.evaluate(({ T, samples, js }) => { if (js) eval(js); window.app.renderFilm(T, { samples, shutter: 0.5 / 30 }); return window.app.stage.renderer.domElement.toDataURL('image/png'); }, { T, samples, js: a.js || '' });
  fs.writeFileSync(path.join(out, `T${T.toFixed(2)}.png`), Buffer.from(png.split(',')[1], 'base64'));
}
await browser.close();
