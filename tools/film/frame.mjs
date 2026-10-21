// Render individual film frames of any program to PNG for quick look-dev checks.
// usage: node tools/film/frame.mjs [program=crash preset=roof] T=7,10.5,19 out=dir [w=1280 h=720 samples=4 port=5173 js=<eval before render>]
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../lib/args.mjs';
import { launch, openApp, programQuery } from '../lib/browser.mjs';
import { cwdPath } from '../lib/paths.mjs';
const a = parseArgs();
const W = +(a.w || 1280), H = +(a.h || 720), samples = +(a.samples || 4);
const out = cwdPath(a.out || 'frames'); fs.mkdirSync(out, { recursive: true });
const { browser, page } = await launch({ width: W, height: H });
page.on('console', m => { if (m.type() === 'warning') console.log('[warning]', m.text()); });
await openApp(page, a, programQuery(a), { w: W, h: H });
console.log(JSON.stringify(await page.evaluate(() => window.app.film.describe?.() ?? { total: window.app.film.total })));
for (const T of String(a.T ?? '0').split(',').map(Number)) {
  const png = await page.evaluate(({ T, samples, js }) => { if (js) eval(js); window.app.film.render(T, { samples, shutter: 0.5 / 30, fps: 30 }); return window.app.stage.renderer.domElement.toDataURL('image/png'); }, { T, samples, js: a.js || '' });
  fs.writeFileSync(path.join(out, `T${T.toFixed(2)}.png`), Buffer.from(png.split(',')[1], 'base64'));
}
await browser.close();
