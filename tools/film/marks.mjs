// Write only the compositor marks of a film (what film.mjs marks=1 writes next to the video), without rendering:
// for re-tuning overlays when only `film.marks(T)` changed.
// usage: node tools/film/marks.mjs program=<id> out=<film.mp4> [w=1920 h=1080 fps=30 from=0 to=end port=5173 q=extra&query]
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../lib/args.mjs';
import { launch, openApp, programQuery } from '../lib/browser.mjs';
import { cwdPath } from '../lib/paths.mjs';
const a = parseArgs();
const W = +(a.w || 1920), H = +(a.h || 1080), fps = +(a.fps || 30);
const { browser, page } = await launch({ width: W, height: H });
await openApp(page, a, programQuery(a), { w: W, h: H });
const total = await page.evaluate(() => window.app.film.total);
const from = +(a.from || 0), to = a.to ? +a.to : total, n = Math.round((to - from) * fps);
const frames = await page.evaluate(({ from, n, fps }) => Array.from({ length: n }, (_, i) => window.app.film.marks(from + i / fps)), { from, n, fps });
const describe = await page.evaluate(() => window.app.film.describe?.() ?? null);
await browser.close();
const out = cwdPath(a.out).replace(/\.mp4$/, '') + '.marks.json';
fs.writeFileSync(out, JSON.stringify({ describe, frames }));
console.log(`wrote ${out} (${n} frames)`);
