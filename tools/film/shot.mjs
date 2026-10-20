// Headless Chrome screenshot of the realtime app (UI included).
// usage: node tools/film/shot.mjs out.png "program=crash&preset=roof&mode=free" [evalJS]   (env W, H, PORT)
import { launch, appBase } from '../lib/browser.mjs';
const [out = 'shot.png', query = '', script = ''] = process.argv.slice(2);
const W = +(process.env.W || 1600), H = +(process.env.H || 900);
const { browser, page } = await launch({ width: W, height: H, swiftshader: true });
const logs = [];
page.on('console', m => logs.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', e => logs.push(`[pageerror] ${e.message}`));
await page.goto(`${appBase()}/?capture&paused&${query}`);
try {
  await page.waitForFunction(() => window.app?.ready || window.app?.error, null, { timeout: 180000 });
} catch (e) { console.log('timeout waiting for app'); }
if (script) await page.evaluate(script);
else await page.evaluate(() => { for (let i = 0; i < 3; i++) window.app.frame(1 / 60); });
await page.waitForTimeout(300);
await page.screenshot({ path: out });
console.log(logs.filter(l => !l.includes('[vite]')).slice(-25).join('\n'));
await browser.close();
