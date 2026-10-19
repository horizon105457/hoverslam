// Headless Chrome screenshots of the running app (uses the system Chrome with GPU).
// usage: node tools/shot.mjs out.png "preset=roof&mode=free" [evalJS]
import { chromium } from 'playwright-core';
const [out = 'shot.png', query = '', script = ''] = process.argv.slice(2);
const W = +(process.env.W || 1600), H = +(process.env.H || 900);
const browser = await chromium.launch({ channel: 'chrome', headless: true, args: ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: W, height: H } });
const logs = [];
page.on('console', m => logs.push(`[${m.type()}] ${m.text()}`));
page.on('pageerror', e => logs.push(`[pageerror] ${e.message}`));
await page.goto(`http://127.0.0.1:5173/?capture&paused&${query}`);
try {
  await page.waitForFunction(() => window.app?.ready || document.getElementById('load-text')?.textContent.startsWith('出错'), null, { timeout: 180000 });
} catch (e) { console.log('timeout waiting for app'); }
if (script) await page.evaluate(script);
else await page.evaluate(() => { for (let i = 0; i < 3; i++) window.app.frame(1 / 60); });
await page.waitForTimeout(300);
await page.screenshot({ path: out });
console.log(logs.filter(l => !l.includes('[vite]')).slice(-25).join('\n'));
await browser.close();
