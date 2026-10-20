import { chromium } from 'playwright-core';
import { appBase } from '../lib/browser.mjs';
const browser = await chromium.launch({ channel: 'chrome', headless: true, args: ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
await page.goto(`${appBase()}/?capture&paused&preset=${process.argv[2] || 'nose'}&T=${process.argv[3] || 11}`);
await page.waitForFunction(() => window.app?.ready, null, { timeout: 180000 });
const r = await page.evaluate(() => {
  const app = window.app, gl = app.stage.renderer.getContext(), px = new Uint8Array(4);
  const sync = () => gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
  for (let i = 0; i < 5; i++) app.frame(1 / 60); sync();
  let t = performance.now();
  for (let i = 0; i < 30; i++) { app.frame(1 / 60); sync(); }
  const full = (performance.now() - t) / 30;
  // CPU-only part (sampling + uploads)
  t = performance.now();
  for (let i = 0; i < 30; i++) { const P = app.player, tt = 2.1 + i * 0.001; const pos = P.sample(tt); app.carView.update(pos, tt, P.sampleDamage(tt)); app.shards.update(P, tt); }
  const cpu = (performance.now() - t) / 30;
  return { full, cpu, info: app.stage.renderer.info.render };
});
console.log(JSON.stringify(r));
await browser.close();
