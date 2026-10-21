// Diagnose a program that does not load: open the app headless and print console messages, page errors, failed
// requests and the GPU. usage: node tools/film/debug-page.mjs "program=<id>&<query>" [wait ms, 20000] [port=<p> via $PORT]
import { chromium } from 'playwright-core';
import { appBase } from '../lib/browser.mjs';
const browser = await chromium.launch({ channel: 'chrome', headless: true, args: ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
page.on('console', m => console.log(`[${m.type()}] ${m.text()}`));
page.on('pageerror', e => console.log(`[pageerror] ${e.message}`));
page.on('requestfailed', r => console.log(`[reqfail] ${r.url()} ${r.failure()?.errorText}`));
await page.goto(`${appBase()}/?capture&paused&${process.argv[2] || ''}`);
await page.waitForTimeout(+(process.argv[3] || 20000));
console.log('load-text:', await page.evaluate(() => document.getElementById('load-text')?.textContent), 'app?', await page.evaluate(() => !!window.app), await page.evaluate(() => !!window.app?.ready));
console.log(await page.evaluate(() => { const gl = document.createElement('canvas').getContext('webgl2'); const d = gl.getExtension('WEBGL_debug_renderer_info'); return d ? gl.getParameter(d.UNMASKED_RENDERER_WEBGL) : 'n/a'; }));
await browser.close();
