// Headless Chrome (system install, GPU) pointed at the running dev server.
// Base URL: `url=` argument, else $APP_URL, else http://127.0.0.1:$PORT (default 5173). Several dev servers
// can run side by side (`npx vite --port 5174`): pass port=5174 to the tools.
import { chromium } from 'playwright-core';

export function appBase(args = {}) {
  return args.url || process.env.APP_URL || `http://127.0.0.1:${args.port || process.env.PORT || 5173}`;
}
export async function launch({ width = 1280, height = 720, swiftshader = false } = {}) {
  const flags = ['--use-angle=metal', '--enable-gpu', '--ignore-gpu-blocklist'];
  if (swiftshader) flags.push('--enable-unsafe-swiftshader');
  const browser = await chromium.launch({ channel: 'chrome', headless: true, args: flags });
  const page = await browser.newPage({ viewport: { width, height }, deviceScaleFactor: 1 });
  return { browser, page };
}
/**
 * Open the app for offline rendering and wait until the program is ready.
 * @param {string} query extra query string, e.g. 'program=crash&preset=roof'
 */
export async function openApp(page, args, query, { w, h, timeout = 300000, log = true } = {}) {
  if (log) {
    page.on('pageerror', e => console.log('[pageerror]', e.message));
    page.on('console', m => { if (m.type() === 'error') console.log('[error]', m.text()); });
  }
  const size = w ? `&w=${w}&h=${h}&pr=1` : '';
  const base = appBase(args);
  try { await page.goto(`${base}/?capture&paused&film${size}&${query}`); } catch (e) {
    if (/ERR_CONNECTION_REFUSED/.test(e.message)) throw new Error(`no dev server at ${base}: start one (FREEZE=1 npm run dev -- --port <p>) and pass port=<p>`);
    throw e;
  }
  await page.waitForFunction(() => window.app?.ready || window.app?.error, null, { timeout });
  const err = await page.evaluate(() => window.app.error && String(window.app.error));
  if (err) throw new Error(`app failed: ${err}`);
  await page.evaluate(() => document.body.classList.add('clean'));
}
/** Program query from tool args: program=<id> plus any passthrough `q=` string; crash keeps preset=. */
export function programQuery(a) {
  const prog = a.program || 'crash';
  const parts = [`program=${prog}`];
  if (a.preset) parts.push(`preset=${a.preset}`);
  if (a.q) parts.push(a.q);
  return parts.join('&');
}
