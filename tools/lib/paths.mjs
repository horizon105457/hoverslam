// Repository paths for Node tools (independent of the current working directory).
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
/** Path inside the repo; absolute arguments pass through unchanged. */
export const repo = (...p) => path.resolve(ROOT, ...p);
/** A path argument as the shell user meant it: relative to the logical cwd ($PWD), so `../x` from a symlinked
 *  checkout (直到用计算机模拟/sim -> this repo) lands next to the link, not next to this repo. $PWD is used only when
 *  it is the same directory as process.cwd() (a parent that changed cwd without updating $PWD is ignored). */
export function cwdPath(p) {
  const pwd = process.env.PWD;
  let base = process.cwd();
  try { if (pwd && fs.realpathSync(pwd) === fs.realpathSync(base)) base = pwd; } catch {}
  return path.resolve(base, p);
}
