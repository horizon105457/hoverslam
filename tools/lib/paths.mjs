// Repository paths for Node tools (independent of the current working directory).
import path from 'path';
import { fileURLToPath } from 'url';
export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
/** Path inside the repo; absolute arguments pass through unchanged. */
export const repo = (...p) => path.resolve(ROOT, ...p);
