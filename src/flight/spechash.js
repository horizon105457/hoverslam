// Spec hash (spec/README.md section 7): FNV-1a 64 over every *.json under spec/, sorted by relative path; per file
// feed the path (UTF-8), 0x00, the raw bytes, 0x00. Mirrors python/coldgas/spec.py spec_hash(). Trajectories,
// policies and benchmark results record it; consumers warn on mismatch.
const OFFSET = 0xcbf29ce484222325n, PRIME = 0x100000001b3n, MASK = (1n << 64n) - 1n;

/** @param {Iterable<Uint8Array>} chunks */
export function fnv1a64(chunks) {
  // 64-bit multiply split in 32-bit halves: BigInt per byte is ~20x slower and the spec is ~60 kB
  let hi = Number(OFFSET >> 32n) >>> 0, lo = Number(OFFSET & 0xffffffffn) >>> 0;
  const PH = Number(PRIME >> 32n), PL = Number(PRIME & 0xffffffffn); // PH = 0x100, PL = 0x1b3
  for (const b of chunks) for (let i = 0; i < b.length; i++) {
    lo = (lo ^ b[i]) >>> 0;
    // (hi:lo) * (PH:PL) mod 2^64, with 16-bit limbs for the low product so nothing exceeds 2^53
    const l0 = lo & 0xffff, l1 = lo >>> 16;
    const p0 = l0 * PL, p1 = l1 * PL; // < 2^25 each
    const loNew = (p0 + ((p1 & 0xffff) << 16)) >>> 0;
    const carry = Math.floor((p0 + (p1 & 0xffff) * 65536) / 4294967296) + Math.floor(p1 / 65536);
    hi = (Math.imul(hi, PL) + Math.imul(lo, PH) + carry) >>> 0;
    lo = loNew;
  }
  return (BigInt(hi) << 32n | BigInt(lo)) & MASK;
}

/** @param {{path:string, bytes:Uint8Array}[]} entries  paths relative to spec/, '/' separators */
export function specHashOf(entries) {
  const sorted = [...entries].sort((a, b) => (a.path < b.path ? -1 : a.path > b.path ? 1 : 0));
  const enc = new TextEncoder(), Z = new Uint8Array(1), chunks = [];
  for (const e of sorted) chunks.push(enc.encode(e.path), Z, e.bytes, Z);
  return fnv1a64(chunks).toString(16).padStart(16, '0');
}

let cached = null;
/** Spec hash of this checkout: Node reads spec/ from disk, Vite inlines the files (import.meta.glob ?raw). */
export async function specHash() {
  if (cached) return cached;
  let entries = null;
  try {
    // Vite rewrites this call at build time; in Node import.meta.glob is undefined and this throws
    const files = import.meta.glob('../../spec/**/*.json', { query: '?raw', import: 'default', eager: true });
    const enc = new TextEncoder();
    entries = Object.entries(files).map(([k, text]) => ({ path: k.replace(/^.*?\/spec\//, ''), bytes: enc.encode(text) }));
  } catch {
    // Node only; specifiers hidden from Vite's import analysis (this branch never runs in the browser)
    const imp = m => import(/* @vite-ignore */ m);
    const fs = await imp('fs'), path = await imp('path'), url = await imp('url');
    const dir = path.resolve(path.dirname(url.fileURLToPath(import.meta.url)), '../../spec');
    const walk = d => fs.readdirSync(d, { withFileTypes: true }).flatMap(e => (e.isDirectory() ? walk(path.join(d, e.name)) : e.name.endsWith('.json') ? [path.join(d, e.name)] : []));
    entries = walk(dir).map(f => ({ path: path.relative(dir, f).split(path.sep).join('/'), bytes: new Uint8Array(fs.readFileSync(f)) }));
  }
  return (cached = specHashOf(entries));
}
