// `node tool.mjs key=value flag ...` -> { key: 'value', flag: true }
export function parseArgs(argv = process.argv.slice(2)) {
  return Object.fromEntries(argv.map(a => { const i = a.indexOf('='); return i < 0 ? [a, true] : [a.slice(0, i), a.slice(i + 1)]; }));
}
