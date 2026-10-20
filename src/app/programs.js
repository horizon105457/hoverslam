// Program registry: every src/programs/<id>/index.js is a program, discovered at build time.
const modules = import.meta.glob('../programs/*/index.js');
export const PROGRAM_IDS = Object.keys(modules).map(p => p.split('/')[2]);
export async function loadProgram(id) {
  const load = modules[`../programs/${id}/index.js`];
  if (!load) throw new Error(`unknown program "${id}" (have: ${PROGRAM_IDS.join(', ')})`);
  return (await load()).default;
}
