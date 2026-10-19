import { loadCarNode } from './load-car-node.mjs';
const car = await loadCarNode({ h: +(process.argv[2] || 0.065) });
console.log(car.stats);
for (const c of car.chunks) console.log(c.name.padEnd(18), c.nodeCount);
console.log(car.groups.map(g => `${g.name}:${g.kind}:${g.hostNodes?.length}`).join(' '));
let m = 0; for (const x of car.nodes.mass) m += x; console.log('total mass', m.toFixed(1));
let cy = 0, cz = 0; for (let n = 0; n < car.nodes.count; n++) { cy += car.nodes.mass[n] * car.nodes.rest[n*3+1]; cz += car.nodes.mass[n] * car.nodes.rest[n*3+2]; } console.log('COM y', (cy/m).toFixed(3), 'z', (cz/m).toFixed(3));
const kinds = {}; for (const k of car.beams.kind) kinds[k] = (kinds[k]||0)+1; console.log('beam kinds', kinds);
let rv = 0; for (const r of car.render.meshes) rv += r.pos.length/3; console.log('render verts', rv);
