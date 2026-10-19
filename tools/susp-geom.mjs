import { loadCarNode } from './load-car-node.mjs';
const car = await loadCarNode({ h: 0.065 });
const R = car.nodes.rest;
for (let b = 0; b < car.beams.count; b++) { const k = car.beams.kind[b]; if (k !== 4 && k !== 5) continue; const a = car.beams.i[b], c = car.beams.j[b]; const d = [R[c*3]-R[a*3], R[c*3+1]-R[a*3+1], R[c*3+2]-R[a*3+2]]; const L = Math.hypot(...d); console.log(car.chunks[car.nodes.chunk[a]].name, k === 4 ? 'arm   ' : 'spring', 'L', L.toFixed(3), 'elev', (Math.asin(d[1]/L)*180/Math.PI).toFixed(0), 'dir', d.map(q=>(q/L).toFixed(2)).join(',')); }
