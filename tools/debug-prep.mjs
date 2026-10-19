import { loadCarNode } from './load-car-node.mjs';
const car = await loadCarNode({ h: +(process.argv[2] || 0.065) });
const R = car.nodes.rest;
const bb = (piece) => { const b=[1e9,1e9,1e9,-1e9,-1e9,-1e9]; for (const c of car.chunks) if (c.piece===piece) for (let n=c.nodeStart;n<c.nodeStart+c.nodeCount;n++) for(let a=0;a<3;a++){b[a]=Math.min(b[a],R[n*3+a]);b[a+3]=Math.max(b[a+3],R[n*3+a]);} return b.map(v=>v.toFixed(3)); };
console.log('hatch', bb('hatch')); console.log('glass_rear', bb('glass_rear'));
// worst embedding per mesh
const tn = car.render.tetNodes;
for (const m of car.render.meshes) { let w = 0; for (let v=0; v<m.aTet.length; v++){ const b0=1-m.aBary[v*3]-m.aBary[v*3+1]-m.aBary[v*3+2]; w=Math.min(w,b0,m.aBary[v*3],m.aBary[v*3+1],m.aBary[v*3+2]); } if (w < -0.5) console.log('outside', m.name, m.piece, w.toFixed(2)); }
{
  const hc = car.chunks.find(c=>c.piece==='hatch'), bc = car.chunks.find(c=>c.piece==='body');
  let best = 1e9, bestn=-1, cnt=0;
  for (let n=hc.nodeStart;n<hc.nodeStart+hc.nodeCount;n++){ if (R[n*3+1] < 1.2) continue; cnt++;
    for (let m=bc.nodeStart;m<bc.nodeStart+bc.nodeCount;m++){ const d=Math.hypot(R[n*3]-R[m*3],R[n*3+1]-R[m*3+1],R[n*3+2]-R[m*3+2]); if(d<best){best=d;bestn=n;} } }
  console.log('hatch nodes y>1.2:', cnt, 'closest body dist', best, bestn>=0 ? [...R.slice(bestn*3,bestn*3+3)] : '');
}
