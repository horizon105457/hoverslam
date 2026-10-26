// Small multiple-shooting SQP layer over the guidance prediction model and the shared sparse QP backend.
// Every call optimizes valve controls with fixed stage durations; terminal time is therefore fixed.
import { createQP } from '../qp/index.js';
import { ND, NU, NX } from './model.js';

const finite = x => Number.isFinite(x);
const asPacked = (model, s) => (ArrayBuffer.isView(s) || Array.isArray(s)) && s.length === NX ? Float64Array.from(s) : model.pack(s);
const vec = (x, n, fallback, name) => {
  if (x === undefined || x === null) return Float64Array.from({ length:n },(_,i)=>fallback[i] ?? fallback[0] ?? 0);
  if (typeof x === 'number') return new Float64Array(n).fill(x);
  if (ArrayBuffer.isView(x) || Array.isArray(x)) {
    if (x.length !== n) throw new RangeError(`${name} must have ${n} entries`);
    return Float64Array.from(x, Number);
  }
  throw new TypeError(`${name} must be a number or length-${n} array`);
};
const defaultStateWeights = Float64Array.from({length:ND},(_,i)=>i<3?1:i<6?0.1:i<9?1:i<12?0.1:i===12?0.001:0.05);
function stateWeights(weights, fallback, name) {
  if (weights === undefined || weights === null) return Float64Array.from(fallback);
  if (typeof weights === 'number' || Array.isArray(weights) || ArrayBuffer.isView(weights)) return vec(weights,ND,fallback,name);
  if (typeof weights !== 'object') throw new TypeError(`${name} must be a number, array or weight object`);
  const out=Float64Array.from(fallback), parts=[['p',0,3],['v',3,3],['rotation',6,3],['attitude',6,3],['w',9,3],['omega',9,3],['gas',12,1],['valve',13,NU]];
  for(const [key,offset,count] of parts) if(weights[key] !== undefined) {
    const value=weights[key];
    if(typeof value==='number') for(let j=0;j<count;j++) out[offset+j]=value;
    else if((Array.isArray(value)||ArrayBuffer.isView(value))&&value.length===count) for(let j=0;j<count;j++) out[offset+j]=+value[j];
    else throw new RangeError(`${name}.${key} must be scalar or length-${count} array`);
  }
  return out;
}
function checkedNonnegative(w,name) { for(let i=0;i<w.length;i++) if(!(finite(w[i])&&w[i]>=0)) throw new RangeError(`${name}[${i}] must be finite and nonnegative`); return w; }
function packedControls(controls,n) {
  if(!controls || controls.length!==n) throw new RangeError(`controls must contain ${n} stage vectors`);
  return Array.from(controls,(u,k)=>{
    if(!u || u.length!==NU) throw new RangeError(`controls[${k}] must have ${NU} entries`);
    return Float64Array.from(u,(x)=>Math.max(0,Math.min(1,finite(+x)?+x:0)));
  });
}
function referencesPacked(model,references,nominal) {
  if(references===undefined||references===null) return nominal.map(x=>Float64Array.from(x));
  if((ArrayBuffer.isView(references)||Array.isArray(references))&&references.length===NX&&typeof references[0]==='number') return nominal.map(()=>Float64Array.from(references));
  if(!Array.isArray(references)||references.length!==nominal.length) throw new RangeError(`references must contain ${nominal.length} node states`);
  return references.map(s=>asPacked(model,s));
}
function rollout(model,x0,controls,dts) {
  const states=[Float64Array.from(x0)];
  for(let k=0;k<controls.length;k++) states.push(model.step(states[k],controls[k],dts[k]));
  return states;
}
function csc(ncol, entries) {
  const byCol=Array.from({length:ncol},()=>[]);
  for(const [r,c] of entries) byCol[c].push(r);
  const p=new Int32Array(ncol+1), rows=[];
  for(let c=0;c<ncol;c++) { const sorted=[...new Set(byCol[c])].sort((a,b)=>a-b); rows.push(...sorted); p[c+1]=rows.length; }
  const i=Int32Array.from(rows), index=new Map();
  for(let c=0;c<ncol;c++) for(let k=p[c];k<p[c+1];k++) index.set(`${i[k]},${c}`,k);
  return {p,i,index};
}
const key=(r,c)=>`${r},${c}`;

function costOf(model,states,controls,dts,refs,Q,Qf,Ru,Rs) {
  let cost=0;
  for(let k=0;k<states.length;k++) {
    const d=model.minus(states[k],refs[k]), w=k===states.length-1?Qf:Q;
    for(let i=0;i<ND;i++) cost+=0.5*w[i]*d[i]*d[i];
  }
  for(let k=0;k<controls.length;k++) for(let j=0;j<NU;j++) cost+=0.5*Ru[j]*controls[k][j]*controls[k][j];
  for(let k=1;k<controls.length;k++) for(let j=0;j<NU;j++) { const d=controls[k][j]-controls[k-1][j]; cost+=0.5*Rs[j]*d*d; }
  return cost;
}

/**
 * Run a bounded, fixed-duration multiple-shooting SQP pass sequence.
 * `initial` is FlightState or packed NX; `controls` is N x NU; `dts` is N fixed positive durations.
 * `references`, when supplied, contains N+1 FlightState/packed-NX node references. State weights are diagonal
 * local-tangent penalties (`p`, `v`, `rotation`, `w`, `gas`, `valve`, or a full ND vector); `weights.control` and
 * `weights.smooth` are nonnegative scalar/NU vectors. Without references, the warm-start nonlinear rollout is the
 * target, so the solver only changes the plan when control regularization/smoothing can improve its cost. Returned
 * `diagnostics.success` means each attempted QP solve succeeded; `stop` reports when nonlinear line search found no
 * improving candidate.
 */
export function optimizeTrajectory({ model, initial, controls, dts, references, iterations=3, weights={}, terminalWeights, trust=0.25, qpSettings }={}) {
  if(!model||typeof model.step!=='function'||typeof model.linearize!=='function'||typeof model.minus!=='function') throw new TypeError('model must provide step, linearize and minus');
  if(!dts||!dts.length) throw new RangeError('dts must contain at least one stage');
  const N=dts.length, dt=Float64Array.from(dts,Number);
  for(let k=0;k<N;k++) if(!(finite(dt[k])&&dt[k]>0)) throw new RangeError(`dts[${k}] must be positive and finite`);
  if(!Number.isInteger(iterations)||iterations<0) throw new RangeError('iterations must be a nonnegative integer');
  const x0=asPacked(model,initial), U=packedControls(controls,N);
  const directStateWeights=['p','v','rotation','attitude','w','omega','gas','valve'].some(k=>weights[k]!==undefined) ? weights : undefined;
  const Q=checkedNonnegative(stateWeights(weights.state ?? directStateWeights,defaultStateWeights,'weights.state'),'weights.state');
  const Qf=checkedNonnegative(stateWeights(terminalWeights ?? weights.terminal,Q,'terminalWeights'),'terminalWeights');
  const Ru=checkedNonnegative(vec(weights.control,NU,new Float64Array(NU).fill(1e-4),'weights.control'),'weights.control');
  const Rs=checkedNonnegative(vec(weights.smooth,NU,new Float64Array(NU).fill(1e-2),'weights.smooth'),'weights.smooth');
  const trustX=vec(typeof trust==='number'?trust:(trust.state ?? 0.25),ND,new Float64Array(ND).fill(0.25),'trust.state');
  const trustU=vec(typeof trust==='number'?trust:(trust.control ?? 0.25),NU,new Float64Array(NU).fill(0.25),'trust.control');
  if(Array.from(trustX).some(x=>!finite(x)||x<0)||Array.from(trustU).some(x=>!finite(x)||x<0)||!(Math.max(...trustX)>0&&Math.max(...trustU)>0)) throw new RangeError('trust radii must be finite, nonnegative, with at least one positive state and control radius');
  const nx=(N+1)*ND, nv=nx+N*NU, ndyn=N*ND, m=ndyn+nv;
  const xi=(k,j)=>k*ND+j, ui=(k,j)=>nx+k*NU+j;
  const pEntries=[];
  for(let j=0;j<nv;j++) pEntries.push([j,j]);
  // A nonzero attitude residual couples the three body-tangent coordinates in its Gauss-Newton block.
  for(let k=0;k<=N;k++) for(let a=0;a<3;a++) for(let b=a+1;b<3;b++) pEntries.push([xi(k,6+a),xi(k,6+b)]);
  for(let k=1;k<N;k++) for(let j=0;j<NU;j++) pEntries.push([ui(k-1,j),ui(k,j)]);
  const P=csc(nv,pEntries.map(([r,c])=>[Math.min(r,c),Math.max(r,c)]));
  const aEntries=[];
  for(let k=0;k<N;k++) {
    const row=k*ND;
    for(let i=0;i<ND;i++) {
      for(let j=0;j<ND;j++) aEntries.push([row+i,xi(k,j)]);
      aEntries.push([row+i,xi(k+1,i)]);
      for(let j=0;j<NU;j++) aEntries.push([row+i,ui(k,j)]);
    }
  }
  for(let j=0;j<nv;j++) aEntries.push([ndyn+j,j]);
  const A=csc(nv,aEntries), Px=new Float64Array(P.i.length), Ax=new Float64Array(A.i.length);
  const solver=createQP({n:nv,m,Pp:P.p,Pi:P.i,Ap:A.p,Ai:A.i,settings:{verbose:false,max_iter:3000,eps_abs:1e-4,eps_rel:1e-4,...(weights.qpSettings||{}),...(qpSettings||{})}});
  let warmStart, diagnostics=[], acceptedStates=rollout(model,x0,U,dt), refs=referencesPacked(model,references,acceptedStates);
  let currentCost=costOf(model,acceptedStates,U,dt,refs,Q,Qf,Ru,Rs), stop='iteration_limit';
  const attDelta=new Float64Array(ND), attPlus=new Float64Array(NX), attMinus=new Float64Array(NX);
  const errPlus=new Float64Array(ND), errMinus=new Float64Array(ND), attJac=new Float64Array(9), hAtt=1e-5;
  try {
    for(let it=0;it<iterations;it++) {
      const q=new Float64Array(nv), l=new Float64Array(m), u=new Float64Array(m), pValues=new Map(), addP=(r,c,v)=>{
        if(r>c){const t=r;r=c;c=t;} const k=key(r,c); pValues.set(k,(pValues.get(k)||0)+v);
      };
      for(let k=0;k<=N;k++) {
        const err=model.minus(acceptedStates[k],refs[k]), w=k===N?Qf:Q;
        for(let j=0;j<ND;j++) if(j<6||j>8) { const a=xi(k,j); addP(a,a,w[j]); q[a]+=w[j]*err[j]; }
        // Residual coordinates live at the reference, while delta is a right/body tangent at the nominal state.
        // Differentiate that chart transition locally, then form the PSD Gauss-Newton attitude block J' W J.
        const Jr=attJac;
        for(let col=0;col<3;col++) {
          attDelta.fill(0); attDelta[6+col]=hAtt; model.plus(acceptedStates[k],attDelta,attPlus);
          attDelta[6+col]=-hAtt; model.plus(acceptedStates[k],attDelta,attMinus);
          model.minus(attPlus,refs[k],errPlus); model.minus(attMinus,refs[k],errMinus);
          for(let row=0;row<3;row++) Jr[row*3+col]=(errPlus[6+row]-errMinus[6+row])/(2*hAtt);
        }
        for(let a=0;a<3;a++) {
          let g=0;
          for(let r=0;r<3;r++) g+=Jr[r*3+a]*w[6+r]*err[6+r];
          q[xi(k,6+a)]+=g;
          for(let b=a;b<3;b++) {
            let h=0;
            for(let r=0;r<3;r++) h+=Jr[r*3+a]*w[6+r]*Jr[r*3+b];
            addP(xi(k,6+a),xi(k,6+b),h);
          }
        }
      }
      for(let k=0;k<N;k++) for(let j=0;j<NU;j++) {
        const a=ui(k,j), c=U[k][j]; addP(a,a,Ru[j]); q[a]+=Ru[j]*c;
        if(k>0) {
          const b=ui(k-1,j), diff=c-U[k-1][j], r=Rs[j];
          addP(a,a,r); addP(b,b,r); addP(b,a,-r); q[a]+=r*diff; q[b]-=r*diff;
        }
      }
      Px.fill(0); for(const [k,v] of pValues) { const p=P.index.get(k); if(p!==undefined) Px[p]=v; }
      Ax.fill(0); l.fill(-Infinity); u.fill(Infinity);
      for(let k=0;k<N;k++) {
        const lin=model.linearize(acceptedStates[k],U[k],dt[k]), nextNom=acceptedStates[k+1], defect=model.minus(lin.next,nextNom), row=k*ND;
        for(let i=0;i<ND;i++) {
          l[row+i]=u[row+i]=defect[i];
          for(let j=0;j<ND;j++) Ax[A.index.get(key(row+i,xi(k,j)))]=-lin.A[i*ND+j];
          Ax[A.index.get(key(row+i,xi(k+1,i)))]=1;
          for(let j=0;j<NU;j++) Ax[A.index.get(key(row+i,ui(k,j)))]=-lin.B[i*NU+j];
        }
      }
      for(let j=0;j<nv;j++) {
        const row=ndyn+j; Ax[A.index.get(key(row,j))]=1;
        if(j<nx) {
          const k=Math.floor(j/ND), a=j%ND, rad=k===0?0:trustX[a]; l[row]=-rad;u[row]=rad;
          if(k>0&&a===12) l[row]=Math.max(l[row],-acceptedStates[k][13]);
          if(k>0&&a>=13) { const valve=acceptedStates[k][14+a-13]; l[row]=Math.max(l[row],-valve);u[row]=Math.min(u[row],1-valve); }
        } else { const s=j-nx,k=Math.floor(s/NU),a=s%NU; l[row]=Math.max(-trustU[a],-U[k][a]);u[row]=Math.min(trustU[a],1-U[k][a]); }
      }
      for(let i=0;i<Px.length;i++) if(!finite(Px[i])) throw new Error(`non-finite QP P value at ${i}`);
      for(let i=0;i<Ax.length;i++) if(!finite(Ax[i])) throw new Error(`non-finite QP A value at ${i}`);
      for(let i=0;i<q.length;i++) if(!finite(q[i])) throw new Error(`non-finite QP q value at ${i}`);
      for(let i=0;i<m;i++) if(l[i]>u[i]) throw new Error(`inconsistent QP bounds at row ${i}: ${l[i]} > ${u[i]}`);
      const solved=solver.solve({Px,Ax,q,l,u,warmStart});
      const status=String(solved?.status ?? '').toLowerCase(), okStatus=['solved','solved inaccurate','optimal','success'].includes(status);
      if(!okStatus||!solved?.x||solved.x.length!==nv||Array.from(solved.x).some(v=>!finite(v))) {
        stop='qp_failure'; diagnostics.push({iteration:it,status:solved?.status??'missing status',accepted:false});
        const error=new Error(`guidance QP failed at SQP iteration ${it}: ${solved?.status??'missing status'}`); error.diagnostics=diagnostics; throw error;
      }
      warmStart=Float64Array.from(solved.x);
      let found=false, alpha=1, candidateCost=currentCost, candidateStates=null, candidateControls=null;
      for(let ls=0;ls<6;ls++,alpha*=0.5) {
        const testU=U.map((stage,k)=>Float64Array.from(stage,(x,j)=>Math.max(0,Math.min(1,x+alpha*solved.x[ui(k,j)]))));
        const testX=rollout(model,x0,testU,dt), c=costOf(model,testX,testU,dt,refs,Q,Qf,Ru,Rs);
        if(c<currentCost-1e-12*Math.max(1,Math.abs(currentCost))) { found=true;candidateCost=c;candidateStates=testX;candidateControls=testU;break; }
      }
      diagnostics.push({iteration:it,status:solved.status,iterations:solved.iterations??null,accepted:found,alpha:found?alpha:0,costBefore:currentCost,costAfter:candidateCost});
      if(!found) { stop='line_search_no_improvement'; break; }
      U.splice(0,U.length,...candidateControls); acceptedStates=candidateStates; currentCost=candidateCost;
    }
    if(iterations===0) stop='zero_iterations';
    return {states:acceptedStates,controls:U,dts:dt,cost:currentCost,
      diagnostics:{qpSolved:diagnostics.every(d=>d.status==='solved'||d.status==='solved inaccurate'),
        stop,iterations:diagnostics.length,acceptedSteps:diagnostics.filter(d=>d.accepted).length,qp:diagnostics}};
  } finally { solver.dispose?.(); }
}
