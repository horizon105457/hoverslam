// Runs a coldgas-policy@1 file (spec/README.md section 7): observation normalisation + MLP, deterministic.
// Forward pass y_j = (sum_i W[j][i] x_i, sequential from 0) + b_j; tanh via fmath (bit-identical to Python).
import { NU, POLICY_FORMAT } from '../contract.js';
import { Observer } from '../obs.js';
import { tanh } from '../fmath.js';

const ACT = { tanh, relu: x => (x > 0 ? x : 0), linear: x => x };
const OUT = { clip01: a => (a > 1 ? 1 : a < 0 ? 0 : a), tanh01: a => 0.5 * (tanh(a) + 1) };

export function mlpForward(policy, x) {
  let h = x;
  for (const L of policy.layers) {
    const f = ACT[L.act || 'linear'], y = new Float64Array(L.b.length);
    for (let j = 0; j < y.length; j++) {
      const row = L.W[j];
      let s = 0;
      for (let i = 0; i < h.length; i++) s += row[i] * h[i];
      y[j] = f(s + L.b[j]);
    }
    h = y;
  }
  return h;
}

export function createMLP({ policy } = {}) {
  if (!policy || policy.format !== POLICY_FORMAT) throw new Error(`mlp: need a ${POLICY_FORMAT} object (config.policy)`);
  const O = policy.obs, clip = O.clip ?? 10, out = OUT[policy.action?.transform || 'clip01'];
  let obs = null;
  return {
    name: 'mlp', config: { task: policy.task, meta: policy.meta },
    reset({ task } = {}) {
      obs = new Observer(task);
      if (obs.size !== O.mean.length) throw new Error(`mlp: policy expects ${O.mean.length} observations, task gives ${obs.size}`);
    },
    act(s) {
      const o = obs.observe(s), x = new Float64Array(o.length);
      for (let i = 0; i < o.length; i++) { const z = (o[i] - O.mean[i]) / O.std[i]; x[i] = z > clip ? clip : z < -clip ? -clip : z; }
      const a = mlpForward(policy, x), u = new Float64Array(NU);
      for (let i = 0; i < NU; i++) u[i] = out(a[i]);
      return s.armed ? u : new Float64Array(NU);
    },
  };
}
