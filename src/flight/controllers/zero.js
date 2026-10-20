// Baseline: thrusters never fire (the original crash).
import { NU } from '../contract.js';
export function createZero() {
  return { name: 'zero', config: {}, reset() {}, act() { return new Float64Array(NU); } };
}
