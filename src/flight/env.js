// Gymnasium-shaped episode wrapper around the rigid twin, same API and semantics as python/coldgas/env.py:
//   const env = new ColdGasEnv('escape-soft-landing', {seed: 0});   // {vehicle}: twin parameter override (fitting)
//   let [obs, info] = env.reset();                     // reset(seed?, {preset?})
//   [obs, reward, terminated, truncated, info] = env.step(u);   // u: NU valve commands, or NA (+ brake, steer)
// Observation: obs.js; reward/termination/metrics: tasks.js (spec/README.md section 8).
import { RigidTwin } from './twin.js';
import { loadTask, sampleInitial, TaskTracker } from './tasks.js';
import { Observer } from './obs.js';
import { NU, NA } from './contract.js';

export class ColdGasEnv {
  constructor(task, { seed = 0, preset = null, vehicle = null } = {}) {
    this.task = typeof task === 'string' || !task.initial ? loadTask(task) : task;
    this.twin = new RigidTwin({ arm: this.task.arm || 'start', vehicle });
    this.tracker = new TaskTracker(this.task);
    this.observer = new Observer(this.task);
    this.seed = seed; this.preset = preset;
    this.nu = NU; this.na = NA; this.obsSize = this.observer.size; // valves / full action incl. drive (5.13)
  }
  reset(seed = this.seed, options = {}) {
    this.seed = seed;
    const init = (this.init = options.init || sampleInitial(this.task, seed, { preset: options.preset ?? this.preset }));
    this.twin.reset(init);
    this.tracker.reset(this.twin);
    this.observer.reset();
    return [this.observer.observe(this.twin.state), this.info()];
  }
  step(u) {
    this.twin.step(u);
    const { reward, terminated, truncated, reason } = this.tracker.update(this.twin);
    const info = this.info();
    if (terminated || truncated) { info.reason = reason; info.metrics = this.tracker.metrics(this.twin); }
    return [this.observer.observe(this.twin.state), reward, terminated, truncated, info];
  }
  get state() { return this.twin.state; }
  info() { const t = this.twin; return { t: t.k * t.dt, phase: t.phase, armed: t.armed, preset: this.init?.preset ?? null, seed: this.seed }; }
}
