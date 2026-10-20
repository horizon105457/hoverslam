// Program interface. A program is one self-contained experience on the shared Stage: the crash viewer, the
// thrust-assisted flight viewer, the thruster-install motion design piece, ... Programs live in
// src/programs/<id>/index.js (default export = class extending Program) and are picked with ?program=<id>.
//
// Contract (the shell and tools/film/*.mjs rely on it):
//   constructor(ctx)    no heavy work
//   async init()        load assets, build the scene and UI; report progress with ctx.ui.loadText()
//   frame(dt)           realtime: advance by dt wall seconds and render (ctx.stage.render(dt))
//   film                null, or { total, render(T, {samples, shutter, fps}), describe() } for offline rendering:
//                       render(T) must be a pure function of film time T (deterministic, no wall clock)
//   dispose()           optional
export class Program {
  static meta = { id: 'program', title: '' };
  constructor(ctx) { this.ctx = ctx; this.film = null; }
  async init() {}
  frame(dt) { this.ctx.stage.render(dt); }
  dispose() {}
}
