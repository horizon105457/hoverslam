// A shot list on a film timeline. Film time T (seconds of finished film) maps to simulation time t per shot;
// by default 1 s of film = 1 s of simulation (the project's rule: no slow motion unless a shot asks for it via
// `dur`/`warp`). Shots may overlap in simulation time (replays). Everything is a pure function of T, so the
// same Sequence drives realtime playback and the deterministic offline film renderer (./film.js).
//
// Shot = { name, t0, t1, cam(u, t) -> CamPose, dur?, warp?(u) -> w }
//   u in [0,1] = progress through the shot on the film timeline, t = t0 + (t1 - t0) * warp(u)
// CamPose = { pos: Vector3, tgt: Vector3, fov, roll? (rad, dutch angle), shake? (0..1),
//             focus?: Vector3 (DOF focus point, default tgt), aperture? (m, film render lens radius) }

export class Sequence {
  /** @param {Array<{name:string, t0:number, t1:number, cam:Function, dur?:number, warp?:Function}>} shots */
  constructor(shots) {
    let acc = 0;
    this.shots = shots.map(s => { const o = { ...s, dur: s.dur ?? s.t1 - s.t0, start: acc }; acc += o.dur; return o; });
    this.total = acc;
  }

  shotAt(T) {
    let shot = this.shots[0];
    for (const s of this.shots) if (T >= s.start) shot = s;
    return shot;
  }

  /** @returns {{t:number, shot:object, u:number, pos, tgt, fov:number, shake:number, roll:number, focus?, aperture?}} */
  evaluate(T) {
    T = ((T % this.total) + this.total) % this.total;
    const shot = this.shotAt(T);
    const u = Math.min(1, (T - shot.start) / shot.dur);
    const w = shot.warp ? shot.warp(u) : u;
    const t = shot.t0 + (shot.t1 - shot.t0) * w;
    const c = shot.cam(u, t);
    return { ...c, t, shot, u, shake: c.shake || 0, roll: c.roll || 0 };
  }

  /** Timeline position (0..1) of the first shot that shows simulation time t, or null. Linear shots only. */
  timelineU(t) {
    const s = this.shots.find(s => t >= Math.min(s.t0, s.t1) && t <= Math.max(s.t0, s.t1));
    return s ? (s.start + s.dur * (t - s.t0) / (s.t1 - s.t0)) / this.total : null;
  }

  describe() {
    return { total: this.total, shots: this.shots.map(s => [s.name, +s.start.toFixed(2), +s.dur.toFixed(2), +s.t0.toFixed(2), +s.t1.toFixed(2)]) };
  }
}

/** Point the camera for realtime playback (the film renderer has its own lens model). */
export function applyPose(camera, pose) {
  camera.position.copy(pose.pos);
  camera.fov = pose.fov;
  camera.updateProjectionMatrix();
  camera.lookAt(pose.tgt);
  if (pose.roll) camera.rotateZ(pose.roll);
}
