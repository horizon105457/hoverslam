// Shot "drive": the kit car (cold-gas kit of the install program, mounted) speeds down the viaduct towards the
// construction gap, filmed from a chase camera low behind it (first-person feel: close, wide, road streaming
// under the lens). The car is the soft-body render mesh posed rigidly along the drive-in of a launch (FlightLog
// t < 0: straight down the inner lane, then the swerve onto the launch heading), wheels spinning with the
// distance travelled. Film ends just as the car reaches the edge.
import * as THREE from 'three';
import { CarRenderer } from '../../render/carRender.js';
import { FlightLog } from '../flight/timeline.js';
import { Kit } from '../install/kit.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { DECK } from '../../scene/layout.js';
import { quatToMat } from '../../core/quat.js';
import { smooth, smoother, inOutCubic } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

// sim t = t0 + scale * T: slightly undercranked (scale > 1) for speed, ending just short of the deck edge
const SHOT = { total: num('total', 4.2), scale: num('scale', 1.15) };
SHOT.t0 = num('t0', num('tend', 0.5) - SHOT.total * SHOT.scale);

export default class DriveShot {
  constructor(ctx) { this.ctx = ctx; }

  async init() {
    const { stage, ui, assets } = this.ctx;
    ui.loadText('载入天空'); const env = await assets.sky();
    ui.loadText('搭建高架场景'); this.world = assets.world();
    ui.loadText('构建车模'); const car = (this.car = await assets.car());
    const all = await fetch('/opening/ghosts.json').then(r => r.json());
    await this.world.ready;
    // the fastest launch of the ensemble: its drive-in is the path (only t < 0.5 is used)
    const pick = P.get('traj') ? all.find(d => d.id.includes(P.get('traj'))) : all.filter(d => d.success).sort((a, b) => Math.hypot(b.state[0][7], b.state[0][9]) - Math.hypot(a.state[0][7], a.state[0][9]))[0];
    this.d = pick; this.log = new FlightLog(pick);
    this.speed = Math.hypot(pick.state[0][7], pick.state[0][9]);

    this.carView = new CarRenderer(car, { envMap: env });
    stage.scene.add(this.carView.group);
    this.carView.attachGTAO(stage.gtao);
    // kit (body frame) under a group posed like the car
    this.kitRoot = new THREE.Group(); this.kitRoot.matrixAutoUpdate = false;
    this.kit = new Kit({ heroId: 'R_UR' });
    this.kitRoot.add(this.kit.group);
    stage.scene.add(this.kitRoot);
    this.kit.group.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });

    // wheels: rest nodes of each wheel chunk, spun about the wheel centre (body x axis)
    this.wheels = car.wheels.map(w => { const ch = car.chunks[w.chunk]; return { start: ch.nodeStart, count: ch.nodeCount, c: w.centre, R: w.radius }; });
    this.pos = new Float32Array(car.nodes.count * 3);

    // reflections: the deck half way along the run
    const mid = this.log.p(SHOT.t0 + SHOT.total * 0.5);
    const probe = stage.captureProbe(v3(mid[0], DECK.top + 1.0, mid[2]), [this.carView.group, this.kitRoot]);
    this.carView.setEnvMap(probe.texture);
    stage.focus.set(mid[0], DECK.top, mid[2]);
    // a sun glint on the mirror glass bloomed into a lamp for seconds: keep bloom to small highlights here
    stage.bloom.strength = num('bloom', 0.14);
    this.wind = this.world.group.userData.wind;
    this.film = { total: SHOT.total, render: (T, o) => this.render(T, o), describe: () => ({ total: SHOT.total, traj: this.d.id, speed_kmh: +(this.speed * 3.6).toFixed(1), t0: SHOT.t0 }) };
  }

  /** Rigid pose of the lattice at sim time t, wheels rolled by the distance travelled. */
  setScene(t) {
    const L = this.log, p = L.p(t), q = L.q(t), c = L.comB(t), R = quatToMat(q, (this._R ||= new Float64Array(9)));
    const X = this.car.nodes.rest, N = this.car.nodes.count, pos = this.pos;
    const put = (n, x, y, z) => {
      x -= c[0]; y -= c[1]; z -= c[2];
      pos[n * 3] = p[0] + R[0] * x + R[1] * y + R[2] * z; pos[n * 3 + 1] = p[1] + R[3] * x + R[4] * y + R[5] * z; pos[n * 3 + 2] = p[2] + R[6] * x + R[7] * y + R[8] * z;
    };
    for (let n = 0; n < N; n++) put(n, X[n * 3], X[n * 3 + 1], X[n * 3 + 2]);
    const dist = this.speed * (t - SHOT.t0);
    for (const w of this.wheels) {
      const th = dist / w.R, cs = Math.cos(th), sn = Math.sin(th);
      for (let n = w.start; n < w.start + w.count; n++) {
        const y = X[n * 3 + 1] - w.c[1], z = X[n * 3 + 2] - w.c[2];
        put(n, X[n * 3], w.c[1] + y * cs - z * sn, w.c[2] + y * sn + z * cs);
      }
    }
    this.carView.update(pos, t, null);
    // kit: same rigid body transform (x_W = p + R (x_B - comB))
    const Q = new THREE.Quaternion(q[0], q[1], q[2], q[3]);
    const off = v3(c[0], c[1], c[2]).applyQuaternion(Q);
    this.kitRoot.matrix.compose(v3(p[0] - off.x, p[1] - off.y, p[2] - off.z), Q, v3(1, 1, 1));
    this.kitRoot.matrixWorldNeedsUpdate = true;
    this.ctx.stage.focus.set(p[0], DECK.top, p[2]);
    if (this.wind) this.wind.value = t;
  }

  camera(T, t) {
    const L = this.log, dc = P.get('devcam');
    // camera heading = the car's heading a moment ago (yaw only): the camera swings out behind the swerve, so the
    // car visibly turns for the gap. r = the car's right (f x up)
    const lag = num('lag', 0.3), v = L.v(t - lag), f = v3(v[0], 0, v[2]).normalize(), r = v3(-f.z, 0, f.x).negate(), p = v3(...L.p(t));
    const at = (back, side, up) => p.clone().addScaledVector(f, -back).addScaledVector(r, side).add(v3(0, up, 0));
    if (dc) { const a = dc.split(',').map(Number); return { pos: at(a[0], a[1], a[2]), tgt: at(-a[3], a[4], a[5]), fov: a[6] || 60, aperture: 0.002 }; }
    // opening: low rear quarter beside the right rear wheel, swinging in behind the car and up to a tight chase;
    // the camera lags the swerve a little (it swings out), so the gap opens up ahead
    const u = smoother(Math.min(1, T / num('swing', 1.3)));
    const back = num('b0', 2.2) + (num('b1', 4.2) - num('b0', 2.2)) * u;
    const side = num('s0', 1.9) * (1 - u) + num('s1', 0.0) * u;
    const up = num('u0', 0.45) + (num('u1', 1.15) - num('u0', 0.45)) * u;
    const look = at(-num('look', 9), side * 0.25, num('ly', 0.9));
    return { pos: at(back, side, up), tgt: look, fov: num('fov0', 66) + (num('fov1', 60) - num('fov0', 66)) * u, aperture: 0.003, focus: p.clone().add(v3(0, 0.6, 0)) };
  }

  evaluate(T) {
    const t = P.has('devt') ? +P.get('devt') : SHOT.t0 + SHOT.scale * T;
    return { ...this.camera(T, t), t, shake: num('shake', 0.25), roll: 0 };
  }

  render(T, { samples = 16, shutter = 0.5 / 30 } = {}) {
    renderFilmFrame(this.ctx.stage, T, x => this.evaluate(x), t => this.setScene(t), { samples, shutter, aperture: 0.003 });
  }

  preview(T) {
    const ev = this.evaluate(T);
    applyPose(this.ctx.stage.camera, ev);
    this.setScene(ev.t);
    this.ctx.stage.render(1 / 60);
  }
}
