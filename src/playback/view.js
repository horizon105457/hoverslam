// Everything that shows a soft-body recording at simulation time t: the GPU-deformed car, glass/debris shards,
// dust/sparks/scrape decals, the lattice x-ray view and the two reflection probes. Programs (crash viewer,
// flight viewer, ...) own the camera, UI and timeline and call `apply(t, dt)` before rendering.
import * as THREE from 'three';
import { CarRenderer } from '../render/carRender.js';
import { Shards, Effects } from '../render/effects.js';
import { LatticeView } from '../render/latticeView.js';
import { RecordingPlayer } from '../softbody/recording.js';
import { DECK } from '../scene/layout.js';

export class PlaybackView {
  /**
   * @param ctx    app context (stage)
   * @param car    buildCar output (with render meshes)
   * @param world  buildWorld output
   * @param env    PMREM environment of the sky
   */
  constructor(ctx, { car, world, env }) {
    const stage = (this.stage = ctx.stage);
    this.car = car; this.world = world;
    this.carView = new CarRenderer(car, { envMap: env });
    stage.scene.add(this.carView.group);
    this.carView.attachGTAO(stage.gtao);
    this.shards = new Shards(stage.scene, env);
    this.fx = new Effects(stage.scene, stage);
    this.fx.decal.patch(world.materials.asphalt);
    this.lattice = new LatticeView(stage.scene, car);
    this.player = null;
  }

  /** Use a recording (meta + frames; frames may still be growing while a live run streams in). */
  setRecording(meta, frames) {
    this.player = new RecordingPlayer(this.car, meta, frames);
    this.refreshMeta(meta, true);
    return this.player;
  }
  /** Re-read the per-run data of a growing live recording (cheap except effects: pass fx=false to skip). */
  refreshMeta(meta, fx = true) {
    this.carView.setRunInfo(meta);
    this.shards.setMeta(meta.particles);
    if (fx) this.fx.setEvents(meta.events || {});
  }

  /** Objects left out of reflection probes. Programs may add their own (e.g. thruster plumes). */
  probeHidden() {
    return [this.carView.group, this.lattice.group, this.shards.glass, this.shards.solid, this.fx.dust.mesh, this.fx.sparks.mesh, ...(this.extraProbeHidden || [])];
  }

  /**
   * Two reflection probes for the car: on the deck where it leaves, and on the street where it lands.
   * The car switches between them by height.
   * @param edgeX    x where the car leaves the deck
   * @param landing  THREE.Vector3 ground point where it lands (y ignored)
   */
  placeProbes(edgeX = 2, landing = new THREE.Vector3(-24, 0, 17)) {
    const deck = new THREE.Vector3(edgeX + 4, DECK.top + 0.9, DECK.halfWidth - 3.2);
    const ground = new THREE.Vector3(landing.x, 0.9, landing.z);
    const key = [deck, ground].map(v => v.toArray().map(x => x.toFixed(1)).join(',')).join('|');
    if (key === this._probeKey) return;
    this._probeKey = key;
    const hide = this.probeHidden();
    this.probes?.forEach(p => p.dispose());
    this.probes = [this.stage.captureProbe(deck, hide), this.stage.captureProbe(ground, hide)];
    this._probe = -1;
    this.pickProbe(landing.y || 0);
  }
  pickProbe(bodyY) {
    if (!this.probes) return;
    const i = bodyY > DECK.top * 0.55 ? 0 : 1;
    if (i === this._probe) return;
    this._probe = i;
    const env = this.probes[i].texture;
    this.carView.setEnvMap(env);
    this.shards.glass.material.envMap = env;
  }

  /** Pose everything at simulation time t. Returns the body centre [x, y, z]. */
  apply(t) {
    const P = this.player;
    const pos = P.sample(t), dmg = P.sampleDamage(t);
    this.carView.update(pos, t, dmg);
    this.lattice.update(pos, dmg);
    this.shards.update(P, t);
    this.fx.update(t);
    const c = P.bodyCentre(t);
    this.stage.focus.set(c[0], Math.min(c[1], 4), c[2]);
    this.pickProbe(c[1]);
    return c;
  }
}
