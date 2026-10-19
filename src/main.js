import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { bakeCarMeshes, buildCar } from './car/carPrep.js';
import { Stage } from './render/stage.js';
import { buildWorld } from './scene/world.js';
import { CarRenderer } from './render/carRender.js';
import { Shards, Effects } from './render/effects.js';
import { LatticeView } from './render/latticeView.js';
import { RecordingPlayer, unpackRecording } from './sim/recording.js';
import { Director } from './director.js';
import { PRESETS, DECK } from './scene/layout.js';
import { CrashAudio } from './audio.js';

const params = new URLSearchParams(location.search);
const $ = id => document.getElementById(id);
const NAMES = {
  body: '车身', door_L: '左车门', door_R: '右车门', hood: '引擎盖', hatch: '尾门', fender_front_L: '左前翼子板', fender_front_R: '右前翼子板',
  bumper_front: '前保险杠', bumper_rear: '后保险杠', mirror_L: '左后视镜', mirror_R: '右后视镜', seat_D: '主驾座椅', seat_P: '副驾座椅',
  windshield: '前风挡', glass_dL: '左车窗', glass_dR: '右车窗', glass_qL: '左三角窗', glass_qR: '右三角窗', glass_rear: '后挡玻璃',
  wheel_FL: '左前轮', wheel_FR: '右前轮', wheel_RL: '左后轮', wheel_RR: '右后轮',
  lamp_L: '左大灯', lamp_R: '右大灯', tail_L: '左尾灯', tail_R: '右尾灯', stop: '高位刹车灯', mirror_gL: '左后视镜片', mirror_gR: '右后视镜片',
  repeater_L: '左侧转向灯', repeater_R: '右侧转向灯', fog: '雾灯',
};
const loadText = t => { $('load-text').textContent = t; };
const DECK_DROP = 9.2;

class App {
  async init() {
    const stage = (this.stage = new Stage($('viewport'), { preserve: params.has('capture'), pixelRatio: params.has('pr') ? +params.get('pr') : Math.min(devicePixelRatio, 2) }));
    if (params.has('w')) { stage.renderer.setPixelRatio(1); stage.resize(+params.get('w'), +params.get('h')); }
    loadText('载入天空');
    const env = await stage.loadSky('/assets/hdri/sky_4k.hdr');
    loadText('搭建高架场景');
    this.world = buildWorld(stage.scene, { renderer: stage.renderer });
    const worldReady = this.world.ready;
    loadText('载入车模');
    const gltf = await new GLTFLoader().loadAsync('/assets/car-model.glb');
    loadText('构建软体晶格');
    await new Promise(r => setTimeout(r, 20));
    const car = (this.car = buildCar(bakeCarMeshes(gltf.scene)));
    this.carView = new CarRenderer(car, { envMap: env });
    stage.scene.add(this.carView.group);
    this.carView.attachGTAO(stage.gtao);
    this.shards = new Shards(stage.scene, env);
    this.fx = new Effects(stage.scene, stage);
    this.fx.decal.patch(this.world.materials.asphalt);
    this.lattice = new LatticeView(stage.scene, car);
    $('techline').textContent = `XPBD 节点-梁晶格 · ${(car.nodes.count / 1e4).toFixed(1)} 万节点 · ${(car.beams.count / 1e4).toFixed(1)} 万梁 · 塑性屈服 · 断裂 · GPU 四面体嵌入`;
    this.controls = new OrbitControls(stage.camera, stage.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enabled = false;
    this.mode = params.get('mode') || 'director';
    this.speed = 1; this.playing = !params.has('paused');
    this.T = +(params.get('T') || 0);
    this.simT = 0;
    this.audio = new CrashAudio();
    this.setupUI();
    const settled = await fetch('/assets/settled.bin').then(r => r.arrayBuffer());
    this.settled = new Float32Array(settled, 8);
    await worldReady;
    loadText('载入预计算结果');
    await this.loadPreset(params.get('preset') || 'roof');
    $('loading').classList.add('done');
    this.ready = true;
    this.last = performance.now();
    if (!params.has('capture')) this.loop();
  }

  // ------------------------------------------------------------------ data
  async loadPreset(name) {
    this.cancelRun();
    this.presetName = name;
    document.querySelectorAll('#presets button').forEach(b => b.classList.toggle('on', b.dataset.preset === name));
    try {
      const res = await fetch(`/sims/${name}.sim`);
      if (!res.ok) throw new Error('missing');
      const stream = res.body.pipeThrough(new DecompressionStream('gzip'));
      const buf = await new Response(stream).arrayBuffer();
      const { meta, frames } = unpackRecording(buf);
      this.useRecording(meta, frames);
    } catch (e) {
      console.warn('no precomputed run for', name, e);
      this.runScenario({ ...PRESETS[name], label: PRESETS[name].label });
    }
  }

  useRecording(meta, frames, live = false) {
    this.player = new RecordingPlayer(this.car, meta, frames);
    this.live = live;
    this.carView.setRunInfo(meta);
    this.shards.setMeta(meta.particles);
    this.fx.setEvents(meta.events || {});
    this.director = !live && frames.length > 2 ? new Director(this.player) : null;
    this.placeProbes();
    this.T = live ? 0 : this.T;
    this.buildTicker(meta);
    this.buildMarks();
  }

  runScenario(sc) {
    this.cancelRun();
    const scenario = { strength: 1, recordHz: 100, duration: 7, substepHz: 8000, ...sc };
    const worker = (this.worker = new Worker(new URL('./sim/worker.js', import.meta.url), { type: 'module' }));
    const c = this.car;
    const car = { h: c.h, nodes: c.nodes, chunks: c.chunks, pieces: c.pieces, beams: c.beams, tets: c.tets, groups: c.groups.map(g => ({ ...g })), wheels: c.wheels, render: { tetCount: c.render.tetCount, tetNodes: c.render.tetNodes } };
    const meta = { N: c.nodes.count, C: c.chunks.length, recordHz: scenario.recordHz, scenario, events: {}, particles: { count: 0, kind: [], spawn: [], size: [] } };
    const frames = [];
    $('progress').hidden = false;
    this.useRecording(meta, frames, true);
    this.mode === 'director' && this.setMode('follow');
    worker.onmessage = e => {
      const m = e.data;
      if (m.type === 'frames') {
        frames.push(...m.frames);
        if (!meta.launch && frames.length) meta.launch = null;
        Object.assign(meta, { groupTime: m.events.groupTime, tearTime: m.events.tearTime, groups: m.events.groups, particles: m.events.particles, events: { dust: m.events.dust, sparks: m.events.sparks, scrape: m.events.scrape, breaks: m.events.breaks } });
        this.carView.setRunInfo(meta);
        this.shards.setMeta(meta.particles);
        if ((this._fxTick = (this._fxTick || 0) + 1) % 4 === 0) this.fx.setEvents(meta.events);
        this.buildTicker(meta);
        $('progress').querySelector('i').style.width = `${(m.progress * 100).toFixed(1)}%`;
        $('progress').querySelector('span').textContent = `计算中 ${(m.progress * scenario.duration).toFixed(2)} / ${scenario.duration} s`;
      } else if (m.type === 'done') {
        Object.assign(meta, m.meta);
        this.useRecording(meta, frames, false);
        $('progress').hidden = true;
        this.worker.terminate(); this.worker = null;
        this.T = 0; this.setMode('director');
      }
    };
    worker.postMessage({ type: 'run', car, settled: this.settled, scenario });
  }
  /**
   * Two reflection probes for the car: on the deck where it leaves, and on the street where it lands
   * (the director's impact point when known). The car switches between them by height.
   */
  placeProbes() {
    const d = this.director;
    const edgeX = d ? this.player.bodyCentre(d.tEdge)[0] : 2;
    const deck = new THREE.Vector3(edgeX + 4, DECK.top + 0.9, DECK.halfWidth - 3.2);
    const ground = d ? new THREE.Vector3(d.L.x, 0.9, d.L.z) : new THREE.Vector3(-24, 0.9, 17);
    const key = [deck, ground].map(v => v.toArray().map(x => x.toFixed(1)).join(',')).join('|');
    if (key === this._probeKey) return;
    this._probeKey = key;
    const hide = [this.carView.group, this.lattice.group, this.shards.glass, this.shards.solid, this.fx.dust.mesh, this.fx.sparks.mesh];
    this.probes?.forEach(p => p.dispose());
    this.probes = [this.stage.captureProbe(deck, hide), this.stage.captureProbe(ground, hide)];
    this._probe = -1;
    this.pickProbe(d ? d.L.y : 0);
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
  cancelRun() { if (this.worker) { this.worker.terminate(); this.worker = null; $('progress').hidden = true; } }

  // ------------------------------------------------------------------ UI
  setupUI() {
    const pre = $('presets');
    for (const [k, p] of Object.entries(PRESETS)) {
      const b = document.createElement('button');
      b.dataset.preset = k;
      b.innerHTML = `${p.label}<small>${p.speed} km/h</small>`;
      b.onclick = () => this.loadPreset(k);
      pre.appendChild(b);
    }
    for (const id of ['speed', 'angle', 'pitch', 'roll', 'yaw', 'strength']) {
      const i = $(`i-${id}`), o = $(`o-${id}`);
      i.oninput = () => { o.textContent = id === 'strength' ? (+i.value).toFixed(2) : i.value; };
    }
    $('b-random').onclick = () => {
      const r = (a, b) => a + Math.random() * (b - a);
      const set = (id, v) => { $(`i-${id}`).value = v; $(`i-${id}`).oninput(); };
      set('speed', Math.round(r(55, 95))); set('angle', Math.round(r(10, 24))); set('pitch', r(-2, 2).toFixed(2)); set('roll', r(-3.2, 1).toFixed(2)); set('yaw', r(-1.2, 1.2).toFixed(2));
    };
    $('b-run').onclick = () => {
      const v = id => +$(`i-${id}`).value;
      document.querySelectorAll('#presets button').forEach(b => b.classList.remove('on'));
      this.runScenario({ label: '自定义', speed: v('speed'), angle: v('angle'), spin: [v('pitch'), v('yaw'), v('roll')], strength: v('strength'), seed: Math.floor(Math.random() * 1e6) });
    };
    $('b-play').onclick = () => { this.playing = !this.playing; $('b-play').textContent = this.playing ? '❚❚' : '▶'; };
    document.querySelectorAll('#modes button').forEach(b => (b.onclick = () => this.setMode(b.dataset.mode)));
    document.querySelectorAll('#speeds button').forEach(b => (b.onclick = () => { this.speed = +b.dataset.speed; document.querySelectorAll('#speeds button').forEach(x => x.classList.toggle('on', x === b)); }));
    $('b-heat').onclick = () => { const on = !this.carView.uniforms.uHeat.value; this.carView.uniforms.uHeat.value = on ? 1 : 0; $('b-heat').classList.toggle('on', on); };
    $('b-lattice').onclick = () => { this.lattice.group.visible = !this.lattice.group.visible; this.carView.group.visible = !this.lattice.group.visible; $('b-lattice').classList.toggle('on', this.lattice.group.visible); };
    $('b-ui').onclick = () => document.body.classList.toggle('clean');
    $('b-sound').onclick = async () => { if (this.audio.enabled) this.audio.stop(); else await this.audio.start(); $('b-sound').classList.toggle('on', this.audio.enabled); };
    let idleTimer = 0;
    const wake = () => { document.body.classList.remove('idle'); clearTimeout(idleTimer); idleTimer = setTimeout(() => document.body.classList.add('idle'), 4000); };
    addEventListener('pointermove', wake); addEventListener('pointerdown', wake); wake();
    addEventListener('keydown', e => { if (e.key === 'h' || e.key === 'H') document.body.classList.toggle('clean'); if (e.key === ' ') $('b-play').click(); });
    const tl = $('timeline');
    const seek = e => {
      const r = tl.getBoundingClientRect(), u = Math.min(1, Math.max(0, (e.clientX - r.left) / r.width));
      if (this.mode === 'director' && this.director) this.T = u * this.director.total; else this.simT = this.timeRange()[0] + u * (this.timeRange()[1] - this.timeRange()[0]);
    };
    tl.onpointerdown = e => { seek(e); tl.setPointerCapture(e.pointerId); tl.onpointermove = seek; };
    tl.onpointerup = () => { tl.onpointermove = null; };
    this.setMode(this.mode);
    if (params.has('kiosk')) {
      document.body.classList.add('kiosk');
      addEventListener('pointerdown', () => { if (!this.audio.enabled) this.audio.start().then(() => $('b-sound').classList.add('on')); }, { once: true });
    }
  }

  setMode(mode) {
    this.mode = mode;
    document.querySelectorAll('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === mode));
    this.controls.enabled = mode === 'free';
    document.body.classList.toggle('cinema', mode === 'director');
    if (mode !== 'director' && this.lastEval) this.simT = this.lastEval.t;
    if (mode === 'free' && this.player?.ready) { const c = this.player.bodyCentre(this.simT); this.controls.target.set(c[0], c[1], c[2]); }
    this.buildMarks();
  }

  timeRange() { return [-3, this.player?.duration || 7]; }

  buildMarks() {
    const box = $('t-marks'); box.innerHTML = '';
    if (!this.player?.ready) return;
    const ev = this.player.meta.events?.breaks || [];
    const toU = t => {
      if (this.mode === 'director' && this.director) { const s = this.director.shots.find(s => t >= Math.min(s.t0, s.t1) && t <= Math.max(s.t0, s.t1)); return s ? (s.start + s.dur * (t - s.t0) / (s.t1 - s.t0)) / this.director.total : null; }
      const [a, b] = this.timeRange(); return (t - a) / (b - a);
    };
    for (const e of ev) { const u = toU(e.t); if (u === null) continue; const s = document.createElement('span'); s.className = e.type === 'detach' ? '' : 'glass'; s.style.left = `${(u * 100).toFixed(2)}%`; box.appendChild(s); }
  }

  buildTicker(meta) {
    const list = (meta.events?.breaks || []).filter(e => e.type === 'detach' || e.type === 'glass' || e.type === 'crack' || e.type === 'lens');
    this.tickerEvents = list.map(e => ({ t: e.t, kind: e.type === 'detach' ? 'detach' : 'glass', text: e.type === 'detach' ? `${NAMES[e.piece] || e.piece} 脱落` : e.type === 'crack' ? '前风挡 龟裂' : `${NAMES[e.group] || e.group} 碎裂` }));
    const ol = $('ticker'); ol.innerHTML = '';
    for (const e of this.tickerEvents) { const li = document.createElement('li'); li.className = e.kind; li.innerHTML = `${e.text}<span>${e.t.toFixed(2)}s</span>`; ol.appendChild(li); e.el = li; }
  }

  updateHUD(t) {
    if (!this.player?.ready) return;
    const meta = this.player.meta;
    let shown = 0;
    for (const e of this.tickerEvents || []) { const on = e.t <= t; e.el.classList.toggle('show', on); if (on) shown++; }
    const ev = meta.events?.breaks || [];
    const det = ev.filter(e => e.type === 'detach' && e.t <= t).length;
    const glass = ev.filter(e => e.type !== 'detach' && e.t <= t).length;
    const parts = meta.particles ? meta.particles.spawn.filter(s => s <= t).length : 0;
    const tear = meta.tearTime ? countBelow(meta.tearTime, t) : 0;
    const c = this.player.bodyCentre(t);
    const html = [['仿真时间', `${t.toFixed(3)} s`], ['车身高度', `${c[1].toFixed(2)} m`], ['脱落部件', det], ['破碎玻璃/灯罩', glass], ['碎片', parts], ['撕裂单元', tear]].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
    if (html !== this._hud) { $('stats').innerHTML = html; this._hud = html; }
  }

  // ------------------------------------------------------------------ frame
  /** Advance and render one frame. dt = wall seconds. */
  frame(dt) {
    const stage = this.stage, cam = stage.camera;
    if (!this.player?.ready) { stage.render(dt); return; }
    let t, shake = 0;
    if (this.mode === 'director' && this.director) {
      if (this.playing) this.T += dt * this.speed;
      // kiosk: after the full cut, show the end card and move on to the next pose
      if (params.has('kiosk') && this.T >= this.director.total && !this._advancing) {
        this._advancing = true;
        this.showEndCard();
        const keys = Object.keys(PRESETS), next = keys[(keys.indexOf(this.presetName) + 1) % keys.length];
        setTimeout(async () => { await this.loadPreset(next); this.T = 0; this._advancing = false; $('endcard').classList.remove('show'); }, 6000);
      }
      if (this._advancing) this.T = Math.min(this.T, this.director.total - 1e-3);
      const ev = (this.lastEval = this.director.evaluate(this.T));
      t = ev.t; shake = ev.shake;
      cam.position.copy(ev.pos); cam.fov = ev.fov; cam.updateProjectionMatrix();
      cam.lookAt(ev.tgt);
      if (this._shot !== ev.shot) { this._shot = ev.shot; $('shot').innerHTML = `镜头 ${this.director.shots.indexOf(ev.shot) + 1}<b>${ev.shot.name}</b>`; }
      const u = (this.T % this.director.total) / this.director.total;
      $('t-fill').style.width = `${u * 100}%`; $('t-head').style.left = `${u * 100}%`;
    } else {
      const [a, b] = this.timeRange();
      const maxT = this.live ? this.player.duration : b;
      if (this.playing) this.simT += dt * this.speed;
      if (this.simT > maxT) this.simT = this.live ? maxT : a;
      t = this.simT;
      const c = this.player.bodyCentre(t);
      if (this.mode === 'follow') {
        const tgt = new THREE.Vector3(c[0], c[1] * 0.6 + 0.4, c[2]);
        this._follow = this._follow || cam.position.clone();
        const desired = tgt.clone().add(new THREE.Vector3(-6.5, 3.2, 7.5));
        this._follow.lerp(desired, 1 - Math.exp(-dt * 3));
        cam.position.copy(this._follow); cam.fov = 45; cam.updateProjectionMatrix(); cam.lookAt(tgt);
      } else {
        this.controls.target.lerp(new THREE.Vector3(c[0], c[1], c[2]), 1 - Math.exp(-dt * 2));
        this.controls.update();
      }
      $('shot').innerHTML = this.live ? '实时计算<b>边算边看</b>' : '';
      const u = (t - a) / (b - a);
      $('t-fill').style.width = `${u * 100}%`; $('t-head').style.left = `${u * 100}%`;
    }
    this.renderAt(t, dt, shake);
    const rate = this._prevT !== undefined && dt > 0 ? (t - this._prevT) / dt : this.speed;
    this._prevT = t;
    this.audio.update(t, rate, this.player.meta, this.player.bodyCentre(t)[1]);
  }

  renderAt(t, dt, shake = 0) {
    const stage = this.stage, P = this.player;
    const pos = P.sample(t);
    const dmg = P.sampleDamage(t);
    this.carView.update(pos, t, dmg);
    this.lattice.update(pos, dmg);
    this.shards.update(P, t);
    this.fx.update(t);
    const c = P.bodyCentre(t);
    stage.focus.set(c[0], Math.min(c[1], 4), c[2]);
    this.pickProbe(c[1]);
    const f = stage.finish.uniforms;
    f.uShake.value.set((Math.random() - 0.5) * shake * 0.01, (Math.random() - 0.5) * shake * 0.01);
    if (this.world.group.userData.wind) this.world.group.userData.wind.value += dt;
    $('t-label').textContent = `${t.toFixed(2)} s`;
    this.updateHUD(t);
    stage.render(dt);
  }

  /**
   * Offline film frame at director time T: temporal (shutter), lens (aperture) and sub-pixel jitter,
   * averaged over `samples` renders. Used by tools/film.mjs.
   */
  renderFilm(T, { samples = 16, shutter = 0.5 / 30, aperture = 0.035, fps = 30 } = {}) {
    const stage = this.stage, cam = stage.camera, d = this.director;
    const w = stage.renderer.domElement.width, h = stage.renderer.domElement.height;
    const golden = 2.399963;
    let ev0 = d.evaluate(T);
    this.updateFilmHUD(ev0);
    stage.renderAccumulated((k, n) => {
      const ev = d.evaluate(T + (k / n - 0.5) * shutter);
      const t = ev.t;
      const P = this.player, pos = P.sample(t), dmg = P.sampleDamage(t);
      this.carView.update(pos, t, dmg); this.shards.update(P, t); this.fx.update(t);
      const c = P.bodyCentre(t); stage.focus.set(c[0], Math.min(c[1], 4), c[2]); this.pickProbe(c[1]);
      // lens: Vogel disk sample, re-aimed at the focus target
      const rr = aperture * Math.sqrt((k + 0.5) / n), th = k * golden;
      cam.fov = ev.fov; cam.position.copy(ev.pos); cam.lookAt(ev.tgt); cam.updateMatrixWorld();
      const right = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0), up = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 1);
      cam.position.addScaledVector(right, Math.cos(th) * rr).addScaledVector(up, Math.sin(th) * rr);
      if (ev.shake) cam.position.add(new THREE.Vector3(Math.sin(T * 91) * 0.02, Math.cos(T * 77) * 0.02, 0).multiplyScalar(ev.shake));
      cam.lookAt(ev.tgt);
      cam.setViewOffset(w, h, ((k * 0.618) % 1) - 0.5, ((k * 0.382 + 0.5) % 1) - 0.5, w, h);
      cam.updateProjectionMatrix();
      if (this.world.group.userData.wind) this.world.group.userData.wind.value = T;
    }, samples);
    cam.clearViewOffset();
  }
  showEndCard() {
    const meta = this.player.meta, ev = meta.events?.breaks || [];
    const det = ev.filter(e => e.type === 'detach').map(e => NAMES[e.piece] || e.piece);
    const glass = ev.filter(e => e.type === 'glass' || e.type === 'lens' || e.type === 'crack').length;
    const label = PRESETS[this.presetName]?.label || meta.label || '';
    $('endcard').innerHTML = `<h3>${label}</h3><p>${meta.scenario?.speed ?? ''} km/h · 坠落 ${DECK_DROP} m</p>
      <div class="nums"><div><b>${det.length}</b><span>部件脱落</span></div><div><b>${glass}</b><span>玻璃/灯罩碎裂</span></div><div><b>${meta.particles?.count ?? 0}</b><span>碎片</span></div><div><b>${(meta.tearTime || []).filter(t => t < 1e8).length}</b><span>金属撕裂单元</span></div></div>
      <p class="parts">${det.join(' · ')}</p>`;
    $('endcard').classList.add('show');
  }
  updateFilmHUD(ev) {
    if (this._shot !== ev.shot) { this._shot = ev.shot; $('shot').innerHTML = `镜头 ${this.director.shots.indexOf(ev.shot) + 1}<b>${ev.shot.name}</b>`; }
    $('t-label').textContent = `${ev.t.toFixed(2)} s`;
    this.updateHUD(ev.t);
  }

  loop() {
    const now = performance.now();
    const dt = Math.min(0.05, (now - this.last) / 1000);
    this.last = now;
    this.frame(dt);
    requestAnimationFrame(() => this.loop());
  }
}
function countBelow(arr, t) { let n = 0; for (let i = 0; i < arr.length; i++) if (arr[i] <= t) n++; return n; }

const app = new App();
window.app = app;
app.init().catch(e => { console.error(e); loadText('出错：' + e.message); });
