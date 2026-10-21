// Program "install": motion-design film of the cold-gas thruster kit being installed on the MINI EV, after the
// Tesla Roadster "Rocket" trim (10 SpaceX cold-gas thrusters fed from COPV air bottles). Dark assembly bay,
// x-ray scan of the car, the hero nozzle (R_UR) assembled from an exploded view, docked and torqued, a camera
// fly-through of its insides (exit cone -> throat -> plenum -> poppet seat -> solenoid -> out), the other nine
// installed at once, the bank pressurised to its fill pressure and a test fire. Every position, direction and
// size comes from spec/propulsion.json; thrust numbers from the reference propulsion model.
//
// Film: `film.render(T)` is a pure function of film time T (choreo.js); realtime playback uses the same pose.
// Params: T=<film s> (start time), paused, nohud (no HUD / labels: clean plates for a compositor, 直到用计算机模拟 00 B3).
import * as THREE from 'three';
import { Program } from '../../app/program.js';
import { Transport } from '../../app/transport.js';
import { Sequence, applyPose } from '../../cinema/sequence.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { smooth, outCubic, range } from '../../cinema/ease.js';
import { CarRenderer } from '../../render/carRender.js';
import { Propulsion } from '../../flight/propulsion.js';
import { PROPULSION } from '../../core/spec.js';
import { Studio } from './studio.js';
import { Kit } from './kit.js';
import { XRay } from './xray.js';
import { HeroFlow, Jets, machFromArea, speedOf, tempOf } from './gas.js';
import { boreRadius } from './nozzle.js';
import { Hud, COL } from './hud.js';
import { Choreo, HERO, TL, BUILD_ORDER, buildTime, valveAt } from './choreo.js';
import { buildShots } from './shots.js';

const MONO = '"SF Mono", "Menlo", monospace';
const PART_LABELS = {
  nozzle: ['拉瓦尔喷管', 'DE LAVAL NOZZLE'], seat: ['阀座', 'VALVE SEAT'], block: ['阀体 · 进气腔', 'VALVE BODY · GALLERY'], fitting: ['进气接头', 'INLET FITTING'],
  can: ['电磁铁壳体', 'SOLENOID CAN'], band: ['状态灯环', 'STATUS RING'], guide: ['导向管', 'GUIDE TUBE'], pole: ['极靴', 'POLE PIECE'], bobbin: ['线圈骨架', 'BOBBIN'],
  coil: ['电磁线圈', 'COIL'], armature: ['衔铁', 'ARMATURE'], poppet: ['提升阀', 'POPPET'], seal: ['PTFE 密封', 'PTFE SEAL'], spring: ['回位弹簧', 'RETURN SPRING'],
  cap: ['端盖', 'END CAP'], connector: ['电连接器', 'CONNECTOR'], transducer: ['压力传感器', 'PRESSURE TRANSDUCER'], transducerCap: ['', ''], flange: ['安装法兰', 'MOUNTING FLANGE'],
};
const GROUP_ZH = { bottom: '底部', rear: '尾部', side: '侧面' };

export default class InstallProgram extends Program {
  static meta = { id: 'install', title: '安装 · 冷气喷口' };

  async init() {
    const { stage, ui, assets, params } = this.ctx;
    ui.setTitle(InstallProgram.meta.title);
    ui.loadText('搭建装配间');
    this.studio = new Studio(stage);
    ui.loadText('载入车模');
    const car = await assets.car();
    this.carView = new CarRenderer(car, { envMap: this.studio.env });
    this.carView.showRest();
    // AO G-buffer: the car through its lattice shader, double-sided (the rear hatch/bumper triangles are wound
    // inwards; a front-sided normal pass culls them and the AO of the kit behind prints through the panel)
    this._aoSide = stage.gtao.normalMaterial.side;
    stage.gtao.normalMaterial.side = THREE.DoubleSide;
    this.carView.attachGTAO(stage.gtao);
    this.rig = new THREE.Group(); this.rig.name = 'rig';
    this.rig.add(this.carView.group);
    stage.scene.add(this.rig);
    ui.loadText('生成冷气推进套件');
    this.xray = new XRay(this.carView, this.rig);
    this.kit = new Kit({ heroId: HERO });
    this.rig.add(this.kit.group);
    this.flow = new HeroFlow(this.kit.hero);

    this.jets = new Jets(this.kit);
    stage.scene.add(this.flow.mesh, this.jets.mesh);
    this.choreo = new Choreo({ kit: this.kit, studio: this.studio, xray: this.xray, rig: this.rig, flow: this.flow, jets: this.jets });
    this.numbers();
    this.seq = new Sequence(buildShots(this.kit));
    this.hud = new Hud(stage);
    // UI: transport only (the film carries its own HUD in the canvas)
    this.T = +(params.get('T') || 0);
    this.playing = !params.has('paused');
    this.speed = 1;
    const tr = (this.transport = new Transport(ui, {
      onPlay: () => { this.playing = !this.playing; tr.setPlaying(this.playing); },
      onSeek: u => { this.T = u * this.seq.total; },
    }));
    tr.setPlaying(this.playing);
    tr.addGroup('<button data-s="0.25">¼×</button><button data-s="1" class="on">1×</button><button id="b-ui" title="隐藏界面 (H)">⤢</button>', 'speeds');
    document.querySelectorAll('#speeds button[data-s]').forEach(b => (b.onclick = () => { this.speed = +b.dataset.s; document.querySelectorAll('#speeds button[data-s]').forEach(x => x.classList.toggle('on', x === b)); }));
    ui.$('b-ui').onclick = () => document.body.classList.toggle('clean');
    addEventListener('keydown', e => { if (e.key === ' ') ui.$('b-play').click(); if (e.key === 'h' || e.key === 'H') document.body.classList.toggle('clean'); });
    tr.setMarks(this.seq.shots.map(s => ({ u: s.start / this.seq.total, cls: 'edge' })));
    this.ctx.stage.camera.far = 80;
    // compile everything once (all states visible) so playback does not hitch on first use
    ui.loadText('编译着色器');
    for (const T of [2, 6, 12, 19, 27, 33, 34.7]) this.choreo.pose(T);
    this.kit.group.traverse(o => { o.userData._v = o.visible; o.visible = true; });
    this.flow.mesh.visible = this.jets.mesh.visible = true;
    stage.renderer.compile(stage.scene, stage.camera);
    this.kit.group.traverse(o => { o.visible = o.userData._v; });
    this.film = {
      total: this.seq.total,
      render: (T, opts) => this.renderFilm(T, opts),
      describe: () => this.seq.describe(),
    };
  }

  /** Physical numbers for the callouts: nominal thrust/mass flow per nozzle from the reference model. */
  numbers() {
    const p = new Propulsion(), u = new Float64Array(p.n).fill(1);
    for (let i = 0; i < 120; i++) p.step(u, 0.001); // valves fully open (tau_open = 15 ms), tank barely touched
    this.thrust = Array.from(p.thrust); this.mdot = Array.from(p.mdot);
    this.sumF = this.thrust.reduce((a, b) => a + b, 0);
    this.isp = this.sumF / this.mdot.reduce((a, b) => a + b, 0) / 9.80665;
    this.Me = p.Me[this.kit.heroIndex];
    this.pc = p.pc;
    this.vol = PROPULSION.tank.bottles.reduce((s, b) => s + b.volume, 0);
  }

  // ------------------------------------------------------------------ scene at film time T
  /** Pose the scene for film time T and the camera lens settings of the pose (near/far, AO scale). */
  apply(T, ev) {
    const sum = this.choreo.pose(T);
    const world = this.rig.matrixWorld;
    const hw = this.kit.hero.group.matrixWorld;
    const inFly = T > TL.valve[0] - 0.2 && T < TL.valve[1] + 1.2;
    this.flow.update(T, valveAt, hw, { amt: inFly ? 1 : 0, near: ev?.near ?? 0.05 });
    this.jets.update(T, TL.fire[0], TL.fire[1], world, 0);
    const cam = this.ctx.stage.camera, near = ev?.near ?? 0.05;
    cam.near = near; cam.far = Math.min(80, Math.max(4, near * 20000));
    // GTAO works in world units: shrink its radius for macro shots, off inside the nozzle
    // fly-through: the open cut lets the top light into the polished bore, whose delta-light glint blooms;
    // dim the direct lights for the whole move (the environment keeps the metal readable)
    // (same for the exploded view and assembly close-ups: a delta-light glint on polished parts is a white blob)
    const close = smooth(range(T, TL.explodeIn - 0.3, TL.explodeIn + 0.3)) * (1 - smooth(range(T, TL.fly[1] - 0.8, TL.fly[1])));
    // (the diffuse fill carries the macro shots instead: no glints, the dark parts stay readable; inside the bore the
    // environment is pulled back too, polished walls all around the lens mirror it at full strength)
    const dimL = 1 - 0.85 * close, fly = smooth(range(T, TL.fly[0] + 0.3, TL.fly[0] + 1.0)) * (1 - smooth(range(T, TL.fly[1] - 0.8, TL.fly[1])));
    this.ctx.stage.sun.intensity *= dimL; this.studio.rim.intensity *= dimL; this.studio.rim2.intensity *= dimL;
    this.ctx.stage.scene.environmentIntensity *= (1 + 0.25 * close) * (1 - 0.5 * fly); this.studio.fill.intensity *= 1 + 1.2 * close;
    const g = this.ctx.stage.gtao;
    // AO only for the solid car (the ghost shell would print its AO onto the kit inside) and not at macro scale
    g.enabled = near > 0.0015 && this.xray.state.solidAll;

    const r = THREE.MathUtils.clamp(near * 12, 0.02, 0.6);
    if (r !== this._aoR) { g.updateGtaoMaterial({ radius: r }); this._aoR = r; }
    // test-fire shake (handled by renderFilmFrame via ev.shake; realtime adds it in frame())
    return sum;
  }

  shake(T) { return T > TL.fire[0] ? Math.exp(-(T - TL.fire[0]) * 5) * 0.35 : 0; }

  // ------------------------------------------------------------------ realtime
  frame(dt) {
    const stage = this.ctx.stage;
    if (this.playing) this.T = (this.T + dt * this.speed) % this.seq.total;
    const T = this.T, ev = this.seq.evaluate(T);
    this.apply(T, ev);
    applyPose(stage.camera, ev);
    const sh = this.shake(T);
    if (sh) stage.camera.position.add(new THREE.Vector3(Math.sin(T * 91) * 0.02, Math.cos(T * 77) * 0.02, 0).multiplyScalar(sh));
    stage.finish.uniforms.uTime.value = T % 100;
    stage.render(dt);
    if (!this.ctx.params.has('nohud')) this.drawHud(T, ev); // opening film: clean plates (own typography on top)
    this.transport.setProgress(T / this.seq.total);
    this.transport.setLabel(`${T.toFixed(2)} s`);
  }

  // ------------------------------------------------------------------ film
  renderFilm(T, { samples = 16, shutter = 0.5 / 30 } = {}) {
    const stage = this.ctx.stage, seq = this.seq;
    stage.finish.uniforms.uTime.value = T % 100;
    const evalT = x => { const e = seq.evaluate(Math.min(seq.total - 1e-4, Math.max(0, x))); e.shake = this.shake(x); return e; };
    renderFilmFrame(stage, T, evalT, (t, T0, ev) => this.apply(t, ev), { samples, shutter });
    const ev = evalT(T);
    if (!this.ctx.params.has('nohud')) this.drawHud(T, ev); // opening film: clean plates (own typography on top)
    this.transport.setProgress(T / seq.total);
  }

  // ------------------------------------------------------------------ HUD script
  drawHud(T, ev) {
    const hud = this.hud, kit = this.kit, h = kit.hero, d = h.d, P = PROPULSION;
    hud.begin(ev);
    const shot = ev.shot, idx = this.seq.shots.indexOf(shot) + 1, n = this.seq.shots.length;
    const intro = smooth(range(T, 1.6, 2.6)), outro = 1 - smooth(range(T, TL.end - 0.9, TL.end - 0.2));
    const chrome = intro * (T < TL.title + 0.3 ? 1 : 1 - smooth(range(T, TL.title, TL.title + 0.6)));
    hud.frame(chrome);
    hud.header({ step: `${String(idx).padStart(2, '0')} / ${String(n).padStart(2, '0')}  ${shot.en}`, T, alpha: chrome });
    const capIn = smooth(range(T, shot.start + 0.15, shot.start + 0.6)) * chrome;
    if (T < TL.title) hud.caption(shot.name, shot.en, capIn);
    const B = (x, y, z) => new THREE.Vector3(x, y, z).applyMatrix4(this.rig.matrixWorld);
    const fmtkN = i => `${(this.thrust[i] / 1000).toFixed(1)} kN`;
    // ---- opening: kit summary
    if (T > 2.2 && T < 4.6) {
      const p = range(T, 2.2, 3.4), a = 1 - smooth(range(T, 4.2, 4.6));
      hud.block(hud.W - 58 * hud.k, hud.H * 0.42, [
        { t: '10 × 冷气喷口', px: 30, weight: 300, spacing: 3 },
        { t: `6 × COPV · ${PROPULSION.tank.p0 / 1e6} MPa · ${Math.round(this.vol * 1000)} L`, px: 13, fam: MONO, color: COL.dim },
        { t: `ΣF ${(this.sumF / 1000).toFixed(1)} kN · Isp ${this.isp.toFixed(0)} s`, px: 13, fam: MONO, color: COL.dim },
      ], p, { align: 'right', alpha: a });
    }
    // ---- x-ray: COPV bank, regulator, mount targets
    if (T > 6.6 && T < 8.4) {
      const a = 1 - smooth(range(T, 7.9, 8.4));
      const bank = kit.bank, c = bank.getCenter(new THREE.Vector3());
      hud.callout(B(-0.25, bank.max.y, c.z), { p: range(T, 6.6, 7.6), side: -1, tag: 'COPV', len: 200, alpha: a, lines: [{ t: '碳纤维气瓶 × 6' }, { t: `TYPE IV · ${P.tank.p0 / 1e6} MPa · ${Math.round(this.vol * 1000)} L · 空气 ${(this.vol * P.tank.p0 / (P.gas.R * P.tank.T0)).toFixed(0)} kg`, fam: MONO }] });
      hud.callout(B(kit.manifold.x, kit.manifold.y, kit.manifold.z + 0.03), { p: range(T, 7.0, 8.0), side: -1, up: 1, dy: 40, tag: 'REG', len: 170, alpha: a, accent: COL.ice, lines: [{ t: '调压阀 · 歧管' }, { t: `${P.regulator.p_set / 1e6} MPa 出口`, fam: MONO }] });
      kit.nozzles.forEach((nz, i) => {
        if (i === kit.heroIndex) return;
        const at = B(...nz.spec.mount);
        hud.callout(at, { p: range(T, 7.2 + i * 0.06, 7.9 + i * 0.06), side: nz.spec.mount[0] < 0 ? -1 : 1, dx: 18, dy: 18, len: 54, tag: '', alpha: a * 0.9, lines: [{ t: nz.spec.id, px: 11, fam: MONO, weight: 600 }] });
      });
    }
    // ---- exploded view: part callouts
    if (T > TL.labels[0] && T < TL.build + 0.9) {
      const a = 1 - smooth(range(T, TL.build + 0.2, TL.build + 0.9));
      const at = (name, x, y, z = 0) => h.parts[name].children[0].localToWorld(new THREE.Vector3(x, y, z));
      const list = [
        ['nozzle', at('nozzle', 0, d.Le * 0.6), 1, -1, `ε ${d.eps} · 喉径 Ø${(d.rt * 2000).toFixed(1)} mm`],
        ['poppet', at('poppet', 0, d.vA + d.seatH + 0.3 * d.rt), -1, -1, `升程 ${(d.lift * 1000).toFixed(1)} mm · 帘面积 ${(2 * Math.PI * d.rs * d.lift / (Math.PI * d.rt * d.rt)).toFixed(2)} A*`],
        ['coil', at('coil', 0, (d.vB + d.vC) / 2, d.rv * 0.8), 1, -1, `τ 开 ${P.valve.tau_open * 1000} ms · 关 ${P.valve.tau_close * 1000} ms`],
        ['armature', at('armature', 0, d.vP - d.lift - d.rt), -1, 1, '软磁衔铁 + 提升阀杆'],
        ['spring', at('spring', 0, d.vP), 1, 1, '常闭 NORMALLY CLOSED'],
        ['transducer', at('transducer', -d.rh - 1.5 * d.rt, d.yPc), -1, 1, '稳压腔压力 p_c'],
        ['flange', at('flange', d.rv * 0.8, d.tf), 1, 1, '6 × M5 · 不锈钢'],
        ['block', at('block', d.rv, d.vG), 1, -1, `进气 Ø${(d.rPort * 2000).toFixed(0)} mm`],
      ];
      list.forEach(([name, p3, side, up, val], i) => {
        const [zh, en] = PART_LABELS[name];
        hud.callout(p3, { p: range(T, TL.labels[0] + i * 0.1, TL.labels[0] + 0.7 + i * 0.1), side, up, dx: 34, dy: 30, len: 150, tag: en, alpha: a, lines: [{ t: zh, px: 15 }, { t: val, fam: MONO, px: 11 }] });
      });
    }
    // ---- assembly checklist
    if (T > TL.build - 0.3 && T < TL.fly[0] + 0.4) {
      const a = smooth(range(T, TL.build - 0.3, TL.build + 0.2)) * (1 - smooth(range(T, TL.fly[0], TL.fly[0] + 0.4)));
      const x = hud.W - 58 * hud.k, y0 = hud.H * 0.26, k = hud.k;
      hud.block(x, y0 - 26 * k, [{ t: 'ASSEMBLY SEQUENCE · 装配序列', px: 11, fam: MONO, weight: 600, color: COL.mint, spacing: 2 }], 1, { align: 'right', alpha: a });
      const steps = [...BUILD_ORDER.map((names, i) => ({ t: buildTime(i) + 0.45, label: PART_LABELS[names[0]] })), { t: TL.insert[1], label: ['插入安装位', 'INSERT'] }, { t: TL.bolts + 1.0, label: ['螺栓紧固', 'TORQUE'] }, { t: TL.hose[1], label: ['连接供气', 'FEED LINE'] }];
      steps.forEach((s, i) => {
        const done = T >= s.t, active = !done && (i === 0 || T >= steps[i - 1].t);
        const col = done ? COL.dim : active ? COL.hot : 'rgba(236,242,240,0.25)';
        hud.block(x, y0 + i * 19 * k, [{ t: `${s.label[0]}  ${s.label[1]}  ${done ? '■' : active ? '▶' : '□'}`, px: 11.5, fam: MONO, color: col }], 1, { align: 'right', alpha: a });
      });
      // the active part: reticle on its landing point
      BUILD_ORDER.forEach((names, i) => {
        const t0 = buildTime(i), u = range(T, t0 + 0.25, t0 + 0.7);
        if (u > 0 && u < 1) hud.reticle(this.partCentre(names[0]), 1 - u, 20, COL.hot);
      });
    }
    // ---- seat & torque
    if (T > TL.insert[1] - 0.1 && T < TL.fly[0] + 0.3) {
      const a = 1 - smooth(range(T, TL.fly[0] - 0.2, TL.fly[0] + 0.3));
      const mnt = B(...h.spec.mount);
      hud.reticle(mnt, range(T, TL.insert[1] - 0.1, TL.insert[1] + 0.5), 34, COL.mint);
      hud.callout(mnt, { p: range(T, TL.insert[1], TL.insert[1] + 0.8), side: -1, up: -1, dx: 60, dy: 70, len: 210, tag: h.spec.id, alpha: a, lines: [{ t: `${h.spec.label} · 已就位`, px: 16 }, { t: `${fmtkN(kit.heroIndex)} · ṁ ${this.mdot[kit.heroIndex].toFixed(1)} kg/s · 螺栓 6 N·m`, fam: MONO }] });
      if (T > TL.hose[0]) hud.callout(kit.hoses[kit.heroIndex].curve.getPointAt(0.25).applyMatrix4(this.rig.matrixWorld), { p: range(T, TL.hose[0], TL.hose[0] + 0.8), side: 1, up: 1, dx: 40, dy: 40, len: 170, tag: 'FEED', alpha: a, accent: COL.ice, lines: [{ t: '不锈钢编织软管', px: 14 }, { t: `Ø${(d.rHose * 2000).toFixed(0)} mm · ${P.regulator.p_set / 1e6} MPa`, fam: MONO }] });
    }
    // ---- fly-through: live flow readout at the camera + station plates + section schematic
    if (T > TL.fly[0] + 0.6 && T < TL.fly[1] - 0.3) this.flyHud(T, ev);
    // ---- nine at once
    if (T > TL.nine - 0.2 && T < 31.3) {
      const a = 1 - smooth(range(T, 30.8, 31.3));
      let done = 1;
      kit.nozzles.forEach((nz, i) => {
        if (i === kit.heroIndex) return;
        const t0 = this.choreo.tInstall[i] + 0.45;
        if (T >= t0) done++;
        const at = B(...nz.spec.exit);
        const side = nz.spec.mount[0] < 0 ? -1 : 1, up = nz.spec.group === 'bottom' ? 1 : -1;
        hud.reticle(at, range(T, t0 - 0.05, t0 + 0.4), 18, COL.mint);
        const dy = 26 + ((i * 7) % 4) * 26, len = 96 + ((i * 5) % 3) * 30;
        hud.callout(at, { p: range(T, t0, t0 + 0.7), side, up, dx: 20 + ((i * 3) % 3) * 12, dy, len, tag: nz.spec.id, alpha: a, lines: [{ t: nz.spec.label, px: 13 }, { t: fmtkN(i), fam: MONO, color: COL.mint }] });
      });
      hud.meter(0, 'INSTALLED · 已安装', `${done} / ${kit.nozzles.length}`, done / kit.nozzles.length, { alpha: smooth(range(T, TL.nine - 0.2, TL.nine + 0.3)) * a });
    }
    // ---- pressurise + test fire
    if (T > TL.fill[0] - 0.4 && T < TL.title + 0.6) {
      const a = smooth(range(T, TL.fill[0] - 0.4, TL.fill[0])) * (1 - smooth(range(T, TL.title, TL.title + 0.6)));
      const f = Math.max(0, Math.min(1, (T - TL.fill[0]) / (TL.fill[1] - TL.fill[0])));
      const fill = f * f * f * (f * (f * 6 - 15) + 10);
      hud.meter(2, 'COPV · 气瓶压力', `${(P.tank.p0 * fill / 1e6).toFixed(1)} MPa`, fill / 1.0, { color: COL.ice, alpha: a });
      hud.meter(1, 'REGULATED · 调压', `${(Math.min(P.regulator.p_set, Math.max(0, P.tank.p0 * fill - P.regulator.dp_min)) / 1e6).toFixed(1)} MPa`, Math.min(1, P.tank.p0 * fill / P.regulator.p_set), { color: COL.mint, alpha: a });
      const fire = T > TL.fire[0] && T < TL.fire[1] + 0.8;
      hud.meter(0, 'ΣF · 总推力', fire ? `${(this.sumF / 1000).toFixed(1)} kN` : '0.0 kN', fire ? (T < TL.fire[1] ? 1 : 1 - range(T, TL.fire[1], TL.fire[1] + 0.3)) : 0, { color: COL.hot, alpha: a });
      if (T > 32.0) kit.nozzles.forEach((nz, i) => {
        // once the body is solid again, only label nozzles whose exit faces the lens (the car hides the rest)
        const ex = B(...nz.spec.exit), toCam = ev.pos.clone().sub(ex).normalize();
        if (this.xray.state.solidAll && toCam.dot(nz.axis) < -0.2) return;
        const side = nz.spec.mount[0] < 0 ? -1 : 1, up = nz.spec.group === 'bottom' ? 1 : -1;
        hud.callout(B(...nz.spec.exit), { p: range(T, 32.0 + i * 0.07, 32.7 + i * 0.07), side, up, dx: 14 + ((i * 3) % 3) * 10, dy: 18 + ((i * 7) % 4) * 18, len: 70, tag: '', alpha: a * 0.95, lines: [{ t: `${nz.spec.id} ${fmtkN(i)}`, px: 10.5, fam: MONO, weight: 600 }] });
      });
      if (T > TL.fire[0] - 0.4) {
        const p = range(T, TL.fire[0] - 0.4, TL.fire[0] + 0.2);
        hud.block(hud.W / 2, hud.H * 0.16, [{ t: 'STATIC TEST FIRE · 点火试喷', px: 13, fam: MONO, weight: 600, color: COL.hot, spacing: 3 }, { t: `${((TL.fire[1] - TL.fire[0]) * 1000).toFixed(0)} ms · 10 / 10 · Isp ${this.isp.toFixed(0)} s`, px: 12, fam: MONO, color: COL.dim }], p, { align: 'center', alpha: a });
      }
    }
    // ---- title card
    if (T > TL.title + 0.3) {
      const p = range(T, TL.title + 0.3, TL.title + 1.6), a = outro;
      const g = hud.g, k = hud.k, cx = hud.W / 2, cy = hud.H * 0.2;
      g.save(); g.globalAlpha = a;
      const w = 520 * k * outCubic(range(T, TL.title + 0.3, TL.title + 1.1));
      g.fillStyle = COL.mint; g.fillRect(cx - w / 2, cy + 22 * k, w, 1.5 * k);
      g.restore();
      hud.block(cx, cy, [{ t: 'COLD GAS · 10 THRUSTERS', px: 46, weight: 300, spacing: 10, lh: 1.6 }], p, { align: 'center', alpha: a });
      hud.block(cx, cy + 66 * k, [{ t: '冷气推进 · 10 喷口', px: 26, weight: 300, spacing: 14, color: COL.line }], range(T, TL.title + 0.8, TL.title + 1.8), { align: 'center', alpha: a });
      hud.block(cx, cy + 112 * k, [{ t: `4 × 底部 ${fmtkN(0)} · 4 × 尾部 ${fmtkN(4)} · 2 × 侧面 ${fmtkN(8)} · 6 × COPV ${P.tank.p0 / 1e6} MPa`, px: 12, fam: MONO, color: COL.dim, spacing: 1.5 }], range(T, TL.title + 1.3, TL.title + 2.4), { align: 'center', alpha: a });
    }
    hud.bars(0, Math.max(1 - smooth(range(T, 0.0, 0.6)) * 1, 1 - outro));
    hud.end();
  }

  /** World-space centre of a hero part (bounding sphere of its mesh), for reticles. */
  partCentre(name) {
    const m = this.kit.hero.parts[name].children[0];
    m.geometry.boundingSphere || m.geometry.computeBoundingSphere();
    return m.geometry.boundingSphere.center.clone().applyMatrix4(m.matrixWorld);
  }

  /** Fly-through HUD: local Mach / speed / static temperature where the camera is, plus a section map. */
  flyHud(T, ev) {
    const hud = this.hud, h = this.kit.hero, d = h.d, k = hud.k;
    const a = smooth(range(T, TL.fly[0] + 0.6, TL.fly[0] + 1.2)) * (1 - smooth(range(T, TL.fly[1] - 0.9, TL.fly[1] - 0.3)));
    // camera in the module frame
    const inv = new THREE.Matrix4().copy(h.group.matrixWorld).invert();
    const c = ev.pos.clone().applyMatrix4(inv);
    // region + local flow state
    const At = Math.PI * d.rt * d.rt;
    let region, M = 0;
    const onAxis = Math.hypot(c.x, c.z) < d.rc * 1.2;
    if (c.y > d.Le + 0.02) region = ['喷管外', 'OUTSIDE'];
    else if (onAxis && c.y > d.yT) { region = ['扩张段', 'DIVERGENT · SUPERSONIC']; const r = boreRadius(d, this.flow.contour, c.y); M = machFromArea((Math.PI * r * r) / At, true); }
    else if (onAxis && c.y > d.yC) { region = ['收敛段 · 喉部', 'CONVERGENT · THROAT']; const r = boreRadius(d, this.flow.contour, c.y); M = machFromArea((Math.PI * r * r) / At, false); }
    else if (c.x < d.vA && onAxis) { region = ['稳压腔', 'PLENUM']; M = machFromArea((d.rc * d.rc) / (d.rt * d.rt), false); }
    else if (c.x < d.vA + d.seatH + d.lift * 2) { region = ['阀座 · 提升阀', 'SEAT · POPPET CURTAIN']; M = machFromArea((2 * Math.PI * d.rs * d.lift) / At, false); }
    else if (c.x < d.vC + 0.03) { region = ['比例电磁阀', 'PROPORTIONAL SOLENOID VALVE']; M = 0; }
    else region = ['喷管外', 'OUTSIDE'];
    const v = speedOf(M), Ts = tempOf(M) - 273.15, open = this.kit.hero.valveS;
    const x = 58 * k, y = hud.H * 0.30;
    hud.block(x, y, [
      { t: 'FLOW · 1:1000 慢放', px: 11, fam: MONO, weight: 600, color: COL.mint, spacing: 2 },
      { t: region[0], px: 24, weight: 400, spacing: 3, lh: 1.25 },
      { t: region[1], px: 11, fam: MONO, weight: 600, color: COL.dim, spacing: 2, lh: 2.4 },
      { t: `MACH  ${M.toFixed(2)}`, px: 15, fam: MONO, weight: 600 },
      { t: `V     ${v.toFixed(0)} m/s`, px: 15, fam: MONO, weight: 600 },
      { t: `T     ${Ts.toFixed(0)} °C`, px: 15, fam: MONO, weight: 600, color: Ts < -100 ? COL.ice : COL.line },
      { t: `阀门  ${(open * 100).toFixed(0)} %`, px: 15, fam: MONO, weight: 600, color: COL.hot },
    ], 1, { alpha: a });
    // section map: half contour of the nozzle + valve outline, camera dot
    const g = hud.g, W = 300 * k, H = 150 * k, x0 = hud.W - 58 * k - W, y0 = hud.H * 0.30 - 20 * k;
    const yMin = d.yBot - 0.005, yMax = d.Le + 0.02, s = W / (yMax - yMin + d.vC * 0.0 + 0.0);
    const map = (yy, rr) => [x0 + (yy - yMin) * s, y0 + H * 0.5 - rr * s];
    g.save(); g.globalAlpha = a;
    g.strokeStyle = COL.faint; g.lineWidth = 1 * k; g.strokeRect(x0 - 8 * k, y0 - 8 * k, W + 16 * k, H + 16 * k);
    g.strokeStyle = COL.line; g.lineWidth = 1.3 * k;
    for (const sg of [1, -1]) { g.beginPath(); this.flow.contour.forEach(([r, yy], i) => { const [px, py] = map(yy, sg * r); i ? g.lineTo(px, py) : g.moveTo(px, py); }); g.stroke(); }
    // valve (side branch, drawn upwards in the map)
    g.strokeStyle = COL.dim;
    const [vx0, vy0] = map(d.yPc - d.rv, d.vA), [vx1, vy1] = map(d.yPc + d.rv, Math.min(d.vC, (y0 + H * 0.5 - (y0 - 4 * k)) / s));
    g.strokeRect(vx0, Math.min(vy0, vy1), vx1 - vx0, Math.abs(vy1 - vy0));
    const [cx, cy] = map(THREE.MathUtils.clamp(c.y, yMin, yMax), THREE.MathUtils.clamp(Math.max(Math.hypot(c.z, c.x) * (c.x >= 0 ? 1 : -1), -d.re), -d.re * 1.2, (H * 0.5 + 4 * k) / s));
    g.fillStyle = COL.hot; g.beginPath(); g.arc(cx, cy, 4 * k, 0, Math.PI * 2); g.fill();
    g.strokeStyle = COL.hot; g.beginPath(); g.arc(cx, cy, 9 * k, 0, Math.PI * 2); g.stroke();
    hud.type('SECTION · 剖面', x0 - 8 * k, y0 - 16 * k, 1, { px: 10.5, fam: MONO, weight: 600, color: COL.dim, spacing: 2 });
    g.restore();
  }

  dispose() { this.ctx.stage.gtao.normalMaterial.side = this._aoSide; this.studio.dispose(); }
}
