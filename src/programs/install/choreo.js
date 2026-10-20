// Master timeline of the install film and the pure scene pose for film time T. Every animated quantity is a
// function of T only (no state carried between frames), so realtime scrubbing, motion-blur sub-samples and the
// offline exporter all agree. Times are film seconds.
import * as THREE from 'three';
import { clamp01, smooth, smoother, outCubic, inOutCubic, outExpo, outBack, range } from '../../cinema/ease.js';
import { PROPULSION } from '../../core/spec.js';

export const HERO = 'R_UR';
export const LIFT = 0.45; // lift height (m)

export const TL = {
  lightsOn: 0.5, scan: [4.7, 7.4], lift: [5.4, 7.8], markers: 7.0,
  explodeIn: 8.0, labels: [8.9, 10.0], build: 10.0, buildStep: 0.2, buildDur: 0.5,
  insert: [13.3, 14.3], bolts: 14.45, hose: [15.25, 15.95], hosePulse: [15.9, 16.6],
  cutBuild: [9.0, 9.8], cutBuildClose: [13.0, 13.6], cut: [16.3, 17.1], cutClose: [24.3, 25.0], valve: [17.3, 24.0], throttle: [21.4, 22.6], fly: [16.2, 25.2], rams: [4.9, 5.5],
  nine: 25.6, fill: [31.2, 33.0], unscan: [33.1, 34.3], fire: [34.6, 34.95], title: 36.4, end: 40.0,
};

// assembly order of the hero valve stack (valve frame parts fly along the valve axis, rank = outward order)
const VALVE_RANK = { seat: 1, seal: 2, poppet: 3, block: 4, fitting: 4, armature: 5, spring: 6, guide: 7, bobbin: 8, coil: 9, can: 10, band: 10, pole: 11, cap: 12, connector: 13 };
export const BUILD_ORDER = [['seat'], ['block', 'fitting'], ['seal'], ['poppet'], ['armature'], ['spring'], ['guide'], ['bobbin'], ['coil'], ['can', 'band'], ['pole'], ['cap'], ['connector'], ['transducer', 'transducerCap'], ['flange']];
export const buildTime = i => TL.build + i * TL.buildStep;

/** Install start of each non-hero nozzle: bottom row first, then rear, then the sills (staggered). */
export function installTimes(nozzles, heroIndex) {
  const order = { bottom: 0, rear: 1, side: 2 }, t = [];
  const others = nozzles.map((z, i) => ({ z, i })).filter(o => o.i !== heroIndex).sort((a, b) => order[a.z.group] - order[b.z.group] || a.i - b.i);
  let g = -1, k = 0, last = '';
  for (const o of others) { if (o.z.group !== last) { g++; last = o.z.group; } t[o.i] = TL.nine + g * 0.55 + k * 0.14; k++; }
  t[heroIndex] = TL.insert[1];
  return t;
}

export const liftAt = T => LIFT * smoother(range(T, TL.lift[0], TL.lift[1]));
/** Hero poppet lift: opens, throttles down to 35 % and back while the lens looks at the section (it is a
 *  proportional valve), closes at the end of the fly-through. */
export const valveAt = T => {
  if (T < TL.valve[0]) return 0;
  if (T >= TL.valve[1]) return 1 - smooth(range(T, TL.valve[1], TL.valve[1] + 0.2));
  const th = TL.throttle;
  return smooth(range(T, TL.valve[0], TL.valve[0] + 0.45)) * (1 - 0.65 * smooth(range(T, th[0], th[0] + 0.35)) * (1 - smooth(range(T, th[1] - 0.35, th[1]))));
};
/** Hero cutaway wedge angle: open while it is assembled (section view), closed for docking, open again for the fly-through. */
export const cutAt = T => (Math.PI / 2) * (T < TL.cut[0] - 0.5
  ? inOutCubic(range(T, TL.cutBuild[0], TL.cutBuild[1])) * (1 - inOutCubic(range(T, TL.cutBuildClose[0], TL.cutBuildClose[1])))
  : T < TL.cutClose[0] ? inOutCubic(range(T, TL.cut[0], TL.cut[1])) : 1 - inOutCubic(range(T, TL.cutClose[0], TL.cutClose[1])));
export const fillAt = T => smoother(range(T, TL.fill[0], TL.fill[1]));

/** x-ray scan plane (body z): front -> rear ghosts the car, rear -> front restores it. */
export function scanAt(T) {
  const Z = 1.85;
  if (T < TL.unscan[0]) return Z - 2 * Z * inOutCubic(range(T, TL.scan[0], TL.scan[1]));
  return -Z + 2 * Z * inOutCubic(range(T, TL.unscan[0], TL.unscan[1]));
}

/** Studio light levels for the opening "lights on" sequence (bars flicker on one by one). */
export function lightsAt(T) {
  const flick = (t0) => { const t = T - t0; if (t < 0) return 0; if (t < 0.06) return 0.8; if (t < 0.12) return 0.1; if (t < 0.17) return 1; if (t < 0.22) return 0.3; return clamp01(0.6 + (t - 0.22) * 4); };
  const b = TL.lightsOn;
  return {
    bars: [flick(b + 0.0), flick(b + 0.35), flick(b + 0.7)],
    env: smooth(range(T, b + 0.5, b + 2.4)), top: smooth(range(T, b + 0.4, b + 1.4)), rim: smooth(range(T, b + 1.2, b + 2.6)),
    slits: smooth(range(T, b + 1.6, b + 3.2)), ring: T > b + 2.0 && T < b + 4.0 ? (T - b - 2.0) / 2.0 : 0,
  };
}

/**
 * The choreography: owns per-nozzle helper meshes (mount markers, trails) and poses the kit for film time T.
 */
export class Choreo {
  constructor({ kit, studio, xray, rig, flow, jets }) {
    Object.assign(this, { kit, studio, xray, rig, flow, jets });
    const P = PROPULSION;
    this.heroIndex = kit.heroIndex;
    this.tInstall = installTimes(P.nozzles, kit.heroIndex);
    this.markers = kit.nozzles.map(n => this.marker(n));
    this.trails = kit.nozzles.map(n => this.trail(n));
    // exploded layout of the hero (per part: frame axis offset + fly-in scatter)
    const h = kit.hero, k = h.d.rt;
    this.explode = {};
    for (const [name, p] of Object.entries(h.parts)) {
      const r = VALVE_RANK[name];
      let off;
      if (r) off = new THREE.Vector3(name === 'fitting' ? 2.6 * k : 0, r * 2.7 * k, 0);         // valve frame: along the valve axis
      else if (name === 'transducer' || name === 'transducerCap') off = new THREE.Vector3(name === 'transducerCap' ? -4.6 * k : -3.2 * k, 0, 0);
      else if (name === 'flange') off = new THREE.Vector3(0, 4.5 * k, 0);
      else if (name.startsWith('bolt')) off = p.userData.home.clone().setY(0).multiplyScalar(0.6).add(new THREE.Vector3(0, 7 * k, 0));
      else off = new THREE.Vector3();
      const seed = (name.length * 7.31 + (r || 0) * 3.7) % 1;
      // fly-in from far along the frame's main axis, slightly fanned
      const far = off.clone().multiplyScalar(2.6).add(new THREE.Vector3(Math.sin(seed * 20) * 0.08, 0.25 + seed * 0.12, Math.cos(seed * 20) * 0.08));
      this.explode[name] = { off, far, rank: r || 0, t0: TL.explodeIn + (r ? r : name.startsWith('bolt') ? 14 + +name.slice(4) * 0.3 : name === 'nozzle' ? 0 : 6) * 0.07 };
    }
    BUILD_ORDER.forEach((names, i) => names.forEach(n => (this.explode[n].tb = buildTime(i))));
    this.bench = 1.0; // m behind the mount along the exhaust axis (out of the car's shadow for the top shots)
    this._above = new THREE.Vector3(0, 2.4 * k, 0);
  }

  /** Holographic target at a mount: ring + cross, facing along the exhaust. */
  marker(n) {
    const d = n.d, g = new THREE.Group();
    const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color(0x93d3b8).multiplyScalar(2.2), transparent: true, opacity: 1, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false });
    const R = Math.max(d.re, d.rh) * 1.25;
    g.add(new THREE.Mesh(new THREE.RingGeometry(R, R * 1.08, 64), mat));
    g.add(new THREE.Mesh(new THREE.RingGeometry(R * 0.32, R * 0.36, 48), mat));
    for (let i = 0; i < 4; i++) { const t = new THREE.Mesh(new THREE.PlaneGeometry(R * 0.35, R * 0.04), mat); const a = (i * Math.PI) / 2; t.position.set(Math.cos(a) * R * 1.35, Math.sin(a) * R * 1.35, 0); t.rotation.z = a; g.add(t); }
    g.position.copy(n.mount).addScaledVector(n.axis, 0.004);
    g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), n.axis);
    g.traverse(o => { o.userData.noAO = true; o.renderOrder = 2; });
    g.userData.mat = mat; g.visible = false;
    this.kit.group.add(g);
    return g;
  }

  /** Motion streak behind a module flying in along its axis. */
  trail(n) {
    const d = n.d;
    const geo = new THREE.CylinderGeometry(d.re * 0.15, d.re * 0.9, 1, 16, 1, true).translate(0, 0.5, 0);
    const m = new THREE.Mesh(geo, new THREE.ShaderMaterial({
      uniforms: { uAmt: { value: 0 } },
      vertexShader: 'varying float vY; void main() { vY = position.y; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
      fragmentShader: 'uniform float uAmt; varying float vY; void main() { gl_FragColor = vec4(vec3(0.62, 0.92, 0.82) * pow(1.0 - vY, 2.0) * uAmt, 1.0); }',
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false,
    }));
    m.userData.noAO = true; m.visible = false; m.renderOrder = 2;
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), n.axis);
    this.kit.group.add(m);
    return m;
  }

  /** Distance a non-hero module still has to travel along its axis (m) at film time T. */
  travel(i, T) { const u = range(T, this.tInstall[i], this.tInstall[i] + 0.7); return 1.3 * (1 - outExpo(u)); }

  /** Pose everything for film time T. Returns a summary for the HUD. */
  pose(T) {
    const { kit, studio, xray, rig } = this;
    // ---- set: lift, lights
    const L = liftAt(T);
    rig.position.y = L; studio.setLift(L, smooth(range(T, TL.rams[0], TL.rams[1])));
    const lt = lightsAt(T);
    studio.setLights(lt);
    // ---- x-ray
    const scan = scanAt(T);
    const inCar = scan > -1.75 && scan < 1.75;
    xray.set({ scan, dir: 1, amt: 1, sheet: inCar ? 1 : 0, lift: L });
    // ---- hero
    const h = kit.hero, H = this.explode;
    const ins = inOutCubic(range(T, TL.insert[0], TL.insert[1]));
    const bench = this.bench * (1 - ins);
    h.group.position.copy(h.mount).addScaledVector(h.axis, bench);
    h.group.visible = T >= TL.explodeIn;
    for (const [name, p] of Object.entries(h.parts)) {
      const e = H[name];
      if (name.startsWith('bolt')) continue;
      const a = outExpo(range(T, e.t0, e.t0 + 0.8));                           // fly into the exploded layout
      const b = e.tb !== undefined ? outBack(range(T, e.tb, e.tb + TL.buildDur), 1.1) : 0; // collapse
      p.visible = T >= e.t0;
      p.position.copy(e.far).lerp(e.off, a).multiplyScalar(1 - b);
    }
    // bolts: hover in the exploded layout, then (module seated) sweep above their holes and screw down 3 turns
    h.bolts.forEach((bo, j) => {
      const e = H[`bolt${j}`];
      const a = outExpo(range(T, e.t0, e.t0 + 0.8));
      const tb = TL.bolts + j * 0.1, u = outCubic(range(T, tb, tb + 0.5));
      const off = e.far.clone().lerp(e.off, a).lerp(this._above, smooth(range(T, tb - 0.4, tb))).multiplyScalar(1 - u);
      bo.position.copy(bo.userData.home).add(off);
      bo.rotation.y = -(1 - u) * Math.PI * 6;
      bo.visible = T >= e.t0;
    });
    h.setValve(valveAt(T));
    h.bandMat.emissiveIntensity = this.band(this.heroIndex, T);
    // ---- the other nine
    kit.nozzles.forEach((n, i) => {
      const t0 = this.tInstall[i];
      if (i !== this.heroIndex) {
        const D = this.travel(i, T);
        n.group.visible = T >= t0;
        n.group.position.copy(n.mount).addScaledVector(n.axis, D);
        n.bandMat.emissiveIntensity = this.band(i, T);
        n.setValve(T > TL.fire[0] && T < TL.fire[1] + 0.05 ? 1 : 0);
        // streak: length = distance covered over the last 0.12 s
        const tr = this.trails[i], D0 = this.travel(i, T - 0.12), len = D0 - D;
        tr.visible = T >= t0 && len > 0.01;
        if (tr.visible) { tr.position.copy(n.mount).addScaledVector(n.axis, D + n.d.Le); tr.scale.set(1, len * 1.6, 1); tr.material.uniforms.uAmt.value = Math.min(1, len * 4); }
      }
      // mount markers: from after the scan until the module arrives
      const mk = this.markers[i], arrive = i === this.heroIndex ? TL.insert[1] : t0 + 0.45;
      const mAmt = smooth(range(T, TL.markers + i * 0.05, TL.markers + 0.4 + i * 0.05)) * (1 - smooth(range(T, arrive - 0.15, arrive)));
      const dim = i === this.heroIndex || T > TL.nine - 0.6 ? 1 : 0.35;
      mk.visible = mAmt > 0.01; mk.userData.mat.opacity = mAmt * dim * (0.75 + 0.25 * Math.sin(T * 9 + i));
      // feed line: grows from the nozzle port to the manifold after the module seats, then a pulse runs home
      const g0 = i === this.heroIndex ? TL.hose[0] : t0 + 0.6, g1 = i === this.heroIndex ? TL.hose[1] : t0 + 1.05;
      const grow = smoother(range(T, g0, g1));
      const p0 = g1, p1 = g1 + 0.5, pu = range(T, p0, p1);
      let pulse = -1, amt = 0, w = 0.05;
      if (T > g0 && T < g1) { pulse = grow; amt = 0.6; w = 0.012; } // a short light at the growing tip
      else if (T >= p0 && T < p1 + 0.15) { pulse = pu; amt = 1 - smooth(range(T, p1, p1 + 0.15)); }
      // pressurisation: a slow wave along every line; test fire: all lines flash
      const fill = fillAt(T);
      if (T > TL.fill[0] && T < TL.fill[1] + 0.3) { pulse = 1 - ((T - TL.fill[0]) * 0.9 + i * 0.07) % 1; amt = Math.max(amt, 0.55); }
      if (T > TL.fire[0] - 0.05 && T < TL.fire[1] + 0.2) { pulse = 1 - range(T, TL.fire[0] - 0.05, TL.fire[1]); amt = 1; }
      kit.setHose(i, grow, pulse, amt, fill > 0 && fill < 1 ? 0.08 : w);
    });
    // ---- manifold -> bottles handshake after each connection, then the fill
    let hp = -1, hpAmt = 0;
    const arrivals = kit.nozzles.map((n, i) => (i === this.heroIndex ? TL.hose[1] : this.tInstall[i] + 1.05) + 0.5);
    for (const a of arrivals) if (T > a && T < a + 0.6) { hp = range(T, a, a + 0.6); hpAmt = 1 - range(T, a + 0.45, a + 0.6); }
    const fill = fillAt(T);
    if (T > TL.fill[0] && T < TL.fill[1]) { hp = ((T - TL.fill[0]) * 0.8) % 1; hpAmt = 0.7; }
    kit.setHP(hp, hpAmt);
    const bottleFlash = arrivals.some(a => T > a + 0.55 && T < a + 0.95) ? 0.7 : 0;
    kit.setTank(fill, Math.max(fill > 0 ? 1 - 0.5 * fill : 0, bottleFlash * 0.4), fill > 0 && fill < 1 ? fill : -1);
    kit.setGauge(PROPULSION.tank.p0 * fill);
    // ---- hero cutaway + flow (need world matrices of the posed rig)
    rig.updateMatrixWorld(true);
    kit.setCut(cutAt(T), h.group.matrixWorld);
    // ---- studio uniforms (the floor ring pulses on beats)
    let ring = lt.ring;
    for (const t of [TL.insert[1], ...this.tInstall.filter((_, i) => i !== this.heroIndex).map(t => t + 0.45)]) if (T > t && T < t + 1.2 && ring === 0) ring = (T - t) / 1.2 * 0.6;
    if (T > TL.fire[0] && T < TL.fire[0] + 1.6) ring = (T - TL.fire[0]) / 1.6;
    // top-down exploded view / assembly: the floor is the backdrop, keep it a quiet black
    const floor = 1 - 0.85 * smooth(range(T, TL.explodeIn - 0.2, TL.explodeIn + 0.4)) * (1 - smooth(range(T, TL.insert[0], TL.insert[1])));
    studio.update(T, { ring, floor });
    return { L, scan, fill };
  }

  /** Status band glow of nozzle i: lights when its feed line is connected, breathes while pressurising. */
  band(i, T) {
    const con = (i === this.heroIndex ? TL.hose[1] : this.tInstall[i] + 1.05) + 0.4;
    let b = T > con ? 1.2 + 2.4 * Math.exp(-(T - con) * 5) : 0;
    if (T > TL.fill[0] && T < TL.fill[1] + 0.5) b += 0.8 * (0.5 + 0.5 * Math.sin(T * 6 + i));
    if (T > TL.fire[0] && T < TL.fire[1] + 0.3) b += 3;
    return b;
  }
}
