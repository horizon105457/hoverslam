// Shot "recon": the photos become a 3D model. In the dark assembly bay of the install film (its Studio, lights
// off): camera frustums of the capture positions ring the empty floor, each with its picture; a sparse point
// cloud is triangulated out of them (rays flicker from the frustums), then the dense Gaussian splats grow in,
// blurry first and sharpening like a training run, and a scan plane turns the splats into the solid car.
// The last frame is the first frame of the install film (same camera, lights still off), so the edit can cut
// straight into its lights-on.
//
// Splats: samples of the car's render surfaces (area-weighted), colour = material albedo with baked soft studio
// shading (a captured splat bakes its lighting). Drawn as screen-facing Gaussian sprites with stochastic
// transparency: each film sub-sample keeps a fragment with probability alpha (hashed threshold re-seeded per
// sample), so the accumulated frame is order-independent alpha blending without sorting a quarter million points.
import * as THREE from 'three';
import { CarRenderer } from '../../render/carRender.js';
import { Studio } from '../install/studio.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { applyPose } from '../../cinema/sequence.js';
import { keyed } from '../install/keyed.js';
import { smooth, smoother, inOutCubic, outCubic, outExpo, range, clamp01 } from '../../cinema/ease.js';

const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const P = new URLSearchParams(location.search);
const num = (k, d) => (P.has(k) ? +P.get(k) : d);

// film timeline (s)
const TL = {
  total: num('total', 4.4),
  frustIn: [0.0, 0.7],        // frustums fly in and settle
  sparse: [0.35, 1.45],       // sparse points triangulated
  dense: [1.1, 2.5],          // gaussians grow
  sharpen: [1.5, 3.0],        // sigma: blurry -> sharp
  scan: [2.6, 3.5],           // splats -> mesh (scan plane, front to rear)
  frustOut: [3.0, 3.6],
};
const NSPLAT = num('n', 260000), NSPARSE = num('ns', 9000), NCAM = 12;
const CAR_Z = [-1.75, 1.75]; // body length range (scan plane travel)

// a hash in GLSL for the stochastic threshold
const HASH = 'float h12(vec2 p) { vec3 p3 = fract(vec3(p.xyx) * .1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }';

export default class ReconShot {
  constructor(ctx) { this.ctx = ctx; }

  async init() {
    const { stage, ui, assets } = this.ctx;
    ui.loadText('搭建装配间');
    this.studio = new Studio(stage);
    this.studio.setLights({ bars: [0, 0, 0], env: 0, top: 0, rim: 0, slits: 0 });
    ui.loadText('构建车模');
    const car = await assets.car();
    this.carView = new CarRenderer(car, { envMap: this.studio.env });
    this.carView.showRest();
    this._aoSide = stage.gtao.normalMaterial.side;
    stage.gtao.normalMaterial.side = THREE.DoubleSide;
    this.carView.attachGTAO(stage.gtao);
    stage.scene.add(this.carView.group);
    // mesh reveal: clip away everything ahead of the scan plane (body z decreasing = front to rear)
    this.clip = new THREE.Plane(v3(0, 0, 1), -CAR_Z[1]);
    for (const m of this.carView.materials) { m.clippingPlanes = [this.clip]; m.clipShadows = true; }

    ui.loadText('采样高斯');
    this.buildSplats();
    this.buildFrustums();
    this.buildScanLine();
    this.seed = 0;
    // camera: high wide orbit over the frustum ring, descending into the first pose of the install film
    this.cam = keyed([
      { t: 0.0, pos: v3(7.6, 5.6, 8.6), tgt: v3(0, 0.6, 0), fov: 42, aperture: 0.004 },
      { t: 1.6, pos: v3(2.0, 3.6, 9.6), tgt: v3(0, 0.6, 0.1), fov: 34, aperture: 0.006 },
      { t: 3.0, pos: v3(-3.6, 1.7, 7.6), tgt: v3(0, 0.55, 0.2), fov: 31, aperture: 0.01 },
      { t: TL.total, pos: v3(-4.4, 0.40, 4.3), tgt: v3(0, 0.55, 0.25), fov: 30, aperture: 0.012 },
    ]);
    this.film = { total: TL.total, render: (T, o) => this.render(T, o), describe: () => ({ total: TL.total, splats: NSPLAT, sparse: NSPARSE, cams: NCAM, TL }) };
  }

  // ------------------------------------------------------------------ splats
  /** Area-weighted surface samples of the car's render meshes with baked studio shading. */
  sample(n, rnd) {
    const parts = this.carView.group.children.map(m => {
      const g = m.geometry, Pp = g.attributes.position.array, Nn = g.attributes.normal.array, I = g.index.array, mat = m.material;
      const area = new Float64Array(I.length / 3); let tot = 0;
      for (let f = 0; f < area.length; f++) {
        const a = I[f * 3] * 3, b = I[f * 3 + 1] * 3, c = I[f * 3 + 2] * 3;
        const ux = Pp[b] - Pp[a], uy = Pp[b + 1] - Pp[a + 1], uz = Pp[b + 2] - Pp[a + 2], vx = Pp[c] - Pp[a], vy = Pp[c + 1] - Pp[a + 1], vz = Pp[c + 2] - Pp[a + 2];
        tot += 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx); area[f] = tot;
      }
      const glass = /Glass|clear cover|optical clear|Optics/.test(mat.name || '') || mat.transmission > 0.3;
      return { Pp, Nn, I, area, tot, col: mat.color.clone(), glass, emissive: mat.emissive?.clone(), name: mat.name };
    });
    const total = parts.reduce((s, p) => s + p.tot * (p.glass ? 0.6 : 1), 0);
    const pos = new Float32Array(n * 3), col = new Float32Array(n * 3), nrm = new Float32Array(n * 3);
    const L = v3(0.35, 1, 0.45).normalize(), Lb = v3(-0.6, 0.3, -0.7).normalize();
    let k = 0;
    for (const p of parts) {
      const m = Math.round(n * p.tot * (p.glass ? 0.6 : 1) / total);
      for (let j = 0; j < m && k < n; j++, k++) {
        const r = rnd() * p.tot; let lo = 0, hi = p.area.length - 1;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (p.area[mid] < r) lo = mid + 1; else hi = mid; }
        let u = rnd(), w = rnd(); if (u + w > 1) { u = 1 - u; w = 1 - w; }
        const a = p.I[lo * 3] * 3, b = p.I[lo * 3 + 1] * 3, c = p.I[lo * 3 + 2] * 3;
        for (let q = 0; q < 3; q++) {
          pos[k * 3 + q] = p.Pp[a + q] + u * (p.Pp[b + q] - p.Pp[a + q]) + w * (p.Pp[c + q] - p.Pp[a + q]);
          nrm[k * 3 + q] = p.Nn[a + q] + u * (p.Nn[b + q] - p.Nn[a + q]) + w * (p.Nn[c + q] - p.Nn[a + q]);
        }
        const N = v3(nrm[k * 3], nrm[k * 3 + 1], nrm[k * 3 + 2]).normalize();
        // baked light: soft key from above, cool back fill, a little sheen on top surfaces (clear coat)
        const d = 0.32 + 0.6 * Math.max(0, N.dot(L)) + 0.18 * Math.max(0, N.dot(Lb)) + 0.25 * Math.max(0, N.y) ** 6;
        const c0 = p.glass ? new THREE.Color(0.05, 0.07, 0.08) : p.col.clone();
        const jit = 0.95 + 0.1 * rnd();
        col[k * 3] = c0.r * d * jit; col[k * 3 + 1] = c0.g * d * jit; col[k * 3 + 2] = c0.b * d * jit;
      }
    }
    return { pos: pos.subarray(0, k * 3), col: col.subarray(0, k * 3), nrm: nrm.subarray(0, k * 3), n: k };
  }

  buildSplats() {
    let s = 12345; const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    const S = this.sample(NSPLAT, rnd);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(S.pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(S.col, 3));
    const rr = new Float32Array(S.n * 2);
    for (let i = 0; i < S.n; i++) { rr[i * 2] = rnd(); rr[i * 2 + 1] = 0.6 + 0.8 * rnd(); }
    g.setAttribute('aRand', new THREE.BufferAttribute(rr, 2));
    this.splatU = {
      uGrow: { value: 0 }, uSigma: { value: 3 }, uOpacity: { value: 1 }, uPix: { value: 1000 }, uSeed: { value: 0 }, uScanZ: { value: 99 },
      uBase: { value: num('r', 0.014) },
    };
    this.splats = new THREE.Points(g, new THREE.ShaderMaterial({
      uniforms: this.splatU, transparent: false, depthWrite: true, depthTest: true, fog: false,
      vertexShader: /* glsl */`
        uniform float uGrow, uSigma, uPix, uScanZ, uBase;
        attribute vec2 aRand; attribute vec3 color;
        varying vec3 vCol; varying float vA; varying vec2 vR;
        void main() {
          // each splat appears at its own moment of the growth (front-to-back-ish wave + random)
          float t0 = 0.75 * aRand.x;
          float a = smoothstep(t0, t0 + 0.25, uGrow);
          vec3 p = position;
          vec4 mv = modelViewMatrix * vec4(p, 1.0);
          gl_Position = projectionMatrix * mv;
          float r = uBase * aRand.y * uSigma * (0.4 + 0.6 * a);
          gl_PointSize = clamp(2.0 * r * uPix / -mv.z, 1.0, 96.0);
          vA = a * step(p.z, uScanZ + 0.02);   // behind the scan plane the mesh takes over
          vCol = color; vR = aRand;
        }`,
      fragmentShader: /* glsl */`
        uniform float uOpacity, uSeed, uSigma; varying vec3 vCol; varying float vA; varying vec2 vR;
        ${HASH}
        void main() {
          vec2 d = gl_PointCoord * 2.0 - 1.0; float r2 = dot(d, d);
          if (r2 > 1.0) discard;
          // gaussian footprint; blurry (large sigma) splats are also fainter, as in early training
          float a = exp(-r2 * 3.2) * vA * uOpacity * mix(1.0, 0.55, clamp((uSigma - 1.0) / 2.0, 0.0, 1.0));
          if (a < h12(gl_FragCoord.xy + vec2(uSeed * 17.31, uSeed * 5.13) + vR * 113.0)) discard;
          gl_FragColor = vec4(vCol, 1.0);
          #include <tonemapping_fragment>
          #include <colorspace_fragment>
        }`,
    }));
    this.splats.frustumCulled = false; this.splats.userData.noAO = true;
    this.ctx.stage.scene.add(this.splats);

    // sparse cloud (SfM keypoints): a subset of surface points, small and bright, additive
    const Sp = this.sample(NSPARSE, rnd);
    const gs = new THREE.BufferGeometry();
    gs.setAttribute('position', new THREE.BufferAttribute(Sp.pos, 3));
    const rs = new Float32Array(Sp.n); for (let i = 0; i < Sp.n; i++) rs[i] = rnd();
    gs.setAttribute('aRand', new THREE.BufferAttribute(rs, 1));
    gs.setAttribute('color', new THREE.BufferAttribute(Sp.col, 3));
    this.sparseU = { uShow: { value: 0 }, uFade: { value: 1 }, uPix: { value: 1000 } };
    this.sparse = new THREE.Points(gs, new THREE.ShaderMaterial({
      uniforms: this.sparseU, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false,
      vertexShader: /* glsl */`
        uniform float uShow, uPix; attribute float aRand; attribute vec3 color; varying float vA; varying vec3 vC;
        void main() {
          vec4 mv = modelViewMatrix * vec4(position, 1.0); gl_Position = projectionMatrix * mv;
          float a = smoothstep(aRand * 0.85, aRand * 0.85 + 0.08, uShow);
          // a pop when it is triangulated
          float pop = 1.0 + 2.5 * exp(-40.0 * max(0.0, uShow - aRand * 0.85));
          gl_PointSize = clamp(0.012 * pop * uPix / -mv.z, 1.5, 12.0);
          vA = a; vC = mix(vec3(0.75, 0.95, 1.0), color * 3.0, 0.35);
        }`,
      fragmentShader: /* glsl */`
        uniform float uFade; varying float vA; varying vec3 vC;
        void main() { vec2 d = gl_PointCoord * 2.0 - 1.0; float r2 = dot(d, d); if (r2 > 1.0) discard;
          gl_FragColor = vec4(vC * exp(-r2 * 2.5) * vA * uFade * 0.9, 1.0); }`,
    }));
    this.sparse.frustumCulled = false; this.sparse.userData.noAO = true;
    this.ctx.stage.scene.add(this.sparse);
    this.sparsePts = Sp;
  }

  // ------------------------------------------------------------------ capture frustums
  buildFrustums() {
    const stage = this.ctx.stage, group = (this.frustGroup = new THREE.Group());
    stage.scene.add(group);
    this.cams = [];
    // the capture walk: a ring at hand height, slightly uneven like a person walking round
    const look = v3(0, 0.6, -0.1);
    for (let i = 0; i < NCAM; i++) {
      const a = (i / NCAM) * Math.PI * 2 + 0.35, R = num('ring', 5.0) + 0.35 * Math.sin(i * 2.1), y = 1.7 + 0.15 * Math.cos(i * 1.7);
      const pos = v3(Math.sin(a) * R * 0.85, y, Math.cos(a) * R);
      this.cams.push({ pos, look, a });
    }
    // pictures: the car seen from each capture position (rendered once, lit, at photo resolution)
    const rtW = 384, rtH = 216, cam = new THREE.PerspectiveCamera(42, rtW / rtH, 0.1, 50);
    this.studio.setLights({ bars: [1, 1, 1], env: 1, top: 1, rim: 1, slits: 1 });
    const hidden = [this.splats, this.sparse, group];
    hidden.forEach(o => (o.visible = false));
    this.clip.constant = 99; // the whole car
    const r = stage.renderer;
    for (const c of this.cams) {
      const rt = new THREE.WebGLRenderTarget(rtW, rtH, { samples: 4 });
      cam.position.copy(c.pos); cam.lookAt(c.look); cam.updateMatrixWorld();
      this.studio.update(0, { floor: 1 });
      r.setRenderTarget(rt); r.render(stage.scene, cam); r.setRenderTarget(null);
      c.tex = rt.texture; c.tex.colorSpace = THREE.SRGBColorSpace;
    }
    hidden.forEach(o => (o.visible = true));
    this.studio.setLights({ bars: [0, 0, 0], env: 0, top: 0, rim: 0, slits: 0 });

    // frustum: apex at the camera, picture plane at depth D (aspect 16:9)
    const D = 0.45, hw = Math.tan(THREE.MathUtils.degToRad(42) / 2) * D * (rtW / rtH), hh = Math.tan(THREE.MathUtils.degToRad(42) / 2) * D;
    const lineGeo = new THREE.BufferGeometry().setFromPoints([
      v3(0, 0, 0), v3(-hw, -hh, -D), v3(0, 0, 0), v3(hw, -hh, -D), v3(0, 0, 0), v3(hw, hh, -D), v3(0, 0, 0), v3(-hw, hh, -D),
      v3(-hw, -hh, -D), v3(hw, -hh, -D), v3(hw, -hh, -D), v3(hw, hh, -D), v3(hw, hh, -D), v3(-hw, hh, -D), v3(-hw, hh, -D), v3(-hw, -hh, -D),
      v3(-hw * 0.3, hh * 1.08, -D), v3(0, hh * 1.35, -D), v3(0, hh * 1.35, -D), v3(hw * 0.3, hh * 1.08, -D), // "up" tick
    ]);
    this.frustMat = new THREE.LineBasicMaterial({ color: new THREE.Color(0.55, 0.95, 1.0).multiplyScalar(2.2), transparent: true, opacity: 1, fog: false, toneMapped: false });
    for (const c of this.cams) {
      const g = new THREE.Group();
      const lines = new THREE.LineSegments(lineGeo, this.frustMat); lines.userData.noAO = true;
      const pic = new THREE.Mesh(new THREE.PlaneGeometry(2 * hw * 0.97, 2 * hh * 0.97), new THREE.MeshBasicMaterial({ map: c.tex, transparent: true, opacity: 1, fog: false, toneMapped: false, side: THREE.DoubleSide }));
      pic.position.z = -D; pic.userData.noAO = true;
      g.add(lines, pic);
      group.add(g);
      c.group = g; c.pic = pic;
    }
    // triangulation rays: frustum apex -> sparse points (a few hundred, flickering in while points appear)
    const NR = 420, rp = new Float32Array(NR * 6), rc = new Float32Array(NR * 6);
    let s = 777; const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    this.rays = [];
    for (let i = 0; i < NR; i++) {
      const c = this.cams[Math.floor(rnd() * NCAM)], k = Math.floor(rnd() * this.sparsePts.n), p = this.sparsePts.pos;
      rp.set([c.pos.x, c.pos.y, c.pos.z, p[k * 3], p[k * 3 + 1], p[k * 3 + 2]], i * 6);
      this.rays.push(0.06 + 0.8 * rnd());
    }
    const rg = new THREE.BufferGeometry();
    rg.setAttribute('position', new THREE.BufferAttribute(rp, 3));
    rg.setAttribute('color', new THREE.BufferAttribute(rc, 3));
    this.rayLines = new THREE.LineSegments(rg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, fog: false, toneMapped: false }));
    this.rayLines.frustumCulled = false; this.rayLines.userData.noAO = true;
    stage.scene.add(this.rayLines);
  }

  buildScanLine() {
    // a glowing ring of light at the scan plane: thin box outline around the car's section
    const g = new THREE.PlaneGeometry(1.9, 1.75);
    this.scanMat = new THREE.ShaderMaterial({
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false,
      uniforms: { uA: { value: 0 } },
      vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
      fragmentShader: /* glsl */`uniform float uA; varying vec2 vUv;
        void main() { vec2 e = min(vUv, 1.0 - vUv); float edge = exp(-min(e.x, e.y) * 90.0); float fill = 0.06;
          gl_FragColor = vec4(vec3(0.5, 0.95, 1.0) * (edge * 1.6 + fill) * uA, 1.0); }`,
    });
    this.scan = new THREE.Mesh(g, this.scanMat);
    this.scan.position.set(0, 0.8, 0); this.scan.userData.noAO = true;
    this.ctx.stage.scene.add(this.scan);
  }

  // ------------------------------------------------------------------ pose at film time T
  pose(T) {
    const U = this.splatU;
    U.uSeed.value = (this.seed = (this.seed + 1) % 4096);
    // frustums: fly in from outside and above, each with a shutter flash on its picture; fade out at the end
    this.cams.forEach((c, i) => {
      const k = range(T, TL.frustIn[0] + i * 0.035, TL.frustIn[1] + i * 0.035), e = outExpo(k);
      const from = c.pos.clone().multiplyScalar(2.4).add(v3(0, 2.5, 0));
      c.group.position.copy(from.lerp(c.pos, e));
      c.group.lookAt(c.look);
      const out = 1 - smooth(range(T, TL.frustOut[0], TL.frustOut[1]));
      c.group.visible = k > 0 && out > 0.001;
      c.group.scale.setScalar(0.4 + 0.6 * e);
      const flash = Math.exp(-18 * Math.max(0, T - (TL.frustIn[0] + i * 0.035 + 0.25)));
      c.pic.material.opacity = out * clamp01(k * 3);
      c.pic.material.color.setScalar(1 + 3 * flash * (k > 0.3 ? 1 : 0));
    });
    this.frustMat.opacity = 1 - smooth(range(T, TL.frustOut[0], TL.frustOut[1]));
    // sparse points + rays
    const sp = range(T, TL.sparse[0], TL.sparse[1]);
    this.sparseU.uShow.value = sp;
    this.sparseU.uFade.value = 1 - smooth(range(T, TL.dense[0] + 0.3, TL.dense[1]));
    const col = this.rayLines.geometry.attributes.color, C = col.array;
    const rf = this.sparseU.uFade.value;
    this.rays.forEach((t0, i) => {
      const a = sp > t0 ? Math.exp(-14 * (sp - t0)) : 0, b = 0.9 * a * rf;
      C.set([0.25 * b, 0.75 * b, 1.0 * b, 0.05 * b, 0.2 * b, 0.3 * b], i * 6);
    });
    col.needsUpdate = true;
    // dense splats: grow, sharpen; then the scan plane hands over to the mesh
    U.uGrow.value = smoother(range(T, TL.dense[0], TL.dense[1]));
    U.uSigma.value = 1 + 2.4 * (1 - outCubic(range(T, TL.sharpen[0], TL.sharpen[1])));
    const sc = inOutCubic(range(T, TL.scan[0], TL.scan[1]));
    const z = CAR_Z[1] - (CAR_Z[1] - CAR_Z[0]) * sc;
    U.uScanZ.value = T < TL.scan[0] ? 99 : z;   // splats remain where body z < scan z (rear part)
    this.clip.constant = T < TL.scan[0] ? -99 : -z; // mesh visible where z >= scan z (normal +z: dist = z - scan)
    this.carView.group.visible = T >= TL.scan[0];
    this.splats.visible = T < TL.scan[1] + 0.05;
    this.scan.position.z = z;
    this.scanMat.uniforms.uA.value = T > TL.scan[0] && T < TL.scan[1] ? Math.sin(Math.PI * range(T, TL.scan[0], TL.scan[1])) ** 0.5 : 0;
    // a faint top light comes up with the mesh so it is not a silhouette; lights-on is the install film's job
    const m = smooth(range(T, TL.scan[0], TL.total));
    this.studio.setLights({ bars: [0, 0, 0], env: 0.18 * m + 0.05, top: 0.12 * m, rim: 0.35 * m, slits: 0.25 * m });
    this.studio.update(T, { floor: 0.5 + 0.5 * m });
  }

  evaluate(T) {
    const dc = P.get('devcam');
    if (dc) { const v = dc.split(',').map(Number); return { pos: v3(v[0], v[1], v[2]), tgt: v3(v[3], v[4], v[5]), fov: v[6] || 40, t: T, aperture: 0.004, shake: 0, roll: 0 }; }
    return { ...this.cam(0, Math.min(TL.total, Math.max(0, T))), t: T, shake: 0, roll: 0 };
  }

  render(T, { samples = 16, shutter = 0.5 / 30 } = {}) {
    const stage = this.ctx.stage, cam = stage.camera;
    renderFilmFrame(stage, T, x => this.evaluate(x), (t, _T, ev) => {
      // projection scale for the point sizes (pixels per unit at depth 1)
      const h = stage.renderer.domElement.height;
      this.splatU.uPix.value = this.sparseU.uPix.value = h / (2 * Math.tan(THREE.MathUtils.degToRad(ev.fov) / 2));
      this.pose(t);
    }, { samples, shutter });
  }

  preview(T) {
    const stage = this.ctx.stage, ev = this.evaluate(T);
    applyPose(stage.camera, ev);
    const h = stage.renderer.domElement.height;
    this.splatU.uPix.value = this.sparseU.uPix.value = h / (2 * Math.tan(THREE.MathUtils.degToRad(ev.fov) / 2));
    this.pose(ev.t);
    stage.render(1 / 60);
  }
}
