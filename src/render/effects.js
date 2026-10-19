// Time-driven effects that can be scrubbed both ways: rigid glass/debris shards, impact dust and
// shock rings, grinding sparks, and scrape/gouge marks burned into the asphalt.
import * as THREE from 'three';
import { puffTexture } from '../scene/textures.js';

const SHARD_COLORS = [
  [0.62, 0.68, 0.72], // clear lens glass
  [0.75, 0.04, 0.03], // red tail lens
  [0.95, 0.55, 0.08], // amber repeater
  [0.56, 0.78, 0.68], // mint paint chip
  [0.05, 0.05, 0.055], // black plastic
  [0.85, 0.86, 0.88], // mirror
  [0.42, 0.55, 0.52], // tempered glass (green edge tint)
];
const GLASSY = new Set([0, 1, 2, 5, 6]);

function shardGeometry() {
  const g = new THREE.IcosahedronGeometry(0.5, 0);
  const P = g.attributes.position;
  let s = 9;
  const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
  const moved = new Map();
  for (let i = 0; i < P.count; i++) {
    const key = `${P.getX(i).toFixed(3)},${P.getY(i).toFixed(3)},${P.getZ(i).toFixed(3)}`;
    if (!moved.has(key)) moved.set(key, [0.65 + r() * 0.5, 0.65 + r() * 0.5, 0.65 + r() * 0.5]);
    const k = moved.get(key);
    P.setXYZ(i, P.getX(i) * k[0], P.getY(i) * k[1], P.getZ(i) * k[2]);
  }
  g.computeVertexNormals();
  return g.toNonIndexed();
}

export class Shards {
  constructor(scene, envMap, max = 9000) {
    this.max = max;
    const geo = shardGeometry();
    geo.computeVertexNormals();
    this.glass = new THREE.InstancedMesh(geo, new THREE.MeshPhysicalMaterial({
      color: 0xffffff, roughness: 0.03, metalness: 0.1, transparent: true, opacity: 0.32, envMap, envMapIntensity: 3.0,
      specularIntensity: 1, ior: 1.52, clearcoat: 1, clearcoatRoughness: 0.02, flatShading: true,
    }), max);
    this.solid = new THREE.InstancedMesh(geo, new THREE.MeshStandardMaterial({ roughness: 0.45, metalness: 0.25, flatShading: true }), max);
    for (const m of [this.glass, this.solid]) {
      m.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      m.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(max * 3), 3);
      m.frustumCulled = false; m.count = 0;
      scene.add(m);
    }
    this.glass.castShadow = false; this.solid.castShadow = true; this.solid.receiveShadow = true;
    this.glass.userData.noAO = true;
    this.pos = new Float32Array(max * 3); this.quat = new Float32Array(max * 4);
    this._m = new THREE.Matrix4(); this._q = new THREE.Quaternion(); this._p = new THREE.Vector3(); this._s = new THREE.Vector3();
  }
  setMeta(particles) { this.meta = particles; }
  update(player, t) {
    const meta = this.meta;
    if (!meta || !player.ready) { this.glass.count = this.solid.count = 0; return; }
    const n = Math.min(player.sampleParticles(t, this.pos, this.quat), this.max, meta.count);
    let ng = 0, ns = 0;
    for (let k = 0; k < n; k++) {
      if (meta.spawn[k] > t) continue;
      const kind = meta.kind[k];
      const tgt = GLASSY.has(kind) ? this.glass : this.solid;
      const i = GLASSY.has(kind) ? ng++ : ns++;
      this._p.set(this.pos[k * 3], this.pos[k * 3 + 1], this.pos[k * 3 + 2]);
      this._q.set(this.quat[k * 4], this.quat[k * 4 + 1], this.quat[k * 4 + 2], this.quat[k * 4 + 3]);
      this._s.set(meta.size[k * 3], meta.size[k * 3 + 1], meta.size[k * 3 + 2]);
      this._m.compose(this._p, this._q, this._s);
      tgt.setMatrixAt(i, this._m);
      const c = SHARD_COLORS[kind] || SHARD_COLORS[0];
      const v = 0.85 + ((k * 2654435761) % 1000) / 1000 * 0.3;
      tgt.instanceColor.setXYZ(i, c[0] * v, c[1] * v, c[2] * v);
    }
    this.glass.count = ng; this.solid.count = ns;
    for (const m of [this.glass, this.solid]) { m.instanceMatrix.needsUpdate = true; m.instanceColor.needsUpdate = true; }
  }
}

// -------------------------------------------------------------------------------------------------
/** Billboarded, analytically animated sprites (dust puffs or sparks). */
class SpriteField {
  constructor(scene, max, material) {
    const g = new THREE.InstancedBufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0], 3));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    this.a0 = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4); // xyz birth pos, birth time
    this.a1 = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4); // xyz velocity, size
    this.a2 = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4); // life, seed, kind, intensity
    g.setAttribute('a0', this.a0); g.setAttribute('a1', this.a1); g.setAttribute('a2', this.a2);
    g.instanceCount = 0;
    this.mesh = new THREE.Mesh(g, material);
    this.mesh.frustumCulled = false;
    this.mesh.userData.noAO = true;
    this.mesh.renderOrder = 5;
    scene.add(this.mesh);
    this.max = max; this.n = 0;
  }
  clear() { this.n = 0; this.mesh.geometry.instanceCount = 0; }
  push(x, y, z, t, vx, vy, vz, size, life, seed, kind, inten) {
    if (this.n >= this.max) return;
    const i = this.n++;
    this.a0.setXYZW(i, x, y, z, t); this.a1.setXYZW(i, vx, vy, vz, size); this.a2.setXYZW(i, life, seed, kind, inten);
  }
  commit() { for (const a of [this.a0, this.a1, this.a2]) a.needsUpdate = true; this.mesh.geometry.instanceCount = this.n; }
}

const DUST_VS = /* glsl */`
  attribute vec4 a0, a1, a2;
  uniform float uTime; uniform vec3 uSun;
  varying vec2 vUv; varying float vAlpha; varying float vShade; varying float vSeed; varying float vWy;
  void main() {
    float age = uTime - a0.w;
    float life = a2.x;
    vSeed = a2.y;
    if (age < 0.0 || age > life) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
    float k = a2.z > 0.5 ? 2.6 : 1.4;                 // rings decelerate harder
    vec3 drift = a1.xyz * (1.0 - exp(-k * age)) / k;
    vec3 p = a0.xyz + drift + vec3(0.0, 0.22 * age + 0.05 * age * age, 0.0);
    float size = a1.w * (0.35 + 1.0 - exp(-1.3 * age)) * (1.0 + 0.25 * age);
    float fadeIn = smoothstep(0.0, 0.08, age);
    vAlpha = fadeIn * pow(1.0 - age / life, 1.6) * a2.w;
    vec4 mv = modelViewMatrix * vec4(p, 1.0);
    float rot = vSeed * 6.283 + age * (vSeed - 0.5) * 0.6;
    vec2 q = mat2(cos(rot), -sin(rot), sin(rot), cos(rot)) * position.xy;
    mv.xy += q * size;
    vWy = (inverse(viewMatrix) * mv).y;
    vUv = position.xy + 0.5;
    vShade = 0.55 + 0.45 * clamp(dot(normalize(vec3(q, 0.6)), normalize((viewMatrix * vec4(uSun, 0.0)).xyz)) * 0.5 + 0.5, 0.0, 1.0);
    gl_Position = projectionMatrix * mv;
  }`;
const DUST_FS = /* glsl */`
  uniform sampler2D uTex; uniform vec3 uColor; uniform vec3 uSunCol; uniform vec3 uAmb;
  varying vec2 vUv; varying float vAlpha; varying float vShade; varying float vSeed; varying float vWy;
  void main() {
    vec2 uv = vUv; if (vSeed > 0.5) uv.x = 1.0 - uv.x;
    float a = texture2D(uTex, uv).r * vAlpha * smoothstep(0.0, 0.35, vWy);
    if (a < 0.003) discard;
    vec3 c = uColor * (uAmb + uSunCol * vShade);
    gl_FragColor = vec4(c, a);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;
const SPARK_VS = /* glsl */`
  attribute vec4 a0, a1, a2;
  uniform float uTime;
  varying vec2 vUv; varying float vHeat;
  void main() {
    float age = uTime - a0.w, life = a2.x;
    if (age < 0.0 || age > life) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
    vec3 v = a1.xyz * exp(-1.5 * age) + vec3(0.0, -9.81 * age, 0.0);
    vec3 p = a0.xyz + a1.xyz * (1.0 - exp(-1.5 * age)) / 1.5 + vec3(0.0, -4.905 * age * age, 0.0);
    p.y = max(p.y, 0.01);
    vec4 c0 = modelViewMatrix * vec4(p, 1.0);
    vec4 c1 = modelViewMatrix * vec4(p - v * 0.018, 1.0);
    vec2 dir = c0.xy - c1.xy; float L = length(dir); dir = L > 1e-5 ? dir / L : vec2(1.0, 0.0);
    vec2 nrm = vec2(-dir.y, dir.x);
    float w = a1.w;
    vec4 mv = mix(c1, c0, position.x + 0.5);
    mv.xy += nrm * position.y * w;
    vUv = position.xy + 0.5;
    vHeat = 1.0 - age / life;
    gl_Position = projectionMatrix * mv;
  }`;
const SPARK_FS = /* glsl */`
  varying vec2 vUv; varying float vHeat;
  void main() {
    float a = smoothstep(0.5, 0.0, abs(vUv.y - 0.5)) * vHeat;
    vec3 c = mix(vec3(1.0, 0.25, 0.04), vec3(1.0, 0.8, 0.45), vHeat) * (1.8 + 4.5 * vHeat); // hot enough to clear the bloom threshold
    gl_FragColor = vec4(c * a, a);
  }`;

export class Effects {
  constructor(scene, stage) {
    this.stage = stage;
    const sunCol = new THREE.Color(0xfff1df).multiplyScalar(1.2);
    this.dustMat = new THREE.ShaderMaterial({
      vertexShader: DUST_VS, fragmentShader: DUST_FS, transparent: true, depthWrite: false,
      uniforms: { uTime: { value: 0 }, uTex: { value: puffTexture() }, uColor: { value: new THREE.Color(0.50, 0.47, 0.42) }, uSun: { value: stage.sunDir }, uSunCol: { value: sunCol }, uAmb: { value: new THREE.Color(0.45, 0.48, 0.52) } },
    });
    this.sparkMat = new THREE.ShaderMaterial({ vertexShader: SPARK_VS, fragmentShader: SPARK_FS, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, uniforms: { uTime: { value: 0 } } });
    this.dust = new SpriteField(scene, 6000, this.dustMat);
    this.sparks = new SpriteField(scene, 12000, this.sparkMat);
    this.decal = new GroundDecal();
  }

  /** Rebuild sprite buffers from the event logs (cheap; called whenever the logs grow). */
  setEvents(ev) {
    this.events = ev;
    this.dust.clear(); this.sparks.clear();
    let s = 1;
    const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    const D = ev.dust || [];
    for (let i = 0; i < D.length; i += 7) {
      const [t, x, y, z, vx, sp, vz] = D.slice(i, i + 7);
      const strength = Math.min(1, (sp - 3) / 10);
      const ground = y < 0.6;
      const puffs = r() < 0.5 + strength ? 1 + Math.round(strength * 2) : 0;
      for (let k = 0; k < puffs; k++) {
        const a = r() * 6.283, out = 0.3 + r() * 1.2 * (0.3 + strength);
        this.dust.push(x + (r() - 0.5) * 0.3, Math.max(0.05, y) + r() * 0.15, z + (r() - 0.5) * 0.3, t + r() * 0.03,
          vx * 0.3 + Math.cos(a) * out, 0.2 + r() * 0.8 * strength, vz * 0.3 + Math.sin(a) * out,
          0.3 + r() * 0.45 + strength * 0.6, 1.0 + r() * 1.6, r(), 0, (ground ? 0.035 : 0.015) + 0.035 * strength);
      }
      if (strength > 0.6 && ground && r() < 0.5) { // ground shock ring
        for (let k = 0; k < 6; k++) {
          const a = (k / 6) * 6.283 + r() * 0.5, sp2 = 3 + r() * 4 * strength;
          this.dust.push(x, 0.12, z, t, Math.cos(a) * sp2, 0.1, Math.sin(a) * sp2, 0.3 + r() * 0.3, 0.9 + r() * 0.8, r(), 1, 0.045);
        }
      }
    }
    const S = ev.sparks || [];
    for (let i = 0; i < S.length; i += 7) {
      const [t, x, y, z, vx, vy, vz] = S.slice(i, i + 7);
      const sp = Math.hypot(vx, vy, vz);
      if (sp < 3.5 || r() > 0.55) continue;
      const n = 1 + Math.floor(r() * 3);
      for (let k = 0; k < n; k++) {
        const j = 0.3 * sp;
        this.sparks.push(x, Math.max(y, 0.02), z, t + r() * 0.02, vx * (0.5 + r() * 0.5) + (r() - 0.5) * j, Math.abs(vy) * 0.5 + r() * 1.6, vz * (0.5 + r() * 0.5) + (r() - 0.5) * j, 0.004 + r() * 0.006, 0.08 + r() * 0.25, r(), 0, 1);
      }
    }
    this.dust.commit(); this.sparks.commit();
    this.decal.setEvents(ev);
  }

  update(t) {
    this.dustMat.uniforms.uTime.value = t;
    this.sparkMat.uniforms.uTime.value = t;
    this.decal.update(t);
  }
}

// -------------------------------------------------------------------------------------------------
/** Scrapes, tyre marks and impact gouges painted into a canvas that overlays the asphalt. */
export class GroundDecal {
  constructor(size = 2048, area = [-80, -12, 84]) {
    this.area = area; // x0, z0, metres
    const c = (this.canvas = document.createElement('canvas'));
    c.width = c.height = size;
    this.g = c.getContext('2d');
    this.tex = new THREE.CanvasTexture(c);
    this.tex.colorSpace = THREE.NoColorSpace;
    this.tex.anisotropy = 8;
    this.drawnUntil = -1e9; this.cursor = 0;
    this.clear();
  }
  clear() {
    this.g.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.drawnUntil = -1e9; this.cursor = 0; this.tex.needsUpdate = true;
  }
  setEvents(ev) {
    // merge scrape + impact events into one time-sorted list
    const list = [];
    const S = ev.scrape || [];
    for (let i = 0; i < S.length; i += 6) if (S[i + 2] < 0.6) list.push({ t: S[i], x: S[i + 1], z: S[i + 3], s: S[i + 4], metal: S[i + 5] });
    const D = ev.dust || [];
    for (let i = 0; i < D.length; i += 7) if (D[i + 2] < 0.5 && D[i + 5] > 8) list.push({ t: D[i], x: D[i + 1], z: D[i + 3], gouge: D[i + 5] });
    list.sort((a, b) => a.t - b.t);
    this.list = list;
    this.clear();
  }
  toPx(x, z) { const [x0, z0, L] = this.area, S = this.canvas.width; return [((x - x0) / L) * S, ((z - z0) / L) * S]; }
  update(t) {
    if (!this.list) return;
    if (t < this.drawnUntil) this.clear();
    const g = this.g, S = this.canvas.width, ppm = S / this.area[2];
    let changed = false;
    while (this.cursor < this.list.length && this.list[this.cursor].t <= t) {
      const e = this.list[this.cursor++];
      const [px, pz] = this.toPx(e.x, e.z);
      if (px < 0 || pz < 0 || px > S || pz > S) continue;
      changed = true;
      if (e.gouge) {
        // spalled asphalt: pale powder + radial cracks
        const R = ppm * (0.25 + Math.min(0.6, (e.gouge - 8) * 0.05));
        g.fillStyle = 'rgba(210,205,195,0.10)'; g.beginPath(); g.arc(px, pz, R * 1.6, 0, 6.283); g.fill();
        g.strokeStyle = 'rgba(25,22,20,0.55)'; g.lineWidth = 1.2;
        for (let k = 0; k < 7; k++) {
          let a = Math.random() * 6.283, x = px, y = pz;
          g.beginPath(); g.moveTo(x, y);
          for (let j = 0; j < 5; j++) { a += (Math.random() - 0.5) * 0.9; x += Math.cos(a) * R * 0.35; y += Math.sin(a) * R * 0.35; g.lineTo(x, y); }
          g.stroke();
        }
      } else if (e.metal) {
        // bright metal scratch + dark smear
        g.fillStyle = `rgba(235,232,225,${Math.min(0.5, 0.12 + e.s * 0.03)})`;
        g.fillRect(px - 1, pz - 1, 2.2, 2.2);
        g.fillStyle = 'rgba(20,20,22,0.10)'; g.fillRect(px - 2.5, pz - 2.5, 5, 5);
      } else {
        // rubber
        g.fillStyle = `rgba(8,8,8,${Math.min(0.35, 0.08 + e.s * 0.02)})`;
        g.beginPath(); g.arc(px, pz, ppm * 0.07, 0, 6.283); g.fill();
      }
    }
    this.drawnUntil = t;
    if (changed) this.tex.needsUpdate = true;
  }
  /** Inject into the asphalt material: R/G/B = paint marks, alpha = coverage. */
  patch(material) {
    const u = { uDecal: { value: this.tex }, uDecalArea: { value: new THREE.Vector3(...this.area) } };
    const prev = material.onBeforeCompile;
    material.onBeforeCompile = (sh, r) => {
      prev?.(sh, r);
      Object.assign(sh.uniforms, u);
      sh.vertexShader = 'varying vec3 vWorldPosD;\n' + sh.vertexShader.replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\n vWorldPosD = (modelMatrix * vec4(transformed, 1.0)).xyz;');
      sh.fragmentShader = 'uniform sampler2D uDecal; uniform vec3 uDecalArea; varying vec3 vWorldPosD;\n' + sh.fragmentShader.replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
        vec2 duv = (vWorldPosD.xz - uDecalArea.xy) / uDecalArea.z;
        if (duv.x > 0.0 && duv.y > 0.0 && duv.x < 1.0 && duv.y < 1.0) {
          vec4 dc = texture2D(uDecal, duv);
          diffuseColor.rgb = mix(diffuseColor.rgb, dc.rgb, dc.a);
          roughnessFactor = mix(roughnessFactor, 0.6, dc.a * 0.6);
        }`);
    };
    material.needsUpdate = true;
  }
}
