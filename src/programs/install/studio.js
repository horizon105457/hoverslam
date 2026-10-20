// Assembly bay for the install film: a procedural softbox environment (PMREM) for the reflections on paint and
// polished metal, a glossy graphite floor with a blurred planar reflection, overhead light bars and floor
// markings; depth comes from exponential fog. The car stands on the floor (no lift: the posts read as props).
// Everything here is static except what the program drives per frame (light levels, floor ring pulse).
// Reconfigures the shared Stage at runtime (fog, background, sun, exposure, bloom) and restores it in dispose().
import * as THREE from 'three';
import { Reflector } from 'three/addons/objects/Reflector.js';

// full light levels: environment intensity, top (shadow-casting) light, hemisphere fill, rim spots
const ENV = 1.35, TOP = 2.6, FILL = 0.55, RIM = [48, 30];

// ---------------------------------------------------------------------------------------------- environment
/**
 * Studio lighting environment: a graphite room with a large overhead softbox, tall strip lights, two broad 3/4
 * fill panels and a cool back strip. Car paint and chrome need *shapes* to reflect (the long highlights that
 * define a product shot); an HDRI sky would put a horizon and trees into the studio. The room and the floor are
 * kept dark grey, not black: paint and carbon reflect them over most of their area, and a black room left the
 * car and the kit as silhouettes.
 */
export function studioEnvironment(renderer) {
  const s = new THREE.Scene();
  const basic = (c, k) => new THREE.MeshBasicMaterial({ color: new THREE.Color(c).multiplyScalar(k), side: THREE.DoubleSide });
  const room = new THREE.Mesh(new THREE.BoxGeometry(26, 11, 26), basic(0x2a2f34, 0.32));
  room.material.side = THREE.BackSide; room.position.y = 4.5;
  s.add(room);
  const panel = (w, h, pos, look, mat) => { const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), mat); m.position.copy(pos); m.lookAt(look); s.add(m); return m; };
  const O = new THREE.Vector3(0, 0.8, 0);
  // overhead key: long softbox along the car (gives the roof/bonnet its sheen)
  panel(4.0, 8.0, new THREE.Vector3(0, 8.5, 0), new THREE.Vector3(0, 0, 0), basic(0xffffff, 2.8));
  // tall side strips, slightly forward/back so the doors catch a vertical highlight
  for (const sx of [-1, 1]) {
    panel(0.45, 6.5, new THREE.Vector3(sx * 9, 3.4, sx * 2.5), O, basic(0xf4f7ff, 4.2));
    panel(0.30, 4.5, new THREE.Vector3(sx * 7.5, 2.4, -sx * 6.5), O, basic(0xdfe9ff, 3.4));
  }
  // broad 3/4 fills where the wide shots look from (front-left, rear-right): soft key on the flanks
  panel(5.0, 2.6, new THREE.Vector3(-6.5, 2.2, 6.0), O, basic(0xfff3e6, 1.1));
  panel(5.0, 2.6, new THREE.Vector3(6.5, 2.2, -6.0), O, basic(0xe6f0ff, 1.0));
  // cool back strip low behind the car (rim on the tail), warm low kicker in front
  panel(12, 0.35, new THREE.Vector3(0, 1.6, -11), O, basic(0xcfe2ff, 3.5));
  panel(9, 0.8, new THREE.Vector3(0, 0.9, 11), O, basic(0xffe6cc, 1.4));
  // floor: dark grey epoxy, bounces a little light into the undersides (bottom nozzles under the floating car)
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(26, 26), basic(0x1c1f22, 0.3));
  floor.rotation.x = -Math.PI / 2; floor.position.y = -0.01;
  s.add(floor);
  const pmrem = new THREE.PMREMGenerator(renderer);
  const env = pmrem.fromScene(s, 0.02, 0.1, 60).texture;
  pmrem.dispose();
  s.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
  return env;
}

// ---------------------------------------------------------------------------------------------- floor
const FLOOR_PARS = /* glsl */`
uniform sampler2D tMirror; uniform mat4 uTexMat; uniform float uMirror, uRing, uRingR, uTime, uLines;
uniform vec2 uMirrorSize;
varying vec4 vMirrorUv; varying vec3 vFloorW;
`;
// polished epoxy floor: blurred mirror (mip + small disk), fresnel, painted bay markings, a pulse ring
const FLOOR_FRAG = /* glsl */`
{
  vec2 muv = vMirrorUv.xy / vMirrorUv.w;
  vec3 refl = vec3(0.0);
  // 8-tap disk on a mip level: wider blur for distant floor (larger footprint per pixel)
  float lod = 1.6;
  vec2 px = 1.0 / uMirrorSize * 3.5;
  for (int i = 0; i < 8; i++) {
    float a = float(i) * 2.39996 + 0.5, r = sqrt((float(i) + 0.5) / 8.0);
    refl += textureLod(tMirror, muv + vec2(cos(a), sin(a)) * r * px, lod).rgb;
  }
  refl /= 8.0;
  vec3 V = normalize(cameraPosition - vFloorW);
  float fres = 0.05 + 0.95 * pow(1.0 - clamp(V.y, 0.0, 1.0), 4.0);
  // bay markings in world xz: car footprint brackets, a turntable ring with ticks, faint grid
  vec2 p = vFloorW.xz;
  float lw = 0.012;
  float grid = 0.0;
  vec2 g = abs(fract(p / 1.0 + 0.5) - 0.5) * 1.0;
  grid = (1.0 - smoothstep(0.0, 0.006, min(g.x, g.y))) * 0.25;
  float r = length(p);
  float ring = 1.0 - smoothstep(0.0, lw, abs(r - uRingR));
  float ang = atan(p.y, p.x);
  float ticks = (1.0 - smoothstep(0.0, 0.004, abs(fract(ang / 6.28318 * 72.0 + 0.5) - 0.5) * r * 0.087)) * step(abs(r - uRingR - 0.09), 0.06);
  float ring2 = 1.0 - smoothstep(0.0, lw * 0.6, abs(r - uRingR - 0.18));
  // footprint brackets (corners of a 2.2 x 3.6 rectangle)
  vec2 q = abs(p - vec2(0.0, -0.0)) - vec2(1.15, 1.85);
  float box = (1.0 - smoothstep(0.0, lw, abs(max(q.x, q.y)))) * step(max(q.x, q.y), 0.02);
  vec2 qa = abs(p) - vec2(1.15, 1.85);
  float corner = step(abs(p.x), 1.17) * step(abs(p.y), 1.87) * (step(0.85, abs(p.x)) * step(1.55, abs(p.y)) + 0.0);
  float bracket = (1.0 - smoothstep(0.0, lw, min(abs(qa.x), abs(qa.y)))) * corner;
  float fadeR = 1.0 - smoothstep(4.0, 9.0, r);
  vec3 lines = vec3(0.55, 0.62, 0.66) * (grid * 0.05 + ring * 0.35 + ticks * 0.18 + ring2 * 0.12 + bracket * 0.4) * fadeR * uLines;
  // pulse ring running outwards (beats: lift, installs, test fire)
  float pr = uRing > 0.0 ? (1.0 - smoothstep(0.0, 0.05 + uRing * 0.25, abs(r - uRing * 6.0))) * (1.0 - uRing) : 0.0;
  lines += vec3(0.58, 0.83, 0.72) * pr * 1.6;
  outgoingLight = outgoingLight * (1.0 - fres * uMirror * 0.5) + refl * fres * uMirror + lines;
}
`;

function floorMaterial(rt, texMat, size) {
  // matte for the lights (no broad sun hotspot), glossy through the blurred planar mirror
  const m = new THREE.MeshStandardMaterial({ color: 0x121518, roughness: 0.7, metalness: 0.0, envMapIntensity: 0.35 });
  m.userData.uniforms = {
    tMirror: { value: rt.texture }, uTexMat: { value: texMat }, uMirror: { value: 0.6 }, uLines: { value: 1 }, uRing: { value: 0 }, uRingR: { value: 3.0 },
    uTime: { value: 0 }, uMirrorSize: { value: new THREE.Vector2(size.x, size.y) },
  };
  m.onBeforeCompile = sh => {
    Object.assign(sh.uniforms, m.userData.uniforms);
    sh.vertexShader = 'uniform mat4 uTexMat; varying vec4 vMirrorUv; varying vec3 vFloorW;\n' + sh.vertexShader
      .replace('#include <project_vertex>', '#include <project_vertex>\n  vMirrorUv = uTexMat * vec4(position, 1.0);\n  vFloorW = (modelMatrix * vec4(position, 1.0)).xyz;');
    sh.fragmentShader = FLOOR_PARS + sh.fragmentShader.replace('#include <opaque_fragment>', FLOOR_FRAG + '#include <opaque_fragment>');
  };
  m.customProgramCacheKey = () => 'install-floor';
  return m;
}

export class Studio {
  /** @param stage Stage */
  constructor(stage) {
    this.stage = stage;
    const r = stage.renderer, scene = stage.scene;
    this.saved = { fog: scene.fog, bg: scene.background, env: scene.environment, envI: scene.environmentIntensity, exp: r.toneMappingExposure,
      bloom: [stage.bloom.strength, stage.bloom.radius, stage.bloom.threshold], sunI: stage.sun.intensity, sunC: stage.sun.color.clone(), sunDir: stage.sunDir.clone(),
      shadow: (({ left, right, top, bottom, near, far }) => ({ left, right, top, bottom, near, far }))(stage.sun.shadow.camera), radius: stage.sun.shadow.radius, bias: stage.sun.shadow.bias, focus: stage.focus.clone(), clip: r.localClippingEnabled, vignette: stage.finish.uniforms.uVignette.value,
      grain: stage.finish.uniforms.uGrain.value, gtao: stage.gtao.blendIntensity };
    this.env = studioEnvironment(r);
    this.group = new THREE.Group(); this.group.name = 'studio';
    scene.add(this.group);

    // shared stage, reconfigured for a graphite studio
    const bg = new THREE.Color(0x0c1013);
    scene.background = bg;
    scene.fog = new THREE.FogExp2(0x0c1013, 0.04);
    scene.environment = this.env;
    scene.environmentIntensity = ENV;
    r.toneMappingExposure = 1.15;
    r.localClippingEnabled = true; // cutaways
    stage.bloom.strength = 0.28; stage.bloom.radius = 0.5; stage.bloom.threshold = 4.0; // brighter room: keep bloom to real glints
    stage.finish.uniforms.uVignette.value = 0.3;
    stage.finish.uniforms.uGrain.value = 0.025;
    stage.gtao.blendIntensity = 0.9;
    // the "sun" becomes a near-vertical top light: a soft contact shadow under the car on the floor
    stage.sun.intensity = TOP; stage.sun.color.set(0xf3f6ff);
    stage.sunDir.set(0.42, 1, 0.3).normalize(); // off vertical: no mirror glint of it in top-down shots
    const sc = stage.sun.shadow.camera;
    sc.left = -3.2; sc.right = 3.2; sc.top = 3.2; sc.bottom = -3.2; sc.near = 100; sc.far = 140; sc.updateProjectionMatrix();
    stage.sun.shadow.radius = 5;
    stage.sun.shadow.bias = -0.0002;
    stage.focus.set(0, 0.6, -0.2);

    // diffuse fill (a hemisphere light has no specular, so no glints on polished parts): lifts the shadow sides
    // of the car and the kit, also in the macro shots where the direct lights are dimmed
    this.fill = new THREE.HemisphereLight(0xdde7ef, 0x15181b, FILL);
    this.group.add(this.fill);
    // rim lights (no shadows: cheap) to separate the car and metal from the background
    const rim = (this.rim = new THREE.SpotLight(0xcfe0ff, RIM[0], 18, 0.55, 0.8, 1.6));
    rim.position.set(-3.0, 4.6, -6); rim.target.position.set(0, 0.6, -0.5);
    const rim2 = (this.rim2 = new THREE.SpotLight(0xffe2c8, RIM[1], 16, 0.5, 0.9, 1.6));
    rim2.position.set(-4.2, 2.6, 4.6); rim2.target.position.set(0, 0.5, 0.3); // front-left: not mirrored into the rear-right finale camera
    this.group.add(rim, rim.target, rim2, rim2.target);

    // glossy floor with a planar mirror (half resolution, mip-mapped for the blur)
    const size = new THREE.Vector2(); r.getDrawingBufferSize(size);
    const msize = new THREE.Vector2(Math.round(size.x / 2), Math.round(size.y / 2));
    const floor = (this.floor = new Reflector(new THREE.PlaneGeometry(80, 80), { textureWidth: msize.x, textureHeight: msize.y, clipBias: 0.002, multisample: 4 }));
    const rt = floor.getRenderTarget();
    rt.texture.generateMipmaps = true; rt.texture.minFilter = THREE.LinearMipmapLinearFilter;
    const texMat = floor.material.uniforms.textureMatrix.value;
    floor.material.dispose();
    floor.material = floorMaterial(rt, texMat, msize);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    // render the mirror once per composed frame (the GTAO normal pass would trigger a second render)
    const mirror = floor.onBeforeRender;
    floor.onBeforeRender = function (renderer, sc2, cam) { if (!this.userData.dirty) return; this.userData.dirty = false; mirror.call(this, renderer, sc2, cam); };
    this.group.add(floor);

    // overhead light bars (motivate the env softbox; they also streak across the floor reflection)
    this.bars = [];
    const housing = new THREE.MeshStandardMaterial({ color: 0x16191c, roughness: 0.6, metalness: 0.5 });
    for (const x of [-1.9, 0, 1.9]) {
      const barMat = new THREE.MeshBasicMaterial({ color: new THREE.Color(0xf2f6ff).multiplyScalar(5), fog: false });
      const bar = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.02, 6.5), barMat); bar.position.set(x, 4.6, -0.2);
      this.bars.push(bar);
      const h = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.06, 6.7), housing); h.position.set(x, 4.64, -0.2);
      bar.userData.noAO = true;
      this.group.add(bar, h);
    }
    // far back wall: dark panels with thin vertical light slits for depth in wide shots
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x171b1f, roughness: 0.7, metalness: 0.2 });
    const wall = new THREE.Mesh(new THREE.PlaneGeometry(40, 9), wallMat); wall.position.set(0, 4.5, -13);
    const slitMat = (this.slitMat = new THREE.MeshBasicMaterial({ color: new THREE.Color(0xcfe0ff).multiplyScalar(2.6) }));
    this.group.add(wall);
    for (let i = -6; i <= 6; i++) {
      if (i === 0) continue;
      const s = new THREE.Mesh(new THREE.PlaneGeometry(0.05, 3.2 + (Math.abs(i) % 3) * 0.6), slitMat);
      s.position.set(i * 1.6, 2.0, -12.95); s.userData.noAO = true; this.group.add(s);
    }
    const wall2 = wall.clone(); wall2.position.set(0, 4.5, 13); wall2.rotation.y = Math.PI; this.group.add(wall2);
  }

  /** Light levels 0..1: overhead bars (each), environment, top light (+ fill), rims, wall slits. */
  setLights({ bars = [1, 1, 1], env = 1, top = 1, rim = 1, slits = 1 }) {
    this.bars.forEach((b, i) => b.material.color.setRGB(0.95, 0.965, 1).multiplyScalar(3.6 * bars[i]));
    this.stage.scene.environmentIntensity = ENV * env;
    this.stage.sun.intensity = TOP * top; this.fill.intensity = FILL * top;
    this.rim.intensity = RIM[0] * rim; this.rim2.intensity = RIM[1] * rim;
    this.slitMat.color.setRGB(0.81, 0.88, 1).multiplyScalar(2.6 * slits);
  }

  /** Per composed frame (and per film sub-sample): allow one mirror render, animate uniforms. */
  /** ring: pulse ring 0..1; floor: 0..1 strength of the mirror and the markings (faded under close-ups) */
  update(T, { ring = 0, floor = 1 } = {}) {
    this.floor.userData.dirty = true;
    const u = this.floor.material.userData.uniforms;
    u.uRing.value = ring; u.uTime.value = T; u.uMirror.value = 0.55 * floor; u.uLines.value = floor;
  }

  dispose() {
    const { stage } = this, s = this.saved, scene = stage.scene, r = stage.renderer;
    scene.remove(this.group);
    scene.fog = s.fog; scene.background = s.bg; scene.environment = s.env; scene.environmentIntensity = s.envI;
    r.toneMappingExposure = s.exp; r.localClippingEnabled = s.clip;
    [stage.bloom.strength, stage.bloom.radius, stage.bloom.threshold] = s.bloom;
    stage.sun.intensity = s.sunI; stage.sun.color.copy(s.sunC); stage.sunDir.copy(s.sunDir);
    Object.assign(stage.sun.shadow.camera, s.shadow); stage.sun.shadow.camera.updateProjectionMatrix(); stage.sun.shadow.radius = s.radius; stage.sun.shadow.bias = s.bias; stage.focus.copy(s.focus);
    stage.finish.uniforms.uVignette.value = s.vignette; stage.finish.uniforms.uGrain.value = s.grain; stage.gtao.blendIntensity = s.gtao;
    this.floor.dispose(); this.env.dispose();
  }
}
