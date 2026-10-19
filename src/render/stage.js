// Renderer, HDR sky, sun with a focus-following shadow frustum, and the cinematic post chain.
import * as THREE from 'three';
import { HDRLoader } from 'three/addons/loaders/HDRLoader.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

// Sun direction of the HDRI (found offline: azimuth 34 deg, elevation 47.9 deg) rotated by ENV_YAW.
const ENV_YAW = -0.5;
const SUN_RAW = new THREE.Vector3(0.553, 0.742, 0.377).normalize();

const FinishShader = {
  uniforms: {
    tDiffuse: { value: null }, uTime: { value: 0 }, uVignette: { value: 0.32 }, uGrain: { value: 0.035 },
    uAberration: { value: 0.0012 }, uShake: { value: new THREE.Vector2() }, uFlash: { value: 0 },
  },
  vertexShader: `varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
  fragmentShader: /* glsl */`
    uniform sampler2D tDiffuse; uniform float uTime, uVignette, uGrain, uAberration, uFlash; uniform vec2 uShake;
    varying vec2 vUv;
    float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233)) + uTime * 61.7) * 43758.5453); }
    void main() {
      vec2 uv = vUv + uShake;
      vec2 d = uv - 0.5;
      float r2 = dot(d, d);
      vec3 c;
      c.r = texture2D(tDiffuse, uv - d * uAberration * r2 * 8.0).r;
      c.g = texture2D(tDiffuse, uv).g;
      c.b = texture2D(tDiffuse, uv + d * uAberration * r2 * 8.0).b;
      // gentle filmic grade: cool shadows, warm highlights
      float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
      c = mix(c * vec3(0.96, 1.0, 1.05), c * vec3(1.04, 1.0, 0.95), smoothstep(0.15, 0.85, l));
      c = mix(vec3(l), c, 1.06);
      c *= 1.0 - uVignette * smoothstep(0.15, 0.75, r2 * 1.6);
      c += (hash(uv * 1000.0) - 0.5) * uGrain;
      c += uFlash;
      gl_FragColor = vec4(c, 1.0);
    }`,
};

export class Stage {
  constructor(container, { pixelRatio = Math.min(devicePixelRatio, 2), preserve = false } = {}) {
    const renderer = (this.renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'high-performance', preserveDrawingBuffer: preserve }));
    renderer.setPixelRatio(pixelRatio);
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.82;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFShadowMap;
    container.appendChild(renderer.domElement);
    this.container = container;

    const scene = (this.scene = new THREE.Scene());
    scene.fog = new THREE.Fog(0xbcc6cf, 140, 700);
    this.camera = new THREE.PerspectiveCamera(40, container.clientWidth / container.clientHeight, 0.1, 1500);
    this.camera.position.set(-50, 2, 30);

    // sun
    const sunDir = SUN_RAW.clone().applyAxisAngle(new THREE.Vector3(0, 1, 0), ENV_YAW);
    this.sunDir = sunDir;
    const sun = (this.sun = new THREE.DirectionalLight(0xfff1df, 3.2));
    sun.castShadow = true;
    sun.shadow.mapSize.set(4096, 4096);
    sun.shadow.bias = -0.00015;
    sun.shadow.normalBias = 0.03;
    sun.shadow.radius = 2.5;
    const sc = sun.shadow.camera;
    sc.left = -38; sc.right = 38; sc.top = 38; sc.bottom = -38; sc.near = 1; sc.far = 260;
    scene.add(sun, sun.target);
    this.focus = new THREE.Vector3(-20, 2, 14);

    const hemi = new THREE.HemisphereLight(0xcfe0f0, 0x4a4740, 0.12);
    scene.add(hemi);

    // post chain
    const size = new THREE.Vector2(); renderer.getDrawingBufferSize(size);
    const rt = new THREE.WebGLRenderTarget(size.x, size.y, { type: THREE.HalfFloatType, samples: 4 });
    const composer = (this.composer = new EffectComposer(renderer, rt));
    composer.addPass(new RenderPass(scene, this.camera));
    const gtao = (this.gtao = new GTAOPass(scene, this.camera, size.x, size.y));
    gtao.output = GTAOPass.OUTPUT.Default;
    gtao.blendIntensity = 0.85;
    gtao.updateGtaoMaterial({ radius: 0.6, distanceExponent: 1.5, thickness: 1.2, scale: 1.0, samples: 16 });
    gtao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 6, rings: 2, samples: 16 });
    composer.addPass(gtao);
    // also keep glass, sprites and other flagged helpers out of the AO buffers
    const ov = gtao._overrideVisibility.bind(gtao);
    gtao._overrideVisibility = function () {
      ov();
      this.scene.traverse(o => { if (o.userData.noAO && o.visible) { o.visible = false; this._visibilityCache.push(o); } });
    };
    // bloom only for genuine highlights (sun glints on glass/clear coat, sparks): with a ~1.0 threshold every
    // sunlit pastel panel (linear ~1.5-2) glowed as if emissive
    const bloom = (this.bloom = new UnrealBloomPass(new THREE.Vector2(size.x, size.y), 0.28, 0.5, 2.4));
    composer.addPass(bloom);
    composer.addPass(new OutputPass());
    const finish = (this.finish = new ShaderPass(FinishShader));
    composer.addPass(finish);
    // offline accumulation (motion blur / depth of field / supersampling for film export)
    this.accum = null;
    this.accumMat = new THREE.ShaderMaterial({
      uniforms: { tNew: { value: null }, tOld: { value: null }, uW: { value: 1 } },
      vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }',
      fragmentShader: 'uniform sampler2D tNew, tOld; uniform float uW; varying vec2 vUv; void main() { gl_FragColor = mix(texture2D(tOld, vUv), texture2D(tNew, vUv), uW); }',
      depthTest: false, depthWrite: false,
    });
    this.quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.accumMat);
    this.quadScene = new THREE.Scene(); this.quadScene.add(this.quad);
    this.quadCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);

    addEventListener('resize', () => this.resize());
  }

  async loadSky(url) {
    const tex = await new HDRLoader().loadAsync(url);
    tex.mapping = THREE.EquirectangularReflectionMapping;
    // Lighting environment: the sky HDRI with the sun disk clamped (the DirectionalLight is the sun)
    // and the empty lower hemisphere replaced by sunlit asphalt, so undersides stay properly dark.
    const envScene = new THREE.Scene();
    const mat = new THREE.ShaderMaterial({
      side: THREE.BackSide, depthWrite: false,
      uniforms: { uSky: { value: tex }, uGround: { value: new THREE.Color(0.30, 0.29, 0.27) }, uYaw: { value: ENV_YAW } },
      vertexShader: 'varying vec3 vDir; void main() { vDir = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
      fragmentShader: /* glsl */`
        uniform sampler2D uSky; uniform vec3 uGround; uniform float uYaw; varying vec3 vDir;
        void main() {
          vec3 d = normalize(vDir);
          float c = cos(-uYaw), s = sin(-uYaw);
          vec3 r = vec3(c * d.x + s * d.z, d.y, -s * d.x + c * d.z);
          vec2 uv = vec2(atan(r.z, r.x) * 0.15915494 + 0.5, asin(clamp(r.y, -1.0, 1.0)) * 0.31830988 + 0.5);
          vec3 sky = min(texture2D(uSky, vec2(uv.x, max(uv.y, 0.505))).rgb, vec3(14.0));
          float g = smoothstep(0.02, -0.06, d.y);
          gl_FragColor = vec4(mix(sky, uGround, g), 1.0);
        }`,
    });
    envScene.add(new THREE.Mesh(new THREE.SphereGeometry(10, 64, 32), mat));
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    const env = pmrem.fromScene(envScene, 0, 0.1, 100).texture;
    pmrem.dispose();
    this.scene.environment = env;
    this.scene.background = tex;
    this.scene.backgroundRotation.y = ENV_YAW;
    this.scene.environmentIntensity = 1.0;
    this.scene.backgroundIntensity = 0.9;
    this.envMap = env;
    return env;
  }

  /**
   * Reflection probe: PMREM of the actual set seen from `position` (objects in `hide` left out), so
   * car paint and glass reflect the street, trees and buildings instead of a bare sky dome.
   */
  captureProbe(position, hide = []) {
    const vis = hide.map(o => o.visible);
    hide.forEach(o => (o.visible = false));
    const focus = this.focus.clone();
    this.focus.copy(position); this.updateSun();
    this._pmrem ||= new THREE.PMREMGenerator(this.renderer);
    const rt = this._pmrem.fromScene(this.scene, 0, 0.05, 1500, { position, size: 256 });
    this.focus.copy(focus); this.updateSun();
    hide.forEach((o, i) => (o.visible = vis[i]));
    return rt;
  }

  resize(w = this.container.clientWidth, h = this.container.clientHeight) {
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
    this.composer.setSize(w, h);
  }

  /** Keep the shadow frustum centred on the action. */
  updateSun() {
    const f = this.focus;
    this.sun.target.position.copy(f);
    this.sun.position.copy(f).addScaledVector(this.sunDir, 120);
    this.sun.target.updateMatrixWorld();
  }

  /**
   * Film-quality frame: `sub(k, n)` positions scene + camera for sub-sample k of n; the
   * post-processed sub-frames are averaged, then grain/vignette is applied once.
   */
  renderAccumulated(sub, n) {
    const r = this.renderer, size = new THREE.Vector2();
    r.getDrawingBufferSize(size);
    if (!this.accum || this.accum[0].width !== size.x || this.accum[0].height !== size.y) {
      this.accum?.forEach(t => t.dispose());
      this.accum = [0, 1].map(() => new THREE.WebGLRenderTarget(size.x, size.y, { type: THREE.FloatType }));
    }
    const finishPass = this.finish;
    finishPass.enabled = false;
    this.composer.renderToScreen = false;
    let [a, b] = this.accum;
    for (let k = 0; k < n; k++) {
      sub(k, n);
      this.updateSun();
      this.composer.render(1 / 60);
      this.accumMat.uniforms.tNew.value = this.composer.readBuffer.texture;
      this.accumMat.uniforms.tOld.value = a.texture;
      this.accumMat.uniforms.uW.value = 1 / (k + 1);
      r.setRenderTarget(b); r.render(this.quadScene, this.quadCam);
      [a, b] = [b, a];
    }
    this.accum = [a, b];
    // grain + vignette once, to screen
    finishPass.enabled = true;
    this.composer.renderToScreen = true;
    finishPass.uniforms.tDiffuse.value = a.texture;
    r.setRenderTarget(null);
    finishPass._fsQuad.render(r);
  }

  render(dt = 1 / 60) {
    this.updateSun();
    this.finish.uniforms.uTime.value = (this.finish.uniforms.uTime.value + dt) % 100;
    this.composer.render(dt);
  }
}
