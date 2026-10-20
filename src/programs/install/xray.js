// X-ray / ghost view of the rest-pose car with a scanning wipe between the solid and the ghosted body.
//
// The ghost needs the rest pose exactly as CarRenderer draws it, i.e. every render vertex at its embedding (tet +
// barycentrics over the rest lattice nodes, incl. the shader's y >= 2 mm clamp), so its positions are evaluated once
// on the CPU from aTet/aBary instead of trusting the raw `position` attribute. The solid car is cut by a clipping
// plane that travels along the body; the ghost is drawn on the other side of the same plane, with a bright
// band where the plane crosses the shell (the "scan line"). While any part is ghosted the car leaves the GTAO
// buffers and stops casting shadows: the invisible shell must not darken the kit inside it.
import * as THREE from 'three';

const GHOST_VS = /* glsl */`
varying vec3 vW; varying vec3 vN;
void main() {
  vec4 w = modelMatrix * vec4(position, 1.0);
  vW = w.xyz; vN = normalize(mat3(modelMatrix) * normal);
  gl_Position = projectionMatrix * viewMatrix * w;
}`;
const GHOST_FS = /* glsl */`
uniform vec3 uColor, uEdge; uniform float uAmt, uFill, uScan, uDir, uBand, uLines, uY0, uRim;
varying vec3 vW; varying vec3 vN;
void main() {
  // ghost side of the scan plane (body z = world z: the rig only translates)
  float s = (vW.z - uScan) * uDir;
  if (s < 0.0) discard;
  vec3 V = cameraPosition - vW;
  float dist = length(V); V /= dist;
  float f = 1.0 - abs(dot(normalize(vN), V));
  float rim = pow(f, 3.0);
  // fine horizontal scan lines (body height), faint
  float ln = 0.7 + 0.3 * smoothstep(0.35, 0.5, abs(fract((vW.y - uY0) * uLines) - 0.5));
  float band = exp(-s * s / (uBand * uBand));
  // recede when the lens is right at the shell (macro shots of the kit inside)
  float near = smoothstep(0.15, 1.2, dist);
  vec3 c = uColor * (uFill + rim * uRim) * ln * uAmt * near + uEdge * band * 2.2 * uAmt;
  gl_FragColor = vec4(c, 1.0);
}`;

export class XRay {
  /** @param carView CarRenderer (rest pose), @param parent the rig group the car lives in */
  constructor(carView, parent) {
    this.carView = carView;
    this.uniforms = {
      uColor: { value: new THREE.Color(0x7fb8d8) }, uEdge: { value: new THREE.Color(0x9fe8d0) }, uAmt: { value: 1 }, uFill: { value: 0.012 }, uRim: { value: 0.32 },
      uScan: { value: 10 }, uDir: { value: 1 }, uBand: { value: 0.02 }, uLines: { value: 140 }, uY0: { value: 0 },
    };
    const mat = (this.material = new THREE.ShaderMaterial({
      uniforms: this.uniforms, vertexShader: GHOST_VS, fragmentShader: GHOST_FS,
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false,
    }));
    this.group = new THREE.Group(); this.group.name = 'xray-car';
    // exterior skin only: the cabin would fill the ghost with clutter
    const keep = /^(Paint|Glass|Trim|Rubber|Wheel|Fascia|Headlamp • (black|bright)|Optics • (deep red|ruby)|Mirror)/;
    const car = carView.car, R = car.nodes.rest, TN = car.render.tetNodes;
    for (const m of carView.group.children) {
      m.userData.glassNoAO = !!m.userData.noAO; m.userData.castDefault = m.castShadow;
      if (!keep.test(m.name)) continue;
      // rest-pose positions through the embedding (mirrors the lattice vertex shader at F = I)
      const src = m.geometry, tet = src.attributes.aTet.array, bar = src.attributes.aBary.array, n = tet.length, pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const t = Math.round(tet[i]) * 4, b1 = bar[i * 3], b2 = bar[i * 3 + 1], b3 = bar[i * 3 + 2], b0 = 1 - b1 - b2 - b3;
        const a = TN[t] * 3, b = TN[t + 1] * 3, c = TN[t + 2] * 3, d = TN[t + 3] * 3;
        for (let k = 0; k < 3; k++) pos[i * 3 + k] = R[a + k] * b0 + R[b + k] * b1 + R[c + k] * b2 + R[d + k] * b3;
        pos[i * 3 + 1] = Math.max(pos[i * 3 + 1], 0.002);
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      geo.setAttribute('normal', src.attributes.normal);
      geo.setIndex(src.index);
      const g = new THREE.Mesh(geo, mat);
      g.frustumCulled = false; g.userData.noAO = true; g.renderOrder = 1;
      this.group.add(g);
    }
    parent.add(this.group);
    // solid side: clip every car material with the complementary half-space
    this.plane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 10);
    for (const m of carView.materials) { m.clippingPlanes = [this.plane]; m.clipShadows = false; }
    // the laser sheet itself
    this.sheet = new THREE.Mesh(new THREE.PlaneGeometry(2.6, 2.2), new THREE.ShaderMaterial({
      uniforms: { uAmt: { value: 0 }, uColor: { value: new THREE.Color(0x9fe8d0) } },
      vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
      fragmentShader: 'uniform float uAmt; uniform vec3 uColor; varying vec2 vUv; void main() { vec2 d = abs(vUv - 0.5) * 2.0; float a = (1.0 - smoothstep(0.6, 1.0, d.x)) * (1.0 - smoothstep(0.55, 1.0, d.y)); float edge = smoothstep(0.985, 1.0, max(d.x, d.y)) * 0.0; gl_FragColor = vec4(uColor * (a * 0.05 + edge) * uAmt, 1.0); }',
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, fog: false,
    }));
    this.sheet.userData.noAO = true; this.sheet.position.y = 1.0;
    parent.add(this.sheet);
    this.set({ scan: 10, dir: 1 });
  }

  /**
   * @param scan  body z of the scan plane
   * @param dir   +1: ghost where z > scan (wipe front -> rear as scan decreases); -1: ghost where z < scan
   * @param amt   ghost brightness, sheet: laser sheet strength, lift: rig height (for the scan lines)
   */
  set({ scan, dir = 1, amt = 1, sheet = 0, lift = 0 }) {
    const u = this.uniforms;
    u.uScan.value = scan; u.uDir.value = dir; u.uAmt.value = amt; u.uY0.value = lift;
    // solid keeps the other side: dir=+1 -> keep z <= scan: n = (0,0,-1), c = scan
    this.plane.normal.set(0, 0, -dir); this.plane.constant = scan * dir;
    this.sheet.position.z = scan; this.sheet.material.uniforms.uAmt.value = sheet;
    this.sheet.visible = sheet > 0.001;
    // fully solid: ghost off; fully ghosted: solid hidden (cheaper than clipping everything away)
    const zMin = -1.75, zMax = 1.75;
    const ghostAll = dir > 0 ? scan < zMin : scan > zMax, solidAll = dir > 0 ? scan > zMax : scan < zMin;
    this.group.visible = !solidAll && amt > 0.001;
    this.carView.group.visible = !ghostAll;
    const ghosting = !solidAll;
    for (const m of this.carView.group.children) { m.userData.noAO = ghosting || m.userData.glassNoAO; m.castShadow = !ghosting && m.userData.castDefault !== false; }
    this.state = { scan, dir, ghostAll, solidAll };
  }
}
