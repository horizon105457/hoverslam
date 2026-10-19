// "X-ray" view of the simulation lattice: glowing nodes coloured by plastic damage and the beam network.
import * as THREE from 'three';

export class LatticeView {
  constructor(scene, car) {
    this.car = car;
    const N = car.nodes.count;
    const B = car.beams;
    // only draw structural lattice edges once (skip duplicates from joints)
    const pairs = [];
    for (let b = 0; b < B.count; b++) pairs.push(B.i[b], B.j[b]);
    this.pairs = Int32Array.from(pairs);
    const lg = new THREE.BufferGeometry();
    this.linePos = new Float32Array(this.pairs.length * 3);
    lg.setAttribute('position', new THREE.BufferAttribute(this.linePos, 3).setUsage(THREE.DynamicDrawUsage));
    this.lineCol = new Float32Array(this.pairs.length * 3);
    lg.setAttribute('color', new THREE.BufferAttribute(this.lineCol, 3).setUsage(THREE.DynamicDrawUsage));
    this.lines = new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.16, blending: THREE.AdditiveBlending, depthWrite: false }));
    const pg = new THREE.BufferGeometry();
    this.ptPos = new Float32Array(N * 3);
    this.ptCol = new Float32Array(N * 3);
    pg.setAttribute('position', new THREE.BufferAttribute(this.ptPos, 3).setUsage(THREE.DynamicDrawUsage));
    pg.setAttribute('color', new THREE.BufferAttribute(this.ptCol, 3).setUsage(THREE.DynamicDrawUsage));
    this.points = new THREE.Points(pg, new THREE.PointsMaterial({ size: 0.014, vertexColors: true, transparent: true, opacity: 0.55, blending: THREE.AdditiveBlending, depthWrite: false }));
    this.group = new THREE.Group();
    this.group.add(this.lines, this.points);
    this.group.visible = false;
    for (const o of [this.lines, this.points]) { o.frustumCulled = false; o.userData.noAO = true; }
    scene.add(this.group);
  }

  update(pos, damage) {
    if (!this.group.visible) return;
    const N = this.car.nodes.count, P = this.ptPos, C = this.ptCol;
    P.set(pos.subarray(0, N * 3));
    for (let n = 0; n < N; n++) {
      const d = damage ? Math.min(1, damage.damage[n] / 255 * 1.6) : 0;
      C[n * 3] = 0.1 + 0.9 * d; C[n * 3 + 1] = 0.35 + 0.25 * (1 - d) - 0.2 * d; C[n * 3 + 2] = 0.7 * (1 - d) + 0.05;
    }
    const L = this.linePos, LC = this.lineCol, pr = this.pairs;
    for (let k = 0; k < pr.length; k += 2) {
      const a = pr[k], b = pr[k + 1];
      const dx = pos[a * 3] - pos[b * 3], dy = pos[a * 3 + 1] - pos[b * 3 + 1], dz = pos[a * 3 + 2] - pos[b * 3 + 2];
      const torn = dx * dx + dy * dy + dz * dz > 0.06; // snapped connector / torn beam: do not draw
      for (const [q, n] of [[k, a], [k + 1, b]]) {
        const m = torn ? a : n;
        L[q * 3] = pos[m * 3]; L[q * 3 + 1] = pos[m * 3 + 1]; L[q * 3 + 2] = pos[m * 3 + 2];
        LC[q * 3] = C[n * 3] * 0.35; LC[q * 3 + 1] = C[n * 3 + 1] * 0.35; LC[q * 3 + 2] = C[n * 3 + 2] * 0.35;
      }
    }
    this.lines.geometry.attributes.position.needsUpdate = true;
    this.lines.geometry.attributes.color.needsUpdate = true;
    this.points.geometry.attributes.position.needsUpdate = true;
    this.points.geometry.attributes.color.needsUpdate = true;
  }
}
