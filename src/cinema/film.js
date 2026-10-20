// Offline film frame: temporal (shutter), lens (aperture) and sub-pixel jitter, averaged over `samples`
// renders by Stage.renderAccumulated. Program-agnostic: the caller supplies the camera timeline and a function
// that poses the scene at simulation time t. Used through `program.film.render(T)` by tools/film/*.mjs.
import * as THREE from 'three';

const GOLDEN = 2.399963;

/**
 * @param stage     Stage
 * @param T         film time (s)
 * @param evaluate  T -> CamPose with .t (simulation time), e.g. sequence.evaluate
 * @param setScene  (t, T, pose) -> void: pose the scene at simulation time t (called once per sample)
 */
export function renderFilmFrame(stage, T, evaluate, setScene, { samples = 16, shutter = 0.5 / 30, aperture = 0.035 } = {}) {
  const cam = stage.camera, w = stage.renderer.domElement.width, h = stage.renderer.domElement.height;
  const right = new THREE.Vector3(), up = new THREE.Vector3();
  stage.renderAccumulated((k, n) => {
    // not before the film starts: evaluate() loops, so frame 0 would blend in the last shot
    const ev = evaluate(Math.max(0, T + (k / n - 0.5) * shutter));
    setScene(ev.t, T, ev);
    // lens: Vogel disk sample, re-aimed at the focus point
    const focus = ev.focus || ev.tgt, ap = ev.aperture ?? aperture;
    const rr = ap * Math.sqrt((k + 0.5) / n), th = k * GOLDEN;
    cam.fov = ev.fov; cam.position.copy(ev.pos); cam.lookAt(ev.tgt); cam.updateMatrixWorld();
    right.setFromMatrixColumn(cam.matrixWorld, 0); up.setFromMatrixColumn(cam.matrixWorld, 1);
    cam.position.addScaledVector(right, Math.cos(th) * rr).addScaledVector(up, Math.sin(th) * rr);
    if (ev.shake) cam.position.add(new THREE.Vector3(Math.sin(T * 91) * 0.02, Math.cos(T * 77) * 0.02, 0).multiplyScalar(ev.shake));
    cam.lookAt(focus);
    if (ev.roll) cam.rotateZ(ev.roll);
    cam.setViewOffset(w, h, ((k * 0.618) % 1) - 0.5, ((k * 0.382 + 0.5) % 1) - 0.5, w, h);
    cam.updateProjectionMatrix();
  }, samples);
  cam.clearViewOffset();
}
