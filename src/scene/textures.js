// Procedural canvas textures (signs, safety stripes, foliage, windows, paint wear).
import * as THREE from 'three';

function canvas(w, h) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  return [c, c.getContext('2d')];
}
function tex(c, { repeat = false, srgb = true, aniso = 8 } = {}) {
  const t = new THREE.CanvasTexture(c);
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  if (repeat) t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = aniso;
  return t;
}
const rnd = (() => { let s = 12345; return () => ((s = (s * 16807) % 2147483647) / 2147483647); })();

/** Yellow/black diagonal anti-collision stripes with grime. */
export function hazardStripes() {
  const [c, g] = canvas(512, 256);
  g.fillStyle = '#d9a90f'; g.fillRect(0, 0, 512, 256);
  g.fillStyle = '#151412';
  for (let x = -256; x < 768; x += 96) { g.beginPath(); g.moveTo(x, 256); g.lineTo(x + 48, 256); g.lineTo(x + 48 + 256, 0); g.lineTo(x + 256, 0); g.fill(); }
  // grime and scuffs
  for (let i = 0; i < 2600; i++) { g.fillStyle = `rgba(40,32,20,${rnd() * 0.12})`; g.fillRect(rnd() * 512, 256 - Math.pow(rnd(), 2) * 256, 2 + rnd() * 6, 1 + rnd() * 3); }
  const grd = g.createLinearGradient(0, 256, 0, 150); grd.addColorStop(0, 'rgba(50,45,35,0.45)'); grd.addColorStop(1, 'rgba(50,45,35,0)');
  g.fillStyle = grd; g.fillRect(0, 150, 512, 106);
  return tex(c, { repeat: true });
}

/** Octagonal Chinese stop sign 停. */
export function stopSign() {
  const [c, g] = canvas(256, 256);
  g.translate(128, 128);
  const oct = r => { g.beginPath(); for (let i = 0; i < 8; i++) { const a = Math.PI / 8 + i * Math.PI / 4; g.lineTo(Math.cos(a) * r, Math.sin(a) * r); } g.closePath(); };
  g.fillStyle = '#f2f2f2'; oct(126); g.fill();
  g.fillStyle = '#c4161c'; oct(116); g.fill();
  g.fillStyle = '#fff'; g.font = 'bold 128px "PingFang SC","Heiti SC",sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText('停', 0, 6);
  return tex(c);
}

/** Blue road guide sign with Chinese text. */
export function guideSign(lines = ['文一路', '← 古墩路   紫金港路 →']) {
  const [c, g] = canvas(1024, 512);
  g.fillStyle = '#1a4f9c'; g.fillRect(0, 0, 1024, 512);
  g.strokeStyle = '#fff'; g.lineWidth = 10; g.strokeRect(14, 14, 996, 484);
  g.fillStyle = '#fff'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.font = 'bold 150px "PingFang SC","Heiti SC",sans-serif'; g.fillText(lines[0], 512, 180);
  g.font = 'bold 82px "PingFang SC","Heiti SC",sans-serif'; g.fillText(lines[1], 512, 370);
  return tex(c);
}

/** Construction warning board. */
export function constructionSign() {
  const [c, g] = canvas(512, 768);
  g.fillStyle = '#f3c300'; g.fillRect(0, 0, 512, 768);
  g.fillStyle = '#111'; g.font = 'bold 120px "PingFang SC","Heiti SC",sans-serif'; g.textAlign = 'center';
  g.fillText('前方施工', 256, 170);
  g.font = 'bold 96px "PingFang SC","Heiti SC",sans-serif'; g.fillText('减速慢行', 256, 330);
  g.beginPath(); g.moveTo(256, 400); g.lineTo(440, 700); g.lineTo(72, 700); g.closePath(); g.fill();
  g.fillStyle = '#f3c300'; g.font = 'bold 220px sans-serif'; g.fillText('!', 256, 680);
  return tex(c);
}

/** Height limit sign 限高 used on piers. */
export function heightSign() {
  const [c, g] = canvas(256, 256);
  g.fillStyle = '#fff'; g.beginPath(); g.arc(128, 128, 124, 0, Math.PI * 2); g.fill();
  g.strokeStyle = '#c4161c'; g.lineWidth = 22; g.beginPath(); g.arc(128, 128, 108, 0, Math.PI * 2); g.stroke();
  g.fillStyle = '#111'; g.font = 'bold 84px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('4.5m', 128, 132);
  return tex(c);
}

/** Leaf atlas: 4x4 variations of broad leaves with alpha. */
export function leafAtlas() {
  const S = 1024, [c, g] = canvas(S, S);
  g.clearRect(0, 0, S, S);
  const cell = S / 4;
  for (let i = 0; i < 16; i++) {
    const cx = (i % 4) * cell + cell / 2, cy = Math.floor(i / 4) * cell + cell / 2;
    const n = 5 + Math.floor(rnd() * 5);
    for (let k = 0; k < n; k++) {
      const a = rnd() * Math.PI * 2, r = rnd() * cell * 0.28;
      const x = cx + Math.cos(a) * r, y = cy + Math.sin(a) * r, L = cell * (0.16 + rnd() * 0.12), W = L * (0.42 + rnd() * 0.2);
      const h = 85 + rnd() * 30, s = 35 + rnd() * 30, l = 22 + rnd() * 20;
      g.save(); g.translate(x, y); g.rotate(rnd() * Math.PI * 2);
      g.fillStyle = `hsl(${h},${s}%,${l}%)`;
      g.beginPath(); g.moveTo(0, -L / 2); g.quadraticCurveTo(W, 0, 0, L / 2); g.quadraticCurveTo(-W, 0, 0, -L / 2); g.fill();
      g.strokeStyle = `hsla(${h},${s}%,${l + 15}%,0.6)`; g.lineWidth = 1.5; g.beginPath(); g.moveTo(0, -L / 2); g.lineTo(0, L / 2); g.stroke();
      g.restore();
    }
  }
  const t = tex(c);
  t.premultiplyAlpha = false;
  return t;
}

/** Tileable leafy hedge surface. */
export function hedgeTexture() {
  const S = 512, [c, g] = canvas(S, S);
  g.fillStyle = '#20361a'; g.fillRect(0, 0, S, S);
  for (let i = 0; i < 9000; i++) {
    const x = rnd() * S, y = rnd() * S, L = 4 + rnd() * 9, h = 80 + rnd() * 35, l = 14 + rnd() * 26;
    g.save(); g.translate(x, y); g.rotate(rnd() * 6.28);
    g.fillStyle = `hsl(${h},${40 + rnd() * 25}%,${l}%)`;
    g.beginPath(); g.ellipse(0, 0, L, L * 0.5, 0, 0, 6.28); g.fill(); g.restore();
  }
  return tex(c, { repeat: true });
}

/** Facade with windows for distant buildings: tiled walls, floor bands, framed glazing reflecting sky, AC units. */
export function facadeTexture(seed = 1, tone = [190, 185, 175]) {
  const W = 512, H = 1024, [c, g] = canvas(W, H);
  const [r0, g0, b0] = tone;
  g.fillStyle = `rgb(${r0},${g0},${b0})`; g.fillRect(0, 0, W, H);
  // fine tile noise
  for (let i = 0; i < 12000; i++) { const v = (rnd() - 0.5) * 14; g.fillStyle = `rgba(${r0 + v},${g0 + v},${b0 + v},0.5)`; g.fillRect(rnd() * W, rnd() * H, 3, 2); }
  const cols = 4 + (seed % 3), rows = 12;
  const cw = W / cols, rh = H / rows;
  for (let r = 0; r < rows; r++) {
    // floor slab band
    g.fillStyle = `rgba(${r0 - 30},${g0 - 30},${b0 - 28},0.55)`; g.fillRect(0, r * rh + rh * 0.86, W, rh * 0.07);
    for (let k = 0; k < cols; k++) {
      const x = k * cw + cw * 0.14, y = r * rh + rh * 0.2, w = cw * 0.72, h = rh * 0.58;
      g.fillStyle = '#2a2f33'; g.fillRect(x - 3, y - 3, w + 6, h + 6);
      const grd = g.createLinearGradient(x, y, x + w * 0.3, y + h);
      const lit = rnd();
      grd.addColorStop(0, `rgb(${120 + lit * 60},${140 + lit * 55},${165 + lit * 50})`);
      grd.addColorStop(1, `rgb(${50 + lit * 30},${60 + lit * 30},${72 + lit * 30})`);
      g.fillStyle = grd; g.fillRect(x, y, w, h);
      g.fillStyle = 'rgba(30,34,38,0.9)'; g.fillRect(x + w / 2 - 2, y, 4, h); g.fillRect(x, y + h * 0.35, w, 3);
      if (rnd() < 0.3) { g.fillStyle = `rgba(230,225,210,${0.5 + rnd() * 0.3})`; g.fillRect(x + rnd() * w * 0.4, y + 2, w * 0.3, h * (0.4 + rnd() * 0.5)); } // curtains
      if (rnd() < 0.35) { g.fillStyle = '#d8d6d0'; g.fillRect(x + w + 4, y + h * 0.55, cw * 0.1, rh * 0.22); g.fillStyle = '#9a9894'; g.fillRect(x + w + 6, y + h * 0.6, cw * 0.06, rh * 0.12); } // AC unit
    }
  }
  // rain streaks / dirt from the roof edge
  for (let i = 0; i < 260; i++) { g.fillStyle = `rgba(55,48,40,${rnd() * 0.07})`; g.fillRect(rnd() * W, rnd() * H * 0.3, 2 + rnd() * 10, 40 + rnd() * 300); }
  return tex(c, { repeat: true });
}

/** Worn white road paint mask (alpha). */
export function paintWear() {
  const S = 512, [c, g] = canvas(S, S);
  g.fillStyle = '#fff'; g.fillRect(0, 0, S, S);
  for (let i = 0; i < 2500; i++) { g.fillStyle = `rgba(0,0,0,${rnd() * 0.35})`; g.beginPath(); g.arc(rnd() * S, rnd() * S, 1 + rnd() * 7, 0, 6.28); g.fill(); }
  for (let i = 0; i < 60; i++) { g.strokeStyle = `rgba(0,0,0,${0.1 + rnd() * 0.25})`; g.lineWidth = 2 + rnd() * 10; g.beginPath(); g.moveTo(rnd() * S, 0); g.lineTo(rnd() * S, S); g.stroke(); }
  return tex(c, { repeat: true, srgb: false });
}

/** Soft round puff for dust/smoke sprites (with some internal structure). */
export function puffTexture() {
  const S = 256, [c, g] = canvas(S, S);
  for (let i = 0; i < 40; i++) {
    const x = S / 2 + (rnd() - 0.5) * S * 0.4, y = S / 2 + (rnd() - 0.5) * S * 0.4, r = S * (0.12 + rnd() * 0.22);
    const grd = g.createRadialGradient(x, y, 0, x, y, r);
    grd.addColorStop(0, `rgba(255,255,255,${0.10 + rnd() * 0.1})`); grd.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = grd; g.beginPath(); g.arc(x, y, r, 0, 6.28); g.fill();
  }
  return tex(c, { srgb: false });
}

/** Faded red/white striped road barrier / bollard sleeves. */
export function redWhite() {
  const [c, g] = canvas(256, 256);
  for (let i = 0; i < 8; i++) { g.fillStyle = i % 2 ? '#f1f1ee' : '#c8211f'; g.fillRect(0, i * 32, 256, 32); }
  return tex(c, { repeat: true });
}
