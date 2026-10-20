// Minimal SVG charts for the gas-study report (no dependencies): line / marker / band panels on a shared
// light/dark style, laid out as small multiples. Palette: the validated reference categorical order (dataviz skill;
// slots 3-5 sit below 3:1 on the light surface, so series are direct-labelled and every figure has a table in the
// report). Every marker carries a <title> (native tooltip when the SVG is opened on its own).
export const COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948'];
export const COLORS_DARK = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300', '#9085e9', '#e66767'];
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const f = x => +x.toFixed(2);

/** Clean tick values for [a, b]. */
export function ticks(a, b, n = 5) {
  const span = b - a || 1, raw = span / n, mag = 10 ** Math.floor(Math.log10(raw)), st = [1, 2, 2.5, 5, 10].map(k => k * mag).find(k => span / k <= n) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(a / st - 1e-9) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(10));
  return out;
}

function style(nSeries) {
  const light = COLORS.slice(0, nSeries).map((c, i) => `.s${i}{stroke:${c};fill:${c}}`).join('');
  const dark = COLORS_DARK.slice(0, nSeries).map((c, i) => `.s${i}{stroke:${c};fill:${c}}`).join('');
  return `<style>
svg{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;font-variant-numeric:tabular-nums}
.bg{fill:#fcfcfb}.t1{fill:#0b0b0b}.t2{fill:#52514e}.t3{fill:#898781}.grid{stroke:#e1e0d9;stroke-width:1}.axis{stroke:#c3c2b7;stroke-width:1}
.shade{fill:#0b0b0b;fill-opacity:0.05}.ref{stroke:#52514e;stroke-width:1;stroke-dasharray:4 3;fill:none}
.ln{fill:none!important;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}.band{stroke:none!important;fill-opacity:0.12}
.mk{stroke:#fcfcfb!important;stroke-width:2}.err{stroke-width:1.5;fill:none!important}
${light}
@media (prefers-color-scheme: dark){.bg{fill:#1a1a19}.t1{fill:#f0efec}.t2{fill:#c3c2b7}.grid{stroke:#2c2c2a}.axis{stroke:#383835}.shade{fill:#f0efec}.ref{stroke:#c3c2b7}.mk{stroke:#1a1a19!important}${dark}}
</style>`;
}

/**
 * One panel at (ox, oy) of size (w, h). p = {title, xlabel, ylabel, x: [min, max], y: [min, max], xticks?, yticks?,
 * xfmt?, yfmt?, series: [{name, cls (slot index), pts: [[x, y, tip?], ...], line, marker, dash, err: [[x, lo, hi]],
 * band: [[x, lo, hi]], label (direct label at the last point; true or text)}], shade: [[x0, x1, text]],
 * refs: [{x | y, text}], notes: [text]}
 */
export function panel(p, ox, oy, w, h) {
  if (p.legendBox) { // legend-only panel (small multiples share one legend)
    const o = p.title ? [`<text class="t1" x="${ox + 56}" y="${oy + 18}" font-size="13" font-weight="600">${esc(p.title)}</text>`] : [];
    p.legendBox.forEach((s, i) => { const y = oy + 50 + 22 * i; o.push(`<line class="ln s${s.cls ?? i}" x1="${ox + 56}" x2="${ox + 80}" y1="${y}" y2="${y}"/><text class="t2" x="${ox + 88}" y="${y + 4}" font-size="11.5">${esc(s.name)}</text>`); });
    (p.lines || []).forEach((t, i) => o.push(`<text class="t3" x="${ox + 56}" y="${oy + 66 + 22 * p.legendBox.length + 16 * i}" font-size="10.5">${esc(t)}</text>`));
    return o.join('\n');
  }
  const nNamed = (p.series || []).filter(s => s.name && s.legend !== false).length;
  const m = { l: 56, r: 16, t: (p.title ? 30 : 12) + (nNamed > 1 ? 16 : 0), b: 42 }, W = w - m.l - m.r, H = h - m.t - m.b;
  const [x0, x1] = p.x, [y0, y1] = p.y, X = x => ox + m.l + ((x - x0) / (x1 - x0)) * W, Y = y => oy + m.t + H - ((y - y0) / (y1 - y0)) * H;
  const xt = p.xticks || ticks(x0, x1, 6), yt = p.yticks || ticks(y0, y1, 5), xf = p.xfmt || (v => v), yf = p.yfmt || (v => v);
  const o = [];
  if (p.title) o.push(`<text class="t1" x="${ox + m.l}" y="${oy + 18}" font-size="13" font-weight="600">${esc(p.title)}</text>`);
  for (const [a, b, txt] of p.shade || []) {
    const xa = X(Math.max(a, x0)), xb = X(Math.min(b, x1));
    o.push(`<rect class="shade" x="${f(xa)}" y="${oy + m.t}" width="${f(Math.max(0, xb - xa))}" height="${H}"/>`);
    if (txt) o.push(`<text class="t3" x="${f((xa + xb) / 2)}" y="${oy + m.t + 12}" font-size="10" text-anchor="middle">${esc(txt)}</text>`);
  }
  for (const v of yt) o.push(`<line class="grid" x1="${ox + m.l}" x2="${ox + m.l + W}" y1="${f(Y(v))}" y2="${f(Y(v))}"/><text class="t3" x="${ox + m.l - 6}" y="${f(Y(v) + 3.5)}" font-size="10" text-anchor="end">${esc(yf(v))}</text>`);
  for (const v of xt) o.push(`<text class="t3" x="${f(X(v))}" y="${oy + m.t + H + 15}" font-size="10" text-anchor="middle">${esc(xf(v))}</text>`);
  o.push(`<line class="axis" x1="${ox + m.l}" x2="${ox + m.l + W}" y1="${oy + m.t + H}" y2="${oy + m.t + H}"/>`);
  if (p.xlabel) o.push(`<text class="t2" x="${ox + m.l + W / 2}" y="${oy + h - 8}" font-size="11" text-anchor="middle">${esc(p.xlabel)}</text>`);
  if (p.ylabel) o.push(`<text class="t2" transform="translate(${ox + 13},${oy + m.t + H / 2}) rotate(-90)" font-size="11" text-anchor="middle">${esc(p.ylabel)}</text>`);
  for (const r of p.refs || []) {
    if (r.y != null) o.push(`<line class="ref" x1="${ox + m.l}" x2="${ox + m.l + W}" y1="${f(Y(r.y))}" y2="${f(Y(r.y))}"/><text class="t2" x="${r.left ? ox + m.l + 4 : ox + m.l + W - 4}" y="${f(Y(r.y) + (r.below ? 13 : -4))}" font-size="10"${r.left ? '' : ' text-anchor="end"'}>${esc(r.text || '')}</text>`);
    else o.push(`<line class="ref" x1="${f(X(r.x))}" x2="${f(X(r.x))}" y1="${oy + m.t}" y2="${oy + m.t + H}"/><text class="t2" x="${f(X(r.x) + (r.left ? -4 : 4))}" y="${r.bottom ? oy + m.t + H - 8 : oy + m.t + 24}" font-size="10"${r.left ? ' text-anchor="end"' : ''}>${esc(r.text || '')}</text>`);
  }
  const clipY = y => Math.max(y0, Math.min(y1, y));
  (p.series || []).forEach(s => {
    const c = `s${s.cls ?? 0}`, pts = s.pts.filter(q => q[1] != null && Number.isFinite(q[1]));
    if (s.band?.length) o.push(`<path class="band ${c}" d="M${s.band.map(q => `${f(X(q[0]))},${f(Y(clipY(q[2])))}`).join('L')}L${[...s.band].reverse().map(q => `${f(X(q[0]))},${f(Y(clipY(q[1])))}`).join('L')}Z"/>`);
    for (const [x, lo, hi] of s.err || []) o.push(`<line class="err ${c}" x1="${f(X(x))}" x2="${f(X(x))}" y1="${f(Y(clipY(lo)))}" y2="${f(Y(clipY(hi)))}"/>`);
    if (s.line !== false && pts.length > 1) o.push(`<path class="ln ${c}"${s.dash ? ` stroke-dasharray="${s.dash}"` : ''} d="M${pts.map(q => `${f(X(q[0]))},${f(Y(clipY(q[1])))}`).join('L')}"/>`);
    if (s.marker) for (const q of pts) o.push(`<circle class="mk ${c}" cx="${f(X(q[0]))}" cy="${f(Y(clipY(q[1])))}" r="${s.r ?? 4.5}"${s.hollow && q[3] ? ' fill-opacity="0.25"' : ''}><title>${esc(q[2] ?? `${s.name}: ${q[0]}, ${f(q[1])}`)}</title></circle>`);
    if (s.label && pts.length) { const q = pts[Math.min(s.labelAt ?? pts.length - 1, pts.length - 1)], end = s.anchor === 'end'; o.push(`<text class="t1" x="${f(X(q[0]) + (end ? -7 : 7))}" y="${f(Y(clipY(q[1])) + (s.dy ?? 3.5))}" font-size="10.5"${end ? ' text-anchor="end"' : ''}>${esc(s.label === true ? s.name : s.label)}</text>`); }
  });
  // legend (>= 2 series): one row under the title
  const named = (p.series || []).filter(s => s.name && s.legend !== false);
  if (named.length > 1) {
    let lx = ox + m.l + (p.legendX ?? 0);
    const ly = oy + m.t - 10 + (p.legendY ?? 0);
    for (const s of named) {
      o.push(`<line class="ln s${s.cls ?? 0}" x1="${f(lx)}" x2="${f(lx + 14)}" y1="${ly}" y2="${ly}"${s.dash ? ` stroke-dasharray="${s.dash}"` : ''}/><text class="t2" x="${f(lx + 18)}" y="${ly + 3.5}" font-size="10">${esc(s.name)}</text>`);
      lx += 30 + [...s.name].reduce((w, ch) => w + (ch.charCodeAt(0) > 0x2e80 ? 10 : 5.6), 0);
    }
  }
  for (const [i, n] of (p.notes || []).entries()) o.push(`<text class="t3" x="${ox + m.l + W - 4}" y="${p.notesTop ? oy + m.t + 14 + 13 * i : oy + m.t + H - 6 - 13 * i}" font-size="10" text-anchor="end">${esc(n)}</text>`);
  return o.join('\n');
}

/** Small multiples: panels laid out in `cols` columns, each pw × ph; optional figure title / caption. */
export function figure(panels, { cols = 2, pw = 420, ph = 300, title = null, caption = null } = {}) {
  const rows = Math.ceil(panels.length / cols), top = title ? 30 : 0, bot = caption ? 22 : 0, W = cols * pw, H = rows * ph + top + bot;
  const n = Math.max(1, ...panels.flatMap(p => (p.series || []).map(s => (s.cls ?? 0) + 1)));
  const body = panels.map((p, i) => panel(p, (i % cols) * pw, top + Math.floor(i / cols) * ph, pw, ph)).join('\n');
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" role="img">${style(n)}
<rect class="bg" width="${W}" height="${H}"/>
${title ? `<text class="t1" x="12" y="20" font-size="15" font-weight="600">${esc(title)}</text>` : ''}
${body}
${caption ? `<text class="t3" x="12" y="${H - 7}" font-size="10.5">${esc(caption)}</text>` : ''}
</svg>\n`;
}
