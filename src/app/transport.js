// Generic playback bar: play/pause, a scrubbable timeline with event marks, a time label and slots for
// program-specific button groups. Programs own the clock; the bar only reports clicks and shows progress.
export class Transport {
  /** @param {{onPlay?:()=>void, onSeek?:(u:number)=>void}} handlers */
  constructor(ui, { onPlay, onSeek } = {}) {
    ui.mount(`<footer id="controls" class="controls">
      <button id="b-play" title="播放/暂停 (空格)">❚❚</button>
      <div id="timeline" class="timeline"><div class="track"><i id="t-fill"></i><b id="t-head"></b></div><div id="t-marks"></div></div>
      <span id="t-label" class="tlabel">0.00 s</span>
    </footer>`);
    this.el = ui.$('controls');
    this.$ = ui.$;
    this.play = ui.$('b-play');
    this.play.onclick = () => onPlay?.();
    const tl = ui.$('timeline');
    const seek = e => { const r = tl.getBoundingClientRect(); onSeek?.(Math.min(1, Math.max(0, (e.clientX - r.left) / r.width))); };
    tl.onpointerdown = e => { seek(e); tl.setPointerCapture(e.pointerId); tl.onpointermove = seek; };
    tl.onpointerup = () => { tl.onpointermove = null; };
  }
  setPlaying(on) { this.play.textContent = on ? '❚❚' : '▶'; }
  setProgress(u) { const p = `${(u * 100).toFixed(3)}%`; this.$('t-fill').style.width = p; this.$('t-head').style.left = p; }
  setLabel(text) { this.$('t-label').textContent = text; }
  /** @param {Array<{u:number, cls?:string}>} marks */
  setMarks(marks) {
    const box = this.$('t-marks'); box.innerHTML = '';
    for (const m of marks) { const s = document.createElement('span'); if (m.cls) s.className = m.cls; s.style.left = `${(m.u * 100).toFixed(2)}%`; box.appendChild(s); }
  }
  /** Append a button group (HTML of buttons) and return its element. */
  addGroup(html, id) {
    const g = document.createElement('div'); g.className = 'group'; if (id) g.id = id; g.innerHTML = html;
    this.el.appendChild(g);
    return g;
  }
}
