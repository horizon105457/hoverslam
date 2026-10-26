// Generic playback bar: play/pause, optional reset/loop controls, a scrubbable timeline with event marks,
// a time label and slots for program-specific button groups. Programs own the clock.
export class Transport {
  /** @param {{onPlay?:()=>void, onSeek?:(u:number)=>void, onReset?:()=>void, onLoop?:(on:boolean)=>void}} handlers */
  constructor(ui, { onPlay, onSeek, onReset, onLoop } = {}) {
    ui.mount(`<footer id="controls" class="controls">
      ${onReset ? '<button id="b-reset" title="重置进度" aria-label="重置进度"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 11a9 9 0 1 1 2.6 6.4M3 4v7h7"/></svg></button>' : ''}
      <button id="b-play" title="暂停播放" aria-label="暂停播放"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M8 5v14M16 5v14"/></svg></button>
      ${onLoop ? '<button id="b-loop" title="开启循环" aria-label="开启循环" aria-pressed="false"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m17 2 4 4-4 4M3 6h18M7 22l-4-4 4-4m14 4H3"/></svg></button>' : ''}
      <div id="timeline" class="timeline"><div class="track"><i id="t-fill"></i><b id="t-head"></b></div><div id="t-marks"></div></div>
      <span id="t-label" class="tlabel">0.00 s</span>
    </footer>`);
    this.el = ui.$('controls');
    this.$ = ui.$;
    this.play = ui.$('b-play');
    this.play.onclick = () => onPlay?.();
    this.looping = false;
    if (onReset) ui.$('b-reset').onclick = () => onReset();
    if (onLoop) {
      this.loop = ui.$('b-loop');
      this.loop.onclick = () => {
        const on = !this.looping;
        onLoop(on);
        this.setLooping(on);
      };
    }
    const tl = ui.$('timeline');
    this.timeline = tl;
    const seek = e => { if (this.seekEnabled) { const r = tl.getBoundingClientRect(); onSeek?.(Math.min(1, Math.max(0, (e.clientX - r.left) / r.width))); } };
    tl.onpointerdown = e => { if (!this.seekEnabled) return; seek(e); tl.setPointerCapture(e.pointerId); tl.onpointermove = seek; };
    tl.onpointerup = () => { tl.onpointermove = null; };
    this.setSeekEnabled(true);
  }
  setPlaying(on) {
    this.play.innerHTML = on
      ? '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M8 5v14M16 5v14"/></svg>'
      : '<svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true"><path d="m8 5 12 7-12 7z"/></svg>';
    const title = on ? '暂停播放' : '继续播放';
    this.play.title = title;
    this.play.setAttribute('aria-label', title);
  }
  setLooping(on) {
    this.looping = !!on;
    if (this.loop) {
      this.loop.setAttribute('aria-pressed', String(this.looping));
      this.loop.classList.toggle('on', this.looping);
      const title = this.looping ? '关闭循环' : '开启循环';
      this.loop.title = title;
      this.loop.setAttribute('aria-label', title);
    }
  }
  setSeekEnabled(on) {
    this.seekEnabled = !!on;
    this.timeline.classList.toggle('disabled', !this.seekEnabled);
    this.timeline.setAttribute('aria-disabled', String(!this.seekEnabled));
    this.timeline.title = this.seekEnabled ? '' : '不可拖动';
    this.timeline.style.opacity = this.seekEnabled ? '' : '.45';
    this.timeline.style.cursor = this.seekEnabled ? '' : 'not-allowed';
  }
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
