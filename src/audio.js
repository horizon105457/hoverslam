// Procedural crash audio driven by the simulation event log. Everything is synthesised (no samples):
// impacts = low thump + resonant metal crunch, tempered glass = noise burst + hundreds of tiny pings,
// detaching parts = clanks, sliding metal = band-passed noise bed, falling = wind. Slow motion lowers
// pitch and stretches decays, like a high-speed camera recording.
export class CrashAudio {
  constructor() {
    this.ctx = null;
    this.enabled = false;
    this.lastT = null;
  }

  async start() {
    if (!this.ctx) {
      const ctx = (this.ctx = new AudioContext());
      this.master = ctx.createGain(); this.master.gain.value = 0.9;
      const comp = ctx.createDynamicsCompressor(); comp.threshold.value = -14; comp.ratio.value = 4;
      this.reverb = this.makeReverb(2.4);
      const wet = ctx.createGain(); wet.gain.value = 0.28;
      this.master.connect(comp); comp.connect(ctx.destination);
      this.master.connect(this.reverb); this.reverb.connect(wet); wet.connect(comp);
      this.noise = this.makeNoise(3);
      // continuous beds: scrape + wind + city ambience
      this.scrape = this.bed(2200, 1.4, 0); this.wind = this.bed(500, 0.6, 0); this.city = this.bed(180, 0.5, 0.05, 'lowpass');
    }
    await this.ctx.resume();
    this.enabled = true;
  }
  stop() { this.enabled = false; this.ctx?.suspend(); }

  makeNoise(sec) {
    const ctx = this.ctx, b = ctx.createBuffer(1, ctx.sampleRate * sec, ctx.sampleRate), d = b.getChannelData(0);
    let last = 0;
    for (let i = 0; i < d.length; i++) { const w = Math.random() * 2 - 1; last = (last + 0.02 * w) / 1.02; d[i] = w * 0.7 + last * 3; }
    return b;
  }
  makeReverb(sec) {
    const ctx = this.ctx, len = ctx.sampleRate * sec, b = ctx.createBuffer(2, len, ctx.sampleRate);
    for (let c = 0; c < 2; c++) { const d = b.getChannelData(c); for (let i = 0; i < len; i++) d[i] = (Math.random() * 2 - 1) * Math.pow(1 - i / len, 3); }
    const conv = ctx.createConvolver(); conv.buffer = b; return conv;
  }
  bed(freq, q, level, type = 'bandpass') {
    const ctx = this.ctx, src = ctx.createBufferSource(); src.buffer = this.noise; src.loop = true;
    const f = ctx.createBiquadFilter(); f.type = type; f.frequency.value = freq; f.Q.value = q;
    const g = ctx.createGain(); g.gain.value = level;
    src.connect(f); f.connect(g); g.connect(this.master); src.start();
    return { f, g };
  }

  // --- one-shots ------------------------------------------------------------------------------
  thump(strength, rate) {
    const ctx = this.ctx, t = ctx.currentTime, o = ctx.createOscillator(), g = ctx.createGain();
    const p = Math.sqrt(rate);
    o.frequency.setValueAtTime(95 * p, t); o.frequency.exponentialRampToValueAtTime(32 * p, t + 0.35 / p);
    g.gain.setValueAtTime(0, t); g.gain.linearRampToValueAtTime(0.9 * strength, t + 0.005); g.gain.exponentialRampToValueAtTime(0.001, t + 0.6 / Math.pow(rate, 0.6));
    o.connect(g); g.connect(this.master); o.start(t); o.stop(t + 1.5 / Math.pow(rate, 0.6));
    this.crunch(strength, rate);
  }
  crunch(strength, rate) {
    const ctx = this.ctx, t0 = ctx.currentTime, p = Math.sqrt(rate);
    const n = 3 + Math.round(strength * 6);
    for (let i = 0; i < n; i++) {
      const t = t0 + (Math.random() * 0.12) / Math.pow(rate, 0.7);
      const src = ctx.createBufferSource(); src.buffer = this.noise; src.playbackRate.value = 0.6 + Math.random() * 0.8;
      const f = ctx.createBiquadFilter(); f.type = 'bandpass'; f.frequency.value = (500 + Math.random() * 2500) * p; f.Q.value = 6 + Math.random() * 10;
      const g = ctx.createGain(); const dur = (0.05 + Math.random() * 0.18) / Math.pow(rate, 0.6);
      g.gain.setValueAtTime(0, t); g.gain.linearRampToValueAtTime(0.5 * strength, t + 0.003); g.gain.exponentialRampToValueAtTime(0.001, t + dur);
      src.connect(f); f.connect(g); g.connect(this.master); src.start(t, Math.random() * 2); src.stop(t + dur + 0.05);
    }
  }
  glass(strength, rate) {
    const ctx = this.ctx, t0 = ctx.currentTime, p = Math.sqrt(rate);
    const src = ctx.createBufferSource(); src.buffer = this.noise;
    const f = ctx.createBiquadFilter(); f.type = 'highpass'; f.frequency.value = 2500 * p;
    const g = ctx.createGain(); const dur = 0.25 / Math.pow(rate, 0.6);
    g.gain.setValueAtTime(0.45 * strength, t0); g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    src.connect(f); f.connect(g); g.connect(this.master); src.start(t0, Math.random()); src.stop(t0 + dur + 0.05);
    const pings = 40 + Math.round(60 * strength);
    for (let i = 0; i < pings; i++) {
      const t = t0 + (Math.pow(Math.random(), 2) * 0.9) / Math.pow(rate, 0.7);
      const o = ctx.createOscillator(), gg = ctx.createGain();
      o.type = 'sine'; o.frequency.value = (3000 + Math.random() * 7000) * p;
      const d = (0.02 + Math.random() * 0.06) / Math.pow(rate, 0.5);
      gg.gain.setValueAtTime(0, t); gg.gain.linearRampToValueAtTime(0.03 + Math.random() * 0.05, t + 0.001); gg.gain.exponentialRampToValueAtTime(0.0005, t + d);
      o.connect(gg); gg.connect(this.master); o.start(t); o.stop(t + d + 0.02);
    }
  }
  clank(rate) {
    const ctx = this.ctx, t = ctx.currentTime, p = Math.sqrt(rate);
    for (const fr of [310, 467, 733, 1180]) {
      const o = ctx.createOscillator(), g = ctx.createGain();
      o.type = 'triangle'; o.frequency.value = fr * (0.9 + Math.random() * 0.2) * p;
      const d = (0.25 + Math.random() * 0.35) / Math.pow(rate, 0.6);
      g.gain.setValueAtTime(0.12, t); g.gain.exponentialRampToValueAtTime(0.0005, t + d);
      o.connect(g); g.connect(this.master); o.start(t); o.stop(t + d + 0.05);
    }
  }

  /** Called every frame with the current sim time and the sim-seconds-per-wall-second rate. */
  update(t, rate, meta, bodyY) {
    if (!this.enabled || !meta?.events) { this.lastT = t; return; }
    const ctx = this.ctx, now = ctx.currentTime;
    const prev = this.lastT;
    this.lastT = t;
    rate = Math.max(0.03, Math.min(1, rate || 1));
    if (prev === null || t < prev || t - prev > 0.5) return; // scrubbing: stay silent
    const ev = meta.events;
    // impacts (dust events) – cluster to avoid machine-gunning
    const D = ev.dust || [];
    let hits = 0, maxS = 0;
    for (let i = 0; i < D.length; i += 7) if (D[i] > prev && D[i] <= t) { hits++; maxS = Math.max(maxS, D[i + 5]); }
    if (hits && now - (this._lastHit || 0) > 0.05) { this._lastHit = now; this.thump(Math.min(1, maxS / 12) * Math.min(1, 0.4 + hits * 0.05), rate); }
    for (const b of ev.breaks || []) {
      if (b.t <= prev || b.t > t) continue;
      if (b.type === 'glass' || b.type === 'crack') this.glass(b.type === 'glass' ? 1 : 0.6, rate);
      else if (b.type === 'lens') this.glass(0.35, rate);
      else if (b.type === 'detach' || b.type === 'latch') this.clank(rate);
    }
    // scrape bed: density of scrape events around t
    const S = ev.scrape || [];
    let sc = 0;
    for (let i = 0; i < S.length; i += 6) { const dt = S[i] - t; if (dt > -0.08 && dt < 0.02 && S[i + 5]) sc++; }
    const p = Math.sqrt(rate);
    this.scrape.g.gain.setTargetAtTime(Math.min(0.5, sc * 0.02), now, 0.05);
    this.scrape.f.frequency.setTargetAtTime(1800 * p, now, 0.1);
    // wind while airborne & fast
    const air = bodyY > 1.2 && t > 0 ? Math.min(1, (bodyY - 1.2) / 6) : 0;
    this.wind.g.gain.setTargetAtTime(0.18 * air * (t > 0.4 ? 1 : 0), now, 0.2);
    this.wind.f.frequency.setTargetAtTime(400 * p, now, 0.2);
  }
}
