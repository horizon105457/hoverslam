// Opening film shots (program "opening"): pick one with ?shot=<id>. Each shot is a self-contained film
// (`film = {total, render(T), describe()}`) rendered with tools/film/film.mjs:
//   node tools/film/film.mjs program=opening q=shot=ghosts port=<p> out=renders/ghosts.mp4   (film: 直到用计算机模拟 段落/00_开头)
//   ghosts  hundreds of rigid-twin launches off the viaduct at once, drawn as translucent ghosts + trails
// Look-dev: devcam=px,py,pz,tx,ty,tz[,fov] fixes the camera (world coordinates), devt=<sim s> fixes sim time.
import { Program } from '../../app/program.js';

const SHOTS = {
  ghosts: () => import('./ghosts.js'),
  drive: () => import('./drive.js'),
  recon: () => import('./recon.js'),
};

export default class OpeningProgram extends Program {
  static meta = { id: 'opening', title: '开头 · 镜头' };

  async init() {
    const id = this.ctx.params.get('shot') || 'ghosts';
    const load = SHOTS[id];
    if (!load) throw new Error(`unknown shot "${id}" (have: ${Object.keys(SHOTS).join(', ')})`);
    const Shot = (await load()).default;
    this.shot = new Shot(this.ctx);
    await this.shot.init();
    this.film = this.shot.film;
    this.T = +(this.ctx.params.get('T') || 0);
    this.ctx.ui.setTitle(`${OpeningProgram.meta.title} · ${id}`);
  }

  frame(dt) {
    if (!this.film) return;
    if (!this.ctx.params.has('paused')) this.T = (this.T + dt) % this.film.total;
    this.shot.preview(this.T);
  }
}
