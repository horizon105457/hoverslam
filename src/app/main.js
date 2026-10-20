// App shell: builds the shared context, starts the program named by ?program=<id> (default: crash) and runs
// the realtime loop. `window.app` is the automation surface used by tools/film/*.mjs:
//   app.ready / app.error, app.stage, app.program, app.frame(dt), app.film = program.film
import '../style.css';
import { createContext } from './context.js';
import { loadProgram, PROGRAM_IDS } from './programs.js';

const params = new URLSearchParams(location.search);
const app = (window.app = { ready: false, error: null, programs: PROGRAM_IDS });

async function boot() {
  const ctx = createContext(params);
  const id = params.get('program') || 'crash';
  document.body.classList.add(`program-${id}`);
  const Program = await loadProgram(id);
  const program = new Program(ctx);
  Object.assign(app, { ctx, stage: ctx.stage, program, frame: dt => program.frame(dt) });
  await program.init();
  app.film = program.film;
  ctx.ui.$('loading').classList.add('done');
  app.ready = true;
  if (params.has('capture')) return;
  let last = performance.now();
  const loop = () => {
    const now = performance.now(), dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    program.frame(dt);
    requestAnimationFrame(loop);
  };
  loop();
}
boot().catch(e => { console.error(e); app.error = e; document.getElementById('load-text').textContent = '出错：' + e.message; });
