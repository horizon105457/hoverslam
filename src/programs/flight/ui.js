// DOM of the flight viewer (mounted into #hud) + its own styles (scoped to body.program-flight).
export const FLIGHT_CSS = `
body.program-flight .controls { right: 22px; bottom: 22px; }
body.program-flight #fpanel { left: 22px; right: auto; top: 150px; width: 236px; max-height: calc(100vh - 420px); }
body.program-flight.cinema #fpanel { top: calc(6.5vh + 150px); max-height: calc(87vh - 420px); }
body.program-flight .flight-hud { position: fixed; inset: 0; width: 100%; height: 100%; pointer-events: none; z-index: 1; }
body.program-flight.clean .flight-hud { opacity: 0; }
body.program-flight #b-ui-restore { display: none; position: fixed; right: 22px; bottom: 22px; z-index: 3; align-items: center; gap: 7px; background: var(--glass); }
body.program-flight.clean #b-ui-restore { display: inline-flex; }
body.program-flight #b-ui svg { vertical-align: middle; }
body.program-flight.clean .flight-trajectories, body.program-flight.clean .visual-controls { visibility: hidden; }
body.program-flight .visual-controls { position: fixed; right: 28px; bottom: 92px; width: min(320px, calc(100vw - 56px)); z-index: 2; padding: 16px 18px; border: 1px solid var(--line); border-radius: 18px; background: var(--glass); font-size: 12px; }
body.program-flight .visual-controls[hidden] { display: none; }
body.program-flight .visual-controls h2 { margin: 0 0 14px; font-size: 14px; font-weight: 400; color: var(--dim); }
body.program-flight .visual-switches { display: flex; gap: 6px; }
body.program-flight.cinema .visual-controls { bottom: calc(6.5vh + 82px); }
body.program-flight #trajectory-legend { display: grid; grid-template-columns: 1fr 1fr; gap: 9px 12px; margin-top: 14px; font-size: 11px; pointer-events: none; }
body.program-flight #trajectory-legend[hidden] { display: none; }
body.program-flight .controls button:disabled { opacity: .4; cursor: default; }
body.program-flight .visual-controls button:disabled { opacity: .4; cursor: default; }
body.program-flight .runs { display: grid; grid-template-columns: 1fr; gap: 6px; margin-bottom: 10px; }
body.program-flight .runs button { text-align: left; padding: 8px 10px; font-size: 13px; }
body.program-flight .runs button small { display: block; font-size: 10px; opacity: .62; margin-top: 2px; }
body.program-flight .runs button.ok::after { content: ' ●'; color: var(--mint); }
body.program-flight .runs button.fail::after { content: ' ●'; color: var(--hot); }
body.program-flight .runs button.on.ok::after, body.program-flight .runs button.on.fail::after { color: #0d1714; }
body.program-flight #fnote { font-size: 11px; color: var(--dim); line-height: 1.5; margin: 6px 0 0; }
body.program-flight #fnote code { font-size: 10px; color: var(--fg); word-break: break-all; }
body.program-flight #t-marks span.ign { background: var(--mint); }
body.program-flight #t-marks span.td { background: #fff; width: 3px; }
body.program-flight .shot { left: auto; right: 36px; text-align: right; bottom: 104px; }
body.program-flight.cinema .shot { bottom: calc(6.5vh + 96px); }
`;

export const FLIGHT_UI = `
<header id="title">
  <h1>坠<span>FALL</span></h1>
  <p>冷气推力 · 飞出高架 · 着陆</p>
  <p class="sub" id="techline">10 喷口冷气推力套件 · XPBD 软体 · 闭环控制</p>
</header>
<aside id="fpanel" class="panel">
  <h2>飞行记录</h2>
  <div id="runs" class="runs"></div>
  <p id="fnote"></p>
</aside>
<button id="b-ui-restore" title="显示界面 (H)" aria-label="显示界面"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7S2 12 2 12Z"/><circle cx="12" cy="12" r="3"/></svg></button>`;

export const FLIGHT_GROUPS = {
  trajectories: '<button id="b-trail" aria-pressed="true" title="从起点到当前时刻的完整轨迹">轨迹</button><button id="b-plan" aria-pressed="false">规划</button><button id="b-explore" aria-pressed="false">探索</button>',
  modes: '<button data-mode="director" class="on">导演</button><button data-mode="free">自由</button><button data-mode="follow">跟随</button>',
  speeds: '<button data-speed="0.1">0.1×</button><button data-speed="0.25">0.25×</button><button data-speed="0.5">0.5×</button><button data-speed="1" class="on">1×</button>',
  view: '<button id="b-plume" class="on">喷流</button><button id="b-tele" class="on">遥测</button><button id="b-ui" title="隐藏界面 (H)" aria-label="隐藏界面"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m3 3 18 18M10.6 5.1 12 5c6.5 0 10 7 10 7a19 19 0 0 1-3.1 4.1M6.2 6.2A21 21 0 0 0 2 12s3.5 7 10 7a12 12 0 0 0 5.8-1.8M9.9 9.9a3 3 0 0 0 4.2 4.2"/></svg></button>',
};
