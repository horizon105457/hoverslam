// DOM of the flight viewer (mounted into #hud) + its own styles (scoped to body.program-flight).
export const FLIGHT_CSS = `
body.program-flight .controls { right: 22px; bottom: 22px; }
body.program-flight #fpanel { left: 22px; right: auto; top: 150px; width: 236px; max-height: calc(100vh - 420px); }
body.program-flight.cinema #fpanel { top: calc(6.5vh + 150px); max-height: calc(87vh - 420px); }
body.program-flight .flight-hud { position: fixed; inset: 0; width: 100%; height: 100%; pointer-events: none; z-index: 1; }
body.program-flight.clean .flight-hud.ui-only { opacity: 0; }
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
</aside>`;

export const FLIGHT_GROUPS = {
  modes: '<button data-mode="director" class="on">导演</button><button data-mode="free">自由</button><button data-mode="follow">跟随</button>',
  speeds: '<button data-speed="0.1">0.1×</button><button data-speed="0.25">0.25×</button><button data-speed="0.5">0.5×</button><button data-speed="1" class="on">1×</button>',
  view: '<button id="b-plume" class="on">喷流</button><button id="b-tele" class="on">遥测</button><button id="b-ui" title="隐藏界面 (H)">⤢</button>',
};
