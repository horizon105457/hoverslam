# 坠 · FALL

五菱宏光 MINI EV 冲出高架坠落的软体破坏仿真。物理：自研 XPBD 节点-梁晶格（离线预计算）；渲染：three.js。

## 快速开始

```sh
npm install
npm run precompute:all   # 生成 public/sims/*.sim（未入库，约 4 分钟）
npm run dev              # http://127.0.0.1:5173/?preset=roof
npm run film -- preset=roof out=renders/x.mp4 w=1920 h=1080 samples=16   # 导出成片，需 dev 运行中
```

## 代码地图

- `src/car/carPrep.js`：GLB → 物理晶格 + 渲染嵌入；部件质量/屈服/连接强度在 `PIECES`
- `src/sim/solver.js`：XPBD 求解器（梁、接触、自碰撞、断裂、静止阻尼）
- `src/sim/recording.js`：录制编码/回放
- `src/render/carRender.js`：GPU 形变车身与车身材质；`stage.js`：灯光与后期；`effects.js`：尘土、火花等特效
- `src/scene/layout.js`：场景尺寸、碰撞体、六种预设；`world.js`：场景建模
- `src/director.js`：导演镜头表，全程 1:1 实时，不慢放

## 注意事项

- 改了 `carPrep.js` 的晶格（包括三角形的顺序）后，要先跑 `npm run settle` 重新生成 `settled.bin`，再重算 sims。否则车一开场就会散架。
- 渲染专用的修正（比如三角形绕序）放在 `carRender.js`，不要改动物理输入。
- 自检工具：`tools/film-frame.mjs` 渲染单帧，`tools/phys-audit*.mjs` 检查穿地和静止抖动，`tools/inspect-materials.mjs` 审计材质和绕序，`tools/quick-run.mjs` 跑短时 A/B 仿真。
