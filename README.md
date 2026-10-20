# 坠 · FALL

五菱宏光 MINI EV 冲出高架坠落的软体破坏仿真，正在扩展为冷气推进（10 喷口，参考 Tesla Roadster "Rocket" 版）的
飞行/着陆控制实验平台。物理：自研 XPBD 节点-梁晶格（离线预计算）+ 刚体冷气推进孪生（JS / Python 同步）；渲染：three.js。

架构、接口契约与分工见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)；坐标系、单位、推进模型与数据格式见
[`spec/README.md`](spec/README.md)。

## 快速开始

```sh
npm install
npm run precompute:all   # 生成 public/sims/*.sim（未入库，约 4 分钟）
npm run dev              # http://127.0.0.1:5173/?program=crash&preset=roof
npm run film -- program=crash preset=roof out=renders/x.mp4 w=1920 h=1080 samples=16   # 导出成片，需 dev 运行中
```

`?program=<id>` 选择节目：`crash`（默认，坠车查看器）、`flight`（冷气辅助逃逸/着陆）、`install`（喷口安装 motion design）。

### 冷气推进

```sh
uv venv python/.venv && uv pip install -p python/.venv -e 'python[test,rl]'   # Python 孪生（numpy；rl = torch + gymnasium）
npm test                                  # spec 生成物是否最新 + JS/Python 双向一致性 + pytest
npm run flight:bench -- experiments/configs/baseline-soft-landing.json --workers 6   # zero / pd / mppi 对比
npm run flight:run -- task=escape-soft-landing controller=pd preset=nose seed=0 out=x.json   # JS 侧出轨迹
npm run flight:settle && npm run flight:sim:all   # 带套件的软体：settle + 软着陆/改出/硬着陆三段（约 4 分钟）
# http://127.0.0.1:5173/?program=flight&run=soft          软体飞行录制
# http://127.0.0.1:5173/?program=flight&traj=/experiments/runs/<...>/traj/<x>.json   刚体轨迹秒看（MPC/PPO 结果）
# http://127.0.0.1:5173/?program=install                  安装动画
npm run film:install && npm run film:flight     # 导出三段成片到 renders/
```

## 代码地图

- `spec/`：跨语言唯一事实源（场景尺寸、坠落预设、10 个喷口与气罐、车辆参数、挑战任务）；`npm run spec` 重新生成 `spec/generated/`
- `src/app/`：外壳（上下文与资源懒加载、节目注册、播放条）；`src/programs/<id>/`：各节目
- `src/car/carPrep.js`：GLB → 物理晶格 + 渲染嵌入；部件质量/屈服/连接强度在 `PIECES`
- `src/softbody/solver.js`：XPBD 求解器（梁、接触、自碰撞、断裂、静止阻尼、外部作用器钩子）；`recording.js`：录制编码/回放
- `src/flight/`：冷气推进刚体孪生（`propulsion.js` 推进参考模型，`contract.js` 接口契约）；`python/`：Python 孪生、MPC/PPO、基准
- `src/render/`：`carRender.js` GPU 形变车身与材质；`stage.js` 灯光与后期；`effects.js` 尘土、火花等特效
- `src/playback/view.js`：在仿真时间 t 呈现一段录制（车身、碎片、特效、反射探针）
- `src/cinema/`：镜头序列（成片时间 → 仿真时间 + 机位）、镜头库、关键时刻分析、离线成片帧
- `src/scene/layout.js`：碰撞体与预设（数据来自 spec）；`world.js`：场景建模
- `tools/`：`sim/`（预计算、settle、短跑测试）、`film/`（成片、单帧、截图）、`audit/`（物理与材质审计）、`spec/`（生成）、`dev/`（一次性分析脚本）

## 注意事项

- 改了 `carPrep.js` 的晶格（包括三角形的顺序）后，要先跑 `npm run settle` 重新生成 `settled.bin`，再 `npm run spec` 并重算 sims。否则车一开场就会散架。
- 渲染专用的修正（比如三角形绕序）放在 `carRender.js`，不要改动物理输入。
- 导演机位全程 1:1 实时，不慢放（镜头可以通过 `dur`/`warp` 显式改变，默认不用）。
- 自检工具：`tools/film/frame.mjs` 渲染单帧，`tools/audit/penetration.mjs`、`render-penetration.mjs`、`rest-jitter.mjs` 检查穿地和静止抖动，`tools/audit/inspect-materials.mjs` 审计材质和绕序，`tools/sim/quick-run.mjs` 跑短时 A/B 仿真。
