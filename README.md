# JetCar Mini

JetCar Mini 是基于五菱宏光 MINI EV 的冷气推进、飞行着陆与软体动力学实验平台，包含双层非线性 MPC、
触地后的地面刹停，以及带规划轨迹的可视化回放。物理采用自研 XPBD 节点-梁晶格和 JS / Python 刚体孪生；渲染采用 three.js。
界面使用中文主标题“QQ飛車”和小字副标题“Jet-Car Mini”。项目原名“坠 · FALL”，原有坠落破坏仿真与短片工具继续保留。

架构、接口契约与分工见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)；坐标系、单位、推进模型与数据格式见
[`spec/README.md`](spec/README.md)。

## 快速开始

```sh
npm install
npm run precompute:all   # 生成 public/sims/*.sim（未入库，约 4 分钟）
npm run dev              # http://127.0.0.1:5173/?program=crash&preset=roof
npm run film -- program=crash preset=roof out=renders/x.mp4 w=1920 h=1080 samples=16   # 导出成片，需 dev 运行中
```

`?program=<id>` 选择节目：`crash`（默认，坠车查看器）、`flight`（冷气辅助逃逸/着陆）、`install`（喷口安装 motion design），
以及短片《直到用计算机模拟》的镜头 `opening`、`slam`。渲染用的服务器另开端口、冻结热更新：`FREEZE=1 npm run dev -- --port 5177`，工具加 `port=5177`。

### 冷气推进

```sh
uv venv python/.venv && uv pip install -p python/.venv -e 'python[test,rl]'   # Python 孪生（numpy；rl = torch + gymnasium）
npm test                                  # spec 生成物是否最新 + JS/Python 双向一致性 + pytest
npm run flight:bench -- experiments/configs/baseline-soft-landing.json --workers 6   # zero / pd / mppi 对比
npm run flight:run -- task=escape-soft-landing controller=pd preset=nose seed=0 out=x.json   # JS 侧出轨迹
npm run flight:run -- task=escape-soft-landing controller=dual preset=flat seed=0 out=/tmp/dual.json   # 双层非线性规划/跟踪（JS + 随项目携带的 WASM）
npm run flight:settle && npm run flight:sim:all   # 带套件的软体：settle + 软着陆/改出/硬着陆三段（约 4 分钟）
# http://127.0.0.1:5173/?program=flight&run=soft          软体飞行录制
# http://127.0.0.1:5173/?program=flight&traj=/experiments/runs/<...>/traj/<x>.json   刚体轨迹秒看（MPC/PPO 结果）
# http://127.0.0.1:5173/?program=install                  安装动画
npm run film:install && npm run film:flight     # 导出三段成片到 renders/
```

## 当前双层 MPC 案例

- `dual-plans-flat`：着陆与地面刹停，显示实际拖尾、规划、预测及探索轨迹。
- `dual-pad-flat`：flat / seed 0 定点降落；软体触地位置误差 **0.09 m**、水平速度 **0.224 m/s**，满足 1.5 m / 0.5 m/s 阈值。
- `dual-recovery-flat`：非凸回收，底部推进组关闭或 ≥ 40%，最小可用稳态推重比 > 1；配置与复现见 [双层控制文档](docs/DUAL_GUIDANCE.md#非凸回收dual-recovery--flat)。
- 可视化入口：`http://127.0.0.1:5173/?program=flight&run=dual-pad-flat&mode=follow&paused`。需要先按[复现说明](docs/DUAL_GUIDANCE.md#定点降落precision-pad--flat)生成录制；`public/sims/` 不纳入 Git。
- 仿真过程中闭环求解，浏览器播放已生成的录制；**在线仿真仍暂停实施**。默认不循环，支持暂停、重置、循环、跟随视角旋转与缩放。
- 实验结果使用[统一大字模板](src/programs/flight/result.js)，填入标题、指标和说明即可。保留当前“探索”名称与展示效果。

双层 MPC 使用随项目携带的 OSQP WASM，运行已有求解器不需要安装 acados 或 Emscripten。案例参数、指标和已知验证限制见
[`docs/DUAL_GUIDANCE.md`](docs/DUAL_GUIDANCE.md) 与 [`experiments/reports/dual-precision-pad/metrics.json`](experiments/reports/dual-precision-pad/metrics.json)。

## 代码地图

双层控制的模块划分、运行方式和支持边界见 [`docs/DUAL_GUIDANCE.md`](docs/DUAL_GUIDANCE.md)。

- `spec/`：跨语言唯一事实源（场景尺寸、坠落预设、10 个喷口与气罐、车辆参数、挑战任务）；`npm run spec` 重新生成 `spec/generated/`
- `src/app/`：外壳（上下文与资源懒加载、节目注册、播放条）；`src/programs/<id>/`：各节目
- `src/car/carPrep.js`：GLB → 物理晶格 + 渲染嵌入；部件质量/屈服/连接强度在 `PIECES`
- `src/softbody/solver.js`：XPBD 求解器（梁、接触、自碰撞、断裂、静止阻尼、外部作用器钩子）；`recording.js`：录制编码/回放
- `src/flight/`：冷气推进刚体孪生（`propulsion.js` 推进参考模型，`contract.js` 接口契约）；`python/`：Python 孪生、MPC/PPO、基准
- `src/render/`：`carRender.js` GPU 形变车身与材质；`stage.js` 灯光与后期；`effects.js` 尘土、火花等特效
- `src/playback/view.js`：在仿真时间 t 呈现一段录制（车身、碎片、特效、反射探针）
- `src/cinema/`：镜头序列（成片时间 → 仿真时间 + 机位）、镜头库、关键时刻分析、离线成片帧
- `src/scene/layout.js`：碰撞体与预设（数据来自 spec）；`world.js`：场景建模
- `tools/`：`sim/`（预计算、settle、短跑测试）、`film/`（成片、单帧、标记点、截图）、`audit/`（物理与材质审计）、`spec/`（生成）、`flight/`（各项实验的脚本）

## 注意事项

- 改了 `carPrep.js` 的晶格（包括三角形的顺序）后，要先跑 `npm run settle` 重新生成 `settled.bin`，再 `npm run spec` 并重算 sims。否则车一开场就会散架。
- 渲染专用的修正（比如三角形绕序）放在 `carRender.js`，不要改动物理输入。
- 导演机位全程 1:1 实时，不慢放（镜头可以通过 `dur`/`warp` 显式改变，默认不用）。
- 自检工具：`tools/film/frame.mjs` 渲染单帧，`tools/audit/penetration.mjs`、`render-penetration.mjs`、`rest-jitter.mjs` 检查穿地和静止抖动，`tools/audit/inspect-materials.mjs` 审计材质和绕序，`tools/sim/quick-run.mjs` 跑短时 A/B 仿真。
