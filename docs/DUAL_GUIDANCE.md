# 双层非线性飞行控制

`dual` 是新增的 JS 控制器，使用随项目携带的 OSQP WASM。现有 `mpc`、刚体孪生、软体仿真和轨迹格式保持兼容。普通用户仍只需要原有 Node/npm 环境；不需要安装 acados、Python 求解器或 Emscripten。

```sh
npm run flight:run -- task=escape-soft-landing controller=dual preset=flat seed=0 out=/tmp/dual-flat.json
node tools/sim/flight.mjs controller=dual preset=flat seed=0 dur=4.5 out=dual-flat outDir=/tmp/dual-soft
node tools/flight/bench.mjs experiments/configs/dual-landing.json out=/tmp/dual-example
```

刚体输出使用原 `coldgas-traj@1`，软体输出使用原 `.sim`。已有查看器继续读取这些文件。浏览器求解器本身可直接加载，但现有 flight 页面仍以轨迹/录制播放为主，本次没有将页面改为在线控制界面。

## 组织

| 模块 | 职责 |
| --- | --- |
| `src/flight/guidance/model.js` | 连续空中预测：六自由度、气体质量、实际阀门动态、喷口力和力矩；提供局部状态扰动与阶段雅可比 |
| `src/flight/guidance/optimizer.js` | 固定阶段时间的稀疏多重射击 SQP，非线性 rollout 线搜索，只接受目标下降的控制更新 |
| `src/flight/guidance/index.js` | 上层剩余飞行规划、时间候选选择，下层短预测窗口跟踪，按时间采样上层状态和阀门前馈 |
| `src/flight/guidance/defaults.json` | 两层规模、频率、权重和信赖域 |
| `src/flight/qp/` | OSQP C 桥接、ESM 封装和预编译 WASM |
| `vendor/osqp/`、`tools/solver/` | 固定版本源码、许可证和维护者复建入口 |

加载使用 `await loadController('dual')`；初始化后 `act(state)` 保持同步，适配现有两种仿真步进。其他控制器不加载 WASM。直接同步 `createController('dual')` 需此前完成异步加载。

上层首次使用 PD 的模拟轨迹作初值，随后复用已有计划。实际控制动作由两层优化结果产生；不是在 PD 输出上叠加一个名为 MPC 的修正。QP 求解失败会显式报告，不能当作有效优化结果。

默认上层为 16 段、最多 3 次 SQP，每 0.8 秒重规划；下层为 6 段、0.6 秒预测、1 次 SQP，每 5 个控制步重规划。上层保留首次选定的绝对着陆时刻，避免每次重新估计剩余时间而不断推迟着陆。终端状态通过加权代价逼近，尚不是硬等式约束；`statistics.lastUpperDiagnostics.terminalError` 可查看实际预测偏差。

## 数学与物理边界

状态为位置、速度、四元数、机体系角速度、剩余气体质量和十个阀门开度，共 24 维；优化局部增量用三维姿态扰动，共 23 维。控制变量为十个喷口的指令开度。预测使用原推进/质量参数，移除指令死区以获得闭阀附近可用的梯度；真实被控对象保留原死区。

每次 SQP 将阶段动力学线性化成等式约束，加入输入范围、状态信赖域、气体和阀门界，构成凸 QP。OSQP 只处理该 QP；非凸动力学的局部搜索在 JS 层。控制更新后重新积分完整非线性模型，用实际目标值决定是否接受。

本次面向空中着陆。接触、车身变形、轮地驱动仍由原仿真负责，不进入预测模型。上层时间目前采用有限个候选，不能称为连续自由终端时间优化。还没有发动机离散启停、障碍规避、翻转阶段约束或全局最优保证，因此不能将本实现表述为完整 Falcon 9 制导系统。扩展这些问题需要明确增加相应决策变量和约束。

悬停起飞与地面制动任务会明确拒绝。定点着陆任务会生成水平减速参考，但本轮没有验证定点任务的可达性。触地后关闭阀门，后续滑行和接触由原被控对象处理。

## 运行核验

采用少量实际运行检查，不新增测试框架。Node 和无头 Chrome 已运行同一预编译 QP；项目 Vite 构建通过。Chrome 中完整 `flat` 刚体闭环成功，耗时约 0.84 秒；浏览器与 Node 的结果只有浮点量级差异。

2026-10-27，本机、`escape-soft-landing`、seed=0，原默认 `mpc` 与新默认 `dual` 的结果如下。`dual` 通过原 worker 批处理入口运行；两者预测模型和目标并不相同，这组数据表示具体样例的计算开销与结果，不能作为相同优化问题的通用加速比。

| 预设 | 控制器 | 整段耗时 / s | 着陆成功 | 触地垂速 / m/s | 触地倾角 / ° | 耗气 / kg |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| flat | mpc | 27.77 | 是 | 1.135 | 1.532 | 42.55 |
| flat | dual | 0.61 | 是 | 0.879 | 0.299 | 43.34 |
| nose | mpc | 23.98 | 是 | 1.211 | 0.323 | 50.04 |
| nose | dual | 0.64 | 是 | 0.806 | 0.286 | 51.76 |

两例说明首版空中着陆闭环可用，且计算开销没有增加；尚未覆盖其他预设、随机初值及复杂机动。新控制器耗气略高，并不代表所有性能指标都优于原控制器。

软体 `flat` 原入口运行 4.5 秒仿真，耗时 142.01 秒，其中控制规划累计 0.56 秒（上层 3 次、下层 24 次）。3.03 秒触地，垂速 0.989 m/s、倾角 0.24°、角速度 0.008 rad/s；通过现有触地及随后 1 秒保持判据。车辆随后继续滑行，在 4.39 秒发生路缘/车身碰撞，记录中 `success=true` 与 `hull_strike_any=true` 同时成立，不能把任务判据通过理解为全程无碰撞。

原 MPC 的软体对照仅运行 3.5 秒仿真，耗时 140.63 秒、规划累计 23.56 秒；因保持窗口尚未结束，结果是 `hold_incomplete`。两段时长不同，不直接比较整段耗时或成功率。软体总耗时主要来自既有高频结构仿真，本次没有修改其步进与精度取舍。

本轮输出保存在 `/tmp/hoverslam-dual-final/`（两例刚体轨迹和汇总）、`/tmp/hoverslam-dual-soft-final/`（软体录制和指标）。这些是本机临时结果，长期复现以本页命令及 `experiments/configs/dual-landing.json` 为准。
