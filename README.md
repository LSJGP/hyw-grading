# hyw-grading

C++ 评分引擎（由原 `grading_mini` 迁出），对智能驾驶仿真结果逐帧评估安全性与规划合规性。

- **Proto**：不在此仓维护，见同级 `[../hyw-proto](../hyw-proto)`
- **构建**：`bazel build //src/entry:grading_main`
- **默认配置**：`config/metrics_default.json`

---

## 在整体系统中的位置

`hyw-sim` 负责闭环仿真（场景加载、规划、动力学积分）；`hyw-grading` 是独立进程/可执行文件，只消费仿真输出的帧数据，不参与仿真步进。

```
┌─────────────────────────────────────────────────────────────────┐
│                         hyw-sim                                 │
│  ScenarioBundle → WorldSimulator → FrameRecord (每帧)           │
│       │                              │                          │
│       │                              ├─ grading_convert         │
│       │                              │     FrameRecord →        │
│       │                              │     MetricFrameInput     │
│       │                              │                          │
│       ├─ online: StreamPipeWriter ───┼─→ popen → grading_main   │
│       │         (stdin JSON lines)  │      --stream             │
│       │                              │                          │
│       └─ offline: WriteSimLogJson ──┼─→ sim_log.json            │
│                                      │         │                │
└──────────────────────────────────────┼─────────┼────────────────┘
                                       │         │
                                       ▼         ▼
┌─────────────────────────────────────────────────────────────────┐
│                       hyw-grading                               │
│  grading_main → SimplePlanner → Grader → MetricManager → Metrics│
│                                      │                          │
│                                      └─→ grading_report.json    │
└─────────────────────────────────────────────────────────────────┘
         ▲
         │  hyw-proto（MetricFrameInput / SimLog / GradingReport）
         └────────────────────────────────────────────────────────
```

---

## 项目架构

### 分层设计


| 层级   | 目录                     | 职责                                                         |
| ---- | ---------------------- | ---------------------------------------------------------- |
| 入口   | `src/entry/`           | CLI 可执行文件：加载 SimLog、解析配置、驱动批处理/流式两种模式                      |
| 规划补全 | `src/planning/`        | 在评分前为缺失 `planning_command` 的帧填充期望速度（供 example 类 metric 使用） |
| 评分核心 | `src/grading/`         | `Grader` 门面、`MetricManager` 调度、DAG 依赖、工厂注册                 |
| 指标实现 | `src/grading/metrics/` | 具体 checker：example（速度/规划）与 safety（碰撞/车道/可行驶区域）             |
| 配置   | `config/`              | JSON 格式的 `GradingRunConfig`（启用哪些 metric、阈值参数）              |
| 测试数据 | `testdata/`            | 离线调试用的 SimLog / 旧版帧 JSON 样例                                |


### 单帧处理流程

1. **输入**：`MetricFrameInput`（Proto，见 `hyw-proto/proto/grading/metric_input.proto`）
2. **SimplePlanner**：若帧内尚无 `planning_command`，按当前速度与配置上限写入 `desired_speed_mps`
3. **Grader::ProcessFrame**：交给 `MetricManager::RunOneFrame`
4. **DAG 调度**：按拓扑序分层执行各 metric 的 `CalculateOneFrame`，结果写入环形历史 `Payload<MetricFrameOutput>`
5. **Finish**：各 metric 调用 `SummarizeResult` 汇总，生成 `GradingReport`

---

## 目录与文件说明

```
hyw-grading/
├── WORKSPACE / BUILD / .bazelrc     # Bazel 工作区；依赖 hyw-proto、Abseil、Protobuf、spdlog
├── config/
│   ├── metrics_default.json         # 默认启用全部 5 个 metric
│   └── metrics_collision_only.json  # 仅碰撞检查的精简配置
├── docs/
│   └── highD_dataset_guide.md       # highD 数据集获取说明（与仿真场景数据来源相关）
├── testdata/
│   ├── sample_sim_log.json          # SimLog JSON 样例
│   └── sample_frames.json           # 旧版纯帧数组 JSON（legacy 格式）
├── third_party/                     # fmt / spdlog 的 Bazel BUILD 封装
└── src/
    ├── entry/
    │   ├── grading_main.cc          # 主程序：批处理 / --stream 在线模式
    │   └── BUILD
    ├── planning/
    │   ├── simple_planner.{h,cc}    # 补全 planning_command
    │   └── BUILD
    └── grading/
        ├── grader.{h,cc}            # 对外 API：Init / ProcessFrame / Finish
        ├── metric_manager.{h,cc}    # 注册 metric、建 DAG、逐帧执行、生成报告
        ├── metric_base.{h,cc}       # 所有 metric 的抽象基类
        ├── metric_factory.{h,cc}    # 名称 → 实例 的工厂（单例）
        ├── metric_register.h        # REGISTER_METRIC 宏，静态注册
        ├── metric_scheduling_policy.{h,cc}  # DAGScheduler：依赖解析与拓扑分层
        ├── metric_result_payload.h  # 帧级输出历史队列
        ├── macros.h                 # 单例、错误传播宏
        ├── BUILD
        └── metrics/
            ├── example/
            │   ├── speed_checker.{h,cc}           # 自车速度是否超过阈值
            │   └── planning_limit_checker.{h,cc}  # 规划期望速度是否超过上限
            └── safety/
                ├── geometry.{h,cc}                # OBB、距离等几何工具
                ├── regulatory_collision_checker.* # NPC–ego 碰撞及法规豁免判定
                ├── lane_departure_checker.*       # 车道/路沿净空
                └── drivable_area_checker.*        # 可行驶区域净空
```

### 核心类


| 类               | 文件                           | 作用                                                                   |
| --------------- | ---------------------------- | -------------------------------------------------------------------- |
| `Grader`        | `grader.h`                   | 评分入口：按配置创建 metric、逐帧评分、输出报告                                          |
| `MetricManager` | `metric_manager.h`           | 管理 metric 实例与帧输出缓存，按 DAG 顺序执行                                        |
| `MetricBase`    | `metric_base.h`              | 定义 `Init` / `CalculateOneFrame` / `SummarizeResult` 接口；支持 metric 间依赖 |
| `MetricFactory` | `metric_factory.h`           | 通过 `REGISTER_METRIC` 静态注册表创建 metric                                  |
| `DAGScheduler`  | `metric_scheduling_policy.h` | 根据 `dependencies()` 生成无环执行计划                                         |
| `SimplePlanner` | `simple_planner.h`           | 评分侧轻量规划补全，不替代 sim 内真实 planner                                        |


### 已注册 Metric


| 名称                             | 类别      | 说明                                                |
| ------------------------------ | ------- | ------------------------------------------------- |
| `speed_checker`                | example | 检测 `vehicle_state.speed` 是否超过 `maxSpeedThreshold` |
| `planning_limit_checker`       | example | 检测 `planning_command.desired_speed_mps` 是否超过上限    |
| `regulatory_collision_checker` | safety  | 基于 ego/NPC OBB 检测碰撞，区分豁免场景（追尾慢车、强行加塞、逆行对向等）       |
| `lane_departure_checker`       | safety  | 依据 `road_context` 与地图边界检查车道/路沿净空                  |
| `drivable_area_checker`        | safety  | 检查 ego 包络是否在可行驶区域内                                |


新增 metric：继承 `MetricBase`，在 `.cc` 末尾 `REGISTER_METRIC(YourChecker, "your_checker")`，并在 `grading_main.cc` 的 `BuildMetricInitSpecs` 与 `src/entry/BUILD` 的 deps 中挂上。

---

## 与 hyw-sim 的联动

联动代码在 **hyw-sim** 侧，不在本仓：


| hyw-sim 文件                   | 作用                                                                                   |
| ---------------------------- | ------------------------------------------------------------------------------------ |
| `cpp/grading_convert.{h,cc}` | 将 `hyw_sim.proto.FrameRecord` + 静态地图 + 自车参数 转为 `grading_mini.proto.MetricFrameInput` |
| `cpp/grading_bridge.{h,cc}`  | 在线 pipe 写帧、离线写 SimLog、调用批处理 grading                                                  |
| `cpp/main.cc`                | 仿真主循环中挂载 hook，按 `--cpp-mode` 选择 online/offline/both                                  |
| `run_sim.py`                 | 透传 `--grading-bin`、`--grading-report`、`--metrics-config`                             |


### 数据转换要点

`ToMetricFrameInput` 映射关系（sim → grading）：

- `FrameRecord.ego` → `VehicleState`
- `FrameRecord.command.desired_speed_mps` → `PlanningCommand`
- `FrameRecord.npcs` → `repeated NpcState`
- `FrameRecord.road` → `RoadContext`
- `VehicleParams` → `EgoVehicleParams`
- **仅 frame_id == 0** 时附带完整 `StaticMap` → `SceneMap`（避免每帧重复传地图）

### 两种集成模式

**1. 在线模式（`--cpp-mode online`，默认）**

仿真每产生一帧 `FrameRecord`，`StreamPipeWriter` 将其转为 JSON 行写入 `grading_main --stream` 的 stdin；评分进程实时打印每帧 PASS/FAIL，仿真结束后写入 `grading_report.json`。

```bash
# 在 hyw-sim 目录
python3 run_sim.py \
  --scenario-dir ../scenarios/waymo_scenario_5 \
  --grading-bin ../hyw-grading/bazel-bin/src/entry/grading_main \
  --metrics-config ../hyw-grading/config/metrics_default.json
```

**2. 离线模式（`--cpp-mode offline`）**

仿真结束后将全量帧写入 `sim_log.json`（`SimLog` Proto 的 JSON 形式），再 shell 调用：

```text
grading_main [--metrics-config <json>] <sim_log.json> <grading_report.json>
```

**3. 两者兼有（`both`）**

仿真过程中 pipe 流式评分，结束后再用 SimLog 做一次批处理（便于对比或调试）。

### 与 sim 调试日志的区别


| 输出                                  | 内容                      | 用途                               |
| ----------------------------------- | ----------------------- | -------------------------------- |
| `output/log/sim_log.json`           | `MetricFrameInput` 序列   | grading 离线输入                     |
| `output/log/sim_*.log`              | `hyw_sim.proto` 调试 JSON | 仅仿真/planner 调试，**不含** grading 输入 |
| `output/report/grading_report.json` | `GradingReport`         | 各 metric 通过与否及说明                 |


---

## 构建与独立运行

### 构建

```bash
cd hyw-grading
bazel build //src/entry:grading_main
# 产物：bazel-bin/src/entry/grading_main
```

依赖同级目录 `../hyw-proto`（`WORKSPACE` 中 `local_repository`）。

### grading_main 用法

**批处理**（读完整个 SimLog 再出报告）：

```bash
./bazel-bin/src/entry/grading_main \
  --metrics-config config/metrics_default.json \
  testdata/sample_sim_log.json /tmp/grading_report.json
```

**流式**（stdin 每行一个 `MetricFrameInput` JSON）：

```bash
./bazel-bin/src/entry/grading_main --stream \
  --metrics-config config/metrics_default.json \
  /tmp/grading_report.json
```

支持的输入格式：

- `SimLog` JSON / 二进制 `.pb`
- 旧版纯帧数组 JSON（`testdata/sample_frames.json`，仅含自车 kinematics）

未指定 `--metrics-config` 时使用内置默认：planning_limit_checker、speed_checker、regulatory_collision_checker。

### 配置文件格式

`config/metrics_default.json` 对应 `GradingRunConfig`：

```json
{
  "simplePlannerMaxSpeedMps": 33.3,
  "spdlogLevel": "info",
  "metrics": [
    { "name": "speed_checker", "paramsJson": "{\"maxSpeedThreshold\": 33.3}" },
    { "name": "regulatory_collision_checker" }
  ]
}
```

---

## 相关文档

- Proto 字段详解：`[../hyw-proto/README.md](../hyw-proto/README.md)`
- 仿真启动参数与 grading 开关：`[../hyw-sim/启动参数说明.md](../hyw-sim/启动参数说明.md)`
- highD 场景数据：`[docs/highD_dataset_guide.md](docs/highD_dataset_guide.md)`

