# PDMS 评分使用与说明

PDMS（**P**lanning **D**riving **M**etric **S**core）是 hyw-grading 中的综合驾驶质量评分，将 7 个子指标聚合为单一数值，用于判断一次仿真是否「整体达标」。实现位于 `src/grading/metrics/safety/pdms_aggregator.cc`，默认配置见 `config/metrics_default.json`。

---

## 1. 在系统中的位置

```
hyw-sim 仿真
    │
    ├─ 逐帧 FrameRecord → grading_convert → MetricFrameInput
    │
    └─ grading_main（在线 --stream 或离线 sim_log.json）
            │
            ├─ 7 个子 metric 逐帧计算
            │
            └─ pdms_aggregator 汇总 → GradingReport.pdms_score
```

当配置中启用 `pdms_aggregator` 时：

- `GradingReport.overall_passed` **仅由 PDMS 是否达标决定**，不再要求每个子 metric 各自 PASS。
- `GradingReport.pdms_score` 与 `pdms_subscores` 会写入 `summary.json`。

---

## 2. 评分公式

PDMS 由两部分相乘得到：**硬性惩罚项** × **加权平均项**。

```
penalties = NC × DAC × SL
weighted_avg = (w_EP·EP + w_TTC·TTC + w_C·C + w_Speed·Speed) / (w_EP + w_TTC + w_C + w_Speed)
PDMS = penalties × weighted_avg
```

| 符号 | 含义 | 取值范围 | 聚合方式 |
|------|------|----------|----------|
| **NC** | No Collision，法规碰撞 | 0 或 1 | 全程逐帧取 **最小值**（任一帧有责碰撞 → 0） |
| **DAC** | Drivable Area Compliance，可行驶区域 | 0 或 1 | 全程逐帧取 **最小值** |
| **SL** | Solid Line，压实线 | 0 或 1 | 全程逐帧取 **最小值** |
| **EP** | Ego Progress，到达终点进度 | [0, 1] | 取 **最后一帧有效评估** 的分数 |
| **TTC** | Time To Collision，碰撞风险 | [0, 1] | 全程逐帧 **通过率**（pass 帧占比） |
| **C** | Comfort，急刹舒适度 | [0, 1] | 全程逐帧 **通过率** |
| **Speed** | 超速 | [0, 1] | 全程逐帧 **通过率** |

**通过判定**：`PDMS >= pass_threshold`（默认 **0.95**）。

**设计意图**：

- NC / DAC / SL 属于「一票否决」类安全项：任意一帧违规，对应项变为 0，整体 PDMS 直接归零。
- EP / TTC / C / Speed 属于「软指标」：允许部分帧不达标，通过加权平均反映整体表现。
- EP 使用末帧分数，强调仿真结束时是否到达 SDC 终点线。

---

## 3. 子指标说明

### 3.1 NC — `regulatory_collision_checker`

检测 ego 与 NPC 的 OBB 几何碰撞，并区分法规豁免场景。

| 帧输出 | `bool_value = true` 表示该帧 pass |
|--------|-----------------------------------|
| 有责碰撞 | `false` |
| 无碰撞，或碰撞但豁免 | `true` |

**豁免情形**（碰撞几何成立，但不计责）：

| 场景 | 条件概要 |
|------|----------|
| 追尾慢车 | ego 在 NPC 后方，ego 速度 < 5 m/s，NPC 相对接近速度 > 2 m/s |
| 强行加塞 | 侧向碰撞，NPC 横向速度指向 ego 车道 |
| 逆行对向 | 前方对向碰撞，接近角 > 135° |

NC 在 PDMS 中取全程 `min`：只要有一帧 `bool_value = false`，NC = 0。

---

### 3.2 DAC — `drivable_area_checker`

检查 ego 是否在地图 `road_edges` 围成的可行驶区域内。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `minClearanceM` | 0.35 | 到路沿的最小净空（米） |
| `checkCenterOnly` | false | false 时检查 OBB 四角；true 时仅检查中心点 |

**依赖**：`scene_map.road_edges`（frame 0 传入，后续帧复用）。无地图时跳过帧，该帧视为 pass，但不影响已违规帧的 min 统计。

---

### 3.3 SL — `solid_line_crossing_checker`

检测 ego OBB 边是否与地图中的**实线**（`road_lines`）相交。

**依赖**：`scene_map.road_lines`。无地图时跳过帧。

---

### 3.4 EP — `ego_progress_checker`

衡量 ego 沿 SDC（Self-Driving Car）参考路线向终点的推进程度。

**依赖**：`sdc_route`（`start` / `end` 位姿），由 hyw-sim 在 frame 0 从场景 SDC track 提取并写入 `MetricFrameInput`。

**单帧分数计算**：

1. 将 ego 投影到 start→end 线段，得沿程比例 `along_ratio ∈ [0, 1]`。
2. 若 ego 已越过终点线（沿终点朝向前方 ≥ 0），分数 = **1.0**。
3. 否则分数 = `along_ratio ^ progress_exponent`（默认指数 **2.0**，靠近终点前分数增长较慢）。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `progressExponent` | 2.0 | 未达终点时的曲线指数 |
| `passThreshold` | 0.95 | 子 metric 自身通过阈值（不影响 PDMS 公式，仅 EP 独立 summary） |

PDMS 取 **最后一帧带 `EgoProgressCheckerCustomInfo` 的 `frame_score`**。无 `sdc_route` 时 EP = 0，会显著拉低 PDMS。

---

### 3.5 TTC — `collision_risk_checker`

评估与相关前方 NPC 的碰撞风险（TTC、净空、闭合速度），判定**尚未碰撞但逼近过快**的风险。

| 帧输出 | `bool_value = true` = 该帧无危险 |
|--------|----------------------------------|

与 `regulatory_collision_checker` 分工：后者判**已发生碰撞**；本项判**风险逼近**。

详细算法与参数见 [`features.md`](../features.md) 中 `collision_risk_checker` 章节。

PDMS 中 TTC = 全部有效帧的 pass 比例。

---

### 3.6 C — `hard_braking_checker`

检测急刹：速度 ≥ `minSpeedMps` 且纵向加速度 < `-minDecelThresholdMps2`。

| 参数 | 默认值 |
|------|--------|
| `minDecelThresholdMps2` | 4.0 |
| `minSpeedMps` | 1.0 |

PDMS 中 C = 无急刹帧占比。

---

### 3.7 Speed — `speed_checker`

检测 `vehicle_state.speed` 是否超过 `maxSpeedThreshold`。

| 参数 | 默认值 |
|------|--------|
| `maxSpeedThreshold` | 33.3 m/s（与 `simplePlannerMaxSpeedMps` 一致） |

PDMS 中 Speed = 未超速帧占比。

---

## 4. 配置与启用

### 4.1 默认配置

`config/metrics_default.json` 已包含 PDMS 所需的全部 8 个 metric（7 个子项 + 聚合器）：

```json
{
  "simplePlannerMaxSpeedMps": 33.3,
  "spdlogLevel": "info",
  "metrics": [
    { "name": "speed_checker", "paramsJson": "{\"maxSpeedThreshold\": 33.3}" },
    { "name": "regulatory_collision_checker" },
    {
      "name": "collision_risk_checker",
      "paramsJson": "{\"pathLateralTolM\": 1.0, \"maxLongitudinalM\": 40.0, \"maxLateralM\": 1.2, \"minClosingSpeedMps\": 0.5, \"minForwardCos\": 0.94, \"pathNpcWidthRatio\": 0.25}"
    },
    {
      "name": "drivable_area_checker",
      "paramsJson": "{\"minClearanceM\": 0.35, \"checkCenterOnly\": false}"
    },
    { "name": "solid_line_crossing_checker" },
    {
      "name": "hard_braking_checker",
      "paramsJson": "{\"minDecelThresholdMps2\": 4.0, \"minSpeedMps\": 1.0}"
    },
    {
      "name": "ego_progress_checker",
      "paramsJson": "{\"progressExponent\": 2.0, \"passThreshold\": 0.95}"
    },
    {
      "name": "pdms_aggregator",
      "paramsJson": "{\"weightEp\": 5, \"weightTtc\": 5, \"weightC\": 2, \"weightSpeed\": 0.5, \"passThreshold\": 0.95}"
    }
  ]
}
```

### 4.2 `pdms_aggregator` 可调参数

对应 proto `PdmsAggregatorConfig`（`hyw-proto/proto/grading/metrics/safety_metric.proto`）：

| JSON 字段 | proto 字段 | 默认值 | 说明 |
|-----------|------------|--------|------|
| `weightEp` | `weight_ep` | 5.0 | EP 权重 |
| `weightTtc` | `weight_ttc` | 5.0 | TTC 权重 |
| `weightC` | `weight_c` | 2.0 | 急刹权重 |
| `weightSpeed` | `weight_speed` | 0.5 | 超速权重 |
| `passThreshold` | `pass_threshold` | 0.95 | PDMS 通过分数线 |

权重仅影响 EP / TTC / C / Speed 四项的相对重要性；NC / DAC / SL 始终为乘法惩罚，无独立权重。

> **注意**：配置中权重字段必须 > 0 才会覆盖默认值；设为 0 或省略则保持内置默认。

### 4.3 依赖关系

`pdms_aggregator` 声明对以下 7 个 metric 的硬依赖，MetricManager 会按 DAG 拓扑序先执行子项再汇总：

```
regulatory_collision_checker ─┐
drivable_area_checker ────────┤
solid_line_crossing_checker ──┼──→ pdms_aggregator
ego_progress_checker ───────┤
collision_risk_checker ───────┤
hard_braking_checker ─────────┤
speed_checker ────────────────┘
```

**启用 `pdms_aggregator` 时必须同时配置上述 7 个子 metric**，否则依赖解析或汇总会不完整。

---

## 5. 使用方法

### 5.1 与 hyw-sim 联调（推荐）

在 `hyw-sim` 目录运行仿真，指定 grading 二进制与 PDMS 配置：

```bash
cd hyw-sim
python3 run_sim.py \
  --scenario-dir ../scenarios/waymo_scenario_5 \
  --grading-bin ../hyw-grading/bazel-bin/src/entry/grading_main \
  --metrics-config ../hyw-grading/config/metrics_default.json
```

在线模式（默认 `--cpp-mode online`）每帧 pipe 到 `grading_main --stream`；结束后在 `output/report/<时间>_<场景>/` 生成报告。

### 5.2 离线批处理

```bash
cd hyw-grading
bazel build //src/entry:grading_main

./bazel-bin/src/entry/grading_main \
  --metrics-config config/metrics_default.json \
  testdata/sample_sim_log.json /tmp/grading_report
```

### 5.3 构建 grading

```bash
cd hyw-grading
bazel build //src/entry:grading_main
```

---

## 6. 输出解读

### 6.1 终端输出

```
=== Result: PASSED === (90 frames)
  PDMS: 0.97
  regulatory_collision_checker: PASS (non_exempt=0 exempt=0 ...)
  ...
  pdms_aggregator: PASS (pdms=0.97 penalties=1 weighted_avg=0.97 ...)
```

`overall_passed` 在启用 PDMS 时等于 `pdms_aggregator.passed`。

### 6.2 `summary.json` 关键字段

```json
{
  "overallPassed": true,
  "pdmsScore": 0.97,
  "pdmsSubscores": [
    { "name": "NC", "score": 1.0 },
    { "name": "DAC", "score": 1.0 },
    { "name": "SL", "score": 1.0 },
    { "name": "EP", "score": 0.92 },
    { "name": "TTC", "score": 1.0 },
    { "name": "C", "score": 0.95 },
    { "name": "Speed", "score": 1.0 }
  ],
  "summaries": [ ... ]
}
```

### 6.3 `pdms_aggregator` 的 detail 字符串

示例：

```
pdms=0.97 penalties=1 weighted_avg=0.97 frames=90 NC=1 DAC=1 SL=1 EP=0.92(final_frame) TTC=1 C=0.95 Speed=1
```

| 字段 | 含义 |
|------|------|
| `penalties` | NC × DAC × SL，为 0 时 PDMS 必为 0 |
| `weighted_avg` | 四项软指标加权平均 |
| `frames` | 参与聚合的最大帧数 |
| `EP=...(final_frame)` | 末帧有效 EP 分数 |

### 6.4 数值算例

假设：

- NC=1, DAC=1, SL=1 → penalties=1
- EP=0.8, TTC=1.0, C=0.9, Speed=1.0
- 默认权重 5 : 5 : 2 : 0.5

```
weighted_avg = (5×0.8 + 5×1.0 + 2×0.9 + 0.5×1.0) / 12.5
             = 11.3 / 12.5 = 0.904
PDMS = 0.904  → 未达 0.95，FAIL
```

若 SL=0（曾压实线）：

```
PDMS = 0 × weighted_avg = 0  → FAIL
```

---

## 7. 数据前置条件

PDMS 完整生效需要仿真侧提供足够输入：

| 子指标 | 所需输入 | 来源 |
|--------|----------|------|
| NC, TTC | ego / NPC 位姿与速度 | 每帧 `FrameRecord` |
| TTC（策略 C） | `planned_trajectory` | sim planner 短期轨迹 |
| DAC, SL | `scene_map`（路沿、车道线） | frame 0 静态地图 |
| EP | `sdc_route`（起终点） | 场景 SDC track，frame 0 |
| C, Speed | `vehicle_state` 速度、加速度 | 每帧 ego 状态 |

Waymo 等带 SDC track 的场景会自动提取 `sdc_route`；无 SDC 时 EP 恒为 0，PDMS 通常无法达标。

---

## 8. 调参建议

| 目标 | 建议 |
|------|------|
| 更重视到达终点 | 增大 `weightEp` |
| 更重视避撞风险 | 增大 `weightTtc`；同时检查 `collision_risk_checker` 参数 |
| 更重视乘坐舒适性 | 增大 `weightC`，或降低 `minDecelThresholdMps2` |
| 放宽整体通过线 | 降低 `passThreshold`（如 0.90） |
| 仅看安全、不看进度 | 临时去掉 `pdms_aggregator`，改用各子 metric 独立 PASS/FAIL |

TTC 误报/漏报调参详见 [`features.md`](../features.md)。

---

## 9. 常见问题

**Q：子 metric 有 FAIL，但 overall 仍 PASSED？**

A：启用了 `pdms_aggregator` 时，整体结果只看 PDMS 分数。例如 TTC 子 summary 可能显示历史 risky 帧，但 pass 帧占比仍使 TTC 分项 ≥ 0.95，且 penalties=1，则整体可通过。

**Q：PDMS 为 0？**

A：优先检查 `pdms_subscores` 中 NC、DAC、SL 是否为 0。任一为 0 则 penalties=0。

**Q：EP 始终为 0？**

A：检查 sim 日志是否含 `sdc_route`；确认场景提供有效 SDC track，且 start/end 不重合。

**Q：DAC/SL 一直跳过？**

A：确认 `scene_map` 在 frame 0 写入，且含 `road_edges` / `road_lines`。

**Q：不想用 PDMS 作为整体判定？**

A：从 `metrics` 列表中移除 `pdms_aggregator`；此时 `overall_passed` 恢复为「所有子 metric 均 PASS」。

---

## 10. 相关文件

| 文件 | 说明 |
|------|------|
| `src/grading/metrics/safety/pdms_aggregator.{h,cc}` | PDMS 聚合实现 |
| `config/metrics_default.json` | 默认启用配置 |
| `src/grading/metric_manager.cc` | `overall_passed` / `pdms_score` 写入逻辑 |
| `hyw-proto/proto/grading/metric_output.proto` | `GradingReport`、`PdmsSubScore` |
| `hyw-proto/proto/grading/metrics/safety_metric.proto` | 各 checker 与 `PdmsAggregatorConfig` |
| `features.md` | `collision_risk_checker` 详细说明 |
| `README.md` | grading 构建、与 sim 联动总览 |
