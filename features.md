# Grading Metrics — 功能说明

本文档记录各 metric 的单帧计算逻辑与演进历程。当前仅收录 `collision_risk_checker`。

---

## `collision_risk_checker` — 碰撞风险（TTC）

**注册名**：`collision_risk_checker`  
**源码**：`src/grading/metrics/example/collision_risk_checker.cc`  
**配置 proto**：`CollisionRiskCheckerConfig`（`hyw-proto/proto/grading/metrics/example_metric.proto`）  
**默认参数**：`config/metrics_default.json`

### 目的

在每一帧评估 ego 与**相关前方 NPC** 的预测碰撞风险。若 bbox 已重叠，或估算的 **TTC（Time To Collision）** 低于临界阈值，则该帧 fail。整段仿真通过条件为 **`risky_frames == 0`**（任意一帧 dangerous 即整体 fail）。

与 `regulatory_collision_checker` 的分工：后者判定**已发生的几何碰撞**；本 metric 判定**尚未碰撞但逼近过快**的风险。

---

### 单帧输入（`MetricFrameInput`）

每帧从 SimLog 解析得到，grading 实际使用的字段如下。

| 字段 | 用途 |
|------|------|
| `frame_id` | 帧序号，用于日志 |
| `vehicle_state` | ego 位姿与速度：`x, y, heading, speed` |
| `ego_vehicle` | ego 尺寸：`length, width`（缺省回退 4.5 m / 1.85 m） |
| `npcs[]` | 全部 NPC：`id, x, y, heading, vx, vy, length, width` |
| `planned_trajectory` | **策略 C** 用：sim planner 输出的短期规划折线（≥2 点）。旧 sim log 无此字段时回退策略 A |

本 metric **不读取** `scene_map`、`road_context`、`planning_command`。

---

### 单帧输出（`MetricFrameOutput`）

| 字段 | 含义 |
|------|------|
| `bool_value` | **`true` = 该帧 pass**（无危险）；`false` = 该帧 fail（存在 dangerous pair） |

无 `custom_info`；危险时仅打 WARN 日志（`ttc / clearance / closing / overlap`）。

---

### 汇总输出（`MetricSummary`）

| 字段 | 含义 |
|------|------|
| `passed` | `risky_frames == 0` |
| `detail` | `risky_frames=N/M overlap_frames=… imminent_frames=… min_ttc_s=… min_clearance_m=…` |

- `risky_frames`：触发 `IsDangerous()` 的帧数  
- `imminent_frames`：worst pair 的 `ttc_s < warn_ttc_s`（默认 3.0 s）的帧数  
- `overlap_frames`：worst pair 已 bbox 重叠的帧数  

---

### 单帧算法步骤

```
对帧内每个 NPC:
  1. 几何风险（bbox）
  2. 空间过滤（C 或 A）
  3. 接近过滤（B）
  4. TTC 计算
取 worst pair（最小 TTC；并列时取更小 clearance）
  5. IsDangerous 判定 → bool_value
```

#### Step 0：边界情况

- `npcs` 为空 → **pass**（`bool_value = true`）
- 所有 NPC 均被过滤 → **pass**

#### Step 1：ego 局部坐标与 bbox 净空

将 NPC 变换到 ego 航向坐标系 `(dx, dy)`，用矩形 bbox 计算：

- `clearance_m`：两 box 最短间隙（未重叠时 > 0）
- `overlap`：两轴分离均 ≤ 0

#### Step 2：空间过滤（策略 C 或 A）

**overlap 的 NPC 跳过过滤**，始终参与后续评估。

```
overlap → 始终评估
↓
有 planned_trajectory（≥2 点）且未 disable → 策略 C
↓（否则）
策略 A：ego 航向前方走廊
```

**策略 C — 规划轨迹投影**（`PassesPlannedPathFilter`）

1. 将 ego、NPC 投影到 `planned_trajectory` 折线，得弧长 `arc_m` 与横向距离 `lateral_m`
2. 保留条件：
   - `lateral_m ≤ path_lateral_tol_m + path_npc_width_ratio × npc_width`
   - 沿程间距 `path_gap = npc_arc − ego_arc − ego_half_len − npc_half_len`
   - `path_gap > −0.5 m`（在 ego 前方）
   - `path_gap ≤ max_longitudinal_m`

**策略 A — 前方走廊**（`PassesForwardCorridorFilter`，无轨迹时回退）

在 ego 坐标系下：

- `dx > 0` 且 `dx ≤ max_longitudinal_m`
- `|dy| ≤ max_lateral_m + path_npc_width_ratio × npc_width`

不满足 → 该 NPC **跳过**（`EvaluateRiskForNpc` 返回 `nullopt`）。

#### Step 3：TTC 与策略 B

未 overlap 时：

1. 算相对速度在**中心连线方向**的 closing speed（仅靠近时为正）
2. **策略 B — 接近锥**（`PassesApproachHeadingFilter`）：
   - `closing_speed > min_closing_speed_mps`（默认 0.5 m/s）
   - `dx / center_dist ≥ min_forward_cos`（默认 cos 20° ≈ 0.94）
3. `ttc_s = clearance_m / closing_speed`（closing ≈ 0 时 TTC 为 inf）

overlap 时直接 `ttc_s = 0`。

#### Step 4：取 worst pair

遍历所有通过过滤的 NPC，取 **最小 `ttc_s`** 的一对；TTC 相同时取更小 `clearance_m`。

#### Step 5：`IsDangerous` 判定

满足任一即 **dangerous**（该帧 fail）：

| 条件 | 默认阈值 |
|------|----------|
| bbox 重叠 | — |
| `ttc_s < critical_ttc_s` | 1.5 s |
| `ttc_s < warn_ttc_s` 且 `clearance_m < 1.0 m` | 3.0 s |
| `clearance_m < near_clearance_m` 且 `closing_speed > 0.5 m/s` | 0.5 m |

`bool_value = !dangerous`。

---

### 当前默认配置

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `warn_ttc_s` | 3.0 | 预警 TTC |
| `critical_ttc_s` | 1.5 | 临界 TTC（主要 fail 条件） |
| `near_clearance_m` | 0.5 | 极近净空 |
| `path_lateral_tol_m` | 1.0 | 策略 C 横向容差 |
| `max_lateral_m` | 1.2 | 策略 A 横向容差 |
| `max_longitudinal_m` | 40.0 | 策略 A/C 纵向范围 |
| `min_forward_cos` | 0.94 (~20°) | 策略 B 前方锥角 |
| `min_closing_speed_mps` | 0.5 | 策略 B 最小接近速度 |
| `path_npc_width_ratio` | 0.25 | 横向容差中 NPC 车宽加成比例 |
| `disable_planned_path` | false | 关闭策略 C |
| `disable_forward_corridor` | false | 关闭策略 A |
| `disable_approach_heading` | false | 关闭策略 B |

`metrics_default.json` 示例：

```json
"pathLateralTolM": 1.0, "maxLongitudinalM": 40.0, "maxLateralM": 1.2,
"minClosingSpeedMps": 0.5, "minForwardCos": 0.94, "pathNpcWidthRatio": 0.25
```

---

## 演进历程（waymo_scenario_0 为主）

在 Waymo replay 场景（约 50+ NPC）上，通过逐步缩小「参与 TTC 评分的 NPC」范围，将误报从 49 帧降至 0 帧。

```mermaid
flowchart LR
  P0[阶段0 原始 TTC] -->|49/90| P1[阶段1 策略A+B]
  P1 -->|5/90| P2[阶段2 策略C]
  P2 -->|1/90| P3[阶段3 收紧容差]
  P3 -->|0/90| OK[scenario_0 通过]
```

### 阶段 0：原始实现

**逻辑**：对每帧**全部 NPC** 算 bbox TTC，取 worst pair；阈值硬编码；`params_json` 被忽略。

**waymo_scenario_0 结果**

| 指标 | 值 |
|------|-----|
| `risky_frames` | **49/90** FAIL |
| `imminent_frames` | 72 |
| `min_ttc_s` | 0.227 s |
| `overlap_frames` | 0 |

**问题**：侧向擦肩、后方、邻道 replay 车均参与评分；TTC 沿中心连线计算，路边车也能产生短 TTC；GIF 中「前方无车」仍可能 fail。

---

### 阶段 1：策略 A + B + 可配置 proto

**改动**

- **策略 A**：ego 航向前方矩形走廊（`dx>0`，横向/纵向范围）
- **策略 B**：要求实质接近速度 + 前方锥角（原默认 cos 30°）
- 新增 `CollisionRiskCheckerConfig`，`grading_main` 解析 `paramsJson`
- 无 relevant NPC 时该帧 pass

**当时默认**：`max_lateral_m=1.8`，`max_longitudinal_m=60`，`min_forward_cos=0.866`

**waymo_scenario_0**：`risky_frames` **49 → 5/90**

| 残留 fail 帧 | worst NPC | 原因 |
|--------------|-----------|------|
| 41–42 | NPC 43 | 邻道静止 replay，在航向走廊内，TTC < 1.5 s |
| 73–74, 88 | NPC 65 | 路边静止车，几何在 heading 前方 |

**结论**：航向走廊 ≠ 实际行驶路径，邻道/路边车仍会漏入。

---

### 阶段 2：策略 C（`planned_trajectory`）

**改动**

- `metric_input.proto` 增加 `PlannedTrajectory`；`grading_convert.cc` 从 sim 导出每帧规划轨迹
- **策略 C**：投影到 ~3 s 规划折线，有轨迹时**替代**策略 A
- 配置：`disable_planned_path`、`path_lateral_tol_m`（当时默认 1.5）

**需重跑 sim**（旧 log 无 `planned_trajectory`）。

**waymo_scenario_0**：`risky_frames` **5 → 1/90**

- NPC 43：`path_lat ≈ 8.5 m` → 滤除 ✓
- NPC 65（frame 73）：`path_lat ≈ 2.77 m`，阈值 `1.5 + 0.5×2.5 ≈ 2.75 m` → **卡在边界内**，仍 fail

---

### 阶段 3：收紧容差（放弃同车道方案）

**决策**：不做 `lane_query` / `scene_map` 同车道判定，仅微调 A/C/B 参数与车宽加成。

**改动**

| 参数 | 阶段 2 | 阶段 3（当前） |
|------|--------|----------------|
| `path_lateral_tol_m` | 1.5 | **1.0** |
| `max_lateral_m` | 1.8 | **1.2** |
| `min_forward_cos` | 0.866 (30°) | **0.94 (~20°)** |
| `max_longitudinal_m` | 60 | **40** |
| NPC 宽度加成 | 固定 `0.5×width` | **`path_npc_width_ratio=0.25`** |

NPC 65 新阈值：`1.0 + 0.25×2.5 = 1.625 m` < 2.77 m → 滤除。

**验证**（现有 sim log，无需重跑 sim）

| 场景 | `collision_risk_checker` |
|------|--------------------------|
| waymo_scenario_0 | **PASS** `risky_frames=0/90` |
| waymo_scenario_1 | PASS `risky_frames=0/90` |
| waymo_scenario_2 | FAIL `risky_frames=5/90`（frame 1–5，TTC≈1.34 s，更像真实前方接近） |

---

### 各阶段效果一览

| 阶段 | 主要变化 | scenario_0 risky_frames |
|------|----------|-------------------------|
| 0 | 全量 NPC TTC | 49/90 |
| 1 | A + B 过滤 | 5/90 |
| 2 | C 规划轨迹投影 | 1/90 |
| 3 | 收紧容差 + width ratio | **0/90** |

---

## 后续升级方向（未实现）

1. **可观测性**：帧级输出 worst NPC `id`、过滤原因、`path_lat` / `path_gap`，便于对照 GIF。
2. **Reference route 投影**：投影到整条参考路径（GIF 蓝线）而非 3 s `planned_trajectory`；需 sim 导出 `reference_points`。
3. **静止车规则**：如 `speed < 0.3 m/s` 且横向偏离 > 1 m 则忽略，针对 Waymo 路边 replay。
4. **同车道 + 前方**（已设计、已放弃）：`scene_map` + `closest_lane_id`；变道场景可能漏报。
5. **TTC 算法**：改为沿 ego 前进轴的 closing speed，或补充 THW，减轻「远距但 TTC 短」误报。
6. **参数标定**：在 scenario 回归集上网格搜索；若漏报相邻车道切入，可将 `pathLateralTolM` 回调至 1.2。

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `src/grading/metrics/example/collision_risk_checker.cc` | 主逻辑 |
| `hyw-proto/proto/grading/metric_input.proto` | `PlannedTrajectory` 等输入 |
| `hyw-sim/cpp/grading_convert.cc` | sim → grading 轨迹导出 |
| `config/metrics_default.json` | 默认 `paramsJson` |
