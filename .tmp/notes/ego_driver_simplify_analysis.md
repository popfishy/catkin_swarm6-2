# ego_planner_driver 结构复杂度分析与精简方案

日期：2026-08-25
范围：`src/ego_planner_driver`（约 4646 行 C++）+ `src/swarm_uav_executor/drivers/ego_swarm.py`（648 行）
对照：原版 `references/ego-planner-swarm`（单层优化）

> ⚠️ **历史快照（2026-08-25）**：本文的 L3 精简分析对应当时行数（4646 行）。此后
> 26082602/26082621 又恢复候选、新增周期碰撞检查与单一 replan 入口，行数与结构已变。
> 仅作历史参考；当前机制以 `src/ego_planner_driver/README.md` 为准。

---

## 1. 复杂度量化

| 模块 | 行数 | 职责 |
|---|---|---|
| `ego_planner_driver_node.cpp` | **1660** | 状态机 + replan + 监督 + yield 提交 + intent 接收/发布 + 周期刷新 + setpoint + 测试接口（40+ 函数混杂） |
| `ego_planner_core.cpp` | 824 | 规划核心：B-spline 优化 + calcSwarmCost + checkClearance + computeSafeClearance + rebound |
| `uniform_bspline.cpp` | 377 | B-spline 数学 |
| `safety_predictor.cpp` | 252 | 预测 + 动态距离 + **YIELD/STAND_ON 角色判定** |
| `yield_candidates.cpp` | 221 | 让路候选 SLOW/LEFT/RIGHT/FAST |
| `safety_logger.cpp` | 129 | 异步日志 |
| `emergency_brake.cpp` | 94 | 制动轨迹 |
| `ego_swarm.py` | 648 | executor：任务状态机 + 运行时距离门禁 + setpoint |

状态机：HOLD/TAKEOFF/EXECUTING/COMPLETED/POSE_STALE/EGO_PLAN_FAILED/
EGO_EXEC_TIMEOUT/EMERGENCY_BRAKE/BRAKE_HOLD/NEGOTIATING —— **10 个状态**。

intent 协议：协议 1.0/2.0、phase(TENTATIVE/ACTIVE/BRAKING)、negotiation 屏障、
traj_id 版本去重、epoch 校验、stale 窗 30s、周期 5Hz 刷新。

---

## 2. 职责重叠矩阵（同一概念多处实现）

| 概念 | 出现处 | 说明 |
|---|---|---|
| 动态安全距离公式 | `ego_planner_core.cpp:170` / `safety_predictor.cpp:143` / `yield_candidates.cpp:162` | **同一公式（closing×reaction+制动余量）实现 3 次**，参数漂移风险 |
| "距离是否安全" | calcSwarmCost(软) / checkClearance(硬1m) / SafetyPredictor(动态) / _distance_safe(1m/2m) | 4 处独立判定 |
| 避碰动作 | 规划软绕开 / yield SLOW·LEFT·RIGHT·FAST / 紧急制动 / executor HOLD | **4 套动作相互覆盖** |
| 轨迹共享 | 原版广播网络 vs 当前 intent 广播 + 周期刷新 | 当前仅 1 套（intent），但协议复杂度高 |

---

## 3. SITL 实证的负资产 / 低效点

1. **yield 候选（阶段 D）**：正交 8m 臂交叉场景 SLOW→FAST→紧急制动全部失败
   （SLOW 预测 3.63m < 门槛 4m、FAST 3.55m、LEFT 2.2m —— 单机机动几何不足）。
   yield 与规划软绕开**功能重叠**（都试图"让路"），且监督优先于规划导致规划绕开被覆盖。
2. **动态距离作硬门**：SITL 实证导致交叉走廊 EGO_PLAN_FAILED（已回退固定 1m，本次修复）。
3. **UAV1 误判**：监督预测基于 intent 名义轨迹，与实际执行（时序偏差）不符 → 误制动。
4. **negotiation（阶段 C）**：TENTATIVE 屏障 + 1s 等待只服务于"多机同时决策"，
   复杂度高（negotiation 状态 + tentative_ 字段 + generation 校验）收益低。

---

## 4. 精简方案

### L1 删除负资产（低风险，立竿见影，约 -900 行）
- 删 `yield_candidates.cpp` + node 的 yield 提交（`yield_candidate_generator_`/
  `yield_peer_versions_`/`yieldCommittedForTest`）→ 监督层只保留 EMERGENCY 制动兜底
- 删 `negotiation`（TENTATIVE phase、`intent_negotiation_wait_s`、`tentative_`、
  `tentative_generation_`）→ intent 只有 ACTIVE（周期发布）
- 合并 3 处动态距离公式 → 单一实现（SafetyPredictor 或 core）

### L2 合并重复（约 -300 行）
- 安全距离唯一实现：`computeSafeClearance` 作为唯一动态距离源，SafetyPredictor 复用
- 避碰动作收敛为 3 套（规划软惩罚 / 制动兜底 / executor 运行时门禁），砍 yield
- `checkClearance`（1m 提交前校验）与 executor `_distance_safe` 语义统一注释

### L3 架构收敛（推荐，回归原版精华 + 最小兜底）
```
[单层规划]  calcSwarmCost（动态软惩罚，借鉴原版时间同步采样 peer）
  + 固定 1m 硬门 checkClearance（对应原版 checkTrajectory 提交校验）
  + 内置硬推（原版 check_collision_and_rebound；当前 enable_rebound 决定启/删）
[最小兜底]  SafetyPredictor 只算 EMERGENCY（触发制动）——砍 YIELD/STAND_ON 角色
  + executor 运行时门禁 HOLD
[intent]    周期广播当前轨迹（保留本次修复，5Hz 固定 epoch）
```
砍：yield_candidates、SafetyPredictor 角色判定、negotiation、协议 1.0 兼容。
预期：4646 → ~3200 行（-30%）；避碰 3 层 → 2 层；动态距离公式 3 → 1；状态 10 → 7。

---

## 5. 保留清单（不可砍）
- **周期 intent 发布**（本次修复）：age 1.2s→0.1-0.3s，动态距离 3-4m→~2m
- **动态距离软惩罚**（calcSwarmCost）：激励绕开（保留软，不硬拒）
- **固定 1m 硬门**：物理安全门（SITL 实证动态门作硬拒会 PLAN_FAILED）
- **紧急制动兜底**（emergency_brake + BRAKE_HOLD）：缺陷8 修复，replan 不可覆盖
- **executor 运行时距离门禁**：最后防线

## 6. 决策点
- yield 候选（阶段 D 投入最大）是否砍：正交交叉场景 SITL 证明几何不足，且与规划绕开重叠
- SafetyPredictor 角色（YIELD/STAND_ON）：砍后监督只做 EMERGENCY 判定（简化）
- 原版 rebound 是否引入：当前 enable_rebound=false（禁），可评估启用或删代码

---

## 7. 执行结果（2026-08-26，L3 全量精简已完成）

### 已删除
- **yield_candidates.cpp/h + test_yield_candidates.cpp**（阶段 D 让路候选 SLOW/LEFT/RIGHT/FAST）
- **negotiation**：intent_negotiation_wait_s、tentative_*、NEGOTIATING 状态、activateTentativeLocked
- **SafetyPredictor 角色**：PairRole/determineRole/roleName/ConflictEpisode::role
- node 的 yield 提交逻辑（yield_candidate_generator_/yield_committed_/yield_peer_versions_/
  episode_maneuver_/peerVersions*）、remainingTrajectory、launch 的 yield_* 参数

### 保留（最小兜底栈）
- **监督只做 EMERGENCY 制动**：runSafetySupervisor 中 level != EMERGENCY 即 return，
  WARNING 级冲突完全交给规划层（calcSwarmCost 动态软惩罚绕开）
- 紧急制动 + BRAKE_HOLD（缺陷8 修复）、executor 运行时距离门禁、周期 intent 发布
- 规划层：动态距离软惩罚（computeSafeClearance）+ 固定 1m 硬门

### 公式合并
- 新增 SafetyPredictor::dynamicMargin(closing, reaction, decel) 为唯一制动余量公式；
  evaluatePair（监督门槛）与 computeSafeClearance（规划软惩罚）共用
- 统一 age 处理：EgoPlannerConfig.safety_peer_data_age_cap_s（=2.0，与 SafetyPredictorConfig 一致），
  修复此前"监督 cap、规划实际 age"的参数漂移

### 量化
- 源码（src+include）：4646 -> 4083 行（-563，-12%）；删除文件另减 ~500 行
- node.cpp：1660 -> 1426 行；状态机 10 -> 9（去 NEGOTIATING）
- 测试：70 -> 57 个（删除 yield/negotiation/角色 13 个），全部通过
- 动态距离公式：3 处 -> 1 处（dynamicMargin）

### 行为变化说明
- active 模式下 WARNING 不再尝试让路候选或按角色决策，只记录日志；预测会撞（EMERGENCY）才制动。
- 交叉场景的绕开完全依赖规划层动态软惩罚（单测 PredictiveAvoidanceDeviatesAroundCrossingPeer）。
