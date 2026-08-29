# 方案 B 无头测试发现与修改需求（2026-08-23）

> ⚠️ **历史快照（2026-08-23）**：方案 B（2m 密化 + planning_horizon 窗口滚动）已被
> 26082621 plan-exec 循环（3m 密化 + 相邻关键点推进）取代。本文仅作历史参考；当前机制以
> `src/ego_planner_driver/README.md` 为准。

## 无头测试结果（prepare，A01-A12 MOVE_TO，timeout 150s）

**status=TIMEOUT**，failed_robot_ids=[]。

### 已验证（方案 B 链路基本工作）
- 上层分层关键点 + 2m 密化 + waypoints 发布 ✅（MOVE_TO 不再发单点 goal）
- C++ planning_horizon 截取 + 滚动重规划 ✅（单测覆盖，无头运行正常）
- 目标可达：UAV1 到达 EKF ENU (50.8,-18,11.9)，接近目标 (50,-17.5,12)（偏差 ~0.96m）

### 三个未达标问题
| # | 问题 | 实测 | 预期 | 根因 |
|---|---|---|---|---|
| 1 | 速度超标 | 峰值 2.2-3.0 m/s | ≤1.5 m/s | 2m 密化 + 5m horizon → horizon 内仅 ~3 个航点 → B-spline 控制点不足，固定 ts=0.4 压缩轨迹；max_vel 软惩罚压不住 |
| 2 | 分层不严格 | z 爬升同时 x/y 移动（非先 15m 再水平） | 先垂直到 15m 层再水平 | B-spline + jerk 平滑使 start→15m 垂直段倾斜；截取段含垂直+水平点，未隔离垂直段 |
| 3 | 到达判定失败 | 终点偏差 ~1.0 m（>0.2 m 容差） | ≤0.2 m | PX4 OFFBOARD 位置环稳态误差；150s 内 12 机未全部 COMPLETED → TIMEOUT |

## 修改需求（决策点，等待用户判断）

### 1. 速度控制（需确认密化/ horizon 参数）
- 选项 A：密化间距 2m → **0.5m**（全段）→ horizon 5m 内 ~10 控制点 → 速度受 max_vel 约束。
  **偏离用户原"2m 密化"指示**，但 0.5m 是用户早前讨论垂直段时提到的值。
- 选项 B：保持 2m 密化 + horizon 5m → **15m**（或更大）→ 2m 间距下控制点 8-10 个。
  保持"2m 密化"，但 horizon 增大削弱局部性。
- 选项 C：max_vel 惩罚权重 `lambda_feasibility` 1.0 → 更大 + 选项 A/B 之一。

### 2. 分层严格性（可能需结构改动）
- 选项 A（结构）：垂直段独立于 B-spline——executor 在垂直过渡期发布纯垂直 setpoint
  （冻结 x/y/yaw），到层后再发水平 waypoints。严格但改动大。
- 选项 B（参数）：垂直段 0.5m 密化（更多垂直控制点）→ B-spline 更贴合垂直，仍可能轻微倾斜。
- 选项 C：接受近似分层（B-spline 大方向"先升后平再降"，不严格 x/y 冻结）。

### 3. 到达判定
- 选项 A：position_tolerance_m 0.2 → 1.0（匹配 PX4 稳态偏差）。
- 选项 B：保持 0.2m，接受 prepare 可能 TIMEOUT（编队到位但未判到达）。

## 建议
- 若接受参数级调整：密化 0.5m + horizon 5m（或 2m+15m）+ 垂直段密化，重跑无头验证速度/分层。
- 若需严格分层（垂直段冻结 x/y/yaw）：需 executor 垂直过渡结构改动（选项 2A），改动较大。
