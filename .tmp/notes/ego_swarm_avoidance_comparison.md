# 两版运行状态机与机间避碰机制对比：原版 ego-planner-swarm vs 当前 executor+ego_planner_driver

日期：2026-08-25（避碰对比）/ 2026-08-26（扩展运行状态机）
范围：`references/ego-planner-swarm`（原版，FSM + 时间同步 swarm 势场 + rebound） vs
      `src/ego_planner_driver` + `src/swarm_uav_executor`（当前，intent 广播 + 任务/状态机 + 监督制动）

> 演进说明：本文记录两版机制的逐轮对比。文中出现的**已废除机制**——周期 5Hz intent 刷新、
> 规划层动态门槛（`data_age` 放大）、tentative 协商屏障、FAST/四候选、监督回调直接制动——
> 均指 26082602/26082621/L3 精简之前的历史设计，不再表示当前行为。当前行为以
> `src/ego_planner_driver/README.md` 为准（状态机 + replan 策略单一权威）。

## 0. 范围说明（不对比部分）

本文对比**完整运行状态机 + 机间避碰**。以下属于**环境感知/单机避障**的内容**不在范围**内
（当前移植版明确不需要，场景无静态障碍、SITL 提供仿真定位）：
- **环境感知**：VIO/视觉定位、深度图像、智能体检测与相对定位漂移补偿（论文 IV-B/IV-C）
- **单机静态避障**：EGO-Planner 的障碍距离 `{p,v}` 对、隐式拓扑轨迹生成（论文 III）、
  `obj_predictor` 移动障碍多项式预测、静态地图/占用栅格
- 上述仅在与运行状态机耦合处（如"收到新障碍触发 replan"）简略提及，不展开机制

---

## 1. 原版机制（references/ego-planner-swarm）

核心思想：**对等互惠**。每架 UAV 本地做同一套 B-spline 优化，把"其他机的已知轨迹"
当作时间同步的移动障碍推入代价，两机同时绕开（对称让路），无监督、无角色、无制动。

### 1.1 ASCII 流程

**机间避碰（规划管线）** —— 每 replan 周期（1s 或碰撞预测）：

```
[PlannerManager::plan()]
  ① 轨迹生成      min-snap → B-spline 参数化（控制点 Q）
  ② 注入 peer     deliverTrajToOptimizer → setSwarmTrajs(&swarm_trajs_buf_)
  ③ 优化          BsplineOptimizer::optimize()（L-BFGS，代价叠加）
                       ├ J_s  平滑         jerk²
                       ├ J_d  可行性       vel/acc 超限惩罚
                       ├ J_w  ★机间避碰    calcSwarmCost（式5-6，唯一机间机制）
                       └ J_c  移动障碍     calcMovingObjCost（订阅 /dynamic/pose_N 外部
                                           动态物体；调用点 1694 被注释，默认未启用）
  ④ 环境硬推       check_collision_and_rebound（grid_map 占用栅格 + A* 路径推出）
                   ★环境避障，非机间（不在对比范围）；机间只有 ③ 软惩罚 + ⑤ 校验
  ⑤ 提交校验       checkTrajectory（绝对时间 t 双机采样，<clearance 拒绝）★机间
  ⑥ 广播/执行      publishSwarmTrajs → traj_server → setpoint
```

```
calcSwarmCost（机间避碰代价，式5-6）对每个控制点 i：
    glb_time  = t_now + 控制点时刻            ← 绝对时钟（时间同步）
    peer_pos  = peer_traj.evaluateDeBoorT(glb_time - peer_start_time)
    ellip_dist = √( dz²/a² + (dx²+dy²)/b² )   a=2.0(垂直宽松), b=1.0
    CLEARANCE = swarm_clearance × 2            ← 固定（论文 C）
    dist_err  = CLEARANCE - ellip_dist
    若 dist_err ≥ 0：cost += dist_err²，梯度同向推离
```

**轨迹广播网络**（论文 V-B.1）：

```
  生成即广播 + 周期共享
    本机 ──/drone_N_planning/swarm_trajs(MultiBsplines)──► 邻机
    本机 ──/planning/broadcast_bspline(单条)────────────► 邻机
  邻机接收：BroadcastBsplineCallback → swarm_trajs_buf_
    · 时间同步检查 |now - start_time| ≤ 0.25s，否则丢弃
    · 收到即做碰撞检查，必要时触发 REPLAN_TRAJ
```

### 1.2 关键代码锚点
- `bspline_optimizer.cpp:776 calcSwarmCost` —— ★机间避碰唯一代价（软惩罚）。固定
  `CLEARANCE = swarm_clearance_*2`，椭圆 `a=2.0`(垂直) `b=1.0`(水平)（对应论文
  `E=diag(1,1,1/c)`，`c=a²=4`），`glb_time = ros::Time::now()+控制点时刻`（时间同步）
- `bspline_optimizer.cpp:1198 check_collision_and_rebound` —— ★环境障碍硬推（非机间）：
  `grid_map_->getInflateOccupancy()`（占用栅格膨胀检测）+ `a_star_->AstarSearch`
  （A* 安全路径）把碰撞段控制点推出。机间避碰**不使用**它
- `bspline_optimizer.cpp:1433 rebound_optimize` —— 重启 L-BFGS；内部对**环境**做
  `getInflateOccupancy`（1499-1502），对**机间**仅"swarm too close → keep optimizing"
  （1483-1490，继续软优化直到 `min_ellip_dist_ > swarm_clearance_`，**非硬推**）
- `plan_container.hpp:219 OneTrajDataOfSwarm` —— drone_id/start_time_/duration_/position_traj_
- `planner_manager.h:41 deliverTrajToOptimizer` —— `setSwarmTrajs(&swarm_trajs_buf_)`
- `planner_manager.cpp:361-368 checkTrajectory` —— 绝对时间 t 双机采样校验
- `bspline_optimizer.h:104 setSwarmTrajs(SwarmTrajData*)` —— peer 轨迹注入点
- `ego_replan_fsm.cpp:51/57/245/323` —— `swarm_trajs_sub/broadcast_bspline_sub` +
  `BroadcastBsplineCallback`（0.25s 时间同步检查）/`swarmTrajsCallback`（广播接收填充缓冲）

### 1.6 原版"多候选轨迹"机制（distinctiveTrajs，环境避障专用）

> 澄清：原版的"多个避障候选轨迹"只针对**环境静态障碍**（grid_map），**机间避碰没有候选**。
> 这是设计机间候选时最容易误照搬的地方。

**触发**：每次 `plan()`（`planFromCurrentTraj`/`planFromGlobalTraj` 共用）当
`manager/use_distinctive_trajs=true`（默认 **false**）时启用（`planner_manager.cpp:25/231`）。

**候选生成**（`bspline_optimizer.cpp:45 distinctiveTrajs`）：
```
① initControlPoints（L434）：按 grid_map 占用栅格扫描初始控制点路径，
   占用状态变化的边界 → segments（轨迹穿过障碍物区域的段）
② distinctiveTrajs(segments)：对每个障碍物段沿相反方向（VARIS=2）生成
   拓扑不同的控制点，最多 MAX_TRAJS=8 条（绕障左/右/上/下）
③ 逐条 BsplineOptimizeTrajRebound 优化（含机间 calcSwarmCost 代价）
④ 选择 final_cost 最小的一条；全失败 → 本轮 plan 失败
```

**选择**（`planner_manager.cpp:234-267`）：`min_cost` 比较（优化后总代价：平滑+可行+
swarm 软惩罚等）。**无优先级、无责任分配**——候选只针对静态障碍几何，机间 peer 轨迹
不参与 segments 划分，只在优化代价里作为软惩罚。

**对机间候选的迁移结论**：
- 原版"多候选"的**触发源（障碍段）和几何（绕障拓扑）不能照搬**——机间"障碍"是移动
  邻机轨迹，没有 grid_map 占用段
- 可借鉴的是**框架**：多候选生成 + 逐条验证 + 确定性选择（cost 最优）
- 机间候选必须自己定义：触发（机间冲突）、候选（SLOW/LEFT/RIGHT）、选择
  （right-of-way + cost）

### 1.3 原版行为特征
- **机间单层**：机间避碰**只有** calcSwarmCost 软惩罚 + checkTrajectory 提交校验
  （拒绝 → FSM 重试 REPLAN_TRAJ）；`check_collision_and_rebound` 硬推属**环境避障**
  （grid_map + A*），机间不使用
- **对称**：两机同时绕开，无 YIELD/STAND_ON 角色
- **固定 clearance**：不随接近速度 / 数据年龄 / 反应时间变化（论文 `C` 常数）
- **垂直更宽松**：`a=2.0` → 垂直半轴 = 2×水平（论文 `E=diag(1,1,1/c)` 数学一致；
  论文文本"shorter principal axes at z-axis"与公式矛盾，以代码为准）
- **广播网络**：生成即广播 + MultiBsplines 周期共享 + 收到轨迹立即碰撞检查（on-demand replan）
- **时间同步**：0.25s 窗口检查广播轨迹时间戳（配合链式网络时间同步）
- **无制动/让路候选/监督**：peer 过近时依赖优化硬推 + 校验拒绝（replan 失败则停）

### 1.4 论文公式与参数（对照代码）
- 式(5) swarm 软障碍惩罚：`J_w,k = Σ_i ∫_{t_s}^{t_e} d_{k,i}(t)² dt`，仅当 `d_{k,i} < 0`；
  `d_{k,i}(t) = ‖E^{1/2}[Φ_k(t) - Φ_i(t)]‖ - (C + ε)` —— 同一时刻 t 采样本机 Φ_k 与邻机 Φ_i
- 式(6)：`min_J = J_EGO + λ_w J_w`（J_EGO 含 smoothness/collision/feasibility/terminal，式1）
- `E := diag(1,1,1/c), c>1` → 代码 `a=2.0,b=1.0`（`c=a²=4`），垂直半轴=C√c=2C（垂直更宽松）
- 论文参数：planning horizon 7.5m；`λ_s=1.0, λ_c=λ_w=λ_t=0.5, λ_d=0.1`；replan 每 1s 或碰撞预测
- 论文 III-B 隐式拓扑规划（`{p,v}` 对反转生成不同局部最小值、并行优化取最低成本）属于
  **单机避障**（配合 EGO-Planner 的障碍距离），非机间避碰部分；机间仅用式(5)-(6) swarm 惩罚

### 1.5 原版完整运行状态机（ego_replan_fsm.cpp）

```
[FSM 状态转换图]  exec_state_，7 态

  INIT ──有odom──────────────────────────────► WAIT_TARGET
  WAIT_TARGET ──有target + trigger───────────► SEQUENTIAL_START
  SEQUENTIAL_START ──id=0 或已收前序轨迹────► [首次规划 planFromGlobalTraj(10)]
        │ 成功                                    │ 失败
        ▼                                         ▼
     EXEC_TRAJ ◄────────────────────── (回到 SEQUENTIAL_START 重试)

  [EXEC_TRAJ 内每 tick] 按条件分支：
     · 近航点（<no_replan_thresh）   → 推进 wp_id_，规划下个 waypoint
     · 超时/偏离（t_cur>replan_thresh）→ REPLAN_TRAJ
     · 到达终点（t_cur>duration-1e-2）→ WAIT_TARGET（悬停，等下一个 trigger）

  REPLAN_TRAJ ──planFromCurrentTraj(1) 滚动──► EXEC_TRAJ（+publishSwarmTrajs）
        │ 失败
        ▼
     （重试 REPLAN_TRAJ）

  GEN_NEW_TRAJ ──planFromGlobalTraj(10)（新目标/脱离紧急）──► EXEC_TRAJ
  EMERGENCY_STOP ──fail_safe: odom_vel<0.1──► GEN_NEW_TRAJ
```

```
[启动]  chain network（论文 V-B.2）：drone_i 等待 drone_{i-1} 初始轨迹
[任务]  PRESET_TARGET 预置航点 / MANUAL_TARGET(/move_base_simple/goal) + /traj_start_trigger
[执行]  traj_server：订阅 /planning/bspline → 采样 → /position_cmd（50Hz）
[广播]  每次生成新轨迹 publishSwarmTrajs + 周期 broadcast_bspline
[peer]  收到广播 → swarm_trajs_buf_ + 碰撞检查 → 必要时 REPLAN_TRAJ
```

- 状态：`INIT / WAIT_TARGET / SEQUENTIAL_START / GEN_NEW_TRAJ / REPLAN_TRAJ /
  EXEC_TRAJ / EMERGENCY_STOP`（7 个）
- 规划触发：定时 `replan_thresh` / 接近航点推进 / 收到新 peer 轨迹 / 预测碰撞 / 新目标
- 执行：`traj_server` 独立节点把最新 B-spline 轨迹按时序采样发布 PositionCommand
- 落地/完成：到达终点 `t_cur > duration` → `WAIT_TARGET`（悬停等下一个 trigger）
- 代码锚点：`ego_replan_fsm.cpp:447-608`（FSM switch）、`traj_server.cpp`（执行发布）、
  `ego_replan_fsm.cpp:479-505`（SEQUENTIAL_START 顺序启动）

---

## 2. 当前机制（src/ego_planner_driver + src/swarm_uav_executor）

核心思想：**意图广播 + 分层安全（26082602 对齐原版 + 26082621 单一入口）**。每机在 replan/
候选/制动提交时广播自身轨迹意图（intent，生成即发）；规划层固定 clearance + 周期碰撞检查；
plan-exec 三分支（推进/到达/定时）+ 单一 replan 入口（replanLoop 唯一提交者）；监督层保留
预测式 WARNING/EMERGENCY 分级（方案 Y：EMERGENCY 只置 emergency_pending_，制动由 replan
线程提交）；executor 提供任务状态机与运行时距离门禁。

### 2.1 ASCII 流程

**整体分层** —— executor（任务/执行）在上，ego（规划/监督）在下：

```
  [executor]  /uav_task(任务) → /waypoints(3m密化) → ego → /setpoint/ego → PX4
              运行时监控 _wait_for_terminal(25Hz)：
                · 邻机 odom < 1m(水平)&2m(垂直) → MIN_DISTANCE_BREACH → HOLD
                · ego exec_state ∈ TERMINAL_BAD  → 运动失败 → HOLD
                · ego exec_state ∈ HOLD/BRAKE_HOLD → COMMAND_HELD
```

```
  [ego：intent 收发 + 单一 replan 入口（26082621）]

  路径① intent 收发
    收：/neighbor_intent → inbox
        · 协议/身份/frame/epoch 校验 + (stamp,traj_id) 版本去重
        · GPS bias 修正激活时刻到本机时间轴 + stale 窗 [now-30, now+30]
        · 前驱 intent 到达 → 事件唤醒首轮规划（SEQUENTIAL_START）
    发：trajectory_intent ← 生成即发（26082602 删除周期 5Hz 刷新）
        · stamp=激活 epoch + traj_id 仅 replan/候选/制动提交时递增

  路径② plan-exec 执行循环 + 单一 replan 入口（方案 A）
    [publishSetpoint 每 tick 三分支]  waypoints = 上层 3m 密化关键点
      分支 A 推进：距当前目标 <0.5m → 切相邻下一个 keypoint（<0.5m 丢弃、3.5m 上限）
      分支 B 到达：距最终关键点 <0.2m（稳定 1s）→ COMPLETED → HOLD
      分支 C 定时：elapsed ≥ replan_period_s(1.0s) → planning_requested_
    [replan 触发 —— 全部只置位 + 事件唤醒 replanLoop]
      ① 1s 定时（分支 C）                    → planning_requested_ = true
      ② collisionCheckTick(10Hz) 冲突
         （current_ vs fresh 邻机同刻 <1.3×1m）→ planning_requested_ = true
      ③ checkArrival 推进（分支 A 切关键点）  → planning_requested_ = true
      ④ runSafetySupervisor EMERGENCY        → emergency_pending_ = true（+peer/ttc）
    [replanLoop（条件变量事件唤醒，唯一提交者）→ replanOnce]
      ① EMERGENCY 且常规 replan 失败且 ttc<1.0s → 制动（EMERGENCY_BRAKE→BRAKE_HOLD）
      ② 常规 replan（绕开）成功 → 提交，清 emergency_pending_ / plan_fail_since_
      ③ 常规 replan 失败 → 候选（SLOW/LEFT/RIGHT + right-of-way）
           · 低 ID STAND_ON 不生成（交常规 replan）；高 ID YIELD 生成
          · SLOW scale{0.5,0.8,1.2}（含加速）/ LEFT-RIGHT lateral{1.3,1.8,2.2}×scale{0.7,0.5}
          · min_clear ≥1.2×1m，cost=ΔT/T（绝对值）+dev/5.0（26082700 dev/2.0→dev/5.0）
           · 候选成功提交 → 恢复；候选全失败 → 进入【重试窗口】（状态保持，见 §2.3）
    calcSwarmCost  固定 swarm_clearance×2=1m 软惩罚（26082602 解耦动态门槛）
    checkClearance 固定 1m 硬门（提交前）

  路径③ 监督 runSafetySupervisor（25Hz，方案 Y）
    SafetyPredictor 预测未来 5s 窗（min_d/CPA/TTC，reaction 不含 data_age）
    level==EMERGENCY（min_d ≤ 物理 1m）→ 只置 emergency_pending_（记录 primary peer+ttc）
        + 唤醒 replanLoop；制动由 replan 线程在"常规 replan 失败且 ttc<1.0s"时提交
    level==WARNING（预测将越 1.3×1m 门槛）→ 只写日志（交规划层软惩罚绕开）
```

### 2.2 关键代码锚点
- `ego_planner_driver_node.cpp` —— replanLoop（条件变量事件唤醒，唯一提交者）/ replanOnce
  （EMERGENCY 制动→常规 replan→候选）/ collisionCheckTick（10Hz 只置位）/
  runSafetySupervisor（25Hz 只置 emergency_pending_）/ checkArrival（三分支）/
  advanceTargetLocked / tryEmergencyBrakeLocked / publishIntent（生成即发）
- `ego_planner_core.cpp:340 calcSwarmCost` —— 固定 `swarm_clearance*2` 软惩罚
- `ego_planner_core.cpp:798 checkClearance` —— 固定 1m 硬校验
- `safety_predictor.cpp` —— WARNING/EMERGENCY 分级（EMERGENCY 判定 = min_d ≤ 物理 1m）
- `emergency_brake.cpp` —— 制动轨迹；`ego_swarm.py:_wait_for_terminal` —— 运行时门禁
- `yield_candidates.cpp` —— SLOW{0.5,0.8,1.2}/LEFT-RIGHT{1.3,1.8,2.2}×{0.7,0.5}（replan 失败分支，
  门槛 1.2、cost dev/5.0；26082700 更新）

### 2.3 当前完整运行状态机

```
[executor 任务层]
  /uav_task(PREPARE) → /uav_task_control(START) → 执行 → COMPLETED/FAILED
  /uav_hold(任意时刻) → HOLD（中断任务）
  _plan_horizontal：分层关键点 → 3m 密化 → /waypoints（目标=相邻下一个 keypoint）
```

```
[ego 飞行状态机转换图]  8 态 + replan 重试窗口（26082621 删 POSE_STALE；26082700 失败状态保持）

  基础切换：
  HOLD ──arm+OFFBOARD───────────► TAKEOFF
  TAKEOFF ──爬升到 takeoff_height► HOLD
  HOLD ──收到 goal/waypoints────► EXECUTING
  EXECUTING ──推进（分支A 距目标<0.5m）► EXECUTING（切相邻下一 keypoint + 置位唤醒）
  EXECUTING ──到达（分支B 距最终<0.2m 稳定）► COMPLETED ──► HOLD
  EXECUTING ──执行超时──────────► EGO_EXEC_TIMEOUT ──(executor 收口)→ HOLD

  replan 失败路径①（非 EMERGENCY）——【重试窗口】状态保持（26082700）：
  EXECUTING/HOLD ──常规 replan 全失败（含候选无解）──► EXECUTING/HOLD（状态不变）
      重试窗口内（plan_fail_since_ 计时 + 1s 定时/冲突/推进事件触发 replanOnce）：
        ├─ replan 成功提交 ─────────────► EXECUTING（退出窗口，清计时）
        ├─ 连续失败 ≥50s ───────────────► EGO_PLAN_FAILED ──(executor 收口)→ HOLD
        └─ EMERGENCY 事件 ──────────────►（转入下方 EMERGENCY 路径）

  replan 失败路径②（EMERGENCY，监督预测碰撞，方案 Y 只置位）：
  EXECUTING ──EMERGENCY + replan 失败 + ttc<1.0s──► EMERGENCY_BRAKE ──制动耗尽──► BRAKE_HOLD（锁存，不重试）
  EXECUTING ──EMERGENCY + replan 失败 + ttc≥1.0s──► EGO_PLAN_FAILED（立即）──► HOLD

  replan 失败路径③（提交段安全校验失败，立即收口）：
  EXECUTING ──提交时最新邻机轨迹使本段不安全──► EGO_PLAN_FAILED ──► HOLD

  （HOLD 锁存进入时位置/yaw，不随 pose 漂移——删 POSE_STALE 后无 pose 过期态）
```

```
[并行支撑]
  intent     生成即发（replan/候选/制动提交，stamp+traj_id）
  replan     单一入口：1s 定时 / 10Hz 冲突 / 推进 / EMERGENCY 全部只置位唤醒 replanLoop
  监督       25Hz：EMERGENCY 只置 emergency_pending_（方案 Y，制动由 replan 线程提交）
  executor   运行时距离门禁 _distance_safe → MIN_DISTANCE_BREACH → HOLD
```

- 状态：HOLD / TAKEOFF / EXECUTING / COMPLETED / EGO_PLAN_FAILED /
  EGO_EXEC_TIMEOUT / EMERGENCY_BRAKE / BRAKE_HOLD（8 个；NEGOTIATING/POSE_STALE 已删除）
- 任务触发：executor 服务（PREPARE/START），与起飞（arm+OFFBOARD→TAKEOFF）解耦
- 规划循环：plan-exec 三分支（推进/到达/定时）全部置位 + replanLoop 事件唤醒（唯一提交者）
- 轨迹执行：ego 直接发布 `PositionTarget` 30Hz（PVA+yaw），无独立 traj_server
- 落地/完成：距最终关键点 <0.2m（稳定 1s）→ COMPLETED → HOLD
- 失败处理：replan 全失败 → 状态保持重试（重试窗口，≥50s 连续失败才 EGO_PLAN_FAILED）；
  EMERGENCY 且 replan 失败 → ttc<1.0s 制动（EMERGENCY_BRAKE→BRAKE_HOLD）/ ttc≥1.0s 立即
  EGO_PLAN_FAILED；提交段安全校验失败 → 立即 EGO_PLAN_FAILED；executor 收口 → HOLD
- 代码锚点：`ego_planner_driver_node.cpp`（状态机/replanLoop/replanOnce/publishSetpoint）、
  `executor.py`（任务状态机）、`ego_swarm.py:_wait_for_terminal`

---

## 3. 差异对比表

| 维度 | 原版 ego-planner-swarm | 当前 executor + ego_planner_driver |
|---|---|---|
| **运行状态机** | FSM 7 态：INIT/WAIT_TARGET/SEQUENTIAL_START/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP | 状态机 8 态：HOLD/TAKEOFF/EXECUTING/COMPLETED/EGO_PLAN_FAILED/EGO_EXEC_TIMEOUT/EMERGENCY_BRAKE/BRAKE_HOLD（26082621 删 POSE_STALE） |
| 启动 | chain network 顺序启动（SEQUENTIAL_START 等前序轨迹） | executor 任务服务 + 初始 HOLD + GCS 标定统一坐标系；首轮 SEQUENTIAL_START（等前驱 intent/超时） |
| 任务触发 | PRESET_TARGET 航点 / MANUAL goal + `/traj_start_trigger` | executor 服务 `/uav_task`(PREPARE) + `/uav_task_control`(START) |
| 起飞 | 无软起飞（arm 后直接规划执行） | arm+OFFBOARD → TAKEOFF 软起飞（takeoff_height）→ HOLD |
| 规划循环 | FSM：EXEC_TRAJ ↔ REPLAN_TRAJ（定时 replan_thresh / 航点推进 / 碰撞/新peer） | plan-exec 三分支（推进 0.5m / 到达 0.2m / 定时 1.0s）全部只置位 + replanLoop 事件唤醒（唯一提交者） |
| 轨迹执行 | traj_server 独立节点：/planning/bspline → /position_cmd 50Hz | ego 直接发布 PositionTarget 30Hz（PVA+yaw，FRAME_LOCAL_NED） |
| 完成/落地 | 到达终点 t_cur>duration → WAIT_TARGET（悬停等下一个 trigger） | 距最终关键点 <0.2m（稳定 1s）→ COMPLETED → HOLD |
| 避碰架构 | **单层**（本地优化器） | **分层**（规划软绕开 + 单一 replan 入口 + 监督 EMERGENCY 只置位 + executor 门禁兜底） |
| peer 轨迹来源 | 广播网络：`MultiBsplines` + `broadcast_bspline` → `swarm_trajs_buf_`（收到即检查碰撞） | intent 话题广播 + 版本去重 + **生成即发**（26082602 删周期 5Hz 刷新） |
| 时间同步 | 绝对时钟 `glb_time = t_now + 控制点时刻`；接收侧 0.25s 时间窗检查 | rebase 后相对时间 + GPS bias 修正 + epoch 校验（[now-30, now+30]） |
| 避碰 clearance | **固定** `2 × swarm_clearance`（论文常数 `C`） | **固定** `2 × swarm_clearance`（26082602 解耦；`dynamicMargin` 仅监督诊断） |
| 反应时间项 | 无（不区分接近速度/意图年龄） | 无（26082602 从规划层删除；监督 reaction 不含 data_age） |
| 垂直安全区 | **垂直更宽松** `a=2.0` → 垂直半轴=2×水平（论文 `E=diag(1,1,1/c)`） | 对称 `kSwarmA=1.0`（垂直=水平=1m） |
| 硬推(环境) | `check_collision_and_rebound`：grid_map 占用栅格 + A* 路径推出（**环境避障**，不在范围） | 无环境障碍；机间无硬推（当前 `enable_rebound=false`） |
| 提交校验 | `checkTrajectory`：时间同步采样，<clearance 拒 | `checkClearance`：固定 1m 硬门（规划/激活/提交三处） |
| 冲突处理 | 无角色；对等互惠软惩罚（两机同时绕开） | right-of-way（低 ID STAND_ON / 高 ID YIELD）+ 常规 replan 软绕开；监督 EMERGENCY 兜底 |
| 让路候选 | 无 | **有**（26082621 恢复为 replan 失败分支）：SLOW{0.5,0.8,1.2} / LEFT-RIGHT{1.3,1.8,2.2}×{0.7,0.5}，min_clear≥1.2×1m，cost=ΔT/T（绝对值）+dev/5.0 |
| 紧急制动 | 无（EMERGENCY_STOP 为全局急停语义） | 方案 Y：监督只置 emergency_pending_，制动由 replan 线程提交（EMERGENCY_BRAKE→BRAKE_HOLD，不可覆盖） |
| 运行时兜底 | 无 | executor 距离门禁（MIN_DISTANCE_BREACH → HOLD） |
| 失败语义 | replan 失败重试/停（FSM 卡住） | replan 全失败 → **状态保持重试窗口**（1s 定时/事件重试，连续失败 ≥50s → EGO_PLAN_FAILED）；EMERGENCY+失败 → 制动（BRAKE_HOLD）/立即 EGO_PLAN_FAILED；executor 收口 → HOLD / BRAKE_HOLD |

---

## 4. 关键差异深入

> 本节 §4.1–4.5 描述 **26082602 之前**的历史设计演进（周期 5Hz 刷新、规划层动态门槛、tentative
> 协商、监督直接制动等均已废除）；§4.4 的"本次修复"即后续被删除的周期 5Hz 刷新。当前机制见
> §2 流程图与 §6/§7 的"变化后"说明，最终以 `ego_planner_driver/README.md` 为准。

### 4.1 时间同步 vs 意图年龄
- 原版：`glb_time` 用绝对时钟，peer 轨迹插值在 `glb_time - peer_start_time`，
  假定 peer 轨迹**始终正确反映 peer 未来**（无年龄概念）。
- 当前：intent 携带激活 epoch，接收方记录 `data_age = now - 接收时刻`，
  把意图"过时多久"计入反应时间（`reaction = 0.25 + age`）。
  → **周期 5Hz 发布（本次修复）**把 age 从 1.2s 降到 0.1-0.3s，动态距离从 3-4m 回到 ~2m。

### 4.2 固定 vs 动态 clearance（SITL 实证）
- 原版固定 1m 级 clearance：UAV2 直线穿越交叉点（几何可穿），无监督介入。
- 当前动态距离（含反应+制动余量）作为**软惩罚**激励绕开（保留），作为**硬校验**
  会拒绝交叉走廊内一切穿越轨迹 → EGO_PLAN_FAILED（比制动兜底更差）。
  → **本次修复**：`checkClearance` 回退固定 1m，动态距离只做软惩罚。

### 4.3 对等互惠 vs 角色让路（L3→26082621）
- 原版：两机同时绕开（对称），无优先级，编队/交叉时双方都偏离。
- 当前（26082621）：**恢复 right-of-way 角色**（低 ID STAND_ON 保持 nominal、高 ID YIELD 生成
  候选 SLOW/LEFT/RIGHT），但候选**只在 replan 常规规划失败分支**生成（不再 25Hz/10Hz 独立提交）；
  常规规划成功时仍是对等软绕开（原版语义）。监督 EMERGENCY 只置位（方案 Y）。
  历史教训（L3）：单机让路候选（SLOW/FAST/LEFT 2.2m）在 8m 臂正交交叉 + 动态 4m 门槛下
  **几何不足**（SITL 实证：SLOW 预测 3.6m < 4m）——26082602 固定 1m 门槛后该约束消失，
  26082621 恢复候选为 replan 失败分支（不再依赖监督门槛）。

### 4.4 intent 周期刷新（本次新增）
- 修改前：intent 仅 replan 提交时发布（~1.5s/条）→ 邻机 `data_age≈1.2s` → 动态门槛 3-4m。
- 修改后：`publishCurrentIntentTick` 5Hz 以固定 epoch + 递增 traj_id 重发当前轨迹，
  邻机接收时刻持续刷新 → `data_age 0.1-0.3s` → 门槛 ~2m；
  `kNeighborStaleS 5→30s` 让固定 epoch 的长轨迹不被判 stale。

### 4.5 运行状态机差异深入
- **状态归属**：原版 FSM 是**规划节点内单机状态机**（无外部任务层），触发靠
  trigger 话题；当前拆为**两层**——executor（任务服务/状态机）负责任务生命周期，
  ego（飞行状态机）负责 HOLD/TAKEOFF/EXECUTING 等底层飞行，职责清晰、可独立测试。
- **启动**：原版 chain network 顺序启动避免初始轨迹冲突；当前用 GCS 标定统一坐标系
  + executor PREPARE/START 屏障（两机近同时启动）。
- **执行发布**：原版 traj_server 独立节点按 50Hz 采样 B-spline 发 PositionCommand；
  当前 ego 30Hz 直接发 PositionTarget（PVA 同刻采样），少一跳、无独立执行节点。
- **完成语义**：原版"到达终点→WAIT_TARGET 悬停等下一个 trigger"（可循环航点）；
  当前"到达目标→COMPLETED→HOLD"，由 executor 向 GCS 回报任务完成。
- **紧急处理**：原版 EMERGENCY_STOP 是全局急停（callEmergencyStop + fail_safe 恢复）；
  当前是受约束制动轨迹（EMERGENCY_BRAKE→BRAKE_HOLD），可被监督反复触发但
  不可被常规 replan 覆盖（缺陷8 修复）。

---

## 5. 遗留差异与风险
1. 原版机间避碰**同样只有软惩罚**（calcSwarmCost）+ 提交校验（拒绝 → FSM 重试），无硬推；
   原版 `check_collision_and_rebound` 是环境障碍硬推（grid_map+A*），不在当前场景。
   绕开能力差异：原版软惩罚 + 全轨迹 replan 重试；当前软惩罚 + 滚动 replan
   （单测绕开 1.05m @ safe 2.35m）。
2. 原版无监督层，冲突完全靠优化；当前监督仅 EMERGENCY 制动兜底（WARNING 交规划层），
   正交交叉大场景下 UAV 可能"制动兜底"而非"规划绕开"（8m 臂 + 动态 2-4m 门槛
   单机绕开余量不足）。
3. 原版 FSM 单机内聚、执行由 traj_server 独立完成；当前 executor/ego 两层状态机
   交互点多（exec_state 语义、HOLD 确认），需保持 exec_state 与 executor 状态一致。
4. 当前 8m 臂正交交叉场景：动态门槛 2-4m > 单机让路余量 → 制动兜底是安全正确行为；
   若要"让路飞过去"，需更小场景（arm≈3m）或双机协调绕开。

---

## 6. 26082602 计划实施后的 avoidance 差异变化（2026-08-26）

实施 `implementation_plan_26082602.md`（规划层避碰门槛解耦为**固定 clearance** + replan 改
**定时 1s + 周期碰撞检查 10Hz** + 发布回**生成即发**）后，当前版机间避碰**回归原版模型**，
§3 对比表的差异变化如下：

| 维度 | 实施前（当前） | 实施后（预期） | 与原版差异 |
|---|---|---|---|
| 避碰 clearance | 动态（`dynamicMargin` + `data_age`） | **固定** `2 × swarm_clearance` | 与原版**一致**（论文常数 `C`） |
| 反应时间项 | `closing × (control_delay + compute_budget + min(age, cap))` | **删除**（规划层不再消费） | 与原版一致（无 age 概念） |
| 发布策略 | 周期 5Hz 刷新（固定 epoch + traj_id 递增） | **生成即发**（replan 提交时） | 与原版一致（replan 即广播） |
| replan 触发 | 周期 replan + 邻机 intent 失效触发 | 定时 1s + **周期碰撞检查 10Hz** | 对齐原版（原版 20Hz；10Hz 低速场景足够，周期 100ms >> replan 耗时） |
| 时间同步 | rebase 相对时间 + GPS bias 修正 | 保留 | 当前独有（原版假定时钟同步 + 0.25s 时间窗） |
| 监督层 | EMERGENCY 动态门槛制动兜底 | 保留（兜底） | 当前独有（原版无监督） |
| peer 轨迹 | 采样广播（≤50 点，插值重建） | 保留 | 表示法差异（原版 B-spline 参数连续重建） |
| 硬校验 | `checkClearance` 固定 1m | 保留 | 与原版 `checkTrajectory` 对应 |

**关键变化**：
1. **规划层避碰语义等价原版**：固定 clearance + 时间同步采样（GPS bias 修正到本机时间轴）=
   原版"固定 `C` + `glb_time` 绝对采样"——同刻采样本身即"预测邻机未来位置"，`data_age` 动态门槛
   非预测来源，解耦后规划层回到原版单层模型。
2. **发布与避碰解耦**：周期 5Hz 刷新删除（其唯一目的——保持 `data_age` 小——随动态门槛消失），
   发布频率不再影响避碰（同原版）。
3. **事件驱动 replan 删除**：由 10Hz 周期碰撞检查覆盖（作用重复，见计划 §5 备用记录）。
4. **剩余本质差异缩小为三点**（均为当前增强/表示层，非机制差异）：
   - **监督层 EMERGENCY 制动兜底**（原版无监督，当前保留安全兜底）；
   - **GPS bias 时间戳修正**（原版假定时钟同步，当前显式修正实机独立时钟漂移）；
   - **采样广播 vs B-spline 参数**（表示法不同，精度等价）。

**§5 遗留风险相应变化**：
## 7. 26082621 实施后的 avoidance 差异变化（2026-08-26，本文件本次更新）

实施 `implementation_plan_26082621.md`（plan-exec 循环重构 + 单一 replan 入口 + EMERGENCY 走
replan（方案 Y））后，§2 流程图/状态机更新为上文；相对 §6（26082602 后）的 avoidance 差异：

| 维度 | 26082602 后 | 26082621 后 | 与原版差异 |
|---|---|---|---|
| 任务航点 | horizon 窗口滚动（5m，`truncateToHorizon`/`advanceConsumed`） | **3m 密化关键点 + 相邻目标(n+1)**（`advanceTargetLocked`，plan-exec 分支 A） | 对齐原版 `wp_id_++ → planNextWaypoint`（最近相邻目标） |
| replan 触发 | 1s 定时 + 10Hz 碰撞检查 +（监督 EMERGENCY 直接 replan/制动） | **全部只置位**：1s 定时 / 10Hz 冲突 / checkArrival 推进 / EMERGENCY 置 `emergency_pending_`；replanLoop 条件变量事件唤醒，**唯一提交者** | 提交唯一化（原版 FSM 单线程天然唯一）；消除多路径竞争 |
| EMERGENCY 制动 | 监督回调先 replanOnce 再急停 | **方案 Y**：监督只置位唤醒，制动由 replan 线程在"常规 replan 失败且 ttc<1.0s"时提交 | 对齐原版 planFromCurrentTraj 优先语义，提交唯一化 |
| 让路候选 | 10Hz 检测冲突直接提交候选 | **只作 replan 常规规划失败分支**（right-of-way STAND_ON/YIELD，SLOW{0.5,0.8,1.2}/L-R{1.3,1.8,2.2}×{0.7,0.5}，min_clear≥1.2×1m，cost dev/5.0） | 恢复 L3 删除的候选为失败兜底；不再与 replan 竞争 |
| 执行状态机 | 9 态（含 POSE_STALE） | **8 态**（删 POSE_STALE；HOLD 锁存位置/yaw 不随 pose 漂移） | 简化 |
| 到达判定 | `pos_tol_`（launch 0.6m）+ near_final | **距最终关键点 <0.2m**（`arrival_reach_thresh_m`，SITL 复盘由 0.1m 放宽） | 明确分支 B 语义 |

**剩余本质差异（相对原版）不变**：监督 EMERGENCY 兜底（方案 Y 置位）、GPS bias 时间戳修正、
采样广播 vs B-spline 参数、executor 任务层 + 运行时距离门禁。

**SITL 验证**（26082621，双机正交 5 轮）：2 PASS（Run2/4 双方 COMPLETED，min_3d 1.38/1.52m）
/ 3 FAIL 均安全 HOLD（Run1 UAV1 规划失败、Run3 staging 到达 HOLD 已由 0.2m 阈值修复、Run5
min_3d 0.605m 触发 executor 距离门禁）——正交交叉仍为竞争（互惠收敛未完成），EMERGENCY 注入
端到端 PASS（BRAKE_HOLD 由 replan 线程提交）。

- 风险 2"正交交叉制动兜底"：规划层固定 1m 后单机绕开余量更大（无 2-4m 动态门槛挤压），
  "制动兜底"更多由监督层 EMERGENCY 判定，而非规划层门槛过高所致；
- 风险 4"动态门槛 > 单机让路余量"：规划层不再有动态门槛，该几何约束消失（监督层动态门槛
  仍作为制动兜底保留）。

