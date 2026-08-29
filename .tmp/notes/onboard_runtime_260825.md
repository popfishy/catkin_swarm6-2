# 机载程序运行逻辑与缺陷完整说明（2026-08-25）

> ⚠️ **历史快照（2026-08-25）**：本文基于当日代码，多处已过期——如"滚动窗口≤5m"、"shadow 冻结
> /active 待修"、"周期 5Hz 刷新"、"tentative 协商屏障未接线"等。当前行为以
> `src/ego_planner_driver/README.md`（状态机 + replan 单一权威）、`src/tcp_to_ros/plans/README.md`
> （当前状态）与 `src/tcp_to_ros/test_records/`（最新记录）为准。本文仅作当日缺陷/设计参考。

> 目的：完整说明当前机载程序运行逻辑与已知缺陷，辅助决策如何修改机载程序。
> 覆盖：`ego_planner_driver`（底层驱动）、`swarm_uav_executor`（执行器入口）、
> `swarm_uav_interfaces`（交互契约）、`swarm_topology_bridge`（通信桥）、
> `safe_valley_exp`（启动组装）。
> 冻结：`safety_supervisor_mode=active` 冻结（缺陷 2–7 修复前）；stale/missing 为假配置。

---

## 0. 部署形态（多 Master 隔离仿真）

```
┌──────────────────────────────────────────────────────────────────────┐
│ 仿真层  ROS Master 11300 / Gazebo 11345                               │
│   multi_uav_ego_15sim.launch：15 架 PX4 SITL (iris)，无头/有头可选     │
└───────────────┬──────────────────────────────────────────────────────┘
                │ UDP fcu_url: 24539+idx ← 34579+idx（每机 MAVROS 连接）
   ┌────────────┴───────────────────────────────────────────────────────┐
   │ GCS_A  ROS Master 11310              UAVn  ROS Master 11310+n       │
   │   bridge + backend + calibration      （11311..11325）              │
   │   + tcp_server(39001)                MAVROS + ego_planner_driver    │
   │                                      + executor + relay + bridge    │
   │                                      + gp_origin_receiver           │
   └───────────────────┬────────────────────────────────────────────────┘
                       │ ZMQ 跨 Master（Topic 桥 4200+offset / Service 代理 14000+offset）
                       ▼
              runtime（TCP JSON Lines 39001）→ tcp_to_ros → 各 UAV
```

关键：每机一个独立 ROS Master，机间一切通信经 `swarm_topology_bridge`（ZMQ）。
GCS_A 侧 = bridge + backend（整组批次）+ calibration + tcp_server。

---

## 1. 机载层数据流（一次 MOVE_TO 任务）

```
GCS_A backend（tcp_to_ros）
  │ ① UavTask.srv（跨 master Service 代理并发确认：1.0s×3 重试，4.0s 内定 ACCEPTED）
  │ ② UavTaskControl START
  ▼
uav_executor_node.py（executor）
  │ 校验/幂等登记 → store 记录 key=(mission,cmd,uav)；
  │ deadline = prepared_at + UavTask.timeout_s（任务执行超时，GCS 下发）
  │ START 后启动执行线程 → EgoSwarmDriver.start_move_to(goal, cancel, deadline)
  ▼
EgoSwarmDriver（executor 内 Python driver）
  │ ① 发布 goal_yaw（冻结集群参考航向）
  │ ② 构造分层关键点 → 2m 密化 → 发布 /waypoints
  │    MOVE_TO: start→15m层→水平→目标z（首飞先 5m 检查点）
  │    FOLLOW_ROUTE leader: 12m 层；FAULT_EXIT: 8m 层；HOVER: 冻结
  │ ③ _wait_for_terminal 循环监控 exec_state（含 LOCAL_TIMEOUT/DRIVER_TIMEOUT/距离门禁）
  ▼
ego_planner_driver_node（C++，真正算轨迹）
  │ replanLoop(1s)：滚动窗口≤5m → min-snap 单段 → 0.4m采样 → B-spline → L-BFGS
  │   → hard-clearance 复验 → 提交 current_ + 发布 intent
  │ publishSetpoint(30Hz)：从 current_ 同刻采样 P/V/A/yaw → /setpoint/ego(mask 2048)
  │ runSafetySupervisor(25Hz)：shadow 只日志 / active 才制动·让路
  ▼
setpoint_relay（30Hz，MAVROS setpoint 唯一出口）
  │ 校验 frame/mask/finite/freshness（候选超时 0.2s）→ 独占发布
  ▼
/mavros/setpoint_raw/local → PX4
  ▼
exec_state topic → EgoSwarmDriver._on_state → _last_cmd_reply
  │ COMPLETED→成功；EGO_PLAN_FAILED 等→失败收口→安全 hold
  ▼
/uav_task_state → bridge → GCS_A backend（整组 ALL 策略汇总）
```

**铁律**：ego_planner_driver 不直接发 MAVROS setpoint，`setpoint_relay` 是
`/mavros/setpoint_raw/local` 唯一发布者。任何改动不得绕过 relay。

---

## 2. ego_planner_driver（C++ 底层驱动）内部

### 2.1 三并发线程 + 状态机

```
                        ┌─────────────────────────────┐
 waypoints/goal ──────▶ │ replanLoop 线程 (1s)         │
 local_pose ──────────▶ │  滚动窗口截取 ≤5m 航点         │
 neighbor_intent ─────▶ │  min-snap 单段闭式解           │
                        │  0.4m 弧长采样 → B-spline      │
                        │  L-BFGS (jerk+feasibility)     │
                        │  activation 重基准(≤3 次)      │
                        │  commit-lock hard-clearance    │──▶ current_（共享轨迹）
                        └─────────────────────────────┘          │
                                                                  ▼
                        ┌─────────────────────────────┐   ┌─────────────────┐
                        │ publishSetpoint 线程 (30Hz)  │──▶│ /setpoint/ego   │
                        │  current_ 同刻采样 P/V/A/yaw │   │ (mask 2048)     │
                        │  轨迹耗尽 → 固定 HOLD(2552)   │   └─────────────────┘
                        └─────────────────────────────┘
                        ┌─────────────────────────────┐
                        │ runSafetySupervisor (25Hz)   │──▶ runtime_logs/ego_planner/
                        │  SafetyPredictor: CPA/TTC/   │    UAVn-ego-planner.log
                        │  动态两级门槛/ID role/episode │
                        │  shadow: 只记录不改行为        │
                        │  active: EMERGENCY→制动轨迹    │
                        │          WARNING→YIELD 让路候选│
                        └─────────────────────────────┘
```

### 2.2 exec_state 状态机

```
HOLD(初始) ──goal──▶ [NEGOTIATING?] ──▶ EXECUTING ──到达稳定1s──▶ COMPLETED
   ▲                   │(barrier未接生产)│      ├─姿态超时─▶ POSE_STALE ─▶ HOLD(自愈)
   │                   └──────直接激活────┘      ├─规划失败─▶ EGO_PLAN_FAILED ─▶ HOLD
   └────────────────────────────────────────────┴─超时─────▶ EGO_EXEC_TIMEOUT
                                            [active] EMERGENCY_BRAKE ─▶ BRAKE_HOLD(任务失败)
```

### 2.3 关键事实
- 阶段 C tentative 协商屏障**只存在于测试路径 `replanOnce()`**，生产 `replanLoop()`
  **直接提交 ACTIVE**（未接线）→ 当前生产无 1s 协商延迟。
- stale/missing 三参数（`neighbor_stale_policy`/`neighbor_intent_stale_s`/
  `neighbor_missing_policy`）为**假配置**：launch 存在，C++ 未读取。
- 已删除过期参数 `terminal_to_idle_s`/`idle_hover_refresh_hz`（IDLE 概念已废弃）。

---

## 3. swarm_uav_executor（执行器入口）内部

### 3.1 节点组成
```
uav_executor_node.py   任务 Service(UavTask/UavTaskControl/UavHold) + store + 状态发布
                        required=true：被杀连带关闭整个机载层 launch
ego_swarm.py           EgoSwarmDriver：动作生成/分层/距离门禁/hold
setpoint_relay_node.py MAVROS setpoint 唯一出口（选源/generation/freshness/finite）
pose_relay_node.py     /mavros/local_position/pose → /local_pose
gp_origin_receiver.py  GCS 下发原点 → MAVROS 回读确认
```

### 3.2 动作分发
```
start_move_to(goal, cancel, deadline)
  ├─ HOVER ─────────────▶ _plan_horizontal（冻结 waypoints，原地保持）
  ├─ FOLLOW_ROUTE+follower─▶ _follower_loop（PI 跟踪 leader + 编队槽位）
  └─ MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE(leader) ─▶ _plan_horizontal（分层关键点→2m 密化→/waypoints）
```

### 3.3 超时体系（2026-08-25 已完全解耦 + 按语义命名）

```
┌────────────────────────────────────────────────────────────────┐
│ launch 参数             → ROS param              → 使用方         │
├────────────────────────────────────────────────────────────────┤
│ uav_task_timeout_s(200s) → ~ego_swarm/task_timeout_s → driver    │
│   = 任务执行超时（DRIVER_TIMEOUT 兜底）            self.task_timeout_s│
│                                                              │
│ ego_hold_timeout_s(2s)  → ~ego_hold_timeout_s  → executor      │
│   = ego HOLD/收口超时（HOLD 确认/任务失败回退/shutdown join）    │
│                                                              │
│ 任务级 UavTask.timeout_s → record.deadline → start_move_to     │
│   = GCS 下发的任务时限（LOCAL_TIMEOUT）                         │
└────────────────────────────────────────────────────────────────┘
  三者互不引用；hold() 只使用调用方 deadline，无 state_timeout 兜底。
```

### 3.4 线程模型
- 每任务一个执行线程 + cancel_event；`_run_hold` 独立线程处理 HOLD 抢占。
- 任务失败 → 自动 `driver.hold(..., clock()+ego_hold_timeout_s)` 安全悬停 → FAILED 终态。

---

## 4. swarm_uav_interfaces（交互契约）

| 类型 | 用途 | 备注 |
|---|---|---|
| `UavTask.srv` | 任务下发（PREPARE 登记） | 全结构化，`accepted=true` 只表示登记成功 |
| `UavTaskControl.srv` | START/ABORT | 两阶段启动屏障 |
| `UavHold.srv` | 安全悬停 | 高优先级，可抢占 |
| `UavTaskState.msg` | 状态回传（ACCEPTED/COMPLETED/FAILED） | `status_seq` 去重 |
| `UavLinkHealth.msg` | 链路健康（DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED） | 取代 lease |
| `UavTrajectoryIntent.msg` | 机间轨迹意图（协议 2.0，phase=TENTATIVE/ACTIVE/BRAKING） | MD5 已变，改 msg 全机同批重建 |
| `UavSafetyLease.srv` | dead definition | 保留不删 |

⚠️ **改 msg = MD5 变更 = 所有 GCS/UAV/bridge 同批重建**，否则跨 master 类型不匹配、消息被丢弃。

---

## 5. swarm_topology_bridge（通信桥）

```
┌─────────────┐  ZMQ Topic 桥 (base_port 4200+offset×100)   ┌─────────────┐
│ UAVn Master │ ── uav_task_state / mavros/state / odom ──▶ │ GCS_A Master│
│  bridge_node│ ── trajectory_intent / gp_origin ──────────▶ │  bridge_node│
│             │ ◀── 反向（GCS→UAV）───────────────────────── │             │
└─────────────┘  Service 代理 (14000+offset×100)            └─────────────┘
   target=UAVn → 绑 ROUTER 调本机 Service；否则注册代理转发
   max_freq=0 不限频；被桥 topic 自动加源前缀 /UAVn/*
   intent 只做 ROS 原生 bytes 透传，不理解 phase
```

- 每机一个实例；同一份 `tcp_to_ros/config/topology_group_a_sim.yaml`。
- Service 请求带 bridge 唯一 ID，支持并发/超时/迟到响应丢弃。
- **不解释业务**：改业务语义不要塞进 bridge。

---

## 6. safe_valley_exp（启动组装，仅 launch/sh/takeoff）

### 6.1 启动链
```
multi_uav_ego_15sim.launch    仿真层（11300/11345，15 架 iris SITL）
startup_offboard_ego.sh 1..15 机载层：逐机 roslaunch uav_offboard_ego.launch（11311..11325）
  └─ uav_offboard_ego.launch
      ├─ ① MAVROS px4.launch（udp://:24539+idx@localhost:34579+idx, tgt_system=idx）
      ├─ ② uav_executor_ego.launch（ego driver + executor + relay + pose_relay）
      ├─ ③ topology_group_a_sim.launch（bridge，run_uav_mock=false）
      └─ ④ gp_origin_receiver.py
offboard_takeoff_15.py 1..15  起飞：注入 COM_RCL_EXCEPT=4/NAV_RCL_ACT=0 → HOLD → arm → OFFBOARD
```

### 6.2 startup_offboard_ego.sh 关键
- 每机独立 Master；`neighbor_intents` = 其余 14 机 `/UAVn/trajectory_intent`（全 mesh）。
- `EGO_REBOUND_UAVS`（默认全关）、`EGO_SAFETY_SUPERVISOR_MODE`（off/shadow/active，默认 shadow）。
- ⚠️ active 冻结：缺陷 2–7 修复前禁止用 active 跑 SITL。
- 日志/pid 写 `$WS/.tmp/logs/`。

### 6.3 offboard_takeoff_15.py
- 只做 `HOLD → arm → OFFBOARD`，**不起飞**（无 MAV_CMD_NAV_TAKEOFF）；起飞由 ego driver
  `TAKEOFF` 状态自动完成（takeoff_height_m=5.0）。先切 Auto 模式规避 PX4 无 RC arm 拒绝。

---

## 7. 缺陷清单（template.md 审查 + 后续处理）

| # | 缺陷 | 状态 |
|---|---|---|
| 1 | `hold()` 忽略调用方 deadline 可阻塞 200s | ✅ **已修复**（6c2b02a→58b6ecb：只用 caller deadline，`ego_hold_timeout_s`=2s，与任务超时完全解耦） |
| 2 | `yield_committed_` 使 WARNING 无条件早退，不再重新验证候选 | ⬜ 待修 |
| 3 | 候选 1m 验证与运行时动态 emergency 门槛不一致 | ⬜ 待修 |
| 4 | 0.6837m EMERGENCY 触发时 intent 年龄 17ms，怀疑时间轴不一致 | ⬜ 待修（需复现） |
| 5 | stale/missing 假配置（launch 有、C++ 未实现） | ⬜ 待决定（实现或删除） |
| 6 | 候选速度上限 1.5m/s 与验收 1.5m/s 无余量（实测峰值 1.58m/s） | ⬜ 待修 |
| 7 | 旧 executor 1m 门禁需 freshness + A/B + 触发统计 | ⬜ 待修 |

**冻结**：缺陷 2–7 全部修复并回归前，生产保持 `safety_supervisor_mode=shadow`，SITL 禁止 active。

---

## 8. 修改决策速查表

| 想改什么 | 改哪个文件/包 | 注意 |
|---|---|---|
| 轨迹平滑/规划行为 | `ego_planner_driver` C++（replanLoop/planOnce/optimizer） | 128 项单测；shadow 下不改行为 |
| 主动避碰 active（制动/让路） | `ego_planner_driver` safety_predictor/yield/emergency | **冻结**：先修缺陷 2–4、6 |
| 动作分层/到达判定/编队 PI | `swarm_uav_executor/drivers/ego_swarm.py` | 分层高度/密化参数在 launch |
| 任务接口/状态协议 | `swarm_uav_interfaces` msg/srv | MD5 变更 → 全机同批重建 |
| 跨机传输/消息路由 | bridge 配置 yaml 或 `bridge_node.py` | 业务语义勿塞 bridge |
| 启动/起飞/参数注入 | `safe_valley_exp` launch/sh/takeoff | 换环境必改绝对路径 |
| 整组批次/确认时序 | `tcp_to_ros` backend（GCS 侧） | 本轮未动，协议可信 |

---

## 9. 当前可信基线

- ✅ **可用**：通信链路（bridge/Service/TCP 未动）、min-snap 轨迹执行/分层/起飞、
  shadow 预测（只写日志不改行为）、超时体系（已解耦命名）。
- 🚫 **冻结**：`safety_supervisor_mode=active`（缺陷 2–7 待修）。
- ⚠️ **假配置**：stale/missing 三参数未接入生产 C++。
- ⚠️ **barrier（阶段 C）未接入生产** `replanLoop()`，仅测试路径生效。
- 测试：239 tests 0 failures（含 closed-loop 动力学 10 项）；环境无残留进程。
- 提交（未推送）：swarm_uav_executor 6c2b02a/58b6ecb/e602762/e7c6774；
  safe_valley_exp bb8ccdf/536c0cb；tcp_to_ros 435c794/cc0fd97/95c411e 等。

---

## 10. 26082602 计划实施预期效果（缺陷视角，2026-08-26）

实施 `implementation_plan_26082602.md`（避碰门槛解耦为固定 clearance + 定时 1s/周期 10Hz
碰撞检查 replan + 发布回生成即发）后，§7 缺陷清单的预期变化：

| # | 缺陷 | 实施后预期 |
|---|---|---|
| 1 | `hold()` 忽略 deadline 阻塞 200s | ✅ 无影响（已修复并保留） |
| 2 | `yield_committed_` 使 WARNING 无条件早退 | ✅ **消除**：yield 候选已随 L3 删除；26082602 解耦动态门槛后规划层无 WARNING 让路逻辑 |
| 3 | 候选 1m 验证与动态 emergency 门槛不一致 | ✅ **消除**：候选已删；规划层回固定 clearance（"候选动态门槛一致性"概念不存在） |
| 4 | EMERGENCY 触发时 intent 时间轴不一致（age 17ms 疑点） | 🟡 **缓解**：GPS bias 修正（`T_own = T_other - bias_other + bias_own`）保证同刻采样时间轴对齐；固定 clearance 简化时间处理；需 SITL 复现确认 |
| 5 | stale/missing 假配置（launch 有、C++ 未实现） | 🟡 **规划层影响消除**：固定 clearance 不依赖 `data_age`，规划层不再需要 stale 策略；监督层保留 `data_age`（cap 2s）更保守；原三参数可标注"规划层不再消费" |
| 6 | 候选速度上限 1.5m/s 无余量 | ✅ **消除**：候选已删（无速度档位概念） |
| 7 | 旧 executor 1m 门禁需 freshness + A/B + 触发统计 | ⬜ 无影响（保留兜底；freshness/A/B 仍为阶段 E 独立事项） |

**预期结论**：
- **缺陷 2/3/6 随 L3 精简 + 本计划解耦而消除**（不再是缺陷）；
- **缺陷 4/5 缓解**（GPS bias 时间戳修正消除时间轴不一致根因；规划层不再依赖 `data_age`）；
- **缺陷 1/7 无影响**（1 已修复保留；7 是兜底门禁的独立优化）。
- 监督层 SafetyPredictor 的 EMERGENCY 动态门槛**保留**作为制动兜底，不受规划层解耦影响。
