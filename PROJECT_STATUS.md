# A/B 实物无人机对接项目进度与交接记录

更新时间：2026-08-17（基于当前工作区源码、历史测试记录和 Git HEAD 整理）

本文档用于后续对话和 coding agent 快速恢复上下文。它记录“现在代码实际做了什么、哪些结论有测试证据、A 组目前卡在哪里”，不替代协议方案和逐步测试手册。

## 1. 当前结论

1. `references/runtime/` 已完成上层行为树任务、任务包校验、Group A/B TCP JSON Lines 客户端；B 组已经完成对接。
2. A 组的 TCP 接入、命令解析、GCS_A 整组聚合、跨 ROS Master bridge、逐机 executor，以及向 `ego_planner_driver` 转发的基础链路已经打通。mock 链路和部分历史 PX4 SITL 链路有证据支持这一点。
3. 当前主要风险不在 TCP 字节收发，而在“任务坐标/航点经过转换后，EGO driver 如何生成并下发最终轨迹”。最新的六批次 SITL 结果尚未在最近的 lease 删除、HOLD 竞态修复之后重跑。
4. 源码中存在一处明确的语义漂移：`validation.py` 和验证脚本仍按“分层、先垂直后水平”描述，但当前 `EgoSwarmDriver.start_move_to()` 已把 MOVE_TO/FAULT_EXIT/leader FOLLOW_ROUTE 直接交给 C++ EGO 规划，`layer_z` 实际未被执行。
5. 当前 HOVER 也存在同类漂移：runtime 的 HOVER 线协议只发送 UAV ID，约定由下层冻结当前位姿；但当前 executor 会把零占位目标交给 EGO `/goal`，存在生成 `(0,0,0)` 目标的风险，不能把它当作“当前位姿悬停”已经验证。

## 2. 工作空间和职责边界

```text
references/runtime                 上层行为树、任务包、TCP Client（独立 Python 仓库）
        │ TCP 39001 / JSON Lines
        ▼
src/tcp_to_ros                     A 组 GCS：TCP Server + ExecuteGroupTask 聚合
        │ 跨 Master Service/Topic
        ▼
src/swarm_topology_bridge          通用 ZMQ Topic/Service 代理，不解释任务语义
        │
        ▼
src/swarm_uav_executor              每机身份校验、幂等、PREPARE/START/ABORT、HOLD、状态上报
        │
        ▼
src/ego_planner_driver              C++ B-spline/L-BFGS 轨迹规划、轨迹意图、30 Hz setpoint
        │
        ▼
MAVROS / PX4 SITL 或实机             实际飞控执行
```

工作空间不是一个统一 Git 仓库；`src/` 下各功能包分别维护 Git 历史，`references/` 和 `verification/` 是参考资料/工程验证目录。

主要权威资料：

- 总体 TCP 协议和交付边界：[`references/docs/实物无人机上下层TCP-IP接口对接方案.md`](references/docs/实物无人机上下层TCP-IP接口对接方案.md)
- A 组现有交接入口：[`src/tcp_to_ros/README.md`](src/tcp_to_ros/README.md)
- 领域词汇和坐标约定：[`src/tcp_to_ros/CONTEXT.md`](src/tcp_to_ros/CONTEXT.md)
- 最新历史实测记录：[`src/tcp_to_ros/docs/test_record_26081402.md`](src/tcp_to_ros/docs/test_record_26081402.md)
- EGO/仿真逻辑说明：[`verification/safe_valley_exp_and_ego_planner_driver_logic.md`](verification/safe_valley_exp_and_ego_planner_driver_logic.md)

## 3. 从 runtime 到最终 setpoint 的实际链路

### 3.1 runtime 侧

`references/runtime/src/uav_bt_runtime/tcp_transport.py` 将一个 Group 批次聚合成一条 TCP JSON Lines 消息：

- `version="1.0"`、`type="COMMAND"`、`mission_id`、`group_id`、`command_id`；
- 一个整组 `assignments` 数组，不是每架飞机独立 TCP 会话；
- 5 秒内未收到 `ACCEPTED` 会主动发送 `HOLD`；
- 支持 Group A：`MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`；支持 Group B：`ATTACK`、`RETURN`。

当前任务包的 Group A 批次：

| 任务 | 线上命令 | 参与集 | 规划意图 |
|---|---|---|---|
| `prepare-a` | `MOVE_TO` | A01-A12 | 进入准备编队，目标高度 12 m |
| `coverage-segment-1` | `FOLLOW_ROUTE` | A01-A12 | A01 为 leader，A02-A12 编队跟随 |
| `fault-exit` | `FAULT_EXIT` | A07-A12 | 每架机各自撤离航线，规划高度 8 m |
| `recovery-a` | `MOVE_TO` | A01-A06、A13-A15 | 9 机恢复编队，目标高度 12 m |
| `coverage-segment-2` | `FOLLOW_ROUTE` | A01-A06、A13-A15 | 第二段覆盖航线 |
| `hold-a-end` | `HOVER` | A01-A06、A13-A15 | 末端稳定悬停 |

runtime 任务坐标是右/前/上（`x=右、y=前、z=上`），不是直接的 ROS ENU。

### 3.2 `tcp_to_ros` 边界

`src/tcp_to_ros/scripts/tcp_server_node.py` 负责拆包、JSON/协议校验、去重和 Action 转发；`gcs_a_backend_node.py` 再把一个整组 Action 拆为逐机 `UavTask` Service：

1. 任务层身份 `A01...A15` 通过 [`config/uav_identity_map.yaml`](src/tcp_to_ros/config/uav_identity_map.yaml) 转换为执行层 `UAV1...UAV15`。
2. `MOVE_TO` 的 `target_pose`、`FAULT_EXIT` 的每个 waypoint、`FOLLOW_ROUTE` leader 的每个 waypoint，都会调用 `task_point_to_local_enu()`。
3. 变换使用冻结的 `reference_heading` 和 `mission_anchor=[10,5,0]`；runtime yaw 不作为飞行朝向，转发时 yaw 被设为冻结的集群参考航向。
4. FOLLOW_ROUTE follower 不接收自己的路线，而是由 GCS_A 注入固定编队槽位；当前 leader 固定为 A01，编队集合必须精确匹配 `formation_offsets.yaml`。
5. GCS_A 只有在所有参与 UAV 的 `UavTask` 都 `accepted=true` 后才向 runtime 报 `ACCEPTED`；之后通过 `UavTaskState` 汇总 `COMPLETED/FAILED`。

### 3.3 executor 和 EGO 接口

`src/swarm_uav_executor/src/swarm_uav_executor/validation.py` 将 ROS `UavTask` 构造成 `MotionGoal`：

- `MOVE_TO`：单个目标点，`layer_z=15`；
- `FOLLOW_ROUTE` leader：waypoints，`layer_z=12`；follower：leader odom + `formation_offset` 的 PI 跟随；
- `FAULT_EXIT`：waypoints 或单点目标，校验 z≈8，`layer_z=8`；
- `HOVER`：runtime 只给 UAV ID 时使用零占位 pose，注释约定应由 driver 冻结当前 pose。

`EgoSwarmDriver` 发布/订阅：

| 方向 | Topic | 用途 |
|---|---|---|
| 入 | `/goal` | 单点 EGO 目标 |
| 入 | `/waypoints` | leader 航线 |
| 入 | `/goal_yaw` | 冻结参考航向 |
| 入 | `/local_pose` | MAVROS 位姿中继 |
| 入 | `/hold` | 安全悬停 |
| 入 | `/neighbor_intent` | 邻机带时轨迹 |
| 出 | `/exec_state` | `HOLD/TAKEOFF/EXECUTING/COMPLETED/...` |
| 出 | `/setpoint` | 30 Hz 位置 setpoint |
| 出 | `/trajectory_intent` | 本机规划轨迹给邻机 |

`setpoint_relay_node.py` 再把 `/setpoint` 转发到 `/mavros/setpoint_position/local`。

## 4. 已完成项和证据边界

### 已完成或基本贯通

- runtime TCP Client、A/B JSON Lines 协议、去重、状态时序和任务包校验已实现；runtime 当前仓库 HEAD 为 `54ac280`，远程为 `git@github.com:popfishy/runtime.git`。
- A 组 mock 全链路曾在 [`test_record_26081021.md`](src/tcp_to_ros/docs/test_record_26081021.md) / [`test_record_26081115.md`](src/tcp_to_ros/docs/test_record_26081115.md) 中通过：TCP → GCS_A → bridge → executor；四种动作的 mock 结果为 `SUCCESS`。
- 历史 15 机 PX4/Gazebo 测试已经证明仿真层、15 机机载启动、MAVROS connected、标定、原点回读、arm/OFFBOARD、5 m 软起飞和 setpoint 流可以建立（见 `test_record_26081402.md`）。
- `gcs_a_backend` 的整组 PREPARE/START/ABORT、幂等、失败即停、HOLD、link_health 和状态去重均有单测/集成测试。
- EGO C++ core 已实现 B-spline 参数化、L-BFGS 平滑/动态可行性/互惠避碰代价、轨迹采样、终点锚定和硬 clearance 检查；`ego_planner_driver` 自带状态机和轨迹 gtest。
- 最新设计已删除机载 safety lease：`swarm_uav_executor` HEAD `0674e87`，共享接口 HEAD `8223794`；断链安全职责改由 GCS_A `link_health + safe_hold` 承担。
- `ego_planner_driver` HEAD `192ad03` 已修复 HOLD 空闲捕获与待规划 goal 的竞态：收到新 goal 后，30 Hz setpoint 不再抢先重新捕获 `steady_target_`。
- `verification/ego_smoke/` 提供单机生命周期、双机 reciprocal、15 机起飞和四种动作冒烟入口；其中 `smoke_commands.py` 的断言仍是旧的“垂直先行/HOVER 冻结”语义，不能直接当作当前 `start_move_to()` 已通过的证据。

### 尚不能宣称完成

- lease 删除、HOLD 竞态修复之后，尚无新的 15 机六批次端到端 PASS 记录；README 中“prepare 六批次重跑”仍是待办。
- `test_record_26081402.md` 的 prepare 失败根因是旧 lease 机制，不能直接作为当前版本执行层根因；它说明的是“修改前”历史状态。
- 历史 `test_record_26081123.md` 中，链路已贯通但真实 SITL 六批次仍出现单机/批次失败；失败集中在 EGO 执行、状态机和 setpoint 竞争，而非 TCP 解析。
- 当前 `build/test_results` 没有可复用的最新测试结果；文档中“133 tests 全绿”是 2026-08-13 的历史记录，不能替代最近版本重跑。
- 本次检查尝试直接运行 `test_ego_swarm_driver.py` 时，当前环境缺少 catkin 生成的 `swarm_uav_interfaces` Python 模块；因此不能把该次收集失败解释成业务逻辑失败，后续应先完成工作空间构建/消息生成，再重跑 catkin test。
- 当前 `references/runtime/task_test.py` 的 CLI 任务名是 `prepare`、`coverage-segment-1`、`fault-exit`、`recovery`、`coverage-segment-2`、`hold-end`，不接受历史记录中的 `--timeout-s` 参数；复测应以当前脚本 `--help` 和任务包定义为准。

## 5. 当前最值得优先排查的代码问题

### 5.1 分层语义在当前 driver 中没有真正执行

`MotionGoal.layer_z` 仍在模型和校验层设置，但当前 `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py` 的 `start_move_to()` 逻辑是：

- MOVE_TO：直接 `_plan_horizontal(goal, ...)`；
- FAULT_EXIT：直接 `_plan_horizontal(goal, ...)`；
- FOLLOW_ROUTE leader：直接 `_plan_horizontal(goal, ...)`；
- FOLLOW_ROUTE follower：只运行 PI 跟随；
- HOVER：也直接 `_plan_horizontal(goal, ...)`。

历史提交 `5707c93` 为“让所有轨迹走 C++ EGO”删除了此前 `98d2e6f/e4e6f7f` 中的 `_plan_vertical_transition()`。因此当前代码与方案中“MOVE_TO 先 15 m 垂直、FAULT_EXIT 先 8 m 垂直、FOLLOW_ROUTE 先 12 m 分层”的约定不一致。

这会带来两个直接后果：

1. `prepare-a` 从非 15 m 高度开始时，EGO 可能直接规划一条同时改变 x/y/z 的长空间轨迹；30 秒或 runtime 共享 deadline 可能被垂直距离耗尽。
2. `fault-exit` 的 runtime waypoints 全部 z=8，但第一 waypoint 就是远处水平点；没有显式的“当前 x/y 先降到 8 m”阶段，路径会斜向下降并同时水平移动。

建议后续先明确设计选择：恢复 driver 层的受约束垂直阶段，或在上层生成带垂直入口/出口的 waypoint 序列；不要只依赖 `layer_z` 注释。

### 5.2 HOVER 语义可能变成飞向零点

runtime 的 `hold-a-end` 计划里实际保存的是 `hold_pose`，但 TCP `_assignment()` 对 HOVER 只发送：

```json
{"uav_id":"A01"}
```

这是协议设计允许的：地面站应使用飞机当前位姿悬停。当前 GCS_A 不会为 HOVER 注入 pose，executor validation 会生成零占位 `MotionGoal`，而 `EgoSwarmDriver` 会把它送到 C++ `/goal`。因此必须单独验证 HOVER 是否真的冻结当前位姿；按当前源码路径，它存在向 `(0,0,0)` 规划的风险。

安全 HOLD（runtime 超时/取消/断线）走的是另一条 `/hold` → `UavHold` 路径，不应与正常 HOVER 混淆。

### 5.3 航点过稀，密化没有接入实际执行链

当前 Python driver 只把中间 waypoint 重复一次，目的是让 B-spline 不立即穿过关键点；它没有按最大间距插值。

当前 C++ core 只在 `start + waypoints` 少于 5 个点时做中点细分。Group A coverage 路线虽然有多个关键点，但相邻航段可达几十米，仍然是稀疏关键点。`verification/plot_ego_bspline_waypoints.py` 展示了密化思路，但没有被 `EgoSwarmDriver` 或 C++ core 调用。

建议先固定一个单机 route，记录：原始 waypoints、driver 发布的 Polygon、C++ `TimedTrajectory` 采样点、最终 `/setpoint`，再决定密化应放在 Python driver 还是 C++ core。

### 5.4 坐标变换必须用已冻结的任务约定核对

GCS_A 使用：

```text
dx = task_x - 10
dy = task_y - 5
(east, north) = task_vector_to_enu(dx, dy, reference_heading)
local_z = task_z - anchor_z
```

这里的 `task_x/task_y` 是 runtime 任务坐标，不是 Gazebo 世界坐标。A 组现场联调时必须同时记录：runtime 原始点、`reference_heading`、`mission_anchor`、GCS_A 转换后的 ENU 点、机载 `/goal` 或 `/waypoints`，否则无法判断是坐标变换问题还是 planner 问题。

## 6. 推荐排查顺序

1. **单机、单目标、绕过完整 BT**：用一个 A ID 发送 `MOVE_TO`，确认 TCP JSON、ROS Action、`UavTask` 请求和 `/goal` 的数值逐层一致。
2. **固定坐标样例**：使用已知 `reference_heading=0` 和 `mission_anchor=[10,5,0]`，例如任务点 `(17.5,50,12)` 先得到 `dx=7.5、dy=45`，按右/前→ENU 约定应转换为 `(east=45,north=-7.5,z=12)`；再测试非零 heading 的旋转结果。
3. **记录 EGO 输入输出**：同时抓 `/local_pose`、`/goal`、`/waypoints`、`/exec_state`、`/setpoint`、`/trajectory_intent` 和 MAVROS pose/state。
4. **先解决语义问题**：明确并实现垂直分层和 HOVER 当前位姿冻结，再调 B-spline 参数；否则 planner 调参会掩盖上层目标错误。
5. **接入航点密化**：规定最大相邻点间距（建议先用 2 m 作为实验值），并在单测中检查输入路线和输出轨迹端点。
6. **逐级验收**：
   - `ego_planner_driver` C++ core/state-machine 单测；
   - 单 Master EGO 冒烟；
   - 双机 reciprocal 冒烟；
   - 单机真实 PX4 SITL `MOVE_TO`；
   - 2~3 机真实 SITL route；
   - 15 机先只跑 `prepare`；
   - 最后按 runtime 顺序跑六批次。
7. 每一步都要记录 command_id、参与 UAV、原始/转换后坐标、开始/终止时间、终态和失败机，不能只记录 runtime 的 SUCCESS/FAILURE。

## 7. 验收标准（后续任务完成的最低证据）

- `MOVE_TO`：先保持 x/y/yaw 不变到 15 m 层，再水平到目标，最后到目标 z；整批共享同一个 runtime deadline。
- `FOLLOW_ROUTE`：A01 的路线输入顺序正确；follower 使用 GCS_A 注入的槽位，leader 丢失能进入失败/HOLD；相邻机间距满足 1 m 水平、2 m 垂直门禁。
- `FAULT_EXIT`：每架 A07-A12 使用自己的路线；先下降至 8 m 再水平撤离，不复制 A07 的路线。
- `HOVER`：必须冻结收到命令时的本机当前 pose，不得生成零点目标。
- 失败/超时/断线：GCS_A 汇总为整组 `FAILED`，并对已 START 且未终止的参与机发安全 HOLD。
- 只有在上述项目通过真实 PX4 SITL 六批次后，才可进入实机不上桨联调；SITL 不能替代现场 failsafe/急停门禁。

## 8. 当前 Git 状态快照

| 仓库 | 当前 HEAD | 说明 |
|---|---|---|
| `references/runtime` | `54ac280` | 已上传 `github.com/popfishy/runtime` |
| `src/tcp_to_ros` | `2398aed` | lease 删除后的 GCS 逻辑；当前有 2 个文件仅权限位未提交改动 |
| `src/swarm_uav_interfaces` | `8223794` | `UavSafetyLease` 标记为 dead definition |
| `src/swarm_topology_bridge` | `7328814` | 当前有 4 个文件仅权限位未提交改动 |
| `src/swarm_uav_executor` | `0674e87` | 已删除 onboard safety lease；当前有 3 个文件仅权限位未提交改动 |
| `src/ego_planner_driver` | `192ad03` | HOLD 捕获竞态修复，工作区干净 |
| `src/safe_valley_exp` | `b53ffc7` | 起飞脚本和仿真工具；当前有若干文件仅权限位未提交改动 |

这些未提交改动经检查是 executable bit 从 `100755` 变为 `100644`，没有语义代码 diff。后续操作不要用 `git reset --hard` 或宽泛清理覆盖它们。

## 9. 常用端口和现场约束

```text
仿真 Master       11300
GCS_A Master      11310
UAVn Master       11310+n（UAV1=11311 ... UAV15=11325）
Gazebo            11345
TCP               39001
```

- 原点/参考航向标定必须在所有相关 UAV disarm 时完成；A01-A12 参与计算，A01-A15 必须回读确认。
- `uav_identity_map.yaml` 中的实机 IP/hostname 仍需现场人工复核；仿真映射不能直接当作实机配置。
- EGO driver 不应自动 arm/OFFBOARD；现场 failsafe、RC、datalink、急停职责必须另行冻结。
- 启动/清理多进程时使用精确 PID；不要使用可能误杀当前 shell 或用户仿真的宽泛 `pkill -f`。
