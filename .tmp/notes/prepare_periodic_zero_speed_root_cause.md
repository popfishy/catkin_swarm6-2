# prepare 任务速度周期性降零诊断

日期：2026-08-23

> ⚠️ **历史快照（2026-08-23）**：本文诊断的"每 ~4s 滚动重规划 + 起点锚定快照"已随
> 26082415 min-snap 统一单段重构与 26082621 单一 replan 入口改写。仅作历史参考；当前机制以
> `src/ego_planner_driver/README.md` 为准。

## 结论

当前证据支持以下根因链，置信度高：

1. `ego_planner_driver` 每约 4 秒滚动重规划一次；
2. 新轨迹的位置起点锚定重规划快照时的 UAV 实际位置；
3. 规划器内部虽然计算并尝试延续 B-spline 的速度/加速度边界，但输出链路只发布
   `geometry_msgs/PoseStamped`；
4. `setpoint_relay_node.py` 又原样把该位置消息转发给
   `/mavros/setpoint_position/local`；
5. 因此 PX4 没有收到规划轨迹的速度、加速度前馈。每次切换到以实际位置为起点的新轨迹时，
   PX4 位置误差会瞬时接近零，位置环的驱动量随之下降，表现为实际速度按重规划周期下降，随后
   又随着移动位置 setpoint 拉开而重新加速。

这不是 relay 丢消息或轨迹空档导致的周期 HOLD：ULog 中位置 setpoint 持续存在，而且其自身运动
速率也呈约 4.5 秒主周期。问题是“用纯位置接口表达带速度语义的滚动轨迹”，导致每次轨迹切换
丢失导数连续性。

## 证据

### 1. 周期与重规划参数锁定

- 运行日志中所有 UAV 均加载：
  - `planning_horizon = 5.0`
  - `replan_period_s = 4.0`
- launch 定义见：
  - `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/launch/ego_planner_driver.launch:19`
  - `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/launch/ego_planner_driver.launch:20`
- 对 UAV1、UAV2、UAV6、UAV12 的 PX4 ULog `trajectory_setpoint` 位置做差分并自相关，
  四机均得到约 **4.5 s** 的主周期。4.0 s 触发值叠加规划计算、定时器调度和飞控响应后出现
  约 4.5 s 的观测周期是吻合的。

抽样结果：

| UAV | PX4 位置 setpoint 导数主周期 |
|---|---:|
| UAV1 | 4.5 s |
| UAV2 | 4.5 s |
| UAV6 | 4.5 s |
| UAV12 | 4.5 s |

### 2. PX4 收到的 setpoint 没有速度前馈

同一批 ULog 的 `trajectory_setpoint` 包含 `vx/vy/vz` 字段，但抽样四机在分析窗口内均为 NaN：

| UAV | 有限速度前馈样本 / 总样本 |
|---|---:|
| UAV1 | 0 / 750 |
| UAV2 | 0 / 750 |
| UAV6 | 0 / 750 |
| UAV12 | 0 / 750 |

位置 setpoint 本身持续更新；按相邻位置 setpoint 求出的运动速率中位数约 0.77–0.91 m/s，
说明 MAVROS/PX4 确实持续收到移动位置目标，而不是 setpoint 流中断。

### 3. 移植版输出确实只有位置

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/ego_planner_driver_node.cpp:552-564`
  的 `publishPose()` 只构造并发布 `geometry_msgs::PoseStamped`。
- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/scripts/setpoint_relay_node.py:26-35`
  仅复制 `PoseStamped.header` 和 `PoseStamped.pose` 到
  `/mavros/setpoint_position/local`，没有速度或加速度字段可供保留。

### 4. 原版 EGO 会发布完整轨迹导数

原版轨迹服务器：

- 在 `/home/ub20tg/catkin_swarm6-2/references/ego-planner-swarm/src/planner/plan_manage/src/traj_server.cpp:176-180`
  同时采样位置、速度和加速度 B-spline；
- 在同文件 `:212-225` 把 position、velocity、acceleration、yaw、yaw_dot 全部写入
  `quadrotor_msgs/PositionCommand`；
- 在同文件 `:229-242` 以 100 Hz 发布 `/position_cmd`。

因此，当前移植版并没有保留原版轨迹服务器到控制器的完整命令语义。

### 5. 每次重规划从实际位置重新锚定

`/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/ego_planner_driver_node.cpp:158-166`
显示：

- 新规划起点位置取当前 `local_pose_`；
- 起始速度、加速度则从旧轨迹采样。

内部 B-spline 边界可能保持非零导数，但新段发布到 PX4 时仅剩位置。新段首个位置又接近 UAV 当前
实际位置，所以 PX4 位置误差周期性被清零。这解释了为何内部 `currentDerivatives()` 的连续性修复
不能单独消除飞控侧的周期降速。

## 版本边界

本次 ULog：

- `/home/ub20tg/catkin_swarm6-2/.ros_home/sitl_iris_0/log/2026-08-23/13_13_18.ulg`
- 文件时间约 2026-08-23 21:19。

当前源码和可执行文件重建时间约 22:05，晚于该次飞行。因此：

- ULog 直接验证的是 21:19 时运行的版本；
- 当前工作树新增的轨迹末端导数 clamp、`0.85 * duration` 提前重规划等改动尚未被该 ULog 验证；
- 这些改动没有改变 `PoseStamped -> /mavros/setpoint_position/local` 的纯位置链路，故上述主机制在
  当前工作树中仍然存在。

当前相对 HEAD 的节点改动主要包括：

1. `currentDerivatives()` 把采样时间 clamp 到轨迹末端，避免轨迹结束后返回默认零速度；
2. 重规划触发从固定 4 秒改为 `min(replan_period_s, 0.85 * duration)`。

它们可改善规划器内部新旧 B-spline 的边界连续性，但无法让 PX4 获得该连续速度。

## 根因排序

### P0：纯位置 setpoint 丢失速度/加速度前馈（高置信）

它与 4 秒重规划一起完整解释：

- 为什么速度波动和重规划同周期；
- 为什么内部延续速度仍无法保证实际速度连续；
- 为什么 PX4 ULog 的 `trajectory_setpoint.vx/vy/vz` 全是 NaN；
- 为什么没有 setpoint 断流也会周期减速。

### P1：轨迹切换位置起点使用实际 pose，而非旧轨迹同一切换时刻的位置（高置信协同因素）

这会把位置跟踪误差在每次切换时重置到接近零。若继续使用纯位置控制，它是周期降速的直接触发点；
若改为完整 position/velocity/acceleration setpoint，则该做法仍需检查位置、速度、加速度是否在同一
时间基准上构造，避免不一致边界。

### P2：旧版本可能在轨迹耗尽后才重规划并用零导数起步（中等置信，当前工作树已部分处理）

飞行发生在当前末端 clamp 和 85% 提前触发补丁之前。旧行为可能进一步放大每周期停顿，但不是
当前代码仍存在的主要链路缺陷。

### P3：relay 空档或 HOLD（低置信，现有证据不支持作为周期根因）

ULog 中位置 setpoint 持续存在；relay 只是无状态复制 PoseStamped。最终轨迹结束后的 HOLD/steady
target 能解释终点悬停，不能解释巡航阶段稳定的约 4.5 秒波动。

## 最小修复建议

### 推荐修复：改用 MAVROS raw local setpoint 传递 P/V/A

1. 由 `ego_planner_driver` 在每个 setpoint tick 同时采样当前轨迹的：
   - position；
   - velocity；
   - acceleration；
   - yaw（需要时增加 yaw rate）。
2. 发布 `mavros_msgs/PositionTarget` 到专用内部话题，或直接发布
   `/mavros/setpoint_raw/local`。
3. 配置正确的 coordinate frame 和 type mask，明确启用 position、velocity、acceleration、yaw，
   忽略未使用字段；需按 MAVROS ENU 到 PX4 NED 的插件语义做 SITL 验证，不要在业务层重复手工换轴。
4. 若保留 relay，应把 relay 输入/输出都改为 `mavros_msgs/PositionTarget`，不能继续经
   `PoseStamped` 降级。
5. HOLD/轨迹结束时显式发送固定位置和零速度、零加速度。

依赖方面，`ego_planner_driver` 的 CMake/package 已声明 `mavros_msgs`，无需引入新的 ROS 包。

### 同时修正切换时刻一致性

- 在同一个切换时间 `t_switch` 上从旧轨迹取得 position/velocity/acceleration；
- 若因跟踪偏差必须以实际 pose 作为新段位置边界，需设计短时间 blending，而不是组合
  “实际 position + 旧轨迹 velocity/acceleration”后立即硬切；
- 新轨迹发布时间、轨迹内部时间零点和首个样本必须统一，避免优化耗时补偿后跳过首段或重复首段。

### 不建议作为最终修复

- 单纯缩短/加长 `replan_period_s`：只会改变波动周期；
- 只调 PX4 位置环参数：会掩盖但不恢复规划速度语义；
- 只保留当前 `currentDerivatives()` clamp：内部导数不会出现在 PX4 setpoint 中；
- 在 PoseStamped 链路上人为把新轨迹起点向前推：可能减少降速，但会制造位置跳变并降低安全性。

## 验证标准

完成修复后应重跑同一 prepare 场景，并至少检查：

1. PX4 ULog `trajectory_setpoint.vx/vy/vz` 在轨迹执行期间为有限值；
2. 重规划切换前后 position、velocity、acceleration setpoint 无非预期阶跃；
3. 实际水平速度不再出现与 4 秒参数锁定的周期谷值；
4. 把 `replan_period_s` 临时改为 3 秒和 5 秒做 A/B 测试，实际速度谱中不应随之出现新的主峰；
5. setpoint 发布频率持续大于 OFFBOARD 最低要求，切换时无空档；
6. HOLD、最终完成、规划失败、pose stale 路径都明确输出零速度/零加速度；
7. 复核最大速度、终点误差和最小机间距，避免只修平滑性却引入限速或避碰回归。

## 链路补全

`/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/README.md:15-26` 记录的实际链路为：

```text
TCP JSON Lines
  -> tcp_to_ros / backend
  -> swarm_topology_bridge
  -> swarm_uav_executor
  -> ego_planner_driver（30 Hz setpoint）
  -> MAVROS / PX4 SITL
```

本次故障定位在最后两段之间的命令语义降级，而不是 TCP 到 executor 的任务分发层。