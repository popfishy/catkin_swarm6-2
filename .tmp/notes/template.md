
本轮没有删除 `/home/ub20tg/catkin_swarm6-2/.tmp` 中的测试证据，也没有修改生产代码。

---

# 安全措施审查结论

## 总体判断

当前问题**不是重新引入了双向通信看门狗**。事实上，配置中的 stale/missing policy 目前大部分还没有接入生产决策代码。

真正影响正常任务的主要是以下三类问题：

1. **让路轨迹只验证 1m，却由更大的动态 emergency 门槛持续监控，前后验收标准互相冲突。**
2. **让路候选一旦提交，WARNING 阶段无条件拒绝重新评估；只有恶化到 EMERGENCY 后才制动。**
3. **EMERGENCY → BRAKE_HOLD → 任务失败本来就是需求定义的不可恢复路径，因此上游一次误判会直接终止任务。**

因此，当前表现更接近“多个各自合理的安全规则组合后形成自我干扰”，和此前双向看门狗影响正常仿真的性质相似，但触发源不是通信超时。

---

## 1. 高优先级：候选验收门槛和运行时门槛互相矛盾

让路候选只检查固定 1m clearance：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/yield_candidates.cpp:126-131`

```cpp
if (distance < config_.clearance_m)
{
  reason = "clearance";
  return false;
}
```

但候选提交后，safety supervisor 使用动态门槛：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/safety_predictor.cpp:137-147`

```text
emergency_distance =
    1.0
  + closing_speed × reaction_time
  + braking_distance
  + 0.10
```

这意味着一条预测最小间距只要略高于 `1.0m`，就可能：

1. 被候选生成器判为可行并提交；
2. 下一次 safety tick 又因为低于约 `1.4–2.0m` 的动态 emergency 门槛被判为紧急；
3. 两机同时制动；
4. 最终进入 `BRAKE_HOLD`，任务失败。

这不是单纯“参数太保守”，而是**两个安全层使用不一致的契约**。

实施计划要求候选至少满足 1m，但也要求 FAST 不能突破动态 emergency 边界。因此，正确做法不是简单降低 emergency 门槛，而是让候选提交前使用与运行时 supervisor 一致的动态门槛，至少保证提交后不会立即被自身否决。

### 建议

让候选验证输出两层结果：

- 硬约束：全程 `distance >= 1.0m`
- 可提交约束：全程不进入动态 emergency 区域
- WARNING 区域可以接受，但必须确认它在后续时间上是改善而不是恶化

尤其应补充断言：

> 一条被提交的 YIELD 候选，在相同 peer intent 和相同 activation epoch 下，下一 safety tick 不得直接被判为 EMERGENCY。

---

## 2. 高优先级：`yield_committed_` 使 WARNING 阶段停止重新评估

当前代码：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/ego_planner_driver_node.cpp:1089-1091`

```cpp
if (prediction.level == SafetyLevel::WARNING &&
    role == PairRole::YIELD &&
    yield_committed_)
  return;
```

让路轨迹一旦提交，只要当前还是 WARNING，就直接返回，不再检查：

- 已提交候选是否仍可行；
- 邻机是否发布了新轨迹；
- 激活时间是否变化；
- 当前预测最小距离是否持续下降；
- 原锁存方向是否已经不可行；
- 是否应该保持方向但更新轨迹。

这超过了“防止左右振荡”的必要程度。

实施计划要求的是：

> LEFT/RIGHT 方向锁存；只有已选方向变得不可行时，才按固定顺序切换。

当前实现实际上变成：

> 候选一旦提交，整个 WARNING 阶段不再重新评估，直到升级成 EMERGENCY。

这很可能正是“让路已经发生，但随后仍被预测为 0.684m 并制动”的关键链路之一。

### 建议

将“锁存方向”和“冻结整条轨迹”分开：

- 锁存 `LEFT/RIGHT` 方向；
- 每个新 peer intent 或固定低频周期重新验证当前已提交候选；
- 若原方向仍可行，可重新生成同方向候选并更新 activation；
- 原方向不可行时，才尝试其他候选；
- 只有全部候选均失败或已经进入真正 emergency braking window，才制动。

---

## 3. 高优先级：日志中的 0.684m 不是由数据年龄放大直接造成的

实际触发记录：

```text
state=EXECUTING
level=EMERGENCY
peer=UAV2
age=0.017444
window=1.886440
distance=2.684013
closing=0.719929
min_d=0.683709
warning_d=1.672116
emergency_d=1.422116
role=STAND_ON
```

关键点：

- intent 年龄只有约 `17ms`；
- 所以这次触发不是 `peer_data_age_cap_s=2.0` 把门槛放大造成的；
- predictor 根据双方发布的轨迹，在约 `1.89s` 后确实预测到 `0.684m`；
- 按当前冻结语义，任何 UAV 进入 EMERGENCY 都必须制动，因此 UAV1 制动本身符合需求。

但同一场景真实最小距离约为：

- CPA 1.94：`1.917m`
- CPA 2.10：`1.960m`

这说明主要问题在：

- 收到的 intent 与飞机真实执行轨迹不一致；
- 或双方 intent 的 activation 时间轴不一致；
- 或让路候选发布后被后续规划/状态覆盖；
- 或 peer intent 的轨迹尾部、重定时和激活时间表达不准确。

因此，不建议先通过降低 `warning_buffer_m` 或 `emergency_buffer_m` 掩盖问题。应先把触发时刻以下三条曲线对齐：

1. 本机实际 odometry；
2. 本机 supervisor 使用的 `current_`；
3. 收到的邻机 active/braking intent。

---

## 4. `BRAKE_HOLD` 不属于超范围安全措施，但会放大误判后果

以下行为符合实施计划：

- EMERGENCY 时生成连续制动轨迹；
- 制动结束后锁存终点；
- 不自动恢复原任务；
- executor 将 `BRAKE_HOLD` 映射为任务失败。

相关实现：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/ego_planner_driver_node.cpp:1162-1177`
- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:39-41`
- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:410-415`

所以不能简单把 `BRAKE_HOLD` 改成自动恢复，否则会违反当前安全语义。

更合理的处理顺序是：

1. 减少错误升级到 EMERGENCY；
2. 保证已提交让路候选不会被自身动态门槛立即否决；
3. 保留真正 emergency 后不可恢复的失败收口。

---

## 5. stale/missing policy 当前不是误刹车来源，但存在“假配置”

launch 中配置了：

- `neighbor_stale_policy=diagnose_only`
- `neighbor_intent_stale_s=2.0`
- `neighbor_missing_policy=continue_after_barrier`

例如：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/launch/ego_planner_driver.launch:70-73`
- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/launch/uav_executor_ego.launch:65-68`

但当前生产 C++ 中没有发现对应的：

- 成员变量；
- 参数读取；
- stale policy 分支；
- missing policy 分支。

只有 `peer_data_age_cap_s` 真正进入了动态门槛计算。

### 结论

好的一面：

- 当前没有 `0.2s` 或 `2.0s` 超时后直接双方制动的通信看门狗；
- 因此不存在此前双向看门狗那种直接副作用。

问题是：

- launch 和测试看到参数存在，会误以为策略已经生效；
- 实际上 `diagnose_only/disabled/brake_if_closing/brake_always` 无法真正切换；
- 这属于未完成需求和可观测性缺陷。

建议后续要么实现完整策略，要么在实现前删除/标记这些假配置，避免误判系统行为。

---

## 6. 旧 executor 1m 门禁是第二套独立失败入口

executor 仍会在 EGO 巡航期间检查邻机实时 odometry：

- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:395-404`

触发后：

```text
发布 HOLD
任务失败：MIN_DISTANCE_BREACH
不自动恢复
```

这确实是预测 supervisor 之外的第二套安全入口。

但实施计划明确要求 Stage E 前继续保留旧 1m 门禁，所以当前不能直接删除。并且本次双机失败码是 `BRAKE_HOLD`，不是 `MIN_DISTANCE_BREACH`，说明它不是本轮失败的直接触发源。

不过仍需做两项改进：

- 增加显式 A/B 参数，方便证明预测门禁已经能提前处理风险；
- 给邻机 odometry 门禁增加 freshness 语义，避免历史旧位姿永久保存在 `_neighbor_poses` 后误触发。

在预测闭环稳定前，旧门禁应保留，但必须统计每轮是否触发；若触发，不能把该轮仅视为“安全通过”。

---

## 7. `hold()` 的 200 秒阻塞属于明确实现缺陷

当前：

- `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:543-553`

```python
end = self._monotonic_clock() + self.state_timeout_s
```

它忽略调用方传入的 `deadline`，默认可能等待约 `200s`。

同时 `_issue_hold()` 会先清空 `_last_cmd_reply`，即使节点此前已经处于 HOLD，也可能失去立即确认机会。

这与避碰数学无关，但属于典型的“安全收口机制本身阻塞任务入口”。应优先修复为：

- 已知当前状态为 HOLD 时立即成功；
- 等待上限使用 `min(deadline, now + state_timeout_s)`；
- 对 `BRAKE_HOLD` 等状态定义明确响应；
- 增加 driver 和 executor 回归测试。

---

## 8. 速度门限没有留真实系统跟踪余量

配置允许候选最大速度恰好为 `1.5m/s`：

- `yield_max_velocity_mps=1.5`
- `emergency_max_velocity_mps=1.5`

但验收要求真实速度也必须 `<=1.5m/s`。

CPA 2.10 场景中 UAV1 实测峰值为 `1.580m/s`。这说明把规划上限和真实验收上限设成完全相同，没有给 PX4 跟踪超调、采样误差或重规划切换留余量。

建议：

- 保持验收门限 `1.5m/s` 不变；
- 将候选生成速度上限降到约 `1.35–1.40m/s` 后标定；
- 或暂时禁用 FAST 候选，直到有足够的闭环速度余量证据；
- 不应通过放宽验收门限来解决。

---

# 安全措施分层结论

## 必须保留

- 真实中心距 `>=1m` 的最终约束；
- emergency 连续制动轨迹；
- emergency 后固定 `BRAKE_HOLD`，不自动恢复任务；
- 低 ID 只在 WARNING 时保持 nominal，EMERGENCY 时仍必须制动；
- bounded tentative 协商窗口；
- `diagnose_only` stale 默认语义；
- Stage E 通过前的旧 executor 1m 门禁；
- `enable_rebound=false`；
- relay 唯一 MAVROS setpoint 出口。

## 偏保守但暂不能简单删除

- `warning_buffer_m=0.35`
- `emergency_buffer_m=0.10`
- `peer_data_age_cap_s=2.0`
- `warning_ttc_s=3.0`
- `emergency_ttc_s=1.5`
- 5 秒预测窗
- emergency 后任务失败

这些参数需要 Stage E 标定，但当前失败证据不足以证明仅靠调小它们即可解决。

## 明确有副作用或实现不完整

1. `yield_committed_` 导致 WARNING 阶段不再重新验证候选。
2. 候选固定 1m 验证与动态 emergency 门槛不一致。
3. `hold()` 忽略调用方 deadline，可能阻塞约 200 秒。
4. stale/missing 参数存在于 launch，但没有完整生产实现。
5. 候选规划速度上限与真实验收上限同为 1.5m/s，没有跟踪余量。
6. 旧 odometry 门禁缺少明确 freshness/A-B 开关。
7. 当前日志记录了预测结果，但没有完整记录四候选及各自拒绝原因，不满足计划中的完整决策链要求。

---

# 建议修复顺序

1. 修复 `EgoSwarmDriver.hold()` 的 deadline 和已处于 HOLD 的立即确认。
2. 去掉 `yield_committed_` 的无条件 WARNING 早退，改为“锁方向但持续验证”。
3. 统一候选提交验证与运行时动态门槛，补“提交后下一 tick 不得直接 EMERGENCY”回归测试。
4. 对齐实际 odometry、`current_`、邻机 intent 的绝对时间轴，重点复现日志中的 `0.683709m`。
5. 实现或明确移除 stale/missing 假配置。
6. 将候选速度上限下调，为真实 `1.5m/s` 验收留余量。
7. 先跑自动闭环动力学，再在原 15 机 launch 中重跑双机正交与换位；双机完成前不启动 UAV3。

当前未提交改动保持不变：

- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/CMakeLists.txt`
- `/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/test/test_closed_loop_dynamics.cpp`
- `/home/ub20tg/catkin_swarm6-2/src/safe_valley_exp/startup_offboard_ego.sh`