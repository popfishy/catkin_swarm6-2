# Cline/DeepSeek 提交代码与文档审查

时间：2026-08-28  
审查方式：只读静态审查；未修改被审查源码、launch、测试或正式文档  
项目边界：固定实验场景，最多 A01–A15 / UAV1–UAV15；较小编队从 1 开始连续编号；只支持当前 launch 配置，不考虑未知 identity、未来调用方或大规模扩展  
基准提交：`tcp_to_ros/b8e5d10ace321dbfc0950c94eee93b74455993cf`  
基准 committer 时间：`2026-08-28 01:56:55 +08:00`

## 1. 审查范围

检查 `src/` 下所有嵌套 Git 仓库中 committer 时间严格晚于基准的提交。

| 仓库 | 提交 | 时间 | 内容 |
|---|---|---|---|
| `ego_planner_driver` | `8532bc4df89e02594fcbc12a9d302ad8799ca99f` | 2026-08-28 14:35:36 +08 | OOM 修复、PlanResult、资源边界、测试 |
| `swarm_uav_executor` | `abd8d145533c215b3e4049fc60f83fa9b6053c9d` | 2026-08-28 14:35:57 +08 | limits 与 30 Hz/0.2 s launch 契约 |
| `tcp_to_ros` | `e753a265de19c2ea19a6e4a4f8b07fef3730b4` | 2026-08-28 01:57:07 +08 | 运行规约文档同步 |
| `tcp_to_ros` | `44ec37f6e811ef4a196e02ae069c38ef1bdf1472` | 2026-08-28 14:36:40 +08 | OOM 计划状态与测试记录 |
| `tcp_to_ros` | `711c7f4335d267c8705221eea22d90eea4703caf` | 2026-08-28 15:02:24 +08 | 15-UAV prepare 证据 |

`swarm_topology_bridge`、`swarm_uav_interfaces`、`safe_valley_exp` 没有晚于基准的提交。审查时六个 `src/` 嵌套仓库工作树均 clean。

## 2. 总体结论

固定 `[0,1]` 外推造成无界点集的核心 OOM 路径已被实质修复：多项式只在真实 `[0,T]` 采样，点数在分配前受限，B-spline 改为一次 QR，主要 `std::bad_alloc` 能收口为资源失败。该修复可信地消除了本次观测到的数 GiB 单 planner 爆炸路径。

按固定实验范围重新评估后，只保留两个会影响当前任务行为的代码问题，以及两个会影响当前测试/验收可信度的问题：

> OOM 根因修复已实施，现有 memory-specific observations 通过；当前任务状态机和动力学结果分类仍需修复，15-UAV 计划级验收尚未通过。

## 3. 影响当前实验的代码问题

### 3.1 [HIGH] 退化段处理可能使当前任务无法完成或无法按失败超时收口

位置：

- `src/ego_planner_driver/src/ego_planner_driver_node.cpp:340-442`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp:493-522`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp:1383-1429`
- `src/ego_planner_driver/test/test_ego_state_machine.cpp:921-960`

当前 `replanOnce()` 在建立 snapshot 前只检查 pose freshness，没有先用真实 odometry：

- 推进 0.5 m 内的中间 keypoint；
- 完成最终 0.2 m + stable window；
- 跳过重复/过近 target。

之后 core 若返回 `DEGENERATE_SEGMENT`，node 直接返回；而 `checkArrival()` 在没有 `current_` 时也直接退出。因此当前实验中，同点或近点目标可能持续停在 HOLD，而不是 `COMPLETED` 或进入普通可重试失败。

另一个相关问题：若此前已设置 `plan_fail_since_`，后续持续返回 `DEGENERATE_SEGMENT` 时，代码既不清除该计时，也不执行 10 秒失败收口，任务可能无法终止。

修复决策：

1. `planned_start` 继续取旧 `current_` 同一时刻的 P/V/A，保持轨迹连续；不改成真实位置。
2. 在 snapshot 前先用 fresh odometry 处理 target 推进和最终到达。
3. 无有效 `current_` 时，真实到达则完成；仍无法形成 segment 则进入普通可重试路径。
4. 为已有 `plan_fail_since_` 后持续退化定义可终止路径。
5. 替换当前“同点目标保持 HOLD”的错误测试，并增加已有失败计时后退化的回归。

### 3.2 [HIGH] 当前 launch 下动力学不可行轨迹仍可能作为 `SUCCESS` 发布

位置：

- `src/ego_planner_driver/src/ego_planner_core.cpp:821-842`
- `src/ego_planner_driver/launch/ego_planner_driver.launch:17-48`
- `src/swarm_uav_executor/launch/uav_executor_ego.launch:38-45`
- `src/ego_planner_driver/test/test_trajectory.cpp:58-69`

core 已执行：

```cpp
traj.checkFeasibility(fea_ratio)
```

但当结果为 false 时，只有启用 time reparam 或 rebound 才处理。当前实验 launch 中二者都关闭，因此超出 `max_vel/max_acc` 的轨迹仍可能继续生成输出并返回 `SUCCESS`。

这会直接影响当前实验：node 可能提交动力学不合法的轨迹、替换合法 `current_` 并发布 intent。计划要求该情况属于 `RETRYABLE_FAILURE`。

修复建议：

1. 当前开关组合下，`checkFeasibility()==false` 必须返回 `RETRYABLE_FAILURE`。
2. 失败不得发布 intent，也不得替换现有合法 `current_`。
3. 使用当前生产参数增加 core/node 回归；不要使用放宽到 `1.5 × max_vel` 的测试判据掩盖结果。

## 4. 当前测试与验收记录问题

### 4.1 [HIGH] 15-UAV 只能认定内存专项观察通过，不能认定计划级验收通过

位置：

- `src/tcp_to_ros/test_records/test_record_26082722.md:61-93`
- `src/tcp_to_ros/plans/README.md:203-217`
- `src/tcp_to_ros/plans/implementation_plan_26082722.md:722-750`

现有运行证明：

- 15 个 planner 存活；
- 未出现 `bad_alloc/-6/-9/OOM kill`；
- RSS 没有回到数 GiB；
- OFFBOARD 未丢失；
- relay 输出观察正常。

但 prepare 实际返回：

```text
FAILURE(A02/HOLD)
```

计划第 15 节明确把任一 task failure 定义为整轮 FAIL。因此正确状态应统一为：

> 实现已提交；本轮 memory-specific observations PASS；计划级 15-UAV acceptance FAIL，需修复当前任务缺陷后重跑。

A02 的具体根因是否属于原 OOM 修复范围，不改变本轮按冻结计划判 FAIL 的事实。

### 4.2 [MEDIUM] 资源门禁和运行记录不能完整复现声称的精确证据

位置：

- `src/ego_planner_driver/test/test_short_final_segment.cpp:217-235`
- `src/ego_planner_driver/test/test_short_final_segment.cpp:299-398`
- `src/tcp_to_ros/test_records/test_record_26082722.md:3,49-59,75-92`

当前问题合并如下：

1. 紧 cap 注入接受 `RESOURCE_LIMIT` 或 `RETRYABLE_FAILURE`，没有证明资源失败一定采用立即收口分类。
2. 注入结果不进入 `rep.resource`，之后的 `rep.resource == 0` 只覆盖普通矩阵。
3. 没有 node 层测试证明 `RESOURCE_LIMIT` 不 activation retry、不 yield、不发布部分 intent。
4. worker 没有检查 `RLIMIT_AS/RLIMIT_CPU` 的 `setrlimit()` 返回值；限制安装失败时仍可能报告测试成功。
5. 测试记录的 2000-loop filter 缺少 suite 前缀：

```text
StressLoopMemoryPlateau:FunctionalMatrixBoundedAndClassified
```

正确形式应为：

```text
ShortFinalSegment.StressLoopMemoryPlateau:ShortFinalSegment.FunctionalMatrixBoundedAndClassified
```

6. 记录时间 `02:xx-03:05 CST` 早于 14:35 实现提交和 15:02 证据提交，且 recorder 为空，无法确认记录对应哪个二进制和提交。
7. 记录未提供可复核的 planner candidate 频率、TimedTrajectory 间隔、cgroup/watchdog 限制、完整 memory current/peak/events、per-planner PSS 和 point/control/sample peaks。

已接受的精简修复决策：

1. 使用方式 B：资源注入拆成独立测试，只接受 `PlanResult::RESOURCE_LIMIT`，并检查失败输出为空。
2. 普通分类矩阵不包含资源注入，继续断言自身 `resource == 0`。
3. 增加 node 层“零 activation retry、零 yield、无部分 intent、立即失败/HOLD”测试。
4. 检查两个 `setrlimit()` 返回值，失败时使用明确 exit code。
5. 使用完整 suite filter、commit、准确起止时间、operator 和原始资源/频率数据重跑并更新记录。

## 5. 确认正确的部分

以下实现对当前固定实验有效：

1. `samplePolynomialByArcLength()` 只在真实 `[0,duration]` 求值，包含精确端点。
2. dense sample count、累计弧长和 parameterization point count 在分配前检查 finite、overflow 和配置 cap。
3. 当前生产配置固定 `max_parameterization_points=256`，node 启动时验证上限；不需要为未知未来调用方继续增加审查项。
4. `DEGENERATE_SEGMENT` core 检查发生在方向归一化、时长估算和 quintic 构造前。
5. B-spline x/y/z 合并为一次 QR 分解和一次 solve，检查 rank/residual/finite，成功后才写输出。
6. 最终 TimedTrajectory 点数受控；当前固定 `ts=0.4` 时 nominal step 为 0.1 s。
7. core 与 replan thread 有窄范围 `std::bad_alloc` containment；主要 `RESOURCE_LIMIT` 路径进入 `EGO_PLAN_FAILED`。
8. 单条 neighbor intent 在 reserve 前校验长度、样本数、时间和 finite；固定 UAV1–UAV15 场景不需要额外 identity cardinality 设计。
9. 四个资源 limit 与方向阈值已接入当前 planner/executor launch 和 C++ 参数读取。
10. planner candidate 30 Hz、relay 30 Hz、candidate timeout 0.2 s、Safety active 和 1 m 三维球形门禁没有被改坏。
11. bridge、消息协议、GPS bias 与 PX4 配置均未被这批提交修改。

## 6. 精简后的处理顺序

1. 修复 fresh-odometry/no-current/失败计时的退化段状态机，并更新对应测试。
2. 修复当前开关组合下动力学不可行仍返回 `SUCCESS` 的问题，并增加 core/node 测试。
3. 按方式 B 修正资源分类测试，检查 `setrlimit()` 返回值。
4. 修正压力测试命令和测试记录时间/证据。
5. 更新计划状态为 memory-specific PASS、计划级 acceptance FAIL。
6. 重新运行自动测试和 15-UAV prepare。

本报告只记录问题；未对上述正式文件执行修改。
## 7. 处理状态（2026-08-28 21:35 更新，本轮 prepare 无头测试后）

按第 6 节顺序逐项落实：

1. ✅ 退化段状态机修复：`ego_planner_driver c26cdb6`（snapshot 前 fresh-odom 推进/最终到达、
   无 current_ 真实到达 COMPLETED、DEGENERATE 已失败计时 10s 软着陆收口、退化等待禁 HOLD 捕获锁存）；
   对应状态机测试替换/新增。
2. ✅ 动力学不可行拒发：生产开关全关下采样后欧氏峰值 >2.5×max_vel/max_acc → `RETRYABLE_FAILURE`
   （不发布 intent、不替换 current_）；core/node 回归用生产参数 max_vel=1.0/max_acc=6.0。
3. ✅ 资源分类方式 B：紧 cap 注入独立测试只接受 `RESOURCE_LIMIT` 且失败输出为空；普通矩阵
   resource==0；node 层零 activation retry/零 yield/无部分 intent/立即 HOLD 测试；两个
   `setrlimit()` 返回检查（失败 exit 110/111）。
4. ✅ 压力测试命令修正：完整 suite filter（`ShortFinalSegment.StressLoopMemoryPlateau:`
   `ShortFinalSegment.FunctionalMatrixBoundedAndClassified`）、起止时间、operator、原始数据重跑，
   更新 `test_record_26082722.md` §2。
5. ✅ 计划状态更新：`plans/README.md` 与 `test_record_26082722.md` 统一为
   memory-specific observations PASS / 计划级 15-UAV acceptance FAIL。
6. ⏳ 自动测试与 15-UAV prepare：自动测试重跑通过（受限短末段 7 tests、ego_planner_driver 172 tests、
   全工作区 303 tests 0 errors 0 failures、500/2000 次压力过 RSS 门禁）；15-UAV active prepare
   **第 3 轮干净重测（2026-08-28 21:20-21:31）返回 `FAILURE(A03/HOLD)`**，11/12 COMPLETED，A03 终点
   0.29m 偏差未满足 0.2m 到达判据 → checkArrival 看门狗转 HOLD → executor COMMAND_HELD。本轮 memory
   observations 全 PASS（planner RSS 11.76-12.41 MB、OFFBOARD 15/15、relay 30.000 Hz/max 间隔
   0.035s、无 OOM）。计划级 acceptance 仍未通过，需修复 A03 类终点到达缺陷后重跑整轮。
