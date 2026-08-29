# ROS1-PX4 无人机编队执行与机间避碰方案调研

> 调研日期：2026-08-27  
> 目标场景：无环境障碍、水平二维方队、无 Z 轴堆叠、低速（小于 2 m/s）、分布式多无人机实机飞行  
> 已有基础：机载电脑时间戳修正；RTK 定位下各机 PX4 local 坐标系统一；地面站任务已解析为各无人机独立航点序列，但航点序列未包含机间避碰  
> 待开发模块：执行各机航点序列，并在形成/变换编队时解决轨迹的时空冲突。本文将该模块称为“编队执行驱动”。

## 1. 执行摘要

### 1.1 核心结论

1. **仓库中现有的 4 篇 Markdown 中英对照论文，没有一篇能直接解决本任务。**
   - *Fully neuromorphic vision and control for autonomous drone flight* 是单机事件视觉与低层控制；
   - *Self-Supervised Learning of Event-Based Optical Flow with Spiking Neural Networks* 是事件光流估计；
   - *Learning high-speed flight in the wild* 是单机、高速、静态障碍视觉避障；
   - *Neural Moving Horizon Estimation for Robust Flight Control*（NeuroMHE）是单机抗扰估计与控制增强。
   - NeuroMHE 可在后续用于抗风或下洗扰动补偿，但不是机间避碰器。

2. **对当前明确限定的无障碍、低速、二维场景，首选架构是“名义航点跟踪器 + 分布式二维 CBF-QP 安全过滤器”。** 每架机独立运行相同程序，接收邻机状态，将本机名义水平速度修正为尽可能接近原命令、同时满足安全约束的速度，再通过 MAVROS 发送给 PX4。该架构与现有航点序列边界最吻合，不需要地图、视觉感知、全编队集中式求解或重写任务分配器。

3. **若项目要求对有界通信延迟给出轨迹级安全论证，第二阶段优先评估 RMADER。** RMADER 是 ROS Noetic 下开源的异步、去中心化多机轨迹规划器，明确设计了 Delay Check 和两阶段轨迹发布，并在论文中给出有界延迟下的递归可行性/无碰撞论证及硬件实验。代价是依赖较重（Gurobi、CGAL、Snapstack 消息等），与 PX4/MAVROS 之间需要适配，且原实现是三维多项式轨迹规划器，不是轻量速度过滤器。

4. **EGO-Swarm 是最容易获得 ROS1 多机规划工程参考的完整代码库，但不应作为首版安全内核。** 它异步、去中心化、规划快、ROS1/catkin 完整，使用 B-spline 广播并有实机演示；但是机间避碰以非线性优化惩罚项实现，不是硬安全约束，对通信延迟也没有 RMADER 级别的保证。在本项目无静态障碍条件下，它的地图、深度输入和三维局部规划链路还会造成不必要复杂度。

5. **不要把 ORCA/RVO2 直接等同于实机安全保证。** ORCA 与本任务的“给定 preferred velocity 后最小修改”接口非常匹配，RVO2 又是成熟的二维 C++ Apache-2.0 库，适合快速做仿真基线；但标准 ORCA 假设全向速度瞬时可实现、双方对避碰各承担一半责任，未直接包含 PX4 加速度/姿态滞后、RTK 误差、丢包和陈旧状态，拥挤对穿时也可能出现振荡或死锁。

### 1.2 推荐优先级

| 优先级 | 方案/论文 | 建议定位 | 与当前任务的匹配度 |
|---|---|---|---|
| 1 | Wang, Ames, Egerstedt 的 Safety Barrier Certificates；Wang 等的 quadrotor differential-flatness 扩展 | 首版“编队执行驱动”的安全过滤内核 | **最高**：直接包裹已有航点跟踪命令，二维、低速、无地图 |
| 2 | RMADER: Robust MADER | 对通信延迟有明确上界和较强安全要求时的轨迹级升级方案 | **高**：分布式、异步、有硬件实验和开源 ROS1 代码，但移植较重 |
| 3 | EGO-Swarm | ROS1 多机 B-spline、轨迹广播、FSM 和仿真框架参考；快速原型备选 | **中高**：工程成熟，但避碰为软惩罚，且静态障碍模块冗余 |
| 4 | Swarm-Formation（仓库已有 PDF） | 需要同时在线优化编队形状、重分配和障碍穿越时参考 | **中**：直接研究编队，但明显超出“已有各机航点，只需执行避碰”的边界 |
| 5 | ORCA / RVO2 | 二维快速基线、压力测试和与 CBF-QP 对照 | **中**：极易集成，但实机动力学与通信鲁棒性不足 |
| 6 | Hönig 等 *Trajectory Planning for Quadrotor Swarms* | 地面站离线/集中式时空去冲突基线 | **较低**：代码只支持二维 (E)CBS，连续平滑依赖 MATLAB，不是在线分布式执行器 |

## 2. 问题重述与筛选标准

### 2.1 本任务真正需要解决的问题

对第 $i$ 架无人机，输入是有序航点：

$$
W_i = \{p_{i,0}, p_{i,1}, \ldots, p_{i,m_i}\},\quad p_{i,k}\in\mathbb{R}^2
$$

各机在同一水平高度飞行，航点序列单独看是可执行的，但多个序列在同一时刻附近可能经过同一空间区域。编队执行驱动应完成：

1. 根据当前位置和当前航点生成名义速度/加速度；
2. 使用邻机状态或邻机短时预测判断未来冲突；
3. 仅在必要时修改本机命令；
4. 保持水平速度小于 2 m/s，并满足 PX4 可跟踪的加速度、加加速度约束；
5. 冲突解除后继续原航点序列，最终到达所有航点；
6. 通信异常、邻机状态过期、规划不可行时进入明确的减速/悬停/降落策略。

这与“在未知障碍地图中从起点重新规划到终点”不同，也与“由控制律实时生成编队形状”不同。已有航点任务应被保留为**名义任务层**，避碰应作为执行期的**安全层**。

### 2.2 论文筛选标准

按以下标准评价：

- **分布式性**：每架机能否仅依赖自身和邻机信息独立求解；
- **输入契合度**：能否接收航点、preferred velocity 或名义轨迹，而不是要求重新定义整个任务；
- **二维适配**：能否固定高度、只优化 X/Y；
- **机间避碰保证**：硬约束/形式化保证，还是仅靠代价函数；
- **通信现实性**：是否考虑异步、时延、丢包和陈旧消息；
- **动力学可执行性**：是否约束速度、加速度、jerk，或仅把无人机当作单积分质点；
- **ROS1-PX4 移植成本**：是否已有 catkin、ROS topic、C++ 实现和实机接口；
- **开源与复现**：代码、许可证、依赖、仿真/硬件证据是否公开。

## 3. 当前仓库论文审查

### 3.1 有 Markdown 中英对照翻译的论文

| 仓库文件 | 研究对象 | 对本任务的判断 |
|---|---|---|
| `in-read/Fully_neuromorphic_vision_and_control_for_autonomous_drone_flight_SCIENCE_ROBOTICS/Fully_neuromorphic_vision_and_control_for_autonomous_drone_flight.md` | 单机事件相机 → 光流 → 神经形态低层控制 | **不适合**。无多机协同、无机间避碰；依赖 Loihi 与事件相机。仓库笔记还指出低速/近悬停时事件流信噪比低、轨迹跟踪较差，和本项目低速工况相冲突。 |
| `in-read/.../references/24.self-supervised_contrast_maximization.md` | 事件相机自监督光流估计 | **不适合**。是感知算法，不输出多机安全运动命令。 |
| `pre-read/Learning_high-speed_flight_in_the_wild_SCIENCE_ROBOTICS/Learning_high-speed_flight_in_the_wild.md` | 单机在复杂静态环境中的高速视觉导航 | **不适合**。研究重点是感知障碍、端到端轨迹预测和高速飞行；本项目无环境障碍、低速且冲突对象是协作无人机。 |
| `pre-read/Neural_Moving_Horizon_Estimation_for_Robust_Flight_Control_IEEE_TRO/Neural_Moving_Horizon_Estimation_for_Robust_Flight_Control.md` | 单机干扰估计与鲁棒控制 | **不能承担避碰，但可作为后续增强**。它估计外力/未建模扰动，不负责生成机间无碰轨迹。其参考文献 [1] 是与本调研相关的 Hönig 等 *Trajectory Planning for Quadrotor Swarms*。 |

因此，若严格把候选限制为“仓库中已经有 Markdown 对照翻译的论文”，答案是：**没有直接适用论文**。继续调研必须利用仓库中只有 PDF 的论文及外部论文/代码。

### 3.2 仓库中只有 PDF、但与任务直接相关的论文

#### 3.2.1 Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments

- 作者：Lun Quan, Longji Yin, Chao Xu, Fei Gao；ICRA 2022。
- 仓库文件：`pre-read/Robust and Efficient Formation Trajectory Planning_IEEE_TRO/Distributed_Swarm_Trajectory_Optimization_for_Formation_Flight_in_Dense_Environments.pdf`。
- 方法：将编队相似度、机间碰撞、障碍和动力学约束纳入多项式轨迹优化；各机分布式、解耦优化并共享轨迹。
- 证据：论文报告真实分布式无人机集群实验；仓库中的后续 T-RO 论文记录了 0.5 m/s 限速实验，落在本项目速度范围内。
- 开源：[ZJU-FAST-Lab/Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation)，ROS/catkin，GPL-3.0。
- 适用性：算法确实覆盖编队保持和机间避碰，但其主要优势是密集障碍环境下编队整体轨迹优化。对本项目而言，地图、障碍和编队形状优化大多是额外负担。
- 结论：**适合作为完整编队规划参考，不是首版执行驱动的最小方案。**

#### 3.2.2 Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments

- 仓库文件：同目录下 `Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments.pdf`；IEEE T-RO 2023。
- 方法扩展：强调 PAPER 条件（高精度、适应性、规划效率、弹性、鲁棒性），加入编队级路径搜索、时空优化、任务重分配和编队对齐。
- 与当前问题的联系：论文明确展示“不恰当初始任务分配会造成全局轨迹交叉和轨迹优化冲突”，这和本项目形成方队时的交叉现象高度一致。
- 不匹配点：本项目已经完成各机航点分解；若不允许执行器改变槽位分配，此论文最有价值的 ALAS/重分配部分无法直接使用。完整移植会让执行驱动重新承担任务分配与编队规划职责。
- 结论：**如果将来允许形成编队前重新分配方队槽位，应吸收其“先低频分配、再高频分布式轨迹优化”的思想；当前阶段不建议完整移植。**

#### 3.2.3 Concurrent-Allocation Task Execution for Multirobot Path-Crossing-Minimal Navigation

- 仓库文件：`pre-read/Path-Crossing-Minimal_Navigation_in_Obstacle_IEEE_TRO/Concurrent-Allocation_Task_Execution_for_Multirobot_Path-Crossing-Minimal_Navigation_in_Obstacle_Environments.pdf`；T-RO 2025。
- 方法：把目标分配、目标收敛、碰撞/障碍规避编码为整数和 CBF 约束，在统一在线优化中减少路径交叉。
- 优点：直接针对“目标分配导致路径交叉”的根因；CBF 约束具有安全过滤价值。
- 局限：论文实验重点为二维地面 AMR，且算法会在线改变目标分配；与“各机航点序列已固定”的接口不完全一致。
- 结论：**适合借鉴 CBF 与任务分配联合设计，不适合直接作为 PX4 执行器移植。**

#### 3.2.4 其他仓库 PDF

- *Obstacle Avoidance of Resilient UAV Swarm Formation with Active Sensing System in the Dense Environment*：依赖点云/GMM 共享和主动感知，解决密集障碍，明显超出本场景。
- *Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments*：重点是狭窄环境、编队规模变化和可变形引导，不适合固定方队的最小执行层。
- *Relational Maneuvering of Leader-Follower UAVs for Flexible Formation*：更偏 leader-follower 编队几何关系控制；不能替代明确的时空避碰安全层。
- *An Overview of Swarm Coordinated Control*：可用于综述分类，不是可直接移植算法。

## 4. 外部重点候选论文与开源实现

### 4.1 Safety Barrier Certificates：首选的轻量安全层

#### 关键论文

1. L. Wang, A. D. Ames, M. Egerstedt, “Safety Barrier Certificates for Collisions-Free Multirobot Systems,” IEEE T-RO, 2017。
2. L. Wang, A. D. Ames, M. Egerstedt, “Safe Certificate-Based Maneuvers for Teams of Quadrotors Using Differential Flatness,” ICRA 2017，DOI: [10.1109/ICRA.2017.7989375](https://doi.org/10.1109/ICRA.2017.7989375)。

#### 核心思想

先由航点跟踪器给出名义命令 $u_i^{nom}$，再求一个尽量不改变名义命令的安全命令：

$$
\min_{u_i,\,\delta_i}\quad \frac{1}{2}\|u_i-u_i^{nom}\|^2 + \rho\|\delta_i\|^2
$$

对每个邻机 $j$，定义水平安全函数：

$$
h_{ij}=\|p_i-p_j\|^2-d_{safe}^2
$$

在最简单的单积分速度模型 $\dot p_i=u_i$ 下，可构造线性 CBF 条件，使 QP 的约束对 $u_i$ 为线性。每架机仅对自身二维命令求解一个小 QP，复杂度低；没有冲突时，解几乎就是 $u_i^{nom}$。

#### 为什么最贴合当前系统

- 输入就是现有航点执行器生成的名义速度；
- 输出可直接映射到 MAVROS 水平速度 setpoint；
- 无需静态地图或深度相机；
- 仅使用统一坐标系中的邻机位置、速度和时间戳；
- 容易固定 Z，只优化 X/Y；
- QP 不可行、通信超时和紧急制动都可做成显式状态机；
- 算法边界清晰，不侵入已有时间同步、坐标统一和任务解析模块。

#### 必须承认的限制

- 基础 T-RO 版本多以单积分/地面机器人为模型；不能直接把理论保证照搬到 PX4 实机。
- 四旋翼扩展利用 differential flatness，但移植到 PX4 速度接口时仍需验证闭环滞后和饱和。
- 分布式双方若各自独立施加完整避碰责任，需统一约束分摊规则，否则会过度保守；若各承担一半，又必须保证双方都在线且运行一致算法。
- 位置误差、通信年龄、控制跟踪误差必须进入 $d_{safe}$ 或鲁棒 CBF，而不能只使用机体几何半径。
- 单纯局部 CBF 可能安全但不保证无死锁。方队对穿尤其应配合优先级/通行权。

#### 开源复现资源

- [robotarium/robotarium-matlab-simulator](https://github.com/robotarium/robotarium-matlab-simulator)：MIT；包含基于 `quadprog` 的 barrier function 示例，适合核对数学和做二维基线，但依赖 MATLAB Optimization Toolbox，平台是 Robotarium 地面机器人。
- [robotarium/quadtarium-python-simulator](https://github.com/robotarium/quadtarium-python-simulator)：引用 quadrotor safety-certificate 论文，可用于四旋翼仿真概念验证。
- 论文作者资料显示相关四旋翼系统曾以 ROS/C++/Python 实现，但没有发现一个可直接接到 PX4/MAVROS、包含完整实机安全链路的官方 ROS1 包。因此，**CBF-QP 需要自行工程实现，不能宣称“克隆即用”。**

### 4.2 RMADER：通信延迟安全性最强的开源候选

#### 论文与代码

- K. Kondo et al., “Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments,” RA-L/ICRA 2023；[arXiv:2303.06222](https://arxiv.org/abs/2303.06222)。
- 代码：[mit-acl/rmader](https://github.com/mit-acl/rmader)，BSD-3-Clause。

#### 关键能力

- 去中心化、异步规划；
- 通过 Delay Check、两阶段轨迹共享和轨迹存储/复核处理通信延迟；
- 论文给出最大通信时延上界内的递归可行性和无碰撞论证；
- 论文报告 100% collision avoidance，并进行了多机、多动态障碍和不同网络条件下的硬件实验；硬件实验最大速度约 2.5–3.0 m/s，因此覆盖本项目小于 2 m/s 的动力范围；
- ROS topic 包括每机命名空间下的 `state`、`term_goal`、`traj`、`goal`、`setpoint` 等；适配器可把 PX4 odometry 转为其 `snapstack_msgs/State`，并把其 setpoint/轨迹转换为 MAVROS setpoint。

#### 工程成本

- 官方说明测试平台是 Ubuntu 20.04 + ROS Noetic；
- 依赖 Gurobi、CGAL、GLPK、DecompROS、NLopt 和 Snapstack 消息；
- 默认参数与 MIT 高棚实验空间和三维多项式轨迹有关；
- PX4/MAVROS 不是其原生飞控接口，需要编写消息、坐标系和控制适配层；
- Gurobi 虽对学术用途可获得许可，但部署和许可证管理比 OSQP/qpOASES 更复杂；
- 它通常接受 terminal goal 并在线生成多项式轨迹。要执行多航点，应由 waypoint manager 在达到容差后逐个提交目标，或将局部航点窗口转换成参考轨迹。

#### 结论

RMADER 是**需要有界通信延迟下轨迹级安全保证时的最佳开源研究基线**。首版可先复现其仿真并测量机载网络延迟分布；如果 CBF 安全层在高密度对穿或时延条件下难以满足要求，再迁移 RMADER，而不是一开始承担全部依赖和适配成本。

### 4.3 EGO-Swarm：ROS1 工程参考最完整，但安全是软约束

- 论文：X. Zhou et al., “EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments,” ICRA 2021；[arXiv:2011.04183](https://arxiv.org/abs/2011.04183)。
- 代码：[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)，GPL-3.0。
- 环境：README 声明 Ubuntu 16.04/18.04/20.04 + ROS/catkin 可编译；依赖 Armadillo；代码规模和社区使用量明显高于多数研究原型。
- 接口：每机订阅 odometry，发布 B-spline；所有无人机通过 `/broadcast_bspline` 广播未来轨迹。参数已有 `max_vel`、`max_acc`、`max_jerk`、`swarm_clearance` 和航点列表。
- 优点：去中心化、异步、毫秒级 B-spline 优化、实机多无人机实验、完整仿真/FSM/轨迹服务器可参考。
- 不足：机间碰撞通过优化 penalty 实现；局部优化失败、通信延迟或轨迹陈旧时不等同于硬安全保证。其默认系统还包含局部地图、深度图/点云和静态障碍处理。
- 移植建议：不要整体搬入地图模块。若选用，应保留 `plan_manage`、B-spline 优化、轨迹广播和轨迹服务器，删除或旁路环境感知，并在输出端再增加独立的紧急安全监督器。

### 4.4 ORCA / RVO2：最好用的二维速度基线

- 论文：J. van den Berg et al., “Reciprocal n-body Collision Avoidance,” ISRR 2009/2011；相关 ORCA 工作。
- 代码：[snape/RVO2](https://github.com/snape/RVO2)，Apache-2.0；现代仓库仍维护 CMake/Bazel/Meson 和 ROS `package.xml`。
- 输入：agent 位置、半径、最大速度、邻域参数和 preferred velocity；输出：二维碰撞规避速度。
- 优点：接口与航点速度跟踪天然吻合；二维；C++；低维线性规划；计算极快；无需显式通信未来轨迹。
- 风险：标准模型假设全向、速度可瞬时改变和互惠责任；对 PX4 只能作为速度参考生成器，不可直接声称真实动力学下必然无碰撞。拥挤或对称场景会有死锁/振荡风险，且仅靠感知当前速度无法自然覆盖网络时延。
- 建议：在相同仿真场景中与 CBF-QP 做 A/B 基线。若 ORCA 的到达时间、轨迹平滑性明显更好，也只能在加入加速度限制、状态年龄膨胀和紧急刹停监督后进入实机。

### 4.5 Hönig 等：适合地面站预处理，不适合分布式执行层

- 论文：W. Hönig et al., “Trajectory Planning for Quadrotor Swarms,” IEEE T-RO 34(4), 2018，DOI: [10.1109/TRO.2018.2853613](https://doi.org/10.1109/TRO.2018.2853613)。
- 代码：[whoenig/multi-robot-trajectory-planning](https://github.com/whoenig/multi-robot-trajectory-planning)，MIT。
- 方法：先用 CBS/ECBS 解决离散多机路径冲突，再连续轨迹平滑和时间伸缩；论文展示大规模仿真和 Crazyswarm 实验。
- 代码限制：公开重写版当前只支持二维 (E)CBS；连续优化部分需 MATLAB/libsvm；测试环境 Ubuntu 18.04；不是在线 ROS1 分布式执行器。
- 结论：非常适合在地面站生成任务时预先消除明显交叉，减轻机载安全层压力；不符合用户当前要求的分布式机载程序。

### 4.6 DENMPC：ROS1 控制框架可参考，但不是分布式多机方案

- 代码：[DentOpt/denmpc](https://github.com/DentOpt/denmpc)，GPL-3.0。
- 优点：catkin、C++、实时 NMPC、AR.Drone `/cmd_vel` 与 `/pose` 示例，展示了期望位姿跟踪和碰撞约束接口。
- 关键不匹配：README 明确列出的是 centralized multi-agent control，而不是每机分布式求解；项目较老，平台是 AR.Drone/TUM simulator，不是 PX4。
- 结论：可参考 OCP 模块化和 ROS 消息结构，不作为主候选。

## 5. 候选方案综合比较

评分：5 为最好；“安全保证”按论文模型和假设评价，不等于未经验证即可保证 PX4 实机安全。

| 方案 | 分布式/异步 | 与已有航点接口 | 二维低速 | 动力学可行性 | 通信现实性 | 形式化安全 | ROS1/PX4 移植 | 开源复现 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 二维 CBF-QP safety filter | 5 | 5 | 5 | 3（需鲁棒化） | 3（需自行处理） | 5（模型假设内） | 5 | 3 |
| RMADER | 5 | 3 | 4 | 5 | 5（有界延迟） | 5 | 3 | 4 |
| EGO-Swarm | 5 | 4 | 4 | 4 | 3 | 2（软惩罚） | 4 | 5 |
| Swarm-Formation | 5 | 2 | 4 | 4 | 3 | 2–3 | 3 | 4 |
| ORCA/RVO2 | 5 | 5 | 5 | 2 | 2–3 | 3（理想模型） | 5 | 5 |
| Hönig ECBS + smoothing | 1（集中式） | 3 | 5 | 4 | 1 | 5（规划模型内） | 2 | 3 |
| DENMPC | 1–2 | 4 | 4 | 5 | 2 | 3 | 3 | 3 |

## 6. 推荐的“编队执行驱动”架构

### 6.1 每架无人机独立运行的节点

```text
地面站/任务解析器
    │ 本机航点序列（统一 world/map 坐标、任务版本号）
    ▼
waypoint_manager
    │ 当前航点、下一航段、到达判定
    ▼
nominal_tracker
    │ u_nom = [vx_nom, vy_nom]
    ▼
distributed_safety_filter  ◄──── swarm_state_rx / neighbor predictor
    │ u_safe = [vx_safe, vy_safe]
    ▼
setpoint_shaper（速度/加速度/jerk 限幅）
    │ geometry_msgs/TwistStamped 或 PositionTarget
    ▼
MAVROS ──► PX4 OFFBOARD

独立 watchdog：通信、定位、求解器、PX4 mode、setpoint stream、地理围栏、急停
```

### 6.2 建议 ROS1 接口

每架机使用独立命名空间，例如 `/uav_03`。

| Topic | 类型建议 | 方向 | 内容 |
|---|---|---|---|
| `/uav_i/mavros/local_position/odom` | `nav_msgs/Odometry` | 输入 | 已统一到公共 world/map 坐标后的本机状态；若 MAVROS 原 topic 仍是各机 local，应订阅统一坐标模块的输出而非原值 |
| `/uav_i/mission/waypoints` | 自定义消息 | 输入 | `mission_id`、有序二维/三维航点、速度上限、容差、任务开始时间 |
| `/swarm/state` | 自定义消息或每机独立 topic | 收发 | `uav_id`、公共时基 stamp、位置、速度、加速度可选、状态质量、当前模式、消息序号 |
| `/swarm/intent` | 自定义消息 | 收发 | 短时预测轨迹或名义速度、当前航点索引、优先级、有效期；首版可选，RMADER 类方案必需 |
| `/uav_i/execution/nominal_cmd` | `geometry_msgs/TwistStamped` | 内部 | 原始航点跟踪速度 |
| `/uav_i/execution/safe_cmd` | `geometry_msgs/TwistStamped` | 内部 | 安全过滤后的速度 |
| `/uav_i/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | 输出 | PX4 OFFBOARD 速度命令；确认 MAVROS 插件期望 ENU/FLU 语义 |
| `/uav_i/execution/status` | 自定义消息 | 输出 | 状态机、最小间距、QP 状态、邻机消息最大年龄、当前航点、降级原因 |

不要只广播当前位置。为了降低“双方都依据过去状态行动”造成的风险，至少应广播：公共时基时间戳、速度、当前名义命令、消息序列号和有效期；更稳妥的是广播 1–3 秒短时预测轨迹。

### 6.3 坐标系和 MAVROS/PX4 边界

1. 安全层内部统一使用右手 ENU 的 `world` 或 `map` 坐标。
2. 不要手工在多个节点重复做 ENU/NED 变换；由单一适配层负责。
3. `mavros/setpoint_velocity/cmd_vel` 常使用 ROS ENU/FLU 语义并由 MAVROS 转换到 MAVLink/PX4 坐标，但必须按当前 MAVROS/PX4 版本做台架测试，不能仅凭 topic 名假设。
4. 固定高度由独立 Z 控制保持，避碰 QP 只产生 X/Y；发生严重冲突时不要临时采用 Z 轴分层，因为需求明确禁止 Z 堆叠。
5. OFFBOARD setpoint 必须持续高频发送；规划器低频更新时，setpoint streamer 仍应保持稳定频率并对命令做时间插值。

### 6.4 CBF-QP 首版设计建议

#### 名义航点跟踪

对当前航点 $p_i^g$：

$$
v_i^{nom} = \operatorname{sat}_{v_{nom,max}}\left(k_p(p_i^g-p_i)\right)
$$

在接近航点时按制动距离平滑减速，而不是到容差边界突然切换。航点切换应带滞回，避免 RTK 抖动导致索引反复变化。

#### 安全半径

不要把 $d_{safe}$ 只设为两架机物理半径之和。建议初始定义：

$$
d_{safe}=d_{body}+d_{rtk}+d_{track}+d_{comm}+d_{margin}
$$

其中：

- $d_{body}$：两机最大水平外廓半径之和；
- $d_{rtk}$：两机定位误差上界或高置信分位数之和；
- $d_{track}$：PX4 跟踪误差和制动距离；
- $d_{comm}$：状态年龄期间的最坏接近距离，可粗略取 $(v_i^{max}+v_j^{max})\tau_{age}$，再加入加速度项；
- $d_{margin}$：风、模型误差、桨叶柔性和试验保守裕度。

初次实机不要直接使用理论最小值。应由单机阶跃/刹停试验、RTK 静态/动态误差和网络时延统计共同标定。

#### 速度与加速度边界

- 任务速度硬上限：小于 2 m/s；首轮多机建议从 0.3–0.5 m/s 起步。
- safety filter 输出之后仍需做向量模长限幅，而非分别对 X/Y 限幅后导致合速度超限。
- 推荐 QP 直接优化加速度，或在速度 QP 后加入与理论一致的可达速度集合：

$$
\|v_i-v_i^{prev}\|\le a_{max}\Delta t
$$

- 若简单限幅会破坏 CBF 可行性，应把限幅约束纳入 QP，而不是在 QP 输出后再次裁剪。

#### 分布式责任与优先级

对称方队形成是典型死锁场景。建议采用确定性、全机一致的规则：

1. 按 `mission_id + conflict_pair + waypoint_index` 计算固定优先级；
2. 优先机尽量保持名义轨迹，让行机承担更多横向绕行或减速责任；
3. 优先级在一次冲突解除前不得高频翻转；
4. 形成编队时优先采用“减速/等待 + 少量横移”，不要让所有无人机同时对称绕行；
5. 若任务允许，可在地面站阶段用匈牙利算法重新分配方队槽位，先减少交叉，再由机载安全层处理剩余误差。

### 6.5 状态机

建议至少包含：

```text
INIT
  -> WAIT_FOR_STATE
  -> WAIT_FOR_MISSION
  -> PRESTREAM_SETPOINT
  -> ARM_AND_OFFBOARD
  -> TAKEOFF_HOLD
  -> TRACK_WAYPOINT
       -> AVOIDING（安全层显著修改命令）
       -> YIELDING（按优先级等待）
       -> TRACK_WAYPOINT
  -> FINAL_HOLD
  -> LAND / DISARM

任意飞行状态：
  -> BRAKE_HOLD（邻机消息陈旧、QP 不可行、RTK 质量下降）
  -> EMERGENCY_LAND（持续故障或人工命令）
```

对于二维同高避碰，所有无人机同时悬停未必安全：若已处于接近状态，制动距离仍可能造成碰撞。因此 `BRAKE_HOLD` 应基于当前相对速度计算可停止性；无法安全刹停时应执行预先验证的确定性逃逸方向，而不是简单把速度置零。

## 7. 建议实施路线

### 阶段 A：数据和接口基线

1. 记录每机统一坐标 odometry、RTK 状态、PX4 模式、实际/目标速度和通信收发时间。
2. 测量端到端状态年龄分布：采样、序列化、无线传输、ROS 回调到 safety filter 使用时刻。
3. 单机标定不同速度下的加速、制动距离、速度跟踪误差和 OFFBOARD 丢失行为。
4. 明确消息时间戳是采样时刻，不是接收或转发时刻；已有时间戳修正成果应直接用于状态年龄计算。

### 阶段 B：二维运动学仿真

实现相同接口的 CBF-QP 与 ORCA/RVO2 两套过滤器，至少覆盖：

- 两机正面对穿；
- 两机 90° 交叉；
- 四机交换方队对角位置；
- 多机同时汇聚到方队槽位；
- 一架机停止/降速，其他机经过；
- 20–300 ms 随机延迟；
- 连续丢包、乱序和重复包；
- RTK 位置偏置/噪声；
- 不同机体加速度和控制延迟。

记录：最小机间距、碰撞率、任务完成率、到达时间、路径长度、速度/加速度峰值、QP 不可行率、等待时间和死锁率。

### 阶段 C：PX4 SITL + MAVROS

1. 每架 PX4 SITL 使用独立 MAVROS namespace 和 MAVLink 端口。
2. 将 safety filter 输出接入实际计划使用的 velocity/position target 插件。
3. 注入网络延迟与丢包，而不是只在 planner 内延迟时间戳。
4. 验证 OFFBOARD setpoint 更新率、模式切换、失联 failsafe 和坐标转换。
5. 用真实 PX4 参数限制最大水平速度、倾角和加速度，防止上层配置失误突破安全边界。

### 阶段 D：单机与双机实机

1. 单机系留/桨叶安全条件下验证 setpoint 链路；
2. 单机低速直线、急停、航点切换；
3. 双机大间距、0.3 m/s、非对称交叉；
4. 双机正面对穿，先设置极保守安全半径；
5. 逐步提高到 0.5、1.0 m/s；在完整安全证据前不要直接测试接近 2 m/s；
6. 每次只改变一个参数，并保留 rosbag、PX4 ulog 和地面站日志。

### 阶段 E：方队形成与故障注入

- 4 机方队形成，再扩展数量；
- 目标槽位交换；
- 人为延迟一架机消息；
- 暂停一架机的上层规划但保持 PX4 悬停；
- 邻机广播中断；
- RTK fixed → float/失锁；
- QP 超时或返回 infeasible；
- 地面站任务版本不同步。

只有在故障注入下仍满足最小安全间距和可预测降级行为，才进入更密集编队。

### 阶段 F：RMADER 对照复现

如果 CBF-QP 出现以下任一问题，应启动 RMADER 迁移评估：

- 交叉场景频繁死锁；
- 需要在较长预测时域内主动绕行，而非短时减速；
- 实测通信延迟显著影响局部安全约束；
- 项目验收要求有界通信延迟下的轨迹级安全论证；
- 航点之间需要动态可行的平滑多项式轨迹，而不只是速度执行。

迁移顺序应是：官方 10-agent 仿真 → 改为二维固定高度 → 接入统一 odometry → 逐航点 goal adapter → setpoint adapter → PX4 SITL → 双机实机。不要一开始改优化器内部。

## 8. 参数与安全验收建议

### 8.1 需要实验标定的参数

| 参数 | 来源 | 不能直接照抄论文的原因 |
|---|---|---|
| `d_safe` | 机体外廓、RTK、跟踪、通信和裕度之和 | 不同机型、桨径、RTK 和网络差异很大 |
| `v_nom_max` / `v_safe_max` | 任务要求与实机标定 | 小于 2 m/s 只是上限，不代表首飞速度 |
| `a_max`, `jerk_max` | PX4 参数和阶跃试验 | 影响刹停距离与 QP 可达集合 |
| 邻机 timeout | 端到端延迟统计的高分位数 | ROS 接收频率不等于消息年龄 |
| 预测时域 | 最大闭环延迟与制动时间 | 太短看不到冲突，太长会保守/增加计算 |
| 航点容差/滞回 | RTK 动态噪声 | 太小导致无法到达，太大导致过早切点 |
| 优先级保持时间 | 冲突持续时间统计 | 太短会左右反复，太长会让低优先级机饥饿 |

### 8.2 建议的最低验收条件

以下阈值应由项目安全负责人根据机体重新填写数值，本文不虚构通用安全距离：

- 全部试验中 `min_pair_distance >= d_acceptance`；
- 正常网络条件下任务完成率达到预定标准；
- 延迟/丢包条件下无碰撞，且系统按设计进入 YIELDING 或 BRAKE_HOLD；
- safety filter 最坏求解时间小于控制周期，并有超时回退；
- 实际水平速度、加速度和倾角不超过设定边界；
- 任一无人机退出 OFFBOARD 时，其他无人机可识别并把它作为不可协作动态体处理；
- 所有日志可通过 `mission_id + uav_id + sequence + timestamp` 对齐重放；
- 紧急停止、遥控接管和地理围栏经过独立验证。

## 9. 风险与容易被忽略的问题

1. **“时钟已同步”不等于“状态是新鲜的”。** 仍需用采样时间计算 age，并拒绝乱序/过期消息。
2. **公共坐标统一不等于定位误差一致。** 不同 RTK fix 质量、杆臂和航向误差必须进入安全裕度。
3. **软 penalty 不是硬约束。** EGO-Swarm/Swarm-Formation 优化代价再大，也可能因局部极小值、求解失败或权重不当突破距离。
4. **QP 有解不等于 PX4 能跟上。** 若模型是瞬时速度而实机有姿态/加速度滞后，理论前向不变集可能被破坏。
5. **后处理限幅可能破坏安全。** 速度、加速度和边界应进入同一个优化问题或使用经证明的安全整形。
6. **对称避碰容易死锁。** 固定 ID 优先级、任务级通行权或槽位重分配应至少采用一种。
7. **只依赖邻机“也会避让”很脆弱。** 当邻机故障、失联或退出 OFFBOARD 时，本机应切换为承担全部避碰责任并增大安全域。
8. **同高飞行还有气动影响。** 即使没有 Z 轴堆叠，近距离横向相互作用、阵风和地效也会增加跟踪误差；NeuroMHE 可作为后续抗扰研究，但不能替代几何安全层。
9. **GPL 许可证会影响产品集成。** EGO-Swarm、Swarm-Formation、DENMPC 为 GPL-3.0；RMADER 为 BSD-3-Clause，RVO2 为 Apache-2.0，Hönig 代码为 MIT。若代码进入闭源产品，应先做许可证评审。

## 10. 最终建议

### 10.1 推荐技术路线

**首版实机：**

1. 保留现有任务解析与坐标统一模块；
2. 新增 `waypoint_manager + nominal_tracker`；
3. 新增每机独立的二维 CBF-QP safety filter；
4. 加入全机一致、冲突期间不翻转的优先级/让行协议；
5. 用消息 age 膨胀安全半径，状态超时进入可验证的制动策略；
6. 输出经加速度/jerk 约束的 MAVROS velocity setpoint；
7. 先与 RVO2 做仿真对照，再逐级进入 SITL、双机和方队实机。

**第二阶段：**

- 若需要更长时域的主动轨迹绕行和有界时延论证，迁移 RMADER；
- 若需要未知环境静态障碍和完整自主规划，才考虑 EGO-Swarm；
- 若任务层允许调整方队槽位，引入 Swarm-Formation/T-RO 2023 的低频 assignment/alignment 思路，从源头减少交叉；
- 若近距离编队受风和下洗干扰明显，再研究 NeuroMHE 作为 PX4 外层或轨迹跟踪器的抗扰增强。

### 10.2 一句话决策

> 对“已有每机航点、无静态障碍、二维低速、需要分布式实机避碰”的当前边界，优先自行实现并严格验证 **CBF-QP 最小干预安全过滤器**；把 **RMADER** 作为最值得复现的开源轨迹级方案，把 **EGO-Swarm** 作为 ROS1 多机工程参考，而不是直接把复杂障碍规划系统整体移植进编队执行驱动。

## 11. 主要来源

### 当前仓库

- Paredes-Vallés et al., *Fully neuromorphic vision and control for autonomous drone flight*：仓库 Markdown 对照译文及 `note.md`。
- Hagenaars et al., *Self-Supervised Learning of Event-Based Optical Flow with Spiking Neural Networks*：仓库 Markdown 对照译文。
- Loquercio et al., *Learning high-speed flight in the wild*：仓库 Markdown 对照译文。
- Wang et al., *Neural Moving Horizon Estimation for Robust Flight Control*：仓库 Markdown 对照译文；代码：[RCL-NUS/NeuroMHE](https://github.com/RCL-NUS/NeuroMHE)。
- Quan et al., *Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments*：仓库 PDF。
- Quan et al., *Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments*：仓库 PDF。
- Hu et al., *Concurrent-Allocation Task Execution for Multirobot Path-Crossing-Minimal Navigation in Obstacle Environments*：仓库 PDF。

### 外部论文与官方代码

- Wang, Ames, Egerstedt, “Safety Barrier Certificates for Collisions-Free Multirobot Systems,” IEEE T-RO 2017：[论文 PDF](https://jdeshmukh.github.io/teaching/cs699-fm-for-robotics-spring-2021/Papers/SafetyBarrierCertificatesForCollisionFreeMultiRobotSystems-WangEtAl.pdf)。
- Wang, Ames, Egerstedt, “Safe Certificate-Based Maneuvers for Teams of Quadrotors Using Differential Flatness,” ICRA 2017：[arXiv](https://arxiv.org/abs/1702.01075)。
- Kondo et al., “Robust MADER,” RA-L/ICRA 2023：[论文](https://arxiv.org/abs/2303.06222)，[代码](https://github.com/mit-acl/rmader)。
- Zhou et al., “EGO-Swarm,” ICRA 2021：[论文](https://arxiv.org/abs/2011.04183)，[代码](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)。
- Quan et al., “Distributed Swarm Trajectory Optimization...,” ICRA 2022：[代码](https://github.com/ZJU-FAST-Lab/Swarm-Formation)。
- Hönig et al., “Trajectory Planning for Quadrotor Swarms,” T-RO 2018：[代码](https://github.com/whoenig/multi-robot-trajectory-planning)。
- van den Berg et al., ORCA / RVO2：[项目主页](https://gamma.cs.unc.edu/RVO2/)，[C++ 代码](https://github.com/snape/RVO2)。
- Dentler et al., DENMPC：[代码](https://github.com/DentOpt/denmpc)。
- Robotarium MATLAB simulator：[代码](https://github.com/robotarium/robotarium-matlab-simulator)。

## 12. 调研局限

- 本报告基于当前仓库内容、论文公开页面、作者/实验室页面和公开 GitHub 仓库；没有在目标机载电脑、指定 ROS/PX4 版本和真实无线网络上编译运行候选代码。
- 仓库中的相关论文多数只有 PDF，不是 Markdown 对照翻译；本文已明确区分两类证据。
- 开源仓库的可编译状态、依赖版本和接口可能随分支变化。正式实施前应固定 commit，并在目标 Ubuntu/ROS/PX4/MAVROS 组合上复现。
- 任何论文中的“collision-free guarantee”都依赖其模型、感知、通信和求解假设。未经机体参数标定、通信故障注入和分阶段实机验证，不能把论文保证直接当作飞行安全认证。
- 本调研使用了 AI 辅助检索与归纳；关键安全决策应由项目团队结合原论文、源代码审查和实测数据复核。