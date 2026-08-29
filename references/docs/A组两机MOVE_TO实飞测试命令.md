# A组两机 MOVE_TO 实飞测试命令

## 1. 首先修改两机目标坐标

编辑任务计划：

```bash
cd /home/yjq/catkin_swarm6-2
nano src/runtime/examples/joint_mission/group_a/plans/two_uav_smoke.json
```

分别修改 A01、A02 的 `target_pose`：

```json
"target_pose": {
  "x": 0.0,
  "y": 2.0,
  "z": 5.0,
  "yaw": 0.0
}
```

- `x`：标定任务坐标系右方，单位 m。
- `y`：标定任务坐标系前方，单位 m。
- `z`：高度，单位 m。
- `yaw`：航向，单位 rad。

坐标是绝对任务坐标，不是“从当前位置移动多少米”。修改后更新校验值：

```bash
cd /home/yjq/catkin_swarm6-2
python3 src/runtime/tools/update_mission_hashes.py \
  src/runtime/examples/joint_mission/group_a
```

> 当前 EGO 执行器处理 `MOVE_TO` 时，会先进入 15 m 动作层，再水平移动，最后下降到目标 `z`。
> 它不是保持 5 m 高度直接水平移动。实飞前必须确认空域和高度限制允许该过程。

## 2. 现场文件准备

准备实机拓扑配置文件，例如：

```text
/home/yjq/catkin_swarm6-2/config/topology_group_a_two_uav_real.yaml
```

该文件至少包含 `GCS_A`、`UAV1`、`UAV2`，IP 必须与现场一致，实机 `port_offset` 设置为 `0`。

准备已经验证过的实机自动起飞脚本。下文使用：

```text
<实机自动起飞脚本>
```

代替它的实际绝对路径和实际参数。

> 不要用 `verification/ego_smoke/offboard_takeoff_15.py` 进行实机起飞。该脚本面向本机多 Master
> 的 PX4 SITL，并包含无 RC 仿真参数。

## 3. 上电顺序

1. 打开发射机，确认可以切换人工模式并接管。
2. 启动 GCS_A 和自组网设备。
3. 将 A01、A02 放在安全起飞点，机头朝向公共任务“前方”。
4. 给 A01、A02 上电，保持未解锁状态。
5. 确认 GCS_A、UAV1、UAV2 网络互通。

在 GCS_A 检查网络，IP 按现场修改：

```bash
ping -c 3 192.168.5.11
ping -c 3 192.168.5.21
```

## 4. UAV1 启动命令

以下命令分别在 UAV1 的独立终端执行。

### UAV1终端1：ROS Master

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.5.11

roscore -p 11311
```

### UAV1终端2：MAVROS

`<UAV1_FCU_URL>` 替换为现场飞控串口或 UDP 地址。

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.5.11

roslaunch mavros px4.launch fcu_url:=<UAV1_FCU_URL>
```

### UAV1终端3：拓扑桥

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.5.11

roslaunch tcp_to_ros topology_group_a_sim.launch \
  uav_name:=UAV1 uav_id:=A01 run_uav_mock:=false \
  config:=/home/yjq/catkin_swarm6-2/config/topology_group_a_two_uav_real.yaml
```

### UAV1终端4：原点接收

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.5.11

rosrun swarm_uav_executor gp_origin_receiver.py
```

### UAV1终端5：任务执行器

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.5.11

roslaunch swarm_uav_executor uav_executor_ego.launch \
  uav_id:=A01 exec_target:=UAV1 service_namespace:=UAV1 \
  state_topic:=/uav_task_state \
  neighbor_intents:=/UAV2/trajectory_intent \
  neighbor_odom_topics:=/UAV2/mavros/local_position/odom \
  configure_px4_params:=false
```

## 5. UAV2 启动命令

以下命令分别在 UAV2 的独立终端执行。

### UAV2终端1：ROS Master

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=192.168.5.21

roscore -p 11312
```

### UAV2终端2：MAVROS

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=192.168.5.21

roslaunch mavros px4.launch fcu_url:=<UAV2_FCU_URL>
```

### UAV2终端3：拓扑桥

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=192.168.5.21

roslaunch tcp_to_ros topology_group_a_sim.launch \
  uav_name:=UAV2 uav_id:=A02 run_uav_mock:=false \
  config:=/home/yjq/catkin_swarm6-2/config/topology_group_a_two_uav_real.yaml
```

### UAV2终端4：原点接收

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=192.168.5.21

rosrun swarm_uav_executor gp_origin_receiver.py
```

### UAV2终端5：任务执行器

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=192.168.5.21

roslaunch swarm_uav_executor uav_executor_ego.launch \
  uav_id:=A02 exec_target:=UAV2 service_namespace:=UAV2 \
  state_topic:=/uav_task_state \
  neighbor_intents:=/UAV1/trajectory_intent \
  neighbor_odom_topics:=/UAV1/mavros/local_position/odom \
  configure_px4_params:=false
```

## 6. GCS_A 启动和标定

以下命令分别在 GCS_A 的独立终端执行。

### GCS终端1：ROS Master

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310

roscore -p 11310
```

### GCS终端2：拓扑桥

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310

roslaunch tcp_to_ros topology_group_a_sim.launch \
  uav_name:=GCS_A run_uav_mock:=false \
  config:=/home/yjq/catkin_swarm6-2/config/topology_group_a_two_uav_real.yaml
```

确认两架飞机状态已经桥接到 GCS_A：

```bash
rostopic echo -n 1 /UAV1/mavros/state
rostopic echo -n 1 /UAV2/mavros/state
rostopic echo -n 1 /UAV1/mavros/global_position/global
rostopic echo -n 1 /UAV2/mavros/global_position/global
```

此时必须满足：两机 `connected: True`、`armed: False`，GPS 数据有效。
+
### GCS终端3：临时设置两机标定范围

正式配置默认使用 A01–A06 计算标定，并等待 A01–A15 回读。仅测试两机时，启动 backend 前执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310

rosparam set /gcs_a_calibration/calibration_uavs "[A01, A02]"
rosparam set /gcs_a_calibration/all_uavs "[A01, A02]"
rosparam set /gcs_a_calibration/identity_map "{A01: UAV1, A02: UAV2}"
```

这三个参数不是 `yaml` 文件参数，而是写入 ROS 参数服务器的临时参数。
程序 `src/tcp_to_ros/scripts/gcs_a_calibration_node.py` 使用私有参数
`~calibration_uavs`、`~all_uavs`、`~identity_map` 读取它们；节点名为
`gcs_a_calibration`，所以完整参数名就是上面的三个名称。必须在启动
`gcs_a_backend.launch` 前设置；节点已经运行后再修改不会更新其内存配置。

该覆盖只用于两机测试，9/12机测试时不得使用。

### GCS终端4：启动 backend 和标定程序

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310

roslaunch tcp_to_ros gcs_a_backend.launch
```

保持两机未解锁，等待至少数秒，然后检查：

```bash
rostopic echo -n 1 /group_a/calibration_ready
rostopic echo -n 1 /group_a/reference_heading
rostopic echo -n 1 /UAV1/gp_origin_confirmed
rostopic echo -n 1 /UAV2/gp_origin_confirmed
```

必须满足：

```text
calibration_ready: True
UAV1 gp_origin_confirmed: True
UAV2 gp_origin_confirmed: True
```

如果一直为 `False`，检查两机是否未解锁、GPS 是否连续、两机机头航向差是否小于 30°。

## 7. GCS_A 启动 TCP Server

### GCS终端5：检查并启动端口39001

```bash
source /opt/ros/noetic/setup.bash
source /home/yjq/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310

rosrun tcp_to_ros check_tcp_port.py --port 39001
rosrun tcp_to_ros tcp_server_node.py
```

保持该终端运行。

## 8. runtime 离线检查

飞机仍处于未解锁状态时执行：

```bash
cd /home/yjq/catkin_swarm6-2

python3 src/runtime/task_test.py \
  --backend memory \
  --group A \
  --task prepare-two-uav
```

确认输出包含：

```text
status = SUCCESS
command_type = MOVE_TO
robot_ids = A01、A02
robot_count = 2
```

## 9. 自动起飞并进入 OFFBOARD

标定和离线检查成功后，运行已经过实机验证的自动起飞脚本：

```bash
cd /home/yjq/catkin_swarm6-2
python3 <实机自动起飞脚本> <A01和A02对应参数>
```

脚本应按以下顺序执行：

```text
持续发送安全 setpoint → 解锁 → OFFBOARD → 软起飞约5m → HOLD
```

在 GCS_A 检查：

```bash
export ROS_MASTER_URI=http://localhost:11310

rostopic echo -n 1 /UAV1/mavros/state
rostopic echo -n 1 /UAV2/mavros/state
rostopic echo -n 1 /UAV1/mavros/local_position/pose
rostopic echo -n 1 /UAV2/mavros/local_position/pose
```

必须满足：

```text
A01、A02 armed: True
A01、A02 mode: OFFBOARD
两机高度稳定
两机安全间距正常
```

## 10. runtime 发送两机 MOVE_TO

runtime 与 GCS_A 在同一台计算机时执行：

```bash
cd /home/yjq/catkin_swarm6-2

python3 src/runtime/task_test.py \
  --backend tcp \
  --group A \
  --task prepare-two-uav \
  --tcp-host 127.0.0.1 \
  --tcp-port 39001 \
  --max-ticks 2400
```

runtime 在其他计算机时，将 `127.0.0.1` 改为 GCS_A 的实际 IP。

## 11. 验收和异常处理

正常结果：

- TCP Server 只收到 A01、A02 的 `MOVE_TO`。
- 两机按“15 m动作层 → 水平移动 → 目标高度”执行。
- runtime 最终输出 `status: SUCCESS`。
- 两机结束后稳定保持目标点。

异常时优先使用遥控器人工接管。任务仍处于活动状态时，也可以在 GCS_A 请求整批 HOLD：

```bash
export ROS_MASTER_URI=http://localhost:11310
rosservice call /group_a/emergency_hold \
  "mission_id: ''
uav_ids: ['A01', 'A02']
reason: 'manual emergency hold'"
```

测试结束后，9/12机正式标定前清除两机临时参数，并重新启动 backend：

```bash
export ROS_MASTER_URI=http://localhost:11310
rosparam delete /gcs_a_calibration/calibration_uavs
rosparam delete /gcs_a_calibration/all_uavs
rosparam delete /gcs_a_calibration/identity_map
```
