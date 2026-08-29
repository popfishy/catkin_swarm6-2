TEST(EgoStateMachine, ReplanSnapshotsKeepSegmentVelocity)
{
  // 回归：periodic replan 在当前段执行中快照时，新段起点必须保持旧轨迹速度
  // （rolling continuity），不是从 0 重新加速。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 4; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  for (int step = 0; step < 60; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  EXPECT_TRUE(std::isfinite(sp.velocity.x) && std::isfinite(sp.velocity.y));
  const double speed = std::sqrt(sp.velocity.x * sp.velocity.x +
                                 sp.velocity.y * sp.velocity.y +
                                 sp.velocity.z * sp.velocity.z);
  EXPECT_GT(speed, 0.05);
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 持续推进，setpoint 应越过首段目标（3m）。
  double max_x = sp.position.x;
  for (int step = 0; step < 100; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    max_x = std::max(max_x, node.lastSetpoint().position.x);
    clock.advance(0.1);
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_GT(max_x, 3.0);
}

TEST(EgoStateMachine, PeriodicReplanTriggersDuringSegmentExecution)
{
  // 分支 C：段内 elapsed >= replan_period_s（1.0s）→ 时间触发置 planning_requested_。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 4; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_FALSE(node.planningRequested());

  // 段内推进超过 replan_period_s（1.0s）且未到最终目标 → 时间触发重规划。
  for (int step = 0; step < 20; ++step)
  {
    node.publishSetpointTick();
    clock.advance(0.1);
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, ReplanUsesTrajectoryStateNotLivePose)
{
  // 滚动重规划起点必须来自旧轨迹同一时刻的完整状态，而不是"实际位置 + 旧导数"。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  for (int step = 0; step < 20; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  const double old_x = node.lastSetpoint().position.x;

  // 注入偏离 100m 的实际位姿。
  const double dev_x = old_x + 100.0;
  node.injectLocalPose(makePose(dev_x, node.lastSetpoint().position.y,
                                node.lastSetpoint().position.z, clock.get()));

  // 滚动 replan：新段起点应取旧轨迹状态（≈old_x），而非注入的 dev_x。
  ASSERT_TRUE(node.replanOnce());
  node.publishSetpointTick();
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  EXPECT_LT(std::abs(sp.position.x - old_x), 1.0);
  EXPECT_GT(std::abs(sp.position.x - dev_x), 50.0);
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, HoldLocksPositionNotPoseDrift)
{
  // 删 POSE_STALE 后：HOLD setpoint = 进入 HOLD 时锁存的位置/yaw，不随 pose 漂移。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);

  ASSERT_TRUE(setupHold(node, clock, 12.0));  // 捕获 HOLD → 锁存 (0,0,12)
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);

  // pose 漂移（如 GPS 抖动）不影响 HOLD setpoint：锁存位置不更新。
  node.injectLocalPose(makePose(5.0, 3.0, 9.0, clock.get()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.y, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);
  EXPECT_EQ(node.execState(), kStateHold);

  // 持续漂移也不改变锁存。
  node.injectLocalPose(makePose(10.0, 10.0, 2.0, clock.get()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);
}
