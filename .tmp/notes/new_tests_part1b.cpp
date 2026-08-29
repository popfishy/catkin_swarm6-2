TEST(EgoStateMachine, PlanExecArrivalCompletesAtFinalKeypoint)
{
  // plan-exec 分支 B：距最终关键点 <0.1m（稳定窗口后）→ COMPLETED，冻结目标位姿。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int x : {0, 3, 6, 9})
  {
    geometry_msgs::Point32 p;
    p.x = x; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 飞到最终关键点 (9,0,5)：0.05m < 0.1m 到达阈值。
  node.injectLocalPose(makePose(8.95, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();  // 到达窗口开始（稳定未满）
  EXPECT_EQ(node.execState(), kStateExecuting);
  clock.advance(1.1);  // > arrival_stable_s
  node.injectLocalPose(makePose(8.95, 0.0, 5.0, clock.get()));  // pose 保持新鲜
  node.arrivalCheckTick();
  EXPECT_EQ(node.execState(), kStateCompleted);
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 9.0, 1e-6);
  EXPECT_NEAR(node.steadyTarget()->pos.z(), 5.0, 1e-6);
}

TEST(EgoStateMachine, RollingSegmentsReachCompleted)
{
  // 相邻目标推进：3m 密化关键点多段滚动后最终 COMPLETED（prepare 场景到达路径）。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 8; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 4000; ++guard)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      ASSERT_TRUE(node.replanOnce());
    if (node.execState() == kStateCompleted)
    {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed);
  if (completed)
    EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
}

TEST(EgoStateMachine, RollingSegmentsReachCompletedWithTrackingLag)
{
  // 模拟 SITL 跟踪滞后 0.05m（< arrival_reach_thresh_m 0.1m）：推进/到达判定
  // 仍收敛到 COMPLETED。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 8; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 4000; ++guard)
  {
    node.publishSetpointTick();
    const mavros_msgs::PositionTarget sp = node.lastSetpoint();
    node.injectLocalPose(makePose(sp.position.x - 0.05, sp.position.y,
                                  sp.position.z, clock.get()));
    clock.advance(0.1);
    if (node.planningRequested())
      ASSERT_TRUE(node.replanOnce());
    if (node.execState() == kStateCompleted)
    {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed);
}
