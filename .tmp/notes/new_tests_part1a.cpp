TEST(EgoStateMachine, PlanExecAdvanceTargetSkipsReachedKeypoints)
{
  // plan-exec 分支 A：距当前目标 <0.5m → 切相邻下一个 keypoint 为目标。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // start(0) + 3m 间距关键点：(0),(3),(6),(9) —— waypoints_[0] 为 start。
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

  // 位置 (3.2,0,5)：距当前目标 (3,0,5)=0.2m <0.5m → 推进到 (6,0,5)。
  node.injectLocalPose(makePose(3.2, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  EXPECT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  EXPECT_NEAR(traj->pts.back().x(), 6.0, 0.6);

  // 位置 (6.2,0,5)：距当前目标 (6,0,5)=0.2m <0.5m → 推进到最终 (9,0,5)。
  node.injectLocalPose(makePose(6.2, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  ASSERT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj2 = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj2));
  EXPECT_NEAR(traj2->pts.back().x(), 9.0, 0.6);

  // 最终关键点已为当前目标（waypoint_consumed_ == size-1）→ 不再推进。
  node.injectLocalPose(makePose(8.9, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  EXPECT_FALSE(node.planningRequested());
}

TEST(EgoStateMachine, PlanExecAdvanceDiscardsTooCloseTargets)
{
  // plan-exec 分支 A：多个关键点挤在 0.5m 内时丢弃推进，直接跳到 >=0.5m 目标。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // start(0), (3),(3.1),(3.2),(6) —— (3)/(3.1)/(3.2) 三连点 <0.5m 内。
  std::vector<geometry_msgs::Point32> wps;
  for (double x : {0.0, 3.0, 3.1, 3.2, 6.0})
  {
    geometry_msgs::Point32 p;
    p.x = x; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());

  // 位置 (3.05,0,5)：距 (3,0,5)=0.05 <0.5 → 丢弃 (3)/(3.1)/(3.2)，目标 → (6,0,5)。
  node.injectLocalPose(makePose(3.05, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  ASSERT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  EXPECT_NEAR(traj->pts.back().x(), 6.0, 0.6);
}
