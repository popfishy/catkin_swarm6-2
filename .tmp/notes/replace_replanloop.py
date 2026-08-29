#!/usr/bin/env python3
"""Surgical replacement of EgoPlannerDriverNode::replanLoop (lines 1562-1741)."""
from pathlib import Path

p = Path("/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/src/ego_planner_driver_node.cpp")
lines = p.read_text(encoding="utf-8").splitlines(keepends=True)

# 1-based inclusive range to replace
start, end = 1562, 1741

new_fn = '''void EgoPlannerDriverNode::replanLoop()
{
  // 单一 replan 入口（方案 A）：replanLoop 是唯一轨迹提交者。常态 replan_period_s
  // （1.0s）定时由 wait_for 超时兜底；planning_requested_/emergency_pending_ 置位
  // 时事件唤醒立即执行（条件变量）。replanOnce 内部按优先级处理：EMERGENCY 制动
  // 分支 → 常规 replan → 候选（replan 失败分支）。
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_ && replan_loop_enabled_)
  {
    replan_cv_.wait_for(lock, std::chrono::duration<double>(replan_period_s_),
        [this]() {
          return stop_ || !replan_loop_enabled_ || planning_requested_ ||
                 emergency_pending_;
        });
    if (stop_ || !replan_loop_enabled_)
      break;
    if (hold_requested_)
    {
      processHoldLocked();
      continue;
    }
    if (exec_state_ == kStateTakeoff)
      continue;
    if (direct_control_active_)
      continue;
    // 对齐原版 SEQUENTIAL_START：首轮（current_ 空 = 新任务未规划）规划前等待
    // 前驱无人机 intent（drone_id 顺序，前机轨迹到达才首轮规划，避免多机同时
    // 盲规划初始轨迹冲突）。leader(id<=1) / 已收到前驱 / 超时
    // first_plan_wait_timeout_s 后放行；否则清除事件等下一个 1s 周期/前驱 intent
    // 事件再试（不 busy-loop）。
    if (!current_)
    {
      const ros::Time now = nowFunc();
      if (!firstPlanReadyLocked(now))
      {
        if (first_plan_requested_at_.isZero())
          first_plan_requested_at_ = now;
        planning_requested_ = false;
        continue;
      }
    }
    if (!planning_requested_ && !emergency_pending_)
      continue;
    lock.unlock();
    replanOnce();  // 唯一提交者：常规 replan / 候选 / EMERGENCY 制动都在这
    lock.lock();
  }
}
'''

lines[start - 1:end] = [new_fn]
p.write_text("".join(lines), encoding="utf-8")
print("replaced lines %d-%d" % (start, end))
