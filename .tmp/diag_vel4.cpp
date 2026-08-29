#include "ego_planner_driver/ego_planner_core.h"
#include "ego_planner_driver/polynomial_traj.h"
#include <cstdio>
using namespace ego_planner_driver;
int main() {
  EgoPlannerConfig c; c.ts=0.4; c.max_vel=1.0; c.max_acc=6.0; c.cruise_velocity=1.0;
  c.lambda_feasibility=0.5; c.enable_rebound=false; c.enable_time_reparam=false;
  c.max_arc_samples=4096; c.max_parameterization_points=256; c.max_trajectory_samples=4096;
  EgoPlannerCore core(c);
  // 临时禁用峰值拒绝以便实测（用宽松 cap 配置）
  GoalPoint s{Eigen::Vector3d(0,0,5),0};
  for (double d: {3.0,5.0,6.0}) {
    std::vector<WayPoint> wps{WayPoint{Eigen::Vector3d(d,0,5),0}};
    for (bool fin: {false,true}) {
      TimedTrajectory t; double cost=0;
      PlanResult r = core.planOnce(s, wps, {}, t, cost, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), fin);
      double vmax=0, amax=0;
      for (auto&v:t.vel) vmax=std::max(vmax,v.norm());
      for (auto&a:t.acc) amax=std::max(amax,a.norm());
      printf("d=%.1f fin=%d r=%d vmax=%.3f amax=%.3f\n", d,(int)fin,(int)r,vmax,amax);
    }
  }
  return 0;
}
