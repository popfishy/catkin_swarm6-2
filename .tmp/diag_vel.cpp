#include "ego_planner_driver/ego_planner_core.h"
#include "ego_planner_driver/polynomial_traj.h"
#include <cstdio>
using namespace ego_planner_driver;
int main() {
  EgoPlannerConfig c; c.ts=0.4; c.max_vel=1.0; c.max_acc=6.0; c.cruise_velocity=1.0;
  c.lambda_feasibility=0.5; c.enable_rebound=false; c.enable_time_reparam=false;
  c.max_arc_samples=4096; c.max_parameterization_points=256; c.max_trajectory_samples=4096;
  EgoPlannerCore core(c);
  GoalPoint s{Eigen::Vector3d(0,0,15),0};
  std::vector<WayPoint> wps;
  for (double x=0.4; x<=60+1e-6; x+=0.4) wps.push_back(WayPoint{Eigen::Vector3d(x,0,15),0});
  TimedTrajectory t; double cost=0;
  PlanResult r = core.planOnce(s, wps, {}, t, cost);
  printf("result=%d pts=%zu\n", (int)r, t.pts.size());
  double vmax=0, amax=0; int vi=0;
  for (size_t i=0;i<t.vel.size();++i) { double v=t.vel[i].norm(); if(v>vmax){vmax=v;vi=(int)i;} }
  for (size_t i=0;i<t.acc.size();++i) amax=std::max(amax,t.acc[i].norm());
  printf("vmax=%.3f at i=%d t=%.2f amax=%.3f duration=%.2f\n", vmax, vi, t.t.empty()?0:t.t[vi], amax, t.duration);
  return 0;
}
