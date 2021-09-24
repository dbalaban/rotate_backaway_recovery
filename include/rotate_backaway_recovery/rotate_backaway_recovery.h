#ifndef ROTATE_BACKAWAY_RECOVERY_ROTATE_BACKAWAY_RECOVERY_H
#define ROTATE_BACKAWAY_RECOVERY_ROTATE_BACKAWAY_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <math.h>>
#include <string>

namespace rotate_backaway_recovery
{

class RotateBackawayRecovery : public nav_core::RecoveryBehavior
{
public:
  RotateBackawayRecovery();
  void initialize(std::string name,
                  tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*,
                  costmap_2d::Costmap2DROS* local_costmap);

  void runBehavior();

  ~RotateBackawayRecovery();

private:
  void runRotateBehavior();
  void runBackawayBehavior();
  double getDistance(geometry_msgs::PoseStamped& start_pose);

  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  bool should_backaway_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_,
         acc_lim_th_, anguar_tolerance_, frequency_,
         min_linear_vel_, max_linear_vel_, acc_lim_x_,
         linear_tolerance_, backaway_dist_;
  base_local_planner::CostmapModel* world_model_;
};
};

#endif