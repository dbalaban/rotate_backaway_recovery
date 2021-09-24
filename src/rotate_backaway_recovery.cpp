#include <rotate_backaway_recovery/rotate_backaway_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>

PLUGINLIB_EXPORT_CLASS(rotate_backaway_recovery::RotateBackawayRecovery, nav_core::RecoveryBehavior)

namespace rotate_backaway_recovery
{

RotateBackawayRecovery::RotateBackawayRecovery() : local_costmap_(NULL), initialized_(false), should_backaway_(false), world_model_(NULL)
{
}

void RotateBackawayRecovery::initialize(std::string name,
                                        tf2_ros::Buffer*,
                                        costmap_2d::Costmap2DROS*,
                                        costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", anguar_tolerance_, 0.10);

    blp_nh.param("acc_lim_x", acc_lim_x_, 2.5);
    blp_nh.param("max_vel_x", max_linear_vel_, 0.5);
    blp_nh.param("min_vel_x", min_linear_vel_, 0.1);
    blp_nh.param("xy_goal_tolerance", linear_tolerance_, 0.1);

    blp_nh.param("backaway_dist", backaway_dist_, 0.07);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateBackawayRecovery::~RotateBackawayRecovery()
{
  delete world_model_;
}

void RotateBackawayRecovery::runRotateBehavior()
{
  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double start_angle = current_angle;

  bool got_180 = false;

  while (n.ok() &&
         (!got_180 ||
          std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > anguar_tolerance_))
  {
    // Update Current Angle
    local_costmap_->getRobotPose(global_pose);
    current_angle = tf2::getYaw(global_pose.pose.orientation);

    // compute the distance left to rotate
    double dist_left;
    if (!got_180)
    {
      // If we haven't hit 180 yet, we need to rotate a half circle plus the distance to the 180 point
      double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
      dist_left = M_PI + distance_to_180;

      if (distance_to_180 < anguar_tolerance_)
      {
        got_180 = true;
      }
    }
    else
    {
      // If we have hit the 180, we just have the distance back to the start
      dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
    }

    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    // check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    while (sim_angle < dist_left)
    {
      double theta = current_angle + sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        should_backaway_ = true;
        return;
      }
      should_backaway_ = false;

      sim_angle += sim_granularity_;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_th_ * dist_left);

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}

double RotateBackawayRecovery::getDistance(geometry_msgs::PoseStamped& start_pose)
{
  geometry_msgs::PoseStamped current_pose;
  local_costmap_->getRobotPose(current_pose);

  const double dx = start_pose.pose.position.x - current_pose.pose.position.x;
  const double dy = start_pose.pose.position.y - current_pose.pose.position.y;

  return std::sqrt(dx*dx + dy*dy);
}

void RotateBackawayRecovery::runBackawayBehavior()
{
  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped start_pose;
  local_costmap_->getRobotPose(start_pose);

  double dist = 0;
  while (n.ok() && dist < backaway_dist_)
  {
    double dist_left = backaway_dist_ - dist;
    double vel = sqrt(2 * acc_lim_x_ * dist_left);
    vel = std::min(std::max(vel, min_linear_vel_), max_linear_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    r.sleep();
    dist = getDistance(start_pose);
  }
}

void RotateBackawayRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");
  runRotateBehavior();

  if (should_backaway_)
  {
    ROS_WARN("Backaway recovery behavior started.");
    runBackawayBehavior();
  }

  should_backaway_ = false;
}

}