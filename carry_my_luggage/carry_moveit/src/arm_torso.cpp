// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_torso");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";

  // Check for command line arguments
  if (argc == 7)
  {
    goal_pose.pose.position.x = atof(argv[1]);
    goal_pose.pose.position.y = atof(argv[2]);
    goal_pose.pose.position.z = atof(argv[3]);
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  }
  else
  {
    nh.param("goal_pose/position/x", goal_pose.pose.position.x, 0.0);
    nh.param("goal_pose/position/y", goal_pose.pose.position.y, 0.0);
    nh.param("goal_pose/position/z", goal_pose.pose.position.z, 0.0);

    double roll, pitch, yaw;
    nh.param("goal_pose/orientation/roll", roll, 0.0);
    nh.param("goal_pose/orientation/pitch", pitch, 0.0);
    nh.param("goal_pose/orientation/yaw", yaw, 0.0);
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }


  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();


  double max_velocity_scaling_factor;
  nh.param("max_velocity_scaling_factor_arm_prepare", max_velocity_scaling_factor, 0.3);  // 默认 0.3
  group_arm_torso.setMaxVelocityScalingFactor(max_velocity_scaling_factor);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if (!success)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");


  ros::Time start = ros::Time::now();
  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}


