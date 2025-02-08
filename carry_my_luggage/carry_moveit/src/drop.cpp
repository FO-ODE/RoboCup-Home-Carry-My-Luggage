// ROS headers
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdexcept>


geometry_msgs::PoseStamped first_detach_position;

// Functions: Set target and lift position
void setGoalAndLiftPositions()
{
    double x1, y1, z1, ow1, ox1, oy1, oz1;
    ros::NodeHandle nh;
    nh.param("first_detach_position/position/x", x1, 0.5);
    nh.param("first_detach_position/position/y", y1, 0.0);
    nh.param("first_detach_position/position/z", z1, 0.6);
    nh.param("first_detach_position/orientation/w", ow1, -0.5);
    nh.param("first_detach_position/orientation/x", ox1, 0.5);
    nh.param("first_detach_position/orientation/y", oy1, 0.5);
    nh.param("first_detach_position/orientation/z", oz1, -0.5);


    first_detach_position.header.frame_id = "base_link";
    first_detach_position.pose.position.x = x1;
    first_detach_position.pose.position.y = y1;
    first_detach_position.pose.position.z = z1; 
    first_detach_position.pose.orientation.w = ow1;
    first_detach_position.pose.orientation.x = ox1;
    first_detach_position.pose.orientation.y = oy1;
    first_detach_position.pose.orientation.z = oz1;


}

// Function: Move the robot arm to the target position
int moveArm(const geometry_msgs::PoseStamped& target_pose)
{
    double max_velocity_scaling_factor;
    ros::NodeHandle nh;
    nh.param("max_velocity_scaling_factor_drop", max_velocity_scaling_factor, 0.3);


    moveit::planning_interface::MoveGroupInterface group("arm_torso");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_link");
    group.setEndEffectorLink("gripper_link");
    group.setPoseTarget(target_pose);
    group.setGoalTolerance(0.05);
    ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink()
                                        << " to a target pose expressed in " << group.getPlanningFrame());
    group.setStartStateToCurrentState();
    group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);

    // Planning path
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group.setPlanningTime(5.0);
    bool success = static_cast<bool>(group.plan(plan));

    if (!success)
    {
        ROS_ERROR("Failed to find a valid plan.");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Plan found in " << plan.planning_time_ << " seconds");

    // Execution path
    ros::Time start_time = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode execution_result = group.move();
    if (!static_cast<bool>(execution_result))
    {
        ROS_ERROR("Execution failed.");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_time).toSec());
    return EXIT_SUCCESS;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drop");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Set target and lift position
    setGoalAndLiftPositions();


    ROS_INFO("Moving arm to the goal position...");
    if (moveArm(first_detach_position) != EXIT_SUCCESS)
    {
        ROS_ERROR("Failed to move to the goal position.");
        return EXIT_FAILURE;
    }
    ros::Duration(1.0).sleep();


    ROS_INFO("Opening the gripper...");
    system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
    ros::Duration(1.0).sleep();


    spinner.stop();
    ROS_INFO("Operation completed successfully.");
    return EXIT_SUCCESS;
}
