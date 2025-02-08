// ROS headers
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GraspPlanning.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PickupGoal.h>
#include <tf/transform_broadcaster.h>
#include <teleop_tools_msgs/IncrementActionGoal.h>
#include <tf/transform_datatypes.h>  // Euler angle conversion
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


// Std C++ headers
#include <string>
#include <vector>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>





using namespace std;

geometry_msgs::PoseStamped closing_in, lift;
double z2, z3;

// Callback function to subscribe to the target pose
void poseCallback(const visualization_msgs::MarkerArray& msg) 
{
    if (msg.markers.empty()) 
    {
        ROS_WARN("Received empty MarkerArray!");
        return;
    }

    double x1, y1, z1, roll, pitch, yaw;
    ros::NodeHandle nh;
    
    // Read positional parameters from YAML
    nh.param("closing_in/position/x", x1, -0.3);
    nh.param("closing_in/position/y", y1, 0.0);
    nh.param("closing_in/position/z", z1, -0.15);
    nh.param("grasp_adjustment/position/z", z2, 0.2);
    nh.param("lift_adjustment/position/z", z3, 0.5);


    // Read Euler Angle (RPY)
    nh.param("closing_in/orientation/roll", roll, -90.0);
    nh.param("closing_in/orientation/pitch", pitch, 0.0);
    nh.param("closing_in/orientation/yaw", yaw, 180.0);

    // Angle to radian conversion
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    // Convert Euler angles to quaternions
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(q, quat);



    // Setting the capture location
    closing_in.header.frame_id = "base_link";
    closing_in.pose.position.x = msg.markers[0].pose.position.x + x1;
    closing_in.pose.position.y = msg.markers[0].pose.position.y + y1;
    closing_in.pose.position.z = msg.markers[0].pose.position.z + z1;
    closing_in.pose.orientation = quat; 
}

// Functions to open the gripper
// trajectory_msgs::JointTrajectory openGripper()
// {
//     trajectory_msgs::JointTrajectory posture;
//     posture.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = {1.0, 1.0}; // open
//     point.time_from_start = ros::Duration(1.0);
//     posture.points.push_back(point);
//     return posture;
// }

// Functions to close the gripper
// trajectory_msgs::JointTrajectory closedGripper()
// {
//     trajectory_msgs::JointTrajectory posture;
//     posture.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = {0.0, 0.0}; // close
//     point.time_from_start = ros::Duration(1.0);
//     posture.points.push_back(point);
//     return posture;
// }


// Function that controls the movement of the robot arm to the target position
int moveArmToTarget(const geometry_msgs::PoseStamped& target)
{   

    double max_velocity_scaling_factor;
    std::string planner;
    ros::NodeHandle nh;
    nh.param("max_velocity_scaling_factor_grasp", max_velocity_scaling_factor, 0.3);
    nh.param<std::string>("PLANNER", planner, "RRTConnectkConfigDefault"); 


    moveit::planning_interface::MoveGroupInterface group("arm"); //arm_torso
    group.setPlannerId(planner);
    group.setPoseReferenceFrame("base_link");
    group.setEndEffectorLink("gripper_link");
    group.setGoalTolerance(0.02);
    group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    group.setPlanningTime(10.0);
    group.setStartStateToCurrentState();
    group.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) 
    {
        ROS_ERROR("Planning failed for target pose.");
        return EXIT_FAILURE;
    }

    if (group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) 
    {
        ROS_ERROR("Failed to execute the planned motion.");
        return EXIT_FAILURE;
    }

    ROS_INFO("Motion executed successfully.");
    return EXIT_SUCCESS;
}


// Functions: torso control 
bool adjustTorsoHeight(double increment)
{
    ros::NodeHandle nh;
    ros::Publisher torsoPub = nh.advertise<teleop_tools_msgs::IncrementActionGoal>("/torso_controller/increment/goal", 1);
    ros::Duration(0.5).sleep(); 

    if (torsoPub.getNumSubscribers() == 0)
    {
        ROS_ERROR("No subscribers on /torso_controller/increment/goal!");
        return false;
    }

    teleop_tools_msgs::IncrementActionGoal msg;
    msg.goal.increment_by.push_back(increment);
    ROS_INFO_STREAM("Adjusting torso height by " << increment << " meters...");
    torsoPub.publish(msg);

    ros::Duration(1.0).sleep(); 
    return true;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Subscribe to the target position
    ros::Subscriber goalPoseSub = nh.subscribe("/text_markers", 1, poseCallback);

    ros::Publisher gripperPub = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);


    // open gripper
    system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
    ros::Duration(1.0).sleep();

    ROS_INFO("Waiting for target poses...");
    ros::Rate loop_rate(10);
    while (ros::ok() && closing_in.pose.position.x == 0.0) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    ROS_INFO("Moving to grasping position...");
    if (moveArmToTarget(closing_in) != EXIT_SUCCESS) return EXIT_FAILURE;
    ros::Duration(1.0).sleep();


    ROS_INFO("Moving to attach position...");
    if (adjustTorsoHeight(z2)!= EXIT_SUCCESS) return EXIT_FAILURE;
    ros::Duration(1.0).sleep();

    // Closing the gripper to grasp the object
    // ROS_INFO("Closing gripper to grasp object...");
    // system("rosservice call /parallel_gripper_controller/grasp");
    // ros::Duration(1.0).sleep();

    // Lift the object, adjust the height of the torso
    // ROS_INFO("Lifting the object...");
    // if (adjustTorsoHeight(z3)!= EXIT_SUCCESS) return EXIT_FAILURE;
    // ros::Duration(1.0).sleep();

      
    

    spinner.stop();
    ROS_INFO("Grasping operation completed.");
    return EXIT_SUCCESS;
}
