#ifndef TIAGO_LOCALIZATION_H_
#define TIAGO_LOCALIZATION_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <queue>

namespace tiago_localization
{

  class Localizer
  {
    private:

      // ros connections
      ros::Publisher vel_pub_;

      ros::Subscriber pose_sub_;

      ros::ServiceClient localization_client_;


    public:
      Localizer();

      virtual ~Localizer();
      
      double covariance_sum;

      double speed;

      double threshold;
      
      bool initialize(ros::NodeHandle& nh);

      void initialize_localization(ros::NodeHandle& nh);

      void robot_spin(double speed);

      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  };
}

#endif