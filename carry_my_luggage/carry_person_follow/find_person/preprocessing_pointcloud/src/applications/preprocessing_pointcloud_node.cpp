#include "preprocessing_pointcloud.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "preprocessing_pointcloud");
  ros::NodeHandle nh;
  std::string pointcloud_topic_name = "/xtion/depth_registered/points";
  std::string base_fame_name = "base_footprint";

  // construct the object
  PreprocessingPointCloud preprocessing(
    pointcloud_topic_name, 
    base_fame_name);
  
  // initialize the object
  if(!preprocessing.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init preprocessing_pointcloud");
    return -1;
  }

  // update the processing
  ros::Rate rate(30);
  while(ros::ok())
  {
    preprocessing.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}