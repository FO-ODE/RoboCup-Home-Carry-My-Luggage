#include <carry_perception/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  std::string pointcloud_topic_name = "/xtion/depth_registered/points";


  std::string base_fame_name = "base_footprint";

  // construct the object
  PlaneSegmentation segmentation(
    pointcloud_topic_name, 
    base_fame_name);
  
  // initialize the object
  if(!segmentation.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  // update the processing
  ros::Rate rate(30);
  while(ros::ok())
  {
    segmentation.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
