#ifndef PLANE_SEGMENTATION_CLASS_H
#define PLANE_SEGMENTATION_CLASS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>

/**
 * @brief PlaneSegementation class, splits RGB-D pointclouds into table surface
 * and objects pointcloud. Publishes both new pointlcouds as ros topics
 * 
 */
class PreprocessingPointCloud
{
public:
  // pcl pointcloud types
  typedef pcl::PointXYZRGB PointT;                // The Point Type
  typedef pcl::PointCloud<PointT> PointCloud;     // The PointCloud Type
  typedef PointCloud::Ptr CloudPtr;               // The PointCloud Pointer Type

public:
  /**
   * @brief Construct a new PreprocessingPointCloud object
   * 
   * @param pointcloud_topic topic of point cloud
   * @param base_frame frame on the ground of the robot (to remove floor points)
   */
    PreprocessingPointCloud(const std::string &pointcloud_topic = "",
                    const std::string &base_frame = "base_link");

  /**
   * @brief Destroy the Plane Segmentation object
   * 
   */
  ~PreprocessingPointCloud();

  /**
   * @brief initalize the all ros subsribers/publishers, member variables
   * 
   * @param nh 
   * @return true success
   * @return false failure
   */
  bool initalize(ros::NodeHandle &nh);

  /**
   * @brief called periodically, update the plane segmentation object
   * 
   * @param time current time
   */
  void update(const ros::Time &time);

private:
  /**
   * @brief apply preprocessing to input cloud and return output cloud
   * 
   * @param input inital cloud
   * @param output preprocessed cloud
   * @return true success
   * @return false failure
   */
  bool preProcessCloud(CloudPtr& input, CloudPtr& output);

private:
  /**
   * @brief callback function for new pointcloud subscriber
   * 
   * @param msg 
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
  bool is_cloud_updated_;             //!< new pointcloud recived
  std::string base_frame_;            //!< robot base frame
  std::string pointcloud_topic_;      //!< pointcloud topic name

  ros::Subscriber point_cloud_sub_;   //!< Subscribers to the PointCloud data

  ros::Publisher preprocessed_cloud_pub_;    //!< Publish table point cloud

  // internal pointclouds
  CloudPtr raw_cloud_;                  //!< Inital raw point cloud
  CloudPtr preprocessed_cloud_;         //!< after preprocessing

  // transformation
  tf::TransformListener tfListener_;    //!< access ros tf tree to get frame transformations
};

#endif