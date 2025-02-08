#include "preprocessing_pointcloud.h"
PreprocessingPointCloud::PreprocessingPointCloud(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false)
{
}

PreprocessingPointCloud::~PreprocessingPointCloud()
{
}

bool PreprocessingPointCloud::initalize(ros::NodeHandle& nh)
{
  // load rosparams
  point_cloud_sub_ = nh.subscribe(pointcloud_topic_, 10, &PreprocessingPointCloud::cloudCallback, this);
  preprocessed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/preprocessed_cloud", 10);

  // Most PCL functions accept pointers as their arguments, as such we first set
  // initalize these pointers, otherwise we will run into segmentation faults...
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);

  return true;
}

void PreprocessingPointCloud::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;

    //#>>>>Note: To check preProcessCloud() you can publish its output for testing
    // apply all preprocessing steps
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_))
      return;
    sensor_msgs::PointCloud2 preprocessed_cloud_msg;
    pcl::toROSMsg(*preprocessed_cloud_, preprocessed_cloud_msg);

    preprocessed_cloud_pub_.publish(preprocessed_cloud_msg);
  }
}

bool PreprocessingPointCloud::preProcessCloud(CloudPtr& input, CloudPtr& output)
{

  CloudPtr ds_cloud(new PointCloud);            // downsampled pointcloud

  // create voxel grid filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*ds_cloud);

  CloudPtr transf_cloud(new PointCloud);        // transformed pointcloud (expressed in base frame)

  // Transform the point cloud to the base_frame link.
  pcl_ros::transformPointCloud(base_frame_, *ds_cloud, *transf_cloud, tfListener_);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.1, 3);
  pass.filter(*output);
  return true;
}

void PreprocessingPointCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert ros msg to pcl raw_cloud
  is_cloud_updated_ = true;
  pcl::fromROSMsg(*msg, *raw_cloud_);
}