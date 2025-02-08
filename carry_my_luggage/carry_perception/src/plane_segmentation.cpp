#include <carry_perception/plane_segmentation.h>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false)
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  // load rosparams
  std::vector<float> pre_pass_limits; 
  std::vector<float> seg_pass_limits; 
  if (!ros::param::get("pre_pass_filter", pre_pass_limits))
  {
    return false;
  }
  if (!ros::param::get("seg_pass_filter", seg_pass_limits))
  {
    return false;
  }
  if (!ros::param::get("ransac_threshold", ransac_thresh_))
  {
    return false;
  }
  pre_pass_low_ = pre_pass_limits[0];
  pre_pass_high_ = pre_pass_limits[1];
  seg_pass_low_ = seg_pass_limits[0];
  seg_pass_high_ = seg_pass_limits[1];

  //#>>>>TODO: subscribe to the pointcloud_topic_ and link it to the right callback
  point_cloud_sub_ = nh.subscribe(pointcloud_topic_, 10, &PlaneSegmentation::cloudCallback, this);

  //#>>>>TODO: advertise the pointcloud for of the table plane
  plane_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/table_point_cloud", 10);

  //#>>>>TODO: advertise the pointcloud for the remaining points (objects)
  objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/objects_point_cloud", 10);

  plane_vertex_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/table_vertices", 10);

  // Most PCL functions accept pointers as their arguments, as such we first set
  // initalize these pointers, otherwise we will run into segmentation faults...
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  return true;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;

    //#>>>>Note: To check preProcessCloud() you can publish its output for testing
    // apply all preprocessing steps
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_))
      return;

    // segment cloud into table and objects
    if(!segmentCloud(preprocessed_cloud_, plane_cloud_, objects_cloud_))
      return;


    //#>>>>TODO: publish both pointclouds obtained by segmentCloud()

    sensor_msgs::PointCloud2 plane_cloud_msg;
    sensor_msgs::PointCloud2 objects_cloud_msg;
    pcl::toROSMsg(*plane_cloud_, plane_cloud_msg);
    pcl::toROSMsg(*objects_cloud_, objects_cloud_msg);

    plane_cloud_pub_.publish(plane_cloud_msg);
    objects_cloud_pub_.publish(objects_cloud_msg);

  }
}

bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  //#>>>>Goal: Subsample and Filter the pointcloud

  //#>>>>Note: Raw pointclouds are typically to dense and need to be made sparse
  //#>>>>TODO: Down sample the pointcloud using VoxelGrid, save result in ds_cloud
  //#>>>>Hint: See https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#voxelgrid 
  //#>>>>TODO: Set useful parameters

  CloudPtr ds_cloud(new PointCloud);            // downsampled pointcloud

  // create voxel grid filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*ds_cloud);

  // std::cout << "PointCloud after filtering: " << ds_cloud->width * ds_cloud->height 
  //     << " data points (" << pcl::getFieldsList (*ds_cloud) << ")." << std::endl;
  
  //#>>>>Note: Its allways a good idea to get rid of useless points first (e.g. floor, ceiling, walls, etc.)
  //#>>>>TODO: Transform the point cloud to the base_frame of the robot. (A frame with z=0 at ground level)
  //#>>>>TODO: Transform the point cloud to the base_frame and store the result in transf_cloud
  //#>>>>Hint: use pcl_ros::transformPointCloud

  CloudPtr transf_cloud(new PointCloud);        // transformed pointcloud (expressed in base frame)

  // Transform the point cloud to the base_frame link.
  pcl_ros::transformPointCloud(base_frame_, *ds_cloud, *transf_cloud, tfListener_);


  //#>>>>TODO: Trim points lower than some z_min to remove the floor from the point cloud.
  //#>>>>Hint: use pcl::PassThrough filter and save result in output
  //#>>>>Hint: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough

  // create pass through filtering object 
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(pre_pass_low_, pre_pass_high_);
  pass.filter(*output);

  // std::cout << "PointCloud after pass: " << output->width * output->height 
  //     << " data points (" << pcl::getFieldsList (*output) << ")." << std::endl;

  return true;
}

bool PlaneSegmentation::segmentCloud(CloudPtr& input, CloudPtr& plane_cloud, CloudPtr& objects_cloud)
{
  //#>>>>Goal: Remove every point that is not an object from the objects_cloud cloud

  // We will use Ransac to segment the pointcloud, here we setup the objects we need for this
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  //#>>>>TODO: set parameters of the SACS segmentation
  //#>>>>TODO: set correct model, play with DistanceThreshold and the free outlier probability
  //#>>>>TODO: then segment the input point cloud
  //#>>>>Note: Checkout the pcl tutorials on plane segmentation
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ransac_thresh_);
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);

  //#>>>>TODO: save inliers in plane_cloud 
  //#>>>>Note: These sould be point that belong to the table 
  extract.setNegative(false); // extract inliers
  extract.filter(*plane_cloud);

  // obtain the corner coordinates of the plane
  pcl::ConvexHull<PointT> chull;
  chull.setInputCloud(plane_cloud);
  pcl::PointCloud<PointT>::Ptr chull_points(new pcl::PointCloud<PointT>);
  chull.reconstruct(*chull_points);

  // get the min and max points of the bounding box
  PointT min_point, max_point;
  pcl::getMinMax3D(*chull_points, min_point, max_point);

  // Create the two corner points of the bounding box
  pcl::PointCloud<PointT>::Ptr corner_points(new pcl::PointCloud<PointT>);
  corner_points->push_back(min_point);
  // corner_points->push_back(PointT(min_point.x, max_point.y, min_point.z));
  corner_points->push_back(max_point);
  // corner_points->push_back(PointT(max_point.x, min_point.y, max_point.z));

  // display the corners of the plane
  sensor_msgs::PointCloud2 corners_msg;
  pcl::toROSMsg(*corner_points, corners_msg);
  corners_msg.header.frame_id = "base_footprint";  // Set the frame_id as appropriate
  plane_vertex_pub_.publish(corners_msg);

  //#>>>>TODO: save outliers in the objects_cloud
  //#>>>>Note: This should be the rest
  extract.setNegative(true); // extract outliers
  extract.filter(*objects_cloud);
  
  // Next, we further refine the the objects_cloud by transforming it into the coordinate frame
  // of the fitted plane. In this transformed frame we remove everything below the table plane and 
  // everything more than 20 cm above the table.
  // Basically, a table aligned bounding box

  // if the plane fit is correct it will result in the coefficients = [nx, ny, nz, d]
  // where n = [nx, ny, nz] is the 3d normal vector perpendicular to the plane
  // and d the distance to the origin
  if(coefficients->values.empty())
    return false;

  //#>>>>TODO: extract the normal vector 'n' perpendicular to the plane and the scalar distance 'd'
  //#>>>>TODO: to the origin from the plane coefficions.
  //#>>>>Note: As always we use Eigen to represent vectors and matices 
  //#>>>>Note: https://eigen.tuxfamily.org/dox/GettingStarted.html
  Eigen::Vector3f n{coefficients->values[0], coefficients->values[1], coefficients->values[2]}; // = ?
  double d{coefficients->values[3]}; // = ?
  
  // Now we construct an Eigen::Affine3f transformation T_plane_base that describes the table plane 
  // with respect to the base_link frame using n and d

  //#>>>>TODO: Build the Rotation (Quaterion) from the table's normal vector n
  //#>>>>TODO: And the floor (world) normal vector: [0,0,1]
  //#>>>>Hint: Use Eigen::Quaternionf::FromTwoVectors()
  Eigen::Quaternionf Q_plane_base = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f{0.0, 0.0, 1.0}); // = ?

  //#>>>>TODO: Build the translation (Vector3) from the table's normal vector n and distance
  //#>>>>TODO: to the origin 
  Eigen::Vector3f t_plane_base = d * n; // = ?

  // Finally we create the Homogenous transformation of the table
  Eigen::Affine3f T_plane_base = Eigen::Affine3f::Identity();
  T_plane_base.rotate(Q_plane_base.toRotationMatrix());
  T_plane_base.translate(t_plane_base);
  
  //#>>>>TODO: Transform the objects_cloud into the table frame and store in transf_cloud
  //#>>>>Hint: Use the function pcl::transformPointCloud() with T_plane_base as input
  //#>>>>https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html
  CloudPtr transf_cloud(new PointCloud);
  pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base);
  
  //#>>>>TODO: filter everything directly below the table and above it (z > 0.01 && < 0.15) 
  //#>>>>Hint: using pcl::PassThrough filter (same as before)
  CloudPtr filterd_cloud(new PointCloud);

  // create pass through filtering object 
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(seg_pass_low_, seg_pass_high_);
  pass.filter(*filterd_cloud);

  //#>>>>TODO: transform back to base_link frame using the inverse transformation
  //#>>>>TODO: and store result in objects_cloud. Object cloud should only contain points associated to objects
  //#>>>>Hint: Eigen::Affine3f has an inverse function
  Eigen::Affine3f T_base_plane = T_plane_base.inverse();
  pcl::transformPointCloud(*filterd_cloud, *objects_cloud, T_base_plane);

  return true;
}

void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert ros msg to pcl raw_cloud
  is_cloud_updated_ = true;

  //#>>>>TODO: Convert the msg to the internal variable raw_cloud_ that holds the raw input pointcloud 
  pcl::fromROSMsg(*msg, *raw_cloud_);
}
