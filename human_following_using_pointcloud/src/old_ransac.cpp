#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>

//including ros libraries
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>
#include <thread>

//including PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>


class Ransac_3D
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber subLaserCloud_;
  ros::Publisher pubNewLaserCloud_;
  int sac_model_number_;
  int sac_method_type_;
  double distance_threshold_;
  int max_iterations_;
  bool optimization_coeff_;

public:
  Ransac_3D(void);
  // ~Ransac_3D(void);
  void ransacSegment(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg);
};

Ransac_3D::Ransac_3D(void):
  nh_("~"),
  sac_model_number_(0),
  sac_method_type_(0),
  distance_threshold_(0.01),
  max_iterations_(100),
  optimization_coeff_(true)
{
  subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_clouded", 100, &Ransac_3D::ransacSegment,this);
//   subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &Ransac_3D::ransacSegment,this);

  ros::NodeHandle private_nh("~");
  pubNewLaserCloud_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 100);
  
  //parameters
  private_nh.param<int>("Sac_model_number",sac_model_number_,0); //pcl::SACMODEL_PLANE
  private_nh.param<int>("Sac_method_type",sac_method_type_,0); // pcl::SAC_RANSAC
  private_nh.param<double>("distance_threshold",distance_threshold_,0.01);
  private_nh.param<int>("max_iteration",max_iterations_,100);
  private_nh.param<bool>("optimization_coeff",optimization_coeff_,true);
}


void Ransac_3D::ransacSegment(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
{
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg,*laserCloudIn);


    // Declare vars for segmentation
    pcl::PointIndices::Ptr inliers_seg(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> segmenter;
    pcl::ExtractIndices<pcl::PointXYZI> extract_seg;

    // Set parameters
    segmenter.setOptimizeCoefficients(optimization_coeff_);
    segmenter.setModelType(sac_model_number_);
    segmenter.setMethodType(sac_method_type_);
    segmenter.setDistanceThreshold(distance_threshold_);
    segmenter.setMaxIterations(max_iterations_);


    // set input cloud and output indices and coeff
    //fitting a plane
    segmenter.setInputCloud(laserCloudIn);
    segmenter.segment (*inliers_seg, *coefficients);

    //check result
    if (inliers_seg->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Extract road point cloud using indices
    extract_seg.setInputCloud(laserCloudIn);
    extract_seg.setIndices(inliers_seg);
    extract_seg.setNegative(true);
    // Declare output clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    extract_seg.filter(*road_cloud);

    //publish point cloud
    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*road_cloud, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    // laserCloudTemp.header = laserCloudMsg->header;
    laserCloudTemp.header.frame_id = "velodyne";
    pubNewLaserCloud_.publish(laserCloudTemp);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "old_ransac");

    Ransac_3D ransac_obj;
    ros::spin();
    return 0;
}