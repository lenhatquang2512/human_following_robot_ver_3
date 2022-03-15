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
  double min_percentage_;

public:
  Ransac_3D(void);
  // ~Ransac_3D(void);
  double point2planedistnace(pcl::PointXYZI pt, pcl::ModelCoefficients::Ptr coefficients);
  void ransacSegment(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg);
};

Ransac_3D::Ransac_3D(void):
  nh_("~"),
  sac_model_number_(0),
  sac_method_type_(0),
  distance_threshold_(0.01),
  // max_iterations_(100),
  optimization_coeff_(true),
  min_percentage_(5)
{
  // subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_clouded", 1, &Ransac_3D::ransacSegment,this);
  subLaserCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Ransac_3D::ransacSegment,this);

  ros::NodeHandle private_nh("~");
  pubNewLaserCloud_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 100);
  
  //parameters
  private_nh.param<int>("Sac_model_number",sac_model_number_,0); //pcl::SACMODEL_PLANE
  private_nh.param<int>("Sac_method_type",sac_method_type_,0); // pcl::SAC_RANSAC
  private_nh.param<double>("distance_threshold",distance_threshold_,0.01);
  // private_nh.param<int>("max_iteration",max_iterations_,100);
  private_nh.param<bool>("optimization_coeff",optimization_coeff_,true);
  private_nh.param<double>("min_percentage",min_percentage_,5.00);
}

double Ransac_3D::point2planedistnace(pcl::PointXYZI pt, pcl::ModelCoefficients::Ptr coefficients)
{
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
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
    // segmenter.setMaxIterations(max_iterations_);


    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZI>);
    int original_size(laserCloudIn->height * laserCloudIn->width);
    int n_planes(0);

    while (laserCloudIn->height*laserCloudIn->width > (original_size*min_percentage_/100))
    {

        // set input cloud and output indices and coeff
        //fitting a plane
        segmenter.setInputCloud(laserCloudIn);
        segmenter.segment (*inliers_seg, *coefficients);

        //check result
        if (inliers_seg->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        //iterate inliers
        double mean_error(0);
        double max_error(0);
        double min_error(100000);
        std::vector<double> err;
        for (size_t i=0;i< inliers_seg->indices.size();i++){

            // Get Point
            pcl::PointXYZI pt = laserCloudIn->points[inliers_seg->indices[i]];

            // Compute distance
            double d = point2planedistnace(pt,coefficients)*1000;// mm
            err.push_back(d);

            // Update statistics
            mean_error += d;
            if (d>max_error) max_error = d;
            if (d<min_error) min_error = d;
        }
        mean_error /= inliers_seg->indices.size();

        // Compute Standard deviation
        double sigma(0);
        for (size_t i=0;i<inliers_seg->indices.size();i++){

            sigma += pow(err[i] - mean_error,2);

            // Get Point
            pcl::PointXYZI pt = laserCloudIn->points[inliers_seg->indices[i]];

            cloud_pub->points.push_back(pt);

        }
        sigma = sqrt(sigma/inliers_seg->indices.size());

        // Extract road point cloud using indices
        extract_seg.setInputCloud(laserCloudIn);
        extract_seg.setIndices(inliers_seg);
        extract_seg.setNegative(true);
        // Declare output clouds
        pcl::PointCloud<pcl::PointXYZI> road_cloud;
        extract_seg.filter(road_cloud);
        laserCloudIn->swap(road_cloud);

         // Display infor
        ROS_INFO("fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
                 n_planes,
                 coefficients->values[0],(coefficients->values[1]>=0?"+":""),
                 coefficients->values[1],(coefficients->values[2]>=0?"+":""),
                 coefficients->values[2],(coefficients->values[3]>=0?"+":""),
                 coefficients->values[3],
                 inliers_seg->indices.size(),original_size);
        ROS_INFO("mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",mean_error,sigma,max_error);
        ROS_INFO("poitns left in cloud %i", laserCloudIn->width* laserCloudIn->height);

        // Nest iteration
        n_planes++;
    }

    //publish point cloud
    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*cloud_pub, laserCloudTemp);
    // laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header = laserCloudMsg->header;
    laserCloudTemp.header.frame_id = "velodyne";
    pubNewLaserCloud_.publish(laserCloudTemp);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac");

    Ransac_3D ransac_obj;
    ros::spin();
    return 0;
}