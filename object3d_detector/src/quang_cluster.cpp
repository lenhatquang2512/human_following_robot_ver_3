// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

//still pcl, but extra libraries
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>

#include<pcl/common/impl/io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <string>


typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
} Feature;

class Object3dDetector {
private:
  /*** Publishers and Subscribers ***/
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher marker_array_pub_;
  
  bool print_fps_;
  std::string frame_id_;
  float z_limit_min_;
  float z_limit_max_;
  int cluster_size_min_;
  int cluster_size_max_;
  
  std::vector<Feature> features_;
  float x_lower_;
  float x_upper_;
  float human_probability_;
  bool human_size_limit_;
  
public:
  Object3dDetector();
  
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  
  void extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
		      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid);
  void classify();
};

Object3dDetector::Object3dDetector() {
  // point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, &Object3dDetector::pointCloudCallback, this);
  point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/ground_cloud", 1, &Object3dDetector::pointCloudCallback, this);
  
  ros::NodeHandle private_nh("~");
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
  pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  
  /*** Parameters ***/
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<std::string>("frame_id", frame_id_, "velodyne");
  private_nh.param<float>("z_limit_min", z_limit_min_, -0.8);
  private_nh.param<float>("z_limit_max", z_limit_max_, 1.2);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 5);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 30000);
  private_nh.param<float>("human_probability", human_probability_, 0.7);
  private_nh.param<bool>("human_size_limit", human_size_limit_, false);
  
}

int frames; clock_t start_time; bool reset = true;//fps
void Object3dDetector::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  extractCluster(pcl_pc);
  classify();
  
  if(print_fps_)if(++frames>10){std::cerr<<"[object3d_detector]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps
}

const int nested_regions_ = 14;
int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // IROS'17 paper.
void Object3dDetector::extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  features_.clear();
  
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_limit_min_, z_limit_max_);
  pass.filter(*pc_indices);
  
  boost::array<std::vector<int>, nested_regions_> indices_array;
  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < nested_regions_; j++) {
      float d2 = pc->points[(*pc_indices)[i]].x * pc->points[(*pc_indices)[i]].x +
	pc->points[(*pc_indices)[i]].y * pc->points[(*pc_indices)[i]].y +
	pc->points[(*pc_indices)[i]].z * pc->points[(*pc_indices)[i]].z;
      if(d2 > range*range && d2 <= (range+zone_[j])*(range+zone_[j])) {
      	indices_array[j].push_back((*pc_indices)[i]);
      	break;
      }
      range += zone_[j];
    }
  }
  
  float tolerance = 0.0;
  for(int i = 0; i < nested_regions_; i++) {
    tolerance += 0.1;
    if(indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pc, indices_array_ptr);
      // tree->setInputCloud(pc);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) 
      {
      	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
           cluster->points.push_back(pc->points[*pit]);
        }

      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;

        // Print out all cluster
        // for (const auto& point: *cluster) {
        //   std::cout << "    " << point.x << " " << point.y << " " << point.z << " " << point.intensity <<  std::endl;
        // }
	
      	Eigen::Vector4f min, max, centroid;
      	pcl::getMinMax3D(*cluster, min, max);
      	pcl::compute3DCentroid(*cluster, centroid);
	
      	// Size limitation is not reasonable, but it can increase fps.
      	if(human_size_limit_ &&
	   (max[0]-min[0] < 0.2 || max[0]-min[0] > 1.0 ||
	    max[1]-min[1] < 0.2 || max[1]-min[1] > 1.0 ||
	    max[2]-min[2] < 0.5 || max[2]-min[2] > 2.0)) 
	  continue;
	
	Feature f;

  // std::cout << "    " << centroid <<  std::endl;

	extractFeature(cluster, f, min, max, centroid);

  // std::cout <<  f.centroid << " " << f.max << " " << f.min <<  std::endl;
	features_.push_back(f);
      }
    }
  }
}

int count = 0;
void Object3dDetector::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
				      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid) {
  f.centroid = centroid;
  f.min = min;
  f.max = max;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize (pc->size());
  
  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
      (*cloud)[i].x = (*pc)[i].x;
      (*cloud)[i].y = (*pc)[i].y;
      (*cloud)[i].z = (*pc)[i].z;
  } 
  
  std::string filename;
  std::string default_s = "cluster_box_";
  std::string end = ".pcd";
  std::string s = std::to_string(count);
  filename = default_s + s + end;
  count++;
  // for (const auto& point: *cloud) {
  //   std::cout << "    " << point.x << " " << point.y << " " << point.z << " " <<  std::endl;
  // }

  if(count <= 50)
  {
    pcl::io::savePCDFile(filename, *cloud);//Save the point cloud to a PCD file
    std::cout << " OK "<< std::endl;

    // //-------------visualization-----------------------------
    // pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // // Define R,G,B colors for the point cloud
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
    // // We add the point cloud to the viewer and pass the color handler
    // viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

    // // viewer.addCoordinateSystem (1.0, "cloud", 0);
    // // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    // // Display the visualiser until 'q' key is pressed
    // while (!viewer.wasStopped ()) 
    // { 
    //     viewer.spinOnce ();
    // }
  }

  //end wrtiting to pcd files

}

void Object3dDetector::classify() {
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseArray pose_array;
  
  for(std::vector<Feature>::iterator it = features_.begin(); it != features_.end(); ++it) {
    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.ns = "object3d";
    marker.id = it-features_.begin();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point p[24];
    p[0].x = it->max[0]; p[0].y = it->max[1]; p[0].z = it->max[2];
    p[1].x = it->min[0]; p[1].y = it->max[1]; p[1].z = it->max[2];
    p[2].x = it->max[0]; p[2].y = it->max[1]; p[2].z = it->max[2];
    p[3].x = it->max[0]; p[3].y = it->min[1]; p[3].z = it->max[2];
    p[4].x = it->max[0]; p[4].y = it->max[1]; p[4].z = it->max[2];
    p[5].x = it->max[0]; p[5].y = it->max[1]; p[5].z = it->min[2];
    p[6].x = it->min[0]; p[6].y = it->min[1]; p[6].z = it->min[2];
    p[7].x = it->max[0]; p[7].y = it->min[1]; p[7].z = it->min[2];
    p[8].x = it->min[0]; p[8].y = it->min[1]; p[8].z = it->min[2];
    p[9].x = it->min[0]; p[9].y = it->max[1]; p[9].z = it->min[2];
    p[10].x = it->min[0]; p[10].y = it->min[1]; p[10].z = it->min[2];
    p[11].x = it->min[0]; p[11].y = it->min[1]; p[11].z = it->max[2];
    p[12].x = it->min[0]; p[12].y = it->max[1]; p[12].z = it->max[2];
    p[13].x = it->min[0]; p[13].y = it->max[1]; p[13].z = it->min[2];
    p[14].x = it->min[0]; p[14].y = it->max[1]; p[14].z = it->max[2];
    p[15].x = it->min[0]; p[15].y = it->min[1]; p[15].z = it->max[2];
    p[16].x = it->max[0]; p[16].y = it->min[1]; p[16].z = it->max[2];
    p[17].x = it->max[0]; p[17].y = it->min[1]; p[17].z = it->min[2];
    p[18].x = it->max[0]; p[18].y = it->min[1]; p[18].z = it->max[2];
    p[19].x = it->min[0]; p[19].y = it->min[1]; p[19].z = it->max[2];
    p[20].x = it->max[0]; p[20].y = it->max[1]; p[20].z = it->min[2];
    p[21].x = it->min[0]; p[21].y = it->max[1]; p[21].z = it->min[2];
    p[22].x = it->max[0]; p[22].y = it->max[1]; p[22].z = it->min[2];
    p[23].x = it->max[0]; p[23].y = it->min[1]; p[23].z = it->min[2];
    for(int i = 0; i < 24; i++)
      marker.points.push_back(p[i]);
    marker.scale.x = 0.02;
    marker.color.a = 1.0;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
    
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
    
    geometry_msgs::Pose pose;
    pose.position.x = it->centroid[0];
    pose.position.y = it->centroid[1];
    pose.position.z = it->centroid[2];
    pose.orientation.w = 1;
    pose_array.poses.push_back(pose);

    std::cout << "Coordinate of all boxes is : " ;
    std::cout << "x = "<< pose.position.x << " y = " << pose.position.y << " z = " << pose.position.z << std::endl;

  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
  if(pose_array.poses.size()) {
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id_;
    pose_array_pub_.publish(pose_array);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object3d_detector");
  Object3dDetector d;
  ros::spin();
  return 0;
}