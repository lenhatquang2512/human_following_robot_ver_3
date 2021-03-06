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

typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  /*** for classification ***/
//   int number_points;
//   float min_distance;
//   Eigen::Matrix3f covariance_3d;
//   Eigen::Matrix3f moment_3d;
  // float partial_covariance_2d[9];
  // float histogram_main_2d[98];
  // float histogram_second_2d[45];
//   float slice[20];
//   float intensity[27];
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
//   std::string model_file_name_;
//   std::string range_file_name_;
//   struct svm_node *svm_node_;
//   struct svm_model *svm_model_;
//   bool use_svm_model_;
//   bool is_probability_model_;
//   float svm_scale_range_[FEATURE_SIZE][2];
  float x_lower_;
  float x_upper_;
  float human_probability_;
  bool human_size_limit_;
  
public:
  Object3dDetector();
//   ~Object3dDetector();
  
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  
  void extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
		      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid);
//   void saveFeature(Feature &f, struct svm_node *x);
  void classify();
};

Object3dDetector::Object3dDetector() {
  point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, &Object3dDetector::pointCloudCallback, this);
  
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
  
  /****** load a pre-trained svm model ******/
//   private_nh.param<std::string>("model_file_name", model_file_name_, "");
//   private_nh.param<std::string>("range_file_name", range_file_name_, "");
  
//   use_svm_model_ = false;
//   if((svm_model_ = svm_load_model(model_file_name_.c_str())) == NULL) {
//     ROS_WARN("[object3d detector] can not load SVM model, use model-free detection.");
//   } else {
//     ROS_INFO("[object3d detector] load SVM model from '%s'.", model_file_name_.c_str());
//     is_probability_model_ = svm_check_probability_model(svm_model_)?true:false;
//     svm_node_ = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
    
//     // load range file, for more details: https://github.com/cjlin1/libsvm/
//     std::fstream range_file;
//     range_file.open(range_file_name_.c_str(), std::fstream::in);
//     if(!range_file.is_open()) {
//       ROS_WARN("[object3d detector] can not load range file, use model-free detection.");
//     } else {
//       ROS_INFO("[object3d detector] load SVM range from '%s'.", range_file_name_.c_str());
//       std::string line;
//       std::vector<std::string> params;
//       std::getline(range_file, line);
//       std::getline(range_file, line);
//       boost::split(params, line, boost::is_any_of(" "));
//       x_lower_ = atof(params[0].c_str());
//       x_upper_ = atof(params[1].c_str());
//       int i = 0;
//       while(std::getline(range_file, line)) {
// 	boost::split(params, line, boost::is_any_of(" "));
// 	svm_scale_range_[i][0] = atof(params[1].c_str());
// 	svm_scale_range_[i][1] = atof(params[2].c_str());
// 	i++;
// 	//std::cerr << i << " " <<  svm_scale_range_[i][0] << " " << svm_scale_range_[i][1] << std::endl;
//       }
//       use_svm_model_ = true;
//     }
//   }
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
int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // for more details, see our IROS'17 paper.
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
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
      	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      	  cluster->points.push_back(pc->points[*pit]);
      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;
	
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
	extractFeature(cluster, f, min, max, centroid);
	features_.push_back(f);
      }
    }
  }
}

void Object3dDetector::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
				      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid) {
  f.centroid = centroid;
  f.min = min;
  f.max = max;
  
//   if(use_svm_model_) {
    // // f1: Number of points included the cluster.
    // f.number_points = pc->size();
    // // f2: The minimum distance to the cluster.
    // f.min_distance = FLT_MAX;
    // float d2; //squared Euclidean distance
    // for(int i = 0; i < pc->size(); i++) {
    //   d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
    //   if(f.min_distance > d2)
	// f.min_distance = d2;
    // }
    //f.min_distance = sqrt(f.min_distance);
    
    // pcl::PCA<pcl::PointXYZI> pca;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZI>);
    // pca.setInputCloud(pc);
    // pca.project(*pc, *pc_projected);
    // f3: 3D covariance matrix of the cluster.
    // pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
    // f4: The normalized moment of inertia tensor.
    // computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
    // Navarro et al. assume that a pedestrian is in an upright position.
    //pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
    // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
    //compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
    // f6 and f7
    //computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
    //computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
    // f8
    // computeSlice(pc, 10, f.slice);
    // // f9
    // computeIntensity(pc, 25, f.intensity);
//   }
}

void Object3dDetector::classify() {
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseArray pose_array;
  
  for(std::vector<Feature>::iterator it = features_.begin(); it != features_.end(); ++it) {
    // if(use_svm_model_) {
    //   saveFeature(*it, svm_node_);
    //   //std::cerr << "test_id = " << it->id << ", number_points = " << it->number_points << ", min_distance = " << it->min_distance << std::endl;
      
    //   // scale data
    //   for(int i = 0; i < FEATURE_SIZE; i++) {
    //   	if(svm_scale_range_[i][0] == svm_scale_range_[i][1]) // skip single-valued attribute
    //   	  continue;
    //   	if(svm_node_[i].value == svm_scale_range_[i][0])
    //   	  svm_node_[i].value = x_lower_;
    //   	else if(svm_node_[i].value == svm_scale_range_[i][1])
    //   	  svm_node_[i].value = x_upper_;
    //   	else
    //   	  svm_node_[i].value = x_lower_ + (x_upper_ - x_lower_) * (svm_node_[i].value - svm_scale_range_[i][0]) / (svm_scale_range_[i][1] - svm_scale_range_[i][0]);
    //   }
      
    //   // predict
    //   if(is_probability_model_) {
	// double prob_estimates[svm_model_->nr_class];
    //   	svm_predict_probability(svm_model_, svm_node_, prob_estimates);
	// if(prob_estimates[0] < human_probability_)
	//   continue;
    //   } else {
    //   	if(svm_predict(svm_model_, svm_node_) != 1)
    //   	  continue;
    //   }
    // }
    
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
    // if(!use_svm_model_) {
    //   marker.color.r = 0.0;
    //   marker.color.g = 0.5;
    //   marker.color.b = 1.0;
    // } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
    // }
    
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
    
    geometry_msgs::Pose pose;
    pose.position.x = it->centroid[0];
    pose.position.y = it->centroid[1];
    pose.position.z = it->centroid[2];
    pose.orientation.w = 1;
    pose_array.poses.push_back(pose);
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