#include <iostream>
#include <map>
#include <path_navi_client.h>


// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <ctime>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/duration.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/distances.h>
#include <pcl/registration/gicp.h>

extern pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr target_1;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr target_2;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr target_3;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr target_4;

extern void SOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int meank, double thresh);
extern void PT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const uint axis, const double minLimits, const double maxLimits);
extern void merged(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f transform);
extern  std::vector<double> GICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
     const double TransformationEpsilon, const double EuclideanFitnessEpsilon, const int MaximumIterations, bool merged_flag);
extern void Voxel( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter);
