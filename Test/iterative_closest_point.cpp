#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>
#include <pcl/filters/statistical_outlier_removal.h>
clock_t start,end;

int main (int argc, char** argv)
{
	
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // 读取点云数据
  // pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_passthrough_filtered_in.pcd", *cloud_in);
  // pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_passthrough_filtered_out.pcd", *cloud_out);

  // 读取点云数据
  pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_in.pcd", *cloud_in);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_passthrough_filtered_out.pcd", *cloud_out);
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);   // 设置输入点云
  sor.setMeanK(50);           // 设置平均值估计所需的点数
  sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
  sor.filter(*cloud_in_filtered); // 执行滤波操作
  
  pcl::io::savePCDFileASCII("cloud_in_filtered.pcd", *cloud_in_filtered);
  
  sor.setInputCloud(cloud_out);   // 设置输入点云
  sor.setMeanK(50);           // 设置平均值估计所需的点数
  sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
  sor.filter(*cloud_out_filtered); // 执行滤波操作
  
  pcl::io::savePCDFileASCII("cloud_out_filtered.pcd", *cloud_out_filtered);


  // 输出标准点云
  // std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
  //for (auto& point : *cloud_in)
  // std::cout << point << std::endl;
  // 输出标准点云
  //std::cout << "Transformed " << cloud_out->size () << " data points to input:" << std::endl;
  //for (auto& point : *cloud_out)
  // std::cout << point << std::endl;

  /*
  // 创建ICP对象
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in_filtered);
  icp.setInputTarget(cloud_out_filtered);

  // 设置ICP参数
  icp.setMaxCorrespondenceDistance(10);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.1);
  icp.setMaximumIterations (1000);
  start=clock();
  // 执行ICP配准

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  // 保存配准后点云数据
  pcl::io::savePCDFileASCII("cloud_icp.pcd", Final);
  //
  //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  //icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  // 获取变换矩阵
  Eigen::Matrix4f transform = icp.getFinalTransformation();


  // 获取旋转矩阵和平移矩阵
  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  Eigen::Vector3f translation = transform.block<3, 1>(0, 3);

  // 打印旋转矩阵和平移矩阵
  std::cout << "Rotation matrix:" << std::endl << rotation << std::endl;
  std::cout << "Translation vector:" << std::endl << translation << std::endl;
  

  // 获取并打印四元数
  Eigen::Quaternionf quaternion(rotation);
  std::cout << "Quaternion'coeffs is:" << std::endl << quaternion.coeffs() <<std::endl;
  std::cout << "Quaternion'vec is:" << std::endl << quaternion.vec() <<std::endl;
  end = clock();
  
  double endtime=(double)(end-start)/CLOCKS_PER_SEC;
  std::cout<<"匹配与转换用时:"<<endtime<<std::endl;		//s为单位
  */



  return (0);

}



