#include <pcl/filters/passthrough.h>
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
    //可用于可视化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云数据
    pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_in_filtered.pcd", *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_out_filtered.pcd", *cloud_out);
    start=clock();
    //设置直通滤波器 x轴平行屏幕向左
    pcl::PassThrough<pcl::PointXYZ> pass_in;
    pass_in.setInputCloud(cloud_in);     // 设置输入点云
    pass_in.setFilterFieldName("x");
    pass_in.setFilterLimits(-2.5, 3.0);
    pass_in.filter(*cloud_in_filtered);
    //y轴垂直屏幕
    pass_in.setInputCloud(cloud_in_filtered); 
    pass_in.setFilterFieldName("z");
    pass_in.setFilterLimits(9, 12);
    pass_in.filter(*cloud_in_filtered);
    //z轴平行屏幕向上
    pass_in.setInputCloud(cloud_in_filtered); 
    pass_in.setFilterFieldName("y");
    pass_in.setFilterLimits(-1.0, 0.8);
    pass_in.filter(*cloud_in_filtered);

    //设置直通滤波器 x
    pcl::PassThrough<pcl::PointXYZ> pass_out;
    pass_out.setInputCloud(cloud_out);     // 设置输入点云
    pass_out.setFilterFieldName("x");
    pass_out.setFilterLimits(-2.5, 3.0);
    pass_out.filter(*cloud_out_filtered);
    //y
    pass_out.setInputCloud(cloud_out_filtered);     // 设置输入点云
    pass_out.setFilterFieldName("z");
    pass_out.setFilterLimits(9, 12);
    pass_out.filter(*cloud_out_filtered);
    //z
    pass_out.setInputCloud(cloud_out_filtered);     // 设置输入点云
    pass_out.setFilterFieldName("y");
    pass_out.setFilterLimits(-1.0, 0.8);
    pass_out.filter(*cloud_out_filtered);

    // 保存配准后点云数据
    pcl::io::savePCDFileASCII("cloud_passthrough_filtered_in.pcd", *cloud_in_filtered);
    pcl::io::savePCDFileASCII("cloud_passthrough_filtered_out.pcd", *cloud_out_filtered);

    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in_filtered);   // 设置输入点云
    sor.setMeanK(50);           // 设置平均值估计所需的点数
    sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
    sor.filter(*cloud_in_filtered); // 执行滤波操作

    pcl::io::savePCDFileASCII("cloud_in_filtered.pcd", *cloud_in_filtered);

    sor.setInputCloud(cloud_out_filtered);   // 设置输入点云
    sor.setMeanK(50);           // 设置平均值估计所需的点数
    sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
    sor.filter(*cloud_out_filtered); // 执行滤波操作

    pcl::io::savePCDFileASCII("cloud_out_filtered.pcd", *cloud_out_filtered);



    // 创建ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //输入实时激光点云数据
    icp.setInputSource(cloud_out_filtered);

    //输入算法目标点云，即地图点云；
    icp.setInputTarget(cloud_in_filtered);

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(3);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.setMaximumIterations (1000);

  

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // 保存配准后点云数据
    pcl::io::savePCDFileASCII("cloud_icp.pcd", Final);

    // 获取变换矩阵
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    /**/
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


    return (0);
}