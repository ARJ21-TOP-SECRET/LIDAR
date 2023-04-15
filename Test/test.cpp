#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ctime>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


clock_t start_1,end_1,start_2,end_2;

void SOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int meank, double thresh );
void PT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const uint axis,const double minLimits,const double maxLimits);
Eigen::Matrix4f ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon,const int MaximumIterations );
void merged(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f transform );
void RANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

int main (int argc, char** argv)
{
    //可用于可视化    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 读取点云数据
    pcl::io::loadPCDFile<pcl::PointXYZ> ("Cloud_data/cloud_in.pcd", *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("Cloud_data/cloud_out.pcd", *cloud_out);

    start_1 = clock();

    SOR(cloud_in, cloud_in_filtered, 50, 1.0);
    SOR(cloud_out, cloud_out_filtered, 50, 1.0);

    // 设置直通滤波器 x轴平行屏幕向左
    PT(cloud_in_filtered, 1, -2.2, 3.0);
    //z轴垂直屏幕向前
    PT(cloud_in_filtered, 3, 9, 12);
    //y轴平行屏幕向上
    PT(cloud_in_filtered, 2, -0.6, 0.8);

    // 设置直通滤波器 x轴平行屏幕向左
    PT(cloud_out_filtered, 1, -2.2, 3.0);
    // z轴垂直屏幕向前
    PT(cloud_out_filtered, 3, 9, 12);
    // y轴平行屏幕向上
    PT(cloud_out_filtered, 2, -0.6, 0.8);

    SOR(cloud_in_filtered, cloud_in_filtered, 50, 1.0);
    SOR(cloud_out_filtered, cloud_out_filtered, 50, 1.0);
    end_1 = clock();

    double endtime_1=(double)(end_1 - start_1) / CLOCKS_PER_SEC;
    std::cout<<"filter time:"<<endtime_1<<std::endl;		//s为单位

    pcl::io::savePCDFileASCII("Cloud_filtered/out_filtered.pcd", *cloud_out_filtered);
    pcl::io::savePCDFileASCII("Cloud_filtered/in_filtered.pcd", *cloud_in_filtered);
    start_2 = clock();
    merged(cloud_out_filtered, cloud_in_filtered, ICP(cloud_in_filtered, cloud_out_filtered, 3, 1e-10, 0.01, 100000)) ;
    end_2 = clock();

    double endtime_2=(double)(end_2 - start_2) / CLOCKS_PER_SEC;
    std::cout<<"icp time:"<<endtime_2<<std::endl;		

    

}

void SOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int meank, double thresh )
{

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   // 设置输入点云
    sor.setMeanK(meank);           // 设置平均值估计所需的点数
    sor.setStddevMulThresh(thresh); // 设置标准差乘数阈值
    sor.filter(*cloud_filtered); // 执行滤波操作

}

void PT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const uint axis,const double minLimits,const double maxLimits)
{

    //设置直通滤波器 x轴平行屏幕向左
    pcl::PassThrough<pcl::PointXYZ> pass_in;
    pass_in.setInputCloud(cloud);     // 设置输入点云
    // std::cout << axis << std::endl;
    switch(axis){
        case 1: pass_in.setFilterFieldName("x"); break;
        case 2: pass_in.setFilterFieldName("y"); break;
        case 3: pass_in.setFilterFieldName("z"); break;
        defult: std::cout << "axis input is wrong" << std::endl;
    }
    pass_in.setFilterLimits(minLimits, maxLimits);
    pass_in.filter(*cloud);

}

Eigen::Matrix4f ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon,const int MaximumIterations )
{
    // 创建ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //输入算法目标点云，即地图点云；
    icp.setInputTarget(cloud_in);

    //输入实时激光点云数据
    icp.setInputSource(cloud_out);

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(Distance);
    icp.setTransformationEpsilon(TransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
    icp.setMaximumIterations (MaximumIterations);

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // 保存配准后点云数据
    pcl::io::savePCDFileASCII("cloud_icp.pcd", Final);

    // 获取变换矩阵
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    // 获取旋转矩阵和平移矩阵
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    // Eigen::Vector3f translation = transform.block<3, 1>(0, 3);

    Eigen::Vector3f euler_angles = rotation.eulerAngles(0, 1, 2);
    euler_angles[0] = 0;  // 将俯仰角设为0，表示不发生变化
    euler_angles[2] = 0;  // 将翻滚角设为0，表示不发生变化
    rotation = (Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX()));
    transform.block<3, 3>(0, 0) = rotation;
    // std::cout << "transform matrix:" << std::endl << transform << std::endl;

    transform(1, 3) = 0.0;
    Eigen::Matrix4f transform_inv = transform.inverse();

    // 获取旋转矩阵和平移矩阵
    Eigen::Matrix3f rotation_inv = transform_inv.block<3, 3>(0, 0);
    Eigen::Vector3f euler_angles_inv = rotation_inv.eulerAngles(0, 1, 2);

    // std::cout << "transform_inv matrix:" << std::endl << transform_inv << std::endl;
    std::cout << "Yaw is:" << std::endl << euler_angles_inv[1] << " Rad" << std::endl;
    std::cout << "X is:" << transform_inv(0, 3) << " m" << "\t";
    std::cout << "Z is:" << transform_inv(2, 3) << " m" << std::endl;

    
    // // 打印旋转矩阵和平移矩阵
    // std::cout << "Rotation matrix:" << std::endl << rotation << std::endl;
    // std::cout << "Translation vector:" << std::endl << translation << std::endl;


    // 获取并打印四元数
    // Eigen::Quaternionf quaternion(rotation);
    // std::cout << "Quaternion'coeffs is:" << std::endl << quaternion.coeffs() <<std::endl;
    return transform;

}

void RANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
       if (cloud_in->size() > 0) {
        //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // 可选择配置，设置模型系数需要优化
        seg.setOptimizeCoefficients(true);

        // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
        //                SACMODEL_PLANE, 三维平面
        //                SACMODEL_LINE,    三维直线
        //                SACMODEL_CIRCLE2D, 二维圆
        //                SACMODEL_CIRCLE3D,  三维圆
        //                SACMODEL_SPHERE,      球
        //                SACMODEL_CYLINDER,    柱
        //                SACMODEL_CONE,        锥
        //                SACMODEL_TORUS,       环面
        //                SACMODEL_PARALLEL_LINE,   平行线
        //                SACMODEL_PERPENDICULAR_PLANE, 垂直平面
        //                SACMODEL_PARALLEL_LINES,  平行线
        //                SACMODEL_NORMAL_PLANE,    法向平面
        //                SACMODEL_NORMAL_SPHERE,   法向球
        //                SACMODEL_REGISTRATION,
        //                SACMODEL_REGISTRATION_2D,
        //                SACMODEL_PARALLEL_PLANE,  平行平面
        //                SACMODEL_NORMAL_PARALLEL_PLANE,   法向平行平面
        //                SACMODEL_STICK
        
        seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
        // you can modify the parameter below
        seg.setMaxIterations(10000);//设置最大迭代次数

        seg.setDistanceThreshold(0.01);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件

        seg.setInputCloud(cloud_in);

        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0) {
            std::cout << "error! Could not found any inliers!" << std::endl;
        }   


        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);

        //true表示的是输入点集以外的点,此处点集指inliers
        extractor.setNegative(true);
        extractor.filter(*obj_cloud);

        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZ> extractor1;//点提取对象
        extractor1.setInputCloud(cloud_in);
        extractor1.setIndices(inliers);

        //false表示的是输入点集的点
        extractor1.setNegative(false);
        extractor1.filter(*ground_cloud);
        std::cout << "filter done." << std::endl;

        // 保存配准后点云数据
        pcl::io::savePCDFileASCII("Cloud_filtered/obj_cloud.pcd", *obj_cloud);
        pcl::io::savePCDFileASCII("Cloud_filtered/ground_cloud.pcd", *ground_cloud);
    }
}

void merged(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f transform )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 将配准前后的两个点云数据合并
    origin_cloud->width = cloud_src->width + cloud_tgt->width;
    origin_cloud->height = 1;
    origin_cloud->points.resize(origin_cloud->width * origin_cloud->height);

    for (size_t i = 0; i < cloud_src->points.size(); i++)
    {
        origin_cloud->points[i].x = cloud_src->points[i].x;
        origin_cloud->points[i].y = cloud_src->points[i].y;
        origin_cloud->points[i].z = cloud_src->points[i].z;
        origin_cloud->points[i].r = 0;
        origin_cloud->points[i].g = 0;
        origin_cloud->points[i].b = 255;
    }

    int j = 0;
    for (size_t i = cloud_src->points.size(); i < origin_cloud->points.size(); i++)
    {
        origin_cloud->points[i].x = cloud_tgt->points[j].x;
        origin_cloud->points[i].y = cloud_tgt->points[j].y;
        origin_cloud->points[i].z = cloud_tgt->points[j].z;
        origin_cloud->points[i].r = 0;
        origin_cloud->points[i].g = 255;
        origin_cloud->points[i].b = 0;
        j++;
    }

    // 对合并后的点云进行ICP变换
    pcl::transformPointCloud(*cloud_src, *cloud_src, transform);

    // 将配准前后的两个点云数据合并
    merged_cloud->width = cloud_src->width + cloud_tgt->width;
    merged_cloud->height = 1;
    merged_cloud->points.resize(merged_cloud->width * merged_cloud->height);

    for (size_t i = 0; i < cloud_src->points.size(); i++)
    {
        merged_cloud->points[i].x = cloud_src->points[i].x;
        merged_cloud->points[i].y = cloud_src->points[i].y;
        merged_cloud->points[i].z = cloud_src->points[i].z;
        merged_cloud->points[i].r = 0;
        merged_cloud->points[i].g = 0;
        merged_cloud->points[i].b = 255;
    }

    j = 0;
    for (size_t i = cloud_src->points.size(); i < merged_cloud->points.size(); i++)
    {
        merged_cloud->points[i].x = cloud_tgt->points[j].x;
        merged_cloud->points[i].y = cloud_tgt->points[j].y;
        merged_cloud->points[i].z = cloud_tgt->points[j].z;
        merged_cloud->points[i].r = 0;
        merged_cloud->points[i].g = 255;
        merged_cloud->points[i].b = 0;
        j++;
    }

    pcl::io::savePCDFileASCII("Cloud_filtered/merged.pcd", *merged_cloud);
    pcl::io::savePCDFileASCII("Cloud_filtered/origin.pcd", *origin_cloud);
}