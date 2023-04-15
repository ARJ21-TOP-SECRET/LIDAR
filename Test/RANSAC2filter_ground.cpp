#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ctime>

using namespace std;
int main (int argc, char** argv)
{
    //可用于可视化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 读取点云数据
    pcl::io::loadPCDFile<pcl::PointXYZ> ("Cloud_data/01_01.pcd", *cloud_in);

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
            cout << "error! Could not found any inliers!" << endl;
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
        cout << "filter done." << endl;

        // 保存配准后点云数据
        pcl::io::savePCDFileASCII("Cloud_filtered/obj_cloud.pcd", *obj_cloud);
        pcl::io::savePCDFileASCII("Cloud_filtered/ground_cloud.pcd", *ground_cloud);
    }

}