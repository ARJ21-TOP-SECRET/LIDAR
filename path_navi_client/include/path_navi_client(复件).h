/*
@Time    : 2023 2023/6/15 下午5:25
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    :
*/
#ifndef PATH_NAVI_CLIENT_H
#define PATH_NAVI_CLIENT_H
#include "icp.hpp"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <path_navi/PathNaviAction.h>

#include <std_msgs/Bool.h>
#include<std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <mutex>
#include <fstream>
#include <random>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

using namespace std;


struct TF_Pose
{
    double distance_x;
    double distance_y;
    double distance_z;
    double roll;
    double pitch;
    double yaw;
};



class PathNaviClient
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber  gps_info_sub_,map_pose_sub_,result_sub_,navi_result_sub_,gnss_ksxt_sub_;
    ros::Publisher system_poweroff_pub_,velocity_pub_;
    actionlib::SimpleActionClient<path_navi::PathNaviAction> ac_client_;

    void doneCb(const actionlib::SimpleClientGoalState& state,
                       const path_navi::PathNaviResultConstPtr& result);
    void activeCb();
    void feedbackCb(const path_navi::PathNaviFeedbackConstPtr& feedback);
    void GpsInfoCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void GnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    Eigen::Matrix3d quaternionToRotationMatrix(const geometry_msgs::Quaternion& quaternion);
    void lookup_tf_pose(std::string parent_link , std::string child_link,TF_Pose& pose);
    tf::TransformListener& tf_;  
public:
    PathNaviClient(tf::TransformListener& tf);
    void rotateRobotByAngle(double theta,double w_z);
    void MoveToFollow(nav_msgs::Path& msg);
    nav_msgs::Path ReadPathDataFromFile();

    void SavePath();
    void SetPowerOff();
    void CancelGoal();
    void Move_com();
    void Move_transform(double x, double y, double z);
    bool SetOrigin();
    ~PathNaviClient();
    bool goal_reached_ = false;
    double altitude_ = 0;  //海拔高度
    double latitude_ = 0;  //纬度
    double longitude_ = 0; //经度
    double status_ = 0; //4，有效解；其他数值属于无效解，GPS数据不可用
    mutex m_pose_;
    double x_,y_,z_,yaw_;
    geometry_msgs::PoseStamped current_pose_;
    // std::string filename_ = "/home/dell/robotconfig/path.txt";
    std::string filename_ = "/home/dell/robotconfig/path_2.txt";
};

#endif //POINT_NAVI_CLIENT_H
