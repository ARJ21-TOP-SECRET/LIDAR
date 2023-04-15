#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

int main (int argc, char** argv)
{

    YY 

    
    Eigen::Matrix4f T;
    Eigen::Vector3f a = (0,0,1);

    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Eigen::Vector3f t = T.block<3,1>(0,3);

    Eigen::Vector3f a_prime = R * a + t;

    Eigen::Matrix3f R_inv = R.inverse();
    Eigen::Vector3f t_inv = -R_inv * t;

    Eigen::Vector3f b = R_inv * a_prime +t_inv;
    std::cout<<
    return 0;
}

