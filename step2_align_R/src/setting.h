/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#ifndef SETTING_H
#define SETTING_H

#include <fstream>
#include <string>
#include <thread>
#include "ceres/ceres.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

typedef Eigen::Matrix<double,3,3> Mat33;
typedef Eigen::Matrix<double,4,4> Mat44;
typedef Eigen::Matrix<double,3,1> Mat31;
typedef Eigen::Matrix<double,4,1> Mat41;
typedef Eigen::Matrix<double,1,4> Mat14;
typedef Eigen::Matrix<double,1,3> Mat13;
typedef Eigen::Matrix<double,1,1> Mat11;

typedef Eigen::Matrix<double,3,1> Vec3;
typedef Eigen::Matrix<double,2,1> Vec2;

struct pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Mat44 Twc;  // no use
    Eigen::Quaterniond qwc;
    Mat31 twc;
    Mat44 Tcw;  // no use
    Eigen::Quaterniond qcw;
    Mat31 tcw;
};

#endif // SETTING_H
