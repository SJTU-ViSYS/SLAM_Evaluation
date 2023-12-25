/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#ifndef TOOL_H
#define TOOL_H

#include <fstream>
#include <string>
#include <thread>
#include "ceres/ceres.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <setting.h>
#include <iomanip>

using namespace std;

class tool {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    vector<pose> ReadTraj(string &path);

    void UseResAlignRef(const vector<pose> &Traj_ref, const Eigen::Quaterniond &q_align, const bool &SAVE, string &save_name);

};

#endif // TOOL_H
