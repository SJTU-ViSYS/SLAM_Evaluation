/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#ifndef OPTIMIZE_H
#define OPTIMIZE_H

#include <string>
#include <thread>
#include "ceres/ceres.h"
#include <setting.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

# include <include/numer_rotation_ref.h>

using namespace std;
using namespace ceres;

class optimize{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void Align_main(vector<pose> &Traj_ref, vector<pose> &Traj_est,
                    Eigen::Quaterniond &q_align);

private:
    void optimize_q(double *pose_q, const vector<pose> &Traj_ref, const vector<pose> &Traj_est, Eigen::Quaterniond &q_align);

};

#endif // OPTIMIZE_H
