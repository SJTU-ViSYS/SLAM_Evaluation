/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#ifndef NUMER_ROTATION_REF_H
#define NUMER_ROTATION_REF_H

#include <iostream>
#include "ceres/ceres.h"
#include "ModelTool.hpp"
#include <opencv2/core.hpp>
#include "setting.h"
using namespace std;
using namespace ceres;

class numer_rotation_ref{
public:

    numer_rotation_ref(pose traj_ref, pose traj_est):
        _traj_ref(traj_ref), _traj_est(traj_est){}

    bool operator()(const double* const _q,
                    double* residuals) const {

        Eigen::Quaterniond q_align(_q[0], _q[1], _q[2], _q[3]);

        Eigen::Quaterniond q_ref_align = q_align * _traj_ref.qcw;
        q_ref_align = q_ref_align.normalized();
        Eigen::Quaterniond q_difference = q_ref_align.inverse() * _traj_est.qcw;

        Eigen::Vector3d t_difference(0.0, 0.0, 0.0);
        double s_difference = 1.0;

        Eigen::Matrix <double, 7, 1> res = logSim3(q_difference, t_difference, s_difference);

        residuals[0] = res(0,0);
        residuals[1] = res(1,0);
        residuals[2] = res(2,0);
        residuals[3] = res(3,0);
        residuals[4] = res(4,0);
        residuals[5] = res(5,0);
        residuals[6] = res(6,0);

        return true;
    }

    static ceres::CostFunction *Create(pose traj_ref, pose traj_est) {
        return (new ceres::NumericDiffCostFunction<numer_rotation_ref, ceres::CENTRAL, 7, 4>(
                new numer_rotation_ref(traj_ref, traj_est)));
    }

private:
    pose _traj_ref, _traj_est;

};

#endif // NUMER_ROTATION_REF_H
