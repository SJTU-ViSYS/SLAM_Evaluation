/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#include <optimize.h>

void optimize::Align_main(vector<pose> &Traj_ref, vector<pose> &Traj_est, Eigen::Quaterniond &q_align)
{
    assert((int)Traj_ref.size() == (int)Traj_est.size() );

    // pose initialization
    Mat33 R_init;
    R_init << -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
            0.0, -1.0, 0.0;
    Eigen::Quaterniond q_init(R_init);
    q_init = q_init.normalized();
    double *q_opt = new double[4];
    q_opt[0] = q_init.w();
    q_opt[1] = q_init.x();
    q_opt[2] = q_init.y();
    q_opt[3] = q_init.z();

    optimize_q(q_opt, Traj_ref, Traj_est, q_align);

}

void optimize::optimize_q(double *pose_q, const vector<pose> &Traj_ref, const vector<pose> &Traj_est, Eigen::Quaterniond &q_align)
{
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    ceres::Problem problem;

    problem.AddParameterBlock(pose_q, 4, new ceres::QuaternionParameterization());

    for(size_t i_pose = 0; i_pose<Traj_ref.size(); i_pose++){
        LossFunction* loss_function = nullptr;
        CostFunction *costFunction;
        costFunction = numer_rotation_ref::Create(Traj_ref[i_pose], Traj_est[i_pose]);
        problem.AddResidualBlock(costFunction, loss_function, pose_q);
    }

    Solver::Options options;
    options.minimizer_type = TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 20;
    options.num_threads = 1;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout<<summary.BriefReport() <<endl;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double tOptim = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    cout<<"[TIME] Optimize usage (S): "<<tOptim<<endl;

    q_align.w() = pose_q[0];
    q_align.x() = pose_q[1];
    q_align.y() = pose_q[2];
    q_align.z() = pose_q[3];

}
