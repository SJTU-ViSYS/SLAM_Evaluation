/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#include <tool.h>

vector<pose> tool::ReadTraj(string &path)
{
    cout<<"begin ReadTraj from: "<<path<<endl;
    std::ifstream f1;
    f1.open(path.data());
    assert(f1.is_open());
    std::string s1;
    vector<pose> vPose;

    while(getline(f1,s1))
    {
        char *char_s = (char *)s1.c_str();
        if(char_s[0]=='-' && char_s[1]=='-' && char_s[2]=='-')
            continue;

        const char *split = " ";
        char *p = strtok(char_s, split);
        vector<double> info;
        while(p != NULL){
            double val;
            std::sscanf(p, "%lf", &val);
            info.push_back(val);
            p=std::strtok(NULL, split);
        }

        // info
        struct pose pose_info;
        pose_info.timestamp = info[0];
        pose_info.twc = Mat31(info[1], info[2], info[3]);
        Eigen::Quaterniond qwc;
        qwc.x() = info[4];
        qwc.y() = info[5];
        qwc.z() = info[6];
        qwc.w() = info[7];
        qwc = qwc.normalized();
        pose_info.qwc = qwc;

        Mat33 Rwc(qwc);
        pose_info.Twc.setIdentity();
        pose_info.Twc.block<3,3>(0,0) = Rwc;
        pose_info.Twc.block<3,1>(0,3) = pose_info.twc;

        pose_info.Tcw = pose_info.Twc.inverse();        
        Eigen::Quaterniond qcw(pose_info.Tcw.block<3,3>(0,0));
        qcw = qcw.normalized();
        pose_info.qcw = qcw;
        pose_info.tcw = pose_info.Tcw.block<3,1>(0,3);

        pose_info.qwc = pose_info.qwc.normalized();
        pose_info.qcw = pose_info.qcw.normalized();

        vPose.push_back(pose_info);
    }

    f1.close();
    return vPose;
}

void tool::UseResAlignRef(const vector<pose> &Traj_ref, const Eigen::Quaterniond &q_align,
                          const bool &SAVE, string &save_name)
{
    cout<<"save name is: "<<save_name<<endl;

    ofstream outfile;
    outfile.open(save_name.c_str());
    outfile << fixed;

    if(SAVE)
    {
        for(size_t i=0; i<Traj_ref.size(); i++){

            Eigen::Quaterniond q_ref_align = q_align * Traj_ref[i].qcw;
            q_ref_align = q_ref_align.normalized();
            Mat33 Rcw(q_ref_align);
            Mat33 Rwc = Rcw.transpose();
            Eigen::Quaterniond qwc(Rwc);
            qwc = qwc.normalized();

            outfile<<setprecision(2) <<Traj_ref[i].timestamp<<setprecision(11)<<" ";
            outfile<<Traj_ref[i].twc(0,0)<<" "<<Traj_ref[i].twc(1,0)<<" "<<Traj_ref[i].twc(2,0)<<" ";
            outfile<<qwc.x()<<" "<< qwc.y()<<" "<< qwc.z()<<" "<< qwc.w() << '\n';
        }
        outfile.close();
    }

}

