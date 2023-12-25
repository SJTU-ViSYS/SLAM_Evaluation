/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <ctime>
#include <cmath>
#include "ceres/ceres.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <setting.h>
#include <optimize.h>
#include <tool.h>

#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>


using namespace std;

void SeqProc(const string &path, const string &gt_name, const string &est_name);

int main(int argc, char **argv)
{
    srand(0);

    if(argc!=4){
        throw runtime_error("wrong number of argument.");
        return 0;
    }
    string path = (string)argv[1];
    string gt_name =  (string)argv[2];
    string est_name = (string)argv[3];

    SeqProc(path, gt_name, est_name);

    return 0;
}


void SeqProc(const string &path, const string &gt_name, const string &est_name)
{
    tool Tool;
    optimize Opt;
    bool SAVE = true;
    bool DEBUG = false;

    string name_gt = gt_name.substr(0, gt_name.length()-4);
    string name_est = est_name.substr(0, est_name.length()-4);

    string name_gt_sync = name_est+"_gt_sync";
    string name_est_info = name_est+"_evoalign_info";
    string name_est_evoalign = name_est+"_evoalign";

    string name_save2 = name_gt+"_optm";
    string name_save_res = path+"/"+"align_res";

    string path1 = path+"/"+name_gt+".txt";
    string path3 = path+"/"+name_gt_sync+".txt";

    string path2 = path+"/"+name_est+".txt";
    string path4 = path+"/"+name_est_info+".txt";
    string path7 = path+"/"+name_est_evoalign+".txt";

    string path6 = path+"/"+name_save2+".txt";

    if(DEBUG)
    {
        cout<<"name_gt: "<<name_gt<<endl;
        cout<<"name_est: "<<name_est<<endl;
        cout<<"name_gt_sync: "<<name_gt_sync<<endl;
        cout<<"name_est_info: "<<name_est_info<<endl;
        cout<<"name_est_evoalign: "<<name_est_evoalign<<endl;
        cout<<"name_save2: "<<name_save2<<endl;
        cout<<"name_save_res: "<<name_save_res<<endl;

        cout<<"pathes are: "<<endl;
        cout<<path1<<endl;
        cout<<path2<<endl;
        cout<<path3<<endl;
        cout<<path4<<endl;
        cout<<path7<<endl;
        cout<<path6<<endl;
    }

    // read traj
    vector<pose> Traj_ref, Traj_est_align;
    Traj_ref = Tool.ReadTraj(path3);
    Traj_est_align = Tool.ReadTraj(path7);

    // align rotation + t
    Eigen::Quaterniond q_align;
    Opt.Align_main(Traj_ref, Traj_est_align, q_align);
    cout<<q_align.w()<<"; "<<q_align.x()<<", "<<q_align.y()<<", "<<q_align.z()<<endl;

    // record
    vector<pose> Traj_ref_all;
    Traj_ref_all = Tool.ReadTraj(path1);
    Tool.UseResAlignRef(Traj_ref_all, q_align, SAVE, path6);

    // record align q
    if(SAVE){
        ofstream outfile;
        outfile.open(name_save_res.c_str(), ofstream::app);
        outfile << fixed;
        outfile << name_est<<": (wxyz)"<<endl;
        outfile << q_align.w()<<"; "<<q_align.x()<<", "<<q_align.y()<<", "<<q_align.z()<<endl;
        outfile.close();
    }

}
