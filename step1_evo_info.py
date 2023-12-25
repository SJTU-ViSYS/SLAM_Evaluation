'''
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
'''

from evo.core import sync
from evo.tools import file_interface
import numpy as np
import copy
import os, sys
import argparse

def record_evo_align(r_a, t_a, s, est_file, name):
    file = open(est_file[:-4]+name+".txt", 'w',encoding='UTF-8')
    file.write(str(r_a)+'\n')
    file.write(str(t_a)+'\n')
    file.write(str(s)+'\n')
    file.close()

def record_traj(traj, file_name, save_name):
    for i in range(traj.num_poses):
        new1 = np.reshape(traj.timestamps[i], (1,-1))
        new2 = np.reshape(traj.positions_xyz[i,:], (1,-1))
        new3 = np.reshape(traj.orientations_quat_wxyz[i,1:4], (1,-1))
        new4 = np.reshape(traj.orientations_quat_wxyz[i,0], (1,-1))
        new = np.concatenate(( new1, new2, new3, new4), axis=1)
        if i==0:
            data_print = new
        else:
            data_print = np.concatenate(( data_print, new), axis=0)
    np.savetxt(file_name[:-4]+save_name+".txt", data_print, fmt="%.2f %.11f %.11f %.11f %.11f %.11f %.11f %.11f", delimiter=" ")       


def main_evo_info(seq_gt_file, seq_est_file):
    print("read gt file from {}".format(seq_gt_file))
    print("read est file from {}".format(seq_est_file))

    gt_file_name = os.path.basename(seq_gt_file)
    est_file_name = os.path.basename(seq_est_file)

    # read in
    traj_ref = file_interface.read_tum_trajectory_file(seq_gt_file)
    traj_est = file_interface.read_tum_trajectory_file(seq_est_file)

    # Synchronizes
    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)
    traj_est_aligned = copy.deepcopy(traj_est)
    # Align positions
    r_a, t_a, s = traj_est_aligned.align(traj_ref, correct_scale=True, correct_only_scale=False)

    # save evo info
    record_evo_align(r_a, t_a, s, est_file_name, "_evoalign_info")
    ## save gt_sync
    record_traj(traj_ref, est_file_name, "_gt_sync")
    ## save est_align
    record_traj(traj_est_aligned, est_file_name, "_evoalign")


if __name__ == "__main__":    
    
    example_cmd = """
    %(prog)s -ref [ground truth/reference name] -est [estimation name]
    """

    parser = argparse.ArgumentParser(description='record evo info', usage=example_cmd)

    parser.add_argument('-ref', help='path of reference file')
    parser.add_argument('-est', help='path of estimation file')

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(2)
    
    args = parser.parse_args()

    seq_gt = args.ref
    seq_est = args.est
    main_evo_info(seq_gt, seq_est)