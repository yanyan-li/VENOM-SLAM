/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:40:42
 * @LastEditTime: 2022-10-06 16:13:27
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/test/main_entrance.cc
 */
#include <iostream>
#include <string>
#include "src/visulizer/Interface.hpp"


int main(int argc, char **argv)
{
    if (argc != 1)
    {
        std::cout << "usage: ./main_entrance " << std::endl;
        return -1;
    }

    std::cout << std::endl << 
        "\033[0;32mVenom SLAM Simulator Software Copyright (C) 2022 Yanyan Li, Technical University of Munich." << std::endl <<
        "This is a free software that is used to learn, teach, and test SLAM strategies." << std::endl <<
        "And you are welcome to contribute it and redistribute it under certain conditions. See LICENSE.txt. \033[0m" << std::endl << std::endl;

    // interface
    simulator::Interface venom_entrance;
    venom_entrance.start();

//     // detect venom for each frame
//     simulator::Track* tracker = new simulator::Track(robot_trajectory, vec_ptr_maplines);
//     tracker->VenomFrameDetection();

//     tracker->VenomAssociation();


//     std::vector<Eigen::Matrix4d> vec_traject_Twc_opti;
    // simulator::pointLocalBundleAdjustment::optimize(recon.tri_point_xyz_, vec_meas_keyframe_mp, robot_trajectory.vec_traject_gt_Twc_, vec_traject_Twc_opti);

   

    //
    return 0;
}
