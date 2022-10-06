/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:40:42
 * @LastEditTime: 2022-10-06 06:43:23
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/test/main_entrance.cc
 */
#include <iostream>
#include <string>
#include "src/estimator/Trajectory.hpp"
#include "src/landmark/MapPoint.hpp"
#include "src/landmark/MapLine.hpp"
#include "src/optimizer/GlobalBundleAdjustment.hpp"
#include "src/feature/Reconstruct.hpp"
#include "src/visulizer/Interface.hpp"
#include "src/estimator/Track.hpp"

int main(int argc, char **argv)
{
    if (argc != 1)
    {
        std::cout << "usage: ./main_entrance " << std::endl;
        return -1;
    }

    std::cout << std::endl << 
        "\033[0;32m Venor SLAM Simulator Software Copyright (C) 2022 Yanyan Li, Technical University of Munich." << std::endl <<
        "This is a free software that is used to learn, teach, and test SLAM strategies." << std::endl <<
        "And you are welcome to contribute it and redistribute it under certain conditions. See LICENSE.txt. \033[0m" << std::endl << std::endl;

    // interface
    simulator::Interface viewer;
    viewer.show();

    

    //--> parameters
    int frame_num = 100;
    double distance = 5.0; // distance between wall and the trajectory center
    bool add_noise_to_meas = false;

    //--> keyframe generation
    // generate a circular trajectory with 100 keyframes
    simulator::Trajectory *robot_trajectory = new simulator::Trajectory(0, frame_num);
    robot_trajectory->GenerateTrajectory(simulator::Trajectory::CYCLE, frame_num);
    // robot_trajectory->PrintTrajectory();
    // std::cout << robot_trajectory->vec_traject_gt_Twc_.size() << "\033[m;35 keyframes are generated.\033[0m" << std::endl;

    //--> landmarks generation
    // mappoints
    std::vector<simulator::MapPoint*> vec_ptr_mappoints;
    std::vector<Eigen::Vector3d> points_gt;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_meas_keyframe_mp;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_gt_keyframe_mp;
    // maplines
    std::vector<simulator::MapLine* > vec_ptr_maplines;
    std::vector<Eigen::Matrix<double, 3, 2>> lines_gt;
    std::vector<std::vector<std::pair<int, Eigen::Matrix<double, 3, 2>>>> vec_meas_keyframe_ml;
    std::vector<std::vector<std::pair<int, Eigen::Matrix<double, 3, 2>>>> vec_gt_keyframe_ml;

    for (int id = 0; id < 200; id++)
    {
        simulator::MapPoint *ptr_mp = new simulator::MapPoint(id, robot_trajectory);
        if (id < 50)                                             // vertical-left
            ptr_mp->GenerateMapPoint(distance, "vertical-left"); // left side of the wall
        else if (id < 100)
            ptr_mp->GenerateMapPoint(-distance, "vertical-left"); // right side
        else if (id < 150)
            ptr_mp->GenerateMapPoint(distance, "vertical-right"); // front side
        else if (id < 200)
            ptr_mp->GenerateMapPoint(-distance, "vertical-right"); // back side

        ptr_mp->AddObservation(robot_trajectory->vec_traject_gt_Twc_, add_noise_to_meas);
        // ptr_mp->print();
        points_gt.push_back(ptr_mp->pos_world_);
        // vec_meas_keyframe_mp: mappoint_id<camera_id, mappoint_value>
        vec_meas_keyframe_mp.push_back(ptr_mp->obs);
        vec_gt_keyframe_mp.push_back(ptr_mp->obs_gt);
    }

#ifdef __VERBOSE__
    for (int j = 0, jend = robot_trajectory->vec_traject_gt_Twc_.size(); j < jend; j++)
    {
        std::cout << "the " << j << " th camera detects " << robot_trajectory->contain_mp_cams_[j]
                  << " mappoints" << std::endl;
    }
#endif

    for (int id = 0; id < 60; id++)
    {
        simulator::MapLine *ptr_ml = new simulator::MapLine(id, robot_trajectory);
        if (id < 10)
            ptr_ml->GenerateMapLine(distance, "vertical-left");
        else if (id < 20)
            ptr_ml->GenerateMapLine(-distance, "vertical-left");
        else if (id < 30)
            ptr_ml->GenerateMapLine(distance, "vertical-right");
        else if (id < 40)
            ptr_ml->GenerateMapLine(-distance, "vertical-right");
        else if (id < 45)
            ptr_ml->GenerateMapLine(distance, "horizontal-left");
        else if (id < 50)
            ptr_ml->GenerateMapLine(-distance, "horizontal-left");
        else if (id < 55)
            ptr_ml->GenerateMapLine(distance, "horizontal-right");
        else if (id < 60)
            ptr_ml->GenerateMapLine(-distance, "horizontal-right");

        ptr_ml->AddObservation(robot_trajectory->vec_traject_gt_Twc_, add_noise_to_meas);
        lines_gt.push_back(ptr_ml->pos_world_);
        vec_meas_keyframe_ml.push_back(ptr_ml->vec_obs_);
        vec_gt_keyframe_ml.push_back(ptr_ml->vec_obs_gt_); 
        vec_ptr_maplines.push_back(ptr_ml);
    }
    
#ifdef __VERBOSE__
    for (int j = 0, jend = robot_trajectory->vec_traject_gt_Twc_.size(); j < jend; j++)
    {
        std::cout << "the " << j << " th camera detects " << robot_trajectory->contain_ml_cams_[j]
                  << " maplines" << std::endl;
    }
#endif

    // detect venom for each frame
    simulator::Track* tracker = new simulator::Track(robot_trajectory, vec_ptr_maplines);
    tracker->VenomFrameDetection();

    tracker->VenomAssociation();

    
    
    simulator::Reconstruct recon;
    recon.Triangulation(vec_meas_keyframe_mp, robot_trajectory->vec_traject_gt_Twc_);

    std::vector<Eigen::Matrix4d> vec_traject_Twc_opti;
    // simulator::pointLocalBundleAdjustment::optimize(recon.tri_point_xyz_, vec_meas_keyframe_mp, robot_trajectory.vec_traject_gt_Twc_, vec_traject_Twc_opti);

   

    //
    return 0;
}
