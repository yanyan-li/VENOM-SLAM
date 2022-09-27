/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-22 16:10:56
 * @LastEditTime: 2022-09-27 17:15:37
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /venom/test/test_show_env.cc
 */
#include <iostream>
#include <string>
#include "src/estimator/Trajectory.hpp"
#include "src/landmark/MapPoint.hpp"
#include "src/landmark/MapLine.hpp"
#include "src/optimizer/GlobalBundleAdjustment.hpp"
#include "src/feature/Reconstruct.hpp"
#include "src/visulizer/Visualizer.hpp"

int main(int argc, char **argv)
{
    if (argc != 1)
    {
        std::cout << "usage: ./show_estimator_env " << std::endl;
        return -1;
    }

    //--> parameters
    int frame_num = 100;
    double distance = 5.0; // distance between wall and the trajectory center
    bool add_noise_to_meas = true;

    //--> keyframe generation
    // generate a circular trajectory with 100 keyframes
    simulator::Trajectory *robot_trajectory = new simulator::Trajectory(0, frame_num);
    robot_trajectory->GenerateTrajectory(simulator::Trajectory::CYCLE, frame_num);
    // robot_trajectory->PrintTrajectory();
    std::cout << robot_trajectory->traject_gt_Twc_.size() << "\033[m;35 keyframes are generated.\033[0m" << std::endl;

    //--> landmarks generation
    // mappoints
    std::vector<Eigen::Vector3d> points_gt;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_meas_keyframe_mp;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_gt_keyframe_mp;
    // maplines
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

        ptr_mp->AddObservation(robot_trajectory->traject_gt_Twc_, add_noise_to_meas);
        // ptr_mp->print();
        points_gt.push_back(ptr_mp->pos_world_);
        // vec_meas_keyframe_mp: mappoint_id<camera_id, mappoint_value>
        vec_meas_keyframe_mp.push_back(ptr_mp->obs);
        vec_gt_keyframe_mp.push_back(ptr_mp->obs_gt);
    }

    for (int j = 0, jend = robot_trajectory->traject_gt_Twc_.size(); j < jend; j++)
    {
        std::cout << "the " << j << " th camera detects " << robot_trajectory->contain_mp_cams_[j]
                  << " mappoints" << std::endl;
    }

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

        ptr_ml->AddObservation(robot_trajectory->traject_gt_Twc_, add_noise_to_meas);
        lines_gt.push_back(ptr_ml->pos_world_);
        vec_meas_keyframe_ml.push_back(ptr_ml->vec_obs_);
        vec_gt_keyframe_ml.push_back(ptr_ml->vec_obs_gt_);
    }

    for (int j = 0, jend = robot_trajectory->traject_gt_Twc_.size(); j < jend; j++)
    {
        std::cout << "the " << j << " th camera detects " << robot_trajectory->contain_ml_cams_[j]
                  << " maplines" << std::endl;
    }

    

    simulator::Reconstruct recon;
    recon.Triangulation(vec_meas_keyframe_mp, robot_trajectory->traject_gt_Twc_);

    std::vector<Eigen::Matrix4d> vec_traject_Twc_opti;
    // simulator::pointLocalBundleAdjustment::optimize(recon.tri_point_xyz_, vec_meas_keyframe_mp, robot_trajectory.traject_gt_Twc_, vec_traject_Twc_opti);

    //  visualization
    simulator::Visualizer viewer;
    viewer.SetEnvParameter(points_gt, lines_gt, robot_trajectory->traject_gt_Twc_,
                           vec_traject_Twc_opti, vec_meas_keyframe_mp,
                           recon.tri_point_inverse_depth_, recon.tri_point_xyz_);
    viewer.show();

    //
    return 0;
}
