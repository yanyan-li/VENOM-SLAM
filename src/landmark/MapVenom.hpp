/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-30 14:02:44
 * @LastEditTime: 2022-10-02 03:19:48
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Venom 
 * @FilePath: /venom/src/landmark/MapVenom.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPVENOM_HPP_
#define __VENOM_SRC_LANDMARK_MAPVENOM_HPP_

#include "../estimator/Trajectory.hpp"
#include <map>
#include <vector>
#include <iostream>
#include <random>
 
#include <eigen3/Eigen/Dense>
namespace simulator
{
    static int vm_id =0;
    class Trajectory;

    class MapVenom{
        public:
            int num_id_;
            int observed_;
            int anchor_frame_id_;// the frame that detects this MapVenom fristly.
            int venom_type_;
            Eigen::Matrix3d rotation_venom_world_; 
            //Trajectory* traject_;
            std::vector< std::pair<int, Eigen::Matrix3d>> vec_obs_;  // noised observation
            std::vector< std::pair<int, Eigen::Matrix3d>> vec_obs_gt_;  // noised observation


        public:
            MapVenom(int anchor_frame_id, int type):anchor_frame_id_(anchor_frame_id),
            venom_type_(type)
            {
                num_id_ = vm_id++;

            };

            

            void AddObservation(int venom_frame_id, Eigen::Matrix3d rotation_cam_venom)
            {
                
                vec_obs_.push_back(std::make_pair(venom_frame_id, rotation_cam_venom));

            }



    };



}




#endif //__VENOM_SRC_LANDMARK_MAPVENOM_HPP_