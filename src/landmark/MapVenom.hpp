/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-30 14:02:44
 * @LastEditTime: 2022-10-01 16:34:43
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Venom 
 * @FilePath: /venom/src/landmark/MapVenom.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPVENOM_HPP_
#define __VENOM_SRC_LANDMARK_MAPVENOM_HPP_

#include "../estimator/Trajectory.hpp"

namespace simulator
{
    class Trajectory;

    class MapVenom{
        public:
            int num_id_;
            int observed_;
            int anchor_frame_id_;// the frame that detects this MapVenom fristly.
            int venom_type;
            Eigen::Matrix3d rotation_venom_world_; 
            Trajectory* traject_;

        public:
            MapVenom(const int id, Trajectory* traject):num_id_(id),anchor_frame_id_(-1)
            {};

            void AddObservation()
            {

            }



    };



}




#endif //__VENOM_SRC_LANDMARK_MAPVENOM_HPP_