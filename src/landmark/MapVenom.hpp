/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-30 14:02:44
 * @LastEditTime: 2022-09-30 14:14:04
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
            int anchor_frame_id_;
            Trajectory* traject_;
        public:
            MapVenom(){};

    };



}




#endif //__VENOM_SRC_LANDMARK_MAPVENOM_HPP_