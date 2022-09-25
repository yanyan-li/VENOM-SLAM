/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-25 05:47:30
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Pose estimation based on measurements.
 * @FilePath: /venom/src/estimator/Track.hpp
 */

#include "Trajectory.hpp"

namespace simulator{
    class Trajectory; 

    class Track
    {
        public:
            Track(std::vector<int> &frames_id, Trajectory*  robot_trajectory)
            {
                for(int i=0, iend = frames_id.size(); i<iend; i++) // frames_id
                {
                    robot_trajectory->traject_gt_Twc_


                }
            }; 

        public:




    };
}