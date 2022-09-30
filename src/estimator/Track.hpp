/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-30 00:30:21
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/estimator/Track.hpp
 */
/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-25 05:47:30
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Pose estimation based on measurements.
 * @FilePath: /venom/src/estimator/Track.hpp
 */
#include "Trajectory.hpp"
#include "../landmark/MapLine.hpp"

namespace simulator
{
    class Trajectory;
    class MapLine;

    class Track
    {
    public:
        // initialization
        Track(Trajectory *robot_trajectory, std::vector<MapLine*> vec_ptr_mls)
        :robot_traject_(robot_trajectory),vec_ptr_mls_(vec_ptr_mls)
        {

        };

        Eigen::Matrix3d VenomFrameDetection()
        {
            // detect venom (non-parallel directions)
            for (int frame_id = 0, frame_id_end = robot_traject_->traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
            {
                for(int observ_ml_i = 0, observ_ml_end= robot_traject_->obs_line_[frame_id].size(); observ_ml_i<observ_ml_end; observ_ml_i++)
                {
                    int ml_id =  robot_traject_->obs_line_[frame_id][observ_ml_i].first;
                    Eigen::Matrix<double, 3, 2> ml_pos = robot_traject_->obs_line_[frame_id][observ_ml_i].second; 
                    MapLine*  ptr_mp_i = vec_ptr_mls_[ml_id]; 

#ifdef __VERBOSE__
                    std::cout<<"\033[0;35m [Venom Simulator Printer]: Vanishing direction type is \033[0m"<<ptr_mp_i->vanishing_direction_type_<<std::endl;
#endif
                }
            }
        }

    public:
        std::vector<std::vector<std::pair<int /* */, Eigen::Matrix<double, 3, 2>>>> obs_line_;
        Trajectory* robot_traject_; 
        std::vector<MapLine*> vec_ptr_mls_;
    
    };
}