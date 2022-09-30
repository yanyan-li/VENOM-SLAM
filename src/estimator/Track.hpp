/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-30 17:04:28
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

#include <algorithm>

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
        Track(Trajectory *robot_trajectory, std::vector<MapLine*> &vec_ptr_mls)
        :robot_traject_(robot_trajectory),vec_ptr_mls_(vec_ptr_mls)
        {

        };

        Eigen::Matrix3d VenomFrameDetection()
        {
            // detect venom (non-parallel directions)
            for (int frame_id = 0, frame_id_end = robot_traject_->traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
            {
                
#ifdef __VERBOSE__
                std::cout << "\033[0;33m [Venom Simulator Printer]: This is th \033[0m" << frame_id <<"\033[0;33m Frame \033[0m" << std::endl;
#endif          
                //vanishing directions detected in this frame      
                std::set<int> set_vd_type;
                std::vector<std::pair<int, Eigen::Vector3d>> vec_type_direction;
                //std::vector<Eigen::Vector3d> vec_vd;
                std::map<int, Eigen::Vector3d> VenomFrame;
                std::pair<int/*venom_id*/, Eigen::Matrix3d> r_venom_frame; 
                for (int observ_ml_i = 0, observ_ml_end = robot_traject_->obs_line_[frame_id].size(); observ_ml_i < observ_ml_end; observ_ml_i++)
                {
                    int ml_id = robot_traject_->obs_line_[frame_id][observ_ml_i].first;
                    Eigen::Matrix<double, 3, 2> ml_pos = robot_traject_->obs_line_[frame_id][observ_ml_i].second;
                    MapLine *ptr_mp_i = vec_ptr_mls_[ml_id];
#ifdef __VERBOSE__OFF
                    std::cout << "\033[0;35m [Venom Simulator Printer]: Vanishing direction type is \033[0m" << ptr_mp_i->vanishing_direction_type_ << std::endl;
#endif
                    // add different typies of vanishing direction into the set
                    int vd_type = ptr_mp_i->vanishing_direction_type_;
                    Eigen::Vector3d vd = ptr_mp_i->vanishing_direction_;

                    if (!set_vd_type.count(vd_type))
                    {
                        vec_type_direction.push_back(std::make_pair(vd_type, vd));// [vd_type] = vd;
                        // vec_vd.push_back(vd);
                        // std::cout<<"vec_vd: "<<vd<<std::endl;
                        set_vd_type.insert(vd_type);
                        // std::cout<<"vec_vd_type:"<<vd_type<<std::endl;
                    }
                }

                if(set_vd_type.size()>=2) // Vanom is detected
                {
                    std::cout<<"venom is detected"<<std::endl;
                    VenomFrameGeneration(vec_type_direction, VenomFrame); 
                }    
            }
        }

        void VenomFrameGeneration(std::vector<std::pair<int, Eigen::Vector3d>> &type_detections, std::map<int, Eigen::Vector3d> &venom_frame)
        {
            for (auto mit = type_detections.begin(), mit_end = type_detections.end(); mit != mit_end; mit++)
            {
                std::cout<<"before," << mit->first << ", " << mit->second << std::endl;
                venom_frame[mit->first] = mit->second;
            }
        }

        // void RotationCamera2Venom(std::map<int, Eigen::Vector3d> &venom_frame)
        // {
        //     venom_frame[0]

        // }


        


    public:
        std::vector<std::vector<std::pair<int /* */, Eigen::Matrix<double, 3, 2>>>> obs_line_;
        Trajectory* robot_traject_; 
        std::vector<MapLine*> vec_ptr_mls_;
        
    
    };
}