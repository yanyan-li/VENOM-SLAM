/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-10-01 03:15:24
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
                // 
                std::vector<Eigen::Vector3d> venom_axis;
                venom_axis = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero());
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
                    int venom_type = VenomFrameGeneration(vec_type_direction, venom_axis); 
                
                }    
            }
        }

        int VenomFrameGeneration(std::vector<std::pair<int, Eigen::Vector3d>> &type_detections, std::vector<Eigen::Vector3d> &venom_axis)
        {
            // type 0: 0, 1
            // type 1: 0, 2
            // type 2: 0, 1, 2
            // type 3: 1, 2
             
            assert(type_detections.size()>=2);
            int venom_type = -1;
            for (auto mit = type_detections.begin(), mit_end = type_detections.end(); mit != mit_end; mit++)
            {
                std::cout<<"before," << mit->first << ", " << mit->second << std::endl;
                if(mit->first==0)
                {
                    venom_axis[0] = mit->second;
                    venom_type =0;
                }
                else if(mit->first==1)
                {
                    venom_axis[1] = mit->second;
                    if (venom_type==0)
                    {
                        venom_type = 1;
                    }else if(venom_type==-1)
                    {
                        venom_type = 3; 
                    }
                }
                else if(mit->first==2)
                    venom_axis[2] = mit->second;        
            }
            return venom_type;
        }

        void RotationCamera2Venom(std::vector<Eigen::Vector3d> &venom_axis, int venom_type)
        {
            Eigen::Matrix3d R_cm(Eigen::Matrix3d::Zero());
            // type 0: 0, 1
            // type 1: 0, 2
            // type 2: 0, 1, 2
            // type 3: 1, 2
            if(venom_type!=3)
            {
                R_cm(0,0) = venom_axis[0](0,0);
                R_cm(1,0) = venom_axis[0](1,0);
                R_cm(2,0) = venom_axis[0](2,0);
                if(venom_type==0)
                {
                    R_cm(0, 1) = venom_axis[1](0, 0);
                    R_cm(1, 1) = venom_axis[1](1, 0);
                    R_cm(2, 1) = venom_axis[1](2, 0);
                    // cross: axis_0 x axis_1 
                    Eigen::Vector3d z_x = venom_axis[0].cross(venom_axis[1]);
                    R_cm(0, 2) = z_x(0, 0);
                    R_cm(1, 2) = z_x(1, 0);
                    R_cm(2, 2) = z_x(2, 0);
                }
                else if(venom_type==1)
                {
                    R_cm(0, 2) = venom_axis[2](0, 0);
                    R_cm(1, 2) = venom_axis[2](1, 0);
                    R_cm(2, 2) = venom_axis[2](2, 0);
                    // cross: axis_0 x axis_1
                    Eigen::Vector3d z_x = venom_axis[0].cross(venom_axis[2]);
                    R_cm(0, 1) = z_x(0, 0);
                    R_cm(1, 1) = z_x(1, 0);
                    R_cm(2, 1) = z_x(2, 0);
                }
                else if(venom_type==2)
                {
                    R_cm(0, 1) = venom_axis[1](0, 0);
                    R_cm(1, 1) = venom_axis[1](1, 0);
                    R_cm(2, 1) = venom_axis[1](2, 0);
                    R_cm(0, 2) = venom_axis[2](0, 0);
                    R_cm(1, 2) = venom_axis[2](1, 0);
                    R_cm(2, 2) = venom_axis[2](2, 0);

                    //TODO: 
                    // SVD refinement
                }
            }else 
            {
                R_cm(0, 1) = venom_axis[1](0, 0);
                R_cm(1, 1) = venom_axis[1](1, 0);
                R_cm(2, 1) = venom_axis[1](2, 0);
                R_cm(0, 2) = venom_axis[2](0, 0);
                R_cm(1, 2) = venom_axis[2](1, 0);
                R_cm(2, 2) = venom_axis[2](2, 0);
                // cross: axis_0 x axis_1
                Eigen::Vector3d z_x = venom_axis[1].cross(venom_axis[2]);
                R_cm(0, 0) = z_x(0, 0);
                R_cm(1, 0) = z_x(1, 0);
                R_cm(2, 0) = z_x(2, 0);
            }
            

        }

        


        


    public:
        std::vector<std::vector<std::pair<int /* */, Eigen::Matrix<double, 3, 2>>>> obs_line_;
        Trajectory* robot_traject_; 
        std::vector<MapLine*> vec_ptr_mls_;
        
        
    
    };
}