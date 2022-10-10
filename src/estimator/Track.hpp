/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-25 05:47:30
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Pose estimation based on measurements.
 * @FilePath: /venom/src/estimator/Track.hpp
 */

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "Trajectory.hpp"
#include "../landmark/MapVenom.hpp"
#include "../landmark/MapLine.hpp"

namespace simulator
{
    class Trajectory;
    class MapLine;
    class MapVeom;

    class Track
    {

    public:
        std::vector<std::vector<std::pair<int /* */, Eigen::Matrix<double, 3, 2>>>> obs_line_;
        Trajectory *robot_traject_;
        std::vector<MapLine *> vec_ptr_mls_;
        

    public:
        // initialization
        Track(Trajectory *robot_trajectory, std::vector<MapLine *> &vec_ptr_mls)
            : robot_traject_(robot_trajectory), vec_ptr_mls_(vec_ptr_mls)
            {};

        Eigen::Matrix3d VenomFrameDetection()
        {
            // detect venom (non-parallel directions)
            for (int frame_id = 0, frame_id_end = robot_traject_->vec_traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
            {

#ifdef __VERBOSE__
                std::cout << "\033[0;33m [Venom Simulator Printer]: This is th \033[0m" << frame_id << "\033[0;33m Frame \033[0m" << std::endl;
#endif
                // vanishing directions detected in this frame
                std::set<int> set_vd_type;
                std::vector<std::pair<int, Eigen::Vector3d>> vec_type_direction;
                // std::vector<Eigen::Vector3d> vec_vd;
                std::map<int, Eigen::Vector3d> VenomFrame;
                //
                std::vector<Eigen::Vector3d> venom_axis;
                venom_axis = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero());
                std::pair<int /*venom_id*/, Eigen::Matrix3d> r_venom_frame;

                //--> loop for detecting the number of vanishing directions
                for (int observ_ml_i = 0, observ_ml_end = robot_traject_->obs_line_[frame_id].size(); observ_ml_i < observ_ml_end; observ_ml_i++)
                {
                    int ml_id = robot_traject_->obs_line_[frame_id][observ_ml_i].first;
                    
                    // detected vanishing direction (maplines)
                    Eigen::Matrix<double, 3, 2> ml_pos_cam = robot_traject_->obs_line_[frame_id][observ_ml_i].second;
                    Eigen::Vector3d vd = (ml_pos_cam.block(0, 0, 3, 1) - ml_pos_cam.block(0, 1, 3, 1)).normalized();
                    
                    // the type of vanishing direction:   For association
                    MapLine *ptr_mp_i = vec_ptr_mls_[ml_id];

                    int vd_type = ptr_mp_i->vanishing_direction_type_;

#ifdef __VERBOSE__OFF //(remove "OFF", if you want to print )
                    std::cout << "\033[0;35m [Venom Simulator Printer]: Vanishing direction type is \033[0m" << ptr_mp_i->vanishing_direction_type_ << std::endl;
#endif
                    // add different typies of vanishing direction into the set
                    // Eigen::Vector3d vd = ptr_mp_i->vanishing_direction_;

                    if (!set_vd_type.count(vd_type))
                    {
                        vec_type_direction.push_back(std::make_pair(vd_type, vd)); // [vd_type] = vd;
                        std::cout<<"vd: "<<vd<<std::endl;
                        // vec_vd.push_back(vd);
                        set_vd_type.insert(vd_type);
                    }
                }

                //--> more than two unparallel directions
                if (set_vd_type.size() >= 2) // Vanom is detected
                {
                    //--> detected  
                    int venom_type = VenomFrameGeneration(vec_type_direction, venom_axis);
                    Eigen::Matrix3d rotation_frame_venom_cam_id = RotationCamera2Venom(venom_type, venom_axis);
#ifdef __VERBOSE__                    
                    std::cout <<"\033[0;35m [Venom Simulator Printer]: venom is detected. " << std::endl
                    <<"The venom_type is "<<venom_type
                    <<". And the rotation from camera to venom is \033[0m"<<std::endl<<rotation_frame_venom_cam_id<<std::endl;
#endif          
                    robot_traject_->SetKeyFrameDetects(frame_id, venom_type,rotation_frame_venom_cam_id);    

                }
            }
        }

        int VenomFrameGeneration(std::vector<std::pair<int, Eigen::Vector3d>> &type_detections, std::vector<Eigen::Vector3d> &venom_axis)
        {
            // type 0: 0 ..
            // type 1: 0, 1
            // type 2: 0, 2
            // type 3: 0, 1, 2
            // type 4: 1, 2
            assert(type_detections.size() >= 2);
            int venom_type = -1;
            for (auto mit = type_detections.begin(), mit_end = type_detections.end(); mit != mit_end; mit++)
            {
                // std::cout << "before," << mit->first << ", " << mit->second << std::endl;
                if (mit->first == 0)   
                {
                    venom_axis[0] = mit->second;
                    venom_type = 0;
                }
                else if (mit->first == 1)
                {
                    
                    if (venom_type == 0)
                    {
                        venom_type = 1;  // means: first-0, second-1 
                        venom_axis[1] = (mit->second - mit->second.dot(venom_axis[0])/(venom_axis[0].dot(venom_axis[0])) * venom_axis[0]).normalized() ;
                        
                        assert(venom_axis[1].dot(venom_axis[0])<1e-6); 
                        
                    }
                    else if (venom_type == -1) // means: first-1, future-second-2
                    {
                        venom_type = 4;
                        venom_axis[1] = mit->second;
                    }
                }
                else if (mit->first == 2)
                {
                    //venom_axis[2] = mit->second;
                    if(venom_type == 0) // means: first-0, second-2
                    {
                        venom_type = 2;
                        venom_axis[2] = (mit->second - mit->second.dot(venom_axis[0])/(venom_axis[0].dot(venom_axis[0])) * venom_axis[0]).normalized() ;
                        assert(venom_axis[2].dot(venom_axis[0])<1e-6);  
                    }
                    else if(venom_type == 1) //means: first-0, second-1, third-2
                    {
                        venom_type = 3;
                        venom_axis[2] = (mit->second - mit->second.dot(venom_axis[0])/(venom_axis[0].dot(venom_axis[0])) * venom_axis[0] -
                                        mit->second.dot(venom_axis[1])/(venom_axis[1].dot(venom_axis[1])) * venom_axis[1]).normalized() ;
                        assert(venom_axis[2].dot(venom_axis[0])<1e-6 && venom_axis[2].dot(venom_axis[1])<1e-6 ) ; 
                    }
                    else
                    {
                        std::cerr<<"problem happpens"<<std::endl;
                    }
                }
            }
            assert(venom_type>=1&&venom_type<=4);
            return venom_type;
        }

        Eigen::Matrix3d RotationCamera2Venom(int venom_type, std::vector<Eigen::Vector3d> &venom_axis)
        {
            Eigen::Matrix3d R_cm(Eigen::Matrix3d::Zero());
            // type 1: 0, 1
            // type 2: 0, 2
            // type 3: 0, 1, 2
            // type 4: 1, 2
            if (venom_type != 4)
            {
                R_cm(0, 0) = venom_axis[0](0, 0);
                R_cm(1, 0) = venom_axis[0](1, 0);
                R_cm(2, 0) = venom_axis[0](2, 0);
                if (venom_type == 1)
                {
                    R_cm(0, 1) = venom_axis[1](0, 0);
                    R_cm(1, 1) = venom_axis[1](1, 0);
                    R_cm(2, 1) = venom_axis[1](2, 0);
                    // cross: axis_0 x axis_1
                    Eigen::Vector3d z_x = (venom_axis[0].cross(venom_axis[1])).normalized();
                    R_cm(0, 2) = z_x(0, 0);
                    R_cm(1, 2) = z_x(1, 0);
                    R_cm(2, 2) = z_x(2, 0);
                }
                else if (venom_type == 2)
                {
                    R_cm(0, 2) = venom_axis[2](0, 0);
                    R_cm(1, 2) = venom_axis[2](1, 0);
                    R_cm(2, 2) = venom_axis[2](2, 0);
                    // cross: axis_0 x axis_1
                    Eigen::Vector3d z_x = (venom_axis[0].cross(venom_axis[2])).normalized();
                    R_cm(0, 1) = z_x(0, 0);
                    R_cm(1, 1) = z_x(1, 0);
                    R_cm(2, 1) = z_x(2, 0);
                }
                else if (venom_type == 3)
                {
                    R_cm(0, 1) = venom_axis[1](0, 0);
                    R_cm(1, 1) = venom_axis[1](1, 0);
                    R_cm(2, 1) = venom_axis[1](2, 0);
                    R_cm(0, 2) = venom_axis[2](0, 0);
                    R_cm(1, 2) = venom_axis[2](1, 0);
                    R_cm(2, 2) = venom_axis[2](2, 0);
                }
            }
            else
            {
                R_cm(0, 1) = venom_axis[1](0, 0);
                R_cm(1, 1) = venom_axis[1](1, 0);
                R_cm(2, 1) = venom_axis[1](2, 0);
                R_cm(0, 2) = venom_axis[2](0, 0);
                R_cm(1, 2) = venom_axis[2](1, 0);
                R_cm(2, 2) = venom_axis[2](2, 0);
                // cross: axis_0 x axis_1
                Eigen::Vector3d z_x = (venom_axis[1].cross(venom_axis[2])).normalized();
                R_cm(0, 0) = z_x(0, 0);
                R_cm(1, 0) = z_x(1, 0);
                R_cm(2, 0) = z_x(2, 0);
            } 
            // TODO: make it orthogonal
            cv::SVD svd; cv::Mat U,W,VT, R_cm_ortho;
            cv::eigen2cv(R_cm, R_cm_ortho);
            svd.compute(R_cm_ortho,W,U,VT);
            R_cm_ortho=U*VT;
            cv::cv2eigen(R_cm_ortho,R_cm);
            std::cout<<"R:"<<R_cm<<", "<<R_cm_ortho<<std::endl;
            return R_cm;
        }
   
        void VenomAssociation()
        {
            //
            std::set<int> set_new_venom;
            std::vector<simulator::MapVenom*> vec_ptr_mv;
            for (int frame_id = 0, frame_id_end = robot_traject_->vec_traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
            {
                
                assert(robot_traject_->contain_mw_cams_[frame_id]<=1);
                
                if(robot_traject_->obs_mw_[frame_id].size())
                {
                    int venom_type = robot_traject_->obs_mw_[frame_id][0].first;

                    //std::cout<<"the venom_type is "<<venom_type<<std::endl;
                     if (!set_new_venom.count(venom_type)) // the first time deteced
                     {
                        //create a new mapvenom
                        simulator::MapVenom* ptr_mv = new simulator::MapVenom(frame_id, venom_type);
                        
                        vec_ptr_mv.push_back(ptr_mv); 

                        set_new_venom.insert(venom_type);
                     }
                }

            }

#ifdef __VERBOSE__
            std::cout<<"vec venom"<< vec_ptr_mv.size()<<std::endl;
            for(int i=0, i_end=vec_ptr_mv.size(); i<i_end; i++)
            {
                std::cout<<"the venom id is "<<vec_ptr_mv[i]->num_id_<<std::endl;
            }
#endif
            //TODO: associate each frame to mv
            for(int i=0, i_end=vec_ptr_mv.size(); i<i_end; i++)
            {
                
                int venom_type = vec_ptr_mv[i]->venom_type_; 
                int anchor_frame_id = vec_ptr_mv[i]->anchor_frame_id_;
                for (int frame_id = 0, frame_id_end = robot_traject_->vec_traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
                {
                    if(venom_type == robot_traject_->obs_mw_[frame_id][0].first)
                    {
                        if(frame_id == anchor_frame_id)
                            continue;
                        assert(frame_id>=anchor_frame_id);
                        vec_ptr_mv[i]->AddObservation(frame_id, robot_traject_->obs_mw_[frame_id][0].second);
                        vec_ptr_mv[i]->observed_+=1;

                        // compute rotation 

                        Eigen::Matrix3d R_1_v = robot_traject_->obs_mw_[frame_id][0].second;
                        Eigen::Matrix3d R_2_v = robot_traject_->obs_mw_[anchor_frame_id][0].second;

                        //
                        Eigen::Matrix3d R_3_v = robot_traject_->vec_traject_gt_Twc_[frame_id].block(0,0,3,3);
                        Eigen::Matrix3d R_4_v = robot_traject_->vec_traject_gt_Twc_[anchor_frame_id].block(0,0,3,3);

                        std::cout<<"from venom1: "<<std::endl<<R_2_v*R_1_v.transpose()<<std::endl<<", from gt: "<<std::endl<<R_4_v*R_3_v.transpose()<<std::endl;
                        std::cout<<"from venom2: "<<std::endl<<R_2_v*R_1_v.inverse()<<std::endl<<", from gt: "<<std::endl<<R_4_v*R_3_v.inverse()<<std::endl;

                    }

                }

            }
        
        }

    };
}