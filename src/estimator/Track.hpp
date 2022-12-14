/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-25 05:47:30
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Pose estimation based on measurements.
 * @FilePath: /venom/src/estimator/Track.hpp
 */

#include <algorithm>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "Trajectory.hpp"
#include "../landmark/MapVenom.hpp"
#include "../landmark/MapLine.hpp"
#include "../landmark/MapPoint.hpp"

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
        // landmarks in the map
        std::vector<MapLine *> vec_ptr_mls_;
        std::vector<MapPoint *> vec_ptr_mps_;

    public:
        std::vector<double> tri_point_inverse_depth_;
        std::vector<Eigen::Vector3d> tri_point_xyz_; // reconstructured 3D mappoints

    public:
        // track venom
        Track(Trajectory *robot_trajectory, std::vector<MapLine *> &vec_ptr_mls)
            : robot_traject_(robot_trajectory), vec_ptr_mls_(vec_ptr_mls){};

        // track point-line features
        Track(Trajectory *robot_trajectory, std::vector<MapLine *> &vec_ptr_mls, std::vector<MapPoint *> &vec_ptr_mps)
            : robot_traject_(robot_trajectory), vec_ptr_mls_(vec_ptr_mls), vec_ptr_mps_(vec_ptr_mps){};

        // detect venom in each frame
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
                std::map<int, Eigen::Vector3d> VenomFrame;
                //
                std::vector<Eigen::Vector3d> venom_axis;
                venom_axis = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero());
                std::pair<int /*venom_id*/, Eigen::Matrix3d> r_venom_frame;

                //--> loop for detecting the number of vanishing directions
                for (int observ_ml_i = 0, observ_ml_end = robot_traject_->obs_line_[frame_id].size(); observ_ml_i < observ_ml_end; observ_ml_i++)
                { 
                    int ml_id = robot_traject_->obs_line_[frame_id][observ_ml_i].first;
                    Eigen::Matrix<double, 3, 2> ml_pos_cam = robot_traject_->obs_line_[frame_id][observ_ml_i].second;
                    
                    // detected vanishing direction (maplines)
                    Eigen::Vector3d vd = (ml_pos_cam.block(0, 0, 3, 1) - ml_pos_cam.block(0, 1, 3, 1)).normalized();

                    // the type of vanishing direction:   For association
                    MapLine *ptr_mp_i = vec_ptr_mls_[ml_id];
                    int vd_type = ptr_mp_i->vanishing_direction_type_;

#ifdef __VERBOSE__OFF //(remove "OFF", if you want to print )
                    std::cout << "\033[0;35m [Venom Simulator Printer]: Vanishing direction type is \033[0m" << ptr_mp_i->vanishing_direction_type_ << std::endl;
#endif
                    // add different typies of vanishing direction into the set
                    if (!set_vd_type.count(vd_type))
                    {
                        // new vd
                        vec_type_direction.push_back(std::make_pair(vd_type, vd)); // [vd_type] = vd;
                        // std::cout<<"vd: "<<vd<<std::endl;
                        // vec_vd.push_back(vd);
                        set_vd_type.insert(vd_type);
                    }
                }

                //--> more than two unparallel directions
                if (set_vd_type.size() >= 2) // Vanom is detected
                {
                    //--> detected
                    int venom_type = VenomFrameGeneration(vec_type_direction, venom_axis);
                    // Rcm
                    Eigen::Matrix3d rotation_frame_venom_cam_id = RotationCamera2Venom(venom_type, venom_axis);
#ifdef __VERBOSE__
                    std::cout << "\033[0;35m [Venom Simulator Printer]: venom is detected. " << std::endl
                              << "The venom_type is " << venom_type
                              << ". And the rotation from camera to venom is \033[0m" << std::endl
                              << rotation_frame_venom_cam_id << std::endl;
#endif
                    robot_traject_->SetKeyFrameDetects(frame_id, venom_type, rotation_frame_venom_cam_id);
                }
            }
        }

        /**
         * @brief
         *
         * @param type_detections   used for association by setting a symbol. Same symbol means that they detect the same venom.
         * @param venom_axis : Rcm  from venom to camera
         * @return int
         */
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
                if (mit->first == 0)
                {
                    venom_axis[0] = mit->second;
                    venom_type = 0;
                }
                else if (mit->first == 1)
                {
                    if (venom_type == 0)
                    {
                        venom_type = 1; // means: first-0, second-1
                        venom_axis[1] = (mit->second - mit->second.dot(venom_axis[0]) / (venom_axis[0].dot(venom_axis[0])) * venom_axis[0]).normalized();

                        assert(venom_axis[1].dot(venom_axis[0]) < 1e-6);
                    }
                    else if (venom_type == -1) // means: first-1, future-second-2
                    {
                        venom_type = 4;
                        venom_axis[1] = mit->second;
                    }
                }
                else if (mit->first == 2)
                {
                    // venom_axis[2] = mit->second;
                    if (venom_type == 0) // means: first-0, second-2
                    {
                        venom_type = 2;
                        venom_axis[2] = (mit->second - mit->second.dot(venom_axis[0]) / (venom_axis[0].dot(venom_axis[0])) * venom_axis[0]).normalized();
                        assert(venom_axis[2].dot(venom_axis[0]) < 1e-6);
                    }
                    else if (venom_type == 1) // means: first-0, second-1, third-2
                    {
                        venom_type = 3;
                        venom_axis[2] = (mit->second - mit->second.dot(venom_axis[0]) / (venom_axis[0].dot(venom_axis[0])) * venom_axis[0] -
                                         mit->second.dot(venom_axis[1]) / (venom_axis[1].dot(venom_axis[1])) * venom_axis[1])
                                            .normalized();
                        assert(venom_axis[2].dot(venom_axis[0]) < 1e-6 && venom_axis[2].dot(venom_axis[1]) < 1e-6);
                    }
                    else
                    {
                        std::cerr << "problem happpens" << std::endl;
                    }
                }
            }
            assert(venom_type >= 1 && venom_type <= 4);

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
            // cv::SVD svd; cv::Mat U,W,VT, R_cm_ortho;
            // cv::eigen2cv(R_cm, R_cm_ortho);
            // svd.compute(R_cm_ortho,W,U,VT);
            // R_cm_ortho=U*VT;
            // cv::cv2eigen(R_cm_ortho,R_cm);
            // std::cout<<"R:"<<R_cm<<", "<<R_cm_ortho<<std::endl;
            std::cout << "rotation venom to camera:"<<R_cm << std::endl;

            return R_cm;
        }

        void VenomAssociation()
        {
            //
            std::set<int> set_new_venom;
            std::vector<simulator::MapVenom *> vec_ptr_mv;
            for (int frame_id = 0, frame_id_end = robot_traject_->vec_traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
            {

                assert(robot_traject_->contain_mw_cams_[frame_id] <= 1);

                if (robot_traject_->obs_mw_[frame_id].size())
                {
                    int venom_type = robot_traject_->obs_mw_[frame_id][0].first;

                    // std::cout<<"the venom_type is "<<venom_type<<std::endl;
                    if (!set_new_venom.count(venom_type)) // the first time deteced
                    {
                        // create a new mapvenom
                        simulator::MapVenom *ptr_mv = new simulator::MapVenom(frame_id, venom_type);

                        vec_ptr_mv.push_back(ptr_mv);

                        set_new_venom.insert(venom_type);
                    }
                }
            }

#ifdef __VERBOSE__
            std::cout << "vec venom" << vec_ptr_mv.size() << std::endl;
            for (int i = 0, i_end = vec_ptr_mv.size(); i < i_end; i++)
            {
                std::cout << "the venom id is " << vec_ptr_mv[i]->num_id_ << std::endl;
            }
#endif
            // TODO: associate each frame to mv
            for (int i = 0, i_end = vec_ptr_mv.size(); i < i_end; i++)
            {

                int venom_type = vec_ptr_mv[i]->venom_type_;
                int anchor_frame_id = vec_ptr_mv[i]->anchor_frame_id_;
                for (int frame_id = 0, frame_id_end = robot_traject_->vec_traject_gt_Twc_.size(); frame_id < frame_id_end; frame_id++)
                {
                    if (venom_type == robot_traject_->obs_mw_[frame_id][0].first)
                    {
                        if (frame_id == anchor_frame_id)
                            continue;
                        assert(frame_id >= anchor_frame_id);
                        vec_ptr_mv[i]->AddObservation(frame_id, robot_traject_->obs_mw_[frame_id][0].second);
                        vec_ptr_mv[i]->observed_ += 1;

                        // compute rotation

                        Eigen::Matrix3d R_1_v = robot_traject_->obs_mw_[frame_id][0].second;
                        Eigen::Matrix3d R_2_v = robot_traject_->obs_mw_[anchor_frame_id][0].second;

                        //
                        Eigen::Matrix3d R_3_v = robot_traject_->vec_traject_gt_Twc_[frame_id].block(0, 0, 3, 3);
                        Eigen::Matrix3d R_4_v = robot_traject_->vec_traject_gt_Twc_[anchor_frame_id].block(0, 0, 3, 3);

                        // std::cout<<"from venom1: "<<std::endl<<R_2_v*R_1_v.transpose()<<std::endl<<", from gt: "<<std::endl<<R_4_v*R_3_v.transpose()<<std::endl;
                        // std::cout<<"from venom2: "<<std::endl<<R_2_v*R_1_v.inverse()<<std::endl<<", from gt: "<<std::endl<<R_4_v*R_3_v.inverse()<<std::endl;
                    }
                }
            }
        }
        
        /**
         * @brief 
         * 
         * @param point_obs 
         * @param Twcs 
         */
        void Triangulation(std::vector/*mappoint_id*/<std::vector/*<frame_id, mp_c>*/<std::pair<int, Eigen::Vector3d>>> point_obs, std::vector<Eigen::Matrix4d> Twcs)
        {
            tri_point_xyz_ = std::vector<Eigen::Vector3d>(point_obs.size(), Eigen::Vector3d::Zero());
            // observations
            for (auto &ob : point_obs)
            {
                Eigen::Vector3d point_camera;
                // less measurements
                if (ob.size() < 3)
                    continue;

                Eigen::MatrixXd A(ob.size() * 2, 4);
                int index = 0;
                Eigen::Matrix4d Twc0 = Twcs[ob[0].first];
                Eigen::Vector3d ob_0 = ob[0].second;

                for (int i = 1; i < ob.size(); ++i)
                {
                    Eigen::Vector3d ob0 = ob[i].second;
                    ob0 /= ob0(2, 0);
                    // P = T_cs_c0 : from c0 to cs
                    Eigen::Matrix4d P = Twcs[ob[i].first].inverse() * Twc0;

                    // Eigen::Vector3d ob_00 = P.inverse().block(0,0,3,3)*ob0+ P.inverse().block(0,3,3,1);
                    // std::cout<<"camera 2: "<<ob_00<<std::endl;
                    // Eigen::Vector3d f = ob0/ ob0(2,0);//.normalized(); //.normalized();
                    // std::cout<<" f "<< f<< ", "<< ob0<<std::endl;
                    A.row(index++) = ob0(0, 0) * P.row(2) - ob0(2, 0) * P.row(0);
                    A.row(index++) = ob0(1, 0) * P.row(2) - ob0(2, 0) * P.row(1);
                }
                Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
                point_camera = svd_V.head(3) / svd_V(3);
                Eigen::Matrix3d Rwc = Twc0.block(0, 0, 3, 3);
                Eigen::Vector3d twc = Twc0.block(0, 3, 3, 1);
                Eigen::Vector3d point_w = Rwc * point_camera + twc;
                tri_point_inverse_depth_.push_back(1 / point_camera(2));
                tri_point_xyz_.push_back(point_w);
            }
        }

        void SaveFrameGTTrajectoryLovelyTUM(const std::string &filename)
        {
            std::cout << "\033[0;33m [Venom Simulator Printer] Saving keyframe trajectory to " << filename << ".\033[0m" << std::endl;
            
            std::ofstream pose_file;
            pose_file.open(filename);
            for (size_t i = 0, i_end = robot_traject_->vec_traject_gt_Twc_.size(); i < i_end; i++)
            {
                Eigen::Matrix4d Twc_i = robot_traject_->vec_traject_gt_Twc_[i];
                Eigen::Matrix3d R = Twc_i.block(0,0,3,3);

                Eigen::Quaterniond quat(R);
                Eigen::Vector3d trans = Twc_i.block(0, 3, 3, 1);
                pose_file << i << " " << trans(0) << " " << trans(1) << " " << trans(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
            }

            pose_file.close();
        }

        void SaveFramePredictedTrajectoryLovelyTUM(const std::string &filename, std::vector<std::pair<int/*frame_id*/,  Eigen::Matrix4d/*frame_pose*/>> &vec_Twcs)
        {
            std::cout << "\033[0;33m [Venom Simulator Printer] Saving keyframe trajectory to " << filename << ".\033[0m" << std::endl;
            
            std::ofstream pose_file;
            pose_file.open(filename);
            for (size_t i = 0, i_end = vec_Twcs.size(); i < i_end; i++)
            {   
                int frame_id = vec_Twcs[i].first;
                Eigen::Matrix4d  Twc_i = vec_Twcs[i].second;
                Eigen::Matrix3d R = Twc_i.block(0,0,3,3);

                Eigen::Quaterniond quat(R);
                Eigen::Vector3d trans = Twc_i.block(0, 3, 3, 1);
                pose_file << frame_id << " " << trans(0) << " " << trans(1) << " " << trans(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
            }

            pose_file.close();
        }
    };
}