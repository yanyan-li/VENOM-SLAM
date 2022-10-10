/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 15:22:39
 * @LastEditTime: 2022-10-10 16:31:17
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: This class is to generate different types of camera poses. 
 * @FilePath: /venom/src/estimator/Trajectory.hpp
 */
#ifndef __VENOM_SRC_TRAJECTORY_HPP__
#define __VENOM_SRC_TRAJECTORY_HPP__ 

#include <map>
#include <vector>
#include <iostream>
#include <random>
 
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
 

namespace simulator
{
    class Trajectory
    {
    public:
        
        enum TrajectoryType
        {
            CYCLE = 0,
            SPHERE = 1
            // RECTANGLE = 2,
            // HEXAGON = 3,
            // LINE = 4
        };

        /**
         * @brief Initialization
         * 
         * @param type 
         * @param num_keyframe 
         */
        Trajectory(const int type,  const int num_keyframe )
        {   
            obs = std::vector< std::vector<std::pair<int /* */, Eigen::Vector3d>>>(num_keyframe, std::vector<std::pair<int /* */, Eigen::Vector3d>>());//.reserve(num_keyframe);
            obs_line_ = std::vector< std::vector<std::pair<int /* */, Eigen::Matrix<double,3,2>>>>(num_keyframe, std::vector<std::pair<int /* */, Eigen::Matrix<double,3,2>>>());//.reserve(num_keyframe);
            obs_mw_ = std::vector< std::vector<std::pair<int /* */, Eigen::Matrix3d>>>(num_keyframe, std::vector<std::pair<int /* */, Eigen::Matrix3d>>());//.reserve(num_keyframe);


            //vec_traject_gt_Twc_ = std::vector<Eigen::Matrix4d>(2*num_keyframe, Eigen::Matrix4d::Zero()) ;
            if(type==0)
            {
                traject_type_=CYCLE;
                std::cout<<"\033[0;35m [Venom Similator Printer] The Cycle trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }
            else if(type==1)
            {
                traject_type_=SPHERE;
                std::cout<<"\033[0;35m [Venom Similator Printer] The Sphere trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }
            else
                return; 
        };

        bool GenerateTrajectory(const TrajectoryType &traject_type, const int &num_keyframe)
        {
            if (num_keyframe < 2)
                return false;
            if (traject_type == CYCLE)
            {
                double r = 3.0;
                int wave_num = 10;
                double wave_high = 1.5;
                GenerateCycleKeyFrames(num_keyframe, wave_num, wave_high, r);
            }
            else if (traject_type == SPHERE)
            {
                double r = 3.0;

                GenerateSphereKeyFrames(num_keyframe, r);
            }
            else
            {
                std::cout << "\033[0;35m [Venom Similator Printer] We only have the CYCLE type trajectory currently.\033[0m" << std::endl;
                return false;
            }

            return true;
        } 

        void CyclePoseGeneration(double theta, double wave_theta, double step, int radius, double wave_high)
        {
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            double theta_ = 0;
            if (theta < M_PI / 2.)
                theta_ = theta + M_PI / 2.;
            else if (theta >= M_PI / 2. && theta < M_PI)
                theta_ = theta - M_PI * 3. / 2.;
            else if (theta >= M_PI && theta < M_PI * 3. / 2.)
                theta_ = theta + M_PI / 2.;
            else if (theta >= M_PI * 3. / 2. && theta < M_PI * 2)
                theta_ = theta - 3. * M_PI / 2.;
            Eigen::Matrix3d Rwb;
            Rwb = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-M_PI/2.-M_PI/6., Eigen::Vector3d::UnitX());
            Eigen::Vector3d twb = Eigen::Vector3d(radius * cos(theta), radius * sin(theta), wave_high * std::cos(wave_theta));

            Twc.block(0, 0, 3, 3) = Rwb;
            Twc.block(0, 3, 3, 1) = twb;

            vec_traject_gt_Twc_.push_back(Twc);
        }
        void SpherePoseGeneration(double radius, int nums_points, int index)
        {
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            double phi = acos(-1.0 + (2.0 * (index + 1) - 1.0) / nums_points);
            double theta = sqrt(nums_points * M_PI) * phi;
            double t_x = radius * cos(theta) * sin(phi);
            double t_y = radius * sin(theta) * sin(phi);
            double t_z = radius * cos(phi);
            Eigen::Vector3d twb;
            twb << t_x, t_y, t_z;

            Eigen::Matrix3d Rwb;
            Rwb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());

            Twc.block(0, 0, 3, 3) = Rwb;
            Twc.block(0, 3, 3, 1) = twb;

            vec_traject_gt_Twc_.push_back(Twc);
        }

        bool GenerateCycleKeyFrames(const int frame_num, int wave_num, double wave_high, double radius_cycle = 0)
        {
            if (frame_num < 2)
                return false;

            for (int n = 0; n < frame_num; n++) // camera id from n=0 to n=frameNum-1
            {
                int radius = radius_cycle;
                //double wave_high = 0.75;
                double theta = n * 2. * M_PI / frame_num;
                double step = n * 1. / frame_num;
                double wave_theta = n * 2. * M_PI / (frame_num / wave_num);

                CyclePoseGeneration(theta, wave_theta, step, radius, wave_high);
            }
        }
        bool GenerateSphereKeyFrames(const int frame_num, double radius)
        {
            if (frame_num < 2)
                return false;
            for (int n = 0; n < frame_num; n++)
            {
                double radius = 2.0;
                double theta = n * 2. * M_PI / frame_num;
                SpherePoseGeneration(radius, frame_num, n);
            }
        }


        void PrintTrajectory()
        {
            for (size_t i = 0; i < vec_traject_gt_Twc_.size(); i++)
            {
                std::cout << "\033[0;35m This is the\033[0m" << i << " th camera: " << std::endl
                          << vec_traject_gt_Twc_[i] << std::endl;
            }
        }

        /**
         * @brief Set the Key Frame Detects object
         * 
         * @param frame_id 
         * @param mappoint_id 
         * @param measument 
         */
        void SetKeyFrameDetects(int frame_id, int mappoint_id, Eigen::Vector3d measument)
        {   
            // the id th cam detects n features  
            contain_mp_cams_[frame_id]++;
            //std::cout<<"frame_id: "<<frame_id <<", "<<contain_mp_cams_[frame_id]<<std::endl;
            // add observation
            AddObservation(frame_id, mappoint_id, measument);
        }
        /**
         * @brief Set the Key Frame Detects object
         * 
         * @param frame_id 
         */
        void SetKeyFrameDetects(int frame_id, int mapline_id, Eigen::Matrix<double, 3, 2> ml_measument)
        {   
            // the id th cam detects n features  
            contain_ml_cams_[frame_id]++;
            AddObservation(frame_id, mapline_id, ml_measument);
        }
        /**
         * @brief Set the Key Frame Detects object
         * 
         * @param frame_id 
         * @param manhattanworld_id 
         * @param mw_measument 
         */
        void SetKeyFrameDetects(int frame_id, int manhattanworld_id, Eigen::Matrix3d mw_measument)
        {
            contain_mw_cams_[frame_id]++;
            AddObservation(frame_id, manhattanworld_id, mw_measument);
        }
        // 
        void AddObservation(int frame_id, int mappoint_id, Eigen::Vector3d measurement)
        {
            //std::cout<<"frame if"<< frame_id <<", "<<obs.size()<<", "<< mappoint_id<<", "<<measurement<<std::endl;
            obs[frame_id].push_back(std::make_pair( mappoint_id, measurement)); ///std::pair<mappoint_id, measurement>);
            //std::cout<<"frame if:"<<obs.size()<<std::endl;
        }
        void AddObservation(int frame_id, int mapline_id, Eigen::Matrix<double,3,2> measurement)
        {
            obs_line_[frame_id].push_back(std::make_pair(mapline_id, measurement));
        }
        void AddObservation(int frame_id, int manhattanworld_id, Eigen::Matrix3d mw_measurement)
        {
            obs_mw_[frame_id].push_back(std::make_pair(manhattanworld_id, mw_measurement));
        }

    public:
        int num_id_; // Trajectory id
        int n_;      // landmarks detected by the Trajectory
        // Eigen::Matrix4d Twc_;
        // Eigen::Matrix3d Rwb_;
        // Eigen::Vector3d twb_;
        std::vector<Eigen::Matrix4d> vec_traject_gt_Twc_;

        // the i th cam, detects the id th mappoint, with the 3D coordinate
        std::vector< std::vector<std::pair<int /* */, Eigen::Vector3d>>> obs;
        std::vector< std::vector<std::pair<int, Eigen::Vector3d>>> obs_gt; 

        // the i th cam, detects the id th mapline, with the 3D coordinate
        std::vector< std::vector<std::pair<int /* */, Eigen::Matrix<double,3,2>>>> obs_line_;
        std::vector< std::vector<std::pair<int, Eigen::Matrix<double,3,2>>>> obs_line_gt_;

        // the i th cam, detects the id th ManhattanWorld, with the RO3 coordinate
        std::vector< std::vector< std::pair<int, Eigen::Matrix3d>>> obs_mw_; 
        std::vector< std::vector< std::pair<int, Eigen::Matrix3d>>> obs_mw_gt_; 


        // std::vector<std::pair<int /* */, int> >  

        TrajectoryType traject_type_;
        // key: frame_id; value: primitive_nums
        std::map<int, int> contain_mp_cams_;
        std::map<int, int> contain_ml_cams_;
        std::map<int, int> contain_mw_cams_;

    protected:
        Eigen::Vector3d pos_normalized_Trajectory;
        std::map<int /*pos_world_id*/,int /*measurements_id*/> associations;
    };
}

#endif // VENOM_SRC_TRAJECTORY_HPP__