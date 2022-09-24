/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 15:22:39
 * @LastEditTime: 2022-09-24 17:22:57
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

        Trajectory(const int type,  const int num_keyframe )
        {
            if(type==0)
            {
                traject_type_=CYCLE;
                std::cout<<"\033[0;34m [Venom Similator Printer] The Cycle trajectory with \033[0m"<< num_keyframe <<"\033[0;34m frames is generated.\033[0m"<< std::endl; 
            }
            else if(type==1)
            {
                traject_type_=SPHERE;
                std::cout<<"\033[0;34m [Venom Similator Printer] The Sphere trajectory with \033[0m"<< num_keyframe <<"\033[0;34m frames is generated.\033[0m"<< std::endl; 
            }
            else
                return; 
            GenerateTrajectory(traject_type_, num_keyframe);    
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
                std::cout << "We only have the CYCLE type trajectory currently." << std::endl;
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

            traject_gt_Twc_.push_back(Twc);
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

            traject_gt_Twc_.push_back(Twc);
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
            for (size_t i = 0; i < traject_gt_Twc_.size(); i++)
            {
                std::cout << "\033[0;35m This is the\033[0m" << i << " th camera: " << std::endl
                          << traject_gt_Twc_[i] << std::endl;
            }
        }

        void setKeyFrameDetects(int id)
        {
            contain_feature_cams_[id]++;
        }

    public:
        int num_id_; // Trajectory id
        int n_;      // landmarks detected by the Trajectory
        // Eigen::Matrix4d Twc_;
        // Eigen::Matrix3d Rwb_;
        // Eigen::Vector3d twb_;
        std::vector<Eigen::Matrix4d> traject_gt_Twc_;
        std::vector<std::pair<int, Eigen::Vector3d>> obs;
        std::vector<std::pair<int, Eigen::Vector3d>> obs_gt;
        TrajectoryType traject_type_;
        std::map<int, int> contain_feature_cams_;

    protected:
        Eigen::Vector3d pos_normalized_Trajectory;
        std::map<int /*pos_world_id*/,int /*measurements_id*/> associations;
    };
}

#endif // VENOM_SRC_TRAJECTORY_HPP__