/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 16:48:58
 * @LastEditTime: 2022-09-20 16:25:29
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/landmark/MapPoint.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPPOINT_HPP__
#define __VENOM_SRC_LANDMARK_MAPPOINT_HPP__

#include "../estimator/Trajectory.hpp"

namespace simulator
{
    // Scalar -> Eigen::Vector3d  // Scalar -> cv::Matx31d
   class Trajectory; // Trajectory trajec_;

   class MapPoint{
       public:
           // id of the mappoint
           int num_id_;
           // how many Trajectorys detect this point
           int observed;
           // position in the world coordinate
           Eigen::Vector3d pos_world_;
           Eigen::Vector3d pos_world_noise_;
           Trajectory* trajec_;
           /**
            * @brief The mappoint is detected by the i_Trajectory ^{th} Trajectory,
            *        and associated to the i_pixel ^{th} pixel. 
            */
           std::map<int /*id_poxe*/, int /*i_pixel*/> observations_;
           std::vector< std::pair<int, Eigen::Vector3d>> obs;  // noised observation
           std::vector< std::pair<int, Eigen::Vector3d>> obs_gt; // gt observation
          
           std::random_device rd;
           std::default_random_engine generator_;
           std::normal_distribution<double> pixel_n_;
      
       public:
       
           MapPoint(const int id, Trajectory* trajec):num_id_(id),observed(0),trajec_(trajec)
           {
               double max_nt = 0.1; double max_nq = 1.*M_PI/180.; double max_pixel_n = 1./240;
               std::random_device rd;
               std::default_random_engine generator(rd());
               std::normal_distribution<double> nt(0., max_nt);
               std::normal_distribution<double> nq(0., max_nq);
               std::normal_distribution<double> pixel_n(0, max_pixel_n);
 
               generator_ = generator;
               pixel_n_ = pixel_n;
           }

           void GenerateMapPoint(const double distance, char axis)
           {
              
               std::uniform_real_distribution<double> point_generate( -4., 4. ); // width distribution
               if(axis == 'x')
                   pos_world_ <<  distance, point_generate(generator_), point_generate(generator_);
               else if(axis == 'y')
                   pos_world_ <<  point_generate(generator_), distance,  point_generate(generator_);
 
               // after generating the mappoint
               // add observatin
           }
          
           /**
            * @brief reproject the 3D mappoint to camera view, we than can obtain whether the mappoint can be detected or not.
            *
            * @param keyframes_Twcs
            * @param add_noise : to generate noisy mappoint
            */
 
           void AddObservation(std::vector<Eigen::Matrix4d> keyframes_Twcs, bool add_nose)
           {
               //
               for(int i = 0; i< keyframes_Twcs.size(); ++i)
               {
                   auto Twc = keyframes_Twcs[i];
                   Eigen::Matrix4d Tcw = Twc.inverse();
                   Eigen::Matrix3d Rcw = Tcw.block(0,0,3,3);
                   Eigen::Vector3d tcw = Tcw.block(0,3,3,1);
 
                   Eigen::Vector3d ob;
 
                   // in the camera coordinate
                   ob = Rcw * pos_world_ + tcw;
 
                   if(ob(2) < 0) continue; // backside of the camera
                   ob = ob / ob(2); // ob: [x, y, 1]
          
                   // 光线 和 图像中心 之间的夹角。这个夹角太大，我们就认为观测不到了
                   Eigen::Vector3d center(0,0,1);
                   Eigen::Vector3d ob_cam = ob;
                   ob_cam.normalize();
                   double fov0 = std::acos(center.dot(ob_cam)); fov0 = fov0 / M_PI * 180.;
                   if(fov0 > 60) continue;
 
                   // key：camera id,  value：number of detected point
                   trajec_->setKeyFrameDetects(i);
                   //std::cout<<"the "<<i<<" th camera. "<<trajec_.contain_feature_cams_[i]<<std::endl;
                   observed++;
                  
                   // observation: <key: Trajectory_id, value: 该相机坐标系下的(x_0,y_0,1)>d
                   obs_gt.emplace_back(i, ob);
                   if(add_nose && obs_gt.size() > 1)
                   //            if(add_nose )
                   {
                       Eigen::Vector3d noise ( pixel_n_(generator_),pixel_n_(generator_), 0 );
                       ob += noise;
                   }
                   obs.emplace_back(i, ob);
               }
 
           }
 
           void print()
           {
               std::cout<<"\033[0;31m [Venom Similator Printer] The global position of mappoint is  \033[0m"<<std::endl
                        <<pos_world_<<std::endl<< "which is detected by "<<observed<<"cameras"<<std::endl;
 
           }
   };
}

#endif // __VENOM_SRC_LANDMARK_MAPPOINT_HPP__