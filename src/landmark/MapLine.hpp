/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 16:49:21
 * @LastEditTime: 2022-09-22 18:36:05
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/landmark/MapLine.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPLINE_HPP__
#define __VENOM_SRC_LANDMARK_MAPLINE_HPP__


#include "../estimator/Trajectory.hpp"


namespace simulator
{
    // Scalar -> Eigen::Vector3d  // Scalar -> cv::Matx31d
   class Trajectory; // Trajectory trajec_;

   class MapLine{
       public:
           // id of the mappoint
           int num_id_;
           // how many Trajectorys detect this point
           int observed;
           // endpoints parametrization in the world coordinate
           Eigen::Matrix<double,3,2> pos_world_;
           Eigen::Matrix<double,3,2> pos_world_noise_; 
           // plucker parametrization in the world coordinate
           Eigen::Matrix<double,3,2> plucker_world_;
           Eigen::Matrix<double,3,2> plucker_world_noise_;

           Trajectory * traject_; 
           /**
            * @brief The mappoint is detected by the i_Trajectory ^{th} Trajectory,
            *        and associated to the i_pixel ^{th} pixel. 
            */
           std::map<int /*id_poxe*/, int /*i_pixel*/> observations_;
           std::vector< std::pair<int, Eigen::Matrix<double,3,2>>> vec_obs_;  // noised observation
           std::vector< std::pair<int, Eigen::Matrix<double,3,2>>> vec_obs_gt_; // gt observation
          
           std::random_device rd;
           std::default_random_engine generator_;
           std::normal_distribution<double> pixel_n_;
      
       public:
           MapLine(const int id, Trajectory* traject):num_id_(id),observed(0),traject_(traject)
           {
               double max_nt = 0.1; double max_nq = 1.*M_PI/180.; double max_pixel_n = 1./240;
               std::random_device rd;
               std::default_random_engine generator(rd());
               std::normal_distribution<double> nt(0., max_nt);
               std::normal_distribution<double> nq(0., max_nq);
               std::normal_distribution<double> pixel_n(0, max_pixel_n);
 
               generator_ = generator;
               pixel_n_ = pixel_n;
           };

           void GenerateMapLine(const double distance, char axis)
           {
               std::uniform_real_distribution<double> point_generate( -4., 4. ); // width distribution
               if(axis == 'x')
               {
                pos_world_(0,0) = distance; pos_world_(0,1) = distance; 
                pos_world_(1,0) = point_generate(generator_); pos_world_(1,1) = pos_world_(1,0);
                pos_world_(2,0) = 2.; pos_world_(2,1) = -2.;

               }
               else if(axis == 'y')
               {
                pos_world_(0,0) = point_generate(generator_); pos_world_(0,1) = pos_world_(0,0);
                pos_world_(1,0) = distance; pos_world_(1,1) = distance; 
                pos_world_(2,0) = 2.; pos_world_(2,1) = -2.;
               }
           }
          
           /**
            * @brief reproject the 3D mapline to camera view, we than can obtain whether the mappoint can be detected or not.
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
                   //Eigen::Vector3d tcw = Tcw.block(0,3,3,1);
                   Eigen::Matrix<double,3,2> tcw;
                   tcw.block(0,0,3,1) = Tcw.block(0,3,3,1);
                   tcw.block(0,1,3,1) = Tcw.block(0,3,3,1);

                   Eigen::Matrix<double, 3,2> ob;
 
                   // in the camera coordinate
                   ob = Rcw * pos_world_ + tcw;
 
                   // 
                   if(ob(2,0) < 0) continue; // backside of the camera
                   if(ob(2,1) < 0) continue; //

                   ob.block(0,0,3,1) = ob.block(0,0,3,1) / ob(2,0); // 
                   ob.block(0,1,3,1) = ob.block(0,1,3,1) / ob(2,1); //      

                   // normalized image center 
                   Eigen::Vector3d center(0,0,1);
                   
                   //
                   Eigen::Matrix<double,3,2> ob_cam = ob;
                   ob_cam.normalize();
                   std::cout<<"\033[0;34m ob_cam: \033[0m"<<ob_cam<<std::endl;
                   
                   // angle between 光线 和 图像中心 之间的夹角。这个夹角太大，我们就认为观测不到了
                   double fov0 = 0; //std::acos(center.dot((ob_cam.block(0,0,3,1)))); 
                   fov0 = fov0 / M_PI * 180.;
                   if(fov0 > 60) continue;
                   fov0 = 0; //std::acos(center.dot((ob_cam.block(0,1,3,1)))); 
                   fov0 = fov0 / M_PI * 180.;
                   if(fov0 > 60) continue;

                   // key：camera id,  value：number of detected point
                   traject_->setKeyFrameDetects(i); 
                   //std::cout<<"the "<<i<<" th camera. "<<trajec_.contain_feature_cams_[i]<<std::endl;
                   observed++;
                  
                   // observation: <key: Trajectory_id, value: 该相机坐标系下的(x_0,y_0,1)>d
                   
                   vec_obs_gt_.emplace_back(i,ob); 
                   if(add_nose && vec_obs_gt_.size() > 1)
                       if (add_nose)
                       {
                           Eigen::Matrix<double,3,2> noise;
                           noise<< pixel_n_(generator_), pixel_n_(generator_),
                                   pixel_n_(generator_), pixel_n_(generator_),
                                   0, 0;
                           std::cout<<"noise:"<<noise<<std::endl;       
                           ob += noise;
                       }
                   vec_obs_.emplace_back(i,ob); //obs.emplace_back(i, ob);
               }
 
           }
 
           void print()
           {
               std::cout<<"\033[0;31m The global position of mappoint is  \033[0m"<<std::endl
                        <<pos_world_<<std::endl<< "which is detected by "<<observed<<"cameras"<<std::endl;
 
           }
   };
}

#endif // __VENOM_SRC_LANDMARK_MAPLINE_HPP__