/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 16:48:58
 * @LastEditTime: 2022-10-07 17:40:39
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

               // initialize 
               obs.reserve(trajec_->vec_traject_gt_Twc_.size());
               obs_gt.reserve(trajec_->vec_traject_gt_Twc_.size());
           }

           void GenerateMapPoint(const double distance, std::string axis)
           {
              
               std::uniform_real_distribution<double> point_generate( -3., 2. ); // width distribution
               if(axis == "vertical-left")
                   pos_world_ <<  distance, point_generate(generator_), point_generate(generator_);
               else if(axis == "vertical-right")
                   pos_world_ <<  point_generate(generator_), distance,  point_generate(generator_);
 
               // after generating the mappoint
               // add observatin  
               bool add_noise = true;
               //std::cout<<"keuframe size: "<<trajec_->vec_traject_gt_Twc_.size()<<std::endl;
               // AddObservation(trajec_->vec_traject_gt_Twc_, add_noise );
               //std::cout<<"add observation7"<<std::endl; 


           }
          
           /**
            * @brief reproject the 3D mappoint to camera view, we than can obtain whether the mappoint can be detected or not.
            *
            * @param keyframes_Twcs
            * @param add_noise : to generate noisy mappoint
            */
           void AddObservation(std::vector<Eigen::Matrix4d> keyframes_Twcs, bool add_noise)
           {
               for(int i = 0; i< keyframes_Twcs.size(); ++i)
               {
                   //std::cout<<i<<", "<< keyframes_Twcs.size()<<std::endl;
                   auto Twc = keyframes_Twcs[i];
                   Eigen::Matrix4d Tcw = Twc.inverse();
                   Eigen::Matrix3d Rcw = Tcw.block(0,0,3,3);
                   Eigen::Vector3d tcw = Tcw.block(0,3,3,1);
 
                   Eigen::Vector3d ob;
 
                   // in the camera coordinate
                   ob = Rcw * pos_world_ + tcw;
 
                   if(ob(2) < 0) continue; // backside of the camera

                   ob = ob / ob(2); // ob: [x, y, 1] 
                   
                   //  
                   double u = ob(0)*trajec_->camera_intrinsic_(0,0) + trajec_->camera_intrinsic_(0,2);
                   double v = ob(1)*trajec_->camera_intrinsic_(1,1) + trajec_->camera_intrinsic_(1,2);

                   // std::cout<<"u and v: "<< u <<", "<< v <<std::endl; 
                   if(!IsInFrame(u,v))
                   {
                    std::cout<<"u and v: "<< u <<", "<< v <<std::endl; 
                    continue;
                   } 

                   // TODO:  add noise

                   // YanyanTODO: CHECK observation     

                   // 光线 和 图像中心 之间的夹角。这个夹角太大，我们就认为观测不到了
                   Eigen::Vector3d center(0,0,1);
                   Eigen::Vector3d ob_cam = ob;
                   ob_cam.normalize();
                   double fov0 = std::acos(center.dot(ob_cam)); fov0 = fov0 / M_PI * 180.;
                   if(fov0 > 60) continue;
                   
                   //std::cout<<"sss"<<std::endl;
                   // i：camera id,  value：number of detected point
                   // this->num_id_: mappoint id
                   // ob_cam: measurement in this camera coordinate 
                   trajec_->SetKeyFrameDetects(i, this->num_id_, ob_cam);
                   //std::cout<<"the "<<i<<" th camera. "<<trajec_.contain_mp_cams_[i]<<std::endl;
                   observed++;
                  
                   // observation: <key: Trajectory_id, value: 该相机坐标系下的(x_0,y_0,1)>d
                   //std::cout<<" s"<<std::endl;
                   obs_gt.emplace_back(i, ob);
                   if(add_noise && obs_gt.size() > 1)
                   //            if(add_noise )
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

           bool IsInFrame(double &u, double &v)
           {
            if(u<0 || u> trajec_->image_cols_)
                return false;
            if(v<0 || v>trajec_->image_rows_)
                return false;

            return true;        
           }
   };
}

#endif // __VENOM_SRC_LANDMARK_MAPPOINT_HPP__