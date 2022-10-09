/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-18 03:29:37
 * @LastEditTime: 2022-10-09 15:35:22
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/feature/Reconstruct.hpp
 */
#ifndef __VENOM_SRC_FEATURE_RECONSTRUCTION_HPP__
#define __VENOM_SRC_FEATURE_RECONSTRUCTION_HPP__

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace simulator
{
    class Reconstruct
   {
       public:
           std::vector<double> tri_point_inverse_depth_;
           std::vector<Eigen::Vector3d> tri_point_xyz_; // reconstructured 3D mappoints
          
           
           /**
            * @brief This function is to reconstruct landmarks based on covisible measurements
            * 
            * @param point_obs 
            * @param Twcs 
            */
           void Triangulation(std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> point_obs,  std::vector<Eigen::Matrix4d> Twcs)
           {
               tri_point_xyz_= std::vector<Eigen::Vector3d>(point_obs.size(), Eigen::Vector3d::Zero());
               // observations
               for(auto &ob:point_obs)
               {
                   Eigen::Vector3d point_camera;
                   // less measurements
                   if(ob.size()<3)
                    continue;

                   Eigen::MatrixXd A(ob.size()*2, 4);
                   int index = 0;
                   Eigen::Matrix4d Twc0 = Twcs[ob[0].first];
                   Eigen::Vector3d ob_0 =  ob[0].second; 

                   for(int i=1; i<ob.size(); ++i)
                   {
                       Eigen::Vector3d ob0 = ob[i].second;
                       ob0 /=ob0(2,0);
                       // P = T_cs_c0 : from c0 to cs
                       Eigen::Matrix4d P = Twcs[ob[i].first].inverse()*Twc0; 
                       
                       // Eigen::Vector3d ob_00 = P.inverse().block(0,0,3,3)*ob0+ P.inverse().block(0,3,3,1);
                       // std::cout<<"camera 2: "<<ob_00<<std::endl;
                       // Eigen::Vector3d f = ob0/ ob0(2,0);//.normalized(); //.normalized();
                       // std::cout<<" f "<< f<< ", "<< ob0<<std::endl;
                       A.row(index++) = ob0(0,0) * P.row(2) - ob0(2,0) * P.row(0);
                       A.row(index++) = ob0(1,0) * P.row(2) - ob0(2,0) * P.row(1);
                    }
                   Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
                   point_camera = svd_V.head(3) / svd_V(3);
                   Eigen::Matrix3d Rwc = Twc0.block(0,0,3,3);
                   Eigen::Vector3d twc = Twc0.block(0,3,3,1);
                   Eigen::Vector3d point_w =  Rwc*point_camera+twc;
                   tri_point_inverse_depth_.push_back( 1/point_camera(2) );
                   tri_point_xyz_.push_back(point_w);
 
               }
 
           }
 
           void Triangulation2(std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> point_obs,  std::vector<Eigen::Matrix4d> Twcs)
           {
               for(auto &ob:point_obs)
               {
                   Eigen::Vector3d point_camera;
                   Eigen::MatrixXd A(ob.size()*2, 4);
                   int index = 0;
                   Eigen::Matrix4d Twc0 = Twcs[ob[0].first];
                   for(int i=1; i<ob.size(); ++i)
                   {
                       Eigen::Vector3d ob_cs = ob[i].second.col(0);
                       // P = T_cs_c0 : from c0 to cs
                       Eigen::Matrix4d P_cs_c0 = Twcs[ob[i].first].inverse()*Twc0;
                      
                       // on the unit sphere
                       Eigen::Vector3d f = ob_cs.normalized();
 
                       //f = [f0 f1 f2]^T
                       //AX=0      A = [A(2*i) A(2*i+1) A(2*i+2) A(2*i+3) ...]^T
                       //A(2*i)   = x(i) * P3 - z(i) * P1
                       //A(2*i+1) = y(i) * P3 - z(i) * P2
                      
                       A.row(index++) = f[0] * P_cs_c0.row(2) - f[2] * P_cs_c0.row(0);
                       A.row(index++) = f[1] * P_cs_c0.row(2) - f[2] * P_cs_c0.row(1);
 
                       // std::cout<<ob0<<std::endl;
                   }
                   Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
                   point_camera = svd_V.head(3) / svd_V(3);
                   Eigen::Matrix3d Rwc = Twc0.block(0,0,3,3);
                   Eigen::Vector3d twc = Twc0.block(0,3,3,1);
                   Eigen::Vector3d point_w =  Rwc*point_camera+twc;
                   tri_point_inverse_depth_.push_back( 1/point_camera(2) );
                   tri_point_xyz_.push_back(point_w);
               }
 
           }
 
   };
 
}


#endif //__VENOM_SRC_FEATURE_RECONSTRUCTION_HPP__