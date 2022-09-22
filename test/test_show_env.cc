/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-22 16:10:56
 * @LastEditTime: 2022-09-22 18:28:56
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/test/test_show_env.cc
 */
#include<iostream>
#include<string>
#include "src/estimator/Trajectory.hpp" 
#include "src/landmark/MapPoint.hpp" 
#include "src/landmark/MapLine.hpp"
#include "src/optimizer/GlobalBundleAdjustment.hpp"
#include "src/feature/Reconstruct.hpp"
#include "src/visulizer/Visualizer.hpp"

int main(int argc, char **argv)
{
   if(argc!=1)
   {
        std::cout<<"usage: ./show_estimator_env "<<std::endl;
        return -1;
   }
   
   // keyframe generation
   int frame_num = 100;
   simulator::Trajectory* robot_trajectory = new simulator::Trajectory(1, frame_num);
   // generate a circular trajectory with 100 keyframes
   robot_trajectory->GenerateTrajectory(simulator::Trajectory::SPHERE, frame_num); 
   // robot_trajectory.PrintTrajectory();
 
   // landmarks generation
   double distance = 5.0; // distance between wall and the trajectory center
   bool add_noise_to_meas = true;

   // mappoints
   std::vector<Eigen::Vector3d> points_gt;
   std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> vec_meas_keyframe_mp;
   std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> vec_gt_keyframe_mp; 

   // maplines
   std::vector<Eigen::Matrix<double,3,2>> lines_gt; 
   std::vector<std::vector<std::pair<int, Eigen::Matrix<double,3,2>>>> vec_meas_keyframe_ml;
   std::vector<std::vector<std::pair<int, Eigen::Matrix<double,3,2>>>> vec_gt_keyframe_ml;

   

 
   for(int id=0; id<200; id++)
   {
       simulator::MapPoint * ptr_mp = new simulator::MapPoint(id, robot_trajectory);
       if(id<50)
           ptr_mp->GenerateMapPoint(distance, 'x');   //left side of the wall
       else if(id<100)
           ptr_mp->GenerateMapPoint(-distance, 'x');  //right side
       else if(id<150)
           ptr_mp->GenerateMapPoint(distance, 'y');   // front side
       else if(id<200)
           ptr_mp->GenerateMapPoint(-distance, 'y');  // back side
 
       ptr_mp->AddObservation( robot_trajectory->traject_gt_Twc_, add_noise_to_meas);   
       //ptr_mp->print();
       points_gt.push_back(ptr_mp->pos_world_);
       // vec_meas_keyframe_mp: mappoint_id<camera_id, mappoint_value>
       vec_meas_keyframe_mp.push_back(ptr_mp->obs);
       vec_gt_keyframe_mp.push_back(ptr_mp->obs_gt);
   }

   for(int id=0; id<40; id++)
   {
    simulator::MapLine * ptr_ml = new simulator::MapLine(id, robot_trajectory);
    if(id<10) 
        ptr_ml->GenerateMapLine(distance,'x');
    else if(id<20)
        ptr_ml->GenerateMapLine(-distance, 'x');
    else if(id<30)
        ptr_ml->GenerateMapLine(distance, 'y');
    else if(id<40)
        ptr_ml->GenerateMapLine(-distance, 'y'); 

    ptr_ml->AddObservation(robot_trajectory->traject_gt_Twc_, add_noise_to_meas);
    lines_gt.push_back(ptr_ml->pos_world_);
    vec_meas_keyframe_ml.push_back(ptr_ml->vec_obs_);
    vec_gt_keyframe_ml.push_back(ptr_ml->vec_obs_gt_);

   }
 
   simulator::Reconstruct recon;
   recon.Triangulation(vec_meas_keyframe_mp,robot_trajectory->traject_gt_Twc_);
  
   std::vector<Eigen::Matrix4d> vec_traject_Twc_opti;
   // simulator::pointLocalBundleAdjustment::optimize(recon.tri_point_xyz_, vec_meas_keyframe_mp, robot_trajectory.traject_gt_Twc_, vec_traject_Twc_opti);
 
   //  visualization
    simulator::Visualizer viewer;
    viewer.SetEnvParameter(points_gt,lines_gt, robot_trajectory->traject_gt_Twc_,
               vec_traject_Twc_opti, vec_meas_keyframe_mp,
               recon.tri_point_inverse_depth_, recon.tri_point_xyz_);
    viewer.show();
 
 
   //   
   return 0;
 
}
 
 

