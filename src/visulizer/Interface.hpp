/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:37:57
 * @LastEditTime: 2022-10-16 15:22:03
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/visulizer/Interface.hpp
 */
#ifndef __VENOM_SRC_VISUALIZER_INTERFACE_HPP__
#define __VENOM_SRC_VISUALIZER_INTERFACE_HPP__ 

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>  
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "src/estimator/Trajectory.hpp"
#include "src/estimator/Track.hpp"
#include "src/landmark/MapPoint.hpp"
#include "src/landmark/MapLine.hpp"
#include "src/optimizer/GlobalBundleAdjustment.hpp"
#include "src/feature/Reconstruct.hpp"


namespace simulator
{
    class Interface
   {
 
       public:
           
           simulator::Trajectory* ptr_robot_trajectory_; // 
           simulator::Track*  ptr_tracker_; //
           // simulator::Reconstruct recon;

           // 3D landmarks of the environment
           std::vector<Eigen::Vector3d> points_gt_;
           std::vector<Eigen::Matrix<double, 3, 2>> lines_gt_;
           // trajectory of the robot
           std::vector<Eigen::Matrix4d> Twcs_true_;
           std::vector<Eigen::Matrix4d> Twcs_;
           // 
           std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> point_obs_;
           std::vector<double> tri_point_inverse_depth_;
           std::vector<Eigen::Vector3d> tri_point_xyz_;
        
        private:
            //--> parameters
            int frame_num_ = 100;
            int traject_type_;
            int vert_lines_;
            int horiz_lines_; 
            int vert_points_;
            int horiz_points_; 
            double distance = 5.0; // distance between wall and the trajectory center
            bool add_noise_to_meas = true;

            //--> keyframe generation
            // generate a circular trajectory with 100 keyframes

            //--> landmarks generation
            // mappoints
            std::vector<simulator::MapPoint *> vec_ptr_mappoints_;
            std::vector<Eigen::Vector3d> points_gt;
            std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_meas_keyframe_mp;
            std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> vec_gt_keyframe_mp;
            // maplines
            std::vector<simulator::MapLine *> vec_ptr_maplines;
            std::vector<Eigen::Matrix<double, 3, 2>> lines_gt;
            std::vector<std::vector<std::pair<int, Eigen::Matrix<double, 3, 2>>>> vec_meas_keyframe_ml;
            std::vector<std::vector<std::pair<int, Eigen::Matrix<double, 3, 2>>>> vec_gt_keyframe_ml;

        public:
            void StartVenom()
            {
                //--> interface
                pangolin::CreateWindowAndBind(" VENOM (0.0.2) SLAM (backend) Simulator", 1024, 768);
                // 3D Mouse handler requires depth testing to be enabled
                glEnable(GL_DEPTH_TEST);
                // Issue specific OpenGl we might need
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                    pangolin::ModelViewLookAt(0, -0.7, 10.8, 0, 0, -50.0, 0.0, 10.0, 10.0));
                // Add named OpenGL viewport to window and provide 3D Handler
                pangolin::View &d_cam = pangolin::CreateDisplay()
                                            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                            .SetHandler(new pangolin::Handler3D(s_cam));

                //
                pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(280));

                pangolin::Var<bool> menuWhite("menu.White Background", false, true);

                pangolin::Var<int> traject_num("menu.KeyFrame Number", 100, 50, 200);
                pangolin::Var<int> traject_type("menu.Trajectory Type", 0, 0, 4);
                pangolin::Var<int> vert_points("menu.Vertical Points", 20, 4, 100);
                pangolin::Var<int> horiz_points("menu.Horizontal Points", 10, 0, 20);
                pangolin::Var<int> vert_lines("menu.Vertical Lines", 20, 4, 50);
                pangolin::Var<int> horiz_lines("menu.Horizontal Lines", 10, 0, 50);

                pangolin::Var<bool> set_env("menu.Build&Fix Environment", false, false);
                pangolin::Var<int> set_traject_num("menu.Num of KeyFrames:", 0);
                pangolin::Var<int> set_traject_type("menu.Selected Trajectory Type:", 0);
                pangolin::Var<int> set_vert_points("menu.Num of Vertical Points:", 0);
                pangolin::Var<int> set_horiz_points("menu.Num of Horizontal Points:", 0);
                pangolin::Var<int> set_vert_lines("menu.Num of Vertical Lines:", 0);
                pangolin::Var<int> set_horiz_lines("menu.Num of Horizontal Lines:", 0);

                pangolin::Var<bool> start_traject_env("menu.Show Cameras & Environment", false, false);
                pangolin::Var<bool> menuShowTrajectory("menu.Show TrajectoryGT", false, true);
                pangolin::Var<bool> menuShowPoint("menu.Groudtruth Point", false, true);
                pangolin::Var<bool> menuShowLine("menu.Groudtruth Line", false, true);

                pangolin::Var<bool> associate_meas("menu.Measuments and 3D Recon", false, false);
                pangolin::Var<bool> menuShowPointRecon("menu.Reconstructed Point", false, true);
                pangolin::Var<bool> menuShowLineRecon("menu.Reconstructed Line", false, true);

                pangolin::Var<bool> rotation_esti("menu.Rot Esti based on E-Graph",false, false );
                pangolin::Var<bool> menuShowEGraph("menu.Anchor Frames in E-Graph", false, true);
                pangolin::Var<bool> menuShowRefinedCamera("menu.Rotation Estimation", false, true);

                
                // pangolin::Var<bool> optimization("menu.Optimization", false, false);
                // pangolin::Var<bool> menuShowTrajectoryOpti("menu.Show TrajectoryOpti", false, true);
                // pangolin::Var<bool> meanShowPointOpti("menu.Optimize Point", false, true);

                pangolin::OpenGlMatrix Twc;
                Twc.SetIdentity();
                std::vector<pangolin::OpenGlMatrix> Twcs;
                Twcs.push_back(Twc);
                
                // bool bFollow = true;
                // bool bLocalizationMode = false;
                // int Single_rpr_id = 0;
                // int Single_line_id = 0;
                // bool line_id_change = false;
                // int cut_i = 0;

                // click control
                bool set_para_click_once = true;
                bool set_start_means_click_once = true;
                bool set_association_click_once = true;
                bool set_optimization_click_here = true;
                bool set_egraph_associate_click_here = true;

                while (!pangolin::ShouldQuit())
                {
                    // clear
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    d_cam.Activate(s_cam);
                    //-------> background
                    if (menuWhite)
                        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                    else
                        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

                    //----step1---> set parameters via buttons
                    if (set_para_click_once)
                    {
                        if (set_env)
                        {
                            // show env parameters
                            set_traject_num = traject_num;
                            set_traject_type = traject_type;
                            set_vert_points = vert_points;
                            set_horiz_points = horiz_points;
                            set_vert_lines = vert_lines;
                            set_horiz_lines = horiz_lines;
                            std::cout << std::endl
                                      << "\033[0;35m[Venom Similator Printer] The Environment was constructed. \033[0m" << std::endl;
                            set_para_click_once = false;
                            // set config parameters for env
                            InputSystemParameters(traject_num, traject_type, vert_lines, horiz_lines, vert_points,horiz_points);
                            SaveSystemConfig("system_parameter.txt");
                        }
                    }

                    //----step2---> set parameters to those SLAM modules
                    if (set_start_means_click_once && !set_para_click_once) // finish set_paras
                    {
                        if (start_traject_env)
                        {
                            // start signal for drawing env&pose
                            SetSystemParameters();
                            // show the set parameters
                            ShowFixedTrajecEnv(points_gt, lines_gt, ptr_robot_trajectory_->vec_traject_gt_Twc_);
                            
                            menuShowTrajectory = true;
                            menuShowPoint = true;
                            menuShowLine = true;
                            std::cout << std::endl
                                      << "\033[0;35m [Venom Similator Printer] Show environments and cameras. \033[0m" << std::endl;
                            set_start_means_click_once = false;
                        }
                    }

                    //----step3---> build 2D measurement and 3D relationships for each camera
                    if (set_association_click_once && !set_start_means_click_once) // finish start_means
                    {
                        if (associate_meas)
                        {
                            // camera-landmark association
                            // 
                            // start the track module
                            StartTracking();
                            
                            // mappoint reconstruction based on observations 
                            ptr_tracker_->Triangulation(vec_meas_keyframe_mp, ptr_robot_trajectory_->vec_traject_gt_Twc_);
                            
                            ShowReconTrajecEnv(ptr_robot_trajectory_->obs, ptr_tracker_->tri_point_inverse_depth_, ptr_tracker_->tri_point_xyz_ );

                            menuShowPointRecon = true;
                            menuShowLineRecon = true;
                            std::cout << std::endl
                                      << "\033[0;35m [Venom Similator Printer] Associate landmarks to each camera. \033[0m" << std::endl;
                            set_association_click_once = false;
                        }
                    }


                    //-------> Rotation estimation based on E-Graph
                    if(set_egraph_associate_click_here &&!set_association_click_once)
                    {
                        if(rotation_esti)
                        {
                            ptr_tracker_->VenomFrameDetection();
                            //ptr_tracker_->VenomAssociation();
                            menuShowEGraph = false; 
                            menuShowRefinedCamera = false;
                            std::cout << std::endl
                                      << "\033[0;35m [Venom Similator Printer] Extensibility Graph Generation. \033[0m" << std::endl;
                            
                            set_egraph_associate_click_here = false;

                        }
                    }



                    // if (set_optimization_click_here && !set_association_click_once) // finish association
                    // {
                    //     if (optimization)
                    //     {
                    //         // optimization
                    //         // TODO:
                    //         // menuShowTrajectoryOpti = true;
                    //         // meanShowPointOpti = true;
                    //         std::cout << std::endl
                    //                   << "\033[0;35m [Venom Similator Printer] Optimize reconstructed landmarks and cameras. \033[0m" << std::endl;
                    //         set_optimization_click_here = false;
                    //     }
                    // }

                    const double lens = 0.5;
                    glLineWidth(2.0);
                    glBegin(GL_LINES);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glVertex3f(0, 0, 0);
                    glVertex3f(lens, 0, 0);

                    glColor3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, lens, 0);

                    glColor3f(0.0f, 0.0f, 1.0f);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, 0, lens);

                    glEnd();

                    if (menuShowPoint)
                    {
                        glPointSize(5);
                        glBegin(GL_POINTS);
                        glColor3f(1.0, 0.0, 0.0);
                        for (auto p : points_gt_)
                        {
                            glVertex3d(p[0], p[1], p[2]);
                        }
                        glEnd();
                    }

                    if (menuShowPointRecon)
                        DrawReconstructedMapPoint();

                    if(menuShowEGraph)
                    {
                        std::vector<pangolin::OpenGlMatrix> MsTrue;
                        CallTrajectoryTrue(MsTrue, Twcs_true_);
                        ShowVenomAnchors(MsTrue);
                    }

                    // if (meanShowPointOpti)
                    // {
                    //     // call bundleadjustment functions

                    //     for (int i = 0; i < point_obs_.size(); ++i)
                    //     {
                    //         const auto &obs = point_obs_[i];
                    //         const auto &ob = obs.begin();
                    //         const int cam_id = ob->first;

                    //         glPointSize(5);
                    //         glBegin(GL_POINTS);
                    //         glColor3f(1.0, 1.0, 1.0);
                    //         glVertex3d(tri_point_xyz_[i](0, 0), tri_point_xyz_[i](1, 0), tri_point_xyz_[i](2, 0));
                    //         glEnd();
                    //     }
                    // }

                    if (menuShowTrajectory)
                    {
                        std::vector<pangolin::OpenGlMatrix> MsTrue;
                        CallTrajectoryTrue(MsTrue, Twcs_true_);
                        DrawAllTrajectory(MsTrue);
                    }

                    // if (menuShowTrajectoryOpti)
                    // {
                    //     std::vector<pangolin::OpenGlMatrix> MsTrue;
                    //     CallTrajectoryTrue(MsTrue, Twcs_);
                    //     DrawAllOptiTrajectory(MsTrue);
                    // }

                    //        if(menuShowTrajectoryOpti)
                    //        {
                    //            std::vector<pangolin::OpenGlMatrix> Ms_point;
                    //            CallTrajectoryPoint(Ms_point);
                    //            DrawAllTrajectory(Ms_point);
                    //        }
                    //
                    if (menuShowLine)
                        DrawTrueLine();

                    if(menuShowRefinedCamera)
                    {
                        //RotationEstimationEGraph();    
                    }
                    //
                    //        if(menuShowOptiLines)
                    //            DrawOptiiLines();

                    pangolin::FinishFrame();
                }
                pangolin::DestroyWindow("Simulation");
                pangolin::QuitAll();
            }

           void SetParameter(std::vector<Eigen::Vector3d> &mappoints, std::vector<Eigen::Matrix4d> &Twcs_gt,
                               std::vector<Eigen::Matrix4d> &Twcs, std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> &point_obs,
                               std::vector<double> &tri_point_inverse_depth, std::vector<Eigen::Vector3d> &tri_point_xyz)
           {
               points_gt_ = mappoints; // ground truth point
               tri_point_xyz_ = tri_point_xyz; // optimized 3D point
               point_obs_ = point_obs;   //  normalized noisy measurement
               tri_point_inverse_depth_ = tri_point_inverse_depth; // inverse depth
 
               Twcs_true_ = Twcs_gt; // ground truth pose
               Twcs_ = Twcs;         // optimized Twcs
           }

           

           void ShowReconTrajecEnv(std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> &point_obs, std::vector<double> &tri_point_inverse_depth, std::vector<Eigen::Vector3d> &tri_point_xyz)
           {
             point_obs_ = point_obs;   //  normalized noisy measurement
             tri_point_xyz_ = tri_point_xyz; // optimized 3D point
             tri_point_inverse_depth_ = tri_point_inverse_depth; // inverse depth

            //  for(int i =0; i<tri_point_xyz_.size();i++)
            //  {
            //     std::cout<<"1: "<<tri_point_xyz_[i]<<std::endl;
            //     std::cout<<"2: "<<tri_point_inverse_depth_[i]<<std::endl;
            //  }
           }
 
           
 
           void DrawReconstructedMapPoint()
           {
               for(int i=0;i<tri_point_xyz_.size();++i)
               {
                   //    const auto& obs = point_obs_[i];
                   //    const auto& ob = obs.begin();
                   //    const int cam_id = ob->first;
                   //   Eigen::Vector3d point_cam = ob->second;
                   //    if(point_cam(2,0)!=1)
                   //     point_cam /=point_cam(2,0);
                   //   point_cam /= tri_point_inverse_depth_[i];
                   //    Eigen::Matrix3d Rwc = Twcs_true_[cam_id].block(0,0,3,3);
                   //    Eigen::Vector3d twc = Twcs_true_[cam_id].block(0,3,3,1);
                   //    Eigen::Vector3d point_w = Rwc*point_cam+twc;
                   if(tri_point_xyz_[i].isZero())
                    continue;
                   glPointSize(5);
                   glBegin(GL_POINTS);
                   glColor3f(1.0, 1.0, 0.0);
                   // glVertex3d( point_cam[0], point_cam[1], point_cam[2]);
                   glVertex3d(tri_point_xyz_[i][0], tri_point_xyz_[i][1], tri_point_xyz_[i][2]);
                   glEnd();
               }
           }

           /**
            * @brief detect and show venom anchors 
            * 
            * @param Ms 
            */
           void ShowVenomAnchors(std::vector<pangolin::OpenGlMatrix>& Ms)
           {
               std::vector<std::pair<int, Eigen::Matrix3d>> vec_venom_anchors;
               std::vector<int> vec_anchor_id;
               std::set<int> set_venom_type;
               // anchors
               for (int frame_id = 0; frame_id < ptr_robot_trajectory_->obs_mw_.size(); frame_id++)
               {
                   // the i th frame detects a venom
                   if (ptr_robot_trajectory_->obs_mw_[frame_id].size() > 0)
                   {
                       // we have foud venoms in this environment
                       int venom_id = ptr_robot_trajectory_->obs_mw_[frame_id][0].first;
                       Eigen::Matrix3d rot_cam_venom = ptr_robot_trajectory_->obs_mw_[frame_id][0].second;

                       if (!set_venom_type.count(venom_id))
                       {
                           vec_venom_anchors.push_back(std::make_pair(venom_id, rot_cam_venom)); // [vd_type] = vd;
                           // std::cout << "rot_cam_venom: " << rot_cam_venom << std::endl;
                           // vec_vd.push_back(vd);
                           set_venom_type.insert(venom_id);
                           vec_anchor_id.push_back(frame_id);
                       }
                   }
               }

               std::vector< std::vector<std::pair<int/*frame_id*/, Eigen::Matrix3d /*Rcm*/ >>> venom_association;
               venom_association = std::vector< std::vector<std::pair<int/*frame_id*/, Eigen::Matrix3d /*Rcm*/ >>>(vec_anchor_id.size(), std::vector<std::pair<int/*frame_id*/, Eigen::Matrix3d /*Rcm*/ >>());
               

               for (int i = 0; i < vec_anchor_id.size(); i++) //visit all anchors
               {
                   // target
                   int anchor_frame_id = vec_anchor_id[i]; // which frame 
                   int anchor_venom_type = vec_venom_anchors[i].first; // which venom
                   Eigen::Matrix3d anchor_rot_cam_venom = vec_venom_anchors[i].second; // rotation Rcm 
                   // save anchor 
                   venom_association[i].push_back(std::make_pair(anchor_frame_id, anchor_rot_cam_venom));
                   
                   for (int j = 0; j < ptr_robot_trajectory_->obs_mw_.size(); j++) // visit all frames
                   {
                       int venom_type = ptr_robot_trajectory_->obs_mw_[j][0].first;
                       Eigen::Matrix3d rot_cam_venom = ptr_robot_trajectory_->obs_mw_[j][0].second;
                       // save other frames 
                       if(venom_type==anchor_venom_type)
                        venom_association[i].push_back(std::make_pair(j, rot_cam_venom) );
                   }
               }
               for (int i = 0; i < Ms.size(); i++)
               {
                   // show anchors 
                   for (int j = 0; j < vec_anchor_id.size(); j++)
                   {
                       if (i == vec_anchor_id[j])
                       {
                           GLfloat r = 0.0f;
                           GLfloat g = 0.0f;
                           GLfloat b = 1.0f;
                           DrawSigCam(Ms[i], r, g, b);
                       }
                   }

                   glLineWidth(2);
                   for(int j = 0; j<venom_association.size(); j++)
                   {

                    // std::cout<<"venom_association[i]:"<<venom_association[j].size()<<std::endl;
                       
                       glBegin(GL_LINES);
                       glColor3f(0.5f*j, 1.f*j, 0.1f*j);
                       int anchor_index = venom_association[j][0].first;
                       //std::cout<<"j:"<<j<<". Anchor_index: "<<anchor_index<<std::endl;
                       for (int k = 1; k < venom_association[j].size() - 1; k++)
                       {
                           int index =  venom_association[j][k].first;
                           //std::cout<<"index:"<<index<<";" ;
                           glVertex3f(Twcs_true_[index](0, 3), Twcs_true_[index](1, 3), Twcs_true_[index](2, 3));
                           glVertex3f(Twcs_true_[anchor_index](0, 3), Twcs_true_[anchor_index](1, 3), Twcs_true_[anchor_index](2, 3));
                       }
                       
                   }
                   glEnd();

               }
           }


           void DrawAllTrajectory(std::vector<pangolin::OpenGlMatrix>& Ms)
           {
               for(auto &M:Ms)
               {
                   GLfloat r = 0.0f; GLfloat g = 1.0f; GLfloat b = 0.0f;
                   DrawSigCam(M, r, g, b);
               }
           }

           void DrawAllOptiTrajectory(std::vector<pangolin::OpenGlMatrix>& Ms)
           {
 
               for(auto &M:Ms)
               {
                   GLfloat r = 0.0f; GLfloat g = 0.0f; GLfloat b = 1.0f;
                   DrawSigCam(M, r, g, b);
               }
 
           }
 
           void DrawSigCam( pangolin::OpenGlMatrix& M , GLfloat& r , GLfloat& g, GLfloat& b)
           {
               //camera size
               const float &w = 0.3;
               const float h = w*0.638;
               const float z = w*0.6;
 
               glPushMatrix();
 
           #ifdef HAVE_GLES
               glMultMatrixf(M.m);
           #else
               glMultMatrixd(M.m);
           #endif
 
               glLineWidth(1.0);
               glColor3f(r, g, b);
               glBegin(GL_LINES);
               glVertex3f(0, 0, 0);
               glVertex3f(w, h, z);
               glVertex3f(0, 0, 0);
               glVertex3f(w, -h, z);
               glVertex3f(0, 0, 0);
               glVertex3f(-w, -h, z);
               glVertex3f(0, 0, 0);
               glVertex3f(-w, h, z);
 
               glVertex3f(w, h, z);
               glVertex3f(w, -h, z);
 
               glVertex3f(-w, h, z);
               glVertex3f(-w, -h, z);
 
               glVertex3f(-w, h, z);
               glVertex3f(w, h, z);
 
               glVertex3f(-w, -h, z);
               glVertex3f(w, -h, z);
               glEnd();
 
               glPopMatrix();
           }
 
           void CallTrajectoryOpti(std::vector<pangolin::OpenGlMatrix>& Ms)
           {}
 
           void CallTrajectoryTrue(std::vector<pangolin::OpenGlMatrix>& Ms, std::vector<Eigen::Matrix4d> Twcs_true_)
           {
               {
                   Ms.clear();
                   for(auto & Twc : Twcs_true_)
                   {
                       pangolin::OpenGlMatrix M;
 
                       Eigen::Matrix3d Rwc = Twc.block(0,0,3,3);
                       Eigen::Vector3d twc = Twc.block(0,3,3,1);
 
                       M.m[0] = Rwc(0,0);
                       M.m[1] = Rwc(1,0);
                       M.m[2] = Rwc(2,0);
                       M.m[3]  = 0.0;
 
                       M.m[4] = Rwc(0,1);
                       M.m[5] = Rwc(1,1);
                       M.m[6] = Rwc(2,1);
                       M.m[7]  = 0.0;
 
                       M.m[8] = Rwc(0,2);
                       M.m[9] = Rwc(1,2);
                       M.m[10] = Rwc(2,2);
                       M.m[11]  = 0.0;
 
                       M.m[12] = twc(0);
                       M.m[13] = twc(1);
                       M.m[14] = twc(2);
                       M.m[15]  = 1.0;
 
                       Ms.push_back(M);
                   }
               }

               glLineWidth(2);
               glBegin(GL_LINES);
               glColor3f(0.0f, 1.f, 0.f);
               for (int i = 0; i < Twcs_true_.size() - 1; i++)
               {
                   glVertex3f(Twcs_true_[i](0,3), Twcs_true_[i](1,3),Twcs_true_[i](2,3));
                   glVertex3f(Twcs_true_[i+1](0,3), Twcs_true_[i+1](1,3),Twcs_true_[i+1](2,3));
               }
               glEnd();
           }
 
           void DrawTrueLine()
           {
               glLineWidth(2.0);
               glBegin(GL_LINES);
               glColor3f(1.0f,0.0f,0.0f);
               for(const auto & Line:lines_gt_)
               {
                   Eigen::Vector3d line0 = Line.block(0,0,3,1);
                   Eigen::Vector3d line1 = Line.block(0,1,3,1);
                   glVertex3f(line0(0),line0(1),line0(2));
                   glVertex3f(line1(0),line1(1),line1(2));
               }
               glEnd();
           }
 
           
           void SetSystemParameters()
           {
               // trajectory
               BuildTrajectory();
               // set mappoins
               BuildEnvMapPoints();
               // set maplines
               BuildEnvMapLines();
               // TODO: set MapPlanes
           }

           void BuildEnvMapLines()
           {
               for (int id = 0; id < vert_lines_+horiz_lines_; id++)
               {
                   simulator::MapLine *ptr_ml = new simulator::MapLine(id, ptr_robot_trajectory_);
                   if (id < 0.25*vert_lines_)
                       ptr_ml->GenerateMapLine(distance, "vertical-left");
                   else if (id < 0.5*vert_lines_)
                       ptr_ml->GenerateMapLine(-distance, "vertical-left");
                   else if (id < 0.75*vert_lines_)
                       ptr_ml->GenerateMapLine(distance, "vertical-right");
                   else if (id < vert_lines_)
                       ptr_ml->GenerateMapLine(-distance, "vertical-right");
                   else if (id < vert_lines_+0.25*horiz_lines_)
                       ptr_ml->GenerateMapLine(distance, "horizontal-left");
                   else if (id < vert_lines_+0.5*horiz_lines_)
                       ptr_ml->GenerateMapLine(-distance, "horizontal-left");
                   else if (id < vert_lines_+0.75*horiz_lines_)
                       ptr_ml->GenerateMapLine(distance, "horizontal-right");
                   else if (id < vert_lines_+horiz_lines_)
                       ptr_ml->GenerateMapLine(-distance, "horizontal-right");

                   ptr_ml->AddObservation(ptr_robot_trajectory_->vec_traject_gt_Twc_, add_noise_to_meas);
                   lines_gt.push_back(ptr_ml->pos_world_);
                   vec_meas_keyframe_ml.push_back(ptr_ml->vec_obs_);
                   vec_gt_keyframe_ml.push_back(ptr_ml->vec_obs_gt_);
                   vec_ptr_maplines.push_back(ptr_ml);
               }

#ifdef __VERBOSE__//OFF
               for (int j = 0, jend = ptr_robot_trajectory_->vec_traject_gt_Twc_.size(); j < jend; j++)
               {
                   std::cout << "the " << j << " th camera detects " << ptr_robot_trajectory_->contain_ml_cams_[j]
                             << " maplines" << std::endl;
               }
#endif
           }

           void BuildEnvMapPoints()
           {
               for (int id = 0; id < vert_points_; id++)
               {
                   simulator::MapPoint *ptr_mp = new simulator::MapPoint(id, ptr_robot_trajectory_);
                   if (id < 0.25*vert_points_)                                             // vertical-left
                       ptr_mp->GenerateMapPoint(distance, "vertical-left"); // left side of the wall
                   else if (id < 0.5*vert_points_)
                       ptr_mp->GenerateMapPoint(-distance, "vertical-left"); // right side
                   else if (id < 0.75*vert_points_)
                       ptr_mp->GenerateMapPoint(distance, "vertical-right"); // front side
                   else if (id < vert_points_)
                       ptr_mp->GenerateMapPoint(-distance, "vertical-right"); // back side

                   ptr_mp->AddObservation(ptr_robot_trajectory_->vec_traject_gt_Twc_, add_noise_to_meas);
                   // ptr_mp->print();
                   points_gt.push_back(ptr_mp->pos_world_);
                   // vec_meas_keyframe_mp: mappoint_id<camera_id, mappoint_value>
                   vec_meas_keyframe_mp.push_back(ptr_mp->obs);
                   vec_gt_keyframe_mp.push_back(ptr_mp->obs_gt);
               }
#ifdef __VERBOSE__OFF
               for (int j = 0, jend = ptr_robot_trajectory_->vec_traject_gt_Twc_.size(); j < jend; j++)
               {
                   std::cout << "the " << j << " th camera detects " << ptr_robot_trajectory_->contain_mp_cams_[j]
                             << " mappoints" << std::endl;
               }
#endif
           }


        private:
            /**
             * @brief Set the System Parameters object
             *
             * @param traject_num
             * @param traject_type 0: cycle trajectory;  1: sphere trajectory;
             * @param vert_lines
             * @param horiz_lines
             * @param vert_points
             * @param horiz_points
             */
            void InputSystemParameters(const int &traject_num, const int &traject_type, const int &vert_lines,
                                       const int &horiz_lines, const int &vert_points, const int &horiz_points)
            {
                //
                frame_num_ = traject_num;
                traject_type_ = traject_type;
                vert_lines_ = vert_lines;
                horiz_lines_ = horiz_lines;
                vert_points_ = vert_points;
                horiz_points_ = horiz_points;
            }

            // TODO:
            void SaveSystemConfig(const std::string &file_name)
            {
                // record those parameters
            }

            void BuildTrajectory()
            {
                // set cameras
                ptr_robot_trajectory_ = new simulator::Trajectory(traject_type_, frame_num_);

                if (traject_type_ == 0)
                    ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::CYCLE, frame_num_);
                else if (traject_type_ == 1)
                    ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::SPHERE, frame_num_);
            }

            void StartTracking()
            {
                // after preparing line landmarks, we then detect venom relationships
                ptr_tracker_ = new simulator::Track(ptr_robot_trajectory_, vec_ptr_maplines);
            }

            /**
             * @brief give the Trajectory and Env Parameter
             *
             * @param mappoints
             * @param maplines
             * @param Twcs_gt
             */
            void ShowFixedTrajecEnv(std::vector<Eigen::Vector3d> &mappoints, std::vector<Eigen::Matrix<double, 3, 2>> &maplines, std::vector<Eigen::Matrix4d> &Twcs_gt)
            {
                Twcs_true_ = Twcs_gt;   // ground truth pose
                points_gt_ = mappoints; // ground truth point
                lines_gt_ = maplines;
            }
   };
 
}

#endif  //__VENOM_SRC_VISUALIZER_INTERFACE_HPP__