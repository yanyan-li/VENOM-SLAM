/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-18 02:53:44
 * @LastEditTime: 2022-09-22 18:26:28
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/src/visulizer/Visualizer.hpp
 */

#ifndef __VENOM_SRC_VISUALIZER_VISUALIZER_HPP__
#define __VENOM_SRC_VISUALIZER_VISUALIZER_HPP__ 

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>

namespace simulator
{
    class Visualizer
   {
 
       public:
           // all mappoints and camera pose
           std::vector<Eigen::Vector3d> points_true_;
           std::vector<Eigen::Matrix<double,3,2>> lines_true_;

           std::vector<Eigen::Matrix4d> Twcs_true_; 
           std::vector<Eigen::Matrix4d> Twcs_;
           std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> point_obs_;
           std::vector<double> tri_point_inverse_depth_;
           std::vector<Eigen::Vector3d> tri_point_xyz_;
 
           void SetParameter(std::vector<Eigen::Vector3d> &mappoints, std::vector<Eigen::Matrix4d> &Twcs_gt,
                               std::vector<Eigen::Matrix4d> &Twcs, std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> &point_obs,
                               std::vector<double> &tri_point_inverse_depth, std::vector<Eigen::Vector3d> &tri_point_xyz)
           {
               points_true_ = mappoints; // ground truth point
               tri_point_xyz_ = tri_point_xyz; // optimized 3D point
               point_obs_ = point_obs;   //  normalized noisy measurement
               tri_point_inverse_depth_ = tri_point_inverse_depth; // inverse depth
 
               Twcs_true_ = Twcs_gt; // ground truth pose
               Twcs_ = Twcs;         // optimized Twcs
           }

           void SetEnvParameter(std::vector<Eigen::Vector3d> &mappoints, std::vector<Eigen::Matrix<double,3,2>> &maplines, std::vector<Eigen::Matrix4d> &Twcs_gt,
                               std::vector<Eigen::Matrix4d> &Twcs, std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> &point_obs,
                               std::vector<double> &tri_point_inverse_depth, std::vector<Eigen::Vector3d> &tri_point_xyz)
           {
               points_true_ = mappoints; // ground truth point
               lines_true_ = maplines;
               tri_point_xyz_ = tri_point_xyz; // optimized 3D point
               point_obs_ = point_obs;   //  normalized noisy measurement
               tri_point_inverse_depth_ = tri_point_inverse_depth; // inverse depth
 
               Twcs_true_ = Twcs_gt; // ground truth pose
               Twcs_ = Twcs;         // optimized Twcs
           }
 
           void show()
           {
               pangolin::CreateWindowAndBind("Meta-Bounds SLAM (backend) Simulator",1024,768);
               // 3D Mouse handler requires depth testing to be enabled
               // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
               glEnable(GL_DEPTH_TEST);
               // Issue specific OpenGl we might need
               // 在OpenGL中使用颜色混合
               glEnable(GL_BLEND);
               // 选择混合选项
               glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
               // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
               pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(275));
               pangolin::Var<bool> menuWhite("menu.White Background",false,true);
               pangolin::Var<bool> menuShowTrajectory("menu.Show TrajectoryGT",true,true);
               pangolin::Var<bool> menuShowTrajectoryOpti("menu.Show TrajectoryOpti", false, true);
           //    pangolin::Var<bool> menuShowTrajectoryPoint("menu.Show TrajectoryPoint",true,true);
           //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
           //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
           //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
               pangolin::Var<bool> menuShowPoint("menu.Groudtruth Point",false,true);
               pangolin::Var<bool> menuShowPointRecon("menu.Reconstructed Point",false,true);
               pangolin::Var<bool> meanShowPointOpti("menu.Optimize Point",false,true);
               pangolin::Var<bool> menuShowLine("menu.Groudtruth Line",false,true);
               // pangolin::Var<bool> menuShowOptiLines("menu.Show Opti Lines",true,true);
 
               // Define Trajectory Render Object (for view / scene browsing)
               // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
               // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
               //                观测目标位置：(0, 0, 0)
               //                观测的方位向量：(0.0,-1.0, 0.0)
               pangolin::OpenGlRenderState s_cam(
                       pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                       pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
               );
 
               // Add named OpenGL viewport to window and provide 3D Handler
               // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
               // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
               // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
               // 最后一个参数（-1024.0f/768.0f）为显示长宽比
               pangolin::View& d_cam = pangolin::CreateDisplay()
                       .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                       .SetHandler(new pangolin::Handler3D(s_cam));
 
               pangolin::OpenGlMatrix Twc;
               Twc.SetIdentity();
 
               std::vector<pangolin::OpenGlMatrix> Twcs;
               Twcs.push_back(Twc);
 
           //    cv::namedWindow("Current Frame");
 
               bool bFollow = true;
               bool bLocalizationMode = false;
 
               int Single_rpr_id = 0;
               int Single_line_id = 0;
               bool line_id_change = false;
 
               int cut_i = 0;
 
               while(!pangolin::ShouldQuit() )
               {
                   // while(!EstOK)
                   // cv::waitKey(100);
                   // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
                   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
 
                   d_cam.Activate(s_cam);
                   // 步骤3：绘制地图和图像
                   // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
                   if(menuWhite)
                       glClearColor(1.0f,1.0f,1.0f,1.0f);
                   else
                       glClearColor(0.0f,0.0f,0.0f,0.0f);
 
 
                   const double lens = 0.5;
                   glLineWidth(2.0);
                   glBegin(GL_LINES);
                   glColor3f(1.0f,0.0f,0.0f);
                   glVertex3f(0,0,0);
                   glVertex3f(lens,0,0);
 
 
                   glColor3f(0.0f,1.0f,0.0f);
                   glVertex3f(0,0,0);
                   glVertex3f(0,lens,0);
 
                   glColor3f(0.0f,0.0f,1.0f);
                   glVertex3f(0,0,0);
                   glVertex3f(0,0,lens);
 
                   glEnd();
 
 
                   if(menuShowPoint)
                   {
                       glPointSize(5);
                       glBegin(GL_POINTS);
                       glColor3f(1.0,0.0,0.0);
                       for(auto p : points_true_)
                       {
                           glVertex3d( p[0], p[1], p[2]);
                       }
                       glEnd();
                   }
 
                   if(menuShowPointRecon)
                       DrawTriPoint();
 
                   if(meanShowPointOpti)
                   {
                       // call bundleadjustment functions
                      
 
                       for(int i=0;i<point_obs_.size();++i)
                       {
                           const auto& obs = point_obs_[i];
                           const auto& ob = obs.begin();
                           const int cam_id = ob->first;
 
                           glPointSize(5);
                           glBegin(GL_POINTS);
                           glColor3f(1.0,1.0,1.0);
                           glVertex3d( tri_point_xyz_[i](0,0), tri_point_xyz_[i](1,0), tri_point_xyz_[i](2,0));
                           glEnd();
                       }
                   }   
  
                   if(menuShowTrajectory)
                   {
                       std::vector<pangolin::OpenGlMatrix> MsTrue;
                       CallTrajectoryTrue(MsTrue, Twcs_true_);
                       DrawAllTrajectory(MsTrue);
                   }
 
                   if(menuShowTrajectoryOpti)
                   {
                       std::vector<pangolin::OpenGlMatrix>  MsTrue;
                       CallTrajectoryTrue(MsTrue, Twcs_);
                       DrawAllOptiTrajectory(MsTrue);
                   }
                  
           //        if(menuShowTrajectoryOpti)
           //        {
           //            std::vector<pangolin::OpenGlMatrix> Ms_point;
           //            CallTrajectoryPoint(Ms_point);
           //            DrawAllTrajectory(Ms_point);
           //        }
           //
                   if(menuShowLine)
                       DrawTrueLine();
           //
           //        if(menuShowOptiLines)
           //            DrawOptiiLines();
 
 
 
                   pangolin::FinishFrame();
               }
           //    std::cerr << "MFK" << std::endl;
               pangolin::DestroyWindow("Simulation");
 
               pangolin::QuitAll();
           }
 
 
           void DrawTriPoint()
           {
               for(int i=0;i<point_obs_.size();++i)
               {
                   const auto& obs = point_obs_[i];
                   const auto& ob = obs.begin();
                   const int cam_id = ob->first;
                   Eigen::Vector3d point_cam = ob->second;
                   point_cam /= tri_point_inverse_depth_[i];
 
                   Eigen::Matrix3d Rwc = Twcs_[cam_id].block(0,0,3,3);
                   Eigen::Vector3d twc = Twcs_[cam_id].block(0,3,3,1);
 
                   Eigen::Vector3d point_w = Rwc*point_cam+twc;
 
 
                   glPointSize(5);
                   glBegin(GL_POINTS);
                   glColor3f(1.0,1.0,0.0);
                   glVertex3d( point_w[0], point_w[1], point_w[2]);
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
               //相机模型大小：宽度占总宽度比例为0.08
               const float &w = 0.3;
               const float h = w*0.638;
               const float z = w*0.6;
 
               //百度搜索：glPushMatrix 百度百科
               glPushMatrix();
 
               //将4*4的矩阵Twc.m右乘一个当前矩阵
               //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
               //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
           #ifdef HAVE_GLES
               glMultMatrixf(M.m);
           #else
               glMultMatrixd(M.m);
           #endif
 
           //    设置绘制图形时线的宽度
               glLineWidth(1.0);
               //设置当前颜色为绿色(相机图标显示为绿色)
               glColor3f(r, g, b);
               //用线将下面的顶点两两相连
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
 
 
               //双缓存交换缓存以显示图像
           //    glutSwapBuffers();
 
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
           }
 
           void DrawTrueLine()
           {
               glLineWidth(2.0);
               glBegin(GL_LINES);
               glColor3f(1.0f,0.0f,0.0f);
               for(const auto & Line:lines_true_)
               {
                   Eigen::Vector3d line0 = Line.block(0,0,3,1);
                   Eigen::Vector3d line1 = Line.block(0,1,3,1);
                   glVertex3f(line0(0),line0(1),line0(2));
                   glVertex3f(line1(0),line1(1),line1(2));
               }
               glEnd();
           }
 
   };
 
}

#endif  //__VENOM_SRC_VISUALIZER_VISUALIZER_HPP__