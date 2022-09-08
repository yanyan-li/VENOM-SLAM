/*
* @Author: yanyan.li yanyan.li.camp@gmail.com
* @Date: 2022-08-10 03:01:19
* @LastEditors: yanyan.li yanyan.li.camp@gmail.com
* @LastEditTime: 2022-09-07 12:14:11
* @FilePath: 
* @Description:  This file provides the architecture of the simulation environment that contains
*                designed trajectory (keyframes), 3D landmarks, normalized measurements and
*                associations (observations between KF and mappoints)
*/
#ifndef __SIMULATION_INCLUDE_BASICS_HPP__
#define __SIMULATION_INCLUDE_BASICS_HPP__
#include <map>
#include <vector>
#include <iostream>
#include <random>
 
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
 
 
namespace ParaLi
{
   class Trajectory{
       public:
           enum TrajectoryType
           {
               CYCLE = 0,
               SPHERE =1,
               //TRIANGLE = 1,
               RECTANGLE = 2,
               HEXAGON =3,
               LINE = 4
           };
          
           Trajectory():traject_type_(CYCLE){};
          
           void PoseGeneration(double theta, double wave_theta, double step, int radius, double wave_high)
           {
               Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
               double theta_ = 0;
               if(theta < M_PI / 2.) theta_ = theta + M_PI/2.;
               else if(theta >= M_PI /2. && theta < M_PI ) theta_ = theta - M_PI*3./2.;
               else if(theta >= M_PI  && theta < M_PI * 3. / 2. ) theta_ = theta+M_PI/2.;
               else if(theta >= M_PI * 3./2. && theta < M_PI * 2 ) theta_ = theta-3.*M_PI/2.;
               Eigen::Matrix3d Rwb;
               Rwb = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
               Eigen::Vector3d twb = Eigen::Vector3d( radius * cos(theta) , radius * sin(theta), wave_high * std::cos(wave_theta));
 
               Twc.block(0, 0, 3, 3) = Rwb;
               Twc.block(0, 3, 3, 1) = twb;
 
               traject_gt_Twc_.push_back(Twc);
           }
           void SpherePoseGeneration(double radius, int nums_points, int index)
           {
               Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
               double phi = acos(-1.0 + (2.0 * (index+1) - 1.0) / nums_points);
               double theta = sqrt(nums_points * M_PI) * phi;
               double t_x = radius * cos(theta) * sin(phi);
               double t_y = radius * sin(theta) * sin(phi);
               double t_z = radius * cos(phi);
               Eigen::Vector3d twb;
               twb<< t_x,t_y,t_z;
              
               Eigen::Matrix3d Rwb;
               Rwb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
              
 
               Twc.block(0, 0, 3, 3) = Rwb;
               Twc.block(0, 3, 3, 1) = twb;
 
               traject_gt_Twc_.push_back(Twc);
 
           }
 
           bool GenerateCycleKeyFrames(const int frame_num, int wave_num, double wave_high,  double radius_cycle=0)
           {
               if(frame_num<2)
                   return false;
 
               for(int n=0; n<frame_num; n++) // camera id from n=0 to n=frameNum-1
               {
                   int radius = 1;
                   double wave_high = 0.75;
                   double theta = n * 2. * M_PI / frame_num;
                   double step = n * 1.  / frame_num;
                   double wave_theta = n * 2. * M_PI/(frame_num/wave_num);
 
                   PoseGeneration(theta, wave_theta,step, radius,wave_high);
               }
 
           }
           bool GenerateSphereKeyFrames(const int frame_num, double radius )
           {
               if(frame_num<2)
                   return false;
               for(int n=0;n<frame_num;n++)
               {
                   double radius = 2.0;
                   double theta = n*2.*M_PI/frame_num;
                   SpherePoseGeneration(radius, frame_num,n);
 
               }   
 
 
           }
 
           bool GenerateTrajectory(const TrajectoryType traject_type, const int num_keyframe)
           {
               if(num_keyframe<2)
                   return false;
               if(traject_type == CYCLE)
               {
                   int wave_num = 10;
                   double wave_high = 0.75;
                   GenerateCycleKeyFrames(num_keyframe, wave_num, wave_high);
               }
               else if(traject_type==SPHERE)
               {
                   double r = 2.0;
                  
                   GenerateSphereKeyFrames(num_keyframe, r);
               }
               else
               {
                   std::cout<<"We only have the CYCLE type trajectory currently."<<std::endl;
                   return false;
               }
 
               return true;
           }
 
           void PrintTrajectory()
           {
               for(size_t i = 0; i< traject_gt_Twc_.size(); i++)
               {
                   std::cout<<"\033[0;35m This is the\033[0m"<<i<<" th camera: "<<std::endl
                            <<traject_gt_Twc_[i]<<std::endl;
               }
           }
 
           void setKeyFrameDetects(int id)
           {
               contain_feature_cams_[id]++;
           }
 
       public:
           int num_id_; // Trajectory id
           int n_; // landmarks detected by the Trajectory
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
           std::map<int /*pos_world_id*/, int /*measurements_id*/> associations;
 
   };
  
 
   // Scalar -> Eigen::Vector3d  // Scalar -> cv::Matx31d
   static Trajectory trajec_;
 
   class MapPoint{
       public:
           // id of the mappoint
           int num_id_;
           // how many Trajectorys detect this point
           int observed;
           // position in the world coordinate
           Eigen::Vector3d pos_world_;
           Eigen::Vector3d pos_world_noise_;
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
           MapPoint(const int id):num_id_(id),observed(0)
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
                   trajec_.setKeyFrameDetects(i);
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
               std::cout<<"\033[0;31m The global position of mappoint is  \033[0m"<<std::endl
                        <<pos_world_<<std::endl<< "which is detected by "<<observed<<"cameras"<<std::endl;
 
           }
   };
 
  
   class Visualizer
   {
 
       public:
           // all mappoints and camera pose
           std::vector<Eigen::Vector3d> points_true_;
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
               // pangolin::Var<bool> menuShowLines("menu.GT Lines",true,true);
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
                   // if(menuShowLines)
                   //     DrawTrueLine();
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
               // glLineWidth(2.0);
               // glBegin(GL_LINES);
               // glColor3f(1.0f,0.0f,0.0f);
               // for(const auto & Line:lines_true_)
               // {
               //     Eigen::Vector3d line0 = Line.block(0,0,3,1);
               //     Eigen::Vector3d line1 = Line.block(0,1,3,1);
               //     glVertex3f(line0(0),line0(1),line0(2));
               //     glVertex3f(line1(0),line1(1),line1(2));
               // }
               // glEnd();
           }
 
   };
 
   class Reconstruct
   {
       public:
           std::vector<double> tri_point_inverse_depth_;
           std::vector<Eigen::Vector3d> tri_point_xyz_;
          
           // Reconstruct(std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> point_obs_meas, ):
           // {};
           void Triangulation(std::vector<std::vector<std::pair< int,Eigen::Vector3d>>> point_obs,  std::vector<Eigen::Matrix4d> Twcs)
           {
               for(auto &ob:point_obs)
               {
                   Eigen::Vector3d point_camera;
                   Eigen::MatrixXd A(ob.size()*2, 4);
                   int index = 0;
                   Eigen::Matrix4d Twc0 = Twcs[ob[0].first];
                   for(int i=1; i<ob.size(); ++i)
                   {
                       Eigen::Vector3d ob0 = ob[i].second.col(0);
                       // P = T_cs_c0 : from c0 to cs
                       Eigen::Matrix4d P = Twcs[ob[i].first].inverse()*Twc0;
                       Eigen::Vector3d f = ob0; //.normalized();
                       std::cout<<" f "<< f<< ", "<< ob0<<std::endl;
                       A.row(index++) = f[0] * P.row(2) - f[2] * P.row(0);
                       A.row(index++) = f[1] * P.row(2) - f[2] * P.row(1);
 
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
 
#endif //__SIMULATION_INCLUDE_BASICS_HPP__
 
 
 

