/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-28 14:43:35
 * @LastEditTime: 2022-09-28 16:05:45
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/test/test_interface.cc
 */


#include <iostream>

#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>

int main(/*int argc, char* argv[]*/)
{  
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind(" VENOM (0.0.1) SLAM (backend) Simulator",1024,768);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);
  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
  );

  // Choose a sensible left UI Panel width based on the width of 20
  // charectors from the default font.
  const int UI_WIDTH = 30* pangolin::default_font().MaxWidth();

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("menu")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  
  
  pangolin::Var<bool> menuWhite("menu.White Background",false,true);

  pangolin::Var<int> traject_num("menu.KeyFrame Number", 100, 50, 200);
  pangolin::Var<int> traject_type("menu.Trajectory Type", 0,0,4);
  pangolin::Var<int> vert_points("menu.Vertical Points",20,4,40);
  pangolin::Var<int> horiz_points("menu.Horizontal Points",10,0,20);
  pangolin::Var<int> vert_lines("menu.Vertical Lines",20,4,40);
  pangolin::Var<int> horiz_lines("menu.Horizontal Lines",10,0,20); 
  
  pangolin::Var<bool> set_env("menu.Build&Fix Environment",false,false);
  pangolin::Var<int> set_traject_num("menu.Num of KeyFrames:", 0);
  pangolin::Var<int> set_traject_type("menu.Selected Trajectory Type:", 0);
  pangolin::Var<int> set_vert_points("menu.Num of Vertical Points:",0);
  pangolin::Var<int> set_horiz_points("menu.Num of Horizontal Points:",0);
  pangolin::Var<int> set_vert_lines("menu.Num of Vertical Lines:",0);
  pangolin::Var<int> set_horiz_lines("menu.Num of Horizontal Lines:",0); 

  pangolin::Var<bool> menuShowTrajectory("menu.Show TrajectoryGT", true, true);
  pangolin::Var<bool> menuShowTrajectoryOpti("menu.Show TrajectoryOpti", false, true);
  //    pangolin::Var<bool> menuShowTrajectoryPoint("menu.Show TrajectoryPoint",true,true);
  //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
  //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
  //    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
  pangolin::Var<bool> menuShowPoint("menu.Groudtruth Point", true, true);
  pangolin::Var<bool> menuShowPointRecon("menu.Reconstructed Point", false, true);
  pangolin::Var<bool> meanShowPointOpti("menu.Optimize Point", false, true);
  pangolin::Var<bool> menuShowLine("menu.Groudtruth Line", true, true);

  // std::function objects can be used for Var's too. These work great with C++11 closures.
  pangolin::Var<std::function<void(void)>> save_window("menu.Save Window", [](){
      pangolin::SaveWindowOnRender("window");
  });

  pangolin::Var<std::function<void(void)>> save_cube_view("menu.Save 3D Window", [&d_cam](){
      pangolin::SaveWindowOnRender("cube", d_cam.v);
  });


  bool click_once = true;

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

    // Overloading of Var<T> operators allows us to treat them like
    // their wrapped types, eg:
    
    if(click_once)
    {
        if (set_env)
        {
            set_traject_num = traject_num;
            set_traject_type = traject_type;
            set_vert_points = vert_points;
            set_horiz_points = horiz_points;
            set_vert_lines = vert_lines;
            set_horiz_lines = horiz_lines;
            std::cout <<std::endl<< "\033[0;35m [Venom Similator Printer] The Environment was constructed. \033[0m" << std::endl;
            click_once = false;
        }
    }

    if(menuWhite)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
    else
        glClearColor(0.0f,0.0f,0.0f,0.0f);
 
    
    if(d_cam.IsShown()) {
        // Activate efficiently by object
        d_cam.Activate(s_cam);

        // Render some stuff
        glColor3f(1.0,1.0,1.0);
        pangolin::glDrawColouredCube();
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
