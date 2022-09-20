<!--
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-19 16:10:01
 * @LastEditTime: 2022-09-20 14:08:58
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/thirdparty/readme.md
-->
### Install Ceres

In the docker environment, basic libs (OpenCV, Pangolin) were installed already. Additional libs if you want to use, following commands can help you to install them.

```
cd thirdparty/ceres
mkdir build 
cd build 
sudo apt-get update
# install reqirements
apt-get install liblapack-dev libsuitesparse-dev  libgflags-dev libgoogle-glog-dev libgtest-dev
cmake ..
make 
sudo make install
```


### Install gtsam 
**gtsam** is a popular optimization library for SLAM/VO/VIO systems, which introduces factors into the pose-landmark graphs. We have to note that the Eigen used here comes from the system.  

```
cd thirdparty/gtsam
mkdir build
cd build 
cmake ..
make 
sudo make install 
```

### Install g2o 

**g2o** is also a friendly optimization library for pose estimation based on the edge-vertex ideas.  
