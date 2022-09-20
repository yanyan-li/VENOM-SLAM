<!--
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-19 16:10:01
 * @LastEditTime: 2022-09-20 13:56:48
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/thirdparty/readme.md
-->
### Install Ceres

In the docker environment, basic libs (OpenCV, Pangolin) were installed already. Additional libs if you want to use, following commands can be q2222bned 

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
### Install g2o 

### Install gtsam


