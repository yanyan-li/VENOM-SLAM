<!--
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-14 16:37:04
 * @LastEditTime: 2022-09-20 17:07:47
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/readme.md
-->
## Venom-Simulator  
A simulator is a super important tool for testing SLAM modules, especially for newly proposed ideas, as it is easy to check the validity of your ideas in a controlled environment.

```
Architecture:
    src                 ## source files
        |-->estimator   ## responsible for ground-truth/noisy camera poses
            |-->Trajectory.hpp
        |-->feature     ## responsible for 3D reconstruction
            |-->Reconstruct.hpp 
        |-->landmark    ## responsible for ground-truth/noisy landmarks
            |-->MapPoint.hpp
            |-->MapLine.hpp    
        |-->optimizer
            |-->factor  ## responsible for building different factors 
            |-->GlobalBundleAdjustment.hpp
        |-->visulizer   ## responsible for visualozation based on Pangolin
    test                ## entrance for testing
        |-->test_pointBA.cc
        |-->test_
    thirdparty          ## optimization libraries
        |-->gtsam
        |-->ceres
        |-->g2o
    Docker              ## docker environments
```

#### 1. Prerequisites based on Docker 

Clone the repo, and it is easy to build your own image and container based on the Dockerfile proposed in the **Docker** folder.
The proposed [docker environment](Docker/readme.md) contains the following  libs.
#### Installed environment
Pangolin, OpenCV, Eigen and minor libs have been installed in already.  

#### Choices for optimization.
*Popular optimization libraries including ceres, gtsam and g2o can be selected here based on your particular preferences.* 
Optimization libraries are not installed in advance, but suggestions for installing can be found  [here](thirdparty/readme.md).

#### 2. Simulation introduction 

Red point are ground truth **landmarks**, while reconstructed **mappoints** generated from noisy measurements are yellow. 

Trajectory: Cycle, Sphere and so on.

##### 2.1 Show Env

```
cd venom
mkdir build
cd build 
cmake ..
make
../bin/show_estimator_env 
```



![env](images/env.png)

​																													Figure 1. Environment 1.

![env2](images/env2.png)

​																													Figure 2. Environment 2.

##### 2.2 Optimization map points 

Commands for testing the simulator

```
cd venom
mkdir build
cd build
cmake ..
make
cd ../bin
./test_pointBA
```

![environment](images/environment.png)



### Acknowledgement

