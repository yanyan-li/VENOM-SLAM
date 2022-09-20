<!--
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-14 16:37:04
 * @LastEditTime: 2022-09-20 14:17:58
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/readme.md
-->
## Venom: Rigid Rotation Estimation 



#### 1. Prerequisites based on Docker 

Clone the repo, and it is easy to build your own image and container based on the Dockerfile proposed in the **Docker** folder.
The [docker environment](Docker/readme.md) contains the following  libs.
#### Pangolin 

#### Eigen 

#### OpenCV 

#### g2o and gtsam 

*Two popular optimization libraries could be installed here to take care of different preferences.* 
Suggetions for installing can be found [here](thirdparty/readme.md).

#### 2. Simulation introduction 

Red point are ground truth **landmarks**, while reconstructed **mappoints** generated from noisy measurements are yellow. 

Trajectory: Cycle, Sphere and so on.

![environment](images/environment.png)

