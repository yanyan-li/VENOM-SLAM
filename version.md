

#### Introduction of Venom-0.01

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
        |-->test_show_env.cc
    thirdparty          ## optimization libraries
        |-->gtsam
        |-->ceres
        |-->g2o
    Docker              ## docker environments
```
