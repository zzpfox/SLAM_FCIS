# ORB_SLAM2 with FCIS Segmentation 

* Note: This repository is mainly built upon [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [FCIS](https://github.com/msracver/FCIS). Many thanks for their great work.

## Installation
Refer to the corresponding original repositories ([ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [FCIS](https://github.com/msracver/FCIS) for installation tutorial.

## Run

* **First**, run 
```bash
python FCIS/experiments/slxrobot/segmentation_for_slam.py
```

* **Then**, run
```bash
./ORB_SLAM2/Examples/ROS/ORB_SLAM2/RGBD_Kinect ORB_SLAM2/Vocabulary/ORBvoc.bin ORB_SLAM2/Examples/RGB-D/kinect2_qhd.yaml
```

The current repository only supports **RGBD sensor** input.

After running, you can toggle `Show DenseMap` button to see the point cloud generated at the moment you pressed this button.
And `Show SegObjects` will show the objects' type and their corresponding positions in the keyframes in which they exist.


### Major Modifications compared to the master branch
* Python Socket server in FCIS will reset to accept new connections automatically when the client is shutdown.
* ORB-SLAM2 now uses `shared_ptr`, 'weak_ptr', and `unique_ptr` instead of the usual pointers. Note that the original code in ORB-SLAM2 uses the pointers generted by `new`, and there is serious memory leak problem as there is hardly any corresponding `delete`. 
* ORB-SLAM2 now support boost serialization over MapPoints, KeyFrames, Map. So the whole program can be saved and loaded again.
If we reuse the map previously stored, the program will load in the map and enter the `Localization Mode` in the first place. After the camera has been successfully relocalized, you can manually turn off the `Localization Mode`.
* Add naive version of RRT motion planning algorithm
* Add OMPL motion planning library (**Please install ompl from source for the latest version**)
* Support automatically exploring unkonwn area and building map
* Add SBPL motion planning library ([SBPL](https://github.com/sbpl/sbpl))
* Support planning in (x, y) and (x, y , theta)
* Show Dense Map with color rendered

### Report
Please refer to [slamreport.pdf](slamreport.pdf) for report reference

### Copyright
For this specific version, copyright belongs to Shanghai LingXian Robotics. For commercial usage, please contact: chentao904@163.com