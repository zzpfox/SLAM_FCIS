# ORB_SLAM2 with FCIS Segmentation

* Note: This repository is mainly built upon [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [FCIS](https://github.com/msracver/FCIS). Many thanks for their great work.

## Installation
Refer to the corresponding original repositories ([ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [FCIS](https://github.com/msracver/FCIS) for installation tutorial).

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

## Note
Please refer to the [smartpointer](https://github.com/CTTC/SLAM_FCIS/tree/smartpointer) branch for ORB-SLAM2 with smart pointers used instead. And this branch also contains the code on how to save and reload map using boost serialization.
