ulimit -c unlimited
rm core
# ./RGBD_Kinect /home/chentao/software/ORB_SLAM2/Vocabulary/ORBvoc.bin /home/chentao/software/ORB_SLAM2/Examples/RGB-D/kinect2_qhd.yaml reusemap
./RGBD_Kinect /home/chentao/software/ORB_SLAM2/Vocabulary/ORBvoc.bin /home/chentao/software/ORB_SLAM2/Examples/RGB-D/kinect2_qhd.yaml automap