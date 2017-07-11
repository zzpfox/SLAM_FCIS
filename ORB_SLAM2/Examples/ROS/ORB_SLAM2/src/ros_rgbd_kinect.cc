/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "Converter.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM, ros::NodeHandle& n)
        : mpSLAM(pSLAM), mn(n)
    {
        mPosPub = mn.advertise<geometry_msgs::PoseStamped>("camera_pose", 5);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System *mpSLAM;
    ros::Publisher mPosPub;
    ros::NodeHandle mn;
};

int main(int argc, char **argv)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::cout << "=============================================" << std::endl;
    std::cout << "Program started at: " << std::ctime(&start_time) << std::endl;
    std::cout << "=============================================" << std::endl;
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3 && argc != 4) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl
            <<"Or: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings reusemap" << endl
            <<"Or: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings automap" << endl;
        ros::shutdown();
        return 1;
    }
    bool reuseMap = false;
    bool autoMap = false;
    std::string argvThree;
    if (argc == 4)
    {
        argvThree = argv[3];
        if (argvThree == "reusemap")
        {
            reuseMap = true;
        }
        else if (argvThree == "automap")
        {
            autoMap = true;
        }
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true, reuseMap, autoMap);
    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM, nh);


    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveMap();

    ros::shutdown();
    sleep(2);
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    PublishPose(Tcw);
}

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if (!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(1);
        poseMSG.pose.position.z = twc.at<float>(2);
        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "camera";
        poseMSG.header.stamp = ros::Time::now();
        mPosPub.publish(poseMSG);
    }
}


