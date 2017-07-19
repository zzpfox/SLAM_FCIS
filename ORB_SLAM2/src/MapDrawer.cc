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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <limits>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <opencv2/core/eigen.hpp>

namespace ORB_SLAM2
{
std::string MapDrawer::msPointCloudPath = "./Data/PointCloud";
MapDrawer::MapDrawer(std::shared_ptr<Map> pMap, const string &strSettingPath)
    : mpMap(pMap),
      mCloud(new pcl::PointCloud<pcl::PointXYZ>()),
      mfHeightUpperBound(1.0),
      mfHeightLowerBound(-0.2)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        cerr << "failed to open " << strSettingPath << endl;
        exit(EXIT_FAILURE);
    }
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mbCalPointCloud = true;
    mbFindObjCalPoints = true;
    std::string planner = static_cast<std::string> (fSettings["PathPlanningPlanner"]);
    float obstacleWidth = fSettings["RobotRadius"];
    mPathPlanning = std::make_shared<PathPlanning2D> (planner,
                                                      mPointSize * 6.0,
                                                      mKeyFrameLineWidth * 4.0,
                                                      obstacleWidth,
                                                      0.025,
                                                      mpMap->msDataFolder,
                                                      mfHeightUpperBound,
                                                      mfHeightLowerBound);
    mpSolution = std::make_shared<std::vector<std::vector<float> > >();
}

void MapDrawer::DrawPointCloud()
{
    if (mbCalPointCloud)
    {
        CalPointCloud();
        mbCalPointCloud = false;
    }
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 1.0);
    for (auto p : mCloud->points) {
//        if (p.y >= -mfHeightUpperBound && p.y <= -mfHeightLowerBound)
        if (p.y >= -mfHeightUpperBound && p.y <= 2.0)
        {
            glVertex3f(p.x, p.y, p.z);
        }
    }

    glEnd();
}

void MapDrawer::CalPointCloud(bool saveOctoMap)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    start = std::chrono::system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();
    int sampleFrequency = 4;
    vector<std::shared_ptr<KeyFrame> > sampledVPKFs;
    int kFNum = vpKFs.size();
    for (int j = 0; j < kFNum; j += sampleFrequency) {
        sampledVPKFs.push_back(vpKFs[j]);
    }
    int numThreads = 3;
    std::thread threads[numThreads];
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    for (int i = 0; i < numThreads; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        clouds.push_back(tmp);
    }
    for (int i = 0; i < numThreads; i++) {
        threads[i] = std::thread(&MapDrawer::GeneratePointCloud, this, std::cref(sampledVPKFs), clouds[i], i,
                                 numThreads);
    }
    for (auto &th : threads) {
        th.join();
    }
    for (int i = 0; i < numThreads; i++)
    {
        (*cloud) += *(clouds[i]);
    }
    {
        unique_lock<mutex> lock(mMutexMCloud);
        if (cloud->points.size() > 0)
        {
            FilterPointCloud(cloud, mCloud);
        }

//        mCloud = cloud;
    }
    if (saveOctoMap)
    {
        if (mpThreadOctomap) {
            mpThreadOctomap->join();
        }
        mpThreadOctomap.reset(new thread(&MapDrawer::BuildOctomap, this, mCloud));
    }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Total Number of Points: " << mCloud->size() << std::endl;
    std::cout << "CalPointCloud Elapsed time: " << elapsed_seconds.count() << "s\n";
}

void MapDrawer::FindObjects()
{
    if (mbFindObjCalPoints)
    {
        mPathPlanning->reset();
        CalPointCloud();

        // get current camera center position
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        std::vector<float> start(2, 0);

        if (!mCameraPose.empty())
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ERwc;
            cv::cv2eigen(Rwc, ERwc);
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Etcw;
            cv::cv2eigen(mCameraPose.rowRange(0, 3).col(3), Etcw);
            Eigen::Vector3f Etwc = -ERwc * Etcw;
            cv::eigen2cv(Etwc, twc);
            float *it = twc.ptr<float>(0);
            start[0] = it[0];
            start[1] = it[2];
        }
        else
        {
            std::cout << "Cannot locate the camera, assuming it's at the origin ..." << std::endl;
        }


        auto objectMap = mpMap->mObjectMap;
        std::vector<std::string> objectLists;
        int maxObjectNameLen = 0;
        for (auto kv : objectMap)
        {
            if (kv.first.length() > maxObjectNameLen)
            {
                maxObjectNameLen = kv.first.size();
            }
            objectLists.push_back(kv.first);
        }
        std::cout << "==========================================" << std::endl;
        std::cout << "               Objects List               " << std::endl;
        std::cout << "------------------------------------------" << std::endl;
        for (int i = 0; i < objectLists.size(); i++)
        {
            if (i % 3 == 0 && i != 0)
            {
                std::cout << std::endl;
            }
            std::cout << std::setw(maxObjectNameLen + 1) << objectLists[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "------------------------------------------" << std::endl;


        std::vector<float> target(2, 0);
        srand(time(NULL));
//        std::cin >> ws;
        std::string object;
        std::cout << "\x1B[33m" << "Enter the Target Object " << "\x1B[0m" << std::endl;
        std::getline(std::cin, object);
        std::cout << "Your input is: [" << object << "]" << std::endl;
        if (objectMap.count(object) > 0)
        {
            if(!RandomlyGetObjPose(object, target, objectMap))
            {
                std::cout << "Object does not exist" << std::endl
                          <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
                mbFindObjCalPoints = false;
                return;
            }
        }
        else
        {
            std::cout << "Object does not exist" << std::endl
                      <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
            mbFindObjCalPoints = false;
            return;
        }
        std::cout << "Start position: " << start[0] << "  " << start[1] << std::endl;
        std::cout << "Target position: " << target[0] << "  " << target[1] << std::endl;
        mbFindObjCalPoints = false;
        if (mPathPlanning->PlanPath(start, target, mCloud))
        {
            UpdateSolution(mPathPlanning->mpSolution);
        }

    }
    else if (mpSolution && mpSolution->size() > 0)
    {
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        std::vector<float> cameraCenter(2, 0);

        if (!mCameraPose.empty())
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ERwc;
            cv::cv2eigen(Rwc, ERwc);
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Etcw;
            cv::cv2eigen(mCameraPose.rowRange(0, 3).col(3), Etcw);
            Eigen::Vector3f Etwc = -ERwc * Etcw;
            cv::eigen2cv(Etwc, twc);
            float *it = twc.ptr<float>(0);
            cameraCenter[0] = it[0];
            cameraCenter[1] = it[2];
        }
        float fToTargetX = cameraCenter[0] - mpSolution->back()[0];
        float fToTargetZ = cameraCenter[1] - mpSolution->back()[1];
        float fToTargetDistanceSquared = pow(fToTargetX, 2.0) + pow(fToTargetZ, 2.0);
        float fToTargetDistance = sqrt(fToTargetDistanceSquared);
        if (fToTargetDistance < 0.5)
        {
            std::cout << "\x1B[35m" << "Fetch object: close to target,  distance: " << fToTargetDistance
                      << "\x1B[0m" << std::endl;
            unique_lock<mutex> lock(mMutexPath);
            mpSolution->clear();
        }
    }
    mPathPlanning->ShowPlannedPath();
}

void MapDrawer::GetPath(std::vector<std::vector<float> > &path)
{
    unique_lock<mutex> lock(mMutexPath);
    if (mpSolution && mpSolution->size() > 0) {
        path = *mpSolution;
    }
}

void MapDrawer::CleanSolution()
{
    {
        unique_lock<mutex> lock(mMutexPath);
        mpSolution->clear();
    }
}

void MapDrawer::UpdateSolution(std::shared_ptr<std::vector<std::vector<float> > > &pSolution)
{
    unique_lock<mutex> lock(mMutexPath);
    mpSolution = pSolution;
}

//void MapDrawer::FindObjects()
//{
//    if (mbFindObjCalPoints)
//    {
//        mPathPlanning->reset();
//        CalPointCloud();
//        auto objectMap = mpMap->mObjectMap;
//        std::vector<std::string> objectLists;
//        int maxObjectNameLen = 0;
//        for (auto kv : objectMap)
//        {
//            if (kv.first.length() > maxObjectNameLen)
//            {
//                maxObjectNameLen = kv.first.size();
//            }
//            objectLists.push_back(kv.first);
//        }
//        std::cout << "==========================================" << std::endl;
//        std::cout << "               Objects List               " << std::endl;
//        std::cout << "------------------------------------------" << std::endl;
//        for (int i = 0; i < objectLists.size(); i++)
//        {
//            if (i % 3 == 0 && i != 0)
//            {
//                std::cout << std::endl;
//            }
//            std::cout << std::setw(maxObjectNameLen + 1) << objectLists[i] << " ";
//        }
//        std::cout << std::endl;
//        std::cout << "------------------------------------------" << std::endl;
//        std::vector<float> start(2, 0);
//        std::vector<float> target(2, 0);
//        srand(time(NULL));
//        std::cin.clear();
//        std::cout << "\x1B[33m" << "Enter the Start Object " << "\x1B[0m" << std::endl;
//        std::string object;
//        std::getline(std::cin, object);
//        std::cout << "Your input is: " << object << std::endl;
//        std::cin.clear();
//        if (objectMap.count(object) > 0)
//        {
//            RandomlyGetObjPose(object, start, objectMap);
//        }
//        else
//        {
//            std::cout << "Object does not exist" << std::endl
//                      <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
//            return;
//        }
//
//        std::cout << "\x1B[33m" << "Enter the Target Object " << "\x1B[0m" << std::endl;
//        std::getline(std::cin, object);
//        std::cout << "Your input is: " << object << std::endl;
//        std::cin.clear();
//        if (objectMap.count(object) > 0)
//        {
//            RandomlyGetObjPose(object, target, objectMap);
//        }
//        else
//        {
//            std::cout << "Object does not exist" << std::endl
//                      <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
//            return;
//        }
//        std::cout << "Start position: " << start[0] << "  " << start[1] << std::endl;
//        std::cout << "Target position: " << target[0] << "  " << target[1] << std::endl;
//        mbFindObjCalPoints = false;
//        mPathPlanning->PlanPath(start, target, mCloud);
//    }
//    mPathPlanning->ShowPlannedPath();
//}

void MapDrawer::GeneratePointCloud(const vector<std::shared_ptr<KeyFrame> > &vpKFs,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   int begin,
                                   int step)
{

    int keyFrameNum = vpKFs.size();

    for (int i = begin; i < keyFrameNum; i += step) {
        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
        std::stringstream ss;
        ss << msPointCloudPath << "/KeyFrame-" << std::setw(8) << std::setfill('0') << pKF->mnId << ".bin";
        std::string pointCloudName = ss.str();
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points;
        std::ifstream is(pointCloudName);
        if (is)
        {
            boost::archive::binary_iarchive ia(is, boost::archive::no_header);
            ia >> points;
        }
        else
        {
            continue;
        }
        for (int j = 0; j < points.size(); j++)
        {
            float xx, yy, zz;
            pcl::PointXYZ tmpPoint = points[j];

            cv::Mat Twc = pKF->GetPoseInverse();
            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ETwc;
            cv::cv2eigen(Twc,ETwc);
            Eigen::Vector4f X3Dc(tmpPoint.x, tmpPoint.y, tmpPoint.z, 1);
            Eigen::Vector4f X3Dw = ETwc * X3Dc;
            xx = X3Dw[0];
            yy = X3Dw[1];  // y axis is facing downward!!
            zz = X3Dw[2];

//            //Multiplication below with OpenCV Mat is much slower
//            cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << tmpPoint.x, tmpPoint.y, tmpPoint.z, 1);
//            cv::Mat x3Dw = pKF->GetPoseInverse() * x3Dc;
//            xx = x3Dw.at<float>(0, 0);
//            yy = x3Dw.at<float>(1, 0);
//            zz = x3Dw.at<float>(2, 0);

//            // This direct method is as fast as the Eigen's method above
//            float *itX =  Twc.ptr<float>(0);
//            xx = itX[0] * tmpPoint.x + itX[1] * tmpPoint.y + itX[2] * tmpPoint.z + itX[3];
//            float *itY =  Twc.ptr<float>(1);
//            yy = itY[0] * tmpPoint.x + itY[1] * tmpPoint.y + itY[2] * tmpPoint.z + itY[3];
//            float *itZ =  Twc.ptr<float>(2);
//            zz = itZ[0] * tmpPoint.x + itZ[1] * tmpPoint.y + itZ[2] * tmpPoint.z + itZ[3];
            pcl::PointXYZ point;
            point.x = xx;
            point.y = yy;
            point.z = zz;
            cloud->points.push_back(point);
//            if (yy >= -mfHeightUpperBound && yy <= -mfHeightLowerBound)
//            {
//                pcl::PointXYZ point;
//                point.x = xx;
//                point.y = yy;
//                point.z = zz;
//                cloud->points.push_back(point);
//            }


        }



//
//        cv::Mat depth = cv::imread(depthImageName.c_str(), cv::IMREAD_UNCHANGED);
//        if (depth.empty())
//        {
//            continue;
//        }
//        depth.convertTo(depth, CV_32F, mpMap->mDepthMapFactor);

//        int imgStep = 3;
//        for (int r = 0; r < depth.rows; r += imgStep) {
//            const float *itD = depth.ptr<float>(r);
//            const float y = mpMap->mLookupY.at<float>(0, r);
//            const float *itX = mpMap->mLookupX.ptr<float>();
//            for (size_t c = 0; c < (size_t) depth.cols; c += imgStep, itD += imgStep, itX += imgStep) {
//                float depthValue = *itD;
//                float xx, yy, zz;
//                if (depthValue > 0.1 && depthValue < 12.0) {
//                    float zc = depthValue;
//                    float xc = *itX * depthValue;
//                    float yc = y * depthValue;
//                    cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << xc, yc, zc, 1);
//                    cv::Mat x3Dw = pKF->GetPoseInverse() * x3Dc;
//                    xx = x3Dw.at<float>(0, 0);
//                    yy = x3Dw.at<float>(1, 0);
//                    zz = x3Dw.at<float>(2, 0);
//                    if (abs(yy) < 0.55) {
//                        point.x = xx;
//                        point.y = yy;
//                        point.z = zz;
//                        unique_lock<mutex> lock(mMutexCloud);
//                        cloud->points.push_back(point);
//                    }
//
//                }
//
//            }
//        }
    }
}

void MapDrawer::SaveDenseMapToPCD()
{
    CalPointCloud();
    boost::filesystem::path parentPath(msPointCloudPath);
    parentPath = parentPath.parent_path();
    boost::filesystem::path resultPath("Results");
    resultPath = parentPath / resultPath;
    boost::filesystem::create_directories(resultPath);
    boost::filesystem::path denseMapASCIIPath("densemap_ascii.pcd");
    denseMapASCIIPath = resultPath / denseMapASCIIPath;
    pcl::io::savePCDFileASCII(denseMapASCIIPath.string(), *mCloud);
    boost::filesystem::path denseMapBinaryPath("densemap_binary.pcd");
    denseMapBinaryPath = resultPath / denseMapBinaryPath;
    pcl::io::savePCDFileBinary(denseMapBinaryPath.string(), *mCloud);
    std::cout << "saving pcd files done ... " << std::endl;
}

void MapDrawer::SaveDenseMapToSTL()
{
    CalPointCloud();
    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (mCloud);
//    n.setInputCloud (mCloud);
//    n.setSearchMethod (tree);
//    n.setKSearch (10);
//    n.compute (*normals);
//    std::cout << "compute normal done ..." << std::endl;
//    //* normals should not contain the point normals + surface curvatures
//
//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*mCloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals


    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (mCloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.1);

    // Reconstruct
    mls.process (*mls_points);

    std::cout << "Smoothing and normal estimation done ..." << std::endl;

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mls_points);

    pcl::PolygonMesh triangles;
    pcl::Poisson<pcl::PointNormal> psn;
    //pcl::PolygonMesh triangles;

    psn.setInputCloud(mls_points);
    psn.setSearchMethod(tree2);
    psn.setDepth(9);
    psn.setSolverDivide(7);
    psn.setIsoDivide(7);
    psn.setSamplesPerNode(2.0);
    psn.reconstruct (triangles);

//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//
//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (0.5);
//
//    // Set typical values for the parameters
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (2000);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);
//
//    // Get result
//    gp3.setInputCloud (mls_points);
//    gp3.setSearchMethod (tree2);
//    gp3.reconstruct (triangles);
    std::cout << "triangulation done ..." << std::endl;

    boost::filesystem::path parentPath(msPointCloudPath);
    parentPath = parentPath.parent_path();
    boost::filesystem::path resultPath("Results");
    resultPath = parentPath / resultPath;
    boost::filesystem::create_directories(resultPath);
    boost::filesystem::path meshPath("densemap.vtk");
    meshPath = resultPath / meshPath;
    pcl::io::saveVTKFile(meshPath.string(), triangles);

    boost::filesystem::path stlPath("densemap.stl");
    stlPath = resultPath / stlPath;
    pcl::io::savePolygonFileSTL(stlPath.string(), triangles);
    std::cout << "saving stl file done ..." << std::endl;

}


void MapDrawer::BuildOctomap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
//    {
//        unique_lock<mutex> lock(mMutexMCloud);
//        if (cloud->points.size() > 0)
//            FilterPointCloud(cloud, output);
//        else
//            return;
//    }
    // Generate octomap
    octomap::OcTree tree(0.05);
    for (auto p:cloud->points) {
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }
    tree.updateInnerOccupancy();
    boost::filesystem::path parentPath(msPointCloudPath);
    parentPath = parentPath.parent_path();
    boost::filesystem::path resultPath("Results");
    resultPath = parentPath / resultPath;
    boost::filesystem::create_directories(resultPath);
    boost::filesystem::path octoMapPath("octomap_office.ot");
    octoMapPath = resultPath / octoMapPath;
    tree.write(octoMapPath.string());
    std::cout << "saving octomap done ..." << std::endl;
    return;
}

void
MapDrawer::FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    cout << "Original Cloud Size:" << cloud->points.size() << endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    cout << "Cloud Size after NaN Removal:" << cloud->points.size() << endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRandomFilter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIQRFilter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel(new pcl::PointCloud<pcl::PointXYZ>);

//    int n_points = cloud->points.size() * sampleRatio;
//    pcl::RandomSample<pcl::PointXYZ> randomFilter;
//    randomFilter.setSample(n_points);
//    randomFilter.setInputCloud(cloud);
//    randomFilter.filter(*cloudRandomFilter);
//    cout << "Cloud Size After Random Sample Filter:" << cloudRandomFilter->points.size() << endl;


    // voxel filter
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setLeafSize(0.03f, 0.03f, 0.03f);       // resolution
    voxelFilter.setInputCloud(cloud);
    voxelFilter.filter(*cloudVoxel);
    cout << "Cloud Size After Voxel Filter:" << cloudVoxel->points.size() << endl;

    if (cloudVoxel->points.size() > 0)
    {
        std::vector<std::vector<float> > vXYZCoords(3, std::vector<float> ());
        std::vector<std::vector<float> > vIQRBounds(3, std::vector<float> (2, 0));
        for (auto &p : cloudVoxel->points)
        {
            vXYZCoords[0].push_back(p.x);
            vXYZCoords[1].push_back(p.y);
            vXYZCoords[2].push_back(p.z);
        }
        for (int i = 0; i < vXYZCoords.size(); i++)
        {
            std::vector<float> &vCoords = vXYZCoords[i];
            auto const Q1 = vCoords.size() / 4;
            auto const Q3 = vCoords.size() * 3 / 4;
            std::nth_element(vCoords.begin(),          vCoords.begin() + Q1, vCoords.end());
            std::nth_element(vCoords.begin() + Q1 + 1, vCoords.begin() + Q3, vCoords.end());
            float q25 = *(vCoords.begin() + Q1);
            float q75 = *(vCoords.begin() + Q3);
            float iqr = q75 - q25;
            float lowerBound = q25 - iqr * 1.5;
            float upperBound = q75 + iqr * 1.5;
            vIQRBounds[i][0] = lowerBound;
            vIQRBounds[i][1] = upperBound;
        }
        cloudIQRFilter->points.clear();
        for (auto &p : cloudVoxel->points)
        {
            if (p.x >= vIQRBounds[0][0] && p.x <= vIQRBounds[0][1] &&
                p.y >= vIQRBounds[1][0] && p.y <= vIQRBounds[1][1] &&
                p.z >= vIQRBounds[2][0] && p.z <= vIQRBounds[2][1])
            {
                cloudIQRFilter->points.push_back(p);
            }
        }
        cout << "Current Frame Cloud Size After IQR Filter:" << cloudIQRFilter->points.size() << endl;
    }
    else
    {
        cloudIQRFilter = cloudVoxel;
    }

    // statistical removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(15);
    sor.setStddevMulThresh(1.0);
    sor.setInputCloud(cloudIQRFilter);
    sor.filter(*output);
    cout << "Cloud Size After Statistical Filter:" << output->points.size() << endl;

}

void MapDrawer::DrawMapPoints()
{
    const vector<std::shared_ptr<MapPoint> > &vpMPs = mpMap->GetAllMapPoints();
    const vector<std::shared_ptr<MapPoint> > &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<std::shared_ptr<MapPoint> > spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        float *it = pos.ptr<float>(0);
        glVertex3f(it[0], it[1], it[2]);
//        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (set<std::shared_ptr<MapPoint> >::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if (!(*sit) || (*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        float *it = pos.ptr<float>(0);
        glVertex3f(it[0], it[1], it[2]);
//        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    const vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();

    if (bDrawKF) {
        for (size_t i = 0; i < vpKFs.size(); i++) {
            std::shared_ptr<KeyFrame> spKF = vpKFs[i];
            cv::Mat Twc = spKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();
        }
    }

    if (bDrawGraph) {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        for (size_t i = 0; i < vpKFs.size(); i++) {
            // Covisibility Graph
            std::weak_ptr<KeyFrame> pKF = vpKFs[i];
            if (pKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spKF = pKF.lock();
            const vector<std::weak_ptr<KeyFrame> > vCovKFs = spKF->GetCovisiblesByWeight(100);
            cv::Mat Ow = spKF->GetCameraCenter();
            if (!vCovKFs.empty()) {
                for (vector<std::weak_ptr<KeyFrame> >::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                     vit != vend; vit++) {
                    std::weak_ptr<KeyFrame> pKFi = *vit;
                    if (pKFi.expired())
                        continue;
                    std::shared_ptr<KeyFrame> spKFi = pKF.lock();
                    if (spKFi->mnId < spKF->mnId)
                        continue;
                    cv::Mat Ow2 = spKFi->GetCameraCenter();
                    float *it = Ow.ptr<float>(0);
                    glVertex3f(it[0], it[1], it[2]);
                    float *it2 = Ow2.ptr<float>(0);
                    glVertex3f(it2[0], it2[1], it2[2]);
//                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                    glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                }
            }

            // Spanning tree
            std::weak_ptr<KeyFrame> pParent = spKF->GetParent();
            if (!pParent.expired()) {
                cv::Mat Owp = pParent.lock()->GetCameraCenter();
                float *it = Ow.ptr<float>(0);
                glVertex3f(it[0], it[1], it[2]);
                float *itp = Owp.ptr<float>(0);
                glVertex3f(itp[0], itp[1], itp[2]);
//                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
            }

            // Loops
            std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >
                sLoopKFs = spKF->GetLoopEdges();
            for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator
                     sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                std::weak_ptr<KeyFrame> pKFi = *sit;
                if (pKFi.expired())
                    continue;
                std::shared_ptr<KeyFrame> spKFi = pKF.lock();
                if (spKFi->mnId < spKF->mnId)
                    continue;
                cv::Mat Owl = spKFi->GetCameraCenter();
                float *it = Ow.ptr<float>(0);
                glVertex3f(it[0], it[1], it[2]);
                float *itl = Owl.ptr<float>(0);
                glVertex3f(itl[0], itl[1], itl[2]);
//                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();
#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();

    //Draw camera on the projection plane
    Twc.m[13] = 5.0;
    glPushMatrix();
#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif
    glLineWidth(mCameraLineWidth);
    glColor3f(0.5f, 0.0f, 0.5f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if (!mCameraPose.empty()) {
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
//            twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ERwc;
            cv::cv2eigen(Rwc,ERwc);
            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Etcw;
            cv::cv2eigen(mCameraPose.rowRange(0, 3).col(3),Etcw);
            Eigen::Vector3f Etwc = -ERwc * Etcw;
            cv::eigen2cv(Etwc, twc);
        }
        float *it1 =  Rwc.ptr<float>(0);
        float *it2 =  Rwc.ptr<float>(1);
        float *it3 =  Rwc.ptr<float>(2);
        M.m[0] = it1[0];
        M.m[1] = it2[0];
        M.m[2] = it3[0];
        M.m[3] = 0.0;

        M.m[4] = it1[1];
        M.m[5] = it2[1];
        M.m[6] = it3[1];
        M.m[7] = 0.0;

        M.m[8] = it1[2];
        M.m[9] = it2[2];
        M.m[10] = it3[2];
        M.m[11] = 0.0;

        float *itTwc =  twc.ptr<float>(0);
        M.m[12] = itTwc[0];
        M.m[13] = itTwc[1];
        M.m[14] = itTwc[2];
        M.m[15] = 1.0;
//        M.m[0] = Rwc.at<float>(0, 0);
//        M.m[1] = Rwc.at<float>(1, 0);
//        M.m[2] = Rwc.at<float>(2, 0);
//        M.m[3] = 0.0;
//
//        M.m[4] = Rwc.at<float>(0, 1);
//        M.m[5] = Rwc.at<float>(1, 1);
//        M.m[6] = Rwc.at<float>(2, 1);
//        M.m[7] = 0.0;
//
//        M.m[8] = Rwc.at<float>(0, 2);
//        M.m[9] = Rwc.at<float>(1, 2);
//        M.m[10] = Rwc.at<float>(2, 2);
//        M.m[11] = 0.0;
//
//        M.m[12] = twc.at<float>(0);
//        M.m[13] = twc.at<float>(1);
//        M.m[14] = twc.at<float>(2);
//        M.m[15] = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::clear()
{
    mCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void MapDrawer::CloseOctoMapThread()
{
    if (mpThreadOctomap) {
        mpThreadOctomap->join();
    }
    mpThreadOctomap.reset();
}


bool MapDrawer::RandomlyGetObjPose(std::string object,
                                   std::vector<float> &output,
                                   std::unordered_map<std::string,
                                                      std::unordered_map<long unsigned int, ObjectPos> > &objectMap)
{
    output.resize(2);
    std::vector<std::vector<double> > poses;
    int maxTrials = 100;
    int count = 0;
    std::unordered_map<long unsigned int, ObjectPos>::iterator randomIt;
    do
    {
        randomIt = std::next(std::begin(objectMap[object]), rand() % objectMap[object].size());
        poses = (*randomIt).second.Pcs;
        count++;
        if (count >= maxTrials)
        {
            std::cout << "\x1B[31m" << "Cannot find object [" << object << "]" << "\x1B[0m" << std::endl;
            return false;
        }
    }while(poses.size() <= 0);
    std::vector<double> dSelectedObj(3, 0.0);
    count = 0;
    do{
        int randInt = rand() % poses.size();
        dSelectedObj = poses[randInt];
        count++;
        if (count >= maxTrials)
        {
            std::cout << "Cannot find object [" << object << "]" << std::endl;
            return false;
        }
    }while(dSelectedObj[0] == 0.0 && dSelectedObj[1] == 0.0 && dSelectedObj[2] == 0.0);

    std::vector<float> fSelectedObj(dSelectedObj.begin(), dSelectedObj.end());
    cv::Mat Twc;
//    cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << fSelectedObj[0], fSelectedObj[1], fSelectedObj[2], 1.0);
    vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();
    bool KeyFrameFound = false;
    for (vector<std::shared_ptr<KeyFrame> >::iterator it = vpKFs.begin(); it != vpKFs.end(); it++)
    {
        if ((*it)->mnId == (*randomIt).first)
        {
            KeyFrameFound = true;
            Twc = (*it)->GetPoseInverse();
        }
    }
    if (!KeyFrameFound)
    {
        std::cout << "\x1B[31m" << "ERROR occured: keyframe missing" << "\x1B[0m" << std::endl;
        return false;
    }
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ETwc;
    cv::cv2eigen(Twc,ETwc);
    Eigen::Vector4f EX3Dc;
    EX3Dc << fSelectedObj[0], fSelectedObj[1], fSelectedObj[2], 1.0;
    Eigen::Vector4f EX3Dw = ETwc * EX3Dc;
//    cv::Mat x3Dw = Twc * x3Dc;
//    output[0] = x3Dw.at<float>(0, 0);
//    output[1] = x3Dw.at<float>(2, 0);
    output[0] = EX3Dw[0];
    output[1] = EX3Dw[2];
    return true;
}

} //namespace ORB_SLAM
