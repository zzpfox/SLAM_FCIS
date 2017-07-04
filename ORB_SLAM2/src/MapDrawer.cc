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
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/filters/random_sample.h>
#include <sstream>
#include <iomanip>
#include <limits>
#include <stdlib.h>
#include <time.h>
#include "OmplPathPlanning.h"
#include <opencv2/core/eigen.hpp>

namespace ORB_SLAM2
{
std::string MapDrawer::msPointCloudPath = "./Data/PointCloud";
MapDrawer::MapDrawer(std::shared_ptr<Map> pMap, const string &strSettingPath)
    : mpMap(pMap),
      mCloud(new pcl::PointCloud<pcl::PointXYZ>),
      mfcLeafSize(0.025)
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
    msPlanner = static_cast<std::string> (fSettings["PathPlanningPlanner"]);
    mbCalPointCloud = true;
    mbFindObjCalPoints = true;
    mvBounds = std::vector<float> (4, 0.0);
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
        glVertex3f(p.x, p.y, p.z);
    }

    glEnd();
}

void MapDrawer::CalPointCloud()
{
    std::chrono::time_point<std::chrono::system_clock> start, start2;
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
    int numThreads = 4;
    std::thread threads[numThreads];
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    for (int i = 0; i < numThreads; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        clouds.push_back(tmp);
    }
    for (int i = 0; i < numThreads; i++) {
        threads[i] = std::thread(&MapDrawer::GeneratePointCloud, this, std::cref(sampledVPKFs), clouds[i], i,
                                 numThreads, 0.55, -0.55);
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
        if (clouds[0]->points.size() > 0)
            FilterPointCloud(cloud, mCloud);
//            mCloud = cloud;
    }
    if (mpThreadOctomap) {
        mpThreadOctomap->join();
    }
    mpThreadOctomap.reset(new thread(&MapDrawer::BuildOctomap, this, mCloud));

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Total Number of Points: " << mCloud->size() << std::endl;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}

void MapDrawer::FindObjects()
{
    if (mbFindObjCalPoints)
    {
        CalPointCloud();
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
        std::vector<float> start(2, 0);
        std::vector<float> target(2, 0);
        srand(time(NULL));
        std::cin.clear();
        std::cout << "\x1B[33m" << "Enter the Start Object " << "\x1B[0m" << std::endl;
        std::string object;
        std::getline(std::cin, object);
        std::cout << "Your input is: " << object << std::endl;
        std::cin.clear();
        if (objectMap.count(object) > 0)
        {
            RandomlyGetObjPose(object, start, objectMap);
        }
        else
        {
            std::cout << "Object does not exist" << std::endl
                      <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
            return;
        }

        std::cout << "\x1B[33m" << "Enter the Target Object " << "\x1B[0m" << std::endl;
        std::getline(std::cin, object);
        std::cout << "Your input is: " << object << std::endl;
        std::cin.clear();
        if (objectMap.count(object) > 0)
        {
            RandomlyGetObjPose(object, target, objectMap);
        }
        else
        {
            std::cout << "Object does not exist" << std::endl
                      <<"Please click \'Fetch Objects\' again to enter a valid object name" << std::endl;
            return;
        }
        std::cout << "Start position: " << start[0] << "  " << start[1] << std::endl;
        std::cout << "Target position: " << target[0] << "  " << target[1] << std::endl;
        mbFindObjCalPoints = false;
        mvStart = start;
        mvTarget = target;
//        SimplePathPlanning(mvStart, mvTarget, mSolution, mObstacles);
        OmplPathPlanning(mvStart, mvTarget, mSolution, mObstacles);
    }
    if (mvStart.size() == 2 && mvTarget.size() == 2)
    {
        glPointSize(mPointSize * 6);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(mvStart[0], 5, mvStart[1]);
        glVertex3f(mvTarget[0], 5, mvTarget[1]);
        glEnd();
    }
    glLineWidth(mKeyFrameLineWidth * 4);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    for (std::vector<std::vector<float> >::iterator st = mSolution.begin(); (st + 1) != mSolution.end(); st++) //in right order
    {
        glVertex3f((*st)[0], 5.0, (*st)[1]);
        glVertex3f((*(st+1))[0], 5.0, (*(st+1))[1]);
    }
    glEnd();
    glPointSize(mPointSize);
    {
        glPushAttrib(GL_ENABLE_BIT);
        if (mSolution.size() > 0 && mvStart.size() == 2 && mvTarget.size() == 2)
        {
            glLineStipple(2, 0x00FF);
            glEnable(GL_LINE_STIPPLE);
            glLineWidth(mKeyFrameLineWidth * 4);
            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_LINES);
            glVertex3f(mvStart[0], 5.0, mvStart[1]);
            glVertex3f(mSolution.front()[0], 5.0, mSolution.front()[1]);
            glEnd();
            glBegin(GL_LINES);
            glVertex3f(mvTarget[0], 5.0, mvTarget[1]);
            glVertex3f(mSolution.back()[0], 5.0, mSolution.back()[1]);
            glEnd();
            glPopAttrib();

        }
    }


    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (int i = 0; i < mObstacles.size(); i++)
    {
        for (int j = 0; j < mObstacles[0].size(); j++)
        {
            if (mObstacles[i][j] == 1)
            {
                glVertex3f(j * mfcLeafSize + mvBounds[0],
                           5,
                           i * mfcLeafSize + mvBounds[2]);
            }
        }
    }
    glEnd();

}

void MapDrawer::GeneratePointCloud(const vector<std::shared_ptr<KeyFrame> > &vpKFs,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   int begin,
                                   int step,
                                   float heightUpperBound,
                                   float heightLowerBound)
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
            yy = X3Dw[1];
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
            if (yy <= heightUpperBound && yy >= heightLowerBound) {
                pcl::PointXYZ point;
                point.x = xx;
                point.y = yy;
                point.z = zz;
                cloud->points.push_back(point);
            }
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

void MapDrawer::BuildOctomap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
//    {
//        unique_lock<mutex> lock(mMutexMCloud);
//        if (cloud->points.size() > 0)
//            FilterPointCloud(cloud, output);
//        else
//            return;
//    }
    // Generate octomap
    octomap::OcTree tree(0.05);
    for (auto p:output->points) {
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
    return;
}

void
MapDrawer::FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRandomFilter(new pcl::PointCloud<pcl::PointXYZ>);
    cout << "Original Cloud Size:" << cloud->points.size() << endl;

    int n_points = cloud->points.size() * 0.25;
    pcl::RandomSample<pcl::PointXYZ> randomFilter;
    randomFilter.setSample(n_points);
    randomFilter.setInputCloud(cloud);
    randomFilter.filter(*cloudRandomFilter);
    cout << "Cloud Size After Random Sample Filter:" << cloudRandomFilter->points.size() << endl;

//        // voxel filter
//        pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
//        voxelFilter.setLeafSize(0.03f, 0.03f, 0.03f);       // resolution
//        voxelFilter.setInputCloud(cloud);
//        voxelFilter.filter(*cloudVoxel);

    // statistical removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.setInputCloud(cloudRandomFilter);
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
    mCloud.reset();
    mSolution.clear();
    mObstacles.clear();
}

void MapDrawer::CloseOctoMapThread()
{
    if (mpThreadOctomap) {
        mpThreadOctomap->join();
    }
    mpThreadOctomap.reset();
}
bool MapDrawer::SimpleRRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
                                    std::vector<std::vector<int> > &mObstacles,
                                    int nMiddleX, int nMiddleY, int ncStepSize)
{
    int nDisMin = std::numeric_limits<int>::max();
    int nConnect = 1;

    std::shared_ptr<RRTNode> pMin = std::make_shared<RRTNode> (std::vector<int> (2, nDisMin));
    for (vector<std::shared_ptr<RRTNode> >::iterator sit = tree.begin(); sit != tree.end(); sit++) {
        std::vector<int> vTmpPoint = (*sit)->mPoint;
        int nTmpDist = abs(nMiddleX - vTmpPoint[0]) + abs(nMiddleY - vTmpPoint[1]);
        if (nTmpDist < nDisMin) {
            nDisMin = nTmpDist;
            pMin = *sit;
        }
    }

    if (nDisMin < 4)
        return false;//means too close

    int nStepX = pMin->mPoint[0] > nMiddleX ? (-1) : 1;
    int nStepY = pMin->mPoint[1] > nMiddleY ? (-1) : 1;// different step along different direction

    //use sin,cos to determine direction
    float fRadius = sqrt(pow(float(pMin->mPoint[0] - nMiddleX), 2.0) + pow(float(pMin->mPoint[1] - nMiddleY), 2.0));
    int nStepSizeX = int(float(ncStepSize * abs(pMin->mPoint[0] - nMiddleX)) / fRadius) + 1;// sin
    int nStepSizeY = int(float(ncStepSize * abs(pMin->mPoint[1] - nMiddleY)) / fRadius) + 1;// cos

    for (int i = pMin->mPoint[0]; abs(i - pMin->mPoint[0]) < nStepSizeX; i += nStepX) {
        int OUT = 0;
        for (int j = pMin->mPoint[1]; abs(j - pMin->mPoint[1]) < nStepSizeY; j += nStepY) {
            if (i < 0 || i >= mObstacles[0].size() ||
                j < 0 || j >= mObstacles.size() ||
                mObstacles[j][i] == 1)//found obstacle or out of bounds
            {
                nConnect = 0;// not directly connect
                OUT = 1;
                break;
            }
        }
        if (OUT == 1)
            break;
    }

    if (nConnect == 1) {// add
//            nStepX = pMinStart->mPoint[0] > nMiddleX ? (-1) : 1;
//            nStepY = pMinStart->mPoint[1] > nMiddleY ? (-1) : 1;

        int x = nStepX * nStepSizeX + pMin->mPoint[0];
        int y = nStepY * nStepSizeY + pMin->mPoint[1];
        std::vector<int> vTmpPoint = {x, y};
        std::shared_ptr<RRTNode> tmpNode = std::make_shared<RRTNode> (vTmpPoint);
        tmpNode->addParent(pMin);
        tree.push_back(tmpNode);
        return true;
    }
    else
    {
        return false;
    }

}

bool MapDrawer::SimpleRRTTreesIntersect(std::vector<std::shared_ptr<RRTNode> > &tree,
                                        std::vector<std::shared_ptr<RRTNode> > &treePop,
                                        std::vector<std::vector<int> > &mObstacles,
                                        std::vector<std::shared_ptr<RRTNode> > &vSolution,
                                        int ncStepSize)
{
    //check whether new node near the other tree, first check near target tree
    bool bConnect = false;
    for (std::vector<std::shared_ptr<RRTNode> >::iterator sit = tree.begin(); sit != tree.end(); sit++) {
        int y = (*sit)->mPoint[1];
        int x = (*sit)->mPoint[0];
        if (abs(treePop.back()->mPoint[0] - x) + abs(treePop.back()->mPoint[1]  - y) <= 2 * ncStepSize) {
            int nOut = 0;
            for (int i = min(treePop.back()->mPoint[0], x); i <= max(treePop.back()->mPoint[0], x); i++) {

                for (int j = min(treePop.back()->mPoint[1], y); j <= max(treePop.back()->mPoint[1], y); j++) {
                    if (mObstacles[j][i] == 1) {
                        nOut = 1;
                        bConnect = false;
                        break;
                    }
                }
                if (nOut == 1)
                    break;
            }
            if (!nOut)//connect
            {
                bConnect = true;//connect to start tree
                vSolution.clear();
                vSolution.push_back(treePop.back());
                while(vSolution.back()->mParent)
                {
                    vSolution.push_back(vSolution.back()->mParent);
                }
                std::reverse(vSolution.begin(), vSolution.end());
                vSolution.push_back(*sit);
                while(vSolution.back()->mParent)
                {
                    vSolution.push_back(vSolution.back()->mParent);
                }
                break;
            }
        }
    }
    return bConnect;
}
void MapDrawer::SimplePathPlanning(std::vector<float> &start, std::vector<float> &target,
                                   std::vector<std::vector<float> > &solution,
                                   std::vector<std::vector<int> > &mObstacles)
{

    if (start.size() != 2 || target.size() != 2)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }

    mvBounds[0] = std::numeric_limits<float>::max();
    mvBounds[1] = std::numeric_limits<float>::min();
    mvBounds[2] = std::numeric_limits<float>::max();
    mvBounds[3] = std::numeric_limits<float>::min();
    for (auto p: mCloud->points)
    {
        if (p.x < mvBounds[0])
        {
            mvBounds[0] = p.x;
        }
        if (p.x > mvBounds[1])
        {
            mvBounds[1] = p.x;
        }
        if (p.z < mvBounds[2])
        {
            mvBounds[2] = p.z;
        }
        if (p.z > mvBounds[3])
        {
            mvBounds[3] = p.z;
        }
    }
    const int nMaxIter = 10000;
    const int ncSizeX = static_cast<int> ((mvBounds[1] - mvBounds[0] + 0.2) / mfcLeafSize);
    const int ncSizeY = static_cast<int> ((mvBounds[3] - mvBounds[2] + 0.2) / mfcLeafSize);
    const float fcObstacleWidth = 9;//these three should choose properly. ncSizeMax*mfcLeafSize is the range of the enviroment.
    //LeafSIze * fcObstacleWidth is the width of the obstacles that extend, equal to half of the robot width
    //37.5cm--->15 is the whole body, 22.5cm-->9 is the test body
    mObstacles.clear();
    mObstacles = std::vector<std::vector<int> > (ncSizeY, std::vector<int> (ncSizeX, 0));
    // origin is (ncSizeMax,ncSizeMax), minimal leaf is mfcLeafSize,
    // so the range is [-ncSizeMax * mfcLeafSize,ncSizeMax * mfcLeafSize]*
    // [-ncSizeMax * mfcLeafSize,ncSizeMax * mfcLeafSize]
    std::vector<int> nStart(2, 0);
    std::vector<int> nTarget(2, 0);
    nStart[0] = static_cast<int> ((start[0] - mvBounds[0]) / mfcLeafSize);
    nStart[1] = static_cast<int> ((start[1] - mvBounds[2]) / mfcLeafSize);
    nTarget[0] = static_cast<int> ((target[0] - mvBounds[0]) / mfcLeafSize);
    nTarget[1] = static_cast<int> ((target[1] - mvBounds[2]) / mfcLeafSize);
    if (nStart[0] < 0 || nStart[0] >= ncSizeX ||
        nStart[1] < 0 || nStart[1] >= ncSizeY ||
        nTarget[0] < 0 || nTarget[0] >= ncSizeX ||
        nTarget[1] < 0 || nTarget[1] >= ncSizeY)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors out of bounds"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
//    std::transform(start.begin(), start.end(), nStart.begin(),
//                   [mfcLeafSize, ncSizeMax](float d) -> int{return
//                       (static_cast<int> (d / mfcLeafSize) + ncSizeMax);});
//    std::transform(target.begin(), target.end(), nTarget.begin(),
//                   [mfcLeafSize, ncSizeMax](float d) -> int{return
//                       (static_cast<int> (d / mfcLeafSize) + ncSizeMax);});

    for (auto p : mCloud->points) {
        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcLeafSize);
        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcLeafSize);
        for (int i = nTmpX - fcObstacleWidth; i < nTmpX + fcObstacleWidth; i++)
        {
            for (int j = nTmpY - fcObstacleWidth; j < nTmpY + fcObstacleWidth; j++)
            {
                if (i >= 0 && i < ncSizeX && j >= 0 && j < ncSizeY)
                {
                    mObstacles[j][i] = 1;
                }

            }
        }
    }
    GetClosestFreePoint(nStart, mObstacles, 40);
    GetClosestFreePoint(nTarget, mObstacles, 40);

    std::vector<std::shared_ptr<RRTNode> > vStartTree;
    std::vector<std::shared_ptr<RRTNode> > vTargetTree;

    vStartTree.push_back(std::make_shared<RRTNode> (nStart));
    vTargetTree.push_back(std::make_shared<RRTNode> (nTarget));
    int nCount = 0;
    int ncStepSize = 6; //pixel

    std::vector<std::shared_ptr<RRTNode> > vSolution;
    srand (time(NULL));
    bool bFoundPath = false;
    while (nCount < nMaxIter)  //max iteration
    {
        nCount++;
        int nMiddleX = rand() % ncSizeX;
        int nMiddleY = rand() % ncSizeY;

        int innerMaxIter = 50;
        for (int iter = 0; iter < innerMaxIter; iter++)
        {
            if (mObstacles[nMiddleY][nMiddleX] == 0)
            {
                break;
            }
            nMiddleX = rand() % ncSizeX;
            nMiddleY = rand() % ncSizeY;
            if (iter >= innerMaxIter - 1)
            {
                std::cout << "\x1B[31m" << "ERROR: cannot generate a collision-free random point"
                          << "\x1B[0m" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        if (!SimpleRRTTreeExpand(vStartTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };
        if (!SimpleRRTTreeExpand(vTargetTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };

        if (!SimpleRRTTreesIntersect(vTargetTree, vStartTree, mObstacles, vSolution, ncStepSize))
        {
            if (!SimpleRRTTreesIntersect(vStartTree, vTargetTree, mObstacles, vSolution, ncStepSize))
            {
                continue;
            }
        }

        bFoundPath = true;
        //the order in Solution is the path, now smooth
        int nSpan = 2; //follow rrt
        while (nSpan < vSolution.size()) {
            bool bChanged = false;
            for (int i = 0; i + nSpan < vSolution.size(); i++) {
                bool bCanErase = true;// true to erase
                int yy = vSolution[i]->mPoint[1];
                int xx = vSolution[i]->mPoint[0];
                int yyy = vSolution[i + nSpan]->mPoint[1];
                int xxx = vSolution[i + nSpan]->mPoint[0];

                for (int ii = min(xx, xxx); ii <= max(xx, xxx); ii++) {
                    for (int jj = min(yy, yyy); jj <= max(yy, yyy); jj++) {
                        if (mObstacles[jj][ii] == 1) {
                            bCanErase = false;
                            break;
                        }
                    }
                    if (!bCanErase)
                        break;
                }


                if (bCanErase) {
                    for (int x = 1; x < nSpan; x++) {
                        vSolution.erase(vSolution.begin() + i + 1);
                    }
                    bChanged = true;
                }
            }

            if (!bChanged) nSpan++;
        }

        solution.clear();
        for (std::vector<std::shared_ptr<RRTNode> >::iterator st = vSolution.begin(); st != vSolution.end(); st++) //in right order
        {
            float yy = (*st)->mPoint[1] * mfcLeafSize + mvBounds[2];
            float xx = (*st)->mPoint[0] * mfcLeafSize + mvBounds[0];
            std::vector<float> point = {xx, yy};
            solution.push_back(point);
        }
        break;
    }

//    glPointSize(2);
//    glBegin(GL_POINTS);
//    glColor3f(1.0,0.0,0.5);
//    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vStartTree.begin();st!=vStartTree.end();st++) {
//
//        int yy = (*st)->mPoint[1];
//        int xx = (*st)->mPoint[0];
//        glVertex3f(xx * mfcLeafSize + mvBounds[0], 5.0, yy * mfcLeafSize + mvBounds[2]);
//    }
//    glEnd();
//
//
//    glPointSize(2);
//    glBegin(GL_POINTS);
//    glColor3f(0.25,0.21,0.16);
//    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vTargetTree.begin();st!=vTargetTree.end();st++) {
//
//        int yy = (*st)->mPoint[1];
//        int xx = (*st)->mPoint[0];
//        glVertex3f(xx * mfcLeafSize + mvBounds[0], 5.0, yy * mfcLeafSize + mvBounds[2]);
//    }
//
//    glEnd();
    if (bFoundPath)
    {
        std::cout << "Find a path after " << nCount << " iterations ..." << std::endl;
    }
    else
    {
        std::cout << "Cannot find a path after " << nMaxIter << " iterations ..." << std::endl;
    }

}

void MapDrawer::GetClosestFreePoint(std::vector<int> &output, std::vector<std::vector<int> > &obstacles, int searchWidth)
{
    if (obstacles[output[1]][output[0]] == 1)
    {
        int nSearchWidth = 40;
        std::vector<int> tmpOutput(2, 0);
        int minDistance = std::numeric_limits<int>::max();
        for (int i = -nSearchWidth; i < nSearchWidth; i++)
        {
            for (int j = -nSearchWidth; j < nSearchWidth; j++)
            {
                int y = output[1] + i;
                int x = output[0] + j;
                if (y >= 0 && y < obstacles.size() && x >= 0 && x < obstacles[0].size())
                {
                    if (obstacles[y][x] == 0 && (abs(i) + abs(j)) < minDistance)
                    {
                        minDistance = abs(i) + abs(j);
                        tmpOutput[0] = x;
                        tmpOutput[1] = y;
                    }

                }
            }
        }
        output = tmpOutput;
    }
}
void MapDrawer::RandomlyGetObjPose(std::string object,
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
            std::cout << "Cannot find object [" << object << "]" << std::endl;
            return;
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
            return;
        }
    }while(dSelectedObj[0] == 0.0 && dSelectedObj[1] == 0.0 && dSelectedObj[2] == 0.0);

    std::vector<float> fSelectedObj(dSelectedObj.begin(), dSelectedObj.end());
    cv::Mat Twc;
    cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << fSelectedObj[0], fSelectedObj[1], fSelectedObj[2], 1.0);
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
        return;
    }
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ETwc;
    cv::cv2eigen(Twc,ETwc);
    Eigen::Vector4f EX3Dc;
    cv::cv2eigen(x3Dc,EX3Dc);
    Eigen::Vector4f EX3Dw = ETwc * EX3Dc;
//    cv::Mat x3Dw = Twc * x3Dc;
//    output[0] = x3Dw.at<float>(0, 0);
//    output[1] = x3Dw.at<float>(2, 0);
    output[0] = EX3Dw[0];
    output[1] = EX3Dw[1];
}

void MapDrawer::OmplPathPlanning(std::vector<float> &start, std::vector<float> &target,
                                 std::vector<std::vector<float> > &solution,
                                 std::vector<std::vector<int> > &mObstacles)
{
    if (start.size() != 2 || target.size() != 2)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }

    mvBounds[0] = std::numeric_limits<float>::max();
    mvBounds[1] = std::numeric_limits<float>::min();
    mvBounds[2] = std::numeric_limits<float>::max();
    mvBounds[3] = std::numeric_limits<float>::min();
    for (auto p: mCloud->points)
    {
        if (p.x < mvBounds[0])
        {
            mvBounds[0] = p.x;
        }
        if (p.x > mvBounds[1])
        {
            mvBounds[1] = p.x;
        }
        if (p.z < mvBounds[2])
        {
            mvBounds[2] = p.z;
        }
        if (p.z > mvBounds[3])
        {
            mvBounds[3] = p.z;
        }
    }
    const float mfcLeafSize = 0.025;
    const int ncSizeX = static_cast<int> ((mvBounds[1] - mvBounds[0] + 0.2) / mfcLeafSize);
    const int ncSizeY = static_cast<int> ((mvBounds[3] - mvBounds[2] + 0.2) / mfcLeafSize);
    const float fcObstacleWidth = 9;
    mObstacles.clear();
    mObstacles = std::vector<std::vector<int> > (ncSizeY, std::vector<int> (ncSizeX, 0));
    std::vector<int> nStart(2, 0);
    std::vector<int> nTarget(2, 0);
    nStart[0] = static_cast<int> ((start[0] - mvBounds[0]) / mfcLeafSize);
    nStart[1] = static_cast<int> ((start[1] - mvBounds[2]) / mfcLeafSize);
    nTarget[0] = static_cast<int> ((target[0] - mvBounds[0]) / mfcLeafSize);
    nTarget[1] = static_cast<int> ((target[1] - mvBounds[2]) / mfcLeafSize);
    if (nStart[0] < 0 || nStart[0] >= ncSizeX ||
        nStart[1] < 0 || nStart[1] >= ncSizeY ||
        nTarget[0] < 0 || nTarget[0] >= ncSizeX ||
        nTarget[1] < 0 || nTarget[1] >= ncSizeY)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors out of bounds"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }

    for (auto p : mCloud->points) {
        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcLeafSize);
        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcLeafSize);
        for (int i = nTmpX - fcObstacleWidth; i < nTmpX + fcObstacleWidth; i++)
        {
            for (int j = nTmpY - fcObstacleWidth; j < nTmpY + fcObstacleWidth; j++)
            {
                if (i >= 0 && i < ncSizeX && j >= 0 && j < ncSizeY)
                {
                    mObstacles[j][i] = 1;
                }

            }
        }
    }
    GetClosestFreePoint(nStart, mObstacles, 40);
    GetClosestFreePoint(nTarget, mObstacles, 40);

    Plane2DEnvironment env(mObstacles, msPlanner);
    if (env.Plan(nStart, nTarget))
    {
        std::vector<std::vector<int> > vSolution;
        env.GetSolution(vSolution);
        if (vSolution.size() <= 0)
        {
            std::cout << "Cannot find a path ... " << std::endl;
        }
        else
        {
            std::cout << "Find a path ..." << std::endl;
            solution.clear();
            for (std::vector<std::vector<int> >::iterator st = vSolution.begin(); st != vSolution.end(); st++) //in right order
            {
                float yy = (*st)[1] * mfcLeafSize + mvBounds[2];
                float xx = (*st)[0] * mfcLeafSize + mvBounds[0];
                std::vector<float> point = {xx, yy};
                solution.push_back(point);
            }

        }
    }
    else
    {
        std::cout << "Cannot find a path ... " << std::endl;
    }
}
void MapDrawer::AutoBuildMap()
{

}
} //namespace ORB_SLAM
