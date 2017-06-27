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

namespace ORB_SLAM2
{
std::string MapDrawer::msDepthImagesPath = "./Data/DepthImages";
MapDrawer::MapDrawer(std::shared_ptr<Map> pMap, const string &strSettingPath)
    : mpMap(pMap),
      mCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mbCalPointCloud = true;
}

void MapDrawer::DrawPointCloud()
{
    if (mbCalPointCloud) {
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
        int numThreads = 6;
        std::thread threads[numThreads];
        for (int i = 0; i < numThreads; i++) {
            threads[i] = std::thread(&MapDrawer::GeneratePointCloud, this, std::cref(sampledVPKFs), cloud, i,
                                     numThreads);
        }
        for (auto &th : threads) {
            th.join();
        }
        {
            unique_lock<mutex> lock(mMutexMCloud);
            if (cloud->points.size() > 0)
                FilterPointCloud(cloud, mCloud);
//            mCloud = cloud;
        }
        if (mpThreadOctomap) {
            mpThreadOctomap->join();
        }
        mpThreadOctomap.reset(new thread(&MapDrawer::BuildOctomap, this, mCloud));
        mbCalPointCloud = false;

        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "Total Number of Points: " << mCloud->size() << std::endl;
        std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";
    }
//    glPointSize(mPointSize);
//    glBegin(GL_POINTS);
//    glColor3f(1.0, 0.0, 1.0);
//    for (auto p : mCloud->points) {
//        glVertex3f(p.x, p.y, p.z);
//    }
//
//    glEnd();
    std::vector<float>  start = {0, 0};
    std::vector<float> target = {1.5, -5.5};
    PathPlanning(start, target);
}

bool MapDrawer::RRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
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
            if (mObstacles[j][i] == 1)//found obstacle
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

bool MapDrawer::RRTTreesIntersect(std::vector<std::shared_ptr<RRTNode> > &tree,
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
void MapDrawer::PathPlanning(std::vector<float> &start, std::vector<float> &target)
{

    if (start.size() != 2 || target.size() != 2)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    glPointSize(mPointSize * 5);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(start[0], 5, start[1]);
    glVertex3f(target[0], 5, target[1]);
    glEnd();
    std::vector<float> vBounds = {std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::min(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::min()};
    for (auto p: mCloud->points)
    {
        if (p.x < vBounds[0])
        {
            vBounds[0] = p.x;
        }
        if (p.x > vBounds[1])
        {
            vBounds[1] = p.x;
        }
        if (p.z < vBounds[2])
        {
            vBounds[2] = p.z;
        }
        if (p.z > vBounds[3])
        {
            vBounds[3] = p.z;
        }
    }
    const int nMaxIter = 10000;
    const float fcLeafSize = 0.025;
    const int ncSizeX = static_cast<int> ((vBounds[1] - vBounds[0] + 0.2) / fcLeafSize);
    const int ncSizeY = static_cast<int> ((vBounds[3] - vBounds[2] + 0.2) / fcLeafSize);
    const float fcObstacleWidth = 9;//these three should choose properly. ncSizeMax*fcLeafSize is the range of the enviroment.
    //LeafSIze * fcObstacleWidth is the width of the obstacles that extend, equal to half of the robot width
    //37.5cm--->15 is the whole body, 22.5cm-->9 is the test body
    std::vector<std::vector<int> > mObstacles(ncSizeY, std::vector<int> (ncSizeX, 0));
    // origin is (ncSizeMax,ncSizeMax), minimal leaf is fcLeafSize,
    // so the range is [-ncSizeMax * fcLeafSize,ncSizeMax * fcLeafSize]*
    // [-ncSizeMax * fcLeafSize,ncSizeMax * fcLeafSize]
    std::vector<int> nStart(2, 0);
    std::vector<int> nTarget(2, 0);
    nStart[0] = static_cast<int> ((start[0] - vBounds[0]) / fcLeafSize);
    nStart[1] = static_cast<int> ((start[1] - vBounds[2]) / fcLeafSize);
    nTarget[0] = static_cast<int> ((target[0] - vBounds[0]) / fcLeafSize);
    nTarget[1] = static_cast<int> ((target[1] - vBounds[2]) / fcLeafSize);
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
//                   [fcLeafSize, ncSizeMax](float d) -> int{return
//                       (static_cast<int> (d / fcLeafSize) + ncSizeMax);});
//    std::transform(target.begin(), target.end(), nTarget.begin(),
//                   [fcLeafSize, ncSizeMax](float d) -> int{return
//                       (static_cast<int> (d / fcLeafSize) + ncSizeMax);});

    std::vector<std::shared_ptr<RRTNode> > vStartTree;
    std::vector<std::shared_ptr<RRTNode> > vTargetTree;

    vStartTree.push_back(std::make_shared<RRTNode> (nStart));
    vTargetTree.push_back(std::make_shared<RRTNode> (nTarget));

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (auto p : mCloud->points) {
        int nTmpX = static_cast<int>((p.x - vBounds[0]) / fcLeafSize);
        int nTmpY = static_cast<int>((p.z - vBounds[2]) / fcLeafSize);
        if (nTmpX > fcObstacleWidth && nTmpX < ncSizeX - fcObstacleWidth
            && nTmpY > fcObstacleWidth && nTmpY < ncSizeY - fcObstacleWidth)//not at the boundary, draw a circle
        {
            int NumOfParts = 60;//divide a circle into NumOfParts parts
            for (int i = 0; i < NumOfParts; i++) {
                float theta = 2 * 3.14159265 * i / float(NumOfParts);
                //set obstacle,middle is [ncSizeMax,ncSizeMax]
                mObstacles[nTmpY + int(fcObstacleWidth * cos(theta))]
                [nTmpX + int(fcObstacleWidth * sin(theta))] = 1;
                glVertex3f(p.x + fcObstacleWidth * fcLeafSize * cos(theta),
                           5,
                           p.z + fcObstacleWidth * fcLeafSize * sin(theta));
            }
        }
        else {
            //set obstacle,middle is [ncSizeMax,ncSizeMax]
            mObstacles[nTmpY][nTmpX] = 1;
            glVertex3f(p.x, 5, p.z);
        }
    }
    glEnd();

    int nCount = 0;
    int ncStepSize = 6; //pixel

    std::vector<std::shared_ptr<RRTNode> > vSolution;
    srand (time(NULL));
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
        if (!RRTTreeExpand(vStartTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };
        if (!RRTTreeExpand(vTargetTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };

        if (!RRTTreesIntersect(vTargetTree, vStartTree, mObstacles, vSolution, ncStepSize))
        {
            if (!RRTTreesIntersect(vStartTree, vTargetTree, mObstacles, vSolution, ncStepSize))
            {
                continue;
            }
        }


        cout << "Path Found!" << endl;
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


        glLineWidth(mKeyFrameLineWidth * 4);
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        for (std::vector<std::shared_ptr<RRTNode> >::iterator st = vSolution.begin(); (st + 1) != vSolution.end(); st++) //in right order
        {
            int yy = (*st)->mPoint[1];
            int xx = (*st)->mPoint[0];
            glVertex3f(xx * fcLeafSize + vBounds[0], 5.0, yy * fcLeafSize + vBounds[2]);
            int yyy = (*(st + 1))->mPoint[1];
            int xxx = (*(st + 1))->mPoint[0];
            glVertex3f(xxx * fcLeafSize + vBounds[0], 5.0, yyy * fcLeafSize + vBounds[2]);

        }
        glEnd();
        break;
    }
    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.5);
    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vStartTree.begin();st!=vStartTree.end();st++) {

        int yy = (*st)->mPoint[1];
        int xx = (*st)->mPoint[0];
        glVertex3f(xx * fcLeafSize + vBounds[0], 5.0, yy * fcLeafSize + vBounds[2]);
    }
    glEnd();


    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(0.25,0.21,0.16);
    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vTargetTree.begin();st!=vTargetTree.end();st++) {

        int yy = (*st)->mPoint[1];
        int xx = (*st)->mPoint[0];
        glVertex3f(xx * fcLeafSize + vBounds[0], 5.0, yy * fcLeafSize + vBounds[2]);
    }

    glEnd();
    cout << "Iteration Times(Max 10000) =  " << nCount << endl;

}

void MapDrawer::GeneratePointCloud(const vector<std::shared_ptr<KeyFrame> > &vpKFs,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   int begin,
                                   int step)
{
    pcl::PointXYZ point;
    int keyFrameNum = vpKFs.size();
    for (int i = begin; i < keyFrameNum; i += step) {
        std::shared_ptr<KeyFrame> pKF = vpKFs[i];

        std::stringstream ss;
        ss << msDepthImagesPath << "/KeyFrame-" << std::setw(8) << std::setfill('0') << pKF->mnId << ".png";
        std::string depthImageName = ss.str();

        cv::Mat depth = cv::imread(depthImageName.c_str(), cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_32F, mpMap->mDepthMapFactor);

        int imgStep = 3;
        for (int r = 0; r < depth.rows; r += imgStep) {
            const float *itD = depth.ptr<float>(r);
            const float y = mpMap->mLookupY.at<float>(0, r);
            const float *itX = mpMap->mLookupX.ptr<float>();
            for (size_t c = 0; c < (size_t) depth.cols; c += imgStep, itD += imgStep, itX += imgStep) {
                float depthValue = *itD;
                float xx, yy, zz;
                if (depthValue > 0.1 && depthValue < 12.0) {
                    float zc = depthValue;
                    float xc = *itX * depthValue;
                    float yc = y * depthValue;
                    cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << xc, yc, zc, 1);
                    cv::Mat x3Dw = pKF->GetPoseInverse() * x3Dc;
                    xx = x3Dw.at<float>(0, 0);
                    yy = x3Dw.at<float>(1, 0);
                    zz = x3Dw.at<float>(2, 0);
                    if (abs(yy) < 0.55) {
                        point.x = xx;
                        point.y = yy;
                        point.z = zz;
                        unique_lock<mutex> lock(mMutexCloud);
                        cloud->points.push_back(point);
                    }

                }

            }
        }
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
    boost::filesystem::path path{"./Results"};
    boost::filesystem::create_directories(path);
    tree.write("./Results/octomap_office.ot");
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
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (set<std::shared_ptr<MapPoint> >::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if (!(*sit) || (*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

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
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                }
            }

            // Spanning tree
            std::weak_ptr<KeyFrame> pParent = spKF->GetParent();
            if (!pParent.expired()) {
                cv::Mat Owp = pParent.lock()->GetCameraCenter();
                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
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
                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
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
            twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
        }

        M.m[0] = Rwc.at<float>(0, 0);
        M.m[1] = Rwc.at<float>(1, 0);
        M.m[2] = Rwc.at<float>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc.at<float>(0, 1);
        M.m[5] = Rwc.at<float>(1, 1);
        M.m[6] = Rwc.at<float>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc.at<float>(0, 2);
        M.m[9] = Rwc.at<float>(1, 2);
        M.m[10] = Rwc.at<float>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15] = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::clear()
{
    mCloud.reset();
}

void MapDrawer::CloseOctoMapThread()
{
    if (mpThreadOctomap) {
        mpThreadOctomap->join();
    }
    mpThreadOctomap.reset();
}
} //namespace ORB_SLAM
