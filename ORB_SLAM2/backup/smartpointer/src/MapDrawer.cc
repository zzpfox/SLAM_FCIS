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

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 1.0);
    for (auto p : mCloud->points) {
        glVertex3f(p.x, p.y, p.z);
    }

    glEnd();
//    PathPlanning();
}

void MapDrawer::PathPlanning()
{
    const int SIZEMAX = 500;
    const float LeafSize = 0.025;
    const float ObstacleWidth = 9;//these three should choose properly. SIZEMAX*LEAFSIze is the range of the enviroment.
    //LeafSIze * ObStacleWidth is the width of the obstacles that extend, equal to half of the robot width
    //37.5cm--->15 is the whole body, 22.5cm-->9 is the test body


    int Obstacles[2 * SIZEMAX][2 * SIZEMAX
    ];//origin is (400,400), minimal leaf is 0.05m, so the range is [-20m,20m]*[-20m,20m]

    for (int i = 0; i < 2 * SIZEMAX; i++)
        for (int j = 0; j < 2 * SIZEMAX; j++)
            Obstacles[i][j] = 0;// init



    int Start[2] = {500, 500};
    int Target[2] = {580, 280};
    vector<int> StartTree;//all the points, tree
    vector<int> StartFather;//father node, only one , to make a line in the final

    StartTree.push_back(Start[0] * 2 * SIZEMAX + Start[1]);
    StartFather.push_back(0);
    int StartTreeCount = 0;

    vector<int> TargetTree;
    vector<int> TargetFather;
    TargetTree.push_back(Target[0] * 2 * SIZEMAX + Target[1]);
    TargetFather.push_back(0);
    int TargetTreeCount = 0;

    int XMAX = 0, XMIN = 2 * SIZEMAX, YMAX = 0, YMIN = 2 * SIZEMAX;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (auto p:mCloud->points) {


        if (int(p.x / LeafSize) + SIZEMAX > ObstacleWidth
            && int(p.z / LeafSize) + SIZEMAX > ObstacleWidth)//not at the boundary, draw a circle
        {
            int NumOfParts = 60;//divide a circle into NumOfParts parts
            for (int i = 0; i < NumOfParts; i++) {
                float theta = 2 * 3.14159265 * float(i) / float(NumOfParts);
                Obstacles[int(p.x / LeafSize) + SIZEMAX + int(ObstacleWidth * cos(theta))][int(p.z / LeafSize) + SIZEMAX
                    +
                        int(ObstacleWidth * sin(theta))] = 1;//set obstacle,middle is 400,400
                glVertex3f(float(int(p.x / LeafSize) + int(ObstacleWidth * cos(theta))) * LeafSize,
                           5,
                           float(int(p.z / LeafSize) + int(ObstacleWidth * sin(theta))) * LeafSize);
            }
        }
        else {
            Obstacles[int(p.x / LeafSize) + SIZEMAX][int(p.z / LeafSize) + SIZEMAX] = 1;//set obstacle,middle is 400,400
            glVertex3f(float(int(p.x / LeafSize)) * LeafSize, 5, float(int(p.z / LeafSize)) * LeafSize);
        }

        if (int(p.x / LeafSize) + SIZEMAX < XMIN)
            XMIN = int(p.x / LeafSize) + SIZEMAX;
        else if (int(p.x / LeafSize) + SIZEMAX > XMAX)
            XMAX = int(p.x / LeafSize) + SIZEMAX;

        if (int(p.z / LeafSize) + SIZEMAX < YMIN)
            YMIN = int(p.z / LeafSize) + SIZEMAX;
        else if (int(p.z / LeafSize) + SIZEMAX > YMAX)
            YMAX = int(p.z / LeafSize) + SIZEMAX;

    }
    glEnd();


    int count = 0;
    int DisMin;
    int XRANGE = XMAX - XMIN;
    int YRANGE = YMAX - YMIN;
    int STEPSIZE = 6; //pixel

    vector<int> Solution;


    while (count < 10000)  //max iteration
    {
        count++;
        if (XRANGE < 1 || YRANGE < 1) {
            cout << "Range error!" << endl;
            break;
        }

        int MiddleX = XMIN + int(XRANGE * drand48());
        int MiddleY = YMIN + int(YRANGE * drand48());//generate a middle point
        while (Obstacles[MiddleX][MiddleY] == 1)// should not be an obstacle
        {
            MiddleX = XMIN + int(XRANGE * drand48());
            MiddleY = YMIN + int(YRANGE * drand48());//generate a middle point
        }

        //check StartTree
        DisMin = 2 * 2 * SIZEMAX;
        int ConnectStart = 1;
        int StartX = SIZEMAX;
        int StartY = SIZEMAX;
        int MinStartX = StartX;
        int MinStartY = StartY;
        int MinStartPos = 0;
        int MinStartCount = 0;
        for (vector<int>::iterator sit = StartTree.begin(), send = StartTree.end(); sit != send; sit++) {

            StartY = (*sit) % (2 * SIZEMAX);
            StartX = (*sit - StartY) / (2 * SIZEMAX);
            if (abs(MiddleX - StartX) + abs(MiddleY - StartY) < DisMin) {
                DisMin = abs(MiddleX - StartX) + abs(MiddleY - StartY);
                MinStartX = StartX;
                MinStartY = StartY;
                MinStartPos = MinStartCount;
            }
            MinStartCount++;
        }

        if (DisMin < 4)
            continue;//means too close

        int StepX = MinStartX > MiddleX ? (-1) : 1;
        int StepY = MinStartY > MiddleY ? (-1) : 1;// different step along different direction
        int STEPSIZEX, STEPSIZEY;

        //use sin,cos to determine direction
        float Radius = sqrt(
            float((MinStartX - MiddleX) * (MinStartX - MiddleX) + (MinStartY - MiddleY) * (MinStartY - MiddleY)));
        STEPSIZEX = int(float(STEPSIZE * abs(MinStartX - MiddleX)) / Radius) + 1;// sin
        STEPSIZEY = int(float(STEPSIZE * abs(MinStartY - MiddleY)) / Radius) + 1;// cos



        for (int i = MinStartX; abs(i - MinStartX) < STEPSIZEX; i += StepX) {
            int OUT = 0;
            for (int j = MinStartY; abs(j - MinStartY) < STEPSIZEY; j += StepY) {

                if (Obstacles[i][j] == 1)//found obstacle
                {

                    ConnectStart = 0;// not directly connect

                    OUT = 1;

                    break;
                }
            }
            if (OUT == 1)
                break;
        }

        int StartSaveX = MinStartX, StartSaveY = MinStartY;//if Connect=0, then use Minstart as StartSave

        if (ConnectStart == 1) {// add
            StepX = MinStartX > MiddleX ? (-1) : 1;
            StepY = MinStartY > MiddleY ? (-1) : 1;

            StartSaveX = StepX * STEPSIZEX + MinStartX;
            StartSaveY = StepY * STEPSIZEY + MinStartY;
            StartTree.push_back(
                ((StartSaveX)) * 2 * SIZEMAX + (StartSaveY));// add the halfway point
            StartFather.push_back(MinStartPos);

        }

        //check TargetTree
        DisMin = 2 * 2 * SIZEMAX;
        int ConnectTarget = 1;
        int TargetX = SIZEMAX;
        int TargetY = SIZEMAX;
        int MinTargetX = TargetX;
        int MinTargetY = TargetY;
        int MinTargetPos = 0;
        int MinTargetCount = 0;
        for (vector<int>::iterator sit = TargetTree.begin(), send = TargetTree.end(); sit != send; sit++) {
            TargetY = (*sit) % (2 * SIZEMAX);
            TargetX = (*sit - TargetY) / (2 * SIZEMAX);
            if (abs(MiddleX - TargetX) + abs(MiddleY - TargetY) < DisMin) {
                DisMin = abs(MiddleX - TargetX) + abs(MiddleY - TargetY);
                MinTargetX = TargetX;
                MinTargetY = TargetY;
                MinTargetPos = MinTargetCount;
            }
            MinTargetCount++;
        }

        if (DisMin < 4)
            continue;//means too close

        StepX = MinTargetX > MiddleX ? (-1) : 1;
        StepY = MinTargetY > MiddleY ? (-1) : 1;

        //use sin,cos to determine direction
        Radius = sqrt(float((MinTargetX - MiddleX) * (MinTargetX - MiddleX) +
            (MinTargetY - MiddleY) * (MinTargetY - MiddleY)));
        STEPSIZEX = int(float(STEPSIZE * abs(MinTargetX - MiddleX)) / Radius) + 1;// sin
        STEPSIZEY = int(float(STEPSIZE * abs(MinTargetY - MiddleY)) / Radius) + 1;// cos


        for (int i = MinTargetX; abs(i - MinTargetX) < STEPSIZEX; i += StepX) {
            int OUT = 0;
            for (int j = MinTargetY; abs(j - MinTargetY) < STEPSIZEY; j += StepY) {
                if (Obstacles[i][j] == 1)//found obstacle
                {

                    ConnectTarget = 0;// not directly connect,as STEPSIZE is small, do not add to tree

                    OUT = 1;


                    break;
                }
            }
            if (OUT == 1)
                break;
        }


        int TargetSaveX = MinTargetX, TargetSaveY = MinTargetY;

        if (ConnectTarget == 1) {// have a long line
            StepX = MinTargetX > MiddleX ? (-1) : 1;
            StepY = MinTargetY > MiddleY ? (-1) : 1;

            TargetSaveX = StepX * STEPSIZEX + MinTargetX;
            TargetSaveY = StepY * STEPSIZEY + MinTargetY;
            TargetTree.push_back(
                ((TargetSaveX)) * 2 * SIZEMAX + (TargetSaveY));// add the halfway point
            TargetFather.push_back(MinTargetPos);

        }


        int CONNECT = 0;


        //check whether new node near the other tree, first check near target tree
        MinTargetCount = 0;
        for (vector<int>::iterator sit = TargetTree.begin(), send = TargetTree.end(); sit != send; sit++) {
            TargetY = (*sit) % (2 * SIZEMAX);
            TargetX = (*sit - TargetY) / (2 * SIZEMAX);
            if (abs(StartSaveX - TargetX) + abs(StartSaveY - TargetY) <= 2 * STEPSIZE) {
                int OUT = 0;
                for (int i = min(StartSaveX, TargetX); i <= max(StartSaveX, TargetX); i++) {

                    for (int j = min(StartSaveY, TargetY); j <= max(StartSaveY, TargetY); j++) {
                        if (Obstacles[i][j] == 1) {
                            OUT = 1;
                            CONNECT = 0;
                            break;
                        }
                    }
                    if (OUT == 1)
                        break;
                }
                if (OUT == 0)//connect
                {
                    CONNECT = 1;//connect to start tree
                    vector<int> temp;
                    temp.push_back((StartSaveX) * 2 * SIZEMAX + (StartSaveY));// put the node from start tree
                    int CurrentPos = MinStartPos;//first add start tree, but reverse order

                    while (StartFather[CurrentPos] != 0) {
                        temp.push_back(StartTree[CurrentPos]);
                        CurrentPos = StartFather[CurrentPos];

                    }

                    temp.push_back(StartTree[CurrentPos]);
                    temp.push_back(StartTree[0]);

                    for (vector<int>::iterator tpn = temp.end() - 1, tps = temp.begin(); tpn >= tps;
                         tpn--) //in right order,vector.end is the one past the last
                    {
                        Solution.push_back(*tpn);
                    }

                    CurrentPos = MinTargetCount;//then add target tree, right order
                    Solution.push_back(TargetTree[CurrentPos]);
                    while (TargetFather[CurrentPos] != 0) {
                        CurrentPos = TargetFather[CurrentPos];

                        Solution.push_back(TargetTree[CurrentPos]);
                    }
                    CurrentPos = TargetFather[CurrentPos];
                    Solution.push_back(TargetTree[CurrentPos]);
                    break;
                }
            }
            MinTargetCount++;

        }

        if (CONNECT == 0) ////check start tree
        {
            MinStartCount = 0;
            for (vector<int>::iterator sit = StartTree.begin(), send = StartTree.end(); sit != send; sit++) {
                StartY = (*sit) % (2 * SIZEMAX);
                StartX = (*sit - StartY) / (2 * SIZEMAX);
                if (abs(TargetSaveX - StartX) + abs(TargetSaveY - StartY) <= 2 * STEPSIZE) {
                    int OUT = 0;
                    for (int i = min(TargetSaveX, StartX); i <= max(TargetSaveX, StartX); i++) {

                        for (int j = min(TargetSaveY, StartY); j <= max(TargetSaveY, StartY); j++) {
                            if (Obstacles[i][j] == 1) {
                                OUT = 1;
                                CONNECT = 0;
                                break;
                            }
                        }
                        if (OUT == 1)
                            break;
                    }
                    if (OUT == 0)//connect
                    {
                        CONNECT = 1;//connect to start tree
                        vector<int> temp;
                        temp.push_back((TargetSaveX) * 2 * SIZEMAX + (TargetSaveY));// put the node from target tree
                        int CurrentPos = MinTargetPos;//first add target tree, but reverse order

                        while (TargetFather[CurrentPos] != 0) {
                            temp.push_back(TargetTree[CurrentPos]);
                            CurrentPos = TargetFather[CurrentPos];

                        }

                        temp.push_back(TargetTree[CurrentPos]);
                        temp.push_back(TargetTree[0]);

                        for (vector<int>::iterator tpn = temp.end() - 1, tps = temp.begin(); tpn != tps;
                             tpn--) //in right order, vector.end is the one past end
                        {
                            Solution.push_back(*tpn);
                        }

                        CurrentPos = MinStartCount;//then add start tree, right order
                        Solution.push_back(StartTree[CurrentPos]);
                        while (StartFather[CurrentPos] != 0) {
                            CurrentPos = StartFather[CurrentPos];

                            Solution.push_back(StartTree[CurrentPos]);
                        }
                        CurrentPos = StartFather[CurrentPos];
                        Solution.push_back(StartTree[CurrentPos]);
                        break;
                    }
                }
                MinStartCount++;

            }
        }


        if (CONNECT == 1) // FOUND solution
        {
            cout << "Path Found!" << endl;
            //the order in Solution is the path, now smooth

            int span = 2; //follow rrt
            while (span < Solution.size()) {
                bool changed = false;
                for (int i = 0; i + span < Solution.size(); i++) {
                    bool CanErase = true;// true to erase
                    int YY = (Solution[i]) % (2 * SIZEMAX);

                    int XX = (Solution[i] - YY) / (2 * SIZEMAX);

                    int YYY = (Solution[i + span]) % (2 * SIZEMAX);
                    int XXX = (Solution[i + span] - YY) / (2 * SIZEMAX);
                    for (int ii = min(XX, XXX); ii <= max(XX, XXX); ii++) {
                        for (int jj = min(YY, YYY); jj <= max(YY, YYY); jj++) {
                            if (Obstacles[ii][jj] == 1) {
                                CanErase = false;
                                break;
                            }
                        }
                        if (!CanErase)
                            break;
                    }


                    if (CanErase) {
                        for (int x = 1; x < span; x++) {
                            Solution.erase(Solution.begin() + i + 1);
                        }
                        changed = true;
                    }
                }

                if (!changed) span++;
            }


            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0, 1.0, 0.0);
            glBegin(GL_LINES);
            for (vector<int>::iterator st = Solution.begin(), end = Solution.end(); (st + 1) != end;
                 st++) //in right order
            {
                int YY = (*st) % (2 * SIZEMAX);
                int XX = (*st - YY) / (2 * SIZEMAX);
                XX = XX - SIZEMAX;
                YY = YY - SIZEMAX;//center at SIZEMAX,SIZEMAX
                glVertex3f(float(XX * LeafSize), 5.0, float(YY * LeafSize));

                YY = (*(st + 1)) % (2 * SIZEMAX);
                XX = (*(st + 1) - YY) / (2 * SIZEMAX);
                XX = XX - SIZEMAX;
                YY = YY - SIZEMAX;//center at SIZEMAX,SIZEMAX
                glVertex3f(float(XX * LeafSize), 5.0, float(YY * LeafSize));

            }
            glEnd();

            break;

        }

    }

    cout << "Iteration Times(Max 10000) =  " << count << endl;

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
