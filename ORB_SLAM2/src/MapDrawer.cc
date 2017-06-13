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
namespace ORB_SLAM2 {


    MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap), mCloud(new pcl::PointCloud<pcl::PointXYZ>){
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
        mbCalPointCloud = true;
    }

    void MapDrawer::DrawPointCloud() {

        if (mbCalPointCloud) {
            std::chrono::time_point<std::chrono::system_clock> start;
            std::chrono::time_point<std::chrono::system_clock> end;
            start = std::chrono::system_clock::now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

            vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
            int sampleFrequency = 4;
            vector<KeyFrame *> sampledVPKFs;
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
            mCloud = cloud;

            mpThreadOctomap = new thread(&MapDrawer::BuildOctomap, this, cloud);
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
    }

    void MapDrawer::GeneratePointCloud(const vector<KeyFrame *> &vpKFs, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       int begin, int step) {
        pcl::PointXYZ point;
        int keyFrameNum = vpKFs.size();
        for (int i = begin; i < keyFrameNum; i += step) {
            KeyFrame *pKF = vpKFs[i];
            cv::Mat depth = pKF->mImDepth;
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

    void MapDrawer::BuildOctomap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (cloud->points.size() > 0)
            FilterPointCloud(cloud, output);
        // Generate octomap
        octomap::OcTree tree(0.05);
        for (auto p:output->points) {
            tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        }
        tree.updateInnerOccupancy();
        boost::filesystem::path path{"./results"};
        boost::filesystem::create_directories(path);
        tree.write("./results/octomap_office.ot");
    }

    void MapDrawer::FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRandomFilter(new pcl::PointCloud <pcl::PointXYZ>);
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

    void MapDrawer::DrawMapPoints() {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

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

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

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
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
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


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
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
        } else
            M.SetIdentity();
    }

} //namespace ORB_SLAM
