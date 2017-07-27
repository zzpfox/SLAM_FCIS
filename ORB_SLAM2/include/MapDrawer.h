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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include <thread>
#include <memory>
#include "PathPlanning2D.h"
#include "PathPlanning2DXYTheta.h"

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(std::shared_ptr<Map> pMap, const string &strSettingPath);

    std::shared_ptr<Map> mpMap;

    bool mbCalPointCloud;
    bool mbFindObjCalPoints;
    std::unique_ptr<std::thread> mpThreadOctomap;
    static std::string msPointCloudPath;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloud;
    std::shared_ptr<PathPlanning2D> mPathPlanning;
    std::shared_ptr<PathPlanning2DXYTheta> mPathPlanningXYTheta;


    void GeneratePointCloud(const vector<std::shared_ptr<KeyFrame> > &vpKFs,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            int begin, int step);

    void FindObjects();

    bool RandomlyGetObjPose(std::string object,
                            std::vector<float> &output,
                            std::unordered_map<std::string,
                                               std::unordered_map<long unsigned int, ObjectPos> > &ObjectMap);

    void CalPointCloud(bool saveOctoMap = false);

    void GetPath(std::vector<std::vector<float> > &path);

    void UpdateSolution(std::shared_ptr<std::vector<std::vector<float> > > &pSolution);

    void CleanSolution();

    void OmplPathPlanning(std::vector<float> &start,
                          std::vector<float> &target,
                          std::vector<std::vector<float> > &solution,
                          std::vector<std::vector<int> > &mObstacles);

    void SimplePathPlanning(std::vector<float> &start,
                            std::vector<float> &target,
                            std::vector<std::vector<float> > &solution,
                            std::vector<std::vector<int> > &mObstacles);

    void FilterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

    void clear();

    void CloseOctoMapThread();

    void DrawPointCloud();

    void DrawMapPoints();

    void GetClosestFreePoint(std::vector<int> &output,
                             std::vector<std::vector<int> > &obstacles,
                             int searchWidth);

    bool SimpleRRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
                             std::vector<std::vector<int> > &mObstacles,
                             int nMiddleX, int nMiddleY, int ncStepSize);

    bool SimpleRRTTreesIntersect(std::vector<std::shared_ptr<RRTNode> > &tree,
                                 std::vector<std::shared_ptr<RRTNode> > &treePop,
                                 std::vector<std::vector<int> > &mObstacles,
                                 std::vector<std::shared_ptr<RRTNode> > &vSolution,
                                 int ncStepSize);

    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(const cv::Mat &Tcw);

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void BuildOctomap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void SaveDenseMapToPCD();

    void SaveDenseMapToSTL();

private:
    bool mbXYThetaPlan;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    float mfHeightUpperBound;
    float mfHeightLowerBound;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
    std::mutex mMutexCloud;
    std::mutex mMutexMCloud;
    std::mutex mMutexPath;

    std::shared_ptr<std::vector<std::vector<float> > > mpSolution;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
