#ifndef ORB_SLAM2_AUTOBUILDMAP_H
#define ORB_SLAM2_AUTOBUILDMAP_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
#include "PathPlanning2D.h"
#include "PathPlanning2DXYTheta.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include <mutex>

namespace ORB_SLAM2
{
class System;
class AutoBuildMap
{
public:
    AutoBuildMap(System *pSystem, std::shared_ptr<Tracking> pTracking,
                 std::shared_ptr<MapDrawer> pMapDrawer,
                 cv::FileStorage &fSettings);
    void UpdateSolution(std::shared_ptr<std::vector<std::vector<float> > > &pSolution);
    bool NeedReplanPath();
    bool PlanPath();
    bool PlanPath(std::vector<float> &target);
    void Show2DMap();
    void SaveMap();
    void CalGridSize();
    void Run();

    bool CloseToTarget(std::vector<float> &target, float disThresh = 1.4);
    bool GoToStartPosition();
    void EnsureAllAreasChecked();
    void Get2DBounds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud0,
                     std::vector<float> &vStart);
    void WorldToGrid(std::vector<float> &input, std::vector<int> &output);
    void GridToWorld(std::vector<int> &input, std::vector<float> &output);
    void ShortenSolution();
    void SetCurrentFrame(Frame &m);
    void GetPath(std::vector<std::vector<float> > &path);
    bool IsStateValid(int x, int y, double yaw);
    bool RobotCollideObstacle(std::vector<int> &bounds,
                              std::vector<std::vector<std::vector<int> > > &triangles);
    void SetupRobot();
    void GetTriangles(Eigen::Vector2i &center,
                      double angle,
                      std::vector<std::vector<std::vector<int> > > &triangles,
                      Eigen::Matrix<double, 2, 4> &robotCorners);
    void GetRobotBounds(Eigen::Matrix<double, 2, 4> &robotCorners,
                        std::vector<int> &bounds);
    bool PointInTriangle(Eigen::Vector2i &point, std::vector<std::vector<int> > triangle);
    bool mbAutoDone;

private:
    System *mpSystem;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloud;
    std::vector<std::vector<int> > mObstacles;
    std::shared_ptr<Tracking> mpTracker;
    std::shared_ptr<MapDrawer> mpMapDrawer;
    float mfcGridSize;
    std::vector<float> mvBounds;
    int mnSizeX;
    int mnSizeY;
    float mfObstacleWidth;
    cv::Mat mCurrentFrameImDepth;
    cv::Mat mCameraCenter;
    cv::Mat mTwc;
    std::shared_ptr<std::vector<std::vector<float> > > mpSolution;
    std::shared_ptr<PathPlanning2D> mptPathPlanning;
    std::shared_ptr<PathPlanning2DXYTheta> mptPathPlanningXYTheta;
    float mfHeightUpperBound;
    float mfHeightLowerBound;
    float mfRobotHalfLength;
    float mfRobotHalfWidth;
    float mLineWidth;
    float mPointSize;
    std::mutex mMutexObstacle;
    std::mutex mMutexFrame;
    std::mutex mMutexPath;
    bool mbXYThetaPlan;
    Eigen::Matrix<int, 2, 4> mRobotCornersUnMoved;




};
}
#endif