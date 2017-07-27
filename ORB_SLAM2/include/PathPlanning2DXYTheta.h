#ifndef ORB_SLAM2_PATHPLANNING2DXYTHETA_H
#define ORB_SLAM2_PATHPLANNING2DXYTHETA_H
#include <vector>
#include <pangolin/pangolin.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
namespace ORB_SLAM2
{
class PathPlanning2DXYTheta
{
public:
    PathPlanning2DXYTheta(std::string planner, float pointSize, float lineWidth,
                          float gridSize,
                          std::string dataFolder,
                          float robotHalfLength,
                          float robotHalfWidth,
                          float upperBound = 0.5, float lowerBound = -0.5);

    bool PlanPath(std::vector<float> &start,
                  std::vector<float> &target,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    bool PlanPath(std::vector<float> &start,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    bool UnvisitedAreasToGo(std::vector<float> &currentPos,
                            std::vector<float> &target,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    bool UnvisitedAreasToGo(std::vector<float> &target,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    void ShowPlannedPath();

    void reset(bool cleanOccupancyMap = true);

    std::vector<float> GetTargetW();

    std::shared_ptr<std::vector<std::vector<float> > > mpSolution;

    void UpdatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);


private:


    void AddPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    bool OmplPathPlanning();

    bool SBPLPathPlanning();

    bool GetClosestFreePoint(std::vector<int> &output,
                             int searchWidth,
                             std::vector<std::pair<float, std::vector<int> > > &candidateOutput);

    void Get2DBounds();
    void CalGridSize();

    void GenerateOccupancyMap();
    void WorldToGrid(std::vector<float> &input, std::vector<int> &output);
    void GridToWorld(std::vector<int> &input, std::vector<float> &output);

    void Convert2DVectorToMat(std::vector<std::vector<int>> &input,
                              cv::Mat &output,
                              bool reverseRow = true);

    float mfStartYaw;
    std::vector<int> mvStartG; //coordinate in grid
    std::vector<int> mvTargetG;
    std::vector<float> mvStartW; //coordinate in world
    std::vector<float> mvTargetW;
    std::vector<float> mvTmpStartW; //coordinate in world
    std::vector<float> mvTmpTargetW;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloud;
    std::vector<std::vector<int> > mObstacles;
    std::vector<std::vector<int> > mObstaclesInflated;
    std::vector<std::vector<int> > mObstaclesSeenNum;
    const float mfcGridSize;
    std::vector<float> mvBounds;
    std::string msPlanner;
    float mLineWidth;
    float mPointSize;
    int mnSizeX;
    int mnSizeY;
    bool mbSBPLPathPlan;
    std::vector<std::pair<float, std::vector<int> > > mvCandidatesValidStart;
    std::vector<std::pair<float, std::vector<int> > > mvCandidatesValidTarget;
    float mfUpperBound;
    float mfLowerBound;
    float mfRobotHalfLength;
    float mfRobotHalfWidth;
    std::string msDataFolder;
    std::string msContourFolder;
    int mnCostLethal;
    int mnCostInscribed;
    int mnCostPossiblyCircumscribedThresh;
};
}

#endif
