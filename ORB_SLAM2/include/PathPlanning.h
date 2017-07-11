#ifndef ORB_SLAM2_PATHPLANNING_H
#define ORB_SLAM2_PATHPLANNING_H
#include <pangolin/pangolin.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>
#include <string>
namespace ORB_SLAM2
{
struct RRTNode
{
    RRTNode(const std::vector<int> &point)
        : mPoint(point)
    {};
    RRTNode(const std::vector<int> &point, std::shared_ptr<RRTNode> parent)
        :
        mPoint(point), mParent(parent)
    {};
    void addParent(std::shared_ptr<RRTNode> parent)
    {
        mParent = parent;
    }
    std::vector<int> mPoint;
    std::shared_ptr<RRTNode> mParent;
};


class PathPlanning2D
{
public:
    PathPlanning2D(std::string planner, float pointSize, float lineWidth,
                   float obstacleWidth, float leafSize);
    bool PlanPath(std::vector<float> &start,
                  std::vector<float> &target,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    bool PlanPath(std::vector<float> &start,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void ShowPlannedPath();

    void reset();

    std::vector<float> GetTargetW();

    std::shared_ptr<std::vector<std::vector<float> > > mpSolution;
private:
    void UpdatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    bool OmplPathPlanning();

    bool SBPLPathPlanning();

//    Deprecated
    void SimplePathPlanning();

    bool GetClosestFreePoint(std::vector<int> &output,
                             int searchWidth,
                             std::vector<std::pair<float, std::vector<int> > > &candidateOutput);

    bool SimpleRRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
                             std::vector<std::vector<int> > &mObstacles,
                             int nMiddleX, int nMiddleY, int ncStepSize);

    bool SimpleRRTTreesIntersect(std::vector<std::shared_ptr<RRTNode> > &tree,
                                 std::vector<std::shared_ptr<RRTNode> > &treePop,
                                 std::vector<std::vector<int> > &mObstacles,
                                 std::vector<std::shared_ptr<RRTNode> > &vSolution,
                                 int ncStepSize);

    void Get2DBounds();
    void CalGridSize();

    void GenerateOccupancyMap();
    void WorldToGrid(std::vector<float> &input, std::vector<int> &output);
    void GridToWorld(std::vector<int> &input, std::vector<float> &output);

    std::vector<int> mvStartG; //coordinate in grid
    std::vector<int> mvTargetG;
    std::vector<float> mvStartW; //coordinate in world
    std::vector<float> mvTargetW;
    std::vector<float> mvTmpStartW; //coordinate in world
    std::vector<float> mvTmpTargetW;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    std::vector<std::vector<int> > mObstacles;
    std::vector<std::vector<float> > mObstaclesHeight;
    const float mfcLeafSize;
    std::vector<float> mvBounds;
    std::string msPlanner;
    float mLineWidth;
    float mPointSize;
    int mnSizeX;
    int mnSizeY;
    float mfObstacleWidth;
    bool mbSBPLPathPlan;
    std::vector<std::pair<float, std::vector<int> > > mvCandidatesValidStart;
    std::vector<std::pair<float, std::vector<int> > > mvCandidatesValidTarget;

};

}

#endif
