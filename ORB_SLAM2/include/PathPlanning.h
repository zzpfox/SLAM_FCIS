#ifndef ORB_SLAM2_PATHPLANNING_H
#define ORB_SLAM2_PATHPLANNING_H
#include <pangolin/pangolin.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
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
    PathPlanning2D(std::string planner, float pointSize, float lineWidth);
    void PlanPath(std::vector<float> &start,
                  std::vector<float> &target,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void ShowPlannedPath();

private:
    void UpdatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void OmplPathPlanning();

//    Deprecated
    void SimplePathPlanning();

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

    void Get2DBounds();
    void CalGridSize();

    void GenerateOccupancyMap();
    void WorldToGrid(std::vector<float> &input, std::vector<int> &output);
    void GridToWorld(std::vector<int> &input, std::vector<float> &output);

    std::vector<float> mvStart; //world coordinate projected onto the plane
    std::vector<float> mvTarget;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    std::vector<std::vector<float> > mSolution;
    std::vector<std::vector<int> > mObstacles;
    const float mfcLeafSize;
    std::vector<float> mvBounds;
    std::string msPlanner;
    float mLineWidth;
    float mPointSize;
    int mnSizeX;
    int mnSizeY;
    float mfObstacleWidth;

};

}

#endif
