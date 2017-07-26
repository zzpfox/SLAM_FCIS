#ifndef ORB_SLAM2_OMPLPATHPLANNING_H
#define ORB_SLAM2_OMPLPATHPLANNING_H
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ORB_SLAM2
{

void SetupPlanner(og::SimpleSetupPtr ss, std::string planner);

class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const std::vector<std::vector<int> > &obstacles, std::string &planner);
    bool Plan(std::vector<int> nStart, std::vector<int> nGoal);
    void GetSolution(std::vector<std::vector<int> > &vSolution);

private:

    bool IsStateValid(const ob::State *state) const;

    og::SimpleSetupPtr mSS;
    int mMaxX;
    int mMaxY;
    std::vector<std::vector<int> > mObstacles;
};


class Plane2DXYThetaEnvironment
{
public:
    Plane2DXYThetaEnvironment(const std::vector<std::vector<int> > &obstacles,
                              std::string &planner,
                              int halfLength, int halfWidth);
    bool Plan(std::vector<int> nStart, std::vector<int> nGoal, float startYaw = 0.0, float goalYaw = 0.0);
    void GetSolution(std::vector<std::vector<float> > &vSolution);

private:
    void GetTriangles(Eigen::Vector2i &center,
                      double angle,
                      std::vector<std::vector<std::vector<int> > > &triangles,
                      Eigen::Matrix<double, 2, 4> &robotCorners) const;
    void GetRobotBounds(Eigen::Matrix<double, 2, 4> &robotCorners, std::vector<int> &bounds) const;
    bool RobotCollideObstacle(std::vector<int> &bounds,
                              std::vector<std::vector<std::vector<int> > > &triangles) const;
    bool PointInTriangle(Eigen::Vector2i &point, std::vector<std::vector<int> > triangle) const;
    void SetupRobot();
    bool IsStateValid(const ob::State *state) const;

    og::SimpleSetupPtr mSS;
    int mMaxX;
    int mMaxY;

    int mdRobotHalfWidth;
    int mdRobotHalfLength;
    std::vector<std::vector<int> > mObstacles;
    Eigen::Matrix<int, 2, 4> mRobotCornersUnMoved;
};

}

#endif
