#include "OmplPathPlanning.h"
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ORB_SLAM2
{

void SetupPlanner(og::SimpleSetupPtr ss, std::string planner)
{
    ob::SpaceInformationPtr si = ss->getSpaceInformation();
    if (planner == "" || planner == "RRTConnect")
    {
        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "RRTstar")
    {
        std::shared_ptr<og::RRTstar> newPlanner = std::make_shared<og::RRTstar>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "RRTsharp")
    {
        std::shared_ptr<og::RRTsharp> newPlanner = std::make_shared<og::RRTsharp>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "TRRT")
    {
        std::shared_ptr<og::TRRT> newPlanner = std::make_shared<og::TRRT>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "pRRT")
    {
        std::shared_ptr<og::pRRT> newPlanner = std::make_shared<og::pRRT>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "SBL")
    {
        std::shared_ptr<og::SBL> newPlanner = std::make_shared<og::SBL>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "pSBL")
    {
        std::shared_ptr<og::pSBL> newPlanner = std::make_shared<og::pSBL>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "FMT")
    {
        std::shared_ptr<og::FMT> newPlanner = std::make_shared<og::FMT>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "BFMT")
    {
        std::shared_ptr<og::BFMT> newPlanner = std::make_shared<og::BFMT>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "SPARS")
    {
        std::shared_ptr<og::SPARS> newPlanner = std::make_shared<og::SPARS>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "SPARStwo")
    {
        std::shared_ptr<og::SPARStwo> newPlanner = std::make_shared<og::SPARStwo>(si);
        ss->setPlanner(newPlanner);
    }
    else if (planner == "PRM")
    {
        std::shared_ptr<og::PRM> newPlanner = std::make_shared<og::PRM>(si);
        ss->setPlanner(newPlanner);
    }
    else
    {
        std::cout << "\x1B[31m" << "Unrecognized planner name: " << planner
                  << "\x1B[0m" << std::endl;
        std::cout << "Using RRTConnect instead ..." << std::endl;
        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
        ss->setPlanner(newPlanner);
    }
}


Plane2DEnvironment::Plane2DEnvironment(const std::vector<std::vector<int> > &obstacles, std::string &planner)
    : mObstacles(obstacles)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>());
    space->addDimension(0.0, mObstacles[0].size());
    space->addDimension(0.0, mObstacles.size());

    mMaxX = mObstacles[0].size() - 1;
    mMaxY = mObstacles.size() - 1;
    mSS = std::make_shared<og::SimpleSetup>(space);

    // set state validity checking for this space
    mSS->setStateValidityChecker([this](const ob::State *state)
                                 { return IsStateValid(state); });
    space->setup();
    mSS->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
    //      mSS->setPlanner(std::make_shared<og::RRTConnect>(mSS->getSpaceInformation()));
//    ob::SpaceInformationPtr si = mSS->getSpaceInformation();
    SetupPlanner(mSS, planner);
//    if (planner == "" || planner == "RRTConnect")
//    {
//        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "RRTstar")
//    {
//        std::shared_ptr<og::RRTstar> newPlanner = std::make_shared<og::RRTstar>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "RRTsharp")
//    {
//        std::shared_ptr<og::RRTsharp> newPlanner = std::make_shared<og::RRTsharp>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "TRRT")
//    {
//        std::shared_ptr<og::TRRT> newPlanner = std::make_shared<og::TRRT>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "pRRT")
//    {
//        std::shared_ptr<og::pRRT> newPlanner = std::make_shared<og::pRRT>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "SBL")
//    {
//        std::shared_ptr<og::SBL> newPlanner = std::make_shared<og::SBL>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "pSBL")
//    {
//        std::shared_ptr<og::pSBL> newPlanner = std::make_shared<og::pSBL>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "FMT")
//    {
//        std::shared_ptr<og::FMT> newPlanner = std::make_shared<og::FMT>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "BFMT")
//    {
//        std::shared_ptr<og::BFMT> newPlanner = std::make_shared<og::BFMT>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "SPARS")
//    {
//        std::shared_ptr<og::SPARS> newPlanner = std::make_shared<og::SPARS>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "SPARStwo")
//    {
//        std::shared_ptr<og::SPARStwo> newPlanner = std::make_shared<og::SPARStwo>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else if (planner == "PRM")
//    {
//        std::shared_ptr<og::PRM> newPlanner = std::make_shared<og::PRM>(si);
//        mSS->setPlanner(newPlanner);
//    }
//    else
//    {
//        std::cout << "\x1B[31m" << "Unrecognized planner name: " << planner
//                  << "\x1B[0m" << std::endl;
//        std::cout << "Using RRTConnect instead ..." << std::endl;
//        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
//        mSS->setPlanner(newPlanner);
//    }

}

bool Plane2DEnvironment::Plan(std::vector<int> nStart, std::vector<int> nGoal)
{
    if (!mSS)
        return false;
    ob::ScopedState<> start(mSS->getStateSpace());
    start[0] = nStart[0];
    start[1] = nStart[1];
    ob::ScopedState<> goal(mSS->getStateSpace());
    goal[0] = nGoal[0];
    goal[1] = nGoal[1];

    mSS->setStartAndGoalStates(start, goal);

    // generate a few solutions; all will be added to the goal;
    if (mSS->getPlanner()) {
        mSS->getPlanner()->clear();
    }

    mSS->solve();
    const std::size_t ns = mSS->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int) ns);
    if (mSS->haveSolutionPath()) {
        mSS->simplifySolution();
        og::PathGeometric &p = mSS->getSolutionPath();
        mSS->getPathSimplifier()->simplifyMax(p);
        mSS->getPathSimplifier()->smoothBSpline(p);
        return true;
    }

    return false;
}

void Plane2DEnvironment::GetSolution(std::vector<std::vector<int> > &vSolution)
{
    vSolution.clear();
    if (!mSS || !mSS->haveSolutionPath())
        return;
    og::PathGeometric &p = mSS->getSolutionPath();
    p.interpolate();
    for (std::size_t i = 0; i < p.getStateCount(); ++i) {
        const int
            x = std::min(mMaxX, (int) p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
        const int
            y = std::min(mMaxY, (int) p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
        std::vector<int> pixel = {x, y};
        vSolution.push_back(pixel);
    }

}

bool Plane2DEnvironment::IsStateValid(const ob::State *state) const
{
    const int x = std::min((int) state->as<ob::RealVectorStateSpace::StateType>()->values[0], mMaxX);
    const int y = std::min((int) state->as<ob::RealVectorStateSpace::StateType>()->values[1], mMaxY);
    return mObstacles[y][x] == 0;
}

Plane2DXYThetaEnvironment::Plane2DXYThetaEnvironment(const std::vector<std::vector<int> > &obstacles,
                                                     std::string &planner,
                                                     int halfLength, int halfWidth):
    mObstacles(obstacles),
    mdRobotHalfLength(halfLength),
    mdRobotHalfWidth(halfWidth)
{
    SetupRobot();
    auto space(std::make_shared<ob::SE2StateSpace>());
    mMaxX = mObstacles[0].size() - 1;
    mMaxY = mObstacles.size() - 1;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(std::max(mMaxY, mMaxX));
    space->setBounds(bounds);
    mSS = std::make_shared<og::SimpleSetup>(space);
    // set state validity checking for this space
    mSS->setStateValidityChecker([this](const ob::State *state)
                                 { return IsStateValid(state); });

    space->setup();
    mSS->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
    //      mSS->setPlanner(std::make_shared<og::RRTConnect>(mSS->getSpaceInformation()));
    ob::SpaceInformationPtr si = mSS->getSpaceInformation();
    SetupPlanner(mSS, planner);
}

bool Plane2DXYThetaEnvironment::IsStateValid(const ob::State *state) const
{
    const int x = std::min((int) state->as<ob::SE2StateSpace::StateType>()->getX(), mMaxX);
    const int y = std::min((int) state->as<ob::SE2StateSpace::StateType>()->getY(), mMaxY);
    const double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();
    if (yaw > boost::math::constants::pi<double>() || yaw <= -boost::math::constants::pi<double>())
    {
        return false;
    }
    Eigen::Vector2i center;
    center << x , y;
    std::vector<std::vector<std::vector<int> > > triangles;
    Eigen::Matrix<double, 2, 4> robotCorners;
    std::vector<int> bounds;
    GetTriangles(center, yaw, triangles, robotCorners);
    GetRobotBounds(robotCorners, bounds);
    bool bCollision = RobotCollideObstacle(bounds, triangles);
    return !bCollision;
}

bool Plane2DXYThetaEnvironment::RobotCollideObstacle(std::vector<int> &bounds,
                                                     std::vector<std::vector<std::vector<int> > > &triangles) const
{
    int obstacleCount = 0;
    int obstacleThresh = 5;
    for (int i = bounds[0]; i < bounds[1]; i++)
    {
        for (int j = bounds[2]; j < bounds[3]; j++)
        {
            if (mObstacles[j][i] == 1)
            {
                Eigen::Vector2i point;
                point << i, j;
                for (int k = 0; k < triangles.size(); k++)
                {
                    if (PointInTriangle(point, triangles[k]))
                    {
                        obstacleCount++;
                        if (obstacleCount >= obstacleThresh)
                        {
                            return true;
                        }
                        break;
                    }
                }
            }
        }
    }
    return false;
}

void Plane2DXYThetaEnvironment::SetupRobot()
{
    mRobotCornersUnMoved << mdRobotHalfWidth, -mdRobotHalfWidth, -mdRobotHalfWidth, mdRobotHalfWidth,
                            mdRobotHalfLength, mdRobotHalfLength, -mdRobotHalfLength, -mdRobotHalfLength;
}

void Plane2DXYThetaEnvironment::GetTriangles(Eigen::Vector2i &center,
                                             double angle,
                                             std::vector<std::vector<std::vector<int> > > &triangles,
                                             Eigen::Matrix<double, 2, 4> &robotCorners) const
{
    triangles.clear();
    Eigen::Rotation2D<double> rotMat(angle);
    robotCorners = rotMat.toRotationMatrix() * mRobotCornersUnMoved.cast<double>();
    robotCorners.colwise() += center.cast<double>();
    std::vector<std::vector<int> > triangle(3, std::vector<int>(2, 0));
    triangle[0][0] = robotCorners(0, 0);
    triangle[0][1] = robotCorners(1, 0);
    triangle[1][0] = robotCorners(0, 1);
    triangle[1][1] = robotCorners(1, 1);
    triangle[2][0] = robotCorners(0, 2);
    triangle[2][1] = robotCorners(1, 2);
    triangles.push_back(triangle);
    triangle[1][0] = robotCorners(0, 3);
    triangle[1][1] = robotCorners(1, 3);
    triangles.push_back(triangle);
}

void Plane2DXYThetaEnvironment::GetRobotBounds(Eigen::Matrix<double, 2, 4> &robotCorners,
                                               std::vector<int> &bounds) const
{
    bounds.resize(4, 0);
    Eigen::Vector2d minCoord = robotCorners.rowwise().minCoeff();
    Eigen::Vector2d maxCoord = robotCorners.rowwise().maxCoeff();
    bounds[0] = std::max(0, static_cast<int>(minCoord(0)));
    bounds[2] = std::max(0, static_cast<int>(minCoord(1)));
    bounds[1] = std::min(mMaxX, static_cast<int>(maxCoord(0)));
    bounds[3] = std::min(mMaxY, static_cast<int>(maxCoord(1)));
}


bool Plane2DXYThetaEnvironment::PointInTriangle(Eigen::Vector2i &point, std::vector<std::vector<int> > triangle) const
{
    // http://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
    // as we are dealing with int type values, we are not considering the accurate checking process

    int x = point(0);
    int y = point(1);
    int x1 = triangle[0][0];
    int y1 = triangle[0][1];
    int x2 = triangle[1][0];
    int y2 = triangle[1][1];
    int x3 = triangle[2][0];
    int y3 = triangle[2][1];

    double denominator = static_cast<double>((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3));
    double a = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / denominator;
    double b = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / denominator;
    double c = 1 - a - b;
    return a >= 0 && a <= 1 && 0 <= b && b <= 1 && 0 <= c && c <= 1;
}

bool Plane2DXYThetaEnvironment::Plan(std::vector<int> nStart, std::vector<int> nGoal,
                                     float startYaw, float goalYaw)
{
    if (!mSS)
        return false;
    ob::ScopedState<ob::SE2StateSpace> start(mSS->getStateSpace());
    start->setX(nStart[0]);
    start->setY(nStart[1]);
    start->setYaw(startYaw);
    ob::ScopedState<ob::SE2StateSpace> goal(mSS->getStateSpace());
    goal->setX(nGoal[0]);
    goal->setY(nGoal[1]);
    goal->setYaw(goalYaw);

    mSS->setStartAndGoalStates(start, goal);

    // generate a few solutions; all will be added to the goal;
    if (mSS->getPlanner()) {
        mSS->getPlanner()->clear();
    }

    mSS->solve(1.0);
    const std::size_t ns = mSS->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int) ns);
    if (mSS->haveSolutionPath()) {
        mSS->simplifySolution();
        og::PathGeometric &p = mSS->getSolutionPath();
        mSS->getPathSimplifier()->simplifyMax(p);
        mSS->getPathSimplifier()->smoothBSpline(p);
        return true;
    }

    return false;
}

void Plane2DXYThetaEnvironment::GetSolution(std::vector<std::vector<float> > &vSolution)
{
    vSolution.clear();
    if (!mSS || !mSS->haveSolutionPath())
        return;
    og::PathGeometric &p = mSS->getSolutionPath();
    p.interpolate();
    for (std::size_t i = 0; i < p.getStateCount(); ++i) {
        const float
            x = std::min(static_cast<float>(mMaxX), static_cast<float>(p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX()));
        const float
            y = std::min(static_cast<float>(mMaxY), static_cast<float>(p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY()));

        const float yaw = static_cast<float>(p.getState(i)->as<ob::SE2StateSpace::StateType>()->getYaw());
        std::vector<float> pixel = {x, y, yaw};
        vSolution.push_back(pixel);
    }
}



}