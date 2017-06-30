#ifndef ORB_SLAM2_OMPLPATHPLANNING_H
#define ORB_SLAM2_OMPLPATHPLANNING_H
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ORB_SLAM2
{
class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const std::vector<std::vector<int> > &obstacles, std::string &planner);
    bool Plan(std::vector<int> nStart, std::vector<int> nGoal);
    void GetSolution(std::vector<std::vector<int> > &vSolution);

private:

    bool IsStateValid(const ob::State *state) const;

    og::SimpleSetupPtr ss_;
    int maxX_;
    int maxY_;
    std::vector<std::vector<int> > mObstacles_;
};

}

#endif
