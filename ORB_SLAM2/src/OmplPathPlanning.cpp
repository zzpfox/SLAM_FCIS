#include "OmplPathPlanning.h"

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ORB_SLAM2
{
Plane2DEnvironment::Plane2DEnvironment(const std::vector<std::vector<int> > &obstacles, std::string &planner)
    : mObstacles_(obstacles)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>());
    space->addDimension(0.0, mObstacles_[0].size());
    space->addDimension(0.0, mObstacles_.size());

    maxX_ = mObstacles_[0].size() - 1;
    maxY_ = mObstacles_.size() - 1;
    ss_ = std::make_shared<og::SimpleSetup>(space);

    // set state validity checking for this space
    ss_->setStateValidityChecker([this](const ob::State *state)
                                 { return IsStateValid(state); });
    space->setup();
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
    //      ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    ob::SpaceInformationPtr si = ss_->getSpaceInformation();
    if (planner == "" || planner == "RRTConnect")
    {
        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "RRTstar")
    {
        std::shared_ptr<og::RRTstar> newPlanner = std::make_shared<og::RRTstar>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "TRRT")
    {
        std::shared_ptr<og::TRRT> newPlanner = std::make_shared<og::TRRT>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "pRRT")
    {
        std::shared_ptr<og::pRRT> newPlanner = std::make_shared<og::pRRT>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "SBL")
    {
        std::shared_ptr<og::SBL> newPlanner = std::make_shared<og::SBL>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "pSBL")
    {
        std::shared_ptr<og::pSBL> newPlanner = std::make_shared<og::pSBL>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "FMT")
    {
        std::shared_ptr<og::FMT> newPlanner = std::make_shared<og::FMT>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "BFMT")
    {
        std::shared_ptr<og::BFMT> newPlanner = std::make_shared<og::BFMT>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "SPARS")
    {
        std::shared_ptr<og::SPARS> newPlanner = std::make_shared<og::SPARS>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "SPARStwo")
    {
        std::shared_ptr<og::SPARStwo> newPlanner = std::make_shared<og::SPARStwo>(si);
        ss_->setPlanner(newPlanner);
    }
    else if (planner == "PRM")
    {
        std::shared_ptr<og::PRM> newPlanner = std::make_shared<og::PRM>(si);
        ss_->setPlanner(newPlanner);
    }
    else
    {
        std::cout << "\x1B[31m" << "Unrecognized planner name: " << planner
                  << "\x1B[0m" << std::endl;
        std::cout << "Using RRTConnect instead ..." << std::endl;
        std::shared_ptr<og::RRTConnect> newPlanner = std::make_shared<og::RRTConnect>(si);
        ss_->setPlanner(newPlanner);
    }

}

bool Plane2DEnvironment::Plan(std::vector<int> nStart, std::vector<int> nGoal)
{
    if (!ss_)
        return false;
    ob::ScopedState<> start(ss_->getStateSpace());
    start[0] = nStart[0];
    start[1] = nStart[1];
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal[0] = nGoal[0];
    goal[1] = nGoal[1];

    ss_->setStartAndGoalStates(start, goal);

    // generate a few solutions; all will be added to the goal;
    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
    }

    ss_->solve();
    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int) ns);
    if (ss_->haveSolutionPath()) {
        ss_->simplifySolution();
        og::PathGeometric &p = ss_->getSolutionPath();
        ss_->getPathSimplifier()->simplifyMax(p);
        ss_->getPathSimplifier()->smoothBSpline(p);
        return true;
    }

    return false;
}

void Plane2DEnvironment::GetSolution(std::vector<std::vector<int> > &vSolution)
{
    vSolution.clear();
    if (!ss_ || !ss_->haveSolutionPath())
        return;
    og::PathGeometric &p = ss_->getSolutionPath();
    p.interpolate();
    for (std::size_t i = 0; i < p.getStateCount(); ++i) {
        const int
            x = std::min(maxX_, (int) p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
        const int
            y = std::min(maxY_, (int) p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
        std::vector<int> pixel = {x, y};
        vSolution.push_back(pixel);
    }

}

bool Plane2DEnvironment::IsStateValid(const ob::State *state) const
{
    const int x = std::min((int) state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxX_);
    const int y = std::min((int) state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxY_);
    return mObstacles_[y][x] == 0;
}

}