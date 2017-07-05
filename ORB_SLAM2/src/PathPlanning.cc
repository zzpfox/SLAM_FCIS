#include "PathPlanning.h"
#include <algorithm>
#include <memory>
#include "OmplPathPlanning.h"
namespace ORB_SLAM2
{

PathPlanning2D::PathPlanning2D(std::string planner, float pointSize, float lineWidth):
    mfcLeafSize(0.025), msPlanner(planner), mPointSize(pointSize), mLineWidth(lineWidth), mfObstacleWidth(9.0)
{
    mvBounds = std::vector<float> (4, 0.0);
}

void PathPlanning2D::UpdatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    mCloud = cloud;
}

void PathPlanning2D::reset()
{
    mvStartG.clear();
    mvStartW.clear();
    mvTmpStartW.clear();
    mvTargetG.clear();
    mvTargetW.clear();
    mvTmpTargetW.clear();
    mCloud.reset();
    mSolution.clear();
    mObstacles.clear();
    mObstaclesHeight.clear();
    mvBounds.clear();
    mvCandidatesValidStart.clear();
    mvCandidatesValidTarget.clear();
}
void PathPlanning2D::PlanPath(std::vector<float> &start,
                              std::vector<float> &target,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    UpdatePointCloud(cloud);
    mvStartW = start;
    mvTargetW = target;
    Get2DBounds();
    WorldToGrid(mvStartW, mvStartG);
    WorldToGrid(mvTargetW, mvTargetG);
    OmplPathPlanning();
}
void PathPlanning2D::ShowPlannedPath()
{
    if (mvStartG.size() == 2 && mvTargetG.size() == 2)
    {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(mvStartW[0], 5, mvStartW[1]);
        glVertex3f(mvTargetW[0], 5, mvTargetW[1]);
        glEnd();
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 1.0, 0.0);
        glVertex3f(mvTmpStartW[0], 5, mvTmpStartW[1]);
        glVertex3f(mvTmpTargetW[0], 5, mvTmpTargetW[1]);
        glEnd();
    }

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.8, 0.5, 0.4);
    for (int k = 0; k < mvCandidatesValidStart.size(); k++)
    {
        std::vector<float> point;
        std::vector<int> tmp = {mvCandidatesValidStart[k].second[1], mvCandidatesValidStart[k].second[0]};
        GridToWorld(tmp, point);
        glVertex3f(point[0], 5, point[1]);
    }
    glEnd();
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.5, 0.5);
    for (int k = 0; k < mvCandidatesValidTarget.size(); k++)
    {
        std::vector<float> point;
        std::vector<int> tmp = {mvCandidatesValidTarget[k].second[1], mvCandidatesValidTarget[k].second[0]};
        GridToWorld(tmp, point);
        glVertex3f(point[0], 5, point[1]);
    }
    glEnd();

    glLineWidth(mLineWidth);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    if (mSolution.size() > 0)
    {
        for (int i = 0; i < static_cast<int> (mSolution.size()) - 1; i++)
        {
            glVertex3f(mSolution[i][0], 5.0, mSolution[i][1]);
            glVertex3f(mSolution[i + 1][0], 5.0, mSolution[i + 1][1]);
        }
    }

    glEnd();

    {
        glPushAttrib(GL_ENABLE_BIT);
        if (mSolution.size() > 0 && mvStartG.size() == 2 && mvTargetG.size() == 2)
        {
            glLineStipple(2, 0x00FF);
            glEnable(GL_LINE_STIPPLE);
            glLineWidth(mLineWidth);
            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_LINES);
            glVertex3f(mvStartW[0], 5.0, mvStartW[1]);
            glVertex3f(mSolution.front()[0], 5.0, mSolution.front()[1]);
            glEnd();
            glBegin(GL_LINES);
            glVertex3f(mvTargetW[0], 5.0, mvTargetW[1]);
            glVertex3f(mSolution.back()[0], 5.0, mSolution.back()[1]);
            glEnd();
            glPopAttrib();

        }
    }

    glPointSize(mPointSize / 6.0);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (int i = 0; i < mObstacles.size(); i++)
    {
        for (int j = 0; j < mObstacles[0].size(); j++)
        {
            if (mObstacles[i][j] == 1)
            {
                std::vector<float> tmpPoint;
                std::vector<int> tmpSrc = {j, i};
                GridToWorld(tmpSrc, tmpPoint);
                glVertex3f(tmpPoint[0], 5, tmpPoint[1]);
            }
        }
    }
    glEnd();
}

void PathPlanning2D::OmplPathPlanning()
{
    if (mvStartG.size() != 2 || mvTargetG.size() != 2)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    GenerateOccupancyMap();

    if (!GetClosestFreePoint(mvStartG, 80, mvCandidatesValidStart))
    {
        std::cout << "Cannot find an obstacle-free place near start point" << std::endl;
        return;
    }
    if (!GetClosestFreePoint(mvTargetG, 80, mvCandidatesValidTarget))
    {
        std::cout << "Cannot find an obstacle-free place near target point" << std::endl;
        return;
    }
    GridToWorld(mvStartG, mvTmpStartW);
    GridToWorld(mvTargetG, mvTmpTargetW);
    Plane2DEnvironment env(mObstacles, msPlanner);
    mSolution.clear();
    if (env.Plan(mvStartG, mvTargetG))
    {
        std::vector<std::vector<int> > vSolution;
        env.GetSolution(vSolution);
        if (vSolution.size() <= 0)
        {
            std::cout << "Cannot find a path ... " << std::endl;
        }
        else
        {
            std::cout << "Find a path ..." << std::endl;
            for (std::vector<std::vector<int> >::iterator st = vSolution.begin(); st != vSolution.end(); st++) //in right order
            {
                std::vector<float> point;
                GridToWorld(*st, point);
                mSolution.push_back(point);
            }

        }
    }
    else
    {
        std::cout << "Cannot find a path ... " << std::endl;
    }
}

//Deprecated
void PathPlanning2D::SimplePathPlanning()
{

    if (mvStartG.size() != 2 || mvTargetG.size() != 2)
    {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    const int nMaxIter = 10000;

    GenerateOccupancyMap();
    if (!GetClosestFreePoint(mvStartG, 40, mvCandidatesValidStart))
    {
        std::cout << "Cannot find an obstacle-free place near start point" << std::endl;
        return;
    }
    if (!GetClosestFreePoint(mvTargetG, 40, mvCandidatesValidTarget))
    {
        std::cout << "Cannot find an obstacle-free place near target point" << std::endl;
        return;
    }

    std::vector<std::shared_ptr<RRTNode> > vStartTree;
    std::vector<std::shared_ptr<RRTNode> > vTargetTree;

    vStartTree.push_back(std::make_shared<RRTNode> (mvStartG));
    vTargetTree.push_back(std::make_shared<RRTNode> (mvTargetG));
    int nCount = 0;
    int ncStepSize = 6; //pixel

    std::vector<std::shared_ptr<RRTNode> > vSolution;
    srand (time(NULL));
    bool bFoundPath = false;
    mSolution.clear();
    while (nCount < nMaxIter)  //max iteration
    {
        nCount++;
        int nMiddleX = rand() % mnSizeX;
        int nMiddleY = rand() % mnSizeY;

        int innerMaxIter = 50;
        for (int iter = 0; iter < innerMaxIter; iter++)
        {
            if (mObstacles[nMiddleY][nMiddleX] == 0)
            {
                break;
            }
            nMiddleX = rand() % mnSizeX;
            nMiddleY = rand() % mnSizeY;
            if (iter >= innerMaxIter - 1)
            {
                std::cout << "\x1B[31m" << "ERROR: cannot generate a collision-free random point"
                          << "\x1B[0m" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        if (!SimpleRRTTreeExpand(vStartTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };
        if (!SimpleRRTTreeExpand(vTargetTree, mObstacles, nMiddleX, nMiddleY, ncStepSize))
        {
            continue;
        };

        if (!SimpleRRTTreesIntersect(vTargetTree, vStartTree, mObstacles, vSolution, ncStepSize))
        {
            if (!SimpleRRTTreesIntersect(vStartTree, vTargetTree, mObstacles, vSolution, ncStepSize))
            {
                continue;
            }
        }

        bFoundPath = true;
        //the order in Solution is the path, now smooth
        int nSpan = 2; //follow rrt
        while (nSpan < vSolution.size()) {
            bool bChanged = false;
            for (int i = 0; i + nSpan < vSolution.size(); i++) {
                bool bCanErase = true;// true to erase
                int yy = vSolution[i]->mPoint[1];
                int xx = vSolution[i]->mPoint[0];
                int yyy = vSolution[i + nSpan]->mPoint[1];
                int xxx = vSolution[i + nSpan]->mPoint[0];

                for (int ii = std::min(xx, xxx); ii <= std::max(xx, xxx); ii++) {
                    for (int jj = std::min(yy, yyy); jj <= std::max(yy, yyy); jj++) {
                        if (mObstacles[jj][ii] == 1) {
                            bCanErase = false;
                            break;
                        }
                    }
                    if (!bCanErase)
                        break;
                }


                if (bCanErase) {
                    for (int x = 1; x < nSpan; x++) {
                        vSolution.erase(vSolution.begin() + i + 1);
                    }
                    bChanged = true;
                }
            }

            if (!bChanged) nSpan++;
        }


        for (std::vector<std::shared_ptr<RRTNode> >::iterator st = vSolution.begin(); st != vSolution.end(); st++) //in right order
        {
            std::vector<int> tmpPoint = {(*st)->mPoint[0], (*st)->mPoint[1]};
            std::vector<float> point;
            GridToWorld(tmpPoint, point);
            mSolution.push_back(point);
        }
        break;
    }

//    glPointSize(2);
//    glBegin(GL_POINTS);
//    glColor3f(1.0,0.0,0.5);
//    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vStartTree.begin();st!=vStartTree.end();st++) {
//
//        int yy = (*st)->mPoint[1];
//        int xx = (*st)->mPoint[0];
//        glVertex3f(xx * mfcLeafSize + mvBounds[0], 5.0, yy * mfcLeafSize + mvBounds[2]);
//    }
//    glEnd();
//
//
//    glPointSize(2);
//    glBegin(GL_POINTS);
//    glColor3f(0.25,0.21,0.16);
//    for(std::vector<std::shared_ptr<RRTNode> >::iterator st=vTargetTree.begin();st!=vTargetTree.end();st++) {
//
//        int yy = (*st)->mPoint[1];
//        int xx = (*st)->mPoint[0];
//        glVertex3f(xx * mfcLeafSize + mvBounds[0], 5.0, yy * mfcLeafSize + mvBounds[2]);
//    }
//
//    glEnd();
    if (bFoundPath)
    {
        std::cout << "Find a path after " << nCount << " iterations ..." << std::endl;
    }
    else
    {
        std::cout << "Cannot find a path after " << nMaxIter << " iterations ..." << std::endl;
    }

}

bool PathPlanning2D::GetClosestFreePoint(std::vector<int> &output, int searchWidth,
                                         std::vector<std::pair<float, std::vector<int> > > &candidateOutput)
{
    if (mObstacles[output[1]][output[0]] == 1) {
        float heightThreshold = 0.6;
        std::vector<std::pair<float, std::vector<int> > > vCandidates(8,
                                                                      std::make_pair(std::numeric_limits<float>::lowest(),
                                                                                     std::vector<int>(2, 0)));
        std::vector<bool> vbCandidateInValid(8, false);
        std::vector<bool> vbCandidateSearchComplete(8, false);
        for (int i = 1; i < searchWidth + 1; i++) {
            std::vector<std::vector<int> > vSearchDirection = {{0, i},
                                                               {i, i},
                                                               {i, 0},
                                                               {i, -i},
                                                               {0, -i},
                                                               {-i, -i},
                                                               {-i, 0},
                                                               {-i, i}};
//            find the minimum of the maximum height along 8 directions
//            if the height is too large, abandon that direction
            for (int j = 0; j < vSearchDirection.size(); j++) {
                if (vbCandidateInValid[j] || vbCandidateSearchComplete[j]) {
                    continue;
                }
                int y = output[1] + vSearchDirection[j][0];
                int x = output[0] + vSearchDirection[j][1];
                if (x < 0 || x >= mObstacles[0].size() || y < 0 || y >= mObstacles.size())
                {
                    vbCandidateInValid[j] = true;
                    continue;
                }
                float currentHeight = mObstaclesHeight[y][x];
                if (currentHeight >= heightThreshold) {
                    vbCandidateInValid[j] = true;
                    continue;
                }
                std::pair<float, std::vector<int> >& candidateHeightCoord = vCandidates[j];
                if (candidateHeightCoord.first < currentHeight) {
                    candidateHeightCoord.first = currentHeight;
                }
                std::vector<int> coord = {y, x};
                candidateHeightCoord.second = coord;
                if (mObstacles[coord[0]][coord[1]] == 0) {
                    vbCandidateSearchComplete[j] = true;
                }
            }
            bool finished = std::all_of(vbCandidateSearchComplete.begin(),
                                        vbCandidateSearchComplete.end(), [](bool i)
                                        { return i; });
            if (finished) {
                break;
            }

        }
        std::vector<std::pair<float, std::vector<int> > > vCandidatesValid;
        for (int i = 0; i < vCandidates.size(); i++) {
            if (!vbCandidateInValid[i]) {
                vCandidatesValid.push_back(vCandidates[i]);
            }
        }
        if (vCandidatesValid.size() > 0) {
            std::sort(vCandidatesValid.begin(), vCandidatesValid.end(),
                      [](const std::pair<float, std::vector<int> > &lhs,
                         const std::pair<float, std::vector<int> > &rhs)
                      { return lhs.first < rhs.first; });
//            for (int k = 0; k < vCandidatesValid.size(); k++)
//            {
//                std::cout << vCandidatesValid[k].second[1] << " " << vCandidatesValid[k].second[0] << "   height " << vCandidatesValid[k].first<<std::endl;
//            }
//            std::cout << "======" << std::endl;
            output[0] = vCandidatesValid[0].second[1];
            output[1] = vCandidatesValid[0].second[0];
            candidateOutput = vCandidatesValid;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;

//        for (int i = -nSearchWidth; i < nSearchWidth; i++)
//        {
//            for (int j = -nSearchWidth; j < nSearchWidth; j++)
//            {
//                int y = output[1] + i;
//                int x = output[0] + j;
//                if (y >= 0 && y < obstacles.size() && x >= 0 && x < obstacles[0].size())
//                {
//                    if (obstacles[y][x] == 0 && (abs(i) + abs(j)) < minDistance)
//                    {
//                        minDistance = abs(i) + abs(j);
//                        tmpOutput[0] = x;
//                        tmpOutput[1] = y;
//                    }
//
//                }
//            }
//        }
//        output = tmpOutput;
//        // TODO: deal with the case when there is no space without obstacles

}

bool PathPlanning2D::SimpleRRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
                                    std::vector<std::vector<int> > &mObstacles,
                                    int nMiddleX, int nMiddleY, int ncStepSize)
{
    int nDisMin = std::numeric_limits<int>::max();
    int nConnect = 1;

    std::shared_ptr<RRTNode> pMin = std::make_shared<RRTNode> (std::vector<int> (2, nDisMin));
    for (std::vector<std::shared_ptr<RRTNode> >::iterator sit = tree.begin(); sit != tree.end(); sit++) {
        std::vector<int> vTmpPoint = (*sit)->mPoint;
        int nTmpDist = abs(nMiddleX - vTmpPoint[0]) + abs(nMiddleY - vTmpPoint[1]);
        if (nTmpDist < nDisMin) {
            nDisMin = nTmpDist;
            pMin = *sit;
        }
    }

    if (nDisMin < 4)
        return false;//means too close

    int nStepX = pMin->mPoint[0] > nMiddleX ? (-1) : 1;
    int nStepY = pMin->mPoint[1] > nMiddleY ? (-1) : 1;// different step along different direction

    //use sin,cos to determine direction
    float fRadius = sqrt(pow(float(pMin->mPoint[0] - nMiddleX), 2.0) + pow(float(pMin->mPoint[1] - nMiddleY), 2.0));
    int nStepSizeX = int(float(ncStepSize * abs(pMin->mPoint[0] - nMiddleX)) / fRadius) + 1;// sin
    int nStepSizeY = int(float(ncStepSize * abs(pMin->mPoint[1] - nMiddleY)) / fRadius) + 1;// cos

    for (int i = pMin->mPoint[0]; abs(i - pMin->mPoint[0]) < nStepSizeX; i += nStepX) {
        int OUT = 0;
        for (int j = pMin->mPoint[1]; abs(j - pMin->mPoint[1]) < nStepSizeY; j += nStepY) {
            if (i < 0 || i >= mObstacles[0].size() ||
                j < 0 || j >= mObstacles.size() ||
                mObstacles[j][i] == 1)//found obstacle or out of bounds
            {
                nConnect = 0;// not directly connect
                OUT = 1;
                break;
            }
        }
        if (OUT == 1)
            break;
    }

    if (nConnect == 1) {// add
//            nStepX = pMinStart->mPoint[0] > nMiddleX ? (-1) : 1;
//            nStepY = pMinStart->mPoint[1] > nMiddleY ? (-1) : 1;

        int x = nStepX * nStepSizeX + pMin->mPoint[0];
        int y = nStepY * nStepSizeY + pMin->mPoint[1];
        std::vector<int> vTmpPoint = {x, y};
        std::shared_ptr<RRTNode> tmpNode = std::make_shared<RRTNode> (vTmpPoint);
        tmpNode->addParent(pMin);
        tree.push_back(tmpNode);
        return true;
    }
    else
    {
        return false;
    }

}

bool PathPlanning2D::SimpleRRTTreesIntersect(std::vector<std::shared_ptr<RRTNode> > &tree,
                                        std::vector<std::shared_ptr<RRTNode> > &treePop,
                                        std::vector<std::vector<int> > &mObstacles,
                                        std::vector<std::shared_ptr<RRTNode> > &vSolution,
                                        int ncStepSize)
{
    //check whether new node near the other tree, first check near target tree
    bool bConnect = false;
    for (std::vector<std::shared_ptr<RRTNode> >::iterator sit = tree.begin(); sit != tree.end(); sit++) {
        int y = (*sit)->mPoint[1];
        int x = (*sit)->mPoint[0];
        if (abs(treePop.back()->mPoint[0] - x) + abs(treePop.back()->mPoint[1]  - y) <= 2 * ncStepSize) {
            int nOut = 0;
            for (int i =  std::min(treePop.back()->mPoint[0], x); i <= std::max(treePop.back()->mPoint[0], x); i++) {

                for (int j =  std::min(treePop.back()->mPoint[1], y); j <=  std::max(treePop.back()->mPoint[1], y); j++) {
                    if (mObstacles[j][i] == 1) {
                        nOut = 1;
                        bConnect = false;
                        break;
                    }
                }
                if (nOut == 1)
                    break;
            }
            if (!nOut)//connect
            {
                bConnect = true;//connect to start tree
                vSolution.clear();
                vSolution.push_back(treePop.back());
                while(vSolution.back()->mParent)
                {
                    vSolution.push_back(vSolution.back()->mParent);
                }
                std::reverse(vSolution.begin(), vSolution.end());
                vSolution.push_back(*sit);
                while(vSolution.back()->mParent)
                {
                    vSolution.push_back(vSolution.back()->mParent);
                }
                break;
            }
        }
    }
    return bConnect;
}

void PathPlanning2D::Get2DBounds()
{
    mvBounds[0] = std::numeric_limits<float>::max();
    mvBounds[1] = std::numeric_limits<float>::lowest();
    mvBounds[2] = std::numeric_limits<float>::max();
    mvBounds[3] = std::numeric_limits<float>::lowest();
    for (auto p: mCloud->points)
    {
        if (p.x < mvBounds[0])
        {
            mvBounds[0] = p.x;
        }
        if (p.x > mvBounds[1])
        {
            mvBounds[1] = p.x;
        }
        if (p.z < mvBounds[2])
        {
            mvBounds[2] = p.z;
        }
        if (p.z > mvBounds[3])
        {
            mvBounds[3] = p.z;
        }
    }
    mvBounds[0] = std::min(mvBounds[0], mvStartW[0]);
    mvBounds[0] = std::min(mvBounds[0], mvTargetW[0]);
    mvBounds[1] = std::max(mvBounds[1], mvStartW[0]);
    mvBounds[1] = std::max(mvBounds[1], mvTargetW[0]);
    mvBounds[2] = std::min(mvBounds[2], mvStartW[1]);
    mvBounds[2] = std::min(mvBounds[2], mvTargetW[1]);
    mvBounds[3] = std::max(mvBounds[3], mvStartW[1]);
    mvBounds[3] = std::max(mvBounds[3], mvTargetW[1]);
}

void PathPlanning2D::CalGridSize()
{
    mnSizeX = static_cast<int> ((mvBounds[1] - mvBounds[0] + 0.2) / mfcLeafSize);
    mnSizeY = static_cast<int> ((mvBounds[3] - mvBounds[2] + 0.2) / mfcLeafSize);
}
void PathPlanning2D::GenerateOccupancyMap()
{
    CalGridSize();
    mObstacles.clear();
    mObstacles = std::vector<std::vector<int> > (mnSizeY, std::vector<int> (mnSizeX, 0));
    mObstaclesHeight.clear();
    mObstaclesHeight = std::vector<std::vector<float> > (mnSizeY, std::vector<float> (mnSizeX, std::numeric_limits<float>::lowest()));
//    std::vector<std::vector<int> > obstaclesOri(mnSizeY, std::vector<int> (mnSizeX,0 ));
//    for (auto p : mCloud->points) {
//        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcLeafSize);
//        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcLeafSize);
//        obstaclesOri[nTmpY][nTmpX] = 1;
//    }
//    for (auto p : mCloud->points) {
//        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcLeafSize);
//        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcLeafSize);
//        for (int i = nTmpX - mfObstacleWidth; i < nTmpX + mfObstacleWidth; i++) {
//            for (int j = nTmpY - mfObstacleWidth; j < nTmpY + mfObstacleWidth; j++) {
//                if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY) {
//                    mObstacles[j][i] = 1;
//
//                    if (obstaclesOri[j][i] == 0) // expanded cost map
//                    {
//                        mObstaclesHeight[nTmpY][nTmpX] = std::numeric_limits<float>::max();
//                    }
//                }
//
//            }
//        }
//    }
    for (auto p : mCloud->points) {
        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcLeafSize);
        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcLeafSize);
        if (mObstaclesHeight[nTmpY][nTmpX] < -p.y)
        {
            mObstaclesHeight[nTmpY][nTmpX] = -p.y;
        }
        for (int i = nTmpX - mfObstacleWidth; i < nTmpX + mfObstacleWidth; i++)
        {
            for (int j = nTmpY - mfObstacleWidth; j < nTmpY + mfObstacleWidth; j++)
            {
                if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY)
                {
                    mObstacles[j][i] = 1;

                }

            }
        }

//        for (int i = nTmpX - mfObstacleWidth; i < nTmpX + mfObstacleWidth; i++)
//        {
//            for (int j = nTmpY - mfObstacleWidth; j < nTmpY + mfObstacleWidth; j++)
//            {
//                if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY)
//                {
//                    if (obstaclesOri[j][i] == 0) // expanded cost map
//                    {
//                        if (mObstaclesHeight[nTmpY][nTmpX] > -p.y)
//                        {
//                            mObstaclesHeight[nTmpY][nTmpX] = -p.y;
//                        }
//                    }
//                    else
//                    {
//                        if (mObstaclesHeight[nTmpY][nTmpX] < -p.y)
//                        {
//                            mObstaclesHeight[nTmpY][nTmpX] = -p.y;
//                        }
//                    }
//                }
//
//            }
//        }
    }
}
void PathPlanning2D::WorldToGrid(std::vector<float> &input, std::vector<int> &output)
{
    output.resize(2, 0);
    output[0] = static_cast<int> ((input[0] - mvBounds[0]) / mfcLeafSize);
    output[1] = static_cast<int> ((input[1] - mvBounds[2]) / mfcLeafSize);
}

void PathPlanning2D::GridToWorld(std::vector<int> &input, std::vector<float> &output)
{
    output.resize(2, 0);
    output[0] = input[0] * mfcLeafSize + mvBounds[0];
    output[1] = input[1] * mfcLeafSize + mvBounds[2];
}
}

