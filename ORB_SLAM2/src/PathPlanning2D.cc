#include "PathPlanning2D.h"
#include <algorithm>
#include "Thirdparty/sbpl/src/include/sbpl/headers.h"
#include "OmplPathPlanning.h"
#include <string>
#include <random>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <boost/filesystem.hpp>
namespace ORB_SLAM2
{

PathPlanning2D::PathPlanning2D(std::string planner, float pointSize,
                               float lineWidth,
                               float obstacleWidth,
                               float gridSize,
                               std::string dataFolder,
                               float upperBound, float lowerBound)
    :
    mfcGridSize(gridSize), msPlanner(planner), mPointSize(pointSize),
    mLineWidth(lineWidth), mfObstacleWidth(obstacleWidth), mbSBPLPathPlan(false),
    mfUpperBound(upperBound), mfLowerBound(lowerBound), msDataFolder(dataFolder)
{
    mvBounds = std::vector<float>(4, 0.0);
    mpSolution = std::make_shared<std::vector<std::vector<float> > >();
    std::unordered_set<std::string> OmplPlanners = {"RRTConnect", "RRTstar",
                                                    "RRTsharp", "TRRT",
                                                    "pRRT", "SBL",
                                                    "pSBL", "FMT",
                                                    "BFMT", "SPARS",
                                                    "SPARStwo", "PRM"};
    std::unordered_set<std::string> SBPLPlanners = {"adstar", "arastar"};
    if (SBPLPlanners.count(msPlanner) > 0) {
        mbSBPLPathPlan = true;
    }
    else if (OmplPlanners.count(msPlanner) > 0) {
        mbSBPLPathPlan = false;
    }
    else {
        std::cout << "\x1B[31m" << "Unrecognized planner name: " << planner
                  << "\x1B[0m" << std::endl;
        std::cout << "Using RRTConnect instead ..." << std::endl;
        msPlanner = "RRTConnect";
        mbSBPLPathPlan = false;
    }
    boost::filesystem::path dataDir(msDataFolder);
    boost::filesystem::path contourDir("Contours");
    boost::filesystem::path contoursPath = dataDir / contourDir;
    boost::filesystem::create_directories(contoursPath);
    msContourFolder = contoursPath.string();
}

void PathPlanning2D::UpdatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    mCloud = cloud;
}

void PathPlanning2D::AddPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    *mCloud += *cloud;
}

void PathPlanning2D::reset(bool cleanOccupancyMap)
{
    mvStartG.clear();
    mvStartW.clear();
    mvTmpStartW.clear();
    mvTargetG.clear();
    mvTargetW.clear();
    mvTmpTargetW.clear();
    if (mCloud) {
        mCloud.reset();
    }

    if (mpSolution) {
        mpSolution->clear();
    }
    if (cleanOccupancyMap) {
        mObstacles.clear();
        mObstaclesSeenNum.clear();
//        mObstaclesHeight.clear();
    }

    mvBounds.clear();
    mvCandidatesValidStart.clear();
    mvCandidatesValidTarget.clear();
}

bool PathPlanning2D::PlanPath(std::vector<float> &start,
                              std::vector<float> &target,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    if (mpSolution) {
        mpSolution->clear();
    }

    UpdatePointCloud(cloud);
    mvStartW = start;
    mvTargetW = target;
    Get2DBounds();
    WorldToGrid(mvStartW, mvStartG);
    WorldToGrid(mvTargetW, mvTargetG);
    bool bFindPath = false;
    if (mbSBPLPathPlan) {
        bFindPath = SBPLPathPlanning();
    }
    else {
        bFindPath = OmplPathPlanning();
    }
    return bFindPath;
}

bool PathPlanning2D::PlanPath(std::vector<float> &start,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    if (mpSolution) {
        mpSolution->clear();
    }
    UpdatePointCloud(cloud);
    mvStartW = start;
    mvTargetW.clear();
    Get2DBounds();
    mvTargetW.resize(2, 0);
    mvTargetW[0] = (mvBounds[0] + mvBounds[1]) / 2.0;
    mvTargetW[1] = mvBounds[3] - 2 * mfcGridSize;
    WorldToGrid(mvStartW, mvStartG);
    WorldToGrid(mvTargetW, mvTargetG);
    bool bFindPath = false;
    if (mbSBPLPathPlan) {
        bFindPath = SBPLPathPlanning();
    }
    else {
        bFindPath = OmplPathPlanning();
    }
    return bFindPath;
}

bool PathPlanning2D::UnvisitedAreasToGo(std::vector<float> &currentPos,
                                        std::vector<float> &target,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    UpdatePointCloud(cloud);
    mvStartW = currentPos;
    mvTargetW.clear();
    Get2DBounds();
    GenerateOccupancyMap();

    std::vector<int> currentPosInGrid;
    WorldToGrid(currentPos, currentPosInGrid);
    cv::Point2f currentPosCV = cv::Point(currentPosInGrid[0], currentPosInGrid[1]);

    cv::Mat cvObstacles, cvObstaclesFlipped;
    Convert2DVectorToMat(mObstacles, cvObstacles, false);
    cvObstacles.convertTo(cvObstacles, CV_8UC1);
    cvObstacles *= 255;
    cv::flip(cvObstacles, cvObstaclesFlipped, 0);


    std::string obstacleImage(msContourFolder + "/obstacle.png");
    cv::imwrite(obstacleImage, cvObstaclesFlipped);

    cv::Mat cvObstaclesSeenNum, cvObstaclesSeenNumFlipped;
    Convert2DVectorToMat(mObstaclesSeenNum, cvObstaclesSeenNum, false);
    cv::flip(cvObstaclesSeenNum, cvObstaclesSeenNumFlipped, 0);
    std::string ObstaclesSeenNumImage(msContourFolder + "/cvObstaclesSeenNum.png");
    cv::imwrite(ObstaclesSeenNumImage, cvObstaclesSeenNumFlipped);

    // remove high frequency noise with filters
    // Gaussian Filter (faster)
    double sigma = 0.8;
    cv::Mat filteredObstacles;
    cv::GaussianBlur(cvObstacles, filteredObstacles, cv::Size(15, 15), sigma, sigma);
//    // Median Filter
//    cv::medianBlur(cvObstacles, filteredObstacles, 7);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filteredObstacles, contours, hierarchy,
                     cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));
    double distance = std::numeric_limits<double>::max();
    int innerBoundaryId = 0;
    bool bFindInnerContour = false;
    for (int i = 0; i < contours.size(); i++) {
        double inOrOut = cv::pointPolygonTest(contours[i], currentPosCV, true);
        if (inOrOut > 0) {
//            std::cout << "contour id: " << i << " dist: " << inOrOut << std::endl;
            if (inOrOut < distance) {
                distance = inOrOut;
                innerBoundaryId = i;
                bFindInnerContour = true;
            }
        }
    }

    // visualization
    cv::Mat contourImage = cv::Mat::zeros(cvObstacles.size(), CV_8UC3);
    cv::Mat contourImageFlipped;
    if (bFindInnerContour) {
//        cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
        drawContours(contourImage, contours, innerBoundaryId, cv::Scalar(255, 255, 0),
                     3, 8, hierarchy, 0, cv::Point());
        cv::flip(contourImage, contourImageFlipped, 0);
        std::string InnerMostContourImage(msContourFolder + "/InnerMostContour.png");
        cv::imwrite(InnerMostContourImage, contourImageFlipped);
    }
    else
    {
        std::cout << "\x1B[31m" << "Cannot find the boundary of the room ... " << "\x1B[0m" << std::endl;
        return false;
    }
    cv::RNG rng(12);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(contourImage, contours, (int) i, color, 2, 8, hierarchy, 0, cv::Point());
    }
    cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
    cv::flip(contourImage, contourImageFlipped, 0);
    std::string AllContoursImage(msContourFolder + "/AllContours.png");
    cv::imwrite(AllContoursImage, contourImageFlipped);


    cv::Mat cvSeenArea = cv::Mat::zeros(cvObstacles.size(), CV_8UC1);
    for (int i = 0; i < cvSeenArea.rows; ++i) {
        uchar *p = cvSeenArea.ptr<uchar>(i);
        for (int j = 0; j < cvSeenArea.cols; ++j) {
            cv::Point2f tmpPoint = cv::Point(j, i);
            double tmpInOrOut = cv::pointPolygonTest(contours[innerBoundaryId], tmpPoint, false);
            if (tmpInOrOut > 0) {
                if (cvObstaclesSeenNum.at<int>(i, j) < 50) {
                    p[j] = 255;
                }
            }
        }
    }

    cv::Mat cvSeenAreaColor;
    cv::cvtColor(cvSeenArea, cvSeenAreaColor, cv::COLOR_GRAY2BGR);
    drawContours(cvSeenAreaColor, contours, innerBoundaryId, cv::Scalar(0, 125, 125),
                 1, 8, hierarchy, 0, cv::Point());
    cv::flip(cvSeenAreaColor, cvSeenAreaColor, 0);
    std::string UnSeenAreaImage(msContourFolder + "/UnSeenArea.png");
    cv::imwrite(UnSeenAreaImage, cvSeenAreaColor);

    cv::Mat cvUnSeenAreaGray = cvSeenArea > 150;
//    cv::cvtColor(cvUnSeenAreaGray, cvUnSeenAreaGray, cv::COLOR_BGR2GRAY);
    cv::Mat labelImage(cvUnSeenAreaGray.size(), CV_32S);
    cv::Mat stats;
    cv::Mat centroids;
    int nLabels = connectedComponentsWithStats(cvUnSeenAreaGray, labelImage, stats, centroids, 8);
//    std::cout << "stats \n" << stats << std::endl;
    int areaThresh = 1000;

//    // get all unvisited area
//    std::unordered_set<int> sLabelOfInterest;
//    for (int i = 1; i < stats.rows; i++)
//    {
//        int area = stats.at<int>(i, 4);
//        if (area > areaThresh)
//        {
//            sLabelOfInterest.insert(i);
//        }
//    }
//    std::unordered_map<int, std::set<std::vector<int> > > vAreaOfInterest;
//    for(int r = 0; r < labelImage.rows; ++r){
//        for(int c = 0; c < labelImage.cols; ++c){
//            int label = labelImage.at<int>(r, c);
//            if (sLabelOfInterest.count(label) > 0)
//            {
//                std::vector<int> pixel = {r, c};
//                vAreaOfInterest[label].insert(pixel);
//            }
//        }
//    }

    // choose the first one whose area is greater than areaThresh
    int interestedLabel = 0;
    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, 4);
        if (area > areaThresh) {
            interestedLabel = i;
        }
    }
//    std::cout << "interestedLabel: " << interestedLabel << std::endl;
    if (interestedLabel > 0) {
        std::vector<std::vector<int> > interestedPixels;
        for (int r = 0; r < labelImage.rows; ++r) {
            for (int c = 0; c < labelImage.cols; ++c) {
                int label = labelImage.at<int>(r, c);
                if (label == interestedLabel) {
                    std::vector<int> pixel = {c, r};
                    interestedPixels.push_back(pixel);
                }
            }
        }
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, interestedPixels.size() - 1);
        std::vector<int> randomInterestedPixel = interestedPixels[dis(gen)];
        while (mObstacles[randomInterestedPixel[1]][randomInterestedPixel[0]] == 1) {
            randomInterestedPixel = interestedPixels[dis(gen)];
        }
        GridToWorld(randomInterestedPixel, target);
        return true;
    }
    else {
        return false;
    }

}

bool PathPlanning2D::UnvisitedAreasToGo(std::vector<float> &target,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    UpdatePointCloud(cloud);
    mvStartW = std::vector<float>(2, 0);
    mvTargetW.clear();
    Get2DBounds();
    GenerateOccupancyMap();

    std::vector<int> originInGrid;
    WorldToGrid(mvStartW, originInGrid);
    cv::Point2f currentPosCV = cv::Point(originInGrid[0], originInGrid[1]);

    cv::Mat cvObstacles, cvObstaclesFlipped;
    Convert2DVectorToMat(mObstacles, cvObstacles, false);
    cvObstacles.convertTo(cvObstacles, CV_8UC1);
    cvObstacles *= 255;
    cv::flip(cvObstacles, cvObstaclesFlipped, 0);
    std::string obstacleImage(msContourFolder + "/obstacle.png");
    cv::imwrite(obstacleImage, cvObstaclesFlipped);

    cv::Mat cvObstaclesSeenNum, cvObstaclesSeenNumFlipped;
    Convert2DVectorToMat(mObstaclesSeenNum, cvObstaclesSeenNum, false);
    cv::flip(cvObstaclesSeenNum, cvObstaclesSeenNumFlipped, 0);
    std::string ObstaclesSeenNumImage(msContourFolder + "/cvObstaclesSeenNum.png");
    cv::imwrite(ObstaclesSeenNumImage, cvObstaclesSeenNumFlipped);

    // remove high frequency noise with filters
    // Gaussian Filter (faster)
    double sigma = 0.8;
    cv::Mat filteredObstacles;
    cv::GaussianBlur(cvObstacles, filteredObstacles, cv::Size(15, 15), sigma, sigma);
//    // Median Filter
//    cv::medianBlur(cvObstacles, filteredObstacles, 7);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filteredObstacles, contours, hierarchy,
                     cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));
    double distance = std::numeric_limits<double>::max();
    int innerBoundaryId = 0;
    bool bFindInnerContour = false;
    for (int i = 0; i < contours.size(); i++) {
        double inOrOut = cv::pointPolygonTest(contours[i], currentPosCV, true);
        if (inOrOut > 0) {
//            std::cout << "contour id: " << i << " dist: " << inOrOut << std::endl;
            if (inOrOut < distance) {
                distance = inOrOut;
                innerBoundaryId = i;
                bFindInnerContour = true;
            }
        }
    }

    // visualization
    cv::Mat contourImage = cv::Mat::zeros(cvObstacles.size(), CV_8UC3);
    cv::Mat contourImageFlipped;
    if (bFindInnerContour) {
//        cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
        drawContours(contourImage, contours, innerBoundaryId, cv::Scalar(255, 255, 0),
                     3, 8, hierarchy, 0, cv::Point());
        cv::flip(contourImage, contourImageFlipped, 0);
        std::string InnerMostContourImage(msContourFolder + "/InnerMostContour.png");
        cv::imwrite(InnerMostContourImage, contourImageFlipped);
    }
    else
    {
        std::cout << "\x1B[31m" << "Cannot find the boundary of the room ... " << "\x1B[0m" << std::endl;
        return false;
    }
    cv::RNG rng(12);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(contourImage, contours, (int) i, color, 2, 8, hierarchy, 0, cv::Point());
    }
    cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
    cv::flip(contourImage, contourImageFlipped, 0);
    std::string AllContoursImage(msContourFolder + "/AllContours.png");
    cv::imwrite(AllContoursImage, contourImageFlipped);



    cv::Mat cvSeenArea = cv::Mat::zeros(cvObstacles.size(), CV_8UC1);
    for (int i = 0; i < cvSeenArea.rows; ++i) {
        uchar *p = cvSeenArea.ptr<uchar>(i);
        for (int j = 0; j < cvSeenArea.cols; ++j) {
            cv::Point2f tmpPoint = cv::Point(j, i);
            double tmpInOrOut = cv::pointPolygonTest(contours[innerBoundaryId], tmpPoint, false);
            if (tmpInOrOut > 0) {
                if (cvObstaclesSeenNum.at<int>(i, j) < 50) {
                    p[j] = 255;
                }
            }
        }
    }

    cv::Mat cvSeenAreaColor;
    cv::cvtColor(cvSeenArea, cvSeenAreaColor, cv::COLOR_GRAY2BGR);
    drawContours(cvSeenAreaColor, contours, innerBoundaryId, cv::Scalar(0, 125, 125),
                 1, 8, hierarchy, 0, cv::Point());
    cv::flip(cvSeenAreaColor, cvSeenAreaColor, 0);
    std::string UnSeenAreaImage(msContourFolder + "/UnSeenArea.png");
    cv::imwrite(UnSeenAreaImage, cvSeenAreaColor);

    cv::Mat cvUnSeenAreaGray = cvSeenArea > 150;
//    cv::cvtColor(cvUnSeenAreaGray, cvUnSeenAreaGray, cv::COLOR_BGR2GRAY);
    cv::Mat labelImage(cvUnSeenAreaGray.size(), CV_32S);
    cv::Mat stats;
    cv::Mat centroids;
    int nLabels = connectedComponentsWithStats(cvUnSeenAreaGray, labelImage, stats, centroids, 8);
//    std::cout << "stats \n" << stats << std::endl;
    int areaThresh = 1000;

//    // get all unvisited area
//    std::unordered_set<int> sLabelOfInterest;
//    for (int i = 1; i < stats.rows; i++)
//    {
//        int area = stats.at<int>(i, 4);
//        if (area > areaThresh)
//        {
//            sLabelOfInterest.insert(i);
//        }
//    }
//    std::unordered_map<int, std::set<std::vector<int> > > vAreaOfInterest;
//    for(int r = 0; r < labelImage.rows; ++r){
//        for(int c = 0; c < labelImage.cols; ++c){
//            int label = labelImage.at<int>(r, c);
//            if (sLabelOfInterest.count(label) > 0)
//            {
//                std::vector<int> pixel = {r, c};
//                vAreaOfInterest[label].insert(pixel);
//            }
//        }
//    }

    // choose the first one whose area is greater than areaThresh
    int interestedLabel = 0;
    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, 4);
        if (area > areaThresh) {
            interestedLabel = i;
        }
    }
//    std::cout << "interestedLabel: " << interestedLabel << std::endl;
    if (interestedLabel > 0) {
        std::vector<std::vector<int> > interestedPixels;
        for (int r = 0; r < labelImage.rows; ++r) {
            for (int c = 0; c < labelImage.cols; ++c) {
                int label = labelImage.at<int>(r, c);
                if (label == interestedLabel) {
                    std::vector<int> pixel = {c, r};
                    interestedPixels.push_back(pixel);
                }
            }
        }
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, interestedPixels.size() - 1);
        std::vector<int> randomInterestedPixel = interestedPixels[dis(gen)];
        while (mObstacles[randomInterestedPixel[1]][randomInterestedPixel[0]] == 1) {
            randomInterestedPixel = interestedPixels[dis(gen)];
        }
        GridToWorld(randomInterestedPixel, target);
        return true;
    }
    else {
        return false;
    }

}

std::vector<float> PathPlanning2D::GetTargetW()
{
    return mvTargetW;
}

void PathPlanning2D::ShowPlannedPath()
{
    glPointSize(mPointSize * 2);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 1.0);
    glVertex3f(0, 5, 0);
    glEnd();
    if (mvStartW.size() == 2 && mvTargetW.size() == 2) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(mvStartW[0], 5, mvStartW[1]);
        glVertex3f(mvTargetW[0], 5, mvTargetW[1]);
        glEnd();

    }


    if (mvTmpStartW.size() == 2 && mvTmpTargetW.size() == 2) {
        glBegin(GL_POINTS);
        glColor3f(1.0, 1.0, 0.0);
        glVertex3f(mvTmpStartW[0], 5, mvTmpStartW[1]);
        glVertex3f(mvTmpTargetW[0], 5, mvTmpTargetW[1]);
        glEnd();
    }

    glBegin(GL_POINTS);
    glColor3f(0.8, 0.5, 0.4);
    for (int k = 0; k < mvCandidatesValidStart.size(); k++) {
        std::vector<float> point;
        std::vector<int> tmp = {mvCandidatesValidStart[k].second[1], mvCandidatesValidStart[k].second[0]};
        GridToWorld(tmp, point);
        glVertex3f(point[0], 5, point[1]);
    }
    glEnd();
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.5, 0.5);
    for (int k = 0; k < mvCandidatesValidTarget.size(); k++) {
        std::vector<float> point;
        std::vector<int> tmp = {mvCandidatesValidTarget[k].second[1], mvCandidatesValidTarget[k].second[0]};
        GridToWorld(tmp, point);
        glVertex3f(point[0], 5, point[1]);
    }
    glEnd();

    glLineWidth(mLineWidth);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    if (mpSolution->size() > 0) {
        for (int i = 0; i < static_cast<int> (mpSolution->size()) - 1; i++) {
            glVertex3f((*mpSolution)[i][0], 5.0, (*mpSolution)[i][1]);
            glVertex3f((*mpSolution)[i + 1][0], 5.0, (*mpSolution)[i + 1][1]);
        }
    }

    glEnd();

    {
        glPushAttrib(GL_ENABLE_BIT);
        if (mpSolution->size() > 0 && mvStartW.size() == 2 && mvTargetW.size() == 2) {
            glLineStipple(2, 0x00FF);
            glEnable(GL_LINE_STIPPLE);
            glLineWidth(mLineWidth);
            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_LINES);
            glVertex3f(mvStartW[0], 5.0, mvStartW[1]);
            glVertex3f(mpSolution->front()[0], 5.0, mpSolution->front()[1]);
            glEnd();
            glBegin(GL_LINES);
            glVertex3f(mvTargetW[0], 5.0, mvTargetW[1]);
            glVertex3f(mpSolution->back()[0], 5.0, mpSolution->back()[1]);
            glEnd();
            glPopAttrib();
        }
    }

    glPointSize(mPointSize / 6.0);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);

    for (int i = 0; i < mLethalObstacles.size(); i++) {
        for (int j = 0; j < mLethalObstacles[0].size(); j++) {
            if (mLethalObstacles[i][j] == 1) {
                std::vector<float> tmpPoint;
                std::vector<int> tmpSrc = {j, i};
                GridToWorld(tmpSrc, tmpPoint);
                glVertex3f(tmpPoint[0], 5, tmpPoint[1]);
            }
        }
    }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.7, 0.2, 1.0);
    for (int i = 0; i < mObstacles.size(); i++) {
        for (int j = 0; j < mObstacles[0].size(); j++) {
            if (mObstacles[i][j] == 1) {
                std::vector<float> tmpPoint;
                std::vector<int> tmpSrc = {j, i};
                GridToWorld(tmpSrc, tmpPoint);
                glVertex3f(tmpPoint[0], 5, tmpPoint[1]);
            }
        }
    }
    glEnd();
}

bool PathPlanning2D::SBPLPathPlanning()
{
    bool bFindPath = false;
    if (mvStartG.size() != 2 || mvTargetG.size() != 2) {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    GenerateOccupancyMap();

    if (!GetClosestFreePoint(mvStartG, 80, mvCandidatesValidStart)) {
        std::cout << "Cannot find an obstacle-free place near start point" << std::endl;
        return false;
    }
    if (!GetClosestFreePoint(mvTargetG, 80, mvCandidatesValidTarget)) {
        std::cout << "Cannot find an obstacle-free place near target point" << std::endl;
        return false;
    }
    GridToWorld(mvStartG, mvTmpStartW);
    GridToWorld(mvTargetG, mvTmpTargetW);

    double allocated_time_secs = 100.0; // in seconds
    double initialEpsilon = 3.0;
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = false;

    // Initialize Environment (should be called before initializing anything else)
    EnvironmentNAV2D environment_nav2D;
    if (!environment_nav2D.InitializeEnvORBSLAM2(mObstacles, mvStartG, mvTargetG, 1)) {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }

    // Initialize MDP Info
    if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    // plan a path
    std::vector<int> solution_stateIDs_V;
    SBPLPlanner *planner;
    if (msPlanner == "adstar") {
        planner = new ADPlanner(&environment_nav2D, bforwardsearch);
    }
    else if (msPlanner == "arastar") {
        planner = new ARAPlanner(&environment_nav2D, bforwardsearch);
    }
    else {
        std::cout << "\x1B[31m" << "Input planner is not adstar or arastar: " << planner
                  << "\x1B[0m" << std::endl;
        std::cout << "Using adstar instead ..." << std::endl;
        planner = new ADPlanner(&environment_nav2D, bforwardsearch);
    }

    // set search mode
    planner->set_search_mode(bsearchuntilfirstsolution);

    if (planner->set_start(MDPCfg.startstateid) == 0) {
        throw SBPL_Exception("ERROR: failed to set start state");
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        throw SBPL_Exception("ERROR: failed to set goal state");
    }

    planner->set_initialsolution_eps(initialEpsilon);

    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
//    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    if (mpSolution) {
        mpSolution->clear();
    }
    if (solution_stateIDs_V.size() <= 0) {
        std::cout << "Cannot find a path ... " << std::endl;
    }
    else {
        for (unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
            std::vector<int> pointInGrid(2, 0);
            environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], pointInGrid[0], pointInGrid[1]);
            std::vector<float> pointInWorld;
            GridToWorld(pointInGrid, pointInWorld);
            mpSolution->push_back(pointInWorld);
        }
        bFindPath = true;
        std::cout << "Find a path ..." << std::endl;
    }

    delete planner;

    return bFindPath;
}

bool PathPlanning2D::OmplPathPlanning()
{
    bool bFindPath = false;
    if (mvStartG.size() != 2 || mvTargetG.size() != 2) {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    GenerateOccupancyMap();

    if (!GetClosestFreePoint(mvStartG, 80, mvCandidatesValidStart)) {
        std::cout << "Cannot find an obstacle-free place near start point" << std::endl;
        return false;
    }
    if (!GetClosestFreePoint(mvTargetG, 80, mvCandidatesValidTarget)) {
        std::cout << "Cannot find an obstacle-free place near target point" << std::endl;
        return false;
    }
    GridToWorld(mvStartG, mvTmpStartW);
    GridToWorld(mvTargetG, mvTmpTargetW);
    Plane2DEnvironment env(mObstacles, msPlanner);
    if (mpSolution) {
        mpSolution->clear();
    }
    if (env.Plan(mvStartG, mvTargetG)) {
        std::vector<std::vector<int> > vSolution;
        env.GetSolution(vSolution);
        if (vSolution.size() <= 0) {
            std::cout << "Cannot find a path ... " << std::endl;
        }
        else {
            bFindPath = true;
            std::cout << "Find a path ..." << std::endl;
            for (std::vector<std::vector<int> >::iterator st = vSolution.begin(); st != vSolution.end();
                 st++) //in right order
            {
                std::vector<float> point;
                GridToWorld(*st, point);
                mpSolution->push_back(point);
            }

        }
    }
    else {
        std::cout << "Cannot find a path ... " << std::endl;
    }
    return bFindPath;
}

//Deprecated
void PathPlanning2D::SimplePathPlanning()
{

    if (mvStartG.size() != 2 || mvTargetG.size() != 2) {
        std::cout << "\x1B[31m" << "ERROR: start and target vectors passed to path planning have wrong dimensions"
                  << "\x1B[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
    const int nMaxIter = 10000;

    GenerateOccupancyMap();
    if (!GetClosestFreePoint(mvStartG, 40, mvCandidatesValidStart)) {
        std::cout << "Cannot find an obstacle-free place near start point" << std::endl;
        return;
    }
    if (!GetClosestFreePoint(mvTargetG, 40, mvCandidatesValidTarget)) {
        std::cout << "Cannot find an obstacle-free place near target point" << std::endl;
        return;
    }

    std::vector<std::shared_ptr<RRTNode> > vStartTree;
    std::vector<std::shared_ptr<RRTNode> > vTargetTree;

    vStartTree.push_back(std::make_shared<RRTNode>(mvStartG));
    vTargetTree.push_back(std::make_shared<RRTNode>(mvTargetG));
    int nCount = 0;
    int ncStepSize = 6; //pixel

    std::vector<std::shared_ptr<RRTNode> > vSolution;
    srand(time(NULL));
    bool bFoundPath = false;
    mpSolution->clear();
    while (nCount < nMaxIter)  //max iteration
    {
        nCount++;
        int nMiddleX = rand() % mnSizeX;
        int nMiddleY = rand() % mnSizeY;

        int innerMaxIter = 50;
        for (int iter = 0; iter < innerMaxIter; iter++) {
            if (mObstacles[nMiddleY][nMiddleX] == 0) {
                break;
            }
            nMiddleX = rand() % mnSizeX;
            nMiddleY = rand() % mnSizeY;
            if (iter >= innerMaxIter - 1) {
                std::cout << "\x1B[31m" << "ERROR: cannot generate a collision-free random point"
                          << "\x1B[0m" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        if (!SimpleRRTTreeExpand(vStartTree, mObstacles, nMiddleX, nMiddleY, ncStepSize)) {
            continue;
        };
        if (!SimpleRRTTreeExpand(vTargetTree, mObstacles, nMiddleX, nMiddleY, ncStepSize)) {
            continue;
        };

        if (!SimpleRRTTreesIntersect(vTargetTree, vStartTree, mObstacles, vSolution, ncStepSize)) {
            if (!SimpleRRTTreesIntersect(vStartTree, vTargetTree, mObstacles, vSolution, ncStepSize)) {
                continue;
            }
            std::reverse(vSolution.begin(), vSolution.end()); // reverse vSolution so that it starts from vStartTree
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


        for (std::vector<std::shared_ptr<RRTNode> >::iterator st = vSolution.begin(); st != vSolution.end();
             st++) //in right order
        {
            std::vector<int> tmpPoint = {(*st)->mPoint[0], (*st)->mPoint[1]};
            std::vector<float> point;
            GridToWorld(tmpPoint, point);
            mpSolution->push_back(point);
        }
        break;
    }
    if (bFoundPath) {
        std::cout << "Find a path after " << nCount << " iterations ..." << std::endl;
    }
    else {
        std::cout << "Cannot find a path after " << nMaxIter << " iterations ..." << std::endl;
    }

}

bool PathPlanning2D::GetClosestFreePoint(std::vector<int> &output, int searchWidth,
                                         std::vector<std::pair<float, std::vector<int> > > &candidateOutput)
{
//    if (mObstacles[output[1]][output[0]] == 1) {
//        float heightThreshold = 1.5;
//        std::vector<std::pair<float, std::vector<int> > > vCandidates(8,
//                                                                      std::make_pair(std::numeric_limits<float>::lowest(),
//                                                                                     std::vector<int>(2, 0)));
//        std::vector<bool> vbCandidateInValid(8, false);
//        std::vector<bool> vbCandidateSearchComplete(8, false);
//        for (int i = 1; i < searchWidth + 1; i++) {
//            std::vector<std::vector<int> > vSearchDirection = {{0, i},
//                                                               {i, i},
//                                                               {i, 0},
//                                                               {i, -i},
//                                                               {0, -i},
//                                                               {-i, -i},
//                                                               {-i, 0},
//                                                               {-i, i}};
////            find the minimum of the maximum height along 8 directions
////            if the height is too large, abandon that direction
//            for (int j = 0; j < vSearchDirection.size(); j++) {
//                if (vbCandidateInValid[j] || vbCandidateSearchComplete[j]) {
//                    continue;
//                }
//                int y = output[1] + vSearchDirection[j][0];
//                int x = output[0] + vSearchDirection[j][1];
//                if (x < 0 || x >= mObstacles[0].size() || y < 0 || y >= mObstacles.size())
//                {
//                    vbCandidateInValid[j] = true;
//                    continue;
//                }
//                float currentHeight = mObstaclesHeight[y][x];
//                if (currentHeight >= heightThreshold) {
//                    vbCandidateInValid[j] = true;
//                    continue;
//                }
//                std::pair<float, std::vector<int> >& candidateHeightCoord = vCandidates[j];
//                if (candidateHeightCoord.first < currentHeight) {
//                    candidateHeightCoord.first = currentHeight;
//                }
//                std::vector<int> coord = {y, x};
//                candidateHeightCoord.second = coord;
//                if (mObstacles[coord[0]][coord[1]] == 0) {
//                    vbCandidateSearchComplete[j] = true;
//                }
//            }
//            bool finished = std::all_of(vbCandidateSearchComplete.begin(),
//                                        vbCandidateSearchComplete.end(), [](bool i)
//                                        { return i; });
//            if (finished) {
//                break;
//            }
//
//        }
//        std::vector<std::pair<float, std::vector<int> > > vCandidatesValid;
//        for (int i = 0; i < vCandidates.size(); i++) {
//            if (!vbCandidateInValid[i]) {
//                vCandidatesValid.push_back(vCandidates[i]);
//            }
//        }
//        if (vCandidatesValid.size() > 0) {
//            std::sort(vCandidatesValid.begin(), vCandidatesValid.end(),
//                      [](const std::pair<float, std::vector<int> > &lhs,
//                         const std::pair<float, std::vector<int> > &rhs)
//                      { return lhs.first < rhs.first; });
//            output[0] = vCandidatesValid[0].second[1];
//            output[1] = vCandidatesValid[0].second[0];
//
//            candidateOutput = vCandidatesValid;
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//    return true;

    std::vector<float> originW(2, 0);
    std::vector<int> originInGrid;
    WorldToGrid(originW, originInGrid);
    cv::Point2f originPosCV = cv::Point(originInGrid[0], originInGrid[1]);

    cv::Mat cvObstacles, cvObstaclesFlipped;
    Convert2DVectorToMat(mObstacles, cvObstacles, false);
    cvObstacles.convertTo(cvObstacles, CV_8UC1);
    cvObstacles *= 255;

    // remove high frequency noise with filters
    // Gaussian Filter (faster)
    double sigma = 0.8;
    cv::Mat filteredObstacles;
    cv::GaussianBlur(cvObstacles, filteredObstacles, cv::Size(15, 15), sigma, sigma);
//    // Median Filter
//    cv::medianBlur(cvObstacles, filteredObstacles, 7);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filteredObstacles, contours, hierarchy,
                     cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));
    double distance = std::numeric_limits<double>::max();
    int innerBoundaryId = 0;
    bool bFindInnerContour = false;
    for (int i = 0; i < contours.size(); i++) {
        double inOrOut = cv::pointPolygonTest(contours[i], originPosCV, true);
        if (inOrOut > 0) {
//            std::cout << "contour id: " << i << " dist: " << inOrOut << std::endl;
            if (inOrOut < distance) {
                distance = inOrOut;
                innerBoundaryId = i;
                bFindInnerContour = true;
            }
        }
    }

    float minDistance = std::numeric_limits<float>::max();
    bool bFindFreePoint = false;
    std::vector<int> tmpOutput;
    tmpOutput = output;
    if (mObstacles[output[1]][output[0]] == 1) {
        for (int i = -searchWidth; i <= searchWidth; i++) {
            for (int j = -searchWidth; j <= searchWidth; j++) {
                int y = output[1] + i;
                int x = output[0] + j;
                if (y >= 0 && y < mObstacles.size() && x >= 0 && x < mObstacles[0].size()) {
                    if (mObstacles[y][x] == 0 && (abs(i) + abs(j)) < minDistance) {
                        if (bFindInnerContour)
                        {
                            cv::Point2f currentPosCV = cv::Point(x, y);
                            double inOrOut = cv::pointPolygonTest(contours[innerBoundaryId], currentPosCV, false);
                            if (inOrOut >= 0)
                            {
                                bFindFreePoint = true;
                                minDistance = abs(i) + abs(j);
                                tmpOutput[0] = x;
                                tmpOutput[1] = y;
                            }
                        }
                        else
                        {
                            bFindFreePoint = true;
                            minDistance = abs(i) + abs(j);
                            tmpOutput[0] = x;
                            tmpOutput[1] = y;
                        }
                    }

                }
            }
        }
        output = tmpOutput;
    }
    else {
        bFindFreePoint = true;
    }
    return bFindFreePoint;
}

bool PathPlanning2D::SimpleRRTTreeExpand(std::vector<std::shared_ptr<RRTNode> > &tree,
                                         std::vector<std::vector<int> > &mObstacles,
                                         int nMiddleX, int nMiddleY, int ncStepSize)
{
    int nDisMin = std::numeric_limits<int>::max();
    int nConnect = 1;

    std::shared_ptr<RRTNode> pMin = std::make_shared<RRTNode>(std::vector<int>(2, nDisMin));
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
        std::shared_ptr<RRTNode> tmpNode = std::make_shared<RRTNode>(vTmpPoint);
        tmpNode->addParent(pMin);
        tree.push_back(tmpNode);
        return true;
    }
    else {
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
        if (abs(treePop.back()->mPoint[0] - x) + abs(treePop.back()->mPoint[1] - y) <= 2 * ncStepSize) {
            int nOut = 0;
            for (int i = std::min(treePop.back()->mPoint[0], x); i <= std::max(treePop.back()->mPoint[0], x); i++) {

                for (int j = std::min(treePop.back()->mPoint[1], y); j <= std::max(treePop.back()->mPoint[1], y); j++) {
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
                while (vSolution.back()->mParent) {
                    vSolution.push_back(vSolution.back()->mParent);
                }
                std::reverse(vSolution.begin(), vSolution.end());
                vSolution.push_back(*sit);
                while (vSolution.back()->mParent) {
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
    for (auto p: mCloud->points) {
        if (p.y >= -mfUpperBound && p.y <= -mfLowerBound) {
            if (p.x < mvBounds[0]) {
                mvBounds[0] = p.x;
            }
            if (p.x > mvBounds[1]) {
                mvBounds[1] = p.x;
            }
            if (p.z < mvBounds[2]) {
                mvBounds[2] = p.z;
            }
            if (p.z > mvBounds[3]) {
                mvBounds[3] = p.z;
            }
        }
    }
    if (mvStartW.size() == 2) {
        mvBounds[0] = std::min(mvBounds[0], mvStartW[0]);
        mvBounds[1] = std::max(mvBounds[1], mvStartW[0]);
        mvBounds[2] = std::min(mvBounds[2], mvStartW[1]);
        mvBounds[3] = std::max(mvBounds[3], mvStartW[1]);
    }
    if (mvTargetW.size() == 2) {
        mvBounds[0] = std::min(mvBounds[0], mvTargetW[0]);
        mvBounds[1] = std::max(mvBounds[1], mvTargetW[0]);
        mvBounds[2] = std::min(mvBounds[2], mvTargetW[1]);
        mvBounds[3] = std::max(mvBounds[3], mvTargetW[1]);
    }
    float margin = mfObstacleWidth * mfcGridSize + 0.2;
    mvBounds[0] -= margin;
    mvBounds[1] += margin;
    mvBounds[2] -= margin;
    mvBounds[3] += margin;
}

void PathPlanning2D::CalGridSize()
{
    mnSizeX = static_cast<int> ((mvBounds[1] - mvBounds[0]) / mfcGridSize);
    mnSizeY = static_cast<int> ((mvBounds[3] - mvBounds[2]) / mfcGridSize);
}
void PathPlanning2D::GenerateOccupancyMap()
{
    CalGridSize();
    mObstaclesSeenNum.clear();
    mObstaclesSeenNum = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));
    mObstacles.clear();
    mObstacles = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));
    mLethalObstacles.clear();
    mLethalObstacles = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));
//    mObstaclesHeight.clear();
//    mObstaclesHeight =
//        std::vector<std::vector<float> >(mnSizeY, std::vector<float>(mnSizeX, std::numeric_limits<float>::lowest()));
//    std::vector<std::vector<int> > obstaclesOri(mnSizeY, std::vector<int> (mnSizeX,0 ));
//    for (auto p : mCloud->points) {
//        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcGridSize);
//        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcGridSize);
//        obstaclesOri[nTmpY][nTmpX] = 1;
//    }
//    for (auto p : mCloud->points) {
//        int nTmpX = static_cast<int>((p.x - mvBounds[0]) / mfcGridSize);
//        int nTmpY = static_cast<int>((p.z - mvBounds[2]) / mfcGridSize);
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
    double radiusSquared = pow(mfObstacleWidth, 2.0);
    for (auto p : mCloud->points) {
        std::vector<int> pointInGrid;
        std::vector<float> pointInWorld = {p.x, p.z};
        WorldToGrid(pointInWorld, pointInGrid);
        int nTmpX = pointInGrid[0];
        int nTmpY = pointInGrid[1];
//        if (p.y >= -mfUpperBound && p.y <= -mfLowerBound) {
//            if (mObstaclesHeight[nTmpY][nTmpX] < -p.y) {
//                mObstaclesHeight[nTmpY][nTmpX] = -p.y;
//            }
//        }
//        if (nTmpX >= 0 && nTmpX < mnSizeX && nTmpY >= 0 && nTmpY < mnSizeY)
//        {
//            if (mObstaclesHeight[nTmpY][nTmpX] < -p.y)
//            {
//                mObstaclesHeight[nTmpY][nTmpX] = -p.y;
//            }
//        }
//        else
//        {
//            std::cout << "==============out of bounds " << nTmpX << "  " << nTmpY << std::endl;
//            std::cout << "bounds : " << mnSizeX << "   " << mnSizeY << std::endl;
//        }
//        if (nTmpX >= 0 && nTmpX < mnSizeX && nTmpY >= 0 && nTmpY < mnSizeY)
//        {
//            mObstacles[nTmpY][nTmpX] = 1;
//        }
        if (p.y >= -mfUpperBound && p.y <= -mfLowerBound)  // y axis is pointing downward
        {
            mLethalObstacles[nTmpY][nTmpX] = 1;
        }
        for (int i = nTmpX - mfObstacleWidth; i <= nTmpX + mfObstacleWidth; i++) {
            for (int j = nTmpY - mfObstacleWidth; j <= nTmpY + mfObstacleWidth; j++) {
                if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY) {
                    if (pow(i - nTmpX, 2.0) + pow(j - nTmpY, 2.0) <= radiusSquared) {
                        if (p.y >= -mfUpperBound)  // y axis is pointing downward
                        {
                            if (p.y <= -mfLowerBound) {
                                mObstacles[j][i] = 1;
                            }
                            mObstaclesSeenNum[j][i] += 1;
                        }
                    }

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
//    cv::Mat outputImg(mObstacles.size(), mObstacles[0].size(), CV_8UC3 );
//    for (int i = 0; i < mObstacles.size(); i++)
//    {
//        for (int j = 0; j < mObstacles[0].size(); j++)
//        {
//            if (mObstacles[i][j] == 1)
//            {
//                outputImg.at<cv::Vec3b>(i, j)[0] = 255;
//                outputImg.at<cv::Vec3b>(i, j)[1] = 255;
//                outputImg.at<cv::Vec3b>(i, j)[2] = 255;
//            }
//            else
//            {
//                outputImg.at<cv::Vec3b>(i, j)[0] = 0;
//                outputImg.at<cv::Vec3b>(i, j)[1] = 0;
//                outputImg.at<cv::Vec3b>(i, j)[2] = 0;
//            }
//
//
//        }
//    }
//    cv::imwrite("obastacle.png", outputImg);

}
void PathPlanning2D::WorldToGrid(std::vector<float> &input, std::vector<int> &output)
{
    output.resize(2, 0);
    output[0] = static_cast<int> ((input[0] - mvBounds[0]) / mfcGridSize);
    output[1] = static_cast<int> ((input[1] - mvBounds[2]) / mfcGridSize);
}

void PathPlanning2D::GridToWorld(std::vector<int> &input, std::vector<float> &output)
{
    output.resize(2, 0);
    output[0] = input[0] * mfcGridSize + mvBounds[0];
    output[1] = input[1] * mfcGridSize + mvBounds[2];
}

void PathPlanning2D::Convert2DVectorToMat(std::vector<std::vector<int> > &input,
                                          cv::Mat &output, bool reverseRow)
{
    int rows = input.size();
    int cols = input[0].size();
    cv::Mat m(rows, cols, CV_32S);
    if (reverseRow) {
        for (int i = 0; i < rows; i++) {
            m.row(i) = cv::Mat(input[rows - 1 - i]).t();
        }
    }
    else {
        for (int i = 0; i < rows; i++) {
            m.row(i) = cv::Mat(input[i]).t();
        }
    }
//    m.convertTo(m, CV_8UC1);
    output = m;
}

}

