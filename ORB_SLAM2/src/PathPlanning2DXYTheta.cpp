#include "PathPlanning2DXYTheta.h"
#include <algorithm>
#include "Thirdparty/sbpl/src/include/sbpl/headers.h"
#include <boost/math/constants/constants.hpp>
#include "OmplPathPlanning.h"
#include <math.h>
#include <string>
#include <random>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <boost/filesystem.hpp>
namespace ORB_SLAM2
{
PathPlanning2DXYTheta::PathPlanning2DXYTheta(std::string planner, float pointSize, float lineWidth,
                                             float gridSize,
                                             std::string dataFolder,
                                             float robotHalfLength,
                                             float robotHalfWidth,
                                             float upperBound, float lowerBound):
    mfcGridSize(gridSize), msPlanner(planner), mPointSize(pointSize),
    mLineWidth(lineWidth), mbSBPLPathPlan(false),
    mfUpperBound(upperBound), mfLowerBound(lowerBound), msDataFolder(dataFolder),
    mfRobotHalfLength(robotHalfLength), mfRobotHalfWidth(robotHalfWidth)
{
    mfStartYaw = 0.0;
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
        mnCostLethal = 254;
        mnCostInscribed = 253;
        mnCostPossiblyCircumscribedThresh = 128;
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

void PathPlanning2DXYTheta::UpdatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    mCloud = cloud;
}

void PathPlanning2DXYTheta::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    *mCloud += *cloud;
}

void PathPlanning2DXYTheta::reset(bool cleanOccupancyMap)
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
        mObstaclesInflated.clear();
    }

    mvBounds.clear();
    mvCandidatesValidStart.clear();
    mvCandidatesValidTarget.clear();
}

bool PathPlanning2DXYTheta::PlanPath(std::vector<float> &start,
                                     std::vector<float> &target,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (mpSolution) {
        mpSolution->clear();
    }

    UpdatePointCloud(cloud);
    if (start.size() == 3)
    {
        mfStartYaw = start[2];
    }
    else
    {
        mfStartYaw = 0;
    }
    std::cout << "Start point yaw angle: " << mfStartYaw << std::endl;
    mvStartW = std::vector<float> (start.begin(), start.begin() + 2);
    mvTargetW = std::vector<float> (target.begin(), target.begin() + 2);
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

bool PathPlanning2DXYTheta::PlanPath(std::vector<float> &start,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (mpSolution) {
        mpSolution->clear();
    }
    UpdatePointCloud(cloud);
    if (start.size() == 3)
    {
        mfStartYaw = start[2];
    }
    else
    {
        mfStartYaw = 0;
    }
    std::cout << "Start point yaw angle: " << mfStartYaw << std::endl;
    mvStartW = std::vector<float> (start.begin(), start.begin() + 2);
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

bool PathPlanning2DXYTheta::UnvisitedAreasToGo(std::vector<float> &currentPos,
                                               std::vector<float> &target,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
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
    Convert2DVectorToMat(mObstaclesInflated, cvObstacles, false);
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
    std::string BlurredImage(msContourFolder + "/GaussianBlur.png");
    cv::imwrite(BlurredImage, filteredObstacles);
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
    cv::RNG rng(12);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(contourImage, contours, (int) i, color, 2, 8, hierarchy, 0, cv::Point());
    }
    cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
    cv::flip(contourImage, contourImageFlipped, 0);
    std::string AllContoursImage(msContourFolder + "/AllContours.png");
    cv::imwrite(AllContoursImage, contourImageFlipped);

    contourImage = cv::Mat::zeros(cvObstacles.size(), CV_8UC3);
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
    int areaThresh = 1600;

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
        while (mObstaclesInflated[randomInterestedPixel[1]][randomInterestedPixel[0]] == 1) {
            randomInterestedPixel = interestedPixels[dis(gen)];
        }
        GridToWorld(randomInterestedPixel, target);
        return true;
    }
    else {
        return false;
    }

}

bool PathPlanning2DXYTheta::UnvisitedAreasToGo(std::vector<float> &target,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
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
    Convert2DVectorToMat(mObstaclesInflated, cvObstacles, false);
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
    std::string BlurredImage(msContourFolder + "/GaussianBlur.png");
    cv::imwrite(BlurredImage, filteredObstacles);
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
    cv::RNG rng(1234);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(contourImage, contours, (int) i, color, 2, 8, hierarchy, 0, cv::Point());
    }
    cv::circle(contourImage, currentPosCV, 5, cv::Scalar(0, 255, 0), -1);
    cv::flip(contourImage, contourImageFlipped, 0);
    std::string AllContoursImage(msContourFolder + "/AllContours.png");
    cv::imwrite(AllContoursImage, contourImageFlipped);

    contourImage = cv::Mat::zeros(cvObstacles.size(), CV_8UC3);
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
    int areaThresh = 1600;

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
        while (mObstaclesInflated[randomInterestedPixel[1]][randomInterestedPixel[0]] == 1) {
            randomInterestedPixel = interestedPixels[dis(gen)];
        }
        GridToWorld(randomInterestedPixel, target);
        return true;
    }
    else {
        return false;
    }

}

std::vector<float> PathPlanning2DXYTheta::GetTargetW()
{
    return mvTargetW;
}

void PathPlanning2DXYTheta::ShowPlannedPath()
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
        if (mpSolution->size() > 1 && mvStartW.size() == 2 && mvTargetW.size() == 2) {
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

    for (int i = 0; i < mObstacles.size(); i++) {
        for (int j = 0; j < mObstacles[0].size(); j++) {
            if (mObstacles[i][j] == 1 || (mbSBPLPathPlan && mObstacles[i][j] == mnCostLethal)) {
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
    for (int i = 0; i < mObstaclesInflated.size(); i++) {
        for (int j = 0; j < mObstaclesInflated[0].size(); j++) {
            if (mObstaclesInflated[i][j] == 1) {
                std::vector<float> tmpPoint;
                std::vector<int> tmpSrc = {j, i};
                GridToWorld(tmpSrc, tmpPoint);
                glVertex3f(tmpPoint[0], 5, tmpPoint[1]);
            }
        }
    }
    glEnd();
}

bool PathPlanning2DXYTheta::SBPLPathPlanning()
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

    float deltaX = mvTargetW[0] - mvTmpTargetW[0];
    float deltaY = mvTargetW[1] - mvTmpTargetW[1];
    float targetYaw = 0;//atan2(deltaY, deltaX);

    double allocated_time_secs = 100.0; // in seconds
    double initialEpsilon = 3.0;
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = true;

    std::vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt_m;
    double halfwidth = mfRobotHalfWidth / mfcGridSize;
    double halflength = mfRobotHalfLength / mfcGridSize;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);

    int mapSize = mObstacles.size() * mObstacles[0].size();
    unsigned char* map = new unsigned char[mapSize];
    for (int i = 0; i < mObstacles.size(); i++)
    {
        for (int j = 0; j < mObstacles[0].size(); j++)
        {
            map[i + j * mObstacles.size()] = mObstacles[i][j];
        }
    }
    EnvNAVXYTHETALAT_InitParms params;
    double startx = mvStartG[1];
    double starty = mvStartG[0];
    double startTheta = mfStartYaw;
    double goalx = mvTargetG[1];
    double goaly = mvTargetG[0];
    double goalTheta = targetYaw;
    params.startx = startx;
    params.starty = starty;
    params.starttheta = startTheta;
    params.goalx = goalx;
    params.goaly = goaly;
    params.goaltheta = goalTheta;
    params.goaltol_x = 1.0;
    params.goaltol_y = 1.0;
    params.goaltol_theta = 0.1;
    params.mapdata = map;
    params.numThetas = NAVXYTHETALAT_THETADIRS;

    // in EnvironmentNAVXYTHETALAT, x aligned with the heading of the robot, angles are positive
    EnvironmentNAVXYTHETALAT environment_navxythetalat;
    double dNominalVelMPerSecs = 0.1;
    double dTimeToTurn45DegsInPlaceSecs = 5;
    unsigned char obsthresh = mnCostLethal;
    char *sMotPrimFile = "../../../Thirdparty/sbpl/mprim/slxrobot.mprim";

    // set robot environment parameters (should be done before initialize function is called)
    if (!environment_navxythetalat.SetEnvParameter("cost_inscribed_thresh", mnCostInscribed)) {
        throw SBPL_Exception("ERROR: failed to set parameters");
    }
    if (!environment_navxythetalat.SetEnvParameter("cost_possibly_circumscribed_thresh", mnCostPossiblyCircumscribedThresh)) {
        throw SBPL_Exception("ERROR: failed to set parameters");
    }

    bool envInitialized = environment_navxythetalat.InitializeEnv(mObstacles.size(), mObstacles[0].size(),
                                                                  perimeterptsV,
                                                                  1.0,
                                                                  dNominalVelMPerSecs,
                                                                  dTimeToTurn45DegsInPlaceSecs,
                                                                  obsthresh,
                                                                  sMotPrimFile,
                                                                  params);
    if (!envInitialized)
    {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }

    environment_navxythetalat.SetStart(startx, starty, startTheta);
    environment_navxythetalat.SetGoal(goalx, goaly, goalTheta);

    // initialize MDP info
    if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    // plan a path
    std::vector<int> solution_stateIDs_V;
    SBPLPlanner *planner;
    if (msPlanner == "adstar") {
        planner = new ADPlanner(&environment_navxythetalat, bforwardsearch);
    }
    else if (msPlanner == "arastar") {
        planner = new ARAPlanner(&environment_navxythetalat, bforwardsearch);
    }
    else {
        std::cout << "\x1B[31m" << "Input planner is not adstar or arastar: " << planner
                  << "\x1B[0m" << std::endl;
        std::cout << "Using adstar instead ..." << std::endl;
        planner = new ADPlanner(&environment_navxythetalat, bforwardsearch);
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
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    if (mpSolution) {
        mpSolution->clear();
    }
    if (solution_stateIDs_V.size() <= 0) {
        std::cout << "Cannot find a path ... " << std::endl;
    }
    else {
        for (unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
            std::vector<int> pointInGrid(3, 0);
            environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[i],
                                                        pointInGrid[1],
                                                        pointInGrid[0],
                                                        pointInGrid[2]);
            std::vector<float> pointInWorld;
            GridToWorld(pointInGrid, pointInWorld);
            pointInWorld.push_back(pointInGrid[2] * 2.0 *
                boost::math::constants::pi<float>() / NAVXYTHETALAT_THETADIRS);
            mpSolution->push_back(pointInWorld);

        }
        bFindPath = true;
        std::cout << "Find a path ..." << std::endl;
    }

    delete planner;
    delete[] map;

    return bFindPath;
}

bool PathPlanning2DXYTheta::OmplPathPlanning()
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

    float deltaX = mvTargetW[0] - mvTmpTargetW[0];
    float deltaY = mvTargetW[1] - mvTmpTargetW[1];
    float targetYaw = atan2(deltaY, deltaX);

    int robotHalfLengthGrids = mfRobotHalfLength / mfcGridSize;
    int robotHalfWidthGrids = mfRobotHalfWidth / mfcGridSize;
    Plane2DXYThetaEnvironment env(mObstacles, msPlanner,
                                  robotHalfLengthGrids,
                                  robotHalfWidthGrids);
    if (mpSolution) {
        mpSolution->clear();
    }
    if (env.Plan(mvStartG, mvTargetG, mfStartYaw, targetYaw)) {
        std::vector<std::vector<float> > vSolution;
        env.GetSolution(vSolution);
        if (vSolution.size() <= 0) {
            std::cout << "Cannot find a path ... " << std::endl;
        }
        else {
            bFindPath = true;
            std::cout << "Find a path ..." << std::endl;
            for (std::vector<std::vector<float> >::iterator st = vSolution.begin(); st != vSolution.end();
                 st++) //in right order
            {
                std::vector<int> pointInGrid(st->begin(), st->begin() + 2);
                std::vector<float> pointInWorld;
                GridToWorld(pointInGrid, pointInWorld);
                pointInWorld.push_back((*st)[2]);
                mpSolution->push_back(pointInWorld);
            }

        }
    }
    else {
        std::cout << "Cannot find a path ... " << std::endl;
    }
    return bFindPath;
}


bool PathPlanning2DXYTheta::GetClosestFreePoint(std::vector<int> &output, int searchWidth,
                                         std::vector<std::pair<float, std::vector<int> > > &candidateOutput)
{
    bool bFindFreePoint = false;
    if (mObstaclesInflated[output[1]][output[0]] == 1) {
        std::vector<float> originW(2, 0);
        std::vector<int> originInGrid;
        WorldToGrid(originW, originInGrid);
        cv::Point2f originPosCV = cv::Point(originInGrid[0], originInGrid[1]);

        cv::Mat cvObstacles, cvObstaclesFlipped;
        Convert2DVectorToMat(mObstaclesInflated, cvObstacles, false);
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
        if (bFindInnerContour)
        {
            std::cout << "find innermost contour " << std::endl;
        }
        else
        {
            std::cout << "cannot find innermost contour " << std::endl;
        }

        float minDistance = std::numeric_limits<float>::max();

        std::vector<int> tmpOutput;
        tmpOutput = output;
        float robotDiagonal = sqrt(pow(mfRobotHalfWidth, 2.0) +
            pow(mfRobotHalfLength, 2.0)) / mfcGridSize * 2;
        for (int i = -searchWidth; i <= searchWidth; i++) {
            for (int j = -searchWidth; j <= searchWidth; j++) {
                int y = output[1] + i;
                int x = output[0] + j;
                if (y >= 0 && y < mObstaclesInflated.size() && x >= 0 && x < mObstaclesInflated[0].size()) {
                    if (mObstaclesInflated[y][x] == 0 && (abs(i) + abs(j)) < minDistance) {
                        if (bFindInnerContour)
                        {
                            cv::Point2f currentPosCV = cv::Point(x, y);
                            double inOrOut = cv::pointPolygonTest(contours[innerBoundaryId], currentPosCV, true);
                            if (inOrOut >= robotDiagonal)
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

void PathPlanning2DXYTheta::Get2DBounds()
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
    float margin = sqrt(pow(mfRobotHalfWidth + 0.2, 2.0) + pow(mfRobotHalfLength + 0.2, 2.0)) + 0.2;
    mvBounds[0] -= margin;
    mvBounds[1] += margin;
    mvBounds[2] -= margin;
    mvBounds[3] += 3 * margin;
}

void PathPlanning2DXYTheta::CalGridSize()
{
    mnSizeX = static_cast<int> ((mvBounds[1] - mvBounds[0]) / mfcGridSize);
    mnSizeY = static_cast<int> ((mvBounds[3] - mvBounds[2]) / mfcGridSize);
}
void PathPlanning2DXYTheta::GenerateOccupancyMap()
{
    CalGridSize();
    mObstaclesSeenNum.clear();
    mObstaclesSeenNum = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));
    mObstacles.clear();
    mObstacles = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));

    mObstaclesInflated.clear();
    mObstaclesInflated = std::vector<std::vector<int> >(mnSizeY, std::vector<int>(mnSizeX, 0));
    if (mbSBPLPathPlan)
    {
        // http://wiki.ros.org/costmap_2d
        int costLethal = 254;
        int costInscribed = 253;
        int costPossiblyCircumscribedThresh = 128;
        double inscribedRadiusSquared = pow(std::min(mfRobotHalfLength, mfRobotHalfWidth) / mfcGridSize, 2.0);
        double circumscribedRadiusSquared = pow(mfRobotHalfWidth / mfcGridSize, 2.0) +
            pow(mfRobotHalfLength / mfcGridSize, 2.0);
        double circumscribedRaidus = sqrt(circumscribedRadiusSquared);
        for (auto p : mCloud->points) {
            std::vector<int> pointInGrid;
            std::vector<float> pointInWorld = {p.x, p.z};
            WorldToGrid(pointInWorld, pointInGrid);
            int nTmpX = pointInGrid[0];
            int nTmpY = pointInGrid[1];
            if (p.y >= -mfUpperBound && p.y <= -mfLowerBound)  // y axis is pointing downward
            {
                mObstacles[nTmpY][nTmpX] = costLethal;
            }
            for (int i = nTmpX - circumscribedRaidus; i <= nTmpX + circumscribedRaidus; i++) {
                for (int j = nTmpY - circumscribedRaidus; j <= nTmpY + circumscribedRaidus; j++) {
                    if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY) {
                        double ToCenterDistanceSquared = pow(i - nTmpX, 2.0) + pow(j - nTmpY, 2.0);
                        if (ToCenterDistanceSquared <= circumscribedRadiusSquared) {
                            if (p.y >= -mfUpperBound)  // y axis is pointing downward
                            {
                                if (p.y <= -mfLowerBound) {
                                    mObstaclesInflated[j][i] = 1;
                                }
                                mObstaclesSeenNum[j][i] += 1;
                            }
                        }
                        if (p.y >= -mfUpperBound && p.y <= -mfLowerBound)  // y axis is pointing downward
                        {
                            if (ToCenterDistanceSquared <= inscribedRadiusSquared) {
                                mObstacles[j][i] = std::max(mObstacles[j][i], costInscribed);
                            }
                            else if(ToCenterDistanceSquared <= circumscribedRadiusSquared)
                            {
                                mObstacles[j][i] = std::max(mObstacles[j][i], costPossiblyCircumscribedThresh);
                            }
                        }
                    }

                }
            }
        }

    }
    else
    {
        double smallRadiusSquared = pow(2, 2.0); // inflated the obstacles a little bit to remove white noise
        double largeRadiusSquared = pow(mfRobotHalfWidth / mfcGridSize + 1, 2.0) + pow(mfRobotHalfLength / mfcGridSize + 1, 2.0);
//        double largeRadiusSquared = pow((std::min(mfRobotHalfWidth, mfRobotHalfLength)) / mfcGridSize, 2.0);
        double largeRadius = sqrt(largeRadiusSquared);
        for (auto p : mCloud->points) {
            std::vector<int> pointInGrid;
            std::vector<float> pointInWorld = {p.x, p.z};
            WorldToGrid(pointInWorld, pointInGrid);
            int nTmpX = pointInGrid[0];
            int nTmpY = pointInGrid[1];
            for (int i = nTmpX - largeRadius; i <= nTmpX + largeRadius; i++) {
                for (int j = nTmpY - largeRadius; j <= nTmpY + largeRadius; j++) {
                    if (i >= 0 && i < mnSizeX && j >= 0 && j < mnSizeY) {
                        if (pow(i - nTmpX, 2.0) + pow(j - nTmpY, 2.0) <= largeRadiusSquared) {
                            if (p.y >= -mfUpperBound)  // y axis is pointing downward
                            {
                                if (p.y <= -mfLowerBound) {
                                    mObstaclesInflated[j][i] = 1;
                                }
                                mObstaclesSeenNum[j][i] += 1;
                            }
                        }
                        if (pow(i - nTmpX, 2.0) + pow(j - nTmpY, 2.0) <= smallRadiusSquared) {
                            if (p.y >= -mfUpperBound && p.y <= -mfLowerBound)  // y axis is pointing downward
                            {
                                mObstacles[j][i] = 1;
                            }
                        }

                    }

                }
            }
        }
    }



}
void PathPlanning2DXYTheta::WorldToGrid(std::vector<float> &input, std::vector<int> &output)
{
    output.resize(2, 0);
    output[0] = static_cast<int> ((input[0] - mvBounds[0]) / mfcGridSize);
    output[1] = static_cast<int> ((input[1] - mvBounds[2]) / mfcGridSize);
}

void PathPlanning2DXYTheta::GridToWorld(std::vector<int> &input, std::vector<float> &output)
{
    output.resize(2, 0);
    output[0] = input[0] * mfcGridSize + mvBounds[0];
    output[1] = input[1] * mfcGridSize + mvBounds[2];
}

void PathPlanning2DXYTheta::Convert2DVectorToMat(std::vector<std::vector<int> > &input,
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
