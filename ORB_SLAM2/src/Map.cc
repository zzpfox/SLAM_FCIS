/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include <boost/filesystem.hpp>

namespace ORB_SLAM2
{
Map::Map()
    : mnMaxKFid(0), mnBigChangeIdx(0)
{
}

Map::Map(cv::FileStorage &fsSettings)
    : mnMaxKFid(0), mnBigChangeIdx(0)
{
    CreateLookup(fsSettings);
    mDepthMapFactor = static_cast<float>(fsSettings["DepthMapFactor"]);
    if (fabs(mDepthMapFactor) < 1e-5)
        mDepthMapFactor = 1;
    else
        mDepthMapFactor = 1.0f / mDepthMapFactor;
}

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(std::shared_ptr<MapPoint> pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(std::shared_ptr<MapPoint> pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    pKF->DeleteDepthImage();
    DeleteSegObjInKeyFrame(pKF);
    mspKeyFrames.erase(pKF);
    // TODO: This only erase the pointer.
    // Delete the KeyFrame
}


void Map::SetReferenceMapPoints(const vector<std::shared_ptr<MapPoint> > &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<std::shared_ptr<KeyFrame> > Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<std::shared_ptr<KeyFrame> >(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<std::shared_ptr<MapPoint> > Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<std::shared_ptr<MapPoint> >(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<std::shared_ptr<MapPoint> > Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mObjectMap.clear();
}

void Map::CreateLookup(cv::FileStorage &fsSettings)
{
    float invfx = 1.0 / float(fsSettings["Camera.fx"]);
    float invfy = 1.0 / float(fsSettings["Camera.fy"]);
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];
    float height = fsSettings["Camera.height"];
    float width = fsSettings["Camera.width"];

    float *it;

    mLookupY = cv::Mat(1, height, CV_32F);
    it = mLookupY.ptr<float>();
    for (size_t r = 0; r < height; ++r, ++it) {
        *it = (r - cy) * invfy;
    }

    mLookupX = cv::Mat(1, width, CV_32F);
    it = mLookupX.ptr<float>();
    for (size_t c = 0; c < width; ++c, ++it) {
        *it = (c - cx) * invfx;
    }
}

void Map::DeleteSegObjInKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexObjectMap);
    for (auto &x:mObjectMap) {
        std::unordered_map<long unsigned int, ObjectPos> &keyFrameMap = x.second;
        keyFrameMap.erase(pKF->mnId);
    }
}

void Map::ShowSegResult()
{
    unique_lock<mutex> lock(mMutexObjectMap);
    boost::filesystem::path path{"./Results"};
    boost::filesystem::create_directories(path);
    std::string segfile("./Results/seg.txt");
    ofstream segFileOut(segfile, ios::out | ios::binary);
    if (segFileOut.is_open()) {
        std::cout << "\x1B[35mPrinting the Hash Maps\x1B[0m" << std::endl;
        for (auto &x:mObjectMap) {
            std::cout << "============================" << std::endl;
            std::cout << "\x1B[36m" << x.first << "\x1B[0m" << ": " << std::endl;
            segFileOut << x.first << ": " << std::endl;
            for (auto &y: x.second) {
                std::cout << "    " << "KeyFrame " << y.first << ": " << std::endl;
                std::cout << y.second << std::endl;
                segFileOut << "    " << "KeyFrame " << y.first << ": " << std::endl;
                segFileOut << y.second << std::endl;
            }
        }
    }
}
} //namespace ORB_SLAM
