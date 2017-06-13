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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <Eigen/Dense>
#include <mutex>
#include <iomanip>
#include <unordered_map>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
    struct ObjectPos {
        std::vector<Eigen::Vector3d> Pcs;

        ObjectPos() {}

        ObjectPos(std::vector<Eigen::Vector3d> pcs) : Pcs(pcs){}

        void addInstance(Eigen::Vector3d &Pc) {
            Pcs.push_back(Pc);
        }


        friend std::ostream &operator<<(std::ostream &os, const ObjectPos &pos) {

            os << std::setw(6) << std::fixed << std::setprecision(3);
            os << "       " << "Pcs: " << std::endl;
            for (auto &Pc: pos.Pcs) {
                os << "       " << "[" << Pc[0] << " " << Pc[1] << " " << Pc[2] << "]" << std::endl;
            }
            os << "       "<< "--------------------------" << std::endl;
            return os;
        }
    };
class Map
{
public:
    Map(cv::FileStorage& fsSettings);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    void showSegResult();


    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // hash map1: class name --> hash map2
    // hash map2: keyframe id --> ObjectPos
    std::unordered_map<std::string, std::unordered_map<long unsigned int, ObjectPos> > mObjectMap;

    std::mutex mMutexObjectMap;
    void CreateLookup(cv::FileStorage& fsSettings);
    cv::Mat mLookupX;
    cv::Mat mLookupY;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
