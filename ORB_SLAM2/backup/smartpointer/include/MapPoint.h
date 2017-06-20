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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include <memory>
#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;

class Map;

class Frame;

class MapPoint: public std::enable_shared_from_this<MapPoint>
{
public:
    MapPoint(const cv::Mat &Pos, std::shared_ptr<KeyFrame> pRefKF, std::shared_ptr<Map> pMap);

    MapPoint(const cv::Mat &Pos, std::shared_ptr<Map> pMap, Frame *pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);

    cv::Mat GetWorldPos();

    cv::Mat GetNormal();

    std::weak_ptr<KeyFrame> GetReferenceKeyFrame();

    std::map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > GetObservations();

    int Observations();

    void AddObservation(std::shared_ptr<KeyFrame> pKF, size_t idx);

    void EraseObservation(std::shared_ptr<KeyFrame> pKF);

    int GetIndexInKeyFrame(std::shared_ptr<KeyFrame> pKF);

    bool IsInKeyFrame(std::shared_ptr<KeyFrame> pKF);

    void SetBadFlag();

    bool isBad();

    void Replace(std::shared_ptr<MapPoint> pMP);

    std::weak_ptr<MapPoint> GetReplaced();

    void IncreaseVisible(int n = 1);

    void IncreaseFound(int n = 1);

    float GetFoundRatio();

    inline int GetFound()
    {
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();

    float GetMaxDistanceInvariance();

    int PredictScale(const float &currentDist, std::shared_ptr<KeyFrame> pKF);

    int PredictScale(const float &currentDist, Frame *pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

protected:

    // Position in absolute coordinates
    cv::Mat mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > mObservations;

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    std::weak_ptr<KeyFrame> mpRefKF;

    // Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    std::weak_ptr<MapPoint> mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    std::weak_ptr<Map> mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H