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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include <memory>
#include <mutex>

namespace ORB_SLAM2
{

class Tracking;

class LoopClosing;

class Map;

class LocalMapping
{
public:
    LocalMapping(std::shared_ptr<Map> pMap, const float bMonocular);

    void SetLoopCloser(std::shared_ptr<LoopClosing> pLoopCloser);

    void SetTracker(std::shared_ptr<Tracking> pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(std::shared_ptr<KeyFrame> pKF);

    // Thread Synch
    void RequestStop();

    void RequestReset();

    bool Stop();

    void Release();

    bool isStopped();

    bool stopRequested();

    bool AcceptKeyFrames();

    void SetAcceptKeyFrames(bool flag);

    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();

    bool isFinished();

    int KeyframesInQueue()
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();

    void ProcessNewKeyFrame();

    void CreateNewMapPoints();

    void MapPointCulling();

    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(std::shared_ptr<KeyFrame> &pKF1, std::shared_ptr<KeyFrame> &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();

    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();

    void SetFinish();

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    std::shared_ptr<Map> mpMap;

    std::shared_ptr<LoopClosing> mpLoopCloser;
    std::shared_ptr<Tracking> mpTracker;

    std::list<std::shared_ptr<KeyFrame> > mlNewKeyFrames;

    std::shared_ptr<KeyFrame> mpCurrentKeyFrame;

    std::list<std::weak_ptr<MapPoint> > mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
