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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId = 0;

mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
    mnFirstKFid(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mfMinDistance(0), mfMaxDistance(0)
{
}

MapPoint::MapPoint(const cv::Mat &Pos, std::shared_ptr<KeyFrame> pRefKF, std::shared_ptr<Map> pMap)
    :
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap.lock()->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, std::shared_ptr<Map> pMap, Frame *pFrame, const int &idxF)
    :
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1),
    mnFound(1), mbBad(false), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap.lock()->mMutexPointCreation);
    mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

std::weak_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(std::shared_ptr<KeyFrame> pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    std::weak_ptr<KeyFrame> wpKF(pKF);
    if (mObservations.count(wpKF))
        return;
    mObservations[pKF] = idx;

    if (pKF->mvuRight[idx] >= 0)
        nObs += 2;
    else
        nObs++;
}

void MapPoint::EraseObservation(std::shared_ptr<KeyFrame> pKF)
{
    bool bBad = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        std::weak_ptr<KeyFrame> wpKF(pKF);
        if (mObservations.count(wpKF)) {
            int idx = mObservations[wpKF];
            if (pKF->mvuRight[idx] >= 0)
                nObs -= 2;
            else
                nObs--;

            mObservations.erase(wpKF);
            if (mpRefKF.expired())
            {
                mpRefKF = mObservations.begin()->first;
            }
            else
            {
                if (mpRefKF.lock() == pKF)
                {
                    if (mObservations.size() > 0)
                    {
                        mpRefKF = mObservations.begin()->first;
                    }
                    else
                    {
                        mpRefKF.reset();
                    }

                }
            }


            // If only 2 observations or less, discard point
            if (nObs <= 2)
                bBad = true;
        }
    }

    if (bBad)
        SetBadFlag();
}

map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
        mpRefKF.reset();
        mpReplaced.reset();
    }
    for (map<std::weak_ptr<KeyFrame>, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        spKF->EraseMapPointMatch(mit->second);
    }
    obs.clear();
    if (mpMap.expired())
    {
        throw std::string("Map died!!!");
    }
    mpMap.lock()->EraseMapPoint(shared_from_this());
}

std::weak_ptr<MapPoint> MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(std::shared_ptr<MapPoint> pMP)
{
    if (pMP->mnId == shared_from_this()->mnId)
        return;

    int nvisible, nfound;
    map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for (map<std::weak_ptr<KeyFrame>, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
        // Replace measurement in keyframe
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        if (!pMP->IsInKeyFrame(spKF)) {
            spKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(spKF, mit->second);
        }
        else {
            spKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();
    if (mpMap.expired())
    {
        throw std::string("Map died!!!");
    }
    mpMap.lock()->EraseMapPoint(shared_from_this());
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible += n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound += n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if (mbBad)
            return;
        observations = mObservations;
    }

    if (observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for (map<std::weak_ptr<KeyFrame>, size_t>::iterator mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        if (!spKF->isBad())
            vDescriptors.push_back(spKF->mDescriptors.row(mit->second));
    }

    if (vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for (size_t i = 0; i < N; i++) {
        Distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++) {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            Distances[i][j] = distij;
            Distances[j][i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++) {
        vector<int> vDists(Distances[i], Distances[i] + N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[0.5 * (N - 1)];

        if (median < BestMedian) {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    std::weak_ptr<KeyFrame> wpKF(pKF);
    if (mObservations.count(wpKF))
        return mObservations[wpKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    std::weak_ptr<KeyFrame> wpKF(pKF);
    return (mObservations.count(wpKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations;
    std::weak_ptr<KeyFrame> pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if (mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos.clone();
    }

    if (observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
    int n = 0;
    for (map<std::weak_ptr<KeyFrame>, size_t>::iterator mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        cv::Mat Owi = spKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali / cv::norm(normali);
        n++;
    }
    if (pRefKF.expired())
        return;
    std::shared_ptr<KeyFrame> spRefKF = pRefKF.lock();
    cv::Mat PC = Pos - spRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = spRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = spRefKF->mvScaleFactors[level];
    const int nLevels = spRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / spRefKF->mvScaleFactors[nLevels - 1];
        mNormalVector = normal / n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, std::shared_ptr<KeyFrame> pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels - 1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame *pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pF->mnScaleLevels)
        nScale = pF->mnScaleLevels - 1;

    return nScale;
}

void MapPoint::SetMap(std::shared_ptr<Map> map)
{
    mpMap = map;
}

} //namespace ORB_SLAM
