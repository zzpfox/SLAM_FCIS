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

#include "KeyFrame.h"
#include "Converter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ORBmatcher.h"
#include "Serialization.h"
#include<mutex>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <chrono>
#include <ctime>
namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId = 0;

std::string KeyFrame::msDepthImagesFolder = "./DepthImages";

cv::Mat KeyFrame::mK;

float KeyFrame::cx, KeyFrame::cy, KeyFrame::fx, KeyFrame::fy, KeyFrame::invfx, KeyFrame::invfy;

float KeyFrame::mbf, KeyFrame::mb, KeyFrame::mThDepth, KeyFrame::mHalfBaseline;

float KeyFrame::mfGridElementWidthInv, KeyFrame::mfGridElementHeightInv;

int KeyFrame::mnMinX, KeyFrame::mnMinY, KeyFrame::mnMaxX, KeyFrame::mnMaxY;

int KeyFrame::mnGridCols, KeyFrame::mnGridRows;

std::shared_ptr<ORBVocabulary> KeyFrame::mpORBvocabulary;

int KeyFrame::mnScaleLevels;
float KeyFrame::mfScaleFactor;
float KeyFrame::mfLogScaleFactor;
vector<float> KeyFrame::mvScaleFactors;
vector<float> KeyFrame::mvLevelSigma2;
vector<float> KeyFrame::mvInvLevelSigma2;

KeyFrame::KeyFrame(Frame &F, std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrameDatabase> pKFDB)
    :
    mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mpKeyFrameDB(pKFDB),
    mbFirstConnection(true), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mpMap(pMap)
{
    mnId = nNextId++;
    mGrid.resize(mnGridCols);
    for (int i = 0; i < mnGridCols; i++) {
        mGrid[i].resize(mnGridRows);
        for (int j = 0; j < mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }
    std::vector<std::shared_ptr<MapPoint> > svpMapPoints = F.mvpMapPoints;
    mvpMapPoints.clear();
    mvpMapPoints.reserve(svpMapPoints.size());
    for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpMapPoints.begin();
         it != svpMapPoints.end(); ++it)
    {
        std::weak_ptr<MapPoint> wit = *it;
        mvpMapPoints.push_back(wit);
    }
    SetPose(F.mTcw);
    cv::Mat imDepth;
    if (mpMap.expired())
    {
        throw std::string("Map died!!!");
    }
    std::shared_ptr<Map> smpMap = mpMap.lock();
//    F.mImDepth.convertTo(imDepth, CV_16U, 1.0 / mpMap.lock()->mDepthMapFactor);
    std::stringstream ss;
    ss << msDepthImagesFolder << "/KeyFrame-" << std::setw(8) << std::setfill('0') << mnId << ".bin";
    mPointCloudName = ss.str();
//    cv::imwrite(mDepthImageName.c_str(), imDepth);

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points;
    pcl::PointXYZ point;
    int imgStep = 3;
    for (int r = 0; r < F.mImDepth.rows; r += imgStep) {
        const float *itD = F.mImDepth.ptr<float>(r);
        const float y = smpMap->mLookupY.at<float>(0, r);
        const float *itX = smpMap->mLookupX.ptr<float>();
        for (size_t c = 0; c < (size_t) F.mImDepth.cols; c += imgStep, itD += imgStep, itX += imgStep) {
            float depthValue = *itD;
            if (depthValue > 0.1 && depthValue < 12.0) {
                float zc = depthValue;
                float xc = *itX * depthValue;
                float yc = y * depthValue;
                point.x = xc;
                point.y = yc;
                point.z = zc;
                points.push_back(point);
            }
        }
    }

    std::ofstream os(mPointCloudName);
    {
        boost::archive::binary_oarchive oa(os, boost::archive::no_header);
        oa << points;
    }

}

KeyFrame::KeyFrame():
    mnFrameId(0), mTimeStamp(0.0),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    N(0), mbFirstConnection(false), mbNotErase(true),
    mbToBeErased(false), mbBad(false)
{
}

void KeyFrame::ComputeBoW()
{
    if (mBowVec.empty() || mFeatVec.empty()) {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc * tcw;

    Twc = cv::Mat::eye(4, 4, Tcw.type());
    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(Twc.rowRange(0, 3).col(3));
    cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
    Cw = Twc * center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame> pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        std::weak_ptr<KeyFrame> pKFw(pKF);
        if (!mConnectedKeyFrameWeights.count(pKFw))
            mConnectedKeyFrameWeights[pKFw] = weight;
        else if (mConnectedKeyFrameWeights[pKFw] != weight)
            mConnectedKeyFrameWeights[pKFw] = weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int, std::shared_ptr<KeyFrame> > > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for (map<std::weak_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
         mit != mend; mit++)
    {
        if (mit->first.expired())
            continue;
        vPairs.push_back(make_pair(mit->second, mit->first.lock()));
    }


    sort(vPairs.begin(), vPairs.end());
    list<std::weak_ptr<KeyFrame> > lKFs;
    list<int> lWs;
    for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
        std::weak_ptr<KeyFrame> wvPairi = vPairs[i].second;
        lKFs.push_front(wvPairi);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<std::weak_ptr<KeyFrame> >(lKFs.begin(), lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > s;
    for (map<std::weak_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin();
         mit != mConnectedKeyFrameWeights.end(); mit++)
        s.insert(mit->first);
    return s;
}

vector<std::weak_ptr<KeyFrame> > KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<std::weak_ptr<KeyFrame> > KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if ((int) mvpOrderedConnectedKeyFrames.size() < N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<std::weak_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

}

vector<std::weak_ptr<KeyFrame> > KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if (mvpOrderedConnectedKeyFrames.empty())
        return vector<std::weak_ptr<KeyFrame> >();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                           KeyFrame::weightComp);
    if (it == mvOrderedWeights.end())
        return vector<std::weak_ptr<KeyFrame> >();
    else {
        int n = it - mvOrderedWeights.begin();
        return vector<std::weak_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
    }
}

int KeyFrame::GetWeight(std::weak_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(std::shared_ptr<MapPoint> pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx].reset();
}

void KeyFrame::EraseMapPointMatch(std::shared_ptr<MapPoint> pMP)
{
    int idx = pMP->GetIndexInKeyFrame(shared_from_this());
    if (idx >= 0)
        mvpMapPoints[idx].reset();
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<MapPoint> pMP)
{
    mvpMapPoints[idx] = pMP;
}

set<std::weak_ptr<MapPoint>, std::owner_less<std::weak_ptr<MapPoint> > > KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<std::weak_ptr<MapPoint>, std::owner_less<std::weak_ptr<MapPoint> > > s;
    for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
        if (mvpMapPoints[i].expired())
            continue;
        std::shared_ptr<MapPoint> pMP = mvpMapPoints[i].lock();
        if (!pMP->isBad())
            s.insert(mvpMapPoints[i]);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints = 0;
    const bool bCheckObs = minObs > 0;
    for (int i = 0; i < N; i++) {
        std::weak_ptr<MapPoint> pMP = mvpMapPoints[i];
        if (!pMP.expired()) {
            std::shared_ptr<MapPoint> spMP = pMP.lock();
            if (!spMP->isBad()) {
                if (bCheckObs) {
                    if (spMP->Observations() >= minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<std::weak_ptr<MapPoint> > KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

std::weak_ptr<MapPoint> KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<std::weak_ptr<KeyFrame>, int, std::owner_less<std::weak_ptr<KeyFrame> > > KFcounter;

    vector<std::weak_ptr<MapPoint> > vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for (vector<std::weak_ptr<MapPoint> >::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
        std::weak_ptr<MapPoint> pMP = *vit;

        if (pMP.expired())
            continue;
        std::shared_ptr<MapPoint> spMP = pMP.lock();
        if (spMP->isBad())
            continue;

        map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations = spMP->GetObservations();

        for (map<std::weak_ptr<KeyFrame>, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            if (mit->first.expired() || (!mit->first.expired() && mit->first.lock()->mnId == mnId))
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if (KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax = 0;
    std::weak_ptr<KeyFrame> pKFmax;
    int th = 15;

    vector<pair<int, std::shared_ptr<KeyFrame> > > vPairs;
    vPairs.reserve(KFcounter.size());
    for (map<std::weak_ptr<KeyFrame>, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
        if (mit->first.expired())
            continue;
        std::shared_ptr<KeyFrame> spKeyFrame = mit->first.lock();
        if (mit->second > nmax) {
            nmax = mit->second;
            pKFmax = mit->first;
        }
        if (mit->second >= th) {
            vPairs.push_back(make_pair(mit->second, mit->first.lock()));
            spKeyFrame->AddConnection(shared_from_this(), mit->second);
        }
    }

    if (vPairs.empty() && !pKFmax.expired()) {
        std::shared_ptr<KeyFrame> spKeyFrame = pKFmax.lock();
        vPairs.push_back(make_pair(nmax, spKeyFrame));
        spKeyFrame->AddConnection(shared_from_this(), nmax);
    }

    sort(vPairs.begin(), vPairs.end());
    list<std::weak_ptr<KeyFrame> > lKFs;
    list<int> lWs;
    for (size_t i = 0; i < vPairs.size(); i++) {
        std::weak_ptr<KeyFrame> wvPairi = vPairs[i].second;
        lKFs.push_front(wvPairi);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<std::weak_ptr<KeyFrame> >(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if (mbFirstConnection && mnId != 0) {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            std::shared_ptr<KeyFrame> spParent = mpParent.lock();
            spParent->AddChild(shared_from_this());
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    std::weak_ptr<KeyFrame> wpKF(pKF);
    mspChildrens.insert(wpKF);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    std::weak_ptr<KeyFrame> wpKF(pKF);
    mspChildrens.erase(wpKF);
}

void KeyFrame::ChangeParent(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(shared_from_this());
}

std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

std::weak_ptr<KeyFrame> KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(std::weak_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(std::weak_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mspLoopEdges.empty()) {
            mbNotErase = false;
        }
    }

    if (mbToBeErased) {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mnId == 0)
            return;
        else if (mbNotErase) {
            mbToBeErased = true;
            return;
        }
    }

    for (map<std::weak_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
         mit != mend; mit++)
    {
        if (mit->first.expired())
            continue;
        mit->first.lock()->EraseConnection(shared_from_this());
    }


    for (size_t i = 0; i < mvpMapPoints.size(); i++)
    {
        if (!mvpMapPoints[i].expired())
            mvpMapPoints[i].lock()->EraseObservation(shared_from_this());
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while (!mspChildrens.empty()) {
            bool bContinue = false;

            int max = -1;
            std::shared_ptr<KeyFrame> pC;
            std::shared_ptr<KeyFrame> pP;

            for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
                 sit != send; sit++) {
                if (sit->expired())
                    continue;
                std::shared_ptr<KeyFrame> pKF = sit->lock();
                if (pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<std::weak_ptr<KeyFrame> > vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                    for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                         spcit != spcend; spcit++) {
                        if (vpConnected[i].expired())
                            continue;
                        if (spcit->expired())
                            continue;
                        std::shared_ptr<KeyFrame> spvpConnectedI = vpConnected[i].lock();
                        if (spvpConnectedI->mnId == spcit->lock()->mnId) {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if (w > max) {
                                pC = pKF;
                                pP = spvpConnectedI;
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if (bContinue) {
                pC->ChangeParent(pP);
                std::weak_ptr<KeyFrame> wpC(pC);
                sParentCandidates.insert(wpC);
                mspChildrens.erase(wpC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if (!mspChildrens.empty())
            for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                if (sit->expired() || mpParent.expired())
                    continue;
                sit->lock()->ChangeParent(mpParent.lock());
            }
        if (!mpParent.expired())
        {
            std::shared_ptr<KeyFrame> spParent = mpParent.lock();
            spParent->EraseChild(shared_from_this());
            mTcp = Tcw * spParent->GetPoseInverse();
            mbBad = true;
        }

    }
    mpKeyFrameDB->erase(shared_from_this());
    if (mpMap.expired())
    {
        throw std::string("Map died!!!");
    }
    mpMap.lock()->EraseKeyFrame(shared_from_this());

}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(std::shared_ptr<KeyFrame> pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF)) {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate = true;
        }
    }

    if (bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
    if (nMinCellX >= mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
    if (nMinCellY >= mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
    if (nMaxCellY < 0)
        return vIndices;

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
            const vector<size_t> vCell = mGrid[ix][iy];
            for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if (z > 0) {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<std::weak_ptr<MapPoint> > vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2, 3);
    for (int i = 0; i < N; i++) {
        std::weak_ptr<MapPoint> pMP = mvpMapPoints[i];
        if (!pMP.expired()) {
            cv::Mat x3Dw = pMP.lock()->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(), vDepths.end());

    return vDepths[(vDepths.size() - 1) / q];
}

void KeyFrame::DeletePointCloud()
{
    remove(mPointCloudName.c_str());
}

void KeyFrame::SetVocabulary(std::shared_ptr<ORBVocabulary> pVocabulary)
{
    mpORBvocabulary = pVocabulary;
}

void KeyFrame::InitializeStaticVariables(std::string depthImageFolder)
{
    msDepthImagesFolder = depthImageFolder;
    mpORBvocabulary = Frame::mpORBvocabulary;
    mK = Frame::mK;
    mbf = Frame::mbf;
    mThDepth = Frame::mThDepth;

    mnGridCols = FRAME_GRID_COLS;
    mnGridRows = FRAME_GRID_ROWS;

    // Scale Level Info
    mnScaleLevels = Frame::mnScaleLevels;
    mfScaleFactor = Frame::mfScaleFactor;
    mfLogScaleFactor = Frame::mfLogScaleFactor;
    mvScaleFactors = Frame::mvScaleFactors;
    mvLevelSigma2 = Frame::mvLevelSigma2;
    mvInvLevelSigma2 = Frame::mvInvLevelSigma2;

    mnMinX = Frame::mnMinX;
    mnMinY = Frame::mnMinY;
    mnMaxX = Frame::mnMaxX;
    mnMaxY = Frame::mnMaxY;

    mfGridElementWidthInv = Frame::mfGridElementWidthInv;
    mfGridElementHeightInv = Frame::mfGridElementHeightInv;

    fx = Frame::fx;
    fy = Frame::fy;
    cx = Frame::cx;
    cy = Frame::cy;
    invfx = Frame::invfx;
    invfy = Frame::invfy;

    mb = Frame::mb;
    mHalfBaseline = mb / 2.0;
}


void KeyFrame::SetMap(std::shared_ptr<Map> map)
{
    mpMap = map;
}

void KeyFrame::SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKeyFrameDB)
{
    mpKeyFrameDB = pKeyFrameDB;
}

void KeyFrame::RestoreKeyFrame(std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDatabase> pKeyFrameDB)
{
    SetMap(map);
    SetKeyFrameDatabase(pKeyFrameDB);
    ComputeBoW();
    pKeyFrameDB->add(shared_from_this());
}

} //namespace ORB_SLAM
