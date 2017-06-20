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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>

namespace ORB_SLAM2
{

LoopClosing::LoopClosing(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrameDatabase> pDB,
                         std::shared_ptr<ORBVocabulary> pVoc, const bool bFixScale)
    :
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mLastLoopKFid(0), mbRunningGBA(false),
    mbFinishedGBA(true),
    mbStopGBA(false), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mpMatchedKF.reset();
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(std::shared_ptr<Tracking> pTracker)
{
    mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run()
{
    mbFinished = false;

    while (1) {
//         Check if there are keyframes in the queue
        if (CheckNewKeyFrames()) {
            // Detect loop candidates and check covisibility consistency
            if (DetectLoop()) {

                // Compute similarity transformation [sR|t]
                // In the stereo/RGBD case s=1
                if (ComputeSim3()) {
                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }
        }

        ResetIfRequested();

        if (CheckFinish())
        {
            while (mbRunningGBA)
            {
                usleep(1000);
            }
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if (pKF->mnId != 0)
    {
        std::weak_ptr<KeyFrame> wpKF(pKF);
        mlpLoopKeyFrameQueue.push_back(wpKF);
    }

}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    std::shared_ptr<KeyFrame> spCurrentKF;
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        if (mpCurrentKF.expired())
        {
            return false;
        }
        spCurrentKF = mpCurrentKF.lock();
        spCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if (spCurrentKF->mnId < mLastLoopKFid + 10) {
        mpKeyFrameDB->add(spCurrentKF);
        spCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<std::weak_ptr<KeyFrame> > vpConnectedKeyFrames = spCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = spCurrentKF->mBowVec;
    float minScore = 1;
    for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
        std::weak_ptr<KeyFrame> pKF = vpConnectedKeyFrames[i];
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        if (spKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = spKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if (score < minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<std::weak_ptr<KeyFrame> > vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(spCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if (vpCandidateKFs.empty()) {
        mpKeyFrameDB->add(spCurrentKF);
        mvConsistentGroups.clear();
        spCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);
    for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++) {
        std::weak_ptr<KeyFrame> pCandidateKF = vpCandidateKFs[i];
        if (pCandidateKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spCandidateKF = pCandidateKF.lock();
        set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > spCandidateGroup = spCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; iG++) {
            set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end();
                 sit != send; sit++) {
                if (sPreviousGroup.count(*sit)) {
                    bConsistent = true;
                    bConsistentForSomeGroup = true;
                    break;
                }
            }

            if (bConsistent) {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if (!vbConsistentGroup[iG]) {
                    ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
                }
                if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent = true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if (!bConsistentForSomeGroup) {
            ConsistentGroup cg = make_pair(spCandidateGroup, 0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(spCurrentKF);

    if (mvpEnoughConsistentCandidates.empty()) {
        spCurrentKF->SetErase();
        return false;
    }
    else {
        return true;
    }
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75, true);

    vector<std::shared_ptr<Sim3Solver> > vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<std::weak_ptr<MapPoint> > > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates = 0; //candidates with enough matches
    if (mpCurrentKF.expired())
        return false;
    std::shared_ptr<KeyFrame> spCurrentKF = mpCurrentKF.lock();
    for (int i = 0; i < nInitialCandidates; i++) {
        std::weak_ptr<KeyFrame> pKF = mvpEnoughConsistentCandidates[i];
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();

        // avoid that local mapping erase it while it is being processed in this thread
        spKF->SetNotErase();

        if (spKF->isBad()) {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(spCurrentKF, spKF, vvpMapPointMatches[i]);

        if (nmatches < 20) {
            vbDiscarded[i] = true;
            continue;
        }
        else {
            std::shared_ptr<Sim3Solver> pSolver = std::make_shared<Sim3Solver>
                (spCurrentKF, spKF, vvpMapPointMatches[i], mbFixScale);
            pSolver->SetRansacParameters(0.99, 20, 300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nInitialCandidates; i++) {
            if (vbDiscarded[i])
                continue;

            std::weak_ptr<KeyFrame> pKF = mvpEnoughConsistentCandidates[i];
            if (pKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spKF = pKF.lock();

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            std::shared_ptr<Sim3Solver> pSolver = vpSim3Solvers[i];
            cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if (!Scm.empty()) {
                vector<std::weak_ptr<MapPoint> > vpMapPointMatches(vvpMapPointMatches[i].size());
                for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
                    if (vbInliers[j])
                        vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(spCurrentKF, spKF, vpMapPointMatches, s, R, t, 7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                const int nInliers = Optimizer::OptimizeSim3(spCurrentKF, spKF, vpMapPointMatches, gScm, 10,
                                                             mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if (nInliers >= 20) {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(spKF->GetRotation()),
                                   Converter::toVector3d(spKF->GetTranslation()), 1.0);
                    mg2oScw = gScm * gSmw;
                    mScw = Converter::toCvMat(mg2oScw);
                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        for (int i = 0; i < nInitialCandidates; i++)
        {
            if (!mvpEnoughConsistentCandidates[i].expired())
            {
                mvpEnoughConsistentCandidates[i].lock()->SetErase();
            }
        }

        spCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    if (mpMatchedKF.expired())
        return false;
    std::shared_ptr<KeyFrame> spMatchedKF = mpMatchedKF.lock();
    vector<std::weak_ptr<KeyFrame> > vpLoopConnectedKFs = spMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for (vector<std::weak_ptr<KeyFrame> >::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++) {
        std::weak_ptr<KeyFrame> pKF = *vit;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        vector<std::weak_ptr<MapPoint> > vpMapPoints = spKF->GetMapPointMatches();
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
            std::weak_ptr<MapPoint> pMP = vpMapPoints[i];
            if (!pMP.expired()) {
                std::shared_ptr<MapPoint> spMP = pMP.lock();
                if (!spMP->isBad() && spMP->mnLoopPointForKF != spCurrentKF->mnId) {
                    mvpLoopMapPoints.push_back(pMP);
                    spMP->mnLoopPointForKF = spCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(spCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++) {
        if (!mvpCurrentMatchedPoints[i].expired())
            nTotalMatches++;
    }

    if (nTotalMatches >= 40) {
        for (int i = 0; i < nInitialCandidates; i++)
            if (!mvpEnoughConsistentCandidates[i].expired() && mvpEnoughConsistentCandidates[i].lock() != mpMatchedKF.lock())
                mvpEnoughConsistentCandidates[i].lock()->SetErase();
        return true;
    }
    else {
        for (int i = 0; i < nInitialCandidates; i++)
            if (!mvpEnoughConsistentCandidates[i].expired())
                mvpEnoughConsistentCandidates[i].lock()->SetErase();
        spCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if (isRunningGBA()) {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if (mpThreadGBA) {
            mpThreadGBA.reset();
        }
    }

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    if (mpCurrentKF.expired())
        return;
    std::shared_ptr<KeyFrame> spCurrentKF = mpCurrentKF.lock();
    spCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = spCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF] = mg2oScw;
    cv::Mat Twc = spCurrentKF->GetPoseInverse();

    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for (vector<std::weak_ptr<KeyFrame>>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
             vit != vend; vit++) {
            std::weak_ptr<KeyFrame> pKFi = *vit;
            if (pKFi.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFi = pKFi.lock();

            cv::Mat Tiw = spKFi->GetPose();

            if (spKFi != spCurrentKF) {
                cv::Mat Tic = Tiw * Twc;
                cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                cv::Mat tic = Tic.rowRange(0, 3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi] = g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
            cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi] = g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end();
             mit != mend; mit++) {
            std::weak_ptr<KeyFrame> pKFi = mit->first;
            if (pKFi.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            vector<std::weak_ptr<MapPoint> > vpMPsi = spKFi->GetMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
                std::weak_ptr<MapPoint> pMPi = vpMPsi[iMP];
                if (pMPi.expired())
                    continue;
                std::shared_ptr<MapPoint> spMPi = pMPi.lock();
                if (spMPi->isBad())
                    continue;
                if (spMPi->mnCorrectedByKF == spCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = spMPi->GetWorldPos();
                Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                spMPi->SetWorldPos(cvCorrectedP3Dw);
                spMPi->mnCorrectedByKF = spCurrentKF->mnId;
                spMPi->mnCorrectedReference = spKFi->mnId;
                spMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *= (1. / s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);

            spKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            spKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++) {
            if (!mvpCurrentMatchedPoints[i].expired()) {
                std::weak_ptr<MapPoint> pLoopMP = mvpCurrentMatchedPoints[i];
                std::shared_ptr<MapPoint> spLoopMP = pLoopMP.lock();
                std::weak_ptr<MapPoint> pCurMP = spCurrentKF->GetMapPoint(i);
                if (!pCurMP.expired())
                    pCurMP.lock()->Replace(spLoopMP);
                else {
                    spCurrentKF->AddMapPoint(spLoopMP, i);
                    spLoopMP->AddObservation(spCurrentKF, i);
                    spLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<std::weak_ptr<KeyFrame>, set<std::weak_ptr<KeyFrame>,
                                     std::owner_less<std::weak_ptr<KeyFrame> > >,
        std::owner_less<std::weak_ptr<KeyFrame> > > LoopConnections;

    for (vector<std::weak_ptr<KeyFrame> >::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
         vit != vend; vit++) {
        std::weak_ptr<KeyFrame> pKFi = *vit;
        if (pKFi.expired())
            continue;
        std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
        vector<std::weak_ptr<KeyFrame> > vpPreviousNeighbors = spKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        spKFi->UpdateConnections();
        LoopConnections[pKFi] = spKFi->GetConnectedKeyFrames();
        for (vector<std::weak_ptr<KeyFrame> >::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end();
             vit_prev != vend_prev; vit_prev++) {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for (vector<std::weak_ptr<KeyFrame> >::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end();
             vit2 != vend2; vit2++) {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    if (mpMatchedKF.expired())
        return;
    std::shared_ptr<KeyFrame> spMatchedKF = mpMatchedKF.lock();
    Optimizer::OptimizeEssentialGraph(mpMap, spMatchedKF, spCurrentKF, NonCorrectedSim3, CorrectedSim3,
                                      LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    spMatchedKF->AddLoopEdge(mpCurrentKF);
    spCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA.reset(new thread(&LoopClosing::RunGlobalBundleAdjustment, this, spCurrentKF->mnId));
    mpThreadGBA->detach();
    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = spCurrentKF->mnId;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end();
         mit != mend; mit++) {
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<std::shared_ptr<MapPoint> > vpReplacePoints(mvpLoopMapPoints.size());
        vector<std::shared_ptr<MapPoint> > svpLoopMapPoints(mvpLoopMapPoints.size());
        std::transform(mvpLoopMapPoints.begin(), mvpLoopMapPoints.end(), svpLoopMapPoints.begin(),
                       std::bind(&std::weak_ptr<MapPoint>::lock, std::placeholders::_1));
        matcher.Fuse(spKF, cvScw, svpLoopMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for (int i = 0; i < nLP; i++) {
            std::shared_ptr<MapPoint> pRep = vpReplacePoints[i];
            if (pRep && !mvpLoopMapPoints[i].expired()) {
                pRep->Replace(mvpLoopMapPoints[i].lock());
            }
        }
    }
}

void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while (1) {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested) {
        mlpLoopKeyFrameQueue.clear();
        mvConsistentGroups.clear();
        mvpEnoughConsistentCandidates.clear();
        mvpCurrentConnectedKFs.clear();
        mvpCurrentMatchedPoints.clear();
        mvpLoopMapPoints.clear();
        mLastLoopKFid = 0;
        mbResetRequested = false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx = mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap, 10, &mbStopGBA, nLoopKF, false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if (idx != mnFullBAIdx)
            return;

        if (!mbStopGBA) {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<std::weak_ptr<KeyFrame> > lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());

            while (!lpKFtoCheck.empty()) {
                std::weak_ptr<KeyFrame> pKF = lpKFtoCheck.front();
                if (pKF.expired())
                    continue;
                std::shared_ptr<KeyFrame> spKF = pKF.lock();
                const set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > sChilds = spKF->GetChilds();
                cv::Mat Twc = spKF->GetPoseInverse();
                for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                    std::weak_ptr<KeyFrame> pChild = *sit;
                    if (pChild.expired())
                        continue;
                    std::shared_ptr<KeyFrame> spChild = pChild.lock();
                    if (spChild->mnBAGlobalForKF != nLoopKF) {
                        cv::Mat Tchildc = spChild->GetPose() * Twc;
                        spChild->mTcwGBA = Tchildc * spKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        spChild->mnBAGlobalForKF = nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                spKF->mTcwBefGBA = spKF->GetPose();
                spKF->SetPose(spKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<std::shared_ptr<MapPoint>  > vpMPs = mpMap->GetAllMapPoints();

            for (size_t i = 0; i < vpMPs.size(); i++) {
                std::shared_ptr<MapPoint> pMP = vpMPs[i];

                if (pMP->isBad())
                    continue;

                if (pMP->mnBAGlobalForKF == nLoopKF) {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else {
                    // Update according to the correction of its reference keyframe
                    std::weak_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
                    if (pRefKF.expired())
                        continue;
                    std::shared_ptr<KeyFrame> spRefKF = pRefKF.lock();
                    if (spRefKF->mnBAGlobalForKF != nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = spRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
                    cv::Mat tcw = spRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
                    cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = spRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
                    cv::Mat twc = Twc.rowRange(0, 3).col(3);

                    pMP->SetWorldPos(Rwc * Xc + twc);
                }
            }

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
