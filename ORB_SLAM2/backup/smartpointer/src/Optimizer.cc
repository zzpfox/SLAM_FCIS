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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{

void Optimizer::GlobalBundleAdjustemnt(std::shared_ptr<Map> pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust)
{
    vector<std::shared_ptr<KeyFrame> > vpKFs = pMap->GetAllKeyFrames();
    vector<std::shared_ptr<MapPoint> > vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const vector<std::shared_ptr<KeyFrame> > &vpKFs, const vector<std::shared_ptr<MapPoint> > &vpMP,
                                 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                 const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++) {

        std::shared_ptr<KeyFrame> spKF = vpKFs[i];
        if (spKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(spKF->GetPose()));
        vSE3->setId(spKF->mnId);
        vSE3->setFixed(spKF->mnId == 0);
        optimizer.addVertex(vSE3);
        if (spKF->mnId > maxKFid)
            maxKFid = spKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++) {

        std::shared_ptr<MapPoint> pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > >
            observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator
                 mit = observations.begin(); mit != observations.end(); mit++) {

            std::weak_ptr<KeyFrame> pKF = mit->first;
            if (pKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spKF = pKF.lock();
            if (spKF->isBad() || spKF->mnId > maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = spKF->mvKeysUn[mit->second];

            if (spKF->mvuRight[mit->second] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = spKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = spKF->fx;
                e->fy = spKF->fy;
                e->cx = spKF->cx;
                e->cy = spKF->cy;

                optimizer.addEdge(e);
            }
            else {
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = spKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = spKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = spKF->fx;
                e->fy = spKF->fy;
                e->cx = spKF->cx;
                e->cy = spKF->cy;
                e->bf = spKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        }
        else {
            vbNotIncludedMP[i] = false;
        }
    }
    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++) {
        std::shared_ptr<KeyFrame> spKF = vpKFs[i];
        if (spKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(spKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == 0) {
            spKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else {
            spKF->mTcwGBA.create(4, 4, CV_32F);
            Converter::toCvMat(SE3quat).copyTo(spKF->mTcwGBA);
            spKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        std::shared_ptr<MapPoint> pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;
    vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
//    vpEdgesMono.reserve(N);
//    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
//    vpEdgesStereo.reserve(N);
//    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
            std::weak_ptr<MapPoint> pMP = pFrame->mvpMapPoints[i];
            if (!pMP.expired()) {
                std::shared_ptr<MapPoint> spMP = pMP.lock();
                // Monocular observation
                if (pFrame->mvuRight[i] < 0) {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    cv::Mat Xw = spMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else  // Stereo observation
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    //SET EDGE
                    Eigen::Matrix<double, 3, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    const float &kp_ur = pFrame->mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    e->bf = pFrame->mbf;
                    cv::Mat Xw = spMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
            }

        }
    }

    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    for (size_t it = 0; it < 4; it++) {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else {
                e->setLevel(0);
                pFrame->mvbOutlier[idx] = false;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(std::shared_ptr<KeyFrame> pKF, bool *pbStopFlag, std::shared_ptr<Map> pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<std::shared_ptr<KeyFrame> > lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<std::weak_ptr<KeyFrame> > vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
        std::weak_ptr<KeyFrame> pKFi = vNeighKFs[i];
        if (pKFi.expired())
            continue;
        std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
        spKFi->mnBALocalForKF = pKF->mnId;
        if (!spKFi->isBad())
            lLocalKeyFrames.push_back(spKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<std::shared_ptr<MapPoint> > lLocalMapPoints;
    for (list<std::shared_ptr<KeyFrame> >::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        vector<std::weak_ptr<MapPoint> > vpMPs = (*lit)->GetMapPointMatches();
        for (vector<std::weak_ptr<MapPoint> >::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            std::weak_ptr<MapPoint> pMP = *vit;
            if (!pMP.expired())
            {
                std::shared_ptr<MapPoint> spMP = pMP.lock();
                if (!spMP->isBad())
                    if (spMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(spMP);
                        spMP->mnBALocalForKF = pKF->mnId;
                    }
            }

        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<std::shared_ptr<KeyFrame> > lFixedCameras;
    for (list<std::shared_ptr<MapPoint> >::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations = (*lit)->GetObservations();
        for (map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > >::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            std::weak_ptr<KeyFrame> pKFi = mit->first;
            if (pKFi.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
            if (spKFi->mnBALocalForKF != pKF->mnId && spKFi->mnBAFixedForKF != pKF->mnId) {
                spKFi->mnBAFixedForKF = pKF->mnId;
                if (!spKFi->isBad())
                    lFixedCameras.push_back(spKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<std::shared_ptr<KeyFrame> >::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (list<std::shared_ptr<KeyFrame> >::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<std::shared_ptr<KeyFrame> > vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<std::shared_ptr<MapPoint> > vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<std::shared_ptr<KeyFrame> > vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<std::shared_ptr<MapPoint> > vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for (list<std::shared_ptr<MapPoint> >::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        std::shared_ptr<MapPoint> pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations = pMP->GetObservations();

        //Set edges
        for (map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            std::weak_ptr<KeyFrame> pKFi = mit->first;
            if (pKFi.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
            if (!spKFi->isBad()) {
                const cv::KeyPoint &kpUn = spKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (spKFi->mvuRight[mit->second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = spKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = spKFi->fx;
                    e->fy = spKFi->fy;
                    e->cx = spKFi->cx;
                    e->cy = spKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(spKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = spKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = spKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = spKFi->fx;
                    e->fy = spKFi->fy;
                    e->cx = spKFi->cx;
                    e->cy = spKFi->cy;
                    e->bf = spKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(spKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<std::shared_ptr<KeyFrame>, std::shared_ptr<MapPoint> > > vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            std::shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            std::shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            std::shared_ptr<KeyFrame> pKFi = vToErase[i].first;
            std::shared_ptr<MapPoint> pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (list<std::shared_ptr<KeyFrame> >::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<std::shared_ptr<MapPoint> >::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        std::shared_ptr<MapPoint> pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
            pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraph(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF,
                                       std::shared_ptr<KeyFrame> pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<std::weak_ptr<KeyFrame>,
                                                 set<std::weak_ptr<KeyFrame>,
                                                     std::owner_less<std::weak_ptr<KeyFrame> > >,
                                                 std::owner_less<std::weak_ptr<KeyFrame> > > &LoopConnections,
                                       const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<std::shared_ptr<KeyFrame> > vpKFs = pMap->GetAllKeyFrames();
    const vector<std::shared_ptr<MapPoint> > vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        std::shared_ptr<KeyFrame> spKF = vpKFs[i];
        if (spKF->isBad())
            continue;
        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = spKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(spKF);

        if (it != CorrectedSim3.end()) {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else {
            Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(spKF->GetRotation());
            Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(spKF->GetTranslation());
            g2o::Sim3 Siw(Rcw, tcw, 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if (spKF == pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;
    }

    set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    for (map<std::weak_ptr<KeyFrame>,
             set<std::weak_ptr<KeyFrame>,
                 std::owner_less<std::weak_ptr<KeyFrame> > >,
             std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end();
         mit != mend; mit++) {
        std::weak_ptr<KeyFrame> pKF = mit->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        const long unsigned int nIDi = spKF->mnId;
        const set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator sit = spConnections.begin(), send = spConnections.end();
             sit != send; sit++) {
            std::weak_ptr<KeyFrame> pKFi = *sit;
            if (pKFi.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFi = pKFi.lock();
            const long unsigned int nIDj = spKFi->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && spKF->GetWeight(*sit) < minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
        }
    }

    // Set normal edges
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        std::shared_ptr<KeyFrame> spKF = vpKFs[i];

        const int nIDi = spKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(spKF);

        if (iti != NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        std::weak_ptr<KeyFrame> pParentKF = spKF->GetParent();

        // Spanning tree edge
        if (!pParentKF.expired()) {
            std::shared_ptr<KeyFrame> spParentKF = pParentKF.lock();
            int nIDj = spParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(spParentKF);

            if (itj != NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > sLoopEdges = spKF->GetLoopEdges();
        for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator sit = sLoopEdges.begin(),
                 send = sLoopEdges.end(); sit != send; sit++) {
            std::weak_ptr<KeyFrame> pLKF = *sit;
            if (pLKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spLKF = pLKF.lock();
            if (spLKF->mnId < spKF->mnId) {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(spLKF);

                if (itl != NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[spLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<std::weak_ptr<KeyFrame> > vpConnectedKFs = spKF->GetCovisiblesByWeight(minFeat);
        for (vector<std::weak_ptr<KeyFrame> >::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
            std::weak_ptr<KeyFrame> pKFn = *vit;
            if (pKFn.expired() || pParentKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spKFn = pKFn.lock();
            std::shared_ptr<KeyFrame> spParentKF = pParentKF.lock();
            if (spKFn != spParentKF && !spKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
                if (!spKFn->isBad() && spKFn->mnId < spKF->mnId) {
                    if (sInsertedEdges.count(make_pair(min(spKF->mnId, spKFn->mnId), max(spKF->mnId, spKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(spKFn);

                    if (itn != NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[spKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(spKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (size_t i = 0; i < vpKFs.size(); i++) {
        std::shared_ptr<KeyFrame> spKFi = vpKFs[i];
        const int nIDi = spKFi->mnId;

        g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *= (1. / s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

        spKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        std::shared_ptr<MapPoint> pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        int nIDr;
        if (pMP->mnCorrectedByKF == pCurKF->mnId) {
            nIDr = pMP->mnCorrectedReference;
        }
        else {
            std::weak_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
            if (pRefKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spRefKF = pRefKF.lock();
            nIDr = spRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame>pKF2, vector<std::weak_ptr<MapPoint> > &vpMatches1, g2o::Sim3 &g2oS12,
                            const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0, 2);
    vSim3->_principle_point1[1] = K1.at<float>(1, 2);
    vSim3->_focal_length1[0] = K1.at<float>(0, 0);
    vSim3->_focal_length1[1] = K1.at<float>(1, 1);
    vSim3->_principle_point2[0] = K2.at<float>(0, 2);
    vSim3->_principle_point2[1] = K2.at<float>(1, 2);
    vSim3->_focal_length2[0] = K2.at<float>(0, 0);
    vSim3->_focal_length2[1] = K2.at<float>(1, 1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<std::weak_ptr<MapPoint> > vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for (int i = 0; i < N; i++) {
        if (vpMatches1[i].expired())
            continue;

        std::weak_ptr<MapPoint> pMP1 = vpMapPoints1[i];
        std::weak_ptr<MapPoint> pMP2 = vpMatches1[i];

        const int id1 = 2 * i + 1;
        const int id2 = 2 * (i + 1);
        if (pMP1.expired() || pMP2.expired())
            continue;
        std::shared_ptr<MapPoint> spMP1 = pMP1.lock();
        std::shared_ptr<MapPoint> spMP2 = pMP2.lock();
        const int i2 = spMP2->GetIndexInKeyFrame(pKF2);

        if (spMP1 && spMP2) {
            if (!spMP1->isBad() && !spMP2->isBad() && i2 >= 0) {
                g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = spMP1->GetWorldPos();
                cv::Mat P3D1c = R1w * P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = spMP2->GetWorldPos();
                cv::Mat P3D2c = R2w * P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double, 2, 1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

        g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double, 2, 1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

        g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++) {
        g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
            continue;

        if (e12->chi2() > th2 || e21->chi2() > th2) {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx].reset();
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
            vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if (nBad > 0)
        nMoreIterations = 10;
    else
        nMoreIterations = 5;

    if (nCorrespondences - nBad < 10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++) {
        g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
            continue;

        if (e12->chi2() > th2 || e21->chi2() > th2) {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx].reset();
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nIn;
}

} //namespace ORB_SLAM
