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


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <thread>
#include <boost/filesystem.hpp>
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include "Communication.h"
#include <iostream>
#include <queue>
#include <mutex>

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, std::shared_ptr<ORBVocabulary> pVoc, std::shared_ptr<FrameDrawer> pFrameDrawer,
                   std::shared_ptr<MapDrawer> pMapDrawer, std::shared_ptr<Map> pMap,
                   std::shared_ptr<KeyFrameDatabase> pKFDB, const string &strSettingPath,
                   const int sensor, std::string dataDir, const bool bReuseMap):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpSystem(pSys), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
    mpMap(pMap), mnLastRelocFrameId(0), mbSegStop(false),
    mbReuseMap(bReuseMap), msDataFolder(dataDir), msDepthImagesFolder("DepthImages")
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = std::make_shared<ORBextractor> (nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::STEREO)
        mpORBextractorRight = std::make_shared<ORBextractor> (nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::MONOCULAR)
        mpIniORBextractor = std::make_shared<ORBextractor> (2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if (sensor == System::STEREO || sensor == System::RGBD) {
        mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if (sensor == System::RGBD) {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }
    StartSegmentationThread();

    boost::filesystem::path depthImageFolder(msDepthImagesFolder);
    boost::filesystem::path depthImageFolderFullPath = msDataFolder / depthImageFolder;
    if (!mbReuseMap)
    {
        std::string deleteDepth;
        while(true)
        {
            std::cout << "\x1B[33m" << "Do you wanna delete the previously stored DepthImages? " << std::endl
                      << "'y' or 'yes': to save" << std::endl
                      << "'n' or 'no': not to save"<< "\x1B[0m" << std::endl;

            std::cin >> deleteDepth;
            std::cout << "Your input is: " << deleteDepth << std::endl;
            std::cin.clear();
            if (std::cin.fail())
            {
                std::cout << "\x1B[33m" << "Invalid input, input must be string"<< "\x1B[0m" << std::endl;
                continue;
            }
            if (deleteDepth == "y" || deleteDepth == "yes")
            {
                std::cout << "\x1B[33m" << "Will delete the DepthImages. Are you sure? ('y' or 'yes' to confirm)" << "\x1B[0m" << std::endl;
                std::cin >> deleteDepth;
                std::cin.clear();
                if (std::cin.fail())
                {
                    std::cout << "\x1B[33m" << "Invalid input, input must be string"<< "\x1B[0m" << std::endl;
                    continue;
                }
                std::cout << "Your input is: " << deleteDepth << std::endl;
                if (deleteDepth == "y" || deleteDepth == "yes")
                {
                    std::cout << "Deleting and recreating the DepthImage folder ..." << std::endl;

                    boost::filesystem::remove_all(depthImageFolderFullPath);
                    boost::filesystem::create_directories(depthImageFolderFullPath);

                    std::cout << "Deleting and recreating the DepthImage folder done ... " << std::endl;
                    break;
                }
            }
            else if (deleteDepth == "n" || deleteDepth == "no")
            {
                std::cout << "\x1B[33m" << "WARNING: Will not delete the DepthImages. Are you sure? ('y' or 'yes' to confirm)" << "\x1B[0m" << std::endl;
                std::cin >> deleteDepth;
                std::cin.clear();
                if (std::cin.fail())
                {
                    std::cout << "\x1B[33m" << "Invalid input, input must be string"<< "\x1B[0m" << std::endl;
                    continue;
                }
                std::cout << "Your input is: " << deleteDepth << std::endl;
                if (deleteDepth == "y" || deleteDepth == "yes")
                {
                    std::cout << "DepthImages folder is not deleted ..." << std::endl;
                    break;
                }
            }
            else
            {
                std::cout << "Unexpected input word, please type one of the following words:" << std::endl
                          << "\x1B[34m" << "   'y', 'yes', 'n', 'no'   " << "\x1B[0m" << std::endl;
            }
        }

    }


    float rows = fSettings["Camera.height"];
    float cols = fSettings["Camera.width"];

    Frame::InitializeStaticVariables(mpORBextractorLeft,
                                     mpORBVocabulary,
                                     mK,
                                     mDistCoef,
                                     mbf,
                                     mThDepth,
                                     rows,
                                     cols);
    KeyFrame::InitializeStaticVariables(depthImageFolderFullPath.string());
    MapDrawer::msDepthImagesPath = depthImageFolderFullPath.string();
}

void Tracking::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(std::shared_ptr<Viewer> pViewer)
{
    mpViewer = pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if (mImGray.channels() == 3) {
        if (mbRGB) {
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
        }
        else {
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
        }
    }
    else if (mImGray.channels() == 4) {
        if (mbRGB) {
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
        }
        else {
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight);
    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    mCurrentFrame = Frame(mImGray, imRGB, imDepth, timestamp, mpORBextractorLeft);
    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor);
    else
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED && !mbReuseMap) {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(shared_from_this());

        if (mState != OK)
            return;
    }
    else {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking) {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK) {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                    bOK = TrackReferenceKeyFrame();
                }
                else {
                    bOK = TrackWithMotionModel();
                    if (!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else {
                bOK = Relocalization();
            }
        }
        else {
            // Localization Mode: Local Mapping is deactivated
            if (mState == LOST) {
                bOK = Relocalization();
            }
            else {
                if (!mbVO) {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty()) {
                        bOK = TrackWithMotionModel();
                    }
                    else {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<std::shared_ptr<MapPoint> > vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty()) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame.N; i++) {
                                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (!mbOnlyTracking) {
            if (bOK)
                bOK = TrackLocalMap();
        }
        else {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if (bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if (bOK)
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(shared_from_this());
        // If tracking were good, check if we insert a keyframe
        if (bOK) {
            // Update motion model
            if (!mLastFrame.mTcw.empty()) {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame.N; i++) {
                std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i].reset();
                    }
            }

            // Delete temporal MapPoints
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if (NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i].reset();
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            if (mpMap->KeyFramesInMap() <= 5) {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if (!mCurrentFrame.mpReferenceKF.expired())
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if (mCurrentFrame.mpReferenceKF.expired())
        return;
    std::shared_ptr<KeyFrame> spReferenceKF = mCurrentFrame.mpReferenceKF.lock();
    if (!mCurrentFrame.mTcw.empty()) {
        cv::Mat Tcr = mCurrentFrame.mTcw * spReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState == LOST);
    }
    else {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState == LOST);
    }

}

void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N > 500) {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

        // Create KeyFrame
        std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame> (mCurrentFrame, mpMap, mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint> (x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        std::weak_ptr<KeyFrame> wpKFini = pKFini;
        mvpLocalKeyFrames.push_back(wpKFini);
        std::vector<std::shared_ptr<MapPoint> > svpLocalMapPoints = mpMap->GetAllMapPoints();
        mvpLocalMapPoints.clear();
        for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpLocalMapPoints.begin();
            it != svpLocalMapPoints.end(); ++it)
        {
            std::weak_ptr<MapPoint> wit = *it;
            mvpLocalMapPoints.push_back(wit);
        }
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(svpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(wpKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState = OK;
    }
}

void Tracking::MonocularInitialization()
{

    if (!mpInitializer) {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100) {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            if (mpInitializer)
                mpInitializer.reset();

            mpInitializer = std::make_shared<Initializer> (mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else {
        // Try to initialize
        if ((int) mCurrentFrame.mvKeys.size() <= 100) {
            mpInitializer.reset();
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                       100);

        // Check if there are enough correspondences
        if (nmatches < 100) {
            mpInitializer.reset();
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame> (mInitialFrame, mpMap, mpKeyFrameDB);
    std::shared_ptr<KeyFrame> pKFcur = std::make_shared<KeyFrame> (mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint> (worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<std::weak_ptr<MapPoint> > vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (!vpAllMapPoints[iMP].expired()) {
            std::shared_ptr<MapPoint> pMP = vpAllMapPoints[iMP].lock();
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    std::weak_ptr<KeyFrame> wpKFcur(pKFcur);
    std::weak_ptr<KeyFrame> wpKFini(pKFini);
    mvpLocalKeyFrames.push_back(wpKFcur);
    mvpLocalKeyFrames.push_back(wpKFini);
    std::vector<std::shared_ptr<MapPoint> > svpLocalMapPoints = mpMap->GetAllMapPoints();
    mvpLocalMapPoints.clear();
    for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpLocalMapPoints.begin();
         it != svpLocalMapPoints.end(); ++it)
    {
        std::weak_ptr<MapPoint> wit = *it;
        mvpLocalMapPoints.push_back(wit);
    }
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(svpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(wpKFini);

    mState = OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for (int i = 0; i < mLastFrame.N; i++) {
        std::weak_ptr<MapPoint> pMP = mLastFrame.mvpMapPoints[i];

        if (!pMP.expired()) {
            std::weak_ptr<MapPoint> pRep = pMP.lock()->GetReplaced();
            if (!pRep.expired()) {
                mLastFrame.mvpMapPoints[i] = pRep.lock();
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<std::shared_ptr<MapPoint> > vpMapPointMatches;
    if (mpReferenceKF.expired())
        return false;
    std::shared_ptr<KeyFrame> spReferenceKF = mpReferenceKF.lock();

    int nmatches = matcher.SearchByBoW(spReferenceKF, mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            std::shared_ptr<MapPoint> spMP = mCurrentFrame.mvpMapPoints[i];
            if (mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i].reset();
                mCurrentFrame.mvbOutlier[i] = false;
                spMP->mbTrackInView = false;
                spMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (spMP->Observations() > 0)
                nmatchesMap++;
        }
    }

    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    std::weak_ptr<KeyFrame> pRef = mLastFrame.mpReferenceKF;
    if (pRef.expired())
        return;
    std::shared_ptr<KeyFrame> spRef = pRef.lock();
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * spRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++) {
        float z = mLastFrame.mvDepth[i];
        if (z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        std::shared_ptr<MapPoint> pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1) {
            bCreateNew = true;
        }

        if (bCreateNew) {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint> (x3D, mpMap, &mLastFrame, i);
            std::weak_ptr<MapPoint> wpNewMP = pNewMP;
            mLastFrame.mvpMapPoints[i] = pNewMP;
            mlpTemporalPoints.push_back(wpNewMP);
            nPoints++;
        }
        else {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), std::shared_ptr<MapPoint> ());

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 20) {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), std::shared_ptr<MapPoint> ());
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
    }

    if (nmatches < 20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            std::shared_ptr<MapPoint> spMP = mCurrentFrame.mvpMapPoints[i];
            if (mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i].reset();
                mCurrentFrame.mvbOutlier[i] = false;
                spMP->mbTrackInView = false;
                spMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (spMP->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i].reset();

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        return false;

    if (mnMatchesInliers < 30)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;

    if (mpReferenceKF.expired())
        return true;
    std::shared_ptr<KeyFrame> spReferenceKF = mpReferenceKF.lock();
    int nRefMatches = spReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != System::MONOCULAR) {
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

    if ((c1a || c1b || c1c) && c2) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle) {
            return true;
        }
        else {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if (!mpLocalMapper->SetNotStop(true))
        return;

    std::shared_ptr<KeyFrame> pKF = std::make_shared<KeyFrame> (mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mSensor != System::MONOCULAR) {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                std::weak_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP.expired())
                    bCreateNew = true;
                else if (pMP.lock()->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i].reset();
                }

                if (bCreateNew) {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint> (x3D, pKF, mpMap);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;

    ImagePair images;
    images.keyFrameId = pKF->mnId;
    images.colorImg = mCurrentFrame.mImColor;
    images.depthImg = mCurrentFrame.mImDepth;

    std::unique_lock<std::mutex> lck(mMutexImagesQueue);
    if (mImagesQueue.size() < mcQueueSize) {
        mImagesQueue.push(images);
    }
    else {
        int count = 0;
        while (mImagesQueue.size() >= mcQueueSize) {
            mImagesQueue.pop();
            std::cout << "[Socket] Image Queue is too large, poping images " << ++count << std::endl;
        }
        mImagesQueue.push(images);
    }
}

void Tracking::PerformSegmentation()
{
    while (!mbSegStop) {
        std::unique_lock<std::mutex> lck(mMutexImagesQueue);
        if (!mImagesQueue.empty() && mState == OK) {
            ImagePair images;
            images = mImagesQueue.front();
            mImagesQueue.pop();
            std::vector<cv::Mat> color_imgs;
            color_imgs.push_back(images.colorImg);
            std::vector<cv::Mat> depth_imgs;
            depth_imgs.push_back(images.depthImg);
            mClient.sendImages(color_imgs);
            mClient.sendImages(depth_imgs);
            Client::ClsPosPairs clsPosPairs;
            mClient.getSegResult(clsPosPairs);
            SaveSegResultToMap(clsPosPairs, images.keyFrameId);
        }
    }
}

void Tracking::SaveSegResultToMap(const Client::ClsPosPairs &clsPosPairs, long unsigned int keyFrameId)
{
    int posPairSize = clsPosPairs.size();
    for (int i = 0; i < posPairSize; i++) {
        std::string objname = clsPosPairs[i].first;
        std::vector<std::vector<double> > poses = clsPosPairs[i].second;

        std::unordered_map<long unsigned int, ObjectPos> objKeyFrameMap;
        ObjectPos objpos;
        int posesSize = poses.size();
        for (int j = 0; j < posesSize; j++) {
            std::vector<double> Pc = poses[j];
            bool zeros = std::all_of(Pc.begin(), Pc.end(), [](double i){return i==0;});
            if (!zeros) {
                objpos.addInstance(Pc);
            }
        }

        objKeyFrameMap[keyFrameId] = objpos;
        std::unique_lock<mutex> lock(mpMap->mMutexObjectMap);
        auto it = mpMap->mObjectMap.find(objname);
        if (it != mpMap->mObjectMap.end()) {
            it->second.insert(objKeyFrameMap.begin(), objKeyFrameMap.end());
        }
        else {
            mpMap->mObjectMap.insert(std::make_pair(objname, objKeyFrameMap));
        }
    }
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for (vector<std::shared_ptr<MapPoint> >::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
         vit != vend; vit++) {
        std::shared_ptr<MapPoint> pMP = *vit;
        if (pMP) {;
            if (pMP->isBad()) {
                vit->reset();
            }
            else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<std::weak_ptr<MapPoint> >::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
         vit != vend; vit++) {
        std::weak_ptr<MapPoint> pMP = *vit;
        if (pMP.expired())
            continue;
        std::shared_ptr<MapPoint> spMP = pMP.lock();
        if (spMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (spMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(spMP, 0.5)) {
            spMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if (nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD)
            th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    vector<std::shared_ptr<MapPoint> > svpLocalMapPoints(mvpLocalMapPoints.size());
    std::transform(mvpLocalMapPoints.begin(), mvpLocalMapPoints.end(), svpLocalMapPoints.begin(),
                   std::bind(&std::weak_ptr<MapPoint>::lock, std::placeholders::_1));
    mpMap->SetReferenceMapPoints(svpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for (vector<std::weak_ptr<KeyFrame> >::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++) {
        std::weak_ptr<KeyFrame> pKF = *itKF;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        const vector<std::weak_ptr<MapPoint> > vpMPs = spKF->GetMapPointMatches();

        for (vector<std::weak_ptr<MapPoint> >::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
             itMP != itEndMP; itMP++) {
            std::weak_ptr<MapPoint> pMP = *itMP;
            if (pMP.expired())
                continue;
            std::shared_ptr<MapPoint> spMP = pMP.lock();
            if (spMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!spMP->isBad()) {
                mvpLocalMapPoints.push_back(pMP);
                spMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<std::weak_ptr<KeyFrame>, int, std::owner_less<std::weak_ptr<KeyFrame> > > keyframeCounter;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            std::shared_ptr<MapPoint> spMP = mCurrentFrame.mvpMapPoints[i];
            if (!spMP->isBad()) {
                const map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > > observations = spMP->GetObservations();
                for (map<std::weak_ptr<KeyFrame>, size_t, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator it = observations.begin(), itend = observations.end();
                     it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else {
                mCurrentFrame.mvpMapPoints[i].reset();
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    int max = 0;
    std::shared_ptr<KeyFrame> pKFmax;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<std::weak_ptr<KeyFrame>, int, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
         it != itEnd; it++) {
        std::weak_ptr<KeyFrame> pKF = it->first;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();

        if (spKF->isBad())
            continue;

        if (it->second > max) {
            max = it->second;
            pKFmax = spKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        spKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<std::weak_ptr<KeyFrame> >::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++) {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        std::weak_ptr<KeyFrame> pKF = *itKF;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        const vector<std::weak_ptr<KeyFrame> > vNeighs = spKF->GetBestCovisibilityKeyFrames(10);

        for (vector<std::weak_ptr<KeyFrame> >::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF; itNeighKF++) {
            std::weak_ptr<KeyFrame> pNeighKF = *itNeighKF;
            if (pNeighKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spNeighKF = pNeighKF.lock();
            if (!spNeighKF->isBad()) {
                if (spNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    spNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > spChilds = spKF->GetChilds();
        for (set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > >::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
            std::weak_ptr<KeyFrame> pChildKF = *sit;
            if (pChildKF.expired())
                continue;
            std::shared_ptr<KeyFrame> spChildKF = pChildKF.lock();
            if (!spChildKF->isBad()) {
                if (spChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    spChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        std::weak_ptr<KeyFrame> pParent = spKF->GetParent();
        if (!pParent.expired()) {
            std::shared_ptr<KeyFrame> spParent = pParent.lock();
            if (spParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                spParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }

    }

    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<std::weak_ptr<KeyFrame> > vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<std::shared_ptr<PnPsolver> > vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<std::shared_ptr<MapPoint> > > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        std::weak_ptr<KeyFrame> pKF = vpCandidateKFs[i];
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        if (spKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(spKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            }
            else {
                std::shared_ptr<PnPsolver> pSolver = std::make_shared<PnPsolver> (mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            std::shared_ptr<PnPsolver> pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty()) {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<std::shared_ptr<MapPoint> > sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j].reset();
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io].reset();

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    if (vpCandidateKFs[i].expired())
                        continue;
                    std::shared_ptr<KeyFrame> svpCandidateKFsi = vpCandidateKFs[i].lock();
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, svpCandidateKFsi, sFound, 10,
                                                                  100);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, svpCandidateKFsi, sFound, 3,
                                                                      64);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io].reset();
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    }
    else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    cout << "System Reseting" << endl;
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();
    mpMapDrawer->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer) {
        mpInitializer.reset();
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
        mpViewer->Release();
}

void Tracking::CloseSegmentaionThread()
{
    mbSegStop = true;
    mSegmentation->join();
    mSegmentation.reset();
}

void Tracking::StartSegmentationThread()
{
    if (mSegmentation)
    {
        mbSegStop = true;
        mSegmentation->join();
    }
    std::queue<ImagePair>().swap(mImagesQueue); // clear images queue
    mbSegStop = false;
    mSegmentation.reset(new std::thread(&Tracking::PerformSegmentation, this));
}


void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];
    float rows = fSettings["Camera.height"];
    float cols = fSettings["Camera.width"];
    Frame::InitializeStaticVariables(mpORBextractorLeft,
                                     mpORBVocabulary,
                                     mK,
                                     mDistCoef,
                                     mbf,
                                     mThDepth,
                                     rows,
                                     cols);
    boost::filesystem::path depthImageFolder(msDepthImagesFolder);
    boost::filesystem::path depthImageFolderFullPath = msDataFolder / depthImageFolder;
    KeyFrame::InitializeStaticVariables(depthImageFolderFullPath.string());
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

} //namespace ORB_SLAM
