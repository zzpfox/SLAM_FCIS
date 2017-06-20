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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer)
    : mSensor(sensor), mbReset(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
         "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
         "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
         "This is free software, and you are welcome to redistribute it" << endl <<
         "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
    else if (mSensor == STEREO)
        cout << "Stereo" << endl;
    else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = std::make_shared<ORBVocabulary> ();
    // bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase> (*mpVocabulary);

    //Create the Map
    mpMap = std::make_shared<Map> (fsSettings);

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = std::make_shared<FrameDrawer> (mpMap);
    mpMapDrawer = std::make_shared<MapDrawer> (mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = std::make_shared<Tracking> (this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                            mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = std::make_shared<LocalMapping> (mpMap, mSensor == MONOCULAR);
    mptLocalMapping.reset(new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper));

    //Initialize the Loop Closing thread and launch
//    mpLoopCloser = std::make_shared<LoopClosing> (mpMap, mpKeyFrameDatabase,
//                                                  mpVocabulary, mSensor != MONOCULAR);
    Eigen::aligned_allocator<LoopClosing> alloc;
    mpLoopCloser = std::allocate_shared<LoopClosing> (alloc, mpMap, mpKeyFrameDatabase,
                                                  mpVocabulary, mSensor != MONOCULAR);
    mptLoopClosing.reset(new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser));

    //Initialize the Viewer thread and launch
    if (bUseViewer) {
        mpViewer = std::make_shared<Viewer> (this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer.reset(new thread(&Viewer::Run, mpViewer));
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mptLocalMapping->detach();
    mptLoopClosing->detach();
    if (bUseViewer){
        mptViewer->detach();
    }
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if (mSensor != STEREO) {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    std::vector<std::shared_ptr<MapPoint> > svpMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedMapPoints.clear();
    mTrackedMapPoints.reserve(svpMapPoints.size());
    for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpMapPoints.begin();
         it != svpMapPoints.end(); ++it)
    {
        std::weak_ptr<MapPoint> wit = *it;
        mTrackedMapPoints.push_back(wit);
    }
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if (mSensor != RGBD) {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    std::vector<std::shared_ptr<MapPoint> > svpMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedMapPoints.clear();
    mTrackedMapPoints.reserve(svpMapPoints.size());
    for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpMapPoints.begin();
         it != svpMapPoints.end(); ++it)
    {
        std::weak_ptr<MapPoint> wit = *it;
        mTrackedMapPoints.push_back(wit);
    }
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if (mSensor != MONOCULAR) {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
//    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;

    std::vector<std::shared_ptr<MapPoint> > svpMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedMapPoints.clear();
    mTrackedMapPoints.reserve(svpMapPoints.size());
    for (std::vector<std::shared_ptr<MapPoint> >::iterator it = svpMapPoints.begin();
         it != svpMapPoints.end(); ++it)
    {
        std::weak_ptr<MapPoint> wit = *it;
        mTrackedMapPoints.push_back(wit);
    }
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn) {
        n = curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpTracker->CloseSegmentaionThread();
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if (mpViewer) {
        mpViewer->RequestFinish();
        while (!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
        usleep(5000);
    }

    if (mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<std::weak_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++) {
        if (*lbL)
            continue;

        std::weak_ptr<KeyFrame> pKF = *lRit;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while (spKF->isBad()) {
            Trw = Trw * spKF->mTcp;
            pKF = spKF->GetParent();
            spKF = pKF.lock();
        }

        Trw = Trw * spKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
          << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++) {
        std::shared_ptr<KeyFrame> spKF = vpKFs[i];
        // pKF->SetPose(pKF->GetPose()*Two);

        if (spKF->isBad())
            continue;

        cv::Mat R = spKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = spKF->GetCameraCenter();
        f << setprecision(6) << spKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<std::shared_ptr<KeyFrame> > vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<std::weak_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++) {
        std::weak_ptr<KeyFrame> pKF = *lRit;
        if (pKF.expired())
            continue;
        std::shared_ptr<KeyFrame> spKF = pKF.lock();
        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (spKF->isBad()) {
            //  cout << "bad parent" << endl;
            Trw = Trw * spKF->mTcp;
            pKF = spKF->GetParent();
            spKF = pKF.lock();
        }

        Trw = Trw * spKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
          << " " << twc.at<float>(0) << " " <<
          Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1)
          << " " <<
          Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2)
          << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<std::weak_ptr<MapPoint>  > System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
