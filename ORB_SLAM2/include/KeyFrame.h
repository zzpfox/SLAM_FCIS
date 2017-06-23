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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include <memory>
#include <mutex>
#include "Serialization.h"
namespace ORB_SLAM2
{

class Map;

class MapPoint;

class Frame;

class KeyFrameDatabase;

class KeyFrame: public std::enable_shared_from_this<KeyFrame>
{
public:
    KeyFrame(Frame &F, std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrameDatabase> pKFDB);

    KeyFrame();

    // Pose functions
    void SetPose(const cv::Mat &Tcw);

    cv::Mat GetPose();

    cv::Mat GetPoseInverse();

    cv::Mat GetCameraCenter();

    cv::Mat GetStereoCenter();

    cv::Mat GetRotation();

    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(std::shared_ptr<KeyFrame> pKF, const int &weight);

    void EraseConnection(std::shared_ptr<KeyFrame> pKF);

    void UpdateConnections();

    void UpdateBestCovisibles();

    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > GetConnectedKeyFrames();

    std::vector<std::weak_ptr<KeyFrame> > GetVectorCovisibleKeyFrames();

    std::vector<std::weak_ptr<KeyFrame> > GetBestCovisibilityKeyFrames(const int &N);

    std::vector<std::weak_ptr<KeyFrame> > GetCovisiblesByWeight(const int &w);

    int GetWeight(std::weak_ptr<KeyFrame> pKF);

    // Spanning tree functions
    void AddChild(std::shared_ptr<KeyFrame> pKF);

    void EraseChild(std::shared_ptr<KeyFrame> pKF);

    void ChangeParent(std::shared_ptr<KeyFrame> pKF);

    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > GetChilds();

    std::weak_ptr<KeyFrame> GetParent();

    bool hasChild(std::weak_ptr<KeyFrame> pKF);

    // Loop Edges
    void AddLoopEdge(std::weak_ptr<KeyFrame> pKF);

    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(std::shared_ptr<MapPoint> pMP, const size_t &idx);

    void EraseMapPointMatch(const size_t &idx);

    void EraseMapPointMatch(std::shared_ptr<MapPoint> pMP);

    void ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<MapPoint> pMP);

    std::set<std::weak_ptr<MapPoint>, std::owner_less<std::weak_ptr<MapPoint> > > GetMapPoints();

    std::vector<std::weak_ptr<MapPoint> > GetMapPointMatches();

    int TrackedMapPoints(const int &minObs);

    std::weak_ptr<MapPoint> GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;

    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();

    void SetErase();

    // Set/check bad flag
    void SetBadFlag();

    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(int a, int b)
    {
        return a > b;
    }

    static bool lId(std::weak_ptr<KeyFrame> pKF1, std::weak_ptr<KeyFrame> pKF2)
    {
        return pKF1.lock()->mnId < pKF2.lock()->mnId;
    }

    void DeleteDepthImage();

    static void SetVocabulary(std::shared_ptr<ORBVocabulary> pVocabulary);

    static void InitializeStaticVariables(std::string depthImageFolder);

    void SetMap(std::shared_ptr<Map> map);

    void SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKeyFrameDB);

    void RestoreKeyFrame(std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDatabase> pKeyFrameDB);


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static std::string msDepthImagesFolder;
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    static int mnGridCols;
    static int mnGridRows;
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    static float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    static int mnScaleLevels;
    static float mfScaleFactor;
    static float mfLogScaleFactor;
    static std::vector<float> mvScaleFactors;
    static std::vector<float> mvLevelSigma2;
    static std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    static int mnMinX;
    static int mnMinY;
    static int mnMaxX;
    static int mnMaxY;
    static cv::Mat mK;

    std::string mDepthImageName;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    template <typename Archive>
    friend void ::boost::serialization::serialize(Archive &ar, KeyFrame &keyframe, const unsigned int file_version);

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<std::weak_ptr<MapPoint> > mvpMapPoints;

    // BoW
    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    static std::shared_ptr<ORBVocabulary> mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t> > > mGrid;

    std::map<std::weak_ptr<KeyFrame>, int, std::owner_less<std::weak_ptr<KeyFrame> > > mConnectedKeyFrameWeights;
    std::vector<std::weak_ptr<KeyFrame> > mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    std::weak_ptr<KeyFrame> mpParent;
    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > mspChildrens;
    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame> > > mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    static float mHalfBaseline; // Only for visualization

    std::weak_ptr<Map> mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;


};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
