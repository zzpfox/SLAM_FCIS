#include "Serialization.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"


namespace boost
{
namespace serialization
{

typedef boost::archive::binary_oarchive BoostBinOar;
typedef boost::archive::binary_iarchive BoostBinIar;
/** Serialization support for cv::Mat */
template<typename Archive>
void save(Archive &ar, const ::cv::Mat &m, const unsigned int file_version)
{
    size_t elem_size = m.elemSize();
    size_t elem_type = m.type();

    ar & m.cols;
    ar & m.rows;
    ar & elem_size;
    ar & elem_type;

    const size_t data_size = m.cols * m.rows * elem_size;
    ar & boost::serialization::make_array(m.ptr(), data_size);
}

/** Serialization support for cv::Mat */
template<typename Archive>
void load(Archive &ar, ::cv::Mat &m, const unsigned int file_version)
{
    int cols, rows;
    size_t elem_size, elem_type;

    ar & cols;
    ar & rows;
    ar & elem_size;
    ar & elem_type;

    m.create(rows, cols, elem_type);

    size_t data_size = m.cols * m.rows * elem_size;
    ar & boost::serialization::make_array(m.ptr(), data_size);
}

template<typename Archive>
void serialize(Archive &ar, ::cv::Mat &m, const unsigned int file_version)
{
    split_free(ar, m, file_version);
}

/** Serialization support for cv::KeyPoint */
template<typename Archive>
void serialize(Archive &ar, ::cv::KeyPoint &p, const unsigned int file_version)
{
    ar & p.pt.x;
    ar & p.pt.y;
    ar & p.size;
    ar & p.angle;
    ar & p.response;
    ar & p.octave;
    ar & p.class_id;
}


template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::KeyFrame &keyframe, const unsigned int file_version)
{
    ar & keyframe.mnId;
    ar & const_cast<long unsigned int &> (keyframe.mnFrameId);
    ar & const_cast<double &>(keyframe.mTimeStamp);
    ar & keyframe.mnTrackReferenceForFrame;
    ar & keyframe.mnFuseTargetForKF;
    ar & keyframe.mnBAFixedForKF;
    ar & keyframe.mnBALocalForKF;
    ar & keyframe.mnLoopQuery;
    ar & keyframe.mnLoopWords;
    ar & keyframe.mLoopScore;
    ar & keyframe.mnRelocQuery;
    ar & keyframe.mnRelocWords;
    ar & keyframe.mRelocScore;
    ar & keyframe.mTcwGBA;
    ar & keyframe.mTcwBefGBA;
    ar & keyframe.mnBAGlobalForKF;
    ar & const_cast<int &> (keyframe.N);
    ar & const_cast<std::vector<cv::KeyPoint> &> (keyframe.mvKeys);
    ar & const_cast<std::vector<cv::KeyPoint> &> (keyframe.mvKeysUn);
    ar & const_cast<std::vector<float> & > (keyframe.mvuRight);
    ar & const_cast<std::vector<float> & > (keyframe.mvDepth);
    ar & const_cast<cv::Mat &> (keyframe.mDescriptors);
    ar & keyframe.mTcp;
    ar & keyframe.mDepthImageName;
    ar & keyframe.Tcw;
    ar & keyframe.Twc;
    ar & keyframe.Ow;
    ar & keyframe.Cw;
    ar & keyframe.mvpMapPoints;
    ar & keyframe.mvpMapPoints;
    ar & keyframe.mGrid;
    ar & keyframe.mConnectedKeyFrameWeights;
    ar & keyframe.mvpOrderedConnectedKeyFrames;
    ar & keyframe.mvOrderedWeights;
    ar & keyframe.mbFirstConnection;
    ar & keyframe.mpParent;
    ar & keyframe.mspChildrens;
    ar & keyframe.mspLoopEdges;
    ar & keyframe.mbNotErase;
    ar & keyframe.mbToBeErased;
    ar & keyframe.mbBad;
}


template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::MapPoint &mappoint, const unsigned int file_version)
{
    ar & mappoint.mnId;
    ar & mappoint.mnFirstKFid;
    ar & mappoint.mnFirstFrame;
    ar & mappoint.nObs;
    ar & mappoint.mTrackProjX;
    ar & mappoint.mTrackProjY;
    ar & mappoint.mTrackProjXR;
    ar & mappoint.mbTrackInView;
    ar & mappoint.mnTrackScaleLevel;
    ar & mappoint.mTrackViewCos;
    ar & mappoint.mnTrackReferenceForFrame;
    ar & mappoint.mnLastFrameSeen;
    ar & mappoint.mnBALocalForKF;
    ar & mappoint.mnFuseCandidateForKF;
    ar & mappoint.mnLoopPointForKF;
    ar & mappoint.mnCorrectedByKF;
    ar & mappoint.mnCorrectedReference;
    ar & mappoint.mPosGBA;
    ar & mappoint.mnBAGlobalForKF;
    ar & mappoint.mWorldPos;
    ar & mappoint.mObservations;
    ar & mappoint.mNormalVector;
    ar & mappoint.mDescriptor;
    ar & mappoint.mpRefKF;
    ar & mappoint.mnVisible;
    ar & mappoint.mnFound;
    ar & mappoint.mbBad;
    ar & mappoint.mpReplaced;
    ar & mappoint.mfMinDistance;
    ar & mappoint.mfMaxDistance;
}

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::Map &map, const unsigned int file_version)
{
    ar & map.mvpKeyFrameOrigins;
    ar & map.mObjectMap;
    ar & map.mDepthMapFactor;
    ar & map.mLookupX;
    ar & map.mLookupY;
    ar & map.mspMapPoints;
    ar & map.mspKeyFrames;
    ar & map.mvpReferenceMapPoints;
    ar & map.mnMaxKFid;
    ar & map.mnBigChangeIdx;

}

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::ObjectPos &objectpos, const unsigned int file_version)
{
    ar & objectpos.Pcs;
}
template void save<BoostBinOar>(BoostBinOar &, const ::cv::Mat &, const unsigned int);
template void load<BoostBinIar>(BoostBinIar &, ::cv::Mat &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::cv::Mat &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::cv::Mat &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::cv::KeyPoint &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::cv::KeyPoint &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::ORB_SLAM2::KeyFrame &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::ORB_SLAM2::KeyFrame &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::ORB_SLAM2::MapPoint &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::ORB_SLAM2::MapPoint &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::ORB_SLAM2::Map &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::ORB_SLAM2::Map &, const unsigned int);

template void serialize<BoostBinOar>(BoostBinOar &, ::ORB_SLAM2::ObjectPos &, const unsigned int);
template void serialize<BoostBinIar>(BoostBinIar &, ::ORB_SLAM2::ObjectPos &, const unsigned int);

}


}

