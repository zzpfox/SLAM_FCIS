#ifndef SERIALIZATION_H
#define SERIALIZATION_H
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/access.hpp>
namespace ORB_SLAM2
{
class KeyFrame;
class MapPoint;
class Map;
class ObjectPos;
}
namespace cv
{
class Mat;
class KeyPoint;
}

namespace boost
{
namespace serialization
{

template<typename Archive>
void save(Archive &ar, const ::cv::Mat &m, const unsigned int file_version);

template<typename Archive>
void load(Archive &ar, ::cv::Mat &m, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::cv::Mat &m, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::cv::KeyPoint &p, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::KeyFrame &keyframe, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::MapPoint &mappoint, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::Map &map, const unsigned int file_version);

template<typename Archive>
void serialize(Archive &ar, ::ORB_SLAM2::ObjectPos &objectpos, const unsigned int file_version);
}
}


#endif
