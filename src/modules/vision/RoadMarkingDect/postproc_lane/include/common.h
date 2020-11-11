#ifndef COMMON_H
#define COMMON_H

#include <cstdio>
#include <sstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <zcm/zcm-cpp.hpp>
#include "zcmmsg/Lane.hpp"
#include "zcmmsg/LaneLine.hpp"

typedef unsigned char uchar_t;


typedef enum {
    TYPE_NONE = Lane::kTypeNone,
    TYPE_STRAIGHT = Lane::kTypeStraight,
    TYPE_LEFT = Lane::kTypeLeft, 
    TYPE_RIGHT = Lane::kTypeRight,
    TYPE_UTURN = Lane::kTypeUTurn,
    TYPE_STRAIGHT_LEFT = Lane::kTypeStraightLeft,
    TYPE_STRAIGHT_RIGHT = Lane::kTypeStraightRight,
    TYPE_STRAIGHT_LEFT_RIGHT = Lane::kTypeStraightLeftRight,
	TYPE_LEFT_RIGHT = Lane::kTypeLeftRight,
	TYPE_LEFT_UTURN = Lane::kTypeLeftUTurn,
	TYPE_STRAIGHT_UTURN = Lane::kTypeStraightUTurn,
	TYPE_MERGE = Lane::kTypeMerge
} MarkType;

typedef enum {
    TYPE_SOLID = LaneLine::kTypeSolid,
    TYPE_DASHED = LaneLine::kTypeDashed,
    TYPE_YELLOW = LaneLine::kTypeYellow,
    TYPE_WHITE = LaneLine::kTypeWhite,
    TYPE_SOLID_WHITE = LaneLine::kTypeSolidWhite,
    TYPE_SOLID_YELLOW = LaneLine::kTypeSolidYellow,
    TYPE_DASHED_WHITE = LaneLine::kTypeDashedWhite,
    TYPE_DASHED_YELLOW = LaneLine::kTypeDashedYellow
} LineType;

#define CHECK_LINETYPE(t1, t2) (static_cast<int>(t1) & static_cast<int>(t2))
#define IsSolid(t) (static_cast<int>(t) == 0x00 || static_cast<int>(t) == 0x02)
#define IsYellow(t) (static_cast<int>(t) == 0x02 || static_cast<int>(t) == 0x03)


#define DEBUG 0
#if DEBUG
#   define PRINT(x) std::cerr << __FILE__ << ":" << __LINE__ << ":\t\"" << #x << "\" = " << (x) << std::endl; std::flush(std::cerr);
#   define IMSHOW(x) {cv::Mat __ ## x ## __; cv::normalize(x, __ ## x ## __, 0, 255, cv::NORM_MINMAX); cv::imshow(#x, __ ## x ## __); cv::waitKey(1);}
#   define DEPLOY_PRINT(x)
#   define DEPLOY_IMSHOW(x, key)
#else
#   define DEPLOY_PRINT(x) std::cerr << __FILE__ << ":" << __LINE__ << ":\t\"" << #x << "\" = " << x << std::endl; std::flush(std::cerr);
#   define DEPLOY_IMSHOW(x, key) {cv::Mat __ ## x ## __; cv::normalize(x, __ ## x ## __, 0, 255, cv::NORM_MINMAX); cv::imshow(#x, __ ## x ## __); key = cv::waitKey(1);}
#   define PRINT(x)
#   define IMSHOW(x)
#endif

#endif // COMMON_H
