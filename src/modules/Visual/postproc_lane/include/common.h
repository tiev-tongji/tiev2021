#ifndef COMMON_H
#define COMMON_H

#include <cstdio>
#include <sstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <zcm/zcm-cpp.hpp>
#include "zcmmsg/LANE.hpp"
#include "zcmmsg/LaneLine.hpp"

typedef unsigned char uchar_t;


typedef enum {
    TYPE_NONE = LANE::TYPE_NONE,
    TYPE_STRAIGHT = LANE::TYPE_STRAIGHT,
    TYPE_LEFT = LANE::TYPE_LEFT, 
    TYPE_RIGHT = LANE::TYPE_RIGHT,
    TYPE_UTURN = LANE::TYPE_UTURN,
    TYPE_STRAIGHT_LEFT = LANE::TYPE_STRAIGHT_LEFT,
    TYPE_STRAIGHT_RIGHT = LANE::TYPE_STRAIGHT_RIGHT,
    TYPE_STRAIGHT_LEFT_RIGHT = LANE::TYPE_STRAIGHT_LEFT_RIGHT
} MarkType;

typedef enum {
    TYPE_SOLID = LaneLine::TYPE_SOLID,
    TYPE_DASHED = LaneLine::TYPE_DASHED,
    TYPE_YELLOW = LaneLine::TYPE_YELLOW,
    TYPE_WHITE = LaneLine::TYPE_WHITE,
    TYPE_SOLID_WHITE = LaneLine::TYPE_SOLID_WHITE,
    TYPE_SOLID_YELLOW = LaneLine::TYPE_SOLID_YELLOW,
    TYPE_DASHED_WHITE = LaneLine::TYPE_DASHED_WHITE,
    TYPE_DASHED_YELLOW = LaneLine::TYPE_DASHED_YELLOW
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
