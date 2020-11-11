#include "tiev_utils.h"
#include <algorithm>
#include <chrono>
#include <math.h>
namespace TiEV {
using namespace std;

double point2PointDis(const Point2d& p1, const Point2d& p2) {
    return sqrt(point2PointSqrDis(p1, p2));
}

double point2PointSqrDis(const Point2d& p1, const Point2d& p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

Point2d offsetPoint(const Point2d& p, const double dis) {
    double _tmp = (dis / GRID_RESOLUTION) / p.len();
    double dx   = -p.y * _tmp;
    double dy   = p.x * _tmp;
    return Point2d(dx, dy);
}

bool isInLocalMap(const Point2d& p) {
    return (p.x >= 0 && p.y >= 0 && p.x < MAX_ROW && p.y < MAX_COL);
}

Pose getLateralPoint(const Pose& p, double lateral_distance) {
    Point2d vec_d     = p.getDirectionVec();
    Point2d off_vec   = offsetPoint(vec_d, lateral_distance);
    Pose    res_point = p;
    res_point.x       = p.x + off_vec.x;
    res_point.y       = p.y + off_vec.y;
    return res_point;
}

void normalizeAngle(double& rad_angle) {
    while(rad_angle > PI)
        rad_angle -= 2 * PI;
    while(rad_angle <= -PI)
        rad_angle += 2 * PI;
}

void shared_mutex::lock() {
    main_mtx.lock();
}

void shared_mutex::lock_shared() {
    shared_mtx.lock();
    if((++shared_cnt) == 1) main_mtx.lock();
    shared_mtx.unlock();
}

void shared_mutex::unlock() {
    main_mtx.unlock();
}

void shared_mutex::unlock_shared() {
    shared_mtx.lock();
    if(--shared_cnt == 0) main_mtx.unlock();
    shared_mtx.unlock();
}

bool shared_mutex::try_lock() {
    return main_mtx.try_lock();
}

bool shared_mutex::try_lock_shared() {
    shared_mtx.lock();
    if(shared_cnt == 0) {
        if(!main_mtx.try_lock()) {
            shared_mtx.unlock();
            return false;
        }
        else
            ++shared_cnt;
    }
    shared_mtx.unlock();
}

std::time_t getTimeStamp() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    auto        tmp       = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    // std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

/*parse TimeStamp into tm struct
  Return: tm
*/
std::tm* gettm(int64_t timestamp) {
    int64_t  milli = timestamp + (int64_t)8 * 60 * 60 * 1000 * 1000;  //此处转化为东八区北京时间，如果是其它时区需要按需求修改
    auto     mTime = std::chrono::microseconds(milli);
    auto     tp    = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>(mTime);
    auto     tt    = std::chrono::system_clock::to_time_t(tp);
    std::tm* now   = std::gmtime(&tt);
    // printf("%4d年%02d月%02d日 %02d:%02d:%02d\n",now->tm_year+1900,now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
    return now;
}
}  // namespace TiEV
