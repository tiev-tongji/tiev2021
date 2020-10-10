#include <iostream>
#include <map>
#include <mutex>
#include <vector>
#include <stdlib.h>
#include <unistd.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "structAIMPATH.hpp"

#define MAXM 151
#define MAXN 401

#define CAR_COLOR CV_RGB(0, 255, 255)
#define WHEEL_COLOR CV_RGB(0, 255, 255)
#define STRING_COLOR CV_RGB(255, 255, 255)
#define AIMPATH_COLOR CV_RGB(0, 255, 0)
#define FRAME_COLOR CV_RGB(127, 127, 127)
#define CARPOINT_COLOR CV_RGB(255, 127, 127)
#define PATHPOINT_COLOR CV_RGB(255, 0, 0)
#define DEVLINE_COLOR CV_RGB(255, 0, 0)

#define FRAME_TOP_Y 25

class Visualization
{
public:
    Visualization();
    virtual ~Visualization(){}

    void getAimPath(const std::vector<TrajectoryPoint> &_apath);
    void clearAimPath();

    void Show();
private:
    void init();

    void MyCar(cv::Mat img);
    void MyPath(cv::Mat img);
    cv::Point MyCoord(cv::Point2f star);
    void MyLine(cv::Mat img, cv::Point2f start, cv::Point2f end, cv::Scalar color);

    std::mutex AIMPATHmtx;

    std::vector<TrajectoryPoint> apath;
    
    double oriheading;
    double res;//m
    short w;//grid width size of map
    short h;//grid height size of map
    cv::Point2f ori;
    cv::Point2f cen;//position of car
    cv::Point2f cen_rear;//position of car

    cv::Mat image_maps;
    int maps_num;

    cv::Mat DrawControlMap();

    cv::Mat AllMaps(const std::map<std::string, cv::Mat> &_maps);

    void Merge(cv::Mat &_visual);
};

