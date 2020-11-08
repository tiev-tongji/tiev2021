#include "road_manager.h"
#include "message_manager.h"
#include <iostream>
#include <thread>
#include "unistd.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace TiEV;

#define CLEAR() printf("\033[2J")
//const string map_file_path = "/home/milesching/workspace/TiEV/src/logs/jiugongge_task1_all_final-1117_modified.txt";
const string map_file_path = "/home/autolab/tiev2019/tiev2019/src/logs/jiugongge_task1_all_final-1117_modified.txt";
const string window_name = "RoadManager Test";
std::mutex window_mtx;

cv::Mat mat = cv::Mat::zeros(401,302,CV_8UC3);

/*

void drawPath(int subgraph, const std::vector<TiEV::Point>& path, int color = 0xffffff){
    std::unique_lock<std::mutex> lck(window_mtx);
    cv::Mat smat = cv::Mat::zeros(TiEV::GRID_ROW, TIEV::GRID_COL,CV_8UC3);
    for(auto p : path){
        smat.at<cv::Vec3b>(p.x, p.y)[0] = (color & 0x0000ff);
        smat.at<cv::Vec3b>(p.x, p.y)[1] = (color & 0x00ff00) >> 8;
        smat.at<cv::Vec3b>(p.x, p.y)[2] = (color & 0xff0000) >> 16;
    }
    smat.copyTo(mat.colRange(subgraph*151, subgraph*151 + 151));
    cv::imshow(window_name, mat);
    cv::waitKey(10);
}

void printRoad(RoadManager* manager){
    while(1){
        usleep(10000);
        auto c = manager->getMaintainedPath();
        drawPath(1, c, 0xec40fa);
    }
}
*/

int main(){
    /*
    cv::namedWindow(window_name);
    MessageManager* mes = MessageManager::getInstance();
    RoadManager* manager = RoadManager::getInstance();
    NavInfo info;
    thread t(&MessageManager::msgRecieve, mes);
    thread t2(&RoadManager::runRoadPublishing, manager, 10000);
    thread t3(printRoad, manager);

    manager->readRoadMapFile(map_file_path);
    vector<Point> res;
    vector<ExtraInformation> inf;
    while(1){
        usleep(0);
        if(mes->getNavInfo(info)){
            manager->getLocalReferenceRoad(info, res, inf);
            drawPath(0, res);
            res = manager->processPath(res);
            manager->maintainPath(res);
        }
        else cout << "No nav" << endl;
    }
    */

    return 0;
}


