#include "decision.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <thread>
using namespace std;
using namespace TiEV;
using namespace cv;

const string planner_para_file_path = "/home/milesching/workspace/TiEV/2019/tiev2019/src/cfg/plannerPara.json";
const string map_file_path = "/home/milesching/workspace/TiEV/src/logs/jiugongge_task1_all_final-1117_modified.txt";
const string window_name = "Decision Test";
std::mutex window_mtx;
MessageManager* msm;
RoadManager* rdm;
Decision* decision;

int main(){
    /*
    Config::initializeInstance(planner_para_file_path);
	msm = MessageManager::getInstance();
	rdm = RoadManager::getInstance();
    decision = Decision::getInstance();

    rdm->readRoadMapFile(map_file_path);
    vector<TiEV::Point> res;
    thread t1(&MessageManager::msgRecieve, msm);
    thread t2(&RoadManager::runRoadPublishing, rdm, 10000);
    thread t3(&Decision::runDecision, decision);

    t1.join();
    t2.join();
    t3.join();
    return 0;
    */
}
