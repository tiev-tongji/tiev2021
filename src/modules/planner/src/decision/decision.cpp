#include "decision.h"
#include "config.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include <unistd.h>
namespace TiEV {
/**
 * 决策规划的线程
 */
void runTiEVFSM() {
    Config const* cfg = Config::getInstance();
    // map managet initialization
    MapManager* mapm = MapManager::getInstance();
    mapm->readGlobalPathFile(cfg->roadmap_file);
    // FSM...
    Context       context;
    FSM::Instance machine{ context };
    while(true) {
        mapm->update();
        context.update();  //更新途灵事件信息
        machine.update();
        mapm->visualization();
    }
}

void sendPath() {
    MapManager*     mapm = MapManager::getInstance();
    MessageManager* msgm = MessageManager::getInstance();
    structAIMPATH   control_path;
    while(true) {
        vector<Pose> maintained_path = mapm->getMaintainedPath();
        control_path.num_points      = maintained_path.size();
        for(const auto& p : maintained_path) {
            TrajectoryPoint tp;
            tp.x     = p.x;
            tp.y     = p.y;
            tp.theta = p.ang;
            tp.v     = p.v;
            tp.a     = p.a;
            tp.k     = p.k;
            tp.t     = p.t;
            control_path.points.push_back(tp);
        }
        msgm->publishPath(control_path);
        usleep(10 * 1000);
    }
}

void requestGlobalPathFromMapServer() {
    time_t          start_time = getTimeStamp();
    MessageManager* msg_m      = MessageManager::getInstance();
    NavInfo         nav_info;
    while(true) {
        msg_m->getNavInfo(nav_info);
        if(getTimeStamp() - start_time < 15e6) {
            usleep(15e6 + start_time - getTimeStamp());
        }
        start_time = getTimeStamp();
    }
}
}  // namespace TiEV
