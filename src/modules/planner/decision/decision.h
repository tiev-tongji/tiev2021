#ifndef _DECISION_H_
#define _DECISION_H_

#include "decision_state.h"
#include <mutex>

using namespace std;

namespace TiEV{
using dc = tinyfsm::Fsm<DecisionFSM>;

static mutex decision_mtx;

class Decision{
public:
    //RoadManager::getInstance() should be loaded with target map
	static Decision* getInstance(){
		decision_mtx.lock();
		static Decision instance;
		decision_mtx.unlock();
		return &instance;
	}

    /* Keep collecting data from message_manager,
     * control path_planner together with other modules to generate a path
     * and maintain the final path to road_manager.
     * Need a separate thread to run on.
     */
    void runDecision();
	void sendPidPath();

private:
	Decision(){};
	~Decision(){};

	void setStopPidPath(structAIMPATH &_pid_path);

private:
	PlanningEvent planning_event;
};

}

#endif
