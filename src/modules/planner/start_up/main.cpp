#include "config/config.h"
#include <thread>
#include <unistd.h>
#include "visualization.h"
#include "message_manager.h"
#include "road_manager.h"
#include "decision.h"

using namespace std;
using namespace TiEV;

int main(){
	Config const *cfg = Config::getInstance();
	
	RoadManager *road_manager = RoadManager::getInstance();
	road_manager->readRoadMapFile(cfg->roadmap_file);

	//start visualization thread
	Visualization *vs = Visualization::getInstance();
	thread visualization = thread(&Visualization::visualize, vs);
	visualization.detach();

	//start message receiver
	MessageManager *msg_manager = MessageManager::getInstance();
	thread msg_receiver_ipc = thread(&MessageManager::msgReceiveIpc, msg_manager);
	msg_receiver_ipc.detach();
	thread msg_receiver_udp = thread(&MessageManager::msgReceiveUdp, msg_manager);
	msg_receiver_udp.detach();
	
	Decision* decision = Decision::getInstance();
	//start send controller path thread
	thread send_traj = thread(&Decision::sendPidPath, decision);
	send_traj.detach();

	//start decision thread
	decision->runDecision();
	cout << "Bye~, TiEV autonomous system exited!!!" << endl;

    return 0;
}
