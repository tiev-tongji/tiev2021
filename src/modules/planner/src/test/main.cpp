#include "message_manager.h"
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace TiEV;
using namespace std;

void sendForwardPath(structAIMPATH& path) {
    path.num_points = 20;
    for(int i = 0; i < 20; ++i) {
        TrajectoryPoint tp;
        tp.x     = i;
        tp.y     = 0;
        tp.theta = 0;
        tp.v     = 10;
        tp.a     = 0;
        tp.k     = 0;
        tp.t     = 0;
        path.points.push_back(tp);
    }
    cout << "send forward path" << endl;
}

void sendBackwardPath(structAIMPATH& path) {
    path.num_points = 20;
    for(int i = 0; i < 20; ++i) {
        TrajectoryPoint tp;
        tp.x     = -i;
        tp.y     = 0;
        tp.theta = 0;
        tp.v     = -2;
        tp.a     = 0;
        tp.k     = 0;
        tp.t     = 0;
        path.points.push_back(tp);
    }
    cout << "send backward path" << endl;
}

int main(int argc, char** argv) {
    /*
    // start message receiver
    MessageManager* msg_manager      = MessageManager::getInstance();
    thread          msg_receiver_ipc = thread(&MessageManager::msgReceiveIpc, msg_manager);
    msg_receiver_ipc.detach();
    thread msg_receiver_udp = thread(&MessageManager::msgReceiveUdp, msg_manager);
    msg_receiver_udp.detach();

    */
    // start send controller path thread
    MessageManager* msgm = MessageManager::getInstance();
    structAIMPATH   control_path;
    while(true) {
        sendForwardPath(control_path);
        //sendBackwardPath(control_path);
        msgm->publishPath(control_path);
        usleep(10 * 1000);
    }
    cout << "Bye~, TiEV autonomous system exited!!!" << endl;
    return 0;
}
