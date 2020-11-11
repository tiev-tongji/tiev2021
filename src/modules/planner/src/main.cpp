#include "decision.h"
#include "message_manager.h"
#include <thread>
#include <unistd.h>

using namespace TiEV;
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
    thread send_traj = thread(sendPath);
    send_traj.detach();

    // start decision thread
    runTiEVFSM();
    cout << "Bye~, TiEV autonomous system exited!!!" << endl;
    return 0;
}
