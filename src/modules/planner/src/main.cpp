#include <unistd.h>

#include <thread>

#include "decision.h"
#include "message_manager.h"

using namespace TiEV;
int main(int argc, char** argv) {
  // start message receiver
  MessageManager& msg_manager = MessageManager::getInstance();
  thread          msg_receiver_ipc =
      thread(&MessageManager::msgReceiveIpc, &msg_manager);
  thread msg_receiver_udp =
      thread(&MessageManager::msgReceiveUdp, &msg_manager);
  // start send controller path thread
  thread send_traj = thread(sendPath);
  // start routing thread
  thread routing_thread = thread(requestGlobalPathFromMapServer);
  // start decision thread
  runTiEVFSM();
  msg_receiver_ipc.join();
  msg_receiver_udp.join();
  send_traj.join();
  routing_thread.join();
  cout << "Bye~, TiEV autonomous system exited!!!" << endl;
  return 0;
}
