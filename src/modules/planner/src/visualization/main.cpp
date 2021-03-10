#include <thread>
#include "visualization.h"

using namespace std;

using namespace TiEV;
int main() {
  Visualization* vis = Visualization::getInstance();
  thread visualization = thread(&Visualization::visualize, vis);
  thread visualization_receive = thread(&Visualization::msgReceiveUdp, vis);
  thread visualization_receive_ipc = thread(&Visualization::msgReceiveIpc, vis);
  visualization.join();
  visualization_receive.join();
  visualization_receive_ipc.join();
  return 0;
}
