#include "visualization.h"
#include <thread>

using namespace std;

using namespace TiEV;
int main() {
    Visualization* vis           = Visualization::getInstance();
    thread         visualization = thread(&Visualization::visualize, vis);
    thread visualization_receive = thread(&Visualization::msgReceiveUdp, vis);
    visualization.join();
    visualization_receive.join();
    return 0;
}
