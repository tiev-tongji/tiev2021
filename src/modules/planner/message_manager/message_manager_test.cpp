#include "message_manager.h"
#include <iostream>
#include <unistd.h>
using namespace TiEV;
using namespace std;

int main(){
    TiEV::MessageManager *manager = MessageManager::getInstance();
    std::thread t(&MessageManager::msgReceiveUdp, manager);

    TiEV::NavInfo nav;
    TiEV::LidarMap map;
    for(int i = 1; i <= 1000; ++i){
        usleep(500000);
        if(manager->getNavInfo(nav)){
            cout << "NavInfo" << i << " - seccessfully received" << endl;
            cout << nav.current_position.to_string() << endl;
        }
        else cout << "NavInfo" << i << " - lost" << endl;

        if(manager->getMap(map)){
            cout << "LidarMap" << i << " - seccessfully received" << endl;
        }
        else cout << "LidarMap" << i << " - lost" << endl;

    }

    t.join();

    return 0;
}
