#include <stdio.h>
#include <unistd.h>
#include <zcm/zcm-cpp.hpp>
#include <string.h>
#include "../structLASERMAP.hpp"
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
using std::string;

class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const zcm::ReceiveBuffer* rbuf,
                           const string& chan,
                           const structLASERMAP *msg)
        {
            lasermap_mutex.lock();
            laser_map = *msg;
            lasermap_mutex.unlock();
        }
        structLASERMAP  laser_map;
        std::mutex lasermap_mutex;
};

void handle_zcm(double& utmX){
    zcm::ZCM zcm{"ipc"};
    //zcm::ZCM zcm {""};
    if (!zcm.good())
        return;
    Handler handlerObject;
    zcm.subscribe("LASERMAP", &Handler::handleMessage, &handlerObject);
    zcm.run();
    utmX = handlerObject.laser_map.utmX;
}

void temp_test(int ind){
    printf("this is %dth time\n", ind);
    double utmX;
    std::thread th(handle_zcm, std::ref(utmX));
    th.detach();
    //handlerObject.lasermap_mutex.lock();
    printf("mHeading %f\n", utmX);
    //handlerObject.lasermap_mutex.unlock();
}


int main(int argc, char *argv[])
{
    printf("ok\n");
    int ind = 0;
    for(; ind<100000; ind++){
        printf("ok\n");
        temp_test(ind);
    }
    return 0;
}
